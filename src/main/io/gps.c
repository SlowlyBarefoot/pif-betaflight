/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_GPS

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/light_led.h"
#include "drivers/time.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gps.h"
#include "io/serial.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/gps_rescue.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"

#include "communication/pif_uart.h"
#include "gps/pif_gps.h"
#include "gps/pif_gps_ublox.h"

#define LOG_ERROR        '?'
#define LOG_IGNORED      '!'
#define LOG_SKIPPED      '>'
#define LOG_NMEA_GGA     'g'
#define LOG_NMEA_RMC     'r'
#define LOG_UBLOX_SOL    'O'
#define LOG_UBLOX_STATUS 'S'
#define LOG_UBLOX_SVINFO 'I'
#define LOG_UBLOX_POSLLH 'P'
#define LOG_UBLOX_VELNED 'V'

#define DEBUG_SERIAL_BAUD  0 // set to 1 to debug serial port baud config (/100)
#define DEBUG_UBLOX_INIT   0 // set to 1 to debug ublox initialization
#define DEBUG_UBLOX_FRAMES 0 // set to 1 to debug ublox received frames

char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];
static char *gpsPacketLogChar = gpsPacketLog;
// **********************
// GPS
// **********************
int32_t GPS_home[2];
uint16_t GPS_distanceToHome;        // distance to home point in meters
int16_t GPS_directionToHome;        // direction to home or hol point in degrees
uint32_t GPS_distanceFlownInCm;     // distance flown since armed in centimeters
int16_t GPS_verticalSpeedInCmS;     // vertical speed in cm/s
float dTnav;             // Delta Time in milliseconds for navigation computations, updated with every good GPS read
int16_t nav_takeoff_bearing;

#define GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S 15 // 5.4Km/h 3.35mph

gpsSolutionData_t gpsSol;
uint32_t GPS_packetCount = 0;
uint32_t GPS_svInfoReceivedCount = 0; // SV = Space Vehicle, counter increments each time SV info is received.
uint8_t GPS_update = 0;             // toogle to distinct a GPS position update (directly or via MSP)

uint8_t GPS_numCh;                              // Details on numCh/svinfo in gps.h
uint8_t GPS_svinfo_chn[GPS_SV_MAXSATS_M8N];
uint8_t GPS_svinfo_svid[GPS_SV_MAXSATS_M8N];
uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS_M8N];
uint8_t GPS_svinfo_cno[GPS_SV_MAXSATS_M8N];

// GPS timeout for wrong baud rate/disconnection/etc in milliseconds (default 2.5second)
#define GPS_TIMEOUT (2500)
// How many entries in gpsInitData array below
#define GPS_INIT_ENTRIES (GPS_BAUDRATE_MAX + 1)
#define GPS_BAUDRATE_CHANGE_DELAY (200)
// How long a UBX configuration message may take, from being queued to its
// ACK or NAK arriving, before pifGpsUblox_CheckRequest() gives up on it.
// Same as the 25 GPS task cycles at 100Hz the native code waited.
#define UBLOX_ACK_TIMEOUT_MS (250)

static serialPort_t *gpsPort;

// The PIF GPS driver of the configured provider. The frames are parsed by
// pif_gps (NMEA) and pif_gps_ublox (UBX); this file only feeds them the bytes
// read from gpsPort and turns what they decode into gpsSol. Only the member
// of gpsConfig()->provider is initialised, and only once gpsPort is open.
static union {
#ifdef USE_GPS_NMEA
    PifGps nmea;
#endif
#ifdef USE_GPS_UBLOX
    PifGpsUblox ublox;
#endif
} gpsDriver;

// Set by the PIF receive events when a frame has brought a new solution, and
// read back by gpsNewFrame() for the byte that completed it.
static bool gpsFrameDone;

typedef struct gpsInitData_s {
    uint8_t index;
    uint8_t baudrateIndex; // see baudRate_e
    const char *ubx;
    const char *mtk;
} gpsInitData_t;

// NMEA will cycle through these until valid data is received
static const gpsInitData_t gpsInitData[] = {
    { GPS_BAUDRATE_115200,  BAUD_115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n", "$PMTK251,115200*1F\r\n" },
    { GPS_BAUDRATE_57600,    BAUD_57600, "$PUBX,41,1,0003,0001,57600,0*2D\r\n", "$PMTK251,57600*2C\r\n" },
    { GPS_BAUDRATE_38400,    BAUD_38400, "$PUBX,41,1,0003,0001,38400,0*26\r\n", "$PMTK251,38400*27\r\n" },
    { GPS_BAUDRATE_19200,    BAUD_19200, "$PUBX,41,1,0003,0001,19200,0*23\r\n", "$PMTK251,19200*22\r\n" },
    // 9600 is not enough for 5Hz updates - leave for compatibility to dumb NMEA that only runs at this speed
    { GPS_BAUDRATE_9600,      BAUD_9600, "$PUBX,41,1,0003,0001,9600,0*16\r\n", "" }
};

#define GPS_INIT_DATA_ENTRY_COUNT (sizeof(gpsInitData) / sizeof(gpsInitData[0]))

#define DEFAULT_BAUD_RATE_INDEX 0

#ifdef USE_GPS_UBLOX
#define UBLOX_MODE_ENABLED    0x1
#define UBLOX_MODE_TEST       0x2

#define UBLOX_USAGE_RANGE     0x1
#define UBLOX_USAGE_DIFFCORR  0x2
#define UBLOX_USAGE_INTEGRITY 0x4

#define UBLOX_GNSS_ENABLE     0x1
#define UBLOX_GNSS_DEFAULT_SIGCFGMASK 0x10000

#define UBLOX_DYNMODE_PEDESTRIAN  3
#define UBLOX_DYNMODE_AIRBORNE_1G 6
#define UBLOX_DYNMODE_AIRBORNE_4G 8

// Payloads of the configuration messages sent to the receiver. pif_gps_ublox
// adds the UBX header and checksum.
typedef struct {
    uint8_t gnssId;
    uint8_t resTrkCh;
    uint8_t maxTrkCh;
    uint8_t reserved1;
    uint32_t flags;
} ubx_configblock;

typedef struct {
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;
} ubx_cfg_msg;

typedef struct {
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
} ubx_cfg_rate;

typedef struct {
    uint8_t mode;
    uint8_t usage;
    uint8_t maxSBAS;
    uint8_t scanmode2;
    uint32_t scanmode1;
} ubx_cfg_sbas;

typedef struct {
    uint8_t msgVer;
    uint8_t numTrkChHw;
    uint8_t numTrkChUse;
    uint8_t numConfigBlocks;
    ubx_configblock configblocks[7];
} ubx_cfg_gnss;

typedef struct {
    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt;
    uint32_t fixedAltVar;
    int8_t minElev;
    uint8_t drLimit;
    uint16_t pDOP;
    uint16_t tDOP;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t staticHoldThresh;
    uint8_t dgnssTimeout;
    uint8_t cnoThreshNumSVs;
    uint8_t cnoThresh;
    uint8_t reserved0[2];
    uint16_t staticHoldMaxDist;
    uint8_t utcStandard;
    uint8_t reserved1[5];
} ubx_cfg_nav5;

// Received payloads that pif_gps_ublox has no type for. They are read from the
// payload bytes of the PifGpsUbxPacket.
typedef struct {
    uint8_t gnssId;
    uint8_t svId;               // Satellite ID
    uint8_t cno;                // Carrier to Noise Ratio (Signal Strength) // dbHz, 0-55.
    int8_t elev;                // Elevation in integer degrees
    int16_t azim;               // Azimuth in integer degrees
    int16_t prRes;              // Pseudo range residual in decimetres
    uint32_t flags;             // Bitmask
} ubx_nav_sat_sv;

typedef struct {
    uint32_t time;              // GPS Millisecond time of week
    uint8_t version;
    uint8_t numSvs;
    uint8_t reserved0[2];
    ubx_nav_sat_sv svs[GPS_SV_MAXSATS_M9N];
} ubx_nav_sat;

// From the UBlox9 document, the largest payload we receive is the NAV-SAT and
// its size is 8 + 12*numCh, with numCh 42 in the case of a M9N. A longer
// packet is dropped by pif_gps_ublox and logged as skipped.
STATIC_ASSERT(sizeof(ubx_nav_sat) <= PIF_GPS_UBLOX_RX_PAYLOAD_SIZE, ubx_nav_sat_fits_pif_rx_payload);
STATIC_ASSERT(sizeof(ubx_cfg_gnss) + 12 <= PIF_GPS_UBLOX_TX_SIZE, ubx_cfg_gnss_fits_pif_tx_buffer);

enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
} ubs_nav_fix_type;

enum {
    NAV_STATUS_FIX_VALID = 1,
    NAV_STATUS_TIME_WEEK_VALID = 4,
    NAV_STATUS_TIME_SECOND_VALID = 8
} ubx_nav_status_bits;

enum {
    NAV_VALID_DATE = 1,
    NAV_VALID_TIME = 2
} ubx_nav_pvt_valid;

// pif_gps_ublox sends through a PifUart. This one has no buffers and no RX
// task: its TX task runs the pif_gps_ublox sender, which hands the bytes
// straight to gpsPort through act_send_data. The bytes received are read from
// gpsPort in gpsUpdate() and given to pifGpsUblox_ParsingPacket(), so that
// passthrough, which reads the port itself, keeps working.
static PifUart gpsUart;

// The CFG-GNSS the receiver answered the poll with, to be sent back with
// SBAS and Galileo set as configured. gnssConfigLength is 0 until it arrives.
static ubx_cfg_gnss gnssConfig;
static uint16_t gnssConfigLength;

#endif // USE_GPS_UBLOX

typedef enum {
    GPS_STATE_UNKNOWN,
    GPS_STATE_INITIALIZING,
    GPS_STATE_INITIALIZED,
    GPS_STATE_CHANGE_BAUD,
    GPS_STATE_CONFIGURE,
    GPS_STATE_RECEIVING_DATA,
    GPS_STATE_LOST_COMMUNICATION,
    GPS_STATE_COUNT
} gpsState_e;

// Max time to wait for received data
#define GPS_MAX_WAIT_DATA_RX 30

gpsData_t gpsData;

PG_REGISTER_WITH_RESET_TEMPLATE(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);

PG_RESET_TEMPLATE(gpsConfig_t, gpsConfig,
    .provider = GPS_NMEA,
    .sbasMode = SBAS_NONE,
    .autoConfig = GPS_AUTOCONFIG_ON,
    .autoBaud = GPS_AUTOBAUD_OFF,
    .gps_ublox_use_galileo = false,
    .gps_ublox_mode = UBLOX_AIRBORNE,
    .gps_set_home_point_once = false,
    .gps_use_3d_speed = false,
    .sbas_integrity = false
);

static void shiftPacketLog(void)
{
    uint32_t i;

    for (i = ARRAYLEN(gpsPacketLog) - 1; i > 0 ; i--) {
        gpsPacketLog[i] = gpsPacketLog[i-1];
    }
}

static bool isConfiguratorConnected() {
    return (getArmingDisableFlags() & ARMING_DISABLED_MSP);
}

static void gpsNewData(uint16_t c);
#ifdef USE_GPS_NMEA
static BOOL gpsNmeaReceive(PifGps *gps, PifGpsNmeaMsgId msgId);
#endif
#ifdef USE_GPS_UBLOX
static BOOL gpsUbloxReceive(PifGpsUblox *ublox, PifGpsUbxPacket *packet);
static void gpsUbloxError(PifGpsUblox *ublox, PifGpsUbxError error);
static uint16_t gpsUartSendData(PifUart *uart, uint8_t *data, uint16_t size);
#endif

static void gpsSetState(gpsState_e state)
{
    gpsData.lastMessage = millis();
    sensorsClear(SENSOR_GPS);

    gpsData.state = state;
    gpsData.state_position = 0;
    gpsData.state_ts = millis();
    gpsData.ackState = UBLOX_ACK_IDLE;
}

static void gpsSetBaudRate(uint32_t baudRate)
{
    serialSetBaudRate(gpsPort, baudRate);
#ifdef USE_GPS_UBLOX
    if (gpsConfig()->provider == GPS_UBLOX) {
        // Only paces the PifUart TX task; the port has already been changed.
        pifUart_ChangeBaudrate(&gpsUart, baudRate);
    }
#endif
}

// Brings up the PIF driver of the configured provider on the open gpsPort.
static bool gpsInitDriver(uint32_t baudRate)
{
    UNUSED(baudRate);

    switch (gpsConfig()->provider) {
#ifdef USE_GPS_NMEA
    case GPS_NMEA:
        if (!pifGps_Init(&gpsDriver.nmea, PIF_ID_AUTO)) {
            return false;
        }
        gpsDriver.nmea.evt_nmea_receive = gpsNmeaReceive;
        return true;
#endif

#ifdef USE_GPS_UBLOX
    case GPS_UBLOX:
        if (!pifGpsUblox_Init(&gpsDriver.ublox, PIF_ID_AUTO)) {
            return false;
        }
        gpsDriver.ublox.evt_ubx_receive = gpsUbloxReceive;
        gpsDriver.ublox.evt_ubx_error = gpsUbloxError;

        if (!pifUart_Init(&gpsUart, PIF_ID_AUTO, baudRate)
            || !pifUart_AttachTxTask(&gpsUart, PIF_ID_AUTO, TM_EXTERNAL, 0, "GpsTx")) {
            pifUart_Clear(&gpsUart);
            pifGpsUblox_Clear(&gpsDriver.ublox);
            return false;
        }
        gpsUart.act_send_data = gpsUartSendData;

        // Last, since this is what starts the PifUart TX task.
        pifGpsUblox_AttachUart(&gpsDriver.ublox, &gpsUart);
        return true;
#endif

    default:
        return false;
    }
}

void gpsInit(void)
{
    gpsData.baudrateIndex = 0;
    gpsData.errors = 0;
    gpsData.timeouts = 0;

    memset(gpsPacketLog, 0x00, sizeof(gpsPacketLog));

    // init gpsData structure. if we're not actually enabled, don't bother doing anything else
    gpsSetState(GPS_STATE_UNKNOWN);

    gpsData.lastMessage = millis();

    if (gpsConfig()->provider == GPS_MSP) { // no serial ports used when GPS_MSP is configured
        gpsSetState(GPS_STATE_INITIALIZED);
        return;
    }

    // The frames are parsed by PIF.
    if (!pifLinker_IsReady()) {
        return;
    }

    const serialPortConfig_t *gpsPortConfig = findSerialPortConfig(FUNCTION_GPS);
    if (!gpsPortConfig) {
        return;
    }

    while (gpsInitData[gpsData.baudrateIndex].baudrateIndex != gpsPortConfig->gps_baudrateIndex) {
        gpsData.baudrateIndex++;
        if (gpsData.baudrateIndex >= GPS_INIT_DATA_ENTRY_COUNT) {
            gpsData.baudrateIndex = DEFAULT_BAUD_RATE_INDEX;
            break;
        }
    }

    portMode_e mode = MODE_RXTX;
#if defined(GPS_NMEA_TX_ONLY)
    if (gpsConfig()->provider == GPS_NMEA) {
        mode &= ~MODE_TX;
    }
#endif

    const uint32_t baudRate = baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex];

    // no callback - buffer will be consumed in gpsUpdate()
    gpsPort = openSerialPort(gpsPortConfig->identifier, FUNCTION_GPS, NULL, NULL, baudRate, mode, SERIAL_NOT_INVERTED);
    if (!gpsPort) {
        return;
    }

    if (!gpsInitDriver(baudRate)) {
        closeSerialPort(gpsPort);
        gpsPort = NULL;
        return;
    }

    // signal GPS "thread" to initialize when it gets to it
    gpsSetState(GPS_STATE_INITIALIZING);
}

#ifdef USE_GPS_NMEA
void gpsInitNmea(void)
{
#if !defined(GPS_NMEA_TX_ONLY)
    uint32_t now;
#endif
    switch (gpsData.state) {
        case GPS_STATE_INITIALIZING:
#if !defined(GPS_NMEA_TX_ONLY)
           now = millis();
           if (now - gpsData.state_ts < 1000) {
               return;
           }
           gpsData.state_ts = now;
           if (gpsData.state_position < 1) {
               gpsSetBaudRate(4800);
               gpsData.state_position++;
           } else if (gpsData.state_position < 2) {
               // print our FIXED init string for the baudrate we want to be at
               serialPrint(gpsPort, "$PSRF100,1,115200,8,1,0*05\r\n");
               gpsData.state_position++;
           } else {
               // we're now (hopefully) at the correct rate, next state will switch to it
               gpsSetState(GPS_STATE_CHANGE_BAUD);
           }
           break;
#endif
        case GPS_STATE_CHANGE_BAUD:
#if !defined(GPS_NMEA_TX_ONLY)
           now = millis();
           if (now - gpsData.state_ts < 1000) {
               return;
           }
           gpsData.state_ts = now;
           if (gpsData.state_position < 1) {
               gpsSetBaudRate(baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
               gpsData.state_position++;
           } else if (gpsData.state_position < 2) {
               serialPrint(gpsPort, "$PSRF103,00,6,00,0*23\r\n");
               gpsData.state_position++;
           } else
#else
           {
               gpsSetBaudRate(baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
           }
#endif
               gpsSetState(GPS_STATE_RECEIVING_DATA);
            break;
    }
}
#endif // USE_GPS_NMEA

#ifdef USE_GPS_UBLOX
// act_send_data of gpsUart, called from its TX task. Takes what fits in the
// port's transmit buffer; pif_gps_ublox comes back for the rest.
static uint16_t gpsUartSendData(PifUart *uart, uint8_t *data, uint16_t size)
{
    UNUSED(uart);

    const uint32_t bytesFree = serialTxBytesFree(gpsPort);
    if (size > bytesFree) {
        size = bytesFree;
    }
    serialWriteBuf(gpsPort, data, size);
    return size;
}

// Brings the state of the request on its way up to date, which is what lets
// pif_gps_ublox accept the next one once the last has been answered or has
// timed out, and returns it.
static PifGpsUbxRequestState ubloxCheckRequest(void)
{
    const PifGpsUbxRequestState state = pifGpsUblox_CheckRequest(&gpsDriver.ublox);

    if (state == GURS_SEND) {
        // The sender stops rescheduling itself if the port had no room at all,
        // so give it another go.
        pifTask_SetTrigger(gpsUart._p_tx_task, 0);
    }
    return state;
}

// Queues a UBX configuration message. Returns false if pif_gps_ublox refused
// it, which it does while the previous request is still on its way; the
// caller tries again on a later cycle. The ACK or NAK is read back with
// ubloxCheckRequest().
static bool ubloxSendConfigMessage(uint8_t msgId, const void *payload, uint16_t length)
{
    return pifGpsUblox_SendUbxMsg(&gpsDriver.ublox, GUCI_CFG, msgId, length, (uint8_t *)payload, UBLOX_ACK_TIMEOUT_MS);
}

static bool ubloxSendPollMessage(uint8_t msgId)
{
    return ubloxSendConfigMessage(msgId, NULL, 0);
}

static bool ubloxSendNAV5Message(bool airborne) {
    ubx_cfg_nav5 cfg_nav5;
    cfg_nav5.mask = 0xFFFF;
    if (airborne) {
#if defined(GPS_UBLOX_MODE_AIRBORNE_1G)
        cfg_nav5.dynModel = UBLOX_DYNMODE_AIRBORNE_1G;
#else
        cfg_nav5.dynModel = UBLOX_DYNMODE_AIRBORNE_4G;
#endif
    } else {
        cfg_nav5.dynModel = UBLOX_DYNMODE_PEDESTRIAN;
    }
    cfg_nav5.fixMode = 3;
    cfg_nav5.fixedAlt = 0;
    cfg_nav5.fixedAltVar = 10000;
    cfg_nav5.minElev = 5;
    cfg_nav5.drLimit = 0;
    cfg_nav5.pDOP = 250;
    cfg_nav5.tDOP = 250;
    cfg_nav5.pAcc = 100;
    cfg_nav5.tAcc = 300;
    cfg_nav5.staticHoldThresh = 0;
    cfg_nav5.dgnssTimeout = 60;
    cfg_nav5.cnoThreshNumSVs = 0;
    cfg_nav5.cnoThresh = 0;
    cfg_nav5.reserved0[0] = 0;
    cfg_nav5.reserved0[1] = 0;
    cfg_nav5.staticHoldMaxDist = 200;
    cfg_nav5.utcStandard = 0;
    cfg_nav5.reserved1[0] = 0;
    cfg_nav5.reserved1[1] = 0;
    cfg_nav5.reserved1[2] = 0;
    cfg_nav5.reserved1[3] = 0;
    cfg_nav5.reserved1[4] = 0;

    return ubloxSendConfigMessage(GUMI_CFG_NAV5, &cfg_nav5, sizeof(cfg_nav5));
}

static bool ubloxSetMessageRate(uint8_t messageClass, uint8_t messageID, uint8_t rate) {
    ubx_cfg_msg cfg_msg;
    cfg_msg.msgClass = messageClass;
    cfg_msg.msgID = messageID;
    cfg_msg.rate = rate;
    return ubloxSendConfigMessage(GUMI_CFG_MSG, &cfg_msg, sizeof(cfg_msg));
}

static bool ubloxSetNavRate(uint16_t measRate, uint16_t navRate, uint16_t timeRef) {
    ubx_cfg_rate cfg_rate;
    cfg_rate.measRate = measRate;
    cfg_rate.navRate = navRate;
    cfg_rate.timeRef = timeRef;
    return ubloxSendConfigMessage(GUMI_CFG_RATE, &cfg_rate, sizeof(cfg_rate));
}

static bool ubloxSetSbas() {
    ubx_cfg_sbas cfg_sbas;

    //NOTE: default ublox config for sbas mode is: UBLOX_MODE_ENABLED, test is disabled
    cfg_sbas.mode = UBLOX_MODE_TEST;
    if (gpsConfig()->sbasMode != SBAS_NONE) {
        cfg_sbas.mode |= UBLOX_MODE_ENABLED;
    }

    //NOTE: default ublox config for sbas mode is: UBLOX_USAGE_RANGE | UBLOX_USAGE_DIFFCORR, integrity is disabled
    cfg_sbas.usage = UBLOX_USAGE_RANGE | UBLOX_USAGE_DIFFCORR;
    if (gpsConfig()->sbas_integrity) {
        cfg_sbas.usage |= UBLOX_USAGE_INTEGRITY;
    }

    cfg_sbas.maxSBAS = 3;
    cfg_sbas.scanmode2 = 0;
    switch (gpsConfig()->sbasMode) {
        case SBAS_AUTO:
            cfg_sbas.scanmode1 = 0;
            break;
        case SBAS_EGNOS:
            cfg_sbas.scanmode1 = 0x00010048; //PRN123, PRN126, PRN136
            break;
        case SBAS_WAAS:
            cfg_sbas.scanmode1 = 0x0004A800; //PRN131, PRN133, PRN135, PRN138
            break;
        case SBAS_MSAS:
            cfg_sbas.scanmode1 = 0x00020200; //PRN129, PRN137
            break;
        case SBAS_GAGAN:
            cfg_sbas.scanmode1 = 0x00001180; //PRN127, PRN128, PRN132
            break;
        default:
            cfg_sbas.scanmode1 = 0;
            break;
    }
    return ubloxSendConfigMessage(GUMI_CFG_SBAS, &cfg_sbas, sizeof(cfg_sbas));
}

// Sends back the CFG-GNSS the receiver answered the poll with, with SBAS
// disabled and Galileo enabled as configured.
static bool ubloxSetGnss(void)
{
    ubx_cfg_gnss cfg_gnss;
    bool isSBASenabled = false;
    bool isM8NwithDefaultConfig = false;

    memcpy(&cfg_gnss, &gnssConfig, gnssConfigLength);

    if ((cfg_gnss.numConfigBlocks >= 2) &&
        (cfg_gnss.configblocks[1].gnssId == 1) && //SBAS
        (cfg_gnss.configblocks[1].flags & UBLOX_GNSS_ENABLE)) { //enabled

        isSBASenabled = true;
    }

    if ((cfg_gnss.numTrkChHw == 32) &&  //M8N
        (cfg_gnss.numTrkChUse == 32) &&
        (cfg_gnss.numConfigBlocks == 7) &&
        (cfg_gnss.configblocks[2].gnssId == 2) && //Galileo
        (cfg_gnss.configblocks[2].resTrkCh == 4) && //min channels
        (cfg_gnss.configblocks[2].maxTrkCh == 8) && //max channels
        !(cfg_gnss.configblocks[2].flags & UBLOX_GNSS_ENABLE)) { //disabled

        isM8NwithDefaultConfig = true;
    }

    if (isSBASenabled && (gpsConfig()->sbasMode == SBAS_NONE)) {
        cfg_gnss.configblocks[1].flags &= ~UBLOX_GNSS_ENABLE; //Disable SBAS
    }

    if (isM8NwithDefaultConfig && gpsConfig()->gps_ublox_use_galileo) {
        cfg_gnss.configblocks[2].flags |= UBLOX_GNSS_ENABLE; //Enable Galileo
    }

    return ubloxSendConfigMessage(GUMI_CFG_GNSS, &cfg_gnss, gnssConfigLength);
}

void gpsInitUblox(void)
{
    uint32_t now;
    // UBX will run at the serial port's baudrate, it shouldn't be "autodetected". So here we force it to that rate

    // Wait until GPS transmit buffer is empty
    if (!isSerialTransmitBufferEmpty(gpsPort))
        return;

    switch (gpsData.state) {
        case GPS_STATE_INITIALIZING:
            now = millis();
            if (now - gpsData.state_ts < GPS_BAUDRATE_CHANGE_DELAY)
                return;

            if (gpsData.state_position < GPS_INIT_ENTRIES) {
                // try different speed to INIT
                baudRate_e newBaudRateIndex = gpsInitData[gpsData.state_position].baudrateIndex;

                gpsData.state_ts = now;

                if (lookupBaudRateIndex(serialGetBaudRate(gpsPort)) != newBaudRateIndex) {
                    // change the rate if needed and wait a little
                    gpsSetBaudRate(baudRates[newBaudRateIndex]);
#if DEBUG_SERIAL_BAUD
                    debug[0] = baudRates[newBaudRateIndex] / 100;
#endif
                    return;
                }

                // print our FIXED init string for the baudrate we want to be at
                serialPrint(gpsPort, gpsInitData[gpsData.baudrateIndex].ubx);

                gpsData.state_position++;
            } else {
                // we're now (hopefully) at the correct rate, next state will switch to it
                gpsSetState(GPS_STATE_CHANGE_BAUD);
            }
            break;

        case GPS_STATE_CHANGE_BAUD:
            gpsSetBaudRate(baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
#if DEBUG_SERIAL_BAUD
            debug[0] = baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex] / 100;
#endif
            gpsSetState(GPS_STATE_CONFIGURE);
            break;

        case GPS_STATE_CONFIGURE:
            // Either use specific config file for GPS or let dynamically upload config
            if (gpsConfig()->autoConfig == GPS_AUTOCONFIG_OFF) {
                gpsSetState(GPS_STATE_RECEIVING_DATA);
                break;
            }

            if (gpsData.ackState == UBLOX_ACK_IDLE) {
                // A step that sends nothing leaves ackState idle, and so does a
                // message pif_gps_ublox refused: the step runs again next time.
                bool sent = false;

                switch (gpsData.state_position) {
                    case 0:
                        gpsData.ubloxUsePVT = true;
                        gpsData.ubloxUseSAT = true;
                        sent = ubloxSendNAV5Message(gpsConfig()->gps_ublox_mode == UBLOX_AIRBORNE);
                        break;
                    case 1: //Disable NMEA Messages
                        sent = ubloxSetMessageRate(GUCI_NMEA_STD, GUMI_NMEA_VTG, 0); // VGS: Course over ground and Ground speed
                        break;
                    case 2:
                        sent = ubloxSetMessageRate(GUCI_NMEA_STD, GUMI_NMEA_GSV, 0); // GSV: GNSS Satellites in View
                        break;
                    case 3:
                        sent = ubloxSetMessageRate(GUCI_NMEA_STD, GUMI_NMEA_GLL, 0); // GLL: Latitude and longitude, with time of position fix and status
                        break;
                    case 4:
                        sent = ubloxSetMessageRate(GUCI_NMEA_STD, GUMI_NMEA_GGA, 0); // GGA: Global positioning system fix data
                        break;
                    case 5:
                        sent = ubloxSetMessageRate(GUCI_NMEA_STD, GUMI_NMEA_GSA, 0); // GSA: GNSS DOP and Active Satellites
                        break;
                    case 6:
                        sent = ubloxSetMessageRate(GUCI_NMEA_STD, GUMI_NMEA_RMC, 0); // RMC: Recommended Minimum data
                        break;
                    case 7: //Enable UBLOX Messages
                        if (gpsData.ubloxUsePVT) {
                            sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_PVT, 1); // set PVT MSG rate
                        } else {
                            sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_SOL, 1); // set SOL MSG rate
                        }
                        break;
                    case 8:
                        if (gpsData.ubloxUsePVT) {
                            gpsData.state_position++;
                        } else {
                            sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_POSLLH, 1); // set POSLLH MSG rate
                        }
                        break;
                    case 9:
                        if (gpsData.ubloxUsePVT) {
                            gpsData.state_position++;
                        } else {
                            sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_STATUS, 1); // set STATUS MSG rate
                        }
                        break;
                    case 10:
                        if (gpsData.ubloxUsePVT) {
                            gpsData.state_position++;
                        } else {
                            sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_VELNED, 1); // set VELNED MSG rate
                        }
                        break;
                    case 11:
                        if (gpsData.ubloxUseSAT) {
                            sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_SAT, 5); // set SAT MSG rate (every 5 cycles)
                        } else {
                            sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_SVINFO, 5); // set SVINFO MSG rate (every 5 cycles)
                        }
                        break;
                    case 12:
                        sent = ubloxSetNavRate(0xC8, 1, 1); // set rate to 5Hz (measurement period: 200ms, navigation rate: 1 cycle)
                        break;
                    case 13:
                        sent = ubloxSetSbas();
                        break;
                    case 14:
                        if ((gpsConfig()->sbasMode == SBAS_NONE) || (gpsConfig()->gps_ublox_use_galileo)) {
                            gnssConfigLength = 0;
                            sent = ubloxSendPollMessage(GUMI_CFG_GNSS);
                        } else {
                            gpsSetState(GPS_STATE_RECEIVING_DATA);
                        }
                        break;
                    case 15:
                        sent = ubloxSetGnss();
                        break;
                    default:
                        break;
                }

                if (sent) {
                    gpsData.ackState = UBLOX_ACK_WAITING;
                }
            } else if (gpsData.ackState == UBLOX_ACK_WAITING) {
                if (gpsData.state_position == 14 && gnssConfigLength) {
                    // The answer to the poll is the CFG-GNSS itself. Whatever
                    // ACK follows it is left to pif_gps_ublox, which holds the
                    // next message back until it has come or timed out.
                    gpsData.ackState = UBLOX_ACK_GOT_ACK;
                } else {
                    switch (ubloxCheckRequest()) {
                        case GURS_SEND:
                            break;
                        case GURS_ACK:
                            gpsData.ackState = UBLOX_ACK_GOT_ACK;
                            break;
                        case GURS_NAK:
                            gpsData.ackState = UBLOX_ACK_GOT_NACK;
                            break;
                        default:
                            gpsSetState(GPS_STATE_LOST_COMMUNICATION);
                            return;
                    }
                }
            }

            switch (gpsData.ackState) {
                case UBLOX_ACK_IDLE:
                case UBLOX_ACK_WAITING:
                    break;
                case UBLOX_ACK_GOT_ACK:
                    if (gpsData.state_position == 15 || (gpsData.state_position == 14 && !gnssConfigLength)) {
                        // ublox should be initialised, try receiving
                        gpsSetState(GPS_STATE_RECEIVING_DATA);
                    } else {
                        gpsData.state_position++;
                        gpsData.ackState = UBLOX_ACK_IDLE;
                    }
                    break;
                case UBLOX_ACK_GOT_NACK:
                    if (gpsData.state_position == 7) { // If we were asking for NAV-PVT...
                        gpsData.ubloxUsePVT = false;   // ...retry asking for NAV-SOL
                        gpsData.ackState = UBLOX_ACK_IDLE;
                    } else {
                        if (gpsData.state_position == 11) { // If we were asking for NAV-SAT...
                            gpsData.ubloxUseSAT = false;   // ...retry asking for NAV-SVINFO
                            gpsData.ackState = UBLOX_ACK_IDLE;
                        } else {
                            gpsSetState(GPS_STATE_CONFIGURE);
                        }
                    }
                    break;
            }

            break;
    }
}
#endif // USE_GPS_UBLOX

void gpsInitHardware(void)
{
    switch (gpsConfig()->provider) {
    case GPS_NMEA:
#ifdef USE_GPS_NMEA
        gpsInitNmea();
#endif
        break;

    case GPS_UBLOX:
#ifdef USE_GPS_UBLOX
        gpsInitUblox();
#endif
        break;
    default:
        break;
    }
}

static void updateGpsIndicator(timeUs_t currentTimeUs)
{
    static uint32_t GPSLEDTime;
    if ((int32_t)(currentTimeUs - GPSLEDTime) >= 0 && (gpsSol.numSat >= 5)) {
        GPSLEDTime = currentTimeUs + 150000;
        LED1_TOGGLE;
    }
}

uint32_t gpsUpdate(PifTask *p_task)
{
    static gpsState_e gpsStateDurationUs[GPS_STATE_COUNT];
    const timeUs_t currentTimeUs = p_task->_last_execute_time;
    timeUs_t executeTimeUs;
    gpsState_e gpsCurrentState = gpsData.state;
    // The period of the next release, which PIF takes from the return value.
    // The two rates below used to be pushed back through rescheduleTask(TASK_SELF);
    // returning them says the same thing at the point where it is decided.
    uint32_t nextPeriodUs = 0;

    // read out available GPS bytes
    if (gpsPort) {
        while (serialRxBytesWaiting(gpsPort)) {
            if (cmpTimeUs(micros(), currentTimeUs) > GPS_MAX_WAIT_DATA_RX) {
                // Wait 1ms and come back
                return TASK_PERIOD_HZ(TASK_GPS_RATE_FAST);
            }
            gpsNewData(serialRead(gpsPort));
        }
        // Restore default task rate
        nextPeriodUs = TASK_PERIOD_HZ(TASK_GPS_RATE);
#ifdef USE_GPS_UBLOX
        if (gpsConfig()->provider == GPS_UBLOX) {
            ubloxCheckRequest();
        }
#endif
   } else if (GPS_update & GPS_MSP_UPDATE) { // GPS data received via MSP
        gpsSetState(GPS_STATE_RECEIVING_DATA);
        onGpsNewData();
        GPS_update &= ~GPS_MSP_UPDATE;
    }

#if DEBUG_UBLOX_INIT
    debug[0] = gpsData.state;
    debug[1] = gpsData.state_position;
    debug[2] = gpsData.ackState;
#endif

    switch (gpsData.state) {
        case GPS_STATE_UNKNOWN:
        case GPS_STATE_INITIALIZED:
            break;

        case GPS_STATE_INITIALIZING:
        case GPS_STATE_CHANGE_BAUD:
        case GPS_STATE_CONFIGURE:
            gpsInitHardware();
            break;

        case GPS_STATE_LOST_COMMUNICATION:
            gpsData.timeouts++;
            if (gpsConfig()->autoBaud) {
                // try another rate
                gpsData.baudrateIndex++;
                gpsData.baudrateIndex %= GPS_INIT_ENTRIES;
            }
            gpsSol.numSat = 0;
            DISABLE_STATE(GPS_FIX);
            gpsSetState(GPS_STATE_INITIALIZING);
            break;

        case GPS_STATE_RECEIVING_DATA:
            // check for no data/gps timeout/cable disconnection etc
            if (millis() - gpsData.lastMessage > GPS_TIMEOUT) {
                gpsSetState(GPS_STATE_LOST_COMMUNICATION);
#ifdef USE_GPS_UBLOX
            } else {
                // A message pif_gps_ublox refuses because the previous one is
                // still waiting for its ACK is sent again on a later cycle.
                if (gpsConfig()->provider == GPS_UBLOX && gpsConfig()->autoConfig == GPS_AUTOCONFIG_ON) { // Only if autoconfig is enabled
                    switch (gpsData.state_position) {
                        case 0:
                            if (!isConfiguratorConnected()) {
                                bool sent;
                                if (gpsData.ubloxUseSAT) {
                                    sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_SAT, 0); // disable SAT MSG
                                } else {
                                    sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_SVINFO, 0); // disable SVINFO MSG
                                }
                                if (sent) {
                                    gpsData.state_position = 1;
                                }
                            }
                            break;
                        case 1:
                            if (STATE(GPS_FIX) && (gpsConfig()->gps_ublox_mode == UBLOX_DYNAMIC)) {
                                if (ubloxSendNAV5Message(true)) {
                                    gpsData.state_position = 2;
                                }
                            }
                            if (isConfiguratorConnected()) {
                                gpsData.state_position = 2;
                            }
                            break;
                        case 2:
                            if (isConfiguratorConnected()) {
                                bool sent;
                                if (gpsData.ubloxUseSAT) {
                                    sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_SAT, 5); // set SAT MSG rate (every 5 cycles)
                                } else {
                                    sent = ubloxSetMessageRate(GUCI_NAV, GUMI_NAV_SVINFO, 5); // set SVINFO MSG rate (every 5 cycles)
                                }
                                if (sent) {
                                    gpsData.state_position = 0;
                                }
                            }
                            break;
                    }
                }
#endif //USE_GPS_UBLOX
            }
            break;
    }

    executeTimeUs = micros() - currentTimeUs;

    if (executeTimeUs > gpsStateDurationUs[gpsCurrentState]) {
        gpsStateDurationUs[gpsCurrentState] = executeTimeUs;
    }
    schedulerSetNextStateTime(gpsStateDurationUs[gpsData.state]);

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTimeUs);
    }
    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        DISABLE_STATE(GPS_FIX_HOME);
    }

    uint8_t minSats = 5;

#if defined(USE_GPS_RESCUE)
    if (gpsRescueIsConfigured()) {
        updateGPSRescueState();
        minSats = gpsRescueConfig()->minSats;
    }
#endif

    static bool hasFix = false;
    if (STATE(GPS_FIX)) {
        if (gpsIsHealthy() && gpsSol.numSat >= minSats && !hasFix) {
            // ready beep sequence on fix or requirements for gps rescue met.
            beeper(BEEPER_READY_BEEP);
            hasFix = true;
        }
    } else {
        hasFix = false;
    }

    return nextPeriodUs;
}

static void gpsNewData(uint16_t c)
{
    if (!gpsNewFrame(c)) {
        return;
    }

    if (gpsData.state == GPS_STATE_RECEIVING_DATA) {
        // new data received and parsed, we're in business
        gpsData.lastLastMessage = gpsData.lastMessage;
        gpsData.lastMessage = millis();
        sensorsSet(SENSOR_GPS);
    }

    GPS_update ^= GPS_DIRECT_TICK;

#if DEBUG_UBLOX_INIT
    debug[3] = GPS_update;
#endif

    onGpsNewData();
}

// Gives one received byte to the PIF parser of the provider. Returns true when
// it completed a frame that brought a new solution.
bool gpsNewFrame(uint8_t c)
{
    gpsFrameDone = false;

    switch (gpsConfig()->provider) {
    case GPS_NMEA:          // NMEA
#ifdef USE_GPS_NMEA
        pifGps_ParsingNmea(&gpsDriver.nmea, c);
#endif
        break;
    case GPS_UBLOX:         // UBX binary
#ifdef USE_GPS_UBLOX
        pifGpsUblox_ParsingPacket(&gpsDriver.ublox, c);
#endif
        break;
    default:
        break;
    }
    return gpsFrameDone;
}

// Check for healthy communications
bool gpsIsHealthy()
{
    return (gpsData.state == GPS_STATE_RECEIVING_DATA);
}

#ifdef USE_GPS_NMEA
// pif_gps keeps what it decodes as doubles in degrees, metres and cm/s.
static int32_t gpsRound(double value)
{
    return (int32_t)(value >= 0 ? value + (double)0.5f : value - (double)0.5f);
}

/* This should work with most of modern GPS devices configured to output 5 frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus.
   pif_gps parses the sentences and verifies their checksum; a sentence is
   only reported here once the checksum is good.

   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     // added by Mis
     - GPS altitude (for OSD displaying)
     - GPS speed (for OSD displaying)
*/
static BOOL gpsNmeaReceive(PifGps *gps, PifGpsNmeaMsgId msgId)
{
    uint8_t i;

    shiftPacketLog();

    if (msgId == PIF_GPS_NMEA_MSG_ID_ERR) {
        *gpsPacketLogChar = LOG_ERROR;
        return FALSE;
    }

    *gpsPacketLogChar = LOG_IGNORED;
    GPS_packetCount++;

    switch (msgId) {
    case PIF_GPS_NMEA_MSG_ID_GGA:
        *gpsPacketLogChar = LOG_NMEA_GGA;
        gpsSetFixState(gps->_fix);
        if (STATE(GPS_FIX)) {
            gpsSol.llh.lat = gpsRound(gps->_coord_deg[PIF_GPS_LAT] * GPS_DEGREES_DIVIDER);
            gpsSol.llh.lon = gpsRound(gps->_coord_deg[PIF_GPS_LON] * GPS_DEGREES_DIVIDER);
            gpsSol.numSat = gps->_num_sat;
            gpsSol.llh.altCm = gpsRound(gps->_altitude * 100);
            gpsSol.hdop = gps->_hdop;
        }
        gpsFrameDone = true;
        break;

    case PIF_GPS_NMEA_MSG_ID_RMC:
        *gpsPacketLogChar = LOG_NMEA_RMC;
        gpsSol.groundSpeed = gpsRound(gps->_ground_speed);            // cm/s
        gpsSol.groundCourse = gpsRound(gps->_ground_course * 10);     // deg * 10
#ifdef USE_RTC_TIME
        // pif_gps leaves the date at 0 until a sentence has carried one
        if (!rtcHasTime() && gps->_utc.day != 0) {
            dateTime_t temp_time;
            temp_time.year = gps->_utc.year + 2000;
            temp_time.month = gps->_utc.month;
            temp_time.day = gps->_utc.day;
            temp_time.hours = gps->_utc.hour;
            temp_time.minutes = gps->_utc.minute;
            temp_time.seconds = gps->_utc.second;
            temp_time.millis = gps->_utc.millisecond;
            rtcSetDateTime(&temp_time);
        }
#endif
        break;

    case PIF_GPS_NMEA_MSG_ID_GSV:
        GPS_numCh = MIN(gps->_sv_num_sv, GPS_SV_MAXSATS_LEGACY);
        for (i = 0; i < GPS_SV_MAXSATS_LEGACY; i++) {
            GPS_svinfo_chn[i] = gps->_sv_chn[i];
            GPS_svinfo_svid[i] = gps->_sv_svid[i];
            GPS_svinfo_quality[i] = gps->_sv_quality[i];
            GPS_svinfo_cno[i] = gps->_sv_cno[i];
        }
        GPS_svInfoReceivedCount++;
        break;

    default:
        break;
    }

    // Nothing to report through pifGps_SendEvent().
    return FALSE;
}

STATIC_ASSERT(PIF_GPS_SV_MAXSATS >= GPS_SV_MAXSATS_LEGACY, pif_gps_holds_legacy_sv_count);
#endif // USE_GPS_NMEA

#ifdef USE_GPS_UBLOX
// UBX support

// Example packet sizes from UBlox u-center from a Glonass capable GPS receiver.
//15:17:55  R -> UBX NAV-STATUS,  Size  24,  'Navigation Status'
//15:17:55  R -> UBX NAV-POSLLH,  Size  36,  'Geodetic Position'
//15:17:55  R -> UBX NAV-VELNED,  Size  44,  'Velocity in WGS 84'
//15:17:55  R -> UBX NAV-CLOCK,  Size  28,  'Clock Status'
//15:17:55  R -> UBX NAV-AOPSTATUS,  Size  24,  'AOP Status'
//15:17:55  R -> UBX 03-09,  Size 208,  'Unknown'
//15:17:55  R -> UBX 03-10,  Size 336,  'Unknown'
//15:17:55  R -> UBX NAV-SOL,  Size  60,  'Navigation Solution'
//15:17:55  R -> UBX NAV,  Size 100,  'Navigation'
//15:17:55  R -> UBX NAV-SVINFO,  Size 328,  'Satellite Status and Information'

static bool next_fix;

// do we have new position information?
static bool _new_position;

// do we have new speed information?
static bool _new_speed;

static void ubloxParseNav(const PifGpsUbxPacket *packet)
{
    uint32_t i;

    switch (packet->msg_id) {
    case GUMI_NAV_POSLLH:
        *gpsPacketLogChar = LOG_UBLOX_POSLLH;
        gpsSol.llh.lon = packet->payload.posllh.lon;
        gpsSol.llh.lat = packet->payload.posllh.lat;
        gpsSol.llh.altCm = packet->payload.posllh.h_msl / 10;  //alt in cm
        gpsSetFixState(next_fix);
        _new_position = true;
        break;
    case GUMI_NAV_STATUS:
        *gpsPacketLogChar = LOG_UBLOX_STATUS;
        next_fix = (packet->payload.status.flags & NAV_STATUS_FIX_VALID) && (packet->payload.status.gps_fix == FIX_3D);
        if (!next_fix)
            DISABLE_STATE(GPS_FIX);
        break;
    case GUMI_NAV_SOL:
        *gpsPacketLogChar = LOG_UBLOX_SOL;
        next_fix = (packet->payload.sol.flags & NAV_STATUS_FIX_VALID) && (packet->payload.sol.gps_fix == FIX_3D);
        if (!next_fix)
            DISABLE_STATE(GPS_FIX);
        gpsSol.numSat = packet->payload.sol.num_sv;
        gpsSol.hdop = packet->payload.sol.p_dop;
#ifdef USE_RTC_TIME
        //set clock, when gps time is available
        if(!rtcHasTime() && (packet->payload.sol.flags & NAV_STATUS_TIME_SECOND_VALID) && (packet->payload.sol.flags & NAV_STATUS_TIME_WEEK_VALID)) {
            //calculate rtctime: week number * ms in a week + ms of week + fractions of second + offset to UNIX reference year - 18 leap seconds
            rtcTime_t temp_time = (((int64_t) packet->payload.sol.week)*7*24*60*60*1000) + packet->payload.sol.i_tow + (packet->payload.sol.f_tow/1000000) + 315964800000LL - 18000;
            rtcSet(&temp_time);
        }
#endif
        break;
    case GUMI_NAV_VELNED:
        *gpsPacketLogChar = LOG_UBLOX_VELNED;
        gpsSol.speed3d = packet->payload.velned.speed;       // cm/s
        gpsSol.groundSpeed = packet->payload.velned.g_speed;    // cm/s
        gpsSol.groundCourse = (uint16_t) (packet->payload.velned.heading / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        _new_speed = true;
        break;
    case GUMI_NAV_PVT:
        *gpsPacketLogChar = LOG_UBLOX_SOL;
        next_fix = (packet->payload.pvt.flags & NAV_STATUS_FIX_VALID) && (packet->payload.pvt.fix_type == FIX_3D);
        gpsSol.llh.lon = packet->payload.pvt.lon;
        gpsSol.llh.lat = packet->payload.pvt.lat;
        gpsSol.llh.altCm = packet->payload.pvt.h_msl / 10;  //alt in cm
        gpsSetFixState(next_fix);
        _new_position = true;
        gpsSol.numSat = packet->payload.pvt.num_sv;
        gpsSol.hdop = packet->payload.pvt.p_dop;
        gpsSol.speed3d = (uint16_t) sqrtf(powf(packet->payload.pvt.g_speed / 10, 2.0f) + powf(packet->payload.pvt.val_d / 10, 2.0f));
        gpsSol.groundSpeed = packet->payload.pvt.g_speed / 10;    // cm/s
        gpsSol.groundCourse = (uint16_t) (packet->payload.pvt.head_mot / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        _new_speed = true;
#ifdef USE_RTC_TIME
        //set clock, when gps time is available
        if (!rtcHasTime() && (packet->payload.pvt.valid & NAV_VALID_DATE) && (packet->payload.pvt.valid & NAV_VALID_TIME)) {
            dateTime_t dt;
            dt.year = packet->payload.pvt.year;
            dt.month = packet->payload.pvt.month;
            dt.day = packet->payload.pvt.day;
            dt.hours = packet->payload.pvt.hour;
            dt.minutes = packet->payload.pvt.min;
            dt.seconds = packet->payload.pvt.sec;
            dt.millis = (packet->payload.pvt.nano > 0) ? packet->payload.pvt.nano / 1000000 : 0;
            rtcSetDateTime(&dt);
        }
#endif
        break;
    case GUMI_NAV_SVINFO:
        *gpsPacketLogChar = LOG_UBLOX_SVINFO;
        GPS_numCh = packet->payload.sv_info.num_ch;
        // If we're getting NAV-SVINFO is because we're dealing with an old receiver that does not support NAV-SAT, so we'll only
        // save up to GPS_SV_MAXSATS_LEGACY channels so the BF Configurator knows it's receiving the old sat list info format.
        if (GPS_numCh > GPS_SV_MAXSATS_LEGACY)
            GPS_numCh = GPS_SV_MAXSATS_LEGACY;
        for (i = 0; i < GPS_numCh; i++) {
            GPS_svinfo_chn[i] = packet->payload.sv_info.channel[i].chn;
            GPS_svinfo_svid[i] = packet->payload.sv_info.channel[i].svid;
            GPS_svinfo_quality[i] = packet->payload.sv_info.channel[i].quality;
            GPS_svinfo_cno[i] = packet->payload.sv_info.channel[i].cno;
        }
        for (i = GPS_numCh; i < GPS_SV_MAXSATS_LEGACY; i++) {
            GPS_svinfo_chn[i] = 0;
            GPS_svinfo_svid[i] = 0;
            GPS_svinfo_quality[i] = 0;
            GPS_svinfo_cno[i] = 0;
        }
        GPS_svInfoReceivedCount++;
        break;
    case GUMI_NAV_SAT:
        {
            const ubx_nav_sat *sat = (const ubx_nav_sat *)packet->payload.bytes;

            *gpsPacketLogChar = LOG_UBLOX_SVINFO; // The logger won't show this is NAV-SAT instead of NAV-SVINFO
            GPS_numCh = sat->numSvs;
            // We can receive here upto GPS_SV_MAXSATS_M9N channels, but since the majority of receivers currently in use are M8N or older,
            // it would be a waste of RAM to size the arrays that big. For now, they're sized GPS_SV_MAXSATS_M8N which means M9N won't show
            // all their channel information on BF Configurator. When M9N's are more widespread it would be a good time to increase those arrays.
            if (GPS_numCh > GPS_SV_MAXSATS_M8N)
                GPS_numCh = GPS_SV_MAXSATS_M8N;
            for (i = 0; i < GPS_numCh; i++) {
                GPS_svinfo_chn[i] = sat->svs[i].gnssId;
                GPS_svinfo_svid[i] = sat->svs[i].svId;
                GPS_svinfo_cno[i] = sat->svs[i].cno;
                GPS_svinfo_quality[i] = sat->svs[i].flags;
            }
            for (i = GPS_numCh; i < GPS_SV_MAXSATS_M8N; i++) {
                GPS_svinfo_chn[i] = 255;
                GPS_svinfo_svid[i] = 0;
                GPS_svinfo_quality[i] = 0;
                GPS_svinfo_cno[i] = 0;
            }

            // Setting the number of channels higher than GPS_SV_MAXSATS_LEGACY is the only way to tell BF Configurator we're sending the
            // enhanced sat list info without changing the MSP protocol. Also, we're sending the complete list each time even if it's empty, so
            // BF Conf can erase old entries shown on screen when channels are removed from the list.
            GPS_numCh = GPS_SV_MAXSATS_M8N;
            GPS_svInfoReceivedCount++;
        }
        break;
    default:
        break;
    }
}

// evt_ubx_receive of pif_gps_ublox: called for every UBX packet whose checksum
// is good, once pif_gps_ublox has matched an ACK or NAK against the request
// on its way.
static BOOL gpsUbloxReceive(PifGpsUblox *ublox, PifGpsUbxPacket *packet)
{
    UNUSED(ublox);

#if DEBUG_UBLOX_FRAMES
    debug[2] = packet->msg_id;
    debug[3] = packet->length;
#endif

    shiftPacketLog();
    GPS_packetCount++;
    *gpsPacketLogChar = LOG_IGNORED;

    switch (packet->class_id) {
    case GUCI_NAV:
        ubloxParseNav(packet);
        break;
    case GUCI_CFG:
        if (packet->msg_id == GUMI_CFG_GNSS && packet->length <= sizeof(gnssConfig)) {
            // The answer to the poll of state 14, sent back from state 15.
            memcpy(&gnssConfig, packet->payload.bytes, packet->length);
            gnssConfigLength = packet->length;
        }
        break;
    default:
        break;
    }

    // we only report a frame when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed) {
        _new_speed = _new_position = false;
        gpsFrameDone = true;
    }

    // Nothing to report through pifGps_SendEvent().
    return FALSE;
}

// evt_ubx_error of pif_gps_ublox: a packet was lost.
static void gpsUbloxError(PifGpsUblox *ublox, PifGpsUbxError error)
{
    UNUSED(ublox);

    switch (error) {
    case GUE_WRONG_CRC:
        shiftPacketLog();
        *gpsPacketLogChar = LOG_ERROR;
        gpsData.errors++;
        break;
    case GUE_BIG_LENGTH:
        shiftPacketLog();
        *gpsPacketLogChar = LOG_SKIPPED;
        break;
    default:
        // A lost sync byte, which the native parser did not count either.
        break;
    }
}
#endif // USE_GPS_UBLOX

static void gpsHandlePassthrough(uint8_t data)
{
     gpsNewData(data);
 #ifdef USE_DASHBOARD
     if (featureIsEnabled(FEATURE_DASHBOARD)) {
         dashboardUpdate(micros());
     }
 #endif

 }

void gpsEnablePassthrough(serialPort_t *gpsPassthroughPort)
{
    waitForSerialPortToFinishTransmitting(gpsPort);
    waitForSerialPortToFinishTransmitting(gpsPassthroughPort);

    if (!(gpsPort->mode & MODE_TX))
        serialSetMode(gpsPort, gpsPort->mode | MODE_TX);

#ifdef USE_DASHBOARD
    if (featureIsEnabled(FEATURE_DASHBOARD)) {
        dashboardShowFixedPage(PAGE_GPS);
    }
#endif

    serialPassthrough(gpsPort, gpsPassthroughPort, &gpsHandlePassthrough, NULL);
}

float GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles

void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (fabsf((float)lat) / 10000000.0f) * 0.0174532925f;
    GPS_scaleLonDown = cos_approx(rads);
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the distance flown and vertical speed from gps position data
//
static void GPS_calculateDistanceFlownVerticalSpeed(bool initialize)
{
    static int32_t lastCoord[2] = { 0, 0 };
    static int32_t lastAlt;
    static int32_t lastMillis;

    int currentMillis = millis();

    if (initialize) {
        GPS_distanceFlownInCm = 0;
        GPS_verticalSpeedInCmS = 0;
    } else {
        if (STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED)) {
            uint16_t speed = gpsConfig()->gps_use_3d_speed ? gpsSol.speed3d : gpsSol.groundSpeed;
            // Only add up movement when speed is faster than minimum threshold
            if (speed > GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S) {
                uint32_t dist;
                int32_t dir;
                GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &lastCoord[GPS_LATITUDE], &lastCoord[GPS_LONGITUDE], &dist, &dir);
                if (gpsConfig()->gps_use_3d_speed) {
                    dist = sqrtf(powf(gpsSol.llh.altCm - lastAlt, 2.0f) + powf(dist, 2.0f));
                }
                GPS_distanceFlownInCm += dist;
            }
        }
        GPS_verticalSpeedInCmS = (gpsSol.llh.altCm - lastAlt) * 1000 / (currentMillis - lastMillis);
        GPS_verticalSpeedInCmS = constrain(GPS_verticalSpeedInCmS, -1500, 1500);
    }
    lastCoord[GPS_LONGITUDE] = gpsSol.llh.lon;
    lastCoord[GPS_LATITUDE] = gpsSol.llh.lat;
    lastAlt = gpsSol.llh.altCm;
    lastMillis = currentMillis;
}

void GPS_reset_home_position(void)
{
    if (!STATE(GPS_FIX_HOME) || !gpsConfig()->gps_set_home_point_once) {
        if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
            GPS_home[GPS_LATITUDE] = gpsSol.llh.lat;
            GPS_home[GPS_LONGITUDE] = gpsSol.llh.lon;
            GPS_calc_longitude_scaling(gpsSol.llh.lat); // need an initial value for distance and bearing calc
            // Set ground altitude
            ENABLE_STATE(GPS_FIX_HOME);
        }
    }
    GPS_calculateDistanceFlownVerticalSpeed(true); //Initialize
}

////////////////////////////////////////////////////////////////////////////////////
#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS 1.113195f
#define TAN_89_99_DEGREES 5729.57795f
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing)
{
    float dLat = *destinationLat2 - *currentLat1; // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(*destinationLon2 - *currentLon1) * GPS_scaleLonDown;
    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS;

    *bearing = 9000.0f + atan2_approx(-dLat, dLon) * TAN_89_99_DEGREES;      // Convert the output radians to 100xdeg
    if (*bearing < 0)
        *bearing += 36000;
}

void GPS_calculateDistanceAndDirectionToHome(void)
{
    if (STATE(GPS_FIX_HOME)) {      // If we don't have home set, do not display anything
        uint32_t dist;
        int32_t dir;
        GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &GPS_home[GPS_LATITUDE], &GPS_home[GPS_LONGITUDE], &dist, &dir);
        GPS_distanceToHome = dist / 100;
        GPS_directionToHome = dir / 100;
    } else {
        GPS_distanceToHome = 0;
        GPS_directionToHome = 0;
    }
}

void onGpsNewData(void)
{
    if (!(STATE(GPS_FIX) && gpsSol.numSat >= 5)) {
        return;
    }

    //
    // Calculate time delta for navigation loop, range 0-1.0f, in seconds
    //
    // Time for calculating x,y speed and navigation pids
    static uint32_t nav_loopTimer;
    dTnav = (float)(millis() - nav_loopTimer) / 1000.0f;
    nav_loopTimer = millis();
    // prevent runup from bad GPS
    dTnav = MIN(dTnav, 1.0f);

    GPS_calculateDistanceAndDirectionToHome();
    if (ARMING_FLAG(ARMED)) {
        GPS_calculateDistanceFlownVerticalSpeed(false);
    }

#ifdef USE_GPS_RESCUE
    rescueNewGpsData();
#endif
}

void gpsSetFixState(bool state)
{
    if (state) {
        ENABLE_STATE(GPS_FIX);
        ENABLE_STATE(GPS_FIX_EVER);
    } else {
        DISABLE_STATE(GPS_FIX);
    }
}
#endif
