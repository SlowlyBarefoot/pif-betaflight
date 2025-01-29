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

#include "platform.h"

#ifdef USE_MAX7456

#include "build/debug.h"

#include "pg/max7456.h"
#include "pg/vcd.h"

#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/max7456.h"
#include "drivers/nvic.h"
#include "drivers/osd.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"

// 10 MHz max SPI frequency
#define MAX7456_MAX_SPI_CLK_HZ 10000000
#define MAX7456_INIT_MAX_SPI_CLK_HZ 5000000

// DEBUG_MAX7456_SIGNAL
#define DEBUG_MAX7456_SIGNAL_MODEREG       0
#define DEBUG_MAX7456_SIGNAL_SENSE         1
#define DEBUG_MAX7456_SIGNAL_REINIT        2
#define DEBUG_MAX7456_SIGNAL_ROWS          3

// DEBUG_MAX7456_SPICLOCK
#define DEBUG_MAX7456_SPICLOCK_OVERCLOCK   0
#define DEBUG_MAX7456_SPICLOCK_DEVTYPE     1
#define DEBUG_MAX7456_SPICLOCK_DIVISOR     2
#define DEBUG_MAX7456_SPICLOCK_X100        3

// VM0 bits
#define VIDEO_MODE_IS_PAL(val)      (((val) & MAX7456_VIDEO_STANDARD_SELECT_MASK) == MAX7456_VIDEO_STANDARD_SELECT_PAL)
#define VIDEO_MODE_IS_NTSC(val)     (((val) & MAX7456_VIDEO_STANDARD_SELECT_MASK) == MAX7456_VIDEO_STANDARD_SELECT_NTSC)

#define VIDEO_SIGNAL_DEBOUNCE_MS    100 // Time to wait for input to stabilize

// STAT register bits
#define STAT_IS_PAL(val)  ((val) & MAX7456_PAL_SIGNAL_DETECT_MASK)
#define STAT_IS_NTSC(val) ((val) & MAX7456_NTSC_SIGNAL_DETECT_MASK)
#define STAT_IS_LOS(val)  ((val) & MAX7456_LOSS_OF_SYNC_MASK)

#define VIN_IS_PAL(val)  (!STAT_IS_LOS(val) && STAT_IS_PAL(val))
#define VIN_IS_NTSC(val)  (!STAT_IS_LOS(val) && STAT_IS_NTSC(val))


// Kluege warning!
// There are occasions that NTSC is not detected even with !LOS (AB7456 specific?)
// When this happens, lower 3 bits of STAT register is read as zero.
// To cope with this case, this macro defines !LOS && !PAL as NTSC.
// Should be compatible with MAX7456 and non-problematic case.

#define VIN_IS_NTSC_alt(val)  (!STAT_IS_LOS(val) && !STAT_IS_PAL(val))

#define MAX7456_SIGNAL_CHECK_INTERVAL_MS 1000 // msec
#define MAX7456_STALL_CHECK_INTERVAL_MS  1000 // msec

// Special address for terminating incremental write
#define END_STRING 0xff

#define NVM_RAM_SIZE            54
#define WRITE_NVR               0xA0

// Device type
#define MAX7456_DEVICE_TYPE_MAX 0
#define MAX7456_DEVICE_TYPE_AT  1

#define CHARS_PER_LINE      30 // XXX Should be related to VIDEO_BUFFER_CHARS_*?

#define MAX7456_SUPPORTED_LAYER_COUNT (DISPLAYPORT_LAYER_BACKGROUND + 1)

typedef struct max7456Layer_s {
    uint8_t buffer[VIDEO_BUFFER_CHARS_PAL];
} max7456Layer_t;

static max7456Layer_t displayLayers[MAX7456_SUPPORTED_LAYER_COUNT];
static displayPortLayer_e activeLayer = DISPLAYPORT_LAYER_FOREGROUND;

extDevice_t max7456Device;
extDevice_t *dev = &max7456Device;

static uint16_t max7456SpiClockDiv;

uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

// We write everything to the active layer and then compare
// it with shadowBuffer to update only changed chars.
// This solution is faster then redrawing entire screen.

static uint8_t shadowBuffer[VIDEO_BUFFER_CHARS_PAL];

//Max bytes to update in one call to max7456DrawScreen()

#define MAX_BYTES2SEND          250
#define MAX_BYTES2SEND_POLLED   12
#define MAX_ENCODE_US           20
#define MAX_ENCODE_US_POLLED    10

static DMA_DATA uint8_t spiBuf[MAX_BYTES2SEND];

static uint8_t  videoSignalCfg;
static uint8_t  videoSignalReg  = MAX7456_ENABLE_DISP_OSD_IMAGE(1); // OSD_ENABLE required to trigger first ReInit

static uint8_t  hosRegValue; // HOS (Horizontal offset register) value
static uint8_t  vosRegValue; // VOS (Vertical offset register) value

static uint8_t max7456DeviceType;

static displayPortBackground_e deviceBackgroundType = DISPLAY_BACKGROUND_TRANSPARENT;

static PifSpiPort *p_spi_port = NULL;
PifMax7456 max7456;

static uint8_t *getLayerBuffer(displayPortLayer_e layer)
{
    return displayLayers[layer].buffer;
}

static uint8_t *getActiveLayerBuffer(void)
{
    return getLayerBuffer(activeLayer);
}

static void max7456SetRegisterVM1(void)
{
    uint8_t backgroundGray = MAX7456_BG_MODE_BRIGHT_28; // this is the device default background gray level
    uint8_t vm1Register = MAX7456_BLINK_TIME_4 | MAX7456_BLINK_DUTY_CYCLE_3BT_BT; // device defaults
    if (deviceBackgroundType != DISPLAY_BACKGROUND_TRANSPARENT) {
        vm1Register |= MAX7456_BG_MODE_GRAY;
        switch (deviceBackgroundType) {
        case DISPLAY_BACKGROUND_BLACK:
            backgroundGray = MAX7456_BG_MODE_BRIGHT_0;
            break;
        case DISPLAY_BACKGROUND_LTGRAY:
            backgroundGray = MAX7456_BG_MODE_BRIGHT_49;
            break;
        case DISPLAY_BACKGROUND_GRAY:
        default:
            backgroundGray = MAX7456_BG_MODE_BRIGHT_28;
            break;
        }
    }
    vm1Register |= backgroundGray;
	pifSpiDevice_WriteRegByte(max7456._p_spi, MAX7456_REG_W_VM1, vm1Register);
}

uint8_t max7456GetRowsCount(void)
{
    return (videoSignalReg & MAX7456_VIDEO_STANDARD_SELECT_PAL) ? VIDEO_LINES_PAL : VIDEO_LINES_NTSC;
}

// When clearing the shadow buffer we fill with 0 so that the characters will
// be flagged as changed when compared to the 0x20 used in the layer buffers.
void max7456ClearShadowBuffer(void)
{
    memset(shadowBuffer, 0, maxScreenSize);
}

// Buffer is filled with the whitespace character (0x20)
static void max7456ClearLayer(displayPortLayer_e layer)
{
    memset(getLayerBuffer(layer), 0x20, VIDEO_BUFFER_CHARS_PAL);
}

void max7456ReInit(void)
{
    uint8_t srdata = 0;

    switch (videoSignalCfg) {
    case VIDEO_SYSTEM_PAL:
        videoSignalReg = MAX7456_VIDEO_STANDARD_SELECT_PAL | MAX7456_ENABLE_DISP_OSD_IMAGE(1);
        break;

    case VIDEO_SYSTEM_NTSC:
        videoSignalReg = MAX7456_VIDEO_STANDARD_SELECT_NTSC | MAX7456_ENABLE_DISP_OSD_IMAGE(1);
        break;

    case VIDEO_SYSTEM_AUTO:
        pifSpiDevice_ReadRegByte(max7456._p_spi, MAX7456_REG_R_STAT, &srdata);

        if (VIN_IS_NTSC(srdata)) {
            videoSignalReg = MAX7456_VIDEO_STANDARD_SELECT_NTSC | MAX7456_ENABLE_DISP_OSD_IMAGE(1);
        } else if (VIN_IS_PAL(srdata)) {
            videoSignalReg = MAX7456_VIDEO_STANDARD_SELECT_PAL | MAX7456_ENABLE_DISP_OSD_IMAGE(1);
        } else {
            // No valid input signal, fallback to default (XXX NTSC for now)
            videoSignalReg = MAX7456_VIDEO_STANDARD_SELECT_NTSC | MAX7456_ENABLE_DISP_OSD_IMAGE(1);
        }
        break;
    }

    if (videoSignalReg & MAX7456_VIDEO_STANDARD_SELECT_PAL) { //PAL
        maxScreenSize = VIDEO_BUFFER_CHARS_PAL;
    } else {              // NTSC
        maxScreenSize = VIDEO_BUFFER_CHARS_NTSC;
    }

    // Set all rows to same charactor black/white level
    pifMax7456_InitBrightness(&max7456);
    pifMax7456_Brightness(&max7456, 0, 2);
    // Re-enable MAX7456 (last function call disables it)

    // Make sure the Max7456 is enabled
    pifSpiDevice_WriteRegByte(max7456._p_spi, MAX7456_REG_W_VM0, videoSignalReg);
    pifSpiDevice_WriteRegByte(max7456._p_spi, MAX7456_REG_W_HOS, hosRegValue);
    pifSpiDevice_WriteRegByte(max7456._p_spi, MAX7456_REG_W_VOS, vosRegValue);

    max7456SetRegisterVM1();

    // Clear shadow to force redraw all screen
    max7456ClearShadowBuffer();
}

void max7456PreInit(const max7456Config_t *max7456Config)
{
    spiPreinitRegister(max7456Config->csTag, max7456Config->preInitOPU ? IOCFG_OUT_PP : IOCFG_IPU, 1);
}

// Here we init only CS and try to init MAX for first time.
// Also detect device type (MAX v.s. AT)

max7456InitStatus_e max7456Init(const max7456Config_t *max7456Config, const vcdProfile_t *pVcdProfile, bool cpuOverclock)
{
    uint8_t data;

    p_spi_port = &spiDevice[SPI_CFG_TO_DEV(max7456Config->spiDevice)].spi_port;

    if (!pifMax7456_Init(&max7456, PIF_ID_AUTO, p_spi_port)) return MAX7456_INIT_NOT_CONFIGURED;

    deviceBackgroundType = DISPLAY_BACKGROUND_TRANSPARENT;

    // initialize all layers
    for (unsigned i = 0; i < MAX7456_SUPPORTED_LAYER_COUNT; i++) {
        max7456ClearLayer(i);
    }

    max7456HardwareReset();

    if (!max7456Config->csTag || !spiSetBusInstance(dev, max7456Config->spiDevice)) {
        return MAX7456_INIT_NOT_CONFIGURED;
    }

    dev->busType_u.spi.csnPin = IOGetByTag(max7456Config->csTag);

    if (!IOIsFreeOrPreinit(dev->busType_u.spi.csnPin)) {
        return MAX7456_INIT_NOT_CONFIGURED;
    }

    IOInit(dev->busType_u.spi.csnPin, OWNER_OSD_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(dev->busType_u.spi.csnPin);

    // Detect MAX7456 existence and device type. Do this at half the speed for safety.

    // Detect MAX7456 and compatible device by reading OSDM (OSD Insertion MUX) register.
    // This register is not modified in this driver, therefore ensured to remain at its default value (0x1B).

    spiSetClkDivisor(dev, spiCalculateDivider(MAX7456_INIT_MAX_SPI_CLK_HZ));

    // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
    data = END_STRING;
    pifSpiDevice_Transfer(max7456._p_spi, &data, NULL, sizeof(data));

    uint8_t osdm;
    pifSpiDevice_ReadRegByte(max7456._p_spi, MAX7456_REG_R_OSDM, &osdm);

    if (osdm != 0x1B) {
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_IPU);
        return MAX7456_INIT_NOT_FOUND;
    }

    // At this point, we can claim the ownership of the CS pin
    max7456.detected = true;
    IOInit(dev->busType_u.spi.csnPin, OWNER_OSD_CS, 0);

    // Detect device type by writing and reading CA[8] bit at CMAL[6].
    // This is a bit for accessing second half of character glyph storage, supported only by AT variant.

    pifSpiDevice_WriteRegByte(max7456._p_spi, MAX7456_REG_W_CMAL, (1 << 6));

    pifSpiDevice_ReadRegByte(max7456._p_spi, MAX7456_REG_R_CMAL, &data);
    if (data & (1 << 6)) {
        max7456DeviceType = MAX7456_DEVICE_TYPE_AT;
    } else {
        max7456DeviceType = MAX7456_DEVICE_TYPE_MAX;
    }

#if defined(USE_OVERCLOCK)
    // Determine SPI clock divisor based on config and the device type.

    switch (max7456Config->clockConfig) {
    case MAX7456_CLOCK_CONFIG_HALF:
        max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ / 2);
        break;

    case MAX7456_CLOCK_CONFIG_NOMINAL:
    default:
        max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ);
        break;

    case MAX7456_CLOCK_CONFIG_DOUBLE:
        max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ * 2);
        break;
    }

    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_OVERCLOCK, cpuOverclock);
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_DEVTYPE, max7456DeviceType);
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_DIVISOR, max7456SpiClockDiv);
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_X100, spiCalculateClock(max7456SpiClockDiv) / 10000);
#else
    UNUSED(max7456Config);
    UNUSED(cpuOverclock);
    max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ);
#endif

    spiSetClkDivisor(dev, max7456SpiClockDiv);

    // force soft reset on Max7456
    pifSpiDevice_WriteRegByte(max7456._p_spi, MAX7456_REG_W_VM0, MAX7456_SOFTWARE_RESET_BIT(1));

    // Wait for 200us before polling for completion of reset
    delayMicroseconds(200);

    // Wait for reset to complete
    do {
        pifSpiDevice_ReadRegBit8(max7456._p_spi, MAX7456_REG_R_VM0, MAX7456_SOFTWARE_RESET_BIT_MASK, &data);
    } while (data != 0x00);

    // Setup values to write to registers
    videoSignalCfg = pVcdProfile->video_system;
    hosRegValue = 32 - pVcdProfile->h_offset;
    vosRegValue = 16 - pVcdProfile->v_offset;

    // Real init will be made later when driver detect idle.
    return MAX7456_INIT_OK;
}

void max7456ClearScreen(void)
{
    max7456ClearLayer(activeLayer);
}

void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    uint8_t *buffer = getActiveLayerBuffer();
    if (x < CHARS_PER_LINE && y < VIDEO_LINES_PAL) {
        buffer[y * CHARS_PER_LINE + x] = c;
    }
}

void max7456Write(uint8_t x, uint8_t y, const char *buff)
{
    if (y < VIDEO_LINES_PAL) {
        uint8_t *buffer = getActiveLayerBuffer();
        for (int i = 0; buff[i] && x + i < CHARS_PER_LINE; i++) {
            buffer[y * CHARS_PER_LINE + x + i] = buff[i];
        }
    }
}

bool max7456LayerSupported(displayPortLayer_e layer)
{
    if (layer == DISPLAYPORT_LAYER_FOREGROUND || layer == DISPLAYPORT_LAYER_BACKGROUND) {
        return true;
    } else {
        return false;
    }
}

bool max7456LayerSelect(displayPortLayer_e layer)
{
    if (max7456LayerSupported(layer)) {
        activeLayer = layer;
        return true;
    } else {
        return false;
    }
}

bool max7456LayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    if ((sourceLayer != destLayer) && max7456LayerSupported(sourceLayer) && max7456LayerSupported(destLayer)) {
        memcpy(getLayerBuffer(destLayer), getLayerBuffer(sourceLayer), VIDEO_BUFFER_CHARS_PAL);
        return true;
    } else {
        return false;
    }
}

bool max7456DmaInProgress(void)
{
    return pifSpiDevice_IsBusy(max7456._p_spi);
}

bool max7456BuffersSynced(void)
{
    for (int i = 0; i < maxScreenSize; i++) {
        if (displayLayers[DISPLAYPORT_LAYER_FOREGROUND].buffer[i] != shadowBuffer[i]) {
            return false;
        }
    }
    return true;
}

bool max7456ReInitIfRequired(bool forceStallCheck)
{
    uint8_t data;
    static timeMs_t lastSigCheckMs = 0;
    static timeMs_t videoDetectTimeMs = 0;
    static uint16_t reInitCount = 0;
    static timeMs_t lastStallCheckMs = MAX7456_STALL_CHECK_INTERVAL_MS / 2; // offset so that it doesn't coincide with the signal check

    const timeMs_t nowMs = millis();

    bool stalled = false;
    if (forceStallCheck || (lastStallCheckMs + MAX7456_STALL_CHECK_INTERVAL_MS < nowMs)) {
        lastStallCheckMs = nowMs;

        // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
        data = END_STRING;
        pifSpiDevice_Transfer(max7456._p_spi, &data, NULL, sizeof(data));

        pifSpiDevice_ReadRegByte(max7456._p_spi, MAX7456_REG_R_VM0, &data);
        stalled = (data != videoSignalReg);
    }

    if (stalled) {
        max7456ReInit();
    } else if ((videoSignalCfg == VIDEO_SYSTEM_AUTO)
              && ((nowMs - lastSigCheckMs) > MAX7456_SIGNAL_CHECK_INTERVAL_MS)) {

        // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
        data = END_STRING;
        pifSpiDevice_Transfer(max7456._p_spi, &data, NULL, sizeof(data));

        // Adjust output format based on the current input format.

        uint8_t videoSense;
        pifSpiDevice_ReadRegByte(max7456._p_spi, MAX7456_REG_R_STAT, &videoSense);

        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_MODEREG, videoSignalReg & MAX7456_VIDEO_STANDARD_SELECT_MASK);
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_SENSE, videoSense & 0x7);
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_ROWS, max7456GetRowsCount());

        if (videoSense & MAX7456_LOSS_OF_SYNC_MASK) {
            videoDetectTimeMs = 0;
        } else {
            if ((VIN_IS_PAL(videoSense) && VIDEO_MODE_IS_NTSC(videoSignalReg))
              || (VIN_IS_NTSC_alt(videoSense) && VIDEO_MODE_IS_PAL(videoSignalReg))) {
                if (videoDetectTimeMs) {
                    if (millis() - videoDetectTimeMs > VIDEO_SIGNAL_DEBOUNCE_MS) {
                        max7456ReInit();
                        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_REINIT, ++reInitCount);
                    }
                } else {
                    // Wait for signal to stabilize
                    videoDetectTimeMs = millis();
                }
            }
        }

        lastSigCheckMs = nowMs;
    }

    return stalled;
}

// Return true if screen still being transferred
bool max7456DrawScreen(void)
{
    static uint16_t pos = 0;

    if (!max7456._font_is_loading) {
        uint8_t *buffer = getActiveLayerBuffer();
        int spiBufIndex = 0;
        int maxSpiBufStartIndex;
        timeDelta_t maxEncodeTime;
        bool setAddress = true;
        bool autoInc = false;
        int posLimit = pos + (maxScreenSize / 2);

        maxSpiBufStartIndex = spiUseMOSI_DMA(dev) ? MAX_BYTES2SEND : MAX_BYTES2SEND_POLLED;
        maxEncodeTime = spiUseMOSI_DMA(dev) ? MAX_ENCODE_US : MAX_ENCODE_US_POLLED;

        // Abort for now if the bus is still busy
        if (pifSpiDevice_IsBusy(max7456._p_spi)) {
            // Not finished yet
            return true;
        }

        timeUs_t startTime = micros();

        // Allow for an ESCAPE, a reset of DMM and a two byte MAX7456_REG_W_DMM command at end of buffer
        maxSpiBufStartIndex -= 4;

        // Initialise the transfer buffer
        while ((spiBufIndex < maxSpiBufStartIndex) && (pos < posLimit) && (cmpTimeUs(micros(), startTime) < maxEncodeTime)) {
            if (buffer[pos] != shadowBuffer[pos]) {
                if (buffer[pos] == 0xff) {
                    buffer[pos] = ' ';
                }

                if (setAddress || !autoInc) {
                    if (buffer[pos + 1] != shadowBuffer[pos + 1]) {
                        // It's worth auto incrementing
                        spiBuf[spiBufIndex++] = MAX7456_REG_W_DMM;
                        spiBuf[spiBufIndex++] = max7456._display_memory_mode | MAX7456_AUTO_INCR_MODE(1);
                        autoInc = true;
                    } else {
                        // It's not worth auto incrementing
                        spiBuf[spiBufIndex++] = MAX7456_REG_W_DMM;
                        spiBuf[spiBufIndex++] = max7456._display_memory_mode;
                        autoInc = false;
                    }

                    spiBuf[spiBufIndex++] = MAX7456_REG_W_DMAH;
                    spiBuf[spiBufIndex++] = pos >> 8;
                    spiBuf[spiBufIndex++] = MAX7456_REG_W_DMAL;
                    spiBuf[spiBufIndex++] = pos & 0xff;

                    setAddress = false;
                }

                spiBuf[spiBufIndex++] = MAX7456_REG_W_DMDI;
                spiBuf[spiBufIndex++] = buffer[pos];

                shadowBuffer[pos] = buffer[pos];
            } else {
                if (!setAddress) {
                    setAddress = true;
                    if (autoInc) {
                        spiBuf[spiBufIndex++] = MAX7456_REG_W_DMDI;
                        spiBuf[spiBufIndex++] = END_STRING;
                    }
                }
            }

            if (++pos >= maxScreenSize) {
                pos = 0;
                break;
            }
        }

        if (autoInc) {
            if (!setAddress) {
                spiBuf[spiBufIndex++] = MAX7456_REG_W_DMDI;
                spiBuf[spiBufIndex++] = END_STRING;
            }

            spiBuf[spiBufIndex++] = MAX7456_REG_W_DMM;
            spiBuf[spiBufIndex++] = max7456._display_memory_mode;
        }

        if (spiBufIndex) {
            pifSpiDevice_Transfer(max7456._p_spi, spiBuf, NULL, spiBufIndex);

            // Non-blocking, so transfer still in progress if using DMA
        }
    }

    return (pos != 0);
}

// should not be used when armed
void max7456RefreshAll(void)
{
    max7456ReInitIfRequired(true);
    while (max7456DrawScreen());
}

#ifdef MAX7456_NRST_PIN
static IO_t max7456ResetPin        = IO_NONE;
#endif

void max7456HardwareReset(void)
{
#ifdef MAX7456_NRST_PIN
#define IO_RESET_CFG      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)

    max7456ResetPin = IOGetByTag(IO_TAG(MAX7456_NRST_PIN));
    IOInit(max7456ResetPin, OWNER_OSD, 0);
    IOConfigGPIO(max7456ResetPin, IO_RESET_CFG);

    // RESET 50ms long pulse, followed by 100us pause
    IOLo(max7456ResetPin);
    delay(50);
    IOHi(max7456ResetPin);
    delayMicroseconds(100);
#else
    // Allow device 50ms to powerup
    delay(50);
#endif
}

void max7456SetBackgroundType(displayPortBackground_e backgroundType)
{
    deviceBackgroundType = backgroundType;

    max7456SetRegisterVM1();
}

#endif // USE_MAX7456
