#include "pif_linker.h"

#include "platform.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "core/pif_log.h"


PifImuSensor g_imu_sensor;
PifTimerManager g_timer_1ms;

serialPort_t *serialPort = NULL;

const PifLogCmdEntry c_psCmdTable[] = {
	{ "help", pifLog_CmdHelp, "This command", NULL },
	{ "version", pifLog_CmdPrintVersion, "Print version", NULL },
	{ "task", pifLog_CmdPrintTask, "Print task", NULL },
	{ "status", pifLog_CmdSetStatus, "Set and print status", NULL },

	{ NULL, NULL, NULL, NULL }
};


BOOL logInit()
{
    serialPort = uartOpen(SERIAL_PORT_IDENTIFIER_TO_UARTDEV(SERIAL_PORT_USART3), NULL, NULL, 115200, MODE_RXTX, SERIAL_PIF);
    if (!serialPort) return FALSE;

    if (!pifUart_AttachRxTask(&serialPort->uart, PIF_ID_LOG_RX_TASK, TM_PERIOD, 200000, "UART-RX-LOG")) return FALSE;	// 200ms
    if (!pifUart_AttachTxTask(&serialPort->uart, PIF_ID_LOG_TX_TASK, TM_EXTERNAL, 0, "UART-TX-LOG")) return FALSE;

    pifLog_Init();
    if (!pifLog_AttachUart(&serialPort->uart, 256)) return FALSE;														// 256bytes
    if (!pifLog_UseCommand(32, c_psCmdTable, "\nDebug> ")) return FALSE;												// 32bytes

    pifLog_Printf(LT_INFO, "Betaflight");
    return TRUE;
}
