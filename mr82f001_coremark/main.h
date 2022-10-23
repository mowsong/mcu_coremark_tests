#ifndef __MAIN_H__

#include "MR82Fx01.h"
#include "stdint.h"

#define BAUD_RATE       (115200UL)

#define UART            UART0
#define UART_RX_PORT    GPIOB
#define UART_TX_PORT    GPIOB
#define UART_RX_PIN     10
#define UART_TX_PIN     11
#define UARTBCKE_Pos    SYSCTRL_APBBCKCON_UART0BCKE_Pos
#define UARTCKEN_Pos    SYSCTRL_PERCKEN_UART0CKEN_Pos
#define UARTCKSEL_Pos   SYSCTRL_PERCKCFG_UART0CKSEL_Pos

#endif
