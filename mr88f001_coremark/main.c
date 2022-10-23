#include "MR88F001.h"
#include <stdio.h> 

volatile uint32_t TICK=0;

#define BAUD_RATE       (115200UL)

#define UART            UART1
#define UART_RX_PORT    GPIOB
#define UART_TX_PORT    GPIOB
#define UART_RX_PIN     0
#define UART_TX_PIN     1
#define UARTBCKE_Pos    SYSCTRL_APBBCKCON_UART1BCKE_Pos
#define UARTCKEN_Pos    SYSCTRL_PERCKEN_UART1CKEN_Pos
#define UARTCKSEL_Pos   SYSCTRL_PERCKCFG_UART1CKSEL_Pos



//--------------------------------------------------------------------------------------------------
void RCC_Configuration(void);
void GPIO_Configuration(void);
void UART_Configuration(void);
    
void WAIT_ms(uint32_t ms);

void GPIO_EnableDigital(GPIO_Type *GPIO, uint32_t Pin);

//--------------------------------------------------------------------------------------------------
void platform_init(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    UART_Configuration();
    
    SysTick_Config(SystemCoreClock/1000);
}

//--------------------------------------------------------------------------------------------------
void RCC_Configuration(void)
{
    SYSCTRL->AHBBCKCON = 0x30UL;    // POR value
    SYSCTRL->AHBBCKCON |= (1U<<SYSCTRL_AHBBCKCON_IOCBCKE_Pos);
    SYSCTRL->APBBCKCON = 0UL;
    SYSCTRL->APBBCKCON |= (1U<<SYSCTRL_APBBCKCON_IWDTBCKE_Pos);
    
    SYSCTRL->PERHRSTEN = 0xECA86420;
    SYSCTRL->PERRST1   = ~0UL;
    SYSCTRL->PERRST2   = ~0UL;
    SYSCTRL->PERRST1   = 0UL;
    SYSCTRL->PERRST2   = 0UL;
    SYSCTRL->PERHRSTEN = 0;
#if defined (SYSTEM_CLOCK_16M)
    SystemCoreClockSelect(CoreClk_HSRC16M);
#elif defined (SYSTEM_CLOCK_24M)  
    SystemCoreClockSelect(CoreClk_HSRC24M);
#elif defined (SYSTEM_CLOCK_32M)
    SystemCoreClockSelect(CoreClk_HSRC32M);
#endif  
    SystemCoreClockUpdate();
}

//--------------------------------------------------------------------------------------------------
void GPIO_Configuration(void)
{    
    GPIO_EnableDigital(UART_RX_PORT, UART_RX_PIN);   // RXD  
    GPIO_EnableDigital(UART_TX_PORT, UART_TX_PIN);   // TXD
}

//--------------------------------------------------------------------------------------------------
void UART_Configuration(void)
{
    SYSCTRL->APBBCKCON |= (1U<<UARTBCKE_Pos);
    SYSCTRL->PERCKEN   |= (1U<<UARTCKEN_Pos);
    SYSCTRL->PERCKCFG  |= (1UL<<UARTCKSEL_Pos);   // APB clock
    
    UART->CR  = 0;
    UART->BRR = (SystemCoreClock + (BAUD_RATE>>1))/(BAUD_RATE) - 1;    
    UART->SR  = ~0U;
    UART->CR  = (1<<UART_CR_DBITS_Pos) | (1<<UART_CR_RXEN_Pos) | (1<<UART_CR_TXEN_Pos);
}

//--------------------------------------------------------------------------------------------------
void SysTick_Handler(void)
{
    TICK++;
}

//--------------------------------------------------------------------------------------------------
uint32_t Get_Tick(void)
{
  return TICK;
}

//--------------------------------------------------------------------------------------------------
void WAIT_ms(uint32_t ms)
{
    uint32_t now = TICK;
    while ((TICK-now) < ms);
}

//--------------------------------------------------------------------------------------------------
void GPIO_EnableDigital(GPIO_Type *GPIO, uint32_t Pin)
{            
    GPIO->FSR &= ~(3U<<(Pin<<1));
    GPIO->FSR |= (2U<<(Pin<<1));
    GPIO->DFS &= ~(1U<<Pin);
}

//--------------------------------------------------------------------------------------------------
int _PutChar(int c)
{
    while ((UART->SR & UART_SR_TXBE_Msk) == 0);
    UART->TDR = c;
    return c;
}

//--------------------------------------------------------------------------------------------------
int _GetChar(void)
{
  while (!(UART->SR & UART_SR_RXBF_Msk));
  return (UART->RDR & 0xFF);
}
    

