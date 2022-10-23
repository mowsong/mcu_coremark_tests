#include "main.h"


extern int ee_printf(const char *fmt, ...) ;

volatile uint32_t TICK = 0;

void SysTick_Handler(void)
{
    TICK++;
}

uint32_t Get_Tick(void)
{
  return TICK;
}

void WAIT_ms(uint32_t ms)
{
    uint32_t now = TICK;
    while ((TICK-now) < ms);
}

void GPIO_EnableDigital(GPIO_Type *GPIO, uint32_t Pin, uint32_t func)
{            
  GPIO->FSR &= ~(3U<<(Pin<<1));
  GPIO->FSR |= (2U<<(Pin<<1));
  GPIO->DFS &= ~(3U<<(Pin<<1));
  GPIO->DFS |= ((func & 0x3U)<<(Pin<<1));
}

void RCC_Configuration(void)
{
  SYSCTRL->PERHRSTEN = 0xECA86420;
  SYSCTRL->PERRST1   = ~0UL;
  SYSCTRL->PERRST2   = ~0UL;
  SYSCTRL->PERRST1   = 0UL;
  SYSCTRL->PERRST2   = 0UL;
  SYSCTRL->PERHRSTEN = 0;
#if 1
  SystemCoreClockSelect(CoreClk_PLL160M);
  SystemCoreClock = 160000000UL;
#else
  SystemCoreClockSelect(CoreClk_HSRC8M);
  SystemCoreClock = 8000000UL;
#endif
}

void GPIO_Configuration(void)
{    
  SYSCTRL->AHBBCKCON |= (1U<<SYSCTRL_AHBBCKCON_IOCBCKE_Pos);
  GPIO_EnableDigital(UART_RX_PORT, UART_RX_PIN, 2);   // RXD  
  GPIO_EnableDigital(UART_TX_PORT, UART_TX_PIN, 2);   // TXD
}

void UART_Configuration(void)
{
  SYSCTRL->APBBCKCON |= (1U<<UARTBCKE_Pos);
  SYSCTRL->PERCKEN   |= (1U<<UARTCKEN_Pos);
  SYSCTRL->PERCKCFG  |= (1UL<<UARTCKSEL_Pos);   // APB clock
  
  UART->CR  = 0;
  UART->BRR = (SystemCoreClock + (BAUD_RATE>>1))/(BAUD_RATE) - 1;    
  UART->SR  = ~0U;
  UART->CR  = (1<<UART_CR_DL_Pos) | (1<<UART_CR_RE_Pos) | (1<<UART_CR_TE_Pos);
}


int platform_init(void)
{
  RCC_Configuration();
  GPIO_Configuration();
  UART_Configuration();
  SysTick_Config(SystemCoreClock/1000);

#if 0

  while (1){
    WAIT_ms(1000);
    ee_printf("Hello\n");
  }
#endif

}



    