/**
  *******************************************************************************
  * @file    system_MR82Fx01.c
  * @author  ngms
  * @version 1.0.7
  * @date    2021-08-22
  * @brief   系统初始化
  *******************************************************************************
  * @attention
  * 
  * 
  *******************************************************************************
  */
  
#include <stdint.h>
#include "MR82Fx01.h"


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

#define XTAL    (8000000UL)            /* Oscillator frequency               */
#define PLL160  (160000000UL)          /* PLL frequency                      */


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemFrequency = XTAL;    /*!< System Clock Frequency (Core Clock)  */
uint32_t SystemCoreClock = XTAL;    /*!< Processor Clock Frequency            */


void SystemCoreClockSelect(eCoreClk cc)
{ 
  uint32_t d;
  uint32_t tmp;
    
    // set all div to 0
    SYSCTRL->SYSCLKDIV = 0x0;
    
    SYSCTRL->APBBCKCON |= (1UL<<SYSCTRL_APBBCKCON_ANACTLBCKE_Pos); 
    
    switch (cc) 
    {
        case CoreClk_HSRC8M:
        {
            // precautionary step, in case trim values differ to much
            FLSCTRL->ACR = 6;
					
			// enable
			SYSCTRL->HSRCCON_b.HSRCEN =1;
                    
            // switch to HSRC
            SYSCTRL->SYSCLKSEL = 0; 
                    
            // default to 0 wait, ACR does not need the FLS bus clock
            FLSCTRL->ACR = 0;    
                    
            SystemCoreClock = (8000000UL);
            break;
        }
                
        case CoreClk_XTAL:
        {
            tmp = FLSCTRL->ACR;
                    
            // precautionary step, in case trim values differ to much
            FLSCTRL->ACR = 6; 
            // enable clock stop detection
            ANACTL->FDETIE = (1<<ANACTL_FDETIE_DEN_Pos);
            // enable XTAL
            SYSCTRL->HXTALCON |= 1;
            
            // add delay for xtal to stabilise
            SYSCTRL->APBBCKCON |= SYSCTRL_APBBCKCON_IWDTBCKE_Msk;
            d = 2000;
            while (d--)
            {
                WDT->IWDTKR = 0x5A5A5A5A;
            }               
 
            if (ANACTL->FDETIF & ANACTL_FDETIF_FDETOUTB_Msk)
            {                
                // switch to HXTAL
                SYSCTRL->SYSCLKSEL = 0x1;   
                // default to 0 wait, ACR does not need the FLS bus clock 
                FLSCTRL->ACR = 0;  
            }
            else
            {
                FLSCTRL->ACR = tmp;
            }
            
            break;
        }
                
        case CoreClk_PLL160M:
        {
            // precautionary step, in case trim values differ to much
            FLSCTRL->ACR = 6; 
                    
            SYSCTRL->PLLCON_b.PLLEN     = 0; // disable PLL
  
            SYSCTRL->PLLDIV_b.CLKR       = 0;
            SYSCTRL->PLLDIV_b.CLKF       = 39;
            SYSCTRL->PLLDIV_b.CLKOD      = 1;
            SYSCTRL->PLLCON_b.BWADJ      = 4;
            SYSCTRL->PLLCON_b.PLLINSEL   = 0;
            SYSCTRL->PLLCON_b.PLLINSEL_A = 0;

            SYSCTRL->PLLCON_b.PLLEN      = 1; // enable PLL
  
            SYSCTRL->APBBCKCON |= SYSCTRL_APBBCKCON_IWDTBCKE_Msk;

            while (SYSCTRL->PLLCON_b.LOCKED == 0) {
                //__NOP();
                WDT->IWDTKR = 0x5A5A5A5A;
            }   
            
            // switch to PLL
            SYSCTRL->SYSCLKSEL = 0x3;   
            
            SystemCoreClock = PLL160;       
            
            break;
        }
                    
        
        case CoreClk_LSRC:
        {
            // switch to LSRC
            SYSCTRL->SYSCLKSEL_b.SYSCLKSEL = 2;  
            FLSCTRL->ACR = 0;          
            break;
        }
        
        default:
        {
            // keep the core clock setting
            break;
        }
    }
}



/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  SystemCoreClock = XTAL;
}


//**********************************************************
// end temporary solution
//**********************************************************


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
  uint32_t d;
    
  SystemCoreClockSelect(CoreClk_HSRC8M);
    
  #if (__FPU_USED == 1)
    SCB->CPACR |= 0x00F00000;
    __ISB();
  #endif
  
  #warning "Add delay to allow time for debug entry in case of SWD remap" 
  SYSCTRL->APBBCKCON |= SYSCTRL_APBBCKCON_IWDTBCKE_Msk;
  d = 200000;
  while (d--)
  {
    WDT->IWDTKR = 0x5A5A5A5A;       
  }
  
  SystemCoreClock = XTAL;
}
