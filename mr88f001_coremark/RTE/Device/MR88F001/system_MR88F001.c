/**
  *******************************************************************************
  * @file    system_MR88F001.c
  * @author  ngms
  * @version 2.0.3
  * @date    2021-11-23
  * @brief   系统初始化
  *******************************************************************************
  * @attention
  * 
  * 
  *******************************************************************************
  */

#include <stdint.h>
#include "MR88F001.h"


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/


uint32_t HXTALFrequency  = (8000000UL);
uint32_t SystemCoreClock = (8000000UL);

/**
 * System core clock selection
 *
 * @param  cc : Core clock selection 
 * @return none
 *
 * @brief  Setup the core clock and update the SystemCoreClock variable.
 * 
 * @note The function call should be followed by a call to SystemCoreClockUpdate().
 */
void SystemCoreClockSelect(eCoreClk cc)
{   
    uint32_t d;
    uint32_t tmp;
    
    SYSCTRL->APBBCKCON |= (1UL<<SYSCTRL_APBBCKCON_ANACTLBCKE_Pos); 
    
    switch (cc) 
    {
        case CoreClk_HSRC8M:
        {
            FLSCTRL->ACR = 1;   
            // select 8M
            SYSCTRL->HSRCCON  = (0<<SYSCTRL_HSRCCON_FSEL_Pos) | 0x1UL;
            // switch to HSRC
            SYSCTRL->SYSCLKSEL = 0; 
            // default to 0 wait, ACR does not need the FLS bus clock
            FLSCTRL->ACR = 0;        
            break;
        }
        
        case CoreClk_HSRC16M:
        {
            FLSCTRL->ACR = 1; 
            // select 16M
            SYSCTRL->HSRCCON  = (1<<SYSCTRL_HSRCCON_FSEL_Pos) | 0x1UL;
            // switch to HSRC
            SYSCTRL->SYSCLKSEL = 0;
            // default to 0 wait, ACR does not need the FLS bus clock             
            FLSCTRL->ACR = 0;
            break;
        }
        
        case CoreClk_HSRC24M:
        {
            FLSCTRL->ACR = 1;   
            // select 24M
            SYSCTRL->HSRCCON  = (2<<SYSCTRL_HSRCCON_FSEL_Pos) | 0x1UL;
            // switch to HSRC
            SYSCTRL->SYSCLKSEL = 0; 
            // default to 0 wait, ACR does not need the FLS bus clock
            FLSCTRL->ACR = 0;      
            break;
        }
        
        case CoreClk_HSRC32M:
        {
            FLSCTRL->ACR = 1;   
            // select 32M
            SYSCTRL->HSRCCON  = (3<<SYSCTRL_HSRCCON_FSEL_Pos) | 0x1UL;
            // switch to HSRC
            SYSCTRL->SYSCLKSEL = 0; 
            // default to 1 wait, no need to change
            break;
        }
        
        case CoreClk_LSRC:
        {
            // switch to LSRC
            SYSCTRL->SYSCLKSEL = 0x2;   
            FLSCTRL->ACR = 0;          
            break;
        }
        
        case CoreClk_XTLE24M:
        {
            tmp = FLSCTRL->ACR;
            FLSCTRL->ACR = 1; 
            // enable clock stop detection
            ANACTL->FDETIE = (1<<ANACTL_FDETIE_DEN_Pos);
            // enable XTAL
            SYSCTRL->HXTALCON |= 1;
            
            // add delay for xtal to stabilise
            SYSCTRL->APBBCKCON |= SYSCTRL_APBBCKCON_IWDTBCKE_Msk;
            d = 200;
            while (d--)
            {
                WDT->IWDTKR = 0x5A5A5A5A;
            }
            
            if (ANACTL->FDETIF & ANACTL_FDETIF_FDETOUTB_Msk)
            {
                // switch to HXTAL
                SYSCTRL->SYSCLKSEL = 0x1;   
                FLSCTRL->ACR = 0;  
            }
            else
            {
                FLSCTRL->ACR = tmp;
            }
            
            break;
        }
        
        case CoreClk_XTGT24M:
        {
            tmp = FLSCTRL->ACR;
            FLSCTRL->ACR = 1; 
            // enable clock stop detection
            ANACTL->FDETIE = (1<<ANACTL_FDETIE_DEN_Pos);
            // enable XTAL
            SYSCTRL->HXTALCON |= 1;
            
            // add delay for xtal to stabilise
            SYSCTRL->APBBCKCON |= SYSCTRL_APBBCKCON_IWDTBCKE_Msk;
            d = 200;
            while (d--)
            {
                WDT->IWDTKR = 0x5A5A5A5A;
            }
            
            if (ANACTL->FDETIF & ANACTL_FDETIF_FDETOUTB_Msk)
            {
                // switch to HXTAL
                SYSCTRL->SYSCLKSEL = 0x1;   
                FLSCTRL->ACR = 1;  
            }
            else
            {
                FLSCTRL->ACR = tmp;
            }
            
            break;
        }
        
        default:
        {
            // keep the core clock setting
            break;
        }
    }
}

/**
 * Update the HXTAL frequency
 *
 * @param  hxtal_freq : user specified xtal frequency
 * @return none
 *
 * @brief  Update the core clock frequency, based on SYSCLKDIV.
 */
void SystemUpdateHXTALFrequency(uint32_t hxtal_freq)
{
    HXTALFrequency = hxtal_freq;
}

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
void SystemCoreClockUpdate(void)          
{
    uint32_t FSELTab[4] = {8000000UL, 16000000UL, 24000000UL, 32000000UL};
    
    uint32_t AHBPresTab[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    
    uint32_t clksel;
    uint32_t fsel;
    uint32_t ahbpres;

    // determine clock source
    clksel = (SYSCTRL->SYSCLKSEL & SYSCTRL_SYSCLKSEL_SYSCLKSEL_Msk)>>SYSCTRL_SYSCLKSEL_SYSCLKSEL_Pos;
    if (0x0 == clksel)
    {
        fsel = (SYSCTRL->HSRCCON & SYSCTRL_HSRCCON_FSEL_Msk)>>SYSCTRL_HSRCCON_FSEL_Pos;
        SystemCoreClock = FSELTab[fsel];
    }
    else if (0x1 == clksel)
    {
        SystemCoreClock = HXTALFrequency;
    }
    else if (0x2 == clksel)
    {
        SystemCoreClock = 32000UL;  // LSRC
    }
    
    ahbpres = (SYSCTRL->SYSCLKDIV & SYSCTRL_SYSCLKDIV_AHBPRES_Msk)>>SYSCTRL_SYSCLKDIV_AHBPRES_Pos;

    SystemCoreClock = SystemCoreClock>>AHBPresTab[ahbpres];         
}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit(void)
{
    uint32_t d;
    
    SystemCoreClockSelect(CoreClk_HSRC8M);
    SystemCoreClockUpdate();

    #warning "Add delay to allow time for debug entry in case of SWD remap"
    SYSCTRL->APBBCKCON |= SYSCTRL_APBBCKCON_IWDTBCKE_Msk;
    d = 200000;
    while (d--)
    {
        WDT->IWDTKR = 0x5A5A5A5A;
    }
}
