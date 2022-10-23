/**
  ******************************************************************************
  * @file    system_sh32f205.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provide system clock config
  *         
  *  @verbatim
  *
  *          ======================================================
  *                                   How to use this driver
  *          ======================================================
  * 1. Modify the file sh32f205_config.h, which in the project folder.
  *     -Modify the System Clock setting as the requirement
  * 2. This function will complete the RCC register's option by the configuration
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * SINOWEALTH IS SUPPLYING THIS SOFTWARE FOR USE EXCLUSIVELY ON SINOWEALTH'S 
  * MICROCONTROLLER PRODUCTS. IT IS PROTECTED UNDER APPLICABLE COPYRIGHT LAWS. 
  * THIS SOFTWARE IS FOR GUIDANCE IN ORDER TO SAVE TIME. AS A RESULT, SINOWEALTH 
  * SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES 
  * WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH FIRMWARE AND/OR
  * THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN 
  * CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2017 Sinowealth</center></h2>
  *
  ******************************************************************************
  */
  

  
/* Includes ------------------------------------------------------------------*/
#include "sh32f205.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup System_Group  System Configuration 
  * @brief System Clock Configuration Function
  * @{
  */ 

/** @defgroup System_Group_Funcs  Private Functions
  * @{
  */ 

#ifdef DEFAULT_SYSTEMINIT    

/**
* @verbatim Feature Description
 1. Complete the Flash accelerator's options:
    Enable or disable Prefetch,I-Cache,D-Cache and do Flash Latency setting.
    Flash latency need setted by system clock frequency. 
    Notice: Must set latency first then switch to higher frequency.
 2. Do system clock setting:
    Select clock source as HSI or HSE.
    Config PLL: Select PLL In Source and prescale option
    PLL Output = PLL In * PLL MUL /PLL DIV / 2
    PLL In must in 4~8MHz
    PLL In * PLL MUL must in 150MHz~300MHz
* @endverbatim
*/
/** 
   *@brief    System Init function, called by startup file
  * @retval   None
  */
void SystemInit(void)
{
    /* 1. UnlockRCC */
    RCC_WriteEnable();
    
    /* 2. Do Flash accelerator setting */
    FLASH->ACR.V32 = (0x5aa5<<16)
                 | (FLASH_LATENCY << FLASH_ACR_LATENCY_Pos) 
                 | (DCACHE_EN     << FLASH_ACR_DCEN_Pos)
                 | (ICACHE_EN     << FLASH_ACR_ICEN_Pos)
                 | (PREFETCH_EN   << FLASH_ACR_PRFTEN_Pos);
     __DSB();

#if (PLL_IN==1 && PLL_EN == 1) || (CLK_SRC==1)
    /* 3. Enable HSE */
    RCC->AHBENR.BIT.SYSCFGEN = 1;
    SYSCFG->SAFR.V32 = (0X5AA5<<16)|(1<<0);
    RCC->CR.BIT.HSEON = 1;
    while(RCC->CR.BIT.HSERDY == 0){CLR_WDT();}    
#endif    

    /* 4. Clock setting */
    RCC->CFGR.V32 = (((PLL_MUL-15)<< RCC_CFGR_PLLF_Pos)    
                    |((PLL_DIV-1) << RCC_CFGR_PLLK_Pos)  
                    |((PLL_IN)    << RCC_CFGR_PLLSRC_Pos)
                    |((PLL_XTPRE) << RCC_CFGR_PLLXTPRE_Pos)
                    |((CLK_APB2)  << RCC_CFGR_PPRE2_Pos) 
                    |((CLK_APB1)  << RCC_CFGR_PPRE1_Pos) 
                    |((CLK_AHB)   << RCC_CFGR_HPRE_Pos) 
                    );  
    
#if PLL_EN
    /*5. PLL On*/
    RCC_CR_PLLON_BIT = 1;    
    __DSB();
    while(RCC_CR_PLLRDY_BIT == 0){CLR_WDT();}
#endif

#if (CLK_SRC == 2)  
    /*6. SWITCH TO PLL*/
    RCC->CR.BIT.SW = 2;
    while(RCC->CR.BIT.SWS != 2){CLR_WDT();}    
#elif (CLK_SRC == 1)
    /* SWTICH TO HSE*/
    RCC->CR.BIT.SW = 1;
    while(RCC->CR.BIT.SWS != 1){CLR_WDT();}    
#else    
    /* SWTICH TO HSI*/
    RCC->CR.BIT.SW = 0;
    while(RCC->CR.BIT.SWS != 0){CLR_WDT();}    
#endif

    /*7. Lock RCC Module*/
    RCC_WriteDisable();
    
}

#endif

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/


