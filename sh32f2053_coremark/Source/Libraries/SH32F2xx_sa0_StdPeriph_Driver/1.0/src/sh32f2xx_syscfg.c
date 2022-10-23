/**
  ******************************************************************************
  * @file    sh32f2xx_syscfg.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provides firmware functions to manage the following 
  *         
  *  @verbatim
  *
  *          ===================================================================
  *                                   How to use this driver
  *          ===================================================================
  *
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
#include "sh32f2xx_syscfg.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup SYSCFG_MODULE SYSCFG 
  * @brief SYSCFG driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup SYSCFG_Private_Functions
  * @{
  */ 

/** @defgroup SYSCFG_Group1 BOD Configuration and function
 *  @brief    BOD Configuration and function
 *
@verbatim    
 ===============================================================================
                      BOD Configuration and function
 ===============================================================================  
   
@endverbatim
  * @{
  */
  
/**
  * @brief  Initialize BOD mode and voltage threshold.
  * @param  SYSCFG_BOD_Mode: Specifie the BOD mode.
  *          This parameter can be a value of @ref SYSCFGBODMode_Type.
  * @param  SYSCFG_BOD_Level: Specifie the BOD voltage threshold.
  *          This parameter can be a value of @ref SYSCFGBODLevel_Type.
  * @retval None
  */ 
void SYSCFG_BODInit(SYSCFGBODMode_Type SYSCFG_BOD_Mode, SYSCFGBODLevel_Type SYSCFG_BOD_Level)
{
    uint32_t temp = 0;

    /* Check the parameters */
    assert_param(IS_SYSCFG_BOD_MODE(SYSCFG_BOD_Mode));
    assert_param(IS_SYSCFG_BOD_LEVEL(SYSCFG_BOD_Level));

    SYSCFG->PWRCR.V32 &= ~(SYSCFG_PWRCR_BODMD_Msk | SYSCFG_PWRCR_VBOD_Msk);
    
    temp = ((uint32_t)SYSCFG_BOD_Mode << 4) | (uint32_t)SYSCFG_BOD_Level;    
    SYSCFG->PWRCR.V32 |= temp;
}    

/**
  * @brief  Enable or disable the specified BOD interrupt.
  * @param  NewState: New state of BOD interrupt.
  *          This parameter can be one of the following values:
  *          @arg ENABLE  
  *          @arg DISABLE 
  * @retval None
  */
void SYSCFG_BODINTConfig(FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_FUNCTION_STATE(NewState));
    
    if (NewState != DISABLE)
    {
        SYSCFG->PWRCR.V32 |= SYSCFG_PWRCR_BODIE_Msk;
    }
    else
    {
        SYSCFG->PWRCR.V32 &= ~SYSCFG_PWRCR_BODIE_Msk;
    }
}

/**
  * @brief  Open or close BOD module.
  * @param  OnOffState: state of the BOD
  *          This parameter can one of the following values:
  *            @arg ON   
  *            @arg OFF  
  * @retval None
  */
void SYSCFG_BODOnOff(CmdState OnOffState)
{   
    /* Check the parameters */
    assert_param(IS_CMD_STATE(OnOffState));
    
    SYSCFG_PWRCR_BODEN_BIT = (uint32_t)OnOffState;
}


/**
  * @brief  Check whether the specified BOD flag is set or not.
  * @param  BOD_Flag: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg SYSCFG_BOD_FLAG_BODIF: BOD interrupt pending flag.
  *            @arg SYSCFG_BOD_FLAG_BODF: BOD output flag.
  * @retval New state of BOD Flag(SET or RESET).
  */
FlagStatus SYSCFG_BODGetFlagStatus(uint8_t BOD_Flag)
{
    FlagStatus bitStatus;

    /* Check the parameters */
    assert_param(IS_SYSCFG_BOD_FLAG(BOD_Flag));
    
    if ((SYSCFG->PWRSR.V32 & BOD_Flag) != RESET)
    {
        bitStatus = SET;
    }
    else
    {
        bitStatus = RESET;
    }
    
    return bitStatus;
}

/**
  * @brief  Clear BOD pending bit.
  * @param  None
  * @retval None
  */
void SYSCFG_BODClearFlag(void)
{
    /* Clear BODIF bit */
    SYSCFG_PWRSR_BODIF_BIT = 0;  
}


  
/**
  * @}
  */


/** @defgroup SYSCFG_Group2 NMI trigger source configuration
 *  @brief    NMI trigger source configuration
 *
@verbatim    
 ===============================================================================
                      NMI trigger source configuration
 ===============================================================================  
   
@endverbatim
  * @{
  */

/**
  * @brief  Enters SLEEP mode.
  * @param  SYSCFG_SLEEPEntry: specifies if SLEEP mode in entered with WFI or WFE instruction.
  *          This parameter can be one of the following values:
  *            @arg SYSCFG_SLEEPEntry_WFI: enter STOP mode with WFI instruction
  *            @arg SYSCFG_SLEEPEntry_WFE: enter STOP mode with WFE instruction
  * @retval None
  */  
void SYSCFG_EnterSLEEPMode(uint8_t SYSCFG_SLEEPEntry)
{
    /* Check the parameters */
    assert_param(IS_SYSCFG_SLEEP_ENTRY(SYSCFG_SLEEPEntry));
    
    /* Select SLEEP mode entry */
    if(SYSCFG_SLEEPEntry == SYSCFG_SLEEPEntry_WFI)
    {   
        /* Request Wait For Interrupt */
        __WFI();
    }
    else
    {
        /* Request Wait For Event */
        __WFE();
    }
}

/**
  * @brief  Enters STOP mode.
  *   
  * @note   In Stop mode, all I/O pins keep the same state as in Run mode.
  * @note   When exiting Stop mode by issuing an interrupt or a wakeup event, 
  *         the HSI RC oscillator is selected as system clock.
  * @param  SYSCFG_STOPEntry: specifies if STOP mode in entered with WFI or WFE instruction.
  *          This parameter can be one of the following values:
  *            @arg SYSCFG_STOPEntry_WFI: enter STOP mode with WFI instruction
  *            @arg SYSCFG_STOPEntry_WFE: enter STOP mode with WFE instruction
  * @retval None
  */
void SYSCFG_EnterSTOPMode(uint8_t SYSCFG_STOPEntry)
{
    /* Check the parameters */
    assert_param(IS_SYSCFG_SLOP_ENTRY(SYSCFG_STOPEntry));
    
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Select STOP mode entry */
    if(SYSCFG_STOPEntry == SYSCFG_STOPEntry_WFI)
    {   
        /* Request Wait For Interrupt */
        __WFI();
    }
    else
    {
        /* Request Wait For Event */
        __WFE();
    }
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);  
}


/**
  * @brief  Configure NMI interrupt trigger source.
  * @param  TriggerSource: Specifie NMI interrupt trigger source.
  *          This parameter can be any combination of the following values:
  *            @arg SYSCFG_NMI_TRIGGER_CSM: CSM trigger NMI interrupt.
  *            @arg SYSCFG_NMI_TRIGGER_BOD: BOD trigger NMI interrupt.
  *            @arg SYSCFG_NMI_TRIGGER_EXTI0: EXTI0 trigger NMI interrupt.       
  * @retval None
  */
void SYSCFG_NMIInterruptTriggerConfig(uint16_t TriggerSource, FunctionalState NewState)
{
    uint32_t temp = 0;
    
    /* Check the parameters */
    assert_param(IS_SYSCFG_NMI_TRIGGER_SOURCE(TriggerSource));
    assert_param(IS_FUNCTION_STATE(NewState));

    temp = SYSCFG->SAFR.V32 & (uint32_t)0x0000FFFF;

    if (NewState != DISABLE)
    {
        temp |= ((uint32_t)0x5AA5 << 16) | TriggerSource;
        SYSCFG->SAFR.V32 = temp;
    }
    else
    {
        temp &= ~TriggerSource;
        temp |= ((uint32_t)0x5AA5 << 16);
        SYSCFG->SAFR.V32 = temp;
    }        
}



/**
  * @}
  */


/** @defgroup SYSCFG_Group3 Special Pin configuration
 *  @brief    Special Pin configuration
 *
@verbatim    
 ===============================================================================
                      Special Pin configuration
 ===============================================================================  
   
@endverbatim
  * @{
  */



/**
  * @brief  Specifie SWJ pin alternate function.
  * @param  SWJPin_Mode: specifie the value for SWJ pin mode.
  *          This parameter can be one of the following values:
  *            @arg SYSCFG_SWJ_All_Function
  *            @arg SYSCFG_SWJ_SWJ_NoJRST  
  *            @arg SYSCFG_SWD_All_Function
  *            @arg SYSCFG_SWD_NO_SWO 
  *            @arg SYSCFG_SWD_PinRemap
  * @retval None
  */
void SYSCFG_SWJPinConfig(uint8_t SWJPin_Mode)
{
    uint32_t temp = 0;

    /* Check the parameters */
    assert_param(IS_SYSCFG_SWJ_PIN_MODE(SWJPin_Mode));

    temp = SYSCFG->SAFR.V32 & (~SYSCFG_SAFR_SWJCFG_Msk);
    
    temp |= (uint32_t)SWJPin_Mode;   
    SYSCFG->SAFR.V32 = ((uint32_t)0x5AA5 << 16) | temp;
}

/**
  * @brief  Specifie XTAL pin alternate
  * @param  OSCPin_Mode: specifie the value for XTAL pin mode.
  *          This parameter can be one of the @ref SYSCFG_OSCPin_Type enum values:
  *            @arg SYSCFG_OSC_GPIO
  *            @arg SYSCFG_OSC_Crystal
  *            @arg SYSCFG_OSC_ExtClock
  * @retval None
  */
void SYSCFG_OSCPinConfig(SYSCFG_OSCPin_Type OSCPin_Mode)
{
    uint32_t temp = 0;

    /* Check the parameter */
    assert_param(IS_SYSCFG_OSC_PIN_MODE(OSCPin_Mode));

    temp = SYSCFG->SAFR.V32 & (~SYSCFG_SAFR_OSCCFG_Msk);
    
    temp |= (uint32_t)OSCPin_Mode;   
    SYSCFG->SAFR.V32 = ((uint32_t)0x5AA5 << 16) | temp;  
}


/**
  * @}
  */


/** @defgroup SYSCFG_Group4 CRAM Lock configuration
 *  @brief    CRAM Lock configuration
 *
@verbatim    
 ===============================================================================
                      CRAM sector lock configuration
 ===============================================================================  
   
@endverbatim
  * @{
  */


/**
  * @brief  Lock or unlock CRAM sectors.
  * @param  SectorNum: specifie CRAM sectors to be locked or unlocked.
  *          This parameter can be any combination of the following values:
  *            @arg CRAM_Sector_0
  *            @arg CRAM_Sector_1
  *            @arg CRAM_Sector_2
  *            @arg CRAM_Sector_3
  *            @arg CRAM_Sector_4
  *            @arg CRAM_Sector_5
  *            @arg CRAM_Sector_6
  *            @arg CRAM_Sector_7
  * @param  NewState: new state of selected CRAM sectors.
  *          This parameter can be one of the following values:
  *            @arg ENABLE  to lock the selected sectors
  *            @arg DISABLE to unlock the selected sectors
  * @retval None
  */
void SYSCFG_CRAMLockConfig(uint8_t SectorNum, FunctionalState NewState)
{
    uint32_t temp = 0;

    /* Check the parameters */
    assert_param(IS_SYSCFG_CRAM_SECTOR(SectorNum));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        SYSCFG->CRAMLOCK.V32 |= ((uint32_t)0x5AA5 << 16) | SectorNum;
    }
    else
    {
        temp = SYSCFG->CRAMLOCK.V32 & (~SectorNum);
        SYSCFG->CRAMLOCK.V32 = ((uint32_t)0x5AA5 << 16) | temp;
    }
}

/**
  * @}
  */


/** @defgroup SYSCFG_Group5 Debug mode stop peripherals
 *  @brief    Debug mode stop peripherals
 *
@verbatim    
 ===============================================================================
                      Debug mode peripherals clock configuration
 ===============================================================================  
   
@endverbatim
  * @{
  */


/**
  * @brief  Enable or disable peripherals clock in debug mode.
  * @param  DBG_Periph: specifie peripherals to be enable or disable.
  *          This parameter can be any combination of the following values:
  *            @arg DBG_Periph_DMA
  *            @arg DBG_Periph_IWDT
  *            @arg DBG_Periph_WWDT
  *            @arg DBG_Periph_GPT
  *            @arg DBG_Periph_TIM
  *            @arg DBG_Periph_MCM
  *            @arg DBG_Periph_UART
  *            @arg DBG_Periph_SPI
  *            @arg DBG_Periph_TWI 
  * @param  NewState: new state of the selected peripherals.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void SYSCFG_DBGPeriphConfig(uint16_t DBG_Periph, FunctionalState NewState)
{
    uint32_t temp = 0;
    
    /* Check the parameters */
    assert_param(IS_DBG_PERIPH(DBG_Periph));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        SYSCFG->DBGCR.V32 |= ((uint32_t)0x5AA5 << 16) | DBG_Periph;
    }
    else
    {
        temp = SYSCFG->DBGCR.V32 & (~DBG_Periph);   
        SYSCFG->DBGCR.V32 = ((uint32_t)0x5AA5 << 16)| temp;
    }         
}

/**
  * @brief  Enable or disable low power clock in debug mode.
  * @param  LowPowerMode: specifie low power mode clock to be enable or disable.
  *          This parameter can be any combination of the following values:
  *           @arg DBG_LOWPOWER_SLEEP
  *           @arg DBG_LOWPOWER_STOP
  * @param  NewState: new state of the low power mode clock.
  *          This parameter can be one of the following values:
  *           @arg ENABLE
  *           @arg DISABLE
  * @retval None
  */
void SYSCFG_DBGLowPowerConfig(uint16_t LowPowerMode, FunctionalState NewState)
{
    uint32_t temp = 0x5AA50000;
    
    /* Check the parameters */
    assert_param(IS_DBG_LOWPOWER_MODE(LowPowerMode));
    assert_param(IS_FUNCTION_STATE(NewState));
    
    if (NewState != DISABLE)
    {
        SYSCFG->DBGCR.V32 |= ((uint32_t)0x5AA5 << 16) | (uint32_t)LowPowerMode;
    }
    else
    {
        temp = SYSCFG->DBGCR.V32 & (~LowPowerMode);   
        SYSCFG->DBGCR.V32 = ((uint32_t)0x5AA5 << 16) | temp;
    }
}

/**
  * @}
  */

  



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


