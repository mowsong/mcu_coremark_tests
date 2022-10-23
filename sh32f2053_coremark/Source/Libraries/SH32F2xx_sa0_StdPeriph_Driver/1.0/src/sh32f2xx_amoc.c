/**
  ******************************************************************************
  * @file    sh32f2xx_amoc.c
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
#include "sh32f2xx_amoc.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup AMOC_MODULE AMOC 
  * @brief AMOC driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup AMOC_Private_Functions
  * @{
  */ 

/** @defgroup AMOC_Group1 COMP Configuration and interrupt flags
 *  @brief    Comparetor configuration and interrupt flags
 *
@verbatim    
 ===============================================================================
                      Comparetor configuration and interrupt flags
 ===============================================================================  
   
@endverbatim
  * @{
  */


/**
  * @brief  Reset the COMPx module registers to their default reset values.
  * @param  COMPx: Where x can be 1,2 or 3 to select the COMP module.
  * @retval None
  */
void COMP_Reset(uint8_t COMPx)
{
    /* Check the parameters */
    assert_param(IS_COMP_ALL_PERIPH(COMPx));

    if (COMPx == COMP1)
    {
        AMOC->CMP1CON.V32 = (uint32_t)0x00;
        AMOC->CMPINTF.V32 = (uint32_t)0x10000;
    }
    else if (COMPx == COMP2)
    {
        AMOC->CMP2CON.V32 = (uint32_t)0x00;
        AMOC->CMPINTF.V32 = (uint32_t)0x20000;
    }
    else
    {
        if (COMPx == COMP3)
        {
            AMOC->CMP3CON.V32 = (uint32_t)0x00;
            AMOC->CMPINTF.V32 = (uint32_t)0x40000;
        }
    }
}

/**
  * @brief  Fills each COMP_InitStruct member with its default value.
  * @param  COMP_InitStruct : pointer to a @ref COMP_InitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
void COMP_StructInit(COMP_InitTypeDef* COMP_InitStruct)
{
    /* Initialize the COMP_NRefVoltageSource member */
    COMP_InitStruct->COMP_NRefVoltageSource = COMP_NRefVoltage_AVDD;

    /* Initialize the COMP_NRefVoltageLevel member */
    COMP_InitStruct->COMP_NRefVoltageLevel = COMP_NRefVoltageLevel_VRef;

    /* Initialize the COMP_OutputCmd member */
    COMP_InitStruct->COMP_OutputCmd = COMP_Output_Disable;

    /* Initialize the COMP_SchmittVoltage member */
    COMP_InitStruct->COMP_SchmittVoltage = COMP_SchmittVoltage_None;

    /* Initialize the COMP_NSelect member */
    COMP_InitStruct->COMP_NSelect = COMP_NSelect_NPin;

    /* Initialize the COMP_PSelect member */
    COMP_InitStruct->COMP_PSelect = COMP_PSelect_P0;

    /* Initialize the COMP_Filter member */
    COMP_InitStruct->COMP_Filter = COMP_Filter_None;
}

/**
  * @brief  Initialize the COMP module according to the specified parameters
  *         in COMP_InitStruct
  * @param  COMPx: where x can be 1 to 3 to select the COMP module.
  * @param  COMP_InitStruct: pointer to an COMP_InitTypeDef structure that contains 
  *         the configuration information for the specified COMP module.
  * @retval None
  */
void COMP_Init(uint8_t COMPx, COMP_InitTypeDef* COMP_InitStruct)
{
    uint32_t temp = 0;
    uint32_t tempAddress = 0;
    uint32_t tempP_Select = 0;
    
    /* Check the parameters */
    assert_param(IS_COMP_ALL_PERIPH(COMPx));
    assert_param(IS_COMP_N_REF_VOLTAGE_SOURCE(COMP_InitStruct->COMP_NRefVoltageSource));
    assert_param(IS_COMP_N_REF_VOLTAGE_LEVEL(COMP_InitStruct->COMP_NRefVoltageLevel));
    assert_param(IS_COMP_OUTPUT_CMD(COMP_InitStruct->COMP_OutputCmd));
    assert_param(IS_COMP_SCHMITT_VOLTAGE(COMP_InitStruct->COMP_SchmittVoltage));
    assert_param(IS_COMP_N_SELECT(COMP_InitStruct->COMP_NSelect));
    assert_param(IS_COMP_FILTER(COMP_InitStruct->COMP_Filter));

    tempAddress = AMOC_BASE + COMPx;

    temp = *(__IO uint32_t *)tempAddress;

    /* Clear all bits except bit7, bit10, bit12~15 */
    temp &= 0xFFF0F480;

    tempP_Select = (uint32_t)COMP_InitStruct->COMP_PSelect;
    
    if (COMPx == COMP3)
    {
        assert_param(IS_COMP_3_P_SELECT(COMP_InitStruct->COMP_PSelect));
        
        if (COMP_InitStruct->COMP_PSelect >= 3)
        {
            tempP_Select -= 1;
        }
    }
    else if (COMPx == COMP2)
    {
        assert_param(IS_COMP_2_P_SELECT(COMP_InitStruct->COMP_PSelect));
    }
    else
    {
        assert_param(IS_COMP_1_P_SELECT(COMP_InitStruct->COMP_PSelect));
    }
    
    temp |= tempP_Select << 3;
    temp |= COMP_InitStruct->COMP_Filter | COMP_InitStruct->COMP_NSelect \
            | COMP_InitStruct->COMP_SchmittVoltage | COMP_InitStruct->COMP_OutputCmd \
            | COMP_InitStruct->COMP_NRefVoltageLevel | COMP_InitStruct->COMP_NRefVoltageSource;
    
    *(__IO uint32_t *)tempAddress = temp;
}

/**
  * @brief  Open or close the COMP Module.
  * @param  COMPx: where x can be 1 to 3 to select the COMP module.
  * @param  OnOffState: state of the COMP module.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void COMP_OnOff(uint8_t COMPx, CmdState OnOffState)
{
    uint32_t tempAddress = 0;
    
    /* Check the parameters */
    assert_param(IS_COMP_ALL_PERIPH(COMPx));
    assert_param(IS_CMD_STATE(OnOffState));

    tempAddress = AMOC_BASE + COMPx;
    
    if (OnOffState != OFF)
    {
        *(__IO uint32_t *)tempAddress |= (uint32_t)0x80;
    }
    else
    {
        *(__IO uint32_t *)tempAddress &= (uint32_t)0xFFFFFF7F;
    }
}


/**
  * @brief  Return the output level (high or low) of the selected COMP module. 
  * @param  COMPx: where x can be 1 to 3 to select COMP module. 
  * @retval Return the selected COMP module output level: low or high.      
  */
uint8_t COMP_GetOutputLevel(uint8_t COMPx)
{
    uint8_t OutputLevel = 0;
    uint32_t temp = 0;
    uint32_t tempAddress = 0;

    /* Check the parameters */
    assert_param(IS_COMP_ALL_PERIPH(COMPx));

    tempAddress = AMOC_BASE + COMPx;
    temp = *(__IO uint32_t *)tempAddress;
    
    if ((temp & 0x800) != (uint32_t)RESET)
    {
        OutputLevel = SET;
    }
    else
    {
        OutputLevel = RESET;
    }

    return OutputLevel;
}

/**
  * @brief  Enable or disable the COMPx's DMA request.
  * @param  COMPx: where x can be 1 to 3 to select the COMP modules. 
  * @param  NewState: new state of the DMA request source.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void COMP_DMAConfig(uint8_t COMPx, FunctionalState NewState)
{
    uint32_t tempAddress = 0;
    
    /* Check the parameters */
    assert_param(IS_COMP_ALL_PERIPH(COMPx));
    assert_param(IS_FUNCTION_STATE(NewState));

    tempAddress = AMOC_BASE + COMPx;

    if (NewState != DISABLE)
    {
        /* Enable the DMA trigger */
        *(__IO uint32_t *)tempAddress |= (uint32_t)0x8000;
    }
    else
    {
        /* Disable the DMA trigger */
        *(__IO uint32_t *)tempAddress &= (uint32_t)0xFFFF7FFF;
    }
}

/**
  * @brief  Enable or disable the specified COMP interrupts.
  * @param  COMPx: where x can be 1 to 3 to select the COMP module.
  * @param  COMP_INT: specifie the COMP interrupt source to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg COMP_INT_RISING
  *            @arg COMP_INT_FALLING
  * @param  NewState: new state of the COMP interrupts.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void COMP_INTConfig(uint8_t COMPx, uint16_t COMP_INT, FunctionalState NewState)
{
    uint32_t tempAddress = 0;
    
    /* Check the parameters */
    assert_param(IS_COMP_ALL_PERIPH(COMPx));
    assert_param(IS_COMP_INT(COMP_INT));
    assert_param(IS_FUNCTION_STATE(NewState));

    tempAddress = AMOC_BASE + COMPx;

    if (NewState != DISABLE)
    {
        *(__IO uint32_t *)tempAddress |= (uint32_t)COMP_INT;
    }
    else
    {
        *(__IO uint32_t *)tempAddress &= ~((uint32_t)COMP_INT);
    }

}

/**
  * @brief  Check whether the specified flag is set or not.
  * @param  COMPx: where x can be 1 to 3 to select the COMP module.
  * @retval The new state of the Flag(SET or RESET).
  */
FlagStatus COMP_GetFlagStatus(uint8_t COMPx)
{
    uint8_t pos = 0;
    FlagStatus bitStatus;
    
    /* Check the parameters */
    assert_param(IS_COMP_ALL_PERIPH(COMPx));
    
    pos = COMPx >> 2;

    if ((AMOC->CMPINTF.V32 & (AMOC_CMPINTF_C1IF_Msk << pos)) != (uint32_t)RESET)
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
  * @brief  Clear the COMP's pending flag.
  * @param  COMPx: where x can be 1 to 3 to select the COMP module.
  * @retval None
  */
void COMP_ClearFlag(uint8_t COMPx)
{
    uint8_t pos = 0;
    
    /* Check the parameters */
    assert_param(IS_COMP_ALL_PERIPH(COMPx));

    pos = COMPx >> 2;

    AMOC->CMPINTF.V32 = (uint32_t)(AMOC_CMPINTF_C1IF_Msk << pos) << 16;
}

/**
  * @}
  */



/** @defgroup AMOC_Group2 OP configuration
 *  @brief    OP configuration
 *
@verbatim    
 ===============================================================================
                                OP configuration
 ===============================================================================  
   
@endverbatim
  * @{
  */
  

/**
  * @brief  On or off the OP Module.
  * @param  OPx: where x can be 1 to 3 to select the OP module.
  * @param  OnOffState: state of the OP module.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void OP_OnOff(uint8_t OPx, CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_OP_ALL_PERIPH(OPx));
    assert_param(IS_CMD_STATE(OnOffState));    

    if (OnOffState != OFF)
    {
        /* Enable the OP module */
        AMOC->OPCON.V32 |= OPx; 
    }
    else
    {
        /* Disable the OP module */
        AMOC->OPCON.V32 &= ~OPx;
    }
}

/**
  * @brief  Configure the internal reference voltage(Vref) value.
  * @param  AMOC_VrefVoltage: specifie the internal reference voltage value.
  *          This parameter can be one of the following values:
  *            @arg AMOC_VrefVoltage_2V5
  *            @arg AMOC_VrefVoltage_1V65
  * @retval None
  */
void AMOC_VrefVoltageConfig(uint8_t AMOC_VrefVoltage)
{
    uint32_t temp = 0;
    
    /* Check the parameters */
    assert_param(IS_AMOC_VREF_VOLTAGE(AMOC_VrefVoltage));

    temp = AMOC->AVREFCON.V32;
    temp &= ~AMOC_AVREFCON_VREFSEL_Msk;
    temp |= AMOC_VrefVoltage;
    AMOC->AVREFCON.V32 = temp;
}

/**
  * @brief  Open or disable the internal reference voltage(Vref).
  * @param  OnOffState: new state of internal reference voltage(Vref).
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void AMOC_VrefOnOff(CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_CMD_STATE(OnOffState));

    AMOC_AVREFCON_VREFEN_BIT = (uint32_t)OnOffState;
}


/**
  * @}
  */


/** @defgroup AMOC_Group3 Temperature Sensor Configuration
 *  @brief    Temperature Sensor Configuration
 *
@verbatim    
 ===============================================================================
                           Temperature Sensor Configuration
 ===============================================================================  
   
@endverbatim
  * @{
  */


/**
  * @brief  Enable or disable the Temperature Sensor CHOP switch.
  * @param  NewState: new state of the Temperature Sensor CHOP switch.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void AMOC_TempSensorCHOPConfig(FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_FUNCTION_STATE(NewState));

    AMOC_TPSCON_TPSCHOP_BIT = (uint32_t)NewState;
}

/**
  * @brief  Enable or disable the Temperature Sensor .
  * @param  OnOffState: state of the Temperature Sensor.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void AMOC_TempSensorOnOff(CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_CMD_STATE(OnOffState));

    AMOC_TPSCON_TPSEN_BIT = (uint32_t)OnOffState;
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


