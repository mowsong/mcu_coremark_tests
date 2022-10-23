/**
  ******************************************************************************
  * @file    sh32f2xx_mpu.c
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
#include "sh32f2xx_mpu.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup MPU_MODULE MPU 
  * @brief MPU driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup MPU_Private_Functions
  * @{
  */ 


/**
  * @brief  Reset the MPU peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
void MPU_Reset(void)
{
    MPU->CTRL = 0x00;
    MPU->RNR = 0x00;
    MPU->RASR = 0x00;
    MPU->RBAR = 0x00;
    MPU->RBAR_A1 = 0x00;
    MPU->RBAR_A2 = 0x00;
    MPU->RBAR_A3 = 0x00;
    MPU->RASR_A1 = 0x00;
    MPU->RASR_A2 = 0x00;
    MPU->RASR_A3 = 0x00;
}


/**
  * @brief  Fill each MPU_InitStruct member with its default value.
  * @param  MPU_InitStruct: pointer to a @ref MPU_InitTypeDef structure which will be initialized.
  * @retval None
  */
void MPU_StructInit(MPU_InitTypeDef* MPU_InitStruct)
{
    /* Initialize the MPU_Region member */
    MPU_InitStruct->MPU_Region = 0;
    
    /* Initialize the MPU_RegionSize member */
    MPU_InitStruct->MPU_RegionSize = MPU_RegionSize_32B;
    
    /* Initialize the MPU_Address member */
    MPU_InitStruct->MPU_Address = 0xFFFFFF00;
    
    /* Initialize the MPU_AccessPermission member */
    MPU_InitStruct->MPU_AccessPermission = MPU_AP_PRI_NO_USER_NO;
    
    /* Initialize the MPU_Execute member */
    MPU_InitStruct->MPU_Execute = MPU_Execute_Enable;
    
    /* Initialize the MPU_Sharable member */
    MPU_InitStruct->MPU_Sharable = MPU_Sharable_Disable;
    
    /* Initialize the MPU_Buffable member */
    MPU_InitStruct->MPU_Buffable = MPU_Buffable_Disable;
    
    /* Initialize the MPU_SubregionDisable member */
    MPU_InitStruct->MPU_SubregionDisable = MPU_Subregion_Disable_None;
    
    /* Initialize the MPU_RegionCmd member */
    MPU_InitStruct->MPU_RegionCmd = DISABLE;
}

/**
  * @brief  Initialize the MPU according to the specified parameters in the MPU_InitStruct.
  * @param  MPU_InitStruct: pointer to a @ref MPU_InitTypeDef structure that
  *         contains the configuration information for the MPU.
  * @retval None
  */
void MPU_Init(MPU_InitTypeDef* MPU_InitStruct)
{
    uint32_t tempReg1 = 0;
    uint32_t tempReg2 = 0;

    /* Check the parameters */
    assert_param(IS_MPU_REGION_SIZE(MPU_InitStruct->MPU_RegionSize));
    assert_param(IS_MPU_AP_MODE(MPU_InitStruct->MPU_AccessPermission));
    assert_param(IS_MPU_EXECUTE_CMD(MPU_InitStruct->MPU_Execute));
    assert_param(IS_MPU_BUFFABLE_CMD(MPU_InitStruct->MPU_Buffable));
    assert_param(IS_MPU_SHARABLE_CMD(MPU_InitStruct->MPU_Sharable));
    assert_param(IS_MPU_SUBREGION_DISABLE(MPU_InitStruct->MPU_SubregionDisable));
    assert_param(IS_FUNCTION_STATE(MPU_InitStruct->MPU_RegionCmd));
    
    tempReg1 = MPU->RBAR;
    tempReg1 &= ~MPU_RBAR_REGION_Msk;
    tempReg1 = (MPU_InitStruct->MPU_Address | MPU_InitStruct->MPU_Region | MPU_RBAR_VALID_Msk); 
    MPU->RBAR = tempReg1;

    tempReg2 = MPU->RASR;
    tempReg2 &= ~(MPU_RASR_XN_Msk | MPU_RASR_AP_Msk | MPU_RASR_S_Msk | MPU_RASR_B_Msk \
                  | MPU_RASR_SRD_Msk | MPU_RASR_SIZE_Msk | MPU_RASR_ENABLE_Msk);
    tempReg2 |= MPU_InitStruct->MPU_Execute | MPU_InitStruct->MPU_AccessPermission \
                | MPU_InitStruct->MPU_Sharable | MPU_InitStruct->MPU_Buffable | MPU_InitStruct->MPU_RegionSize \
                | MPU_InitStruct->MPU_SubregionDisable | MPU_InitStruct->MPU_RegionCmd;
    MPU->RASR = tempReg2;
}

/**
  * @brief  Open or close the MPU peripheral.
  * @param  OnOffState: state of the MPU.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void MPU_OnOff(CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_CMD_STATE(OnOffState));

    if (OnOffState != OFF)
    {
        __ISB();
        /* Enable MPU function */
        MPU->CTRL |= MPU_CTRL_ENABLE_Msk; 
        __DSB();
    }
    else
    {
        __ISB();
        /* Disable MPU function */
        MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
        __DSB();
    }
}

/**
  * @brief  Get the current region number.
  * @param  None
  * @retval The Region Number.
  */
uint8_t MPU_GetCurrentRegionNumber(void)
{
    /* Get the current region number */
    return MPU->RNR;
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

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/


