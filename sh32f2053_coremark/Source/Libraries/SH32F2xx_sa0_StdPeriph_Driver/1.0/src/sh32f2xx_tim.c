/**
  ******************************************************************************
  * @file    sh32f2xx_tim.c
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
#include "sh32f2xx_tim.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup TIM_MODULE TIM 
  * @brief TIM driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup TIM_Private_Functions
  * @{
  */ 

/** @defgroup PPP_Group1 Initialization and Configuration
 *  @brief    Initialization and Configuration
 *
@verbatim    
 ===============================================================================
                      Initialization and Configuration
 ===============================================================================  
   
@endverbatim
  * @{
  */

/**
  * @brief  Reset the TIMx peripheral registers to their default reset values.
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @retval None
  */
void TIM_Reset(TIM_TypeDef* TIMx)
{
    /* Check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));
    
    if (TIMx == TIM5)
    {
        RCC_APB1PeriphReset(RCC_APB1_TIM5);
    }
    else if (TIMx == TIM6)
    {
        RCC_APB1PeriphReset(RCC_APB1_TIM6);
    }
    else if (TIMx == TIM7)
    {
        RCC_APB1PeriphReset(RCC_APB1_TIM7);
    }
    else
    {
        RCC_APB1PeriphReset(RCC_APB1_TIM8);
    }
}

/**
  * @brief  Fills each TIM_InitStruct member with its default value.
  * @param  TIM_InitStruct : pointer to a @ref TIM_InitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
void TIM_StructInit(TIM_InitTypeDef* TIM_InitStruct)
{
    TIM_InitStruct->TIM_Prescaler = 0x00;
    TIM_InitStruct->TIM_Period = 0x00;
    TIM_InitStruct->TIM_CLKSource = TIM_CLKSource_PCLK;
    TIM_InitStruct->TIM_OPMode = TIM_OPMode_Continue;
}

/**
  * @brief  Initializes the TIMx peripheral according to the specified parameters 
  *         in the TIM_InitStruct.
  * @param  TIMx: where x can be  5 to 8 to select the TIM peripheral.
  * @param  TIM_InitStruct: pointer to a @ref TIM_InitTypeDef structure that contains 
  *         the configuration information for the specified TIM peripheral.
  * @retval None
  */
void TIM_Init(TIM_TypeDef* TIMx, TIM_InitTypeDef* TIM_InitStruct)
{
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));
    assert_param(IS_TIM_CLK_SOURCE(TIM_InitStruct->TIM_CLKSource));
    assert_param(IS_TIM_OP_MODE(TIM_InitStruct->TIM_OPMode));

    TIMx->PSQ = TIM_InitStruct->TIM_Prescaler;
    TIMx->TPR = TIM_InitStruct->TIM_Period;
    
    TIMx->CR.V32 &= ~(TIM_CR_CLKS_Msk | TIM_CR_OPM_Msk);
    TIMx->CR.V32 |= TIM_InitStruct->TIM_CLKSource | TIM_InitStruct->TIM_OPMode; 
}

/**
  * @brief  Enable or disable the specified TIM interrupt.
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.       
  * @param  NewState: new state of the TIM interrupt.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void TIM_INTConfig(TIM_TypeDef* TIMx, FunctionalState NewState)
{
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));
    assert_param(IS_FUNCTION_STATE(NewState));
    
    if (NewState != DISABLE)
    {
        TIMx->CR.V32 |= TIM_CR_IE_Msk;
    }
    else
    {
        TIMx->CR.V32 &= (~TIM_CR_IE_Msk);
    }
}

/**
  * @brief  Open or close the TIM peripheral.
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @param  OnOffState: state of the TIM peripheral.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void TIM_OnOff(TIM_TypeDef* TIMx,      CmdState OnOffState)
{
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));
    assert_param(IS_CMD_STATE(OnOffState));

    if (OnOffState != OFF)
    {
        TIMx->CR.V32 |= TIM_CR_STR_Msk;
    }
    else
    {
        TIMx->CR.V32 &= (~TIM_CR_STR_Msk);
    }
}

/**
  * @brief  Open or close the TIM peripheral.
  * @param  TIMx: where x can be 5 or 7 to select the TIM peripheral.
  * @param  OnOffState: state of the Cascade TIM peripheral.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void TIM_CascadeOnOff(TIM_TypeDef* TIMx,         CmdState OnOffState)
{
    /* check the parameters */
    assert_param(IS_TIM_CASCADE_PERIPH(TIMx));
    assert_param(IS_CMD_STATE(OnOffState));

    if (OnOffState != OFF)
    {
        TIMx->CR.V32 |= TIM_CR_CASCEN_Msk;
    }
    else
    {
        TIMx->CR.V32 &= (~TIM_CR_CASCEN_Msk);
    }
}


/**
  * @brief  Enable or disable the TIMx's output pin.
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @param  NewState: new state of the TIMx's output pin.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void TIM_OutPutConfig(TIM_TypeDef* TIMx, FunctionalState NewState)
{
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));
    assert_param(IS_FUNCTION_STATE(NewState));
    
    if (NewState != DISABLE)
    {
        TIMx->CR.V32 |= TIM_CR_TC_Msk;
    }
    else
    {
        TIMx->CR.V32 &= ~TIM_CR_TC_Msk;
    }
}

/**
  * @brief  Enable or disable the TIMx's trigger CM3 core function.
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @param  NewState: new state of the TIMx's trigger function.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void TIM_CM3EventTrigger(TIM_TypeDef* TIMx, FunctionalState NewState)
{
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));
    assert_param(IS_FUNCTION_STATE(NewState));
    
    if (NewState != DISABLE)

    {
        TIMx->CR.V32 |= TIM_CR_ETEN_Msk;
    }
    else
    {
        TIMx->CR.V32 &= ~TIM_CR_ETEN_Msk;
    }
}

/**
  * @brief  Enable or disable the TIMx's trigger function.
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @param  NewState: new state of the TIMx's trigger function.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void TIM_ExtPeripheralsTrigger(TIM_TypeDef* TIMx, FunctionalState NewState)
{
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));
    assert_param(IS_FUNCTION_STATE(NewState));
    
    if (NewState != DISABLE)

    {
        TIMx->CR.V32 |= TIM_CR_TRIGEN_Msk;
    }
    else
    {
        TIMx->CR.V32 &= ~TIM_CR_TRIGEN_Msk;
    }
}

/**
  * @brief  Check whether the specified TIM flag is set or not.
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @retval The new state of TIMx's flag.
  */
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx)
{
    FlagStatus bitStatus;

    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));
    
    if ((TIMx->TIMINTF.V32 & TIM_TIMINTF_TF_Msk) != RESET)
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
  * @brief  Clear the TIMx's pending flag.
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @retval None
  */
void TIM_ClearFlag(TIM_TypeDef* TIMx)
{
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));
    
    TIMx->TIMINTF.V32 = TIM_TIMINTF_TFC_Msk;
}

/**
  * @brief  Get the TIMx Period Register value
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @retval Counter Register value
  */
uint32_t TIM_GetCounter(TIM_TypeDef* TIMx)
{
    
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));

    return TIMx->TCNT;
}

/**
  * @brief  Get the TIMx Period Register value
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @retval Prescaler Register value
  */
uint32_t TIM_GetPrescaler(TIM_TypeDef* TIMx)
{    
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));
    
    return TIMx->PSQ;
}

/**
  * @brief  Get the TIMx Period Register value
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @retval Period Register value
  */
uint32_t TIM_GetPeriod(TIM_TypeDef* TIMx)
{    
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));

    return TIMx->TPR;
}

/**
  * @brief  Set the TIMx Counter Register value
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @param  Counter: specifies the Counter register new value.
  * @retval None
  */
void TIM_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter)
{
    
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));

    TIMx->TCNT = Counter;
}

/**
  * @brief  Set the TIMx Prescaler Register value
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @param  Prescaler: specifies the Prescaler register new value.
  * @retval None
  */
void TIM_SetPrescaler(TIM_TypeDef* TIMx, uint32_t Prescaler)
{  
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));

    TIMx->PSQ = Prescaler;
}

/**
  * @brief  Set the TIMx Period Register value
  * @param  TIMx: where x can be 5 to 8 to select the TIM peripheral.
  * @param  Period: specifies the Period register new value.
  * @retval None
  */
void TIM_SetPeriod(TIM_TypeDef* TIMx, uint32_t Period)
{    
    /* check the parameters */
    assert_param(IS_TIM_ALL_PERIPH(TIMx));

    TIMx->TPR = Period;
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


