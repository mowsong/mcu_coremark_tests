/**
  ******************************************************************************
  * @file    sh32f2xx_nvic.c
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
#include "sh32f2xx_nvic.h"
#include "sh32f2xx_rcc.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup NVIC_MODULE NVIC 
  * @brief PPP driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup NVIC_Private_Functions
  * @{
  */ 

  
  
/**
  * @brief  Initializes the NVIC peripheral according to the specified parameters 
  *         in the NVIC_InitStruct.
  * @param  NVIC_InitStruct: pointer to a NVIC_InitTypeDef structure that contains
  *         the configuration information for the specified NVIC peripheral.
  * @retval None
  */
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct)
{
    uint32_t TempPriority = 0x00;
    uint32_t TempPre = 0x00;
    uint32_t TempSub = 0x0F;
    
    /* Check the parameters */
    assert_param(IS_FUNCTION_STATE(NVIC_InitStruct->NVIC_IRQChannelCmd));

    if (((SCB->AIRCR) & (uint32_t)0x700) < 0x300)
    {
        SCB->AIRCR = ((uint32_t)0x05FA << 16) | 0x300;
    }
    
    if (NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE)
    {
        /* Compute the Corresponding IRQ Priority --------------------------------*/    
        TempPriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700)) >> 0x08;
        TempPre = (0x4 - TempPriority);
        TempSub = TempSub >> TempPriority;

        TempPriority = (uint32_t)NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority << TempPre;
        TempPriority |=  NVIC_InitStruct->NVIC_IRQChannelSubPriority & TempSub;
        TempPriority = TempPriority << 0x04;
            
        NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel] = TempPriority;

        /* Enable the Selected IRQ Channels --------------------------------------*/
        NVIC->ISER[NVIC_InitStruct->NVIC_IRQChannel >> 0x05] = \
          (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
    }
    else
    {
        /* Disable the Selected IRQ Channels -------------------------------------*/
        NVIC->ICER[NVIC_InitStruct->NVIC_IRQChannel >> 0x05] = \
              (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
    }
}


/**
  * @brief  Configures the priority grouping: pre-emption priority and subpriority.
  * @param  NVIC_PriorityGroup: specifies the priority grouping bits length. 
  *          This parameter can be one of the following values:
  *            @arg NVIC_PriorityGroup_0: 4 bits for pre-emption priority
  *                                       0 bits for subpriority
  *            @arg NVIC_PriorityGroup_1: 3 bits for pre-emption priority
  *                                       1 bits for subpriority
  *            @arg NVIC_PriorityGroup_2: 2 bits for pre-emption priority
  *                                       2 bits for subpriority
  *            @arg NVIC_PriorityGroup_3: 1 bits for pre-emption priority
  *                                       3 bits for subpriority
  *            @arg NVIC_PriorityGroup_4: 0 bits for pre-emption priority
  *                                       4 bits for subpriority
  * @retval None
  */
void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup)
{
    uint32_t temp = 0;
    
    /* Check the parameters */
    assert_param(IS_NVIC_PRIORITY_GROUP(NVIC_PriorityGroup));
    
    temp = SCB->AIRCR;
    temp &= ~(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk);
    /* Set the PRIGROUP[10:8] bits according to NVIC_PriorityGroup value */
    temp |= (0x05FA << 16) | NVIC_PriorityGroup;
    SCB->AIRCR = temp;
}

/**
  * @brief  Sets the vector table location and Offset.
  * @param  NVIC_VectTab: specifies if the vector table is in RAM or FLASH memory.
  *          This parameter can be one of the following values:
  *            @arg NVIC_VectTab_RAM
  *            @arg NVIC_VectTab_FLASH
  * @param  Offset: Vector Table base offset field. This value must be a multiple 
  *         of 0x200.
  * @retval None
  */
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset)
{ 
    SCB->VTOR = NVIC_VectTab | (Offset & (uint32_t)0x1FFFFF80);
}


/**
  * @brief  Config SysTick timer.
  * @param  MilliSecond: specifie milliseconds to be set to SysTick.
  *          This parameter can be 0 to 1000:
  * @retval None
  */
void SysTick_TimeConfig(uint16_t MilliSecond)
{
    RCC_Clocks_TypeDef clockStatus;

    RCC_GetClocksFreq(&clockStatus);
    SysTick->LOAD = ((clockStatus.hclkFreq / 1000) >> 3) * MilliSecond;
    SysTick->VAL = 0;
    SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;    
}

/**
  * @brief  Get the SysTick Counter value.
  * @param  None
  * @retval SysTick Counter Register value.
  */
uint32_t SysTick_GetCounter(void)
{
    /* Get the SysTick Counter value */
    return SysTick->VAL;
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


