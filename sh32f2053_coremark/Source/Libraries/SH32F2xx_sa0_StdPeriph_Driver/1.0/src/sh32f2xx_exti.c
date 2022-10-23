/**
  ******************************************************************************
  * @file    sh32f2xx_exti.c
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
#include "sh32f2xx_exti.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup EXTI_MODULE EXTI 
  * @brief EXTI driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup EXTI_Private_Functions EXTI Private Functions
  * @brief EXTI Private Functions
  *
  * @{
  */ 

/**
  * @brief  Reset the EXTI peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
void EXTI_Reset(void)
{
    EXTI->IMR = 0x00000000;
    EXTI->EMR = 0x00000000;
    EXTI->DMR = 0x00000000;
    EXTI->TMSR = 0x00000000;
    EXTI->RTSR = 0x00000000;
    EXTI->FTSR = 0x00000000;
    EXTI->CFGL.V32 = 0x00000000;
    EXTI->CFGH.V32 = 0x00000000;
    EXTI->SAMPL.V32 = 0x00000000;
    EXTI->SAMPH.V32 = 0x00000000;
    EXTI->PR.V32 = 0xFFFF0000;
}

/**
  * @brief  Initializes the EXTI peripheral according to the specified
  *         parameters in the EXTI_InitStruct.
  * @param  EXTI_InitStruct: pointer to a @ref EXTI_InitTypeDef structure
  *         that contains the configuration information for the EXTI peripheral.
  * @retval None
  */
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct)
{
    uint8_t pinPosition = 0;
    uint32_t EXTI_Linex = 0;
    uint32_t tempAddress = 0;

    /* Check the parameters */
    assert_param(IS_EXTI_LINE(EXTI_InitStruct->EXTI_Line));
    assert_param(IS_EXTI_MODE(EXTI_InitStruct->EXTI_Mode));
    assert_param(IS_EXTI_TRIGGER(EXTI_InitStruct->EXTI_Trigger));
    assert_param(IS_EXTI_SAMPLE_CYCLE(EXTI_InitStruct->EXTI_SampleCycle));
    assert_param(IS_EXTI_SAMPLE_CLOCK_DIV(EXTI_InitStruct->EXTI_SampleClockDivision));
    
    tempAddress = (uint32_t)EXTI_BASE;        
    
    if (EXTI_InitStruct->EXTI_LineCmd != DISABLE)
    {              
        /* clear select exti line enable bit */
        EXTI->IMR &= ~EXTI_InitStruct->EXTI_Line; 
        EXTI->EMR &= ~EXTI_InitStruct->EXTI_Line;
        EXTI->DMR &= ~EXTI_InitStruct->EXTI_Line;
        
        tempAddress += EXTI_InitStruct->EXTI_Mode;
        *(__IO uint32_t *)tempAddress |= EXTI_InitStruct->EXTI_Line;
        
        EXTI->TMSR &= ~EXTI_InitStruct->EXTI_Line;
        EXTI->RTSR &= ~EXTI_InitStruct->EXTI_Line;
        EXTI->FTSR &= ~EXTI_InitStruct->EXTI_Line;
        
        if (EXTI_InitStruct->EXTI_Trigger >= EXTI_Trigger_HighLevel)
        {
            EXTI->TMSR |= EXTI_InitStruct->EXTI_Line;
        }
        else
        {
            EXTI->TMSR &= ~EXTI_InitStruct->EXTI_Line;
        }
        
        if ((EXTI_InitStruct->EXTI_Trigger == EXTI_Trigger_Rising_Falling) || \
            (EXTI_InitStruct->EXTI_Trigger == EXTI_Trigger_HighLevel_LowLevel))
        {
            EXTI->RTSR |= EXTI_InitStruct->EXTI_Line;
            EXTI->FTSR |= EXTI_InitStruct->EXTI_Line;
        }
        else
        {
            tempAddress = (uint32_t)EXTI_BASE;
            tempAddress += (EXTI_InitStruct->EXTI_Trigger & 0x1F);
            *(__IO uint32_t *) tempAddress |= EXTI_InitStruct->EXTI_Line;
        }  
        for (pinPosition=0;pinPosition<16;pinPosition++)
        {
            EXTI_Linex = ((uint32_t)0x01) << pinPosition;
            if ((EXTI_InitStruct->EXTI_Line & EXTI_Linex) == EXTI_Linex)
            {
                if (pinPosition > 0x07)
                {
                    EXTI->SAMPH.V32 &= (~(((uint32_t)0x0F)<<((pinPosition-8)*4)));
                    EXTI->SAMPH.V32 |= (((uint32_t)EXTI_InitStruct->EXTI_SampleCycle | ((uint32_t)EXTI_InitStruct->EXTI_SampleClockDivision << 2))<<((pinPosition-8)*4));
                }
                else
                {
                    EXTI->SAMPL.V32 &= (~(((uint32_t)0x0F)<<(pinPosition*4)));
                    EXTI->SAMPL.V32 |= (((uint32_t)EXTI_InitStruct->EXTI_SampleCycle | ((uint32_t)EXTI_InitStruct->EXTI_SampleClockDivision << 2))<<(pinPosition*4));
                }
            }
        } 

    }
    else
    {
        tempAddress += EXTI_InitStruct->EXTI_Mode;
        *(__IO uint32_t *)tempAddress &= ~EXTI_InitStruct->EXTI_Line;
    }
}

/**
  * @brief  Fills each EXTI_InitStruct member with its reset value.
  * @param  EXTI_InitStruct: pointer to a @ref EXTI_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct)
{
    EXTI_InitStruct->EXTI_Line = (uint16_t)0x0000;
    EXTI_InitStruct->EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct->EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStruct->EXTI_SampleCycle = EXTI_SampleCycle_1;
    EXTI_InitStruct->EXTI_SampleClockDivision = EXTI_SampleClockDivision_1;
    EXTI_InitStruct->EXTI_LineCmd = DISABLE;
}

/**
  * @brief  Select the GPIO pin used as EXTI Line.
  * @param  EXTI_PortSourceGPIOx: select the GPIO port to be used as source for
  *          EXTI lines where x can be A..E. 
  * @param  EXTI_PinSourcex: specifie the EXTI line to be configured.
  *          This parameter can be EXTI_PinSourcex where x can be 0 to 15.
  * @retval None
  */
void EXTI_PinConfig(uint8_t EXTI_PortSourceGPIOx,uint8_t EXTI_PinSourcex)
{    
    /* Check the parameters */
    assert_param(IS_EXTI_PORT_SOURCE(EXTI_PortSourceGPIOx));
    assert_param(IS_EXTI_PIN_SOURCE(EXTI_PinSourcex));
    
    if (EXTI_PinSourcex > EXTI_PinSource7)
    {
        EXTI->CFGH.V32 &= (~(((uint32_t)0x0F)<<((EXTI_PinSourcex - EXTI_PinSource8)*4))); 
        EXTI->CFGH.V32 |= ((EXTI_PortSourceGPIOx)<<((EXTI_PinSourcex - EXTI_PinSource8)*4));
    }
    else
    {
        EXTI->CFGL.V32 &= (~(((uint32_t)0x0F)<<(EXTI_PinSourcex*4)));
        EXTI->CFGL.V32 |= ((EXTI_PortSourceGPIOx)<<(EXTI_PinSourcex*4));
    } 
}

/**
  * @brief  Generates a Software interrupt on selected EXTI line.
  * @param  EXTI_Line: specifies the EXTI line on which the software interrupt
  *         will be generated.
  *          This parameter can be any combination of EXTI_Linex where x can be 0 to 15.
  * @retval None
  */
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line)
{ 
    /* Check the parameters */
    assert_param(IS_EXTI_LINE(EXTI_Line));
    
    EXTI->SWIER = EXTI_Line;
}

/**
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  EXTI_Line: specifies the EXTI line flag to check.
  *          This parameter can be one of EXTI_Linex where x can be 0 to 15.
  * @retval The new state of EXTI_Line (SET or RESET).
  */
FlagStatus EXTI_GetPendingFlagStatus(uint32_t EXTI_Line)
{
    FlagStatus bitStatus;

    /* Check the parameters */
    assert_param(IS_EXTI_GET_ONE_LINE(EXTI_Line));

    if ((EXTI->PR.V32 & EXTI_Line) != (uint32_t)RESET)
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
  * @brief  Clears the EXTI's line pending flags.
  * @param  EXTI_Line: specifies the EXTI lines flags to clear.
  *          This parameter can be any combination of EXTI_Linex where x can be 0 to 15.
  * @retval None
  */
void EXTI_ClearPendingFlag(uint32_t EXTI_Line)
{
    /* Check the parameters */
    assert_param(IS_EXTI_LINE(EXTI_Line));
        
    EXTI->PR.V32 = (uint32_t)EXTI_Line << 16;
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


