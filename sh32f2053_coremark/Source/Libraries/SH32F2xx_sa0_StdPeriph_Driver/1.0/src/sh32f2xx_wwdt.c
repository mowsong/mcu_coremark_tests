/**
  ******************************************************************************
  * @file    sh32f2xx_wwdt.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    29-April-2017 
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
#include "sh32f2xx_wwdt.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup WWDT_MODULE WWDT 
  * @brief WWDT driver modules
  * @{
  */ 


/** @defgroup WWDT_Public_Functions Public Functions
  * @{
  */

/**
  * @brief  Deinitializes the WWDT peripheral registers to their default reset values.
  */
void WWDT_Reset(void)
{   
    RCC_APB1PeriphReset(RCC_APB1_WWDT);
}

/**
  * @brief  Sets the WWDT Prescaler.
  * @param  WWDT_Prescaler: specifies the WWDT Prescaler.
  *   This parameter can be one of the following values:
  *     @arg WWDT_Prescaler_1: 		WWDT counter clock = (PCLK1/256)/1
  *     @arg WWDT_Prescaler_2: 		WWDT counter clock = (PCLK1/256)/2
  *     @arg WWDT_Prescaler_4: 		WWDT counter clock = (PCLK1/256)/4
  *     @arg WWDT_Prescaler_8: 		WWDT counter clock = (PCLK1/256)/8
  *     @arg WWDT_Prescaler_16: 	WWDT counter clock = (PCLK1/256)/16
  *     @arg WWDT_Prescaler_32: 	WWDT counter clock = (PCLK1/256)/32
  *     @arg WWDT_Prescaler_64: 	WWDT counter clock = (PCLK1/256)/64
  *     @arg WWDT_Prescaler_128: 	WWDT counter clock = (PCLK1/256)/128  
  */
void WWDT_SetPrescaler(uint32_t WWDT_Prescaler)
{
  uint32_t tmpreg = 0;
  /* Check the parameters */
  assert_param(IS_WWDT_PRESCALER(WWDT_Prescaler));
  /* Clear WWDTPR[2:0] bits */
  tmpreg = WWDT->CR.V32 & (~WWDT_CR_WWDTPR_Msk);
  /* Set WWDTPR[2:0] bits according to WWDT_Prescaler value */
  tmpreg |= ((0x5AA5 << WWDT_CR_LOCK_Pos)|WWDT_Prescaler);
  /* Store the new value */
  WWDT->CR.V32 = tmpreg;
}

/**
  * @brief  Sets the WWDT window value.
  * @param  WindowValue: specifies the window value to be compared to the downcounter.
  */
void WWDT_SetWindowValue(uint8_t WindowValue)
{
  /* Set WWDTWTR[7:0] bits according to WindowValue value */
  WWDT->WTR.V32 = ((0x5AA5 << WWDT_WTR_LOCK_Pos)|WindowValue);
}

/**
  * @brief  Controles the WWDT Early Wakeup interrupt.(Enable or disable)
  * @note   Once enabled this interrupt cannot be disabled except by a system reset.
  * @param  EnableINT: Enable or disable the WWDT Early Wakeup interrupt
  */
void WWDT_EnableINT(bool_t EnableINT)
{
  uint32_t tmpreg = 0;  
    
  /* Clear WWDTIE bits */
  tmpreg = WWDT->CR.V32 & (~WWDT_CR_WWDTIE_Msk);
  
  /* Set WWDTIE bits according to WWDT_Prescaler value */
  tmpreg |= ((0x5AA5 << WWDT_CR_LOCK_Pos)
  				| (EnableINT << WWDT_CR_WWDTIE_Pos ));
  				
  /* Store the new value */
  WWDT->CR.V32 = tmpreg;
}

/**
  * @brief  Sets the WWDT counter value.
  * @param  Counter: specifies the watchdog counter value.
  *   This parameter must be a number between 0x0 and 0xFF (to prevent generating
  *   an immediate reset) 
  */
void WWDT_SetCounter(uint8_t Counter)
{
  uint32_t tmpreg = 0;  

  /* Clear WWDTRLR[7:0] bits */
  tmpreg = WWDT->CR.V32 & (~WWDT_CR_WWDTRLR_Msk);
  /* Sets the WWDT counter value */
  tmpreg |= ((0x5AA5 << WWDT_CR_LOCK_Pos) | (Counter & 0xFF ));
  /* Store the new value */
  WWDT->CR.V32 = tmpreg;
}

/**
  * @brief  Enables WWDT and load the counter value.                  
  * @param  WWDT_Prescaler: specifies the WWDT Prescaler.
  *   This parameter can be one of the following values:
  *     @arg WWDT_Prescaler_1: 		WWDT counter clock = (PCLK1/256)/1
  *     @arg WWDT_Prescaler_2: 		WWDT counter clock = (PCLK1/256)/2
  *     @arg WWDT_Prescaler_4: 		WWDT counter clock = (PCLK1/256)/4
  *     @arg WWDT_Prescaler_8: 		WWDT counter clock = (PCLK1/256)/8
  *     @arg WWDT_Prescaler_16: 	WWDT counter clock = (PCLK1/256)/16
  *     @arg WWDT_Prescaler_32: 	WWDT counter clock = (PCLK1/256)/32
  *     @arg WWDT_Prescaler_64: 	WWDT counter clock = (PCLK1/256)/64
  *     @arg WWDT_Prescaler_128: 	WWDT counter clock = (PCLK1/256)/128  
  * @param  EnableINT: Controles the WWDT Early Wakeup interrupt.
  * @param  Counter: specifies the watchdog counter value.
  *   This parameter must be a number between 0x0 and 0xFF (to prevent generating
  *   an immediate reset)
  * @param  WindowValue: specifies the window value to be compared to the downcounter.
  * @retval None
  */
void WWDT_Enable(uint32_t WWDT_Prescaler, bool_t EnableINT, uint8_t Counter, uint8_t WindowValue)
{
  /* Check the parameters */
  assert_param(IS_WWDT_PRESCALER(WWDT_Prescaler));
  
  WWDT->CR.V32 = (( 0x5AA5 << WWDT_CR_LOCK_Pos )
  				| ( EnableINT << WWDT_CR_WWDTIE_Pos )
  				| ( ENABLE << WWDT_CR_WWDTON_Pos )  				
  				| ( WWDT_Prescaler )
  				| ( Counter & 0xFF ));

  /* Set WWDTWTR[7:0] bits according to WindowValue value */
  WWDT->WTR.V32 = ((0x5AA5 << WWDT_WTR_LOCK_Pos) | WindowValue);
}


/**
  * @brief  Checks whether the Early Wakeup interrupt flag is set or not.
  * @retval The new state of the Early Wakeup interrupt flag (SET or RESET)
  */
FlagStatus WWDT_GetINTFlagStatus(void)
{
  /* Return the flag status */
	return (WWDT->SR.BIT.WWDTIF ? SET : RESET);
}

/**
  * @brief  Clears Early Wakeup interrupt flag.
  */
void WWDT_ClearFlag(void)
{
	WWDT->SR.BIT.WWDTIF = RESET;
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


