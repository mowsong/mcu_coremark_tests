/**
  ******************************************************************************
  * @file    sh32f2xx_TIM.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file contains all the functions prototypes for the PPP firmware
  *          library.
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
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SH32F2xx_TIM_H
#define __SH32F2xx_TIM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup TIM_MODULE
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/

/** @defgroup TIM_Exported_Types
  * @{
  */
  
typedef struct
{
    uint32_t TIM_Prescaler;   /*!< Specifie the TIM prescaler value.
                                   This parameter can be 0 to 0xFFFF */
                                   
    uint32_t TIM_Period;      /*!< Specifie the TIM period value.
                                   This parameter can be 0 to 0xFFFF */ 
                                   
    uint16_t TIM_CLKSource;   /*!< Specifie the TIM's clock sources.
                                   This parameter can be a value of @ref TIM_Clock_Sources */
                                   
    uint16_t TIM_OPMode;      /*!< Specifie the TIM OP mode.
                                   This parameter can be a value of @ref TIM_OP_Mode */
}TIM_InitTypeDef;

/**
  * @}
  */

  
/* Exported constants --------------------------------------------------------*/

/** @defgroup TIM_Exported_Constants
  * @{
  */ 
  
#define IS_TIM_ALL_PERIPH(PERIPH)  ((PERIPH == TIM5) || (PERIPH == TIM6) \
                                   || (PERIPH == TIM7) || (PERIPH == TIM8))
     
/** @defgroup TIM_Clock_Sources
  * @{
  */   
  
#define TIM_CLKSource_PCLK          (uint16_t)0x0000
#define TIM_CLKSource_TX            (uint16_t)0x0010
#define TIM_CLKSource_LSI           (uint16_t)0x0020
#define IS_TIM_CLK_SOURCE(SOURCE)   ((SOURCE == TIM_CLKSource_PCLK) \
                                    || (SOURCE == TIM_CLKSource_LSI) \
                                    || (SOURCE == TIM_CLKSource_TX))
  
/**
  * @}
  */     
     


/** @defgroup TIM_OP_Mode
  * @{
  */  
  
#define TIM_OPMode_Continue    (uint16_t)0x0000
#define TIM_OPMode_OnePulse    (uint16_t)0x0008
#define IS_TIM_OP_MODE(MODE)   ((MODE == TIM_OPMode_Continue) \
                               || (MODE == TIM_OPMode_OnePulse))
  
/**
  * @}
  */
     
     
     
     


/** @defgroup TIM_Trigger
  * @{
  */    
  
typedef enum
{
    TIM_TRIGGER_NONE = 0x00,
    TIM_TRIGGER_DMA = 0x01,
    TIM_TRIGGER_ADC = 0x02, /* only used in TIM8 */
    TIM_TRIGGER_CM3 = 0x03
}TIM_TriggerType;
  
/**
  * @}
  */


/** @defgroup TIM_Cascade
  * @{
  */    
  
#define IS_TIM_CASCADE_PERIPH(PERIPH)  ((PERIPH == TIM5) || (PERIPH == TIM7))
  
/**
  * @}
  */








/**
  * @}
  */ 


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 
void TIM_Reset(TIM_TypeDef* TIMx);
void TIM_StructInit(TIM_InitTypeDef* TIM_InitStruct);
void TIM_Init(TIM_TypeDef* TIMx, TIM_InitTypeDef* TIM_InitStruct);
void TIM_INTConfig(TIM_TypeDef* TIMx,    FunctionalState NewState);
void TIM_OnOff(TIM_TypeDef* TIMx,      CmdState OnOffState);
void TIM_CascadeOnOff(TIM_TypeDef* TIMx,         CmdState OnOffState);
void TIM_OutPutConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CM3EventTrigger(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_ExtPeripheralsTrigger(TIM_TypeDef* TIMx, FunctionalState NewState);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx);
void TIM_ClearFlag(TIM_TypeDef* TIMx);
uint32_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint32_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
uint32_t TIM_GetPeriod(TIM_TypeDef* TIMx);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter);
void TIM_SetPrescaler(TIM_TypeDef* TIMx, uint32_t Prescaler);
void TIM_SetPeriod(TIM_TypeDef* TIMx, uint32_t Period);






#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_TIM_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
