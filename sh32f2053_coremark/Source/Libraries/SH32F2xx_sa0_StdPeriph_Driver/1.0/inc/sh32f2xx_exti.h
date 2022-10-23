/**
  ******************************************************************************
  * @file    sh32f2xx_exti.h
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
#ifndef __SH32F2xx_EXTI_H
#define __SH32F2xx_EXTI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup EXTI_MODULE
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/

/** @defgroup EXTI_Exported_Types
  * @{
  */


/** 
  * @brief  EXTI mode enumeration  
  */

typedef enum
{
    EXTI_Mode_Interrupt = 0x00,
    EXTI_Mode_WakeupEvent = 0x04,
    EXTI_Mode_DMAEvent = 0x2C
}EXTIMode_TypeDef;

#define IS_EXTI_MODE(MODE)    ((MODE == EXTI_Mode_Interrupt) \
                              || (MODE == EXTI_Mode_WakeupEvent) \
                              || (MODE ==EXTI_Mode_DMAEvent))

/** 
  * @brief  EXTI Trigger enumeration  
  */

typedef enum
{
    EXTI_Trigger_Rising = 0x0C,
    EXTI_Trigger_Falling = 0x10,  
    EXTI_Trigger_Rising_Falling = 0x0F,
    EXTI_Trigger_HighLevel = 0x2C,
    EXTI_Trigger_LowLevel = 0x30,
    EXTI_Trigger_HighLevel_LowLevel = 0x3F
}EXTITrigger_TypeDef;

#define IS_EXTI_TRIGGER(TRIGGER)   ((TRIGGER == EXTI_Trigger_Rising) \
                                   || (TRIGGER == EXTI_Trigger_Falling) \
                                   || (TRIGGER == EXTI_Trigger_Rising_Falling) \
                                   || (TRIGGER == EXTI_Trigger_HighLevel) \
                                   || (TRIGGER == EXTI_Trigger_LowLevel) \
                                   || (TRIGGER == EXTI_Trigger_HighLevel_LowLevel))

/** 
  * @brief  EXTI sample cycle enumeration  
  */

typedef enum
{
    EXTI_SampleCycle_1 = 0x00,
    EXTI_SampleCycle_2 = 0x01,
    EXTI_SampleCycle_3 = 0x02,
    EXTI_SampleCycle_4 = 0x03
}EXTISampleCycle_TypeDef;

#define IS_EXTI_SAMPLE_CYCLE(CYCLE)    ((CYCLE == EXTI_SampleCycle_1) \
                                       || (CYCLE == EXTI_SampleCycle_2) \
                                       || (CYCLE == EXTI_SampleCycle_3) \
                                       || (CYCLE == EXTI_SampleCycle_4))

                                       
/** 
  * @brief  EXTI sample clock division enumeration  
  */

typedef enum
{
    EXTI_SampleClockDivision_1 = 0x00,
    EXTI_SampleClockDivision_4 = 0x01,
    EXTI_SampleClockDivision_16 = 0x02,
    EXTI_SampleClockDivision_128 = 0x03
}EXTISampleClockDivision_TypeDef;

#define IS_EXTI_SAMPLE_CLOCK_DIV(DIV)   ((DIV == EXTI_SampleClockDivision_1) \
                                        || (DIV == EXTI_SampleClockDivision_4) \
                                        || (DIV == EXTI_SampleClockDivision_16) \
                                        || (DIV == EXTI_SampleClockDivision_128))

/** 
  * @brief  EXTI Init Structure definition  
  */

typedef struct
{
    uint32_t EXTI_Line;               /*!< Specifies the EXTI lines to be enabled or disabled.
                                         This parameter can be any combination value of @ref EXTI_Lines */
   
    EXTIMode_TypeDef EXTI_Mode;       /*!< Specifies the mode for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */

    EXTITrigger_TypeDef EXTI_Trigger; /*!< Specifies the trigger signal active edge for the EXTI lines.
                                         This parameter can be a value of @ref EXTITrigger_TypeDef */
    EXTISampleCycle_TypeDef EXTI_SampleCycle;
    
    EXTISampleClockDivision_TypeDef EXTI_SampleClockDivision;
    
    uint8_t EXTI_LineCmd;     /*!< Specifies the new state of the selected EXTI lines.
                                         This parameter can be set either to ENABLE or DISABLE */ 
}EXTI_InitTypeDef;

/**
  * @}
  */


/* Exported constants --------------------------------------------------------*/

/** @defgroup EXTI_Exported_Constants
  * @{
  */


/** @defgroup EXTI_Lines
  * @{
  */
  
#define EXTI_Line0       ((uint32_t)0x00001)     /*!< External interrupt line 0 */
#define EXTI_Line1       ((uint32_t)0x00002)     /*!< External interrupt line 1 */
#define EXTI_Line2       ((uint32_t)0x00004)     /*!< External interrupt line 2 */
#define EXTI_Line3       ((uint32_t)0x00008)     /*!< External interrupt line 3 */
#define EXTI_Line4       ((uint32_t)0x00010)     /*!< External interrupt line 4 */
#define EXTI_Line5       ((uint32_t)0x00020)     /*!< External interrupt line 5 */
#define EXTI_Line6       ((uint32_t)0x00040)     /*!< External interrupt line 6 */
#define EXTI_Line7       ((uint32_t)0x00080)     /*!< External interrupt line 7 */
#define EXTI_Line8       ((uint32_t)0x00100)     /*!< External interrupt line 8 */
#define EXTI_Line9       ((uint32_t)0x00200)     /*!< External interrupt line 9 */
#define EXTI_Line10      ((uint32_t)0x00400)     /*!< External interrupt line 10 */
#define EXTI_Line11      ((uint32_t)0x00800)     /*!< External interrupt line 11 */
#define EXTI_Line12      ((uint32_t)0x01000)     /*!< External interrupt line 12 */
#define EXTI_Line13      ((uint32_t)0x02000)     /*!< External interrupt line 13 */
#define EXTI_Line14      ((uint32_t)0x04000)     /*!< External interrupt line 14 */
#define EXTI_Line15      ((uint32_t)0x08000)     /*!< External interrupt line 15 */

#define IS_EXTI_LINE(LINE)   (((LINE & (uint32_t)0xFFFF0000) == 0x00) && (LINE != 0x00))

#define IS_EXTI_GET_ONE_LINE(LINE)  ((LINE == EXTI_Line0) || (LINE == EXTI_Line1) \
                                    || (LINE == EXTI_Line2) || (LINE == EXTI_Line3) \
                                    || (LINE == EXTI_Line4) || (LINE == EXTI_Line5) \
                                    || (LINE == EXTI_Line6) || (LINE == EXTI_Line7) \
                                    || (LINE == EXTI_Line8) || (LINE == EXTI_Line9) \
                                    || (LINE == EXTI_Line10) || (LINE == EXTI_Line11) \
                                    || (LINE == EXTI_Line12) || (LINE == EXTI_Line13) \
                                    || (LINE == EXTI_Line14) || (LINE == EXTI_Line15))

/**
  * @}
  */


/** @defgroup EXTI_Port_Sources
  * @{
  */
#define EXTI_PortSourceGPIOA            ((uint8_t)0x00)
#define EXTI_PortSourceGPIOB            ((uint8_t)0x01)
#define EXTI_PortSourceGPIOC            ((uint8_t)0x02)
#define EXTI_PortSourceGPIOD            ((uint8_t)0x03)
#define EXTI_PortSourceGPIOE            ((uint8_t)0x04)
#define IS_EXTI_PORT_SOURCE(SOURCE)     ((SOURCE == EXTI_PortSourceGPIOA) \
                                        || (SOURCE == EXTI_PortSourceGPIOB) \
                                        || (SOURCE == EXTI_PortSourceGPIOC) \
                                        || (SOURCE == EXTI_PortSourceGPIOD) \
                                        || (SOURCE == EXTI_PortSourceGPIOE))

/**
  * @}
  */



/** @defgroup EXTI_Pin_Sources
  * @{
  */
  
#define EXTI_PinSource0            ((uint8_t)0x00)
#define EXTI_PinSource1            ((uint8_t)0x01)
#define EXTI_PinSource2            ((uint8_t)0x02)
#define EXTI_PinSource3            ((uint8_t)0x03)
#define EXTI_PinSource4            ((uint8_t)0x04)
#define EXTI_PinSource5            ((uint8_t)0x05)
#define EXTI_PinSource6            ((uint8_t)0x06)
#define EXTI_PinSource7            ((uint8_t)0x07)
#define EXTI_PinSource8            ((uint8_t)0x08)
#define EXTI_PinSource9            ((uint8_t)0x09)
#define EXTI_PinSource10           ((uint8_t)0x0A)
#define EXTI_PinSource11           ((uint8_t)0x0B)
#define EXTI_PinSource12           ((uint8_t)0x0C)
#define EXTI_PinSource13           ((uint8_t)0x0D)
#define EXTI_PinSource14           ((uint8_t)0x0E)
#define EXTI_PinSource15           ((uint8_t)0x0F)
#define IS_EXTI_PIN_SOURCE(SOURCE)  ((SOURCE == EXTI_PinSource0) \
                                    || (SOURCE == EXTI_PinSource1) \
                                    || (SOURCE == EXTI_PinSource2) \
                                    || (SOURCE == EXTI_PinSource3) \
                                    || (SOURCE == EXTI_PinSource4) \
                                    || (SOURCE == EXTI_PinSource5) \
                                    || (SOURCE == EXTI_PinSource6) \
                                    || (SOURCE == EXTI_PinSource7) \
                                    || (SOURCE == EXTI_PinSource8) \
                                    || (SOURCE == EXTI_PinSource9) \
                                    || (SOURCE == EXTI_PinSource10) \
                                    || (SOURCE == EXTI_PinSource11) \
                                    || (SOURCE == EXTI_PinSource12) \
                                    || (SOURCE == EXTI_PinSource13) \
                                    || (SOURCE == EXTI_PinSource14) \
                                    || (SOURCE == EXTI_PinSource15))

/**
  * @}
  */



/**
  * @}
  */ 


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

/** @defgroup EXTI_Exported_Functions
  * @{
  */
  
extern void EXTI_Reset(void);
extern void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
extern void EXTI_StructInit(EXTI_InitTypeDef * EXTI_InitStruct);
extern void EXTI_PinConfig(uint8_t EXTI_PortSourceGPIOx,uint8_t EXTI_PinSourcex);
extern void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);
extern FlagStatus EXTI_GetPendingFlagStatus(uint32_t EXTI_Line);
extern void EXTI_ClearPendingFlag(uint32_t EXTI_Line);

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_EXTI_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
