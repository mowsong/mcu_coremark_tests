/**
  ******************************************************************************
  * @file    sh32f2xx_gpt.h
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
#ifndef __SH32F2xx_GPT_H
#define __SH32F2xx_GPT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup GPT_MODULE
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/


/** @defgroup GPT_Exported_Types
  * @{
  */


/** 
  * @brief   GPT Time Base Init structure definition  
  */
  
typedef struct
{
	uint16_t CounterMode;  /*!< Specifie the GPTx's counter mode.
                                This parameter can be a value of @ref GPT_Counter_Mode */
                                
	uint16_t Prescaler;    /*!< Specifie the GPTx's prescaler value.
                                This parameter can be 0 to 0xFFFF */
                                
	uint16_t Period;       /*!< Specifie the GPTx's period value.
                                This parameter can be 0 to 0xFFFF */
	
}GPT_TimeBaseInitTypeDef;


/** 
  * @brief   GPT Output Compare Init structure definition  
  */
  
typedef struct
{
	uint16_t WaveMode;        /*!< Specifie the GPTx's wave mode.
                                   This parameter can be a value of @ref GPT_Wave_Mode */
                                   
	uint16_t StartLevel;      /*!< Specifie the GPTx output wave start level.
                                   This parameter can be a value of @ref GPT_Start_Level */
                                   
	uint16_t PeriodStatus;    /*!< Specifie the level status when period occur.
                                   This parameter can be a value of @ref GPT_Period_Status */
                                    
	uint16_t CompareStatus;   /*!< Specifie the level status when compare occur.
                                   This parameter can be a value of @ref GPT_Compare_Status */
                                   
    uint16_t Pulse;           /*!< Specifie the pulse value to be loaded into Compare Register.
                                   This parameter can be 0 to 0xFFFF */
}GPT_OCInitTypeDef;


/** 
  * @brief   GPT Input Capture Init structure definition  
  */
  
typedef struct
{
    uint16_t GPT_Channel;      /*!< Specifie the GPT channel.
                                    This parameter can be any combination of @ref GPT_Channel */
                                    
    uint16_t GPT_ICPolarity;   /*!< Specifie the active edge of the input signal.
                                    This parameter can be a value of @ref GPT_Input_Capture_Polarity */
                                    
    uint16_t GPT_ICSelection;  /*!< Specifie the input.
                                    This parameter can be a value of @ref GPT_Input_Capture_Selection */
                                    
    uint16_t GPT_ICFilter;     /*!< Specifie the input capture filter.
                                    This parameter can be a value of @ref GPT_Input_Capture_Filter */
}GPT_ICInitTypeDef;



/** 
  * @brief   GPT Dead Time Init structure definition  
  */
  
typedef struct
{
    uint16_t GPT_AutoSetDVDCmd;  /*!< Specifie the GPTx Auto set DVD function to be enabled or disabled.
                                      This parameter can be a value of @ref GPT_Dead_Time_DVD_Auto_Set */
                                      
    uint16_t GPT_GTDVU;          /*!< Specifie the GPTx dead time GTDVU register value.
                                      This parameter can be 0 to 0xFFFF. */
                                      
    uint16_t GPT_GTDVD;          /*!< Specifie the GPTx dead time GTDVD register value.
                                      This parameter can be 0 to 0xFFFF. */
                                      
    uint16_t GPT_GTDVUBufCmd;    /*!< Specifie the GPTx dead time GTDVU buffer function to be enabled or disabled.
                                      This parameter can be a value of @ref GPT_Dead_Time_GTDVU_Buffer */
                                      
    uint16_t GPT_GTDBU;          /*!< Specifie QEI position counter mode.
                                      This parameter can be 0 to 0xFFFF */
                                      
    uint16_t GPT_GTDVDBufCmd;    /*!< Specifie QEI position counter mode.
                                      This parameter can be a value of @ref GPT_Dead_Time_GTDVD_Buffer */
                                      
    uint16_t GPT_GTDBD;          /*!< Specifie QEI position counter mode.
                                      This parameter can be 0 to 0xFFFF */
}GPT_DeadTimeInitTypeDef;


/**
  * @}
  */



/* Exported constants --------------------------------------------------------*/

/** @defgroup GPT_Exported_Constants
  * @{
  */ 
#define IS_GPT_ALL_PERIPH(PERIPH)  ((PERIPH == GPT0) || (PERIPH == GPT1) \
                                   || (PERIPH == GPT2) || (PERIPH == GPT3))

/** @defgroup GPT_Clock_Division
  * @{
  */

#define GPT_CLK_DIV_1         (uint8_t)0x00
#define GPT_CLK_DIV_4         (uint8_t)0x40
#define GPT_CLK_DIV_16        (uint8_t)0x80
#define GPT_CLK_DIV_128       (uint8_t)0xC0
#define IS_GPT_CLK_DIV(DIV)   ((DIV == GPT_CLK_DIV_1) || (DIV == GPT_CLK_DIV_4) \
                              || (DIV == GPT_CLK_DIV_16) || (DIV == GPT_CLK_DIV_128))

/**
  * @}
  */


/** @defgroup GPT_Sample_Numbers
  * @{
  */

#define GPT_SAMPLE_NUM_1        (uint8_t)0x00
#define GPT_SAMPLE_NUM_2        (uint8_t)0x10
#define GPT_SAMPLE_NUM_3        (uint8_t)0x20
#define GPT_SAMPLE_NUM_4        (uint8_t)0x30
#define IS_GPT_SAMPLE_NUM(NUM)  ((NUM == GPT_SAMPLE_NUM_1) || (NUM == GPT_SAMPLE_NUM_2) \
                                || (NUM == GPT_SAMPLE_NUM_3) || (NUM == GPT_SAMPLE_NUM_4))

/**
  * @}
  */


/** @defgroup GPT_POE_Filter
  * @{
  */

#define GPT_POE_Filter_None         (uint8_t)0x00
#define GPT_POE_Filter_8            (uint8_t)0x10
#define GPT_POE_Filter_16           (uint8_t)0x20
#define GPT_POE_Filter_128          (uint8_t)0x30
#define IS_GPT_POE_FILTER(FILTER)   ((FILTER == GPT_POE_Filter_None) \
                                    || (FILTER == GPT_POE_Filter_8) \
                                    || (FILTER == GPT_POE_Filter_16) \
                                    || (FILTER == GPT_POE_Filter_128))

/**
  * @}
  */


/** @defgroup GPT_POE_Hiz
  * @{
  */

#define GPT_HIZ_GPT0AB             (uint8_t)0x01
#define GPT_HIZ_GPT1AB             (uint8_t)0x02
#define GPT_HIZ_GPT2AB             (uint8_t)0x04
#define GPT_HIZ_GPT3AB             (uint8_t)0x08
#define IS_GPT_HIZ(HIZ)            (((HIZ & 0x0F) != 0x00) && ((HIZ | 0x0F) == 0x0F))

/**
  * @}
  */


/** @defgroup GPT_Common_Interrupt_Sources
  * @{
  */

#define GPT_COMMON_INT_POE          (uint8_t)0x80
#define GPT_COMMON_INT_ETIP         (uint8_t)0x01
#define GPT_COMMON_INT_ETIN         (uint8_t)0x02
#define IS_GPT_COMMON_INT(INT)       (((INT & (uint8_t)0x83) != 0x00) && (INT != 0x00))

/**
  * @}
  */


/** @defgroup GPT_Common_DMA_Sources
  * @{
  */

#define GPT_COMMON_DMA_ETIP                 (uint8_t)0x04
#define GPT_COMMON_DMA_ETIN                 (uint8_t)0x08
#define IS_GPT_COMMON_DMA_SOURCE(SOURCE)    (((SOURCE & 0xF3) == 0x00) && (SOURCE != 0x00))

/**
  * @}
  */


/** @defgroup GPT_Common_Flags
  * @{
  */

#define GPT_COMMON_FLAG_ETIP              (uint8_t)0x01
#define GPT_COMMON_FLAG_ETIN              (uint8_t)0x02
#define GPT_COMMON_FLAG_GTPOE             (uint8_t)0x04

#define IS_GPT_COMMON_FLAG(FLAG)          (((FLAG & 0x07) != 0x00) && (FLAG != 0x00))

#define IS_GPT_COMMON_GET_ONE_FLAG(FLAG)   ((FLAG == GPT_COMMON_FLAG_ETIP) \
                                           || (FLAG == GPT_COMMON_FLAG_ETIN) \
                                           || (FLAG == GPT_COMMON_FLAG_GTPOE))

/**
  * @}
  */


/** @defgroup GPT_Counter_Mode
  * @{
  */

#define GPT_CounterMode_Up           (uint16_t)0x0002
#define GPT_CounterMode_Down         (uint16_t)0x0003
#define GPT_CounterMode_Center       (uint16_t)0x0004
#define IS_GPT_COUNTER_MODE(MODE)    ((MODE == GPT_CounterMode_Up) \
	                                 || (MODE == GPT_CounterMode_Down) \
	                                 || (MODE == GPT_CounterMode_Center))

/**
  * @}
  */


/** @defgroup GPT_Wave_Mode
  * @{
  */

#define GPT_WaveMode_Sawtooth               (uint16_t)0x0000
#define GPT_WaveMode_SawtoothOnePulse       (uint16_t)0x0001
#define GPT_WaveMode_Triangular_PWM1        (uint16_t)0x0004
#define GPT_WaveMode_Triangular_PWM2        (uint16_t)0x0005
#define GPT_WaveMode_Triangular_PWM3        (uint16_t)0x0006
#define GPT_WaveMode_Triangular_PWM4        (uint16_t)0x0007
#define IS_GPT_WAVE_MODE(MODE)              ((MODE == GPT_WaveMode_Sawtooth) \
	                                        || (MODE == GPT_WaveMode_SawtoothOnePulse) \
	                                        || (MODE == GPT_WaveMode_Triangular_PWM1) \
	                                        || (MODE == GPT_WaveMode_Triangular_PWM2) \
	                                        || (MODE == GPT_WaveMode_Triangular_PWM3) \
	                                        || (MODE == GPT_WaveMode_Triangular_PWM4))

/**
  * @}
  */



/** @defgroup GPT_Start_Level
  * @{
  */

#define GPT_StartLevel_Low                  (uint16_t)0x0000
#define GPT_StartLevel_High                 (uint16_t)0x0010
#define IS_GPT_START_LEVEL(LEVEL)           ((LEVEL == GPT_StartLevel_Low) \
	                                        || (LEVEL == GPT_StartLevel_High))

/**
  * @}
  */


/** @defgroup GPT_Period_Status
  * @{
  */

#define GPT_PeriodStatus_Keep              (uint16_t)0x0000
#define GPT_PeriodStatus_Low               (uint16_t)0x0004
#define GPT_PeriodStatus_High              (uint16_t)0x0008
#define GPT_PeriodStatus_Toggle            (uint16_t)0x000C
#define IS_GPT_PERIOD_STATUS(STATUS)       ((STATUS == GPT_PeriodStatus_Keep) \
	                                       || (STATUS == GPT_PeriodStatus_Low) \
	                                       || (STATUS == GPT_PeriodStatus_High) \
	                                       || (STATUS == GPT_PeriodStatus_Toggle))

/**
  * @}
  */





/** @defgroup GPT_Compare_Status
  * @{
  */

#define GPT_CompareStatus_Keep          (uint16_t)0x0000
#define GPT_CompareStatus_Low           (uint16_t)0x0001
#define GPT_CompareStatus_High          (uint16_t)0x0002
#define GPT_CompareStatus_Toggle        (uint16_t)0x0003
#define IS_GPT_COMPARE_STATUS(STATUS)   ((STATUS == GPT_CompareStatus_Keep) \
	                                    || (STATUS == GPT_CompareStatus_Low) \
	                                    || (STATUS == GPT_CompareStatus_High) \
	                                    || (STATUS == GPT_CompareStatus_Toggle))

/**
  * @}
  */

/** @defgroup GPT_Hardware_Action
  * @{
  */

#define GPT_HardwareAction_None              (uint16_t)0x0000
#define GPT_HardwareAction_Rising            (uint16_t)0x0001
#define GPT_HardwareAction_Falling           (uint16_t)0x0002
#define GPT_HardwareAction_RisingFalling     (uint16_t)0x0003
#define IS_GPT_HARDWARE_ACTION(ACTION)       ((ACTION == GPT_HardwareAction_None) \
                                             || (ACTION == GPT_HardwareAction_Rising) \
                                             || (ACTION == GPT_HardwareAction_Falling) \
                                             || (ACTION == GPT_HardwareAction_RisingFalling))

/**
  * @}
  */

/** @defgroup GPT_Counter_Clear_Sources
  * @{
  */

#define GPT_CounterClearSource_None          (uint16_t)0x0000
#define GPT_CounterClearSource_CCRA          (uint16_t)0x1000
#define GPT_CounterClearSource_CCRB          (uint16_t)0x2000
#define GPT_CounterClearSource_GPT0          (uint16_t)0x3000
#define GPT_CounterClearSource_GPT1          (uint16_t)0x3040
#define GPT_CounterClearSource_GPT2          (uint16_t)0x3080
#define GPT_CounterClearSource_GPT3          (uint16_t)0x30C0
#define IS_GPT_COUNTER_CLEAR_SOURCE(SOURCE)  ((SOURCE == GPT_CounterClearSource_None) \
                                             || (SOURCE == GPT_CounterClearSource_CCRA) \
                                             || (SOURCE == GPT_CounterClearSource_CCRB) \
                                             || (SOURCE == GPT_CounterClearSource_GPT0) \
                                             || (SOURCE == GPT_CounterClearSource_GPT1) \
                                             || (SOURCE == GPT_CounterClearSource_GPT2) \
                                             || (SOURCE == GPT_CounterClearSource_GPT3))

/**
  * @}
  */


/** @defgroup GPT_Channel
  * @{
  */

#define GPT_Channel_A                    (uint16_t)0x0001
#define GPT_Channel_B                    (uint16_t)0x0002
#define IS_GPT_CHANNEL(CHANNEL)          (((CHANNEL & (uint16_t)0xFFFC) == 0x00) && (CHANNEL != 0x00))

#define IS_GPT_GET_ONE_CHANNEL(CHANNEL)  ((CHANNEL == GPT_Channel_A) || (CHANNEL == GPT_Channel_B))

/**
  * @}
  */

/** @defgroup GPT_Channel
  * @{
  */

#define GPT_StopLevel_Low          (uint16_t)0x0000
#define GPT_StopLevel_High         (uint16_t)0x0040
#define IS_GPT_STOP_LEVEL(LEVEL)   ((LEVEL == GPT_StopLevel_Low) \
                                   || (LEVEL == GPT_StopLevel_High))
/**
  * @}
  */


/** @defgroup GPT_Channel
  * @{
  */

#define GPT_Retain_Register        (uint16_t)0x0000
#define GPT_Retain_Keep            (uint16_t)0x0080
#define IS_GPT_RETAIN(RETAIN)      ((RETAIN == GPT_Retain_Register) \
                                   || (RETAIN == GPT_Retain_Keep))

/**
  * @}
  */


/** @defgroup GPT_Input_Capture_Polarity
  * @{
  */

#define GPT_ICPolarity_Rising         (uint16_t)0x0000
#define GPT_ICPolarity_Falling        (uint16_t)0x0001
#define GPT_ICPolarity_BothEdge       (uint16_t)0x0002
#define IS_GPT_IC_POLARITY(POLARITY)  ((POLARITY == GPT_ICPolarity_Rising) \
                                      || (POLARITY == GPT_ICPolarity_Falling) \
                                      || (POLARITY == GPT_ICPolarity_BothEdge))

/**
  * @}
  */


/** @defgroup GPT_Input_Capture_Selection
  * @{
  */

#define GPT_ICSelection_Pin          (uint16_t)0x0000
#define GPT_ICSelection_RXD1         (uint16_t)0x0004
#define GPT_ICSelection_RXD2         (uint16_t)0x0008
#define GPT_ICSelection_RXD3         (uint16_t)0x000C
#define GPT_ICSelection_AB           (uint16_t)0x0004   
#define GPT_ICSelection_A1B1           (uint16_t)0x0008
#define GPT_ICSelection_A2B2           (uint16_t)0x0010
#define IS_GPT_IC_SELECT(PIN)        ((PIN == GPT_ICSelection_Pin) \
                                     || (PIN == GPT_ICSelection_RXD1) \
                                     || (PIN == GPT_ICSelection_RXD2) \
                                     || (PIN == GPT_ICSelection_RXD3) \
                                     || (PIN == GPT_ICSelection_AB) \
                                     || (PIN == GPT_ICSelection_A1B1) \
                                     || (PIN == GPT_ICSelection_A2B2))

/**
  * @}
  */

  
/** @defgroup GPT_Input_Capture_Filter
  * @{
  */

#define GPT_ICFilter_None           (uint16_t)0x0000
#define GPT_ICFilter_30HCLK         (uint16_t)0x0001
#define GPT_ICFilter_60HCLK         (uint16_t)0x0002
#define GPT_ICFilter_120HCLK         (uint16_t)0x0003
#define GPT_ICFilter_180HCLK         (uint16_t)0x0004
#define GPT_ICFilter_240HCLK         (uint16_t)0x0005
#define GPT_ICFilter_300HCLK         (uint16_t)0x0006
#define GPT_ICFilter_360HCLK         (uint16_t)0x0007
#define GPT_ICFilter_480HCLK         (uint16_t)0x0008
#define GPT_ICFilter_600HCLK         (uint16_t)0x0009
#define GPT_ICFilter_720HCLK         (uint16_t)0x000A
#define GPT_ICFilter_960HCLK         (uint16_t)0x000B
#define GPT_ICFilter_1200HCLK         (uint16_t)0x000C
#define GPT_ICFilter_1440HCLK         (uint16_t)0x000D
#define GPT_ICFilter_1680HCLK         (uint16_t)0x000E
#define GPT_ICFilter_1920HCLK         (uint16_t)0x000F
#define IS_GPT_IC_Filter(Filter)      ((Filter == GPT_ICFilter_None) \
                                      || (Filter == GPT_ICFilter_30HCLK) \
                                      || (Filter == GPT_ICFilter_60HCLK) \
                                      || (Filter == GPT_ICFilter_120HCLK) \
                                      || (Filter == GPT_ICFilter_180HCLK) \
                                      || (Filter == GPT_ICFilter_240HCLK) \
                                      || (Filter == GPT_ICFilter_300HCLK) \
                                      || (Filter == GPT_ICFilter_360HCLK) \
                                      || (Filter == GPT_ICFilter_480HCLK) \
                                      || (Filter == GPT_ICFilter_600HCLK) \
                                      || (Filter == GPT_ICFilter_720HCLK) \
                                      || (Filter == GPT_ICFilter_960HCLK) \
                                      || (Filter == GPT_ICFilter_1200HCLK) \
                                      || (Filter == GPT_ICFilter_1440HCLK) \
                                      || (Filter == GPT_ICFilter_1680HCLK) \
                                      || (Filter == GPT_ICFilter_1920HCLK))

/**
  * @}
  */



/** @defgroup GPT_Flags
  * @{
  */

#define GPT_FLAG_CCRA                   (uint16_t)0x0001
#define GPT_FLAG_CCRB                   (uint16_t)0x0002
#define GPT_FLAG_OV                     (uint16_t)0x0004
#define GPT_FLAG_UD                     (uint16_t)0x0008
#define GPT_FLAG_DTE                    (uint16_t)0x0010
#define GPT_FLAG_OS                     (uint16_t)0x0020

#define IS_GPT_FLAG(FLAG)               (((FLAG & (uint16_t)0x3F) != 0x00) && (FLAG != 0x00))

#define IS_GPT_GET_ONE_FLAG(FLAG)       ((FLAG == GPT_FLAG_CCRA) \
                                        || (FLAG == GPT_FLAG_CCRB) \
                                        || (FLAG == GPT_FLAG_OV) \
                                        || (FLAG == GPT_FLAG_UD) \
                                        || (FLAG == GPT_FLAG_DTE) \
                                        || (FLAG == GPT_FLAG_OS))

/**
  * @}
  */


/** @defgroup GPT_Interrupt_Sources
  * @{
  */

#define GPT_INT_CCRA                  (uint16_t)0x0001
#define GPT_INT_CCRB                  (uint16_t)0x0002
#define GPT_INT_OV                    (uint16_t)0x0004
#define GPT_INT_UD                    (uint16_t)0x0008
#define GPT_INT_OVUD                  (uint16_t)0x000C
#define GPT_INT_DTE                   (uint16_t)0x0010
#define GPT_INT_OS                    (uint16_t)0x0020
#define IS_GPT_INT(INT)               (((INT & (uint16_t)0xFFC0) == 0x00) && (INT != 0x00))

/**
  * @}
  */



 

/** @defgroup GPT_DMA_Sources
  * @{
  */

#define GPT_DMA_CCRA                 (uint16_t)0x0001
#define GPT_DMA_CCRB                 (uint16_t)0x0002
#define GPT_DMA_OV                   (uint16_t)0x0004
#define GPT_DMA_UD                   (uint16_t)0x0008
#define GPT_DMA_OVUD                 (uint16_t)0x000C
#define IS_GPT_DMA_SOURCE(SOURCE)    (((SOURCE & (uint16_t)0xFFF0) == 0x00) && (SOURCE != 0x00))

/**
  * @}
  */






/** @defgroup GPT_Period_Buffer 
  * @{
  */ 

#define GPT_PeriodBuf_None         (uint16_t)0x0000
#define GPT_PeriodBuf_Single       (uint16_t)0x0010
#define GPT_PeriodBuf_Double       (uint16_t)0x0020
#define IS_GPT_PERIOD_BUF(CMD)     ((CMD == GPT_PeriodBuf_None) \
                                   || (CMD == GPT_PeriodBuf_Single) \
                                   || (CMD == GPT_PeriodBuf_Double))

/**
  * @}
  */

/** @defgroup GPT_CCRA_Buffer 
  * @{
  */ 

#define GPT_CCRABuf_None                 (uint16_t)0x0000
#define GPT_CCRABuf_Single               (uint16_t)0x0001
#define GPT_CCRABuf_Double               (uint16_t)0x0002
#define IS_GPT_CC_CHANNEL_A_BUF(CMD)     ((CMD == GPT_CCRABuf_None) \
                                         || (CMD == GPT_CCRABuf_Single) \
                                         || (CMD == GPT_CCRABuf_Double))
    
/**
  * @}
  */

/** @defgroup GPT_CCRB_Buffer 
  * @{
  */ 

#define GPT_CCRBBuf_None                  (uint16_t)0x0000
#define GPT_CCRBBuf_Single                (uint16_t)0x0004
#define GPT_CCRBBuf_Double                (uint16_t)0x0008
#define IS_GPT_CC_CHANNEL_B_BUF(CMD)      ((CMD == GPT_CCRBBuf_None) \
                                          || (CMD == GPT_CCRBBuf_Single) \
                                          || (CMD == GPT_CCRBBuf_Double))

/**
  * @}
  */

/** @defgroup GPT_Trigger_ADC_Source 
  * @{
  */ 

#define GPT_TriggerADCSourceA_Rising          (uint16_t)0x0100
#define GPT_TriggerADCSourceA_Falling         (uint16_t)0x0200
#define GPT_TriggerADCSourceB_Rising          (uint16_t)0x0400
#define GPT_TriggerADCSourceB_Falling         (uint16_t)0x0800
#define IS_GPT_TRIGGER_ADC_SOURCE(SOURCE)     (((SOURCE & (uint16_t)0xF0FF) == 0x00) && (SOURCE != 0x00))

/**
  * @}
  */

/** @defgroup GPT_Trigger_ADC_Buffer 
  * @{
  */ 

#define GPT_TriggerADCBuf_None            (uint16_t)0x0004
#define GPT_TriggerADCBuf_Single          (uint16_t)0x0000
#define GPT_TriggerADCBuf_Double          (uint16_t)0x0400
#define IS_GPT_TRIGGER_ADC_BUF(CMD)       ((CMD == GPT_TriggerADCBuf_None) \
                                          || (CMD == GPT_TriggerADCBuf_Single) \
                                          || (CMD == GPT_TriggerADCBuf_Double))

/**
  * @}
  */

/** @defgroup GPT_Trigger_ADC_Timing 
  * @{
  */ 

#define GPT_TriggerADCTiming_None         (uint16_t)0x0000
#define GPT_TriggerADCTiming_Peak         (uint16_t)0x0100
#define GPT_TriggerADCTiming_Trough       (uint16_t)0x0200
#define GPT_TriggerADCTiming_Both         (uint16_t)0x0300
#define IS_GPT_TRIGGER_ADC_TIMING(TIMING) ((TIMING == GPT_TriggerADCTiming_None) \
                                          || (TIMING == GPT_TriggerADCTiming_Peak) \
                                          || (TIMING == GPT_TriggerADCTiming_Trough) \
                                          || (TIMING == GPT_TriggerADCTiming_Both))

/**
  * @}
  */

/** @defgroup GPT_Interrupt_Reduction_Mode 
  * @{
  */ 

#define GPT_IntReductionMode_None         (uint16_t)0x0000
#define GPT_IntReductionMode_Peak         (uint16_t)0x0010
#define GPT_IntReductionMode_Trough       (uint16_t)0x0020
#define GPT_IntReductionMode_Both         (uint16_t)0x0030
#define IS_GPT_INT_REDUCTION_MODE(MODE)   ((MODE == GPT_IntReductionMode_None) \
                                          || (MODE == GPT_IntReductionMode_Peak) \
                                          || (MODE == GPT_IntReductionMode_Trough) \
                                          || (MODE == GPT_IntReductionMode_Both))

/**
  * @}
  */

/** @defgroup GPT_Interrupt_Reduction_Times 
  * @{
  */ 

#define GPT_IntReductionTimes_0             (uint16_t)0x0000
#define GPT_IntReductionTimes_1             (uint16_t)0x0100
#define GPT_IntReductionTimes_2             (uint16_t)0x0200
#define GPT_IntReductionTimes_3             (uint16_t)0x0300
#define GPT_IntReductionTimes_4             (uint16_t)0x0400
#define GPT_IntReductionTimes_5             (uint16_t)0x0500
#define GPT_IntReductionTimes_6             (uint16_t)0x0600
#define GPT_IntReductionTimes_7             (uint16_t)0x0700
#define IS_GPT_INT_REDUCTION_TIMES(TIMES)   ((TIMES == GPT_IntReductionTimes_0) \
                                            || (TIMES == GPT_IntReductionTimes_1) \
                                            || (TIMES == GPT_IntReductionTimes_2) \
                                            || (TIMES == GPT_IntReductionTimes_3) \
                                            || (TIMES == GPT_IntReductionTimes_4) \
                                            || (TIMES == GPT_IntReductionTimes_5) \
                                            || (TIMES == GPT_IntReductionTimes_6) \
                                            || (TIMES == GPT_IntReductionTimes_7))

/**
  * @}
  */

/** @defgroup GPT_Interrupt_Reduction_Times 
  * @{
  */ 

#define GPT_AssociateInt_CCRA                 (uint16_t)0x0001
#define GPT_AssociateInt_CCRB                 (uint16_t)0x0002
#define GPT_AssociateInt_TriggerADC_A         (uint16_t)0x1000
#define GPT_AssociateInt_TriggerADC_B         (uint16_t)0x4000
#define IS_GPT_ASSOCIATE_INT_SOURCE(SOURCE)   (((SOURCE & (uint16_t)0xAFFC) == 0x00) && (SOURCE != 0x00))

/**
  * @}
  */

  
/** @defgroup GPT_Dead_Time_DVD_Auto_Set
  * @{
  */ 

#define GPT_AutoSetDVD_Enable         (uint16_t)0x0100
#define GPT_AutoSetDVD_Disable        (uint16_t)0x0000
#define IS_GPT_AUTO_SET_DVD_CMD(CMD)  ((CMD == GPT_AutoSetDVD_Enable) \
                                      || (CMD == GPT_AutoSetDVD_Disable)) 
/**
  * @}
  */  
  
/** @defgroup GPT_Dead_Time_GTDVU_Buffer 
  * @{
  */ 

#define GPT_GTDVUBuf_Enable           (uint16_t)0x0010  
#define GPT_GTDVUBuf_Disable          (uint16_t)0x0000
#define IS_GPT_GTDVU_BUFFER_CMD(CMD)  ((CMD == GPT_GTDVUBuf_Enable) \
                                      || (CMD == GPT_GTDVUBuf_Disable))
/**
  * @}
  */

/** @defgroup GPT_Dead_Time_GTDVD_Buffer 
  * @{
  */ 

#define GPT_GTDVDBuf_Enable           (uint16_t)0x0020  
#define GPT_GTDVDBuf_Disable          (uint16_t)0x0000
#define IS_GPT_GTDVD_BUFFER_CMD(CMD)  ((CMD == GPT_GTDVDBuf_Enable) \
                                      || (CMD == GPT_GTDVDBuf_Disable))
/**
  * @}
  */



/** @defgroup GPT_ChannelA_Output_Short_Circuit_Protection 
  * @{
  */ 

#define GPT_CCRA_OS_Low               (uint8_t)0x00
#define GPT_CCRA_OS_High              (uint8_t)0x01
#define IS_GPT_CCRA_OS_LEVEL(LEVEL)   ((LEVEL == GPT_CCRA_OS_Low) \
                                      || (LEVEL == GPT_CCRA_OS_High))

/**
  * @}
  */  

/** @defgroup GPT_ChannelB_Output_Short_Circuit_Protection 
  * @{
  */ 

#define GPT_CCRB_OS_Low               (uint8_t)0x00
#define GPT_CCRB_OS_High              (uint8_t)0x02
#define IS_GPT_CCRB_OS_LEVEL(LEVEL)   ((LEVEL == GPT_CCRB_OS_Low) \
                                      || (LEVEL == GPT_CCRB_OS_High))
                                      
/**
  * @}
  */

/**
  * @}
  */ 


/* Exported macro ------------------------------------------------------------*/

/** @defgroup GPT_Clock_Division
  * @{
  */



/**
  * @}
  */



/* Exported functions --------------------------------------------------------*/ 

/** @defgroup GPT_Exported_Functions
  * @{
  */

void GPT_CommonWriteProtect(FunctionalState NewState);
void GPT_CommonExternalInputLineFilter(uint8_t GPT_CLKDiv, uint8_t GPT_SampleNum);
void GPT_CommonPortCtrlLineFilter(uint8_t GPT_CommonFilter);
void GPT_CommonPortCtrlHizConfig(uint8_t GPT_CommonHiz, FunctionalState NewState);
void GPT_CommonINTConfig(uint8_t GPT_CommonIT, FunctionalState NewState);
void GPT_CommonDMAConfig(uint8_t GPT_CommonDMASource, FunctionalState NewState);
FlagStatus GPT_CommonGetFlagStatus(uint8_t GPT_CommonFlag);
void GPT_CommonClearFlag(uint8_t GPT_CommonFlag);
void GPT_WriteProtect(GPT_TypeDef* GPTx, FunctionalState NewState);
void GPT_OnOff(GPT_TypeDef* GPTx, CmdState OnOffState);
void GPT_TimeBaseInit(GPT_TypeDef* GPTx, GPT_TimeBaseInitTypeDef* GPT_TimeBaseInitStruct);
void GPT_OCAInit(GPT_TypeDef* GPTx, GPT_OCInitTypeDef* GPT_OCInitStruct);
void GPT_OCBInit(GPT_TypeDef* GPTx, GPT_OCInitTypeDef* GPT_OCInitStruct);
void GPT_OCStartStopRetainConfig(GPT_TypeDef* GPTx, uint16_t GPT_Channel, uint16_t GPT_Retain);
void GPT_OCStopLevelConfig(GPT_TypeDef* GPTx, uint16_t GPT_Channel, uint16_t GPT_StopLevel);
void GPT_ICInit(GPT_TypeDef* GPTx, GPT_ICInitTypeDef* GPT_ICInitStruct);
void GPT_CascadeConfig(GPT_TypeDef* GPTx, FunctionalState NewState);
void GPT_OutputHizConfig(GPT_TypeDef* GPTx, FunctionalState NewState);
void GPT_CascadePinAssociation(GPT_TypeDef* GPTx, uint16_t GPT_Channel, FunctionalState NewState);
void GPT_StartCounterWithHardware(GPT_TypeDef* GPTx, uint16_t GPT_HardwareAction);
void GPT_StopCounterWithHardware(GPT_TypeDef* GPTx, uint16_t GPT_HardwareAction);
void GPT_ClearCounterWithHardware(GPT_TypeDef* GPTx, uint16_t GPT_HardwareAction);
void GPT_SoftwareClearCounter(GPT_TypeDef* GPTx);
void GPT_SelectCounterClearSource(GPT_TypeDef* GPTx, uint16_t GPT_CounterClearSource);
void GPT_PeriodBufConfig(GPT_TypeDef* GPTx, uint16_t GPT_PeriodBufCmd, uint16_t GPT_PeriodSingleBuf, uint16_t GPT_PeriodDoubleBuf);
uint16_t GPT_GetPeriodSingleBuffer(GPT_TypeDef* GPTx);
uint16_t GPT_GetPeriodDoubleBuffer(GPT_TypeDef* GPTx);
void GPT_CCBufConfig(GPT_TypeDef* GPTx, uint16_t GPT_CCRABufCmd, uint16_t GPT_CCRBBufCmd);
void GPT_SetCompareASingleBuf(GPT_TypeDef* GPTx, uint16_t CompareASingleBuf);
void GPT_SetCompareADoubleBuf(GPT_TypeDef* GPTx, uint16_t CompareADoubleBuf);
void GPT_SetCompareBSingleBuf(GPT_TypeDef* GPTx, uint16_t CompareBSingleBuf);
void GPT_SetCompareBDoubleBuf(GPT_TypeDef* GPTx, uint16_t CompareBDoubleBuf);
uint16_t GPT_GetCaptureA(GPT_TypeDef* GPTx);
uint16_t GPT_GetCaptureASingleBuf(GPT_TypeDef* GPTx);
uint16_t GPT_GetCaptureADoubleBuf(GPT_TypeDef* GPTx);
uint16_t GPT_GetCompareB(GPT_TypeDef* GPTx);
uint16_t GPT_GetCompareBSingleBuf(GPT_TypeDef* GPTx);
uint16_t GPT_GetCompareBDoubleBuf(GPT_TypeDef* GPTx);
void GPT_ForceBufferOperation(GPT_TypeDef* GPTx);
void GPT_TriggerADCConfig(GPT_TypeDef* GPTx, uint16_t GPT_TriggerSource, FunctionalState NewState);
void GPT_TriggerADCSourceABufConfig(GPT_TypeDef* GPTx, uint16_t GPT_TriggerABufCmd, uint16_t GPT_TriggerATiming);
void GPT_TriggerADCSourceBBufConfig(GPT_TypeDef* GPTx, uint16_t GPT_TriggerBBufCmd, uint16_t GPT_TriggerBTiming);
void GPT_SetTriggerADCValue(GPT_TypeDef* GPTx, uint16_t GPT_Channel, uint16_t Value);
void GPT_SetTriggerADCSingleBufValue(GPT_TypeDef* GPTx, uint16_t GPT_Channel, uint16_t SingleBufValue);
void GPT_SetTriggerADCDoubleBufValue(GPT_TypeDef* GPTx, uint16_t GPT_Channel, uint16_t DoubleBufValue);
void GPT_IntReductionConfig(GPT_TypeDef* GPTx, uint16_t GPT_ReductionMode, uint16_t GPT_ReductionTimes);
void GPT_AssociateIntReductionConfig(GPT_TypeDef* GPTx, uint16_t GPT_AssociateFunc, FunctionalState NewState);
uint8_t GPT_GetIntReductionTimes(GPT_TypeDef* GPTx);
void GPT_INTConfig(GPT_TypeDef* GPTx, uint16_t GPT_IT, FunctionalState NewState);
void GPT_DMAConfig(GPT_TypeDef* GPTx, uint16_t GPT_DMASource, FunctionalState NewState);
FlagStatus GPT_GetFlagStatus(GPT_TypeDef* GPTx, uint16_t GPT_Flag);
void GPT_ClearFlag(GPT_TypeDef* GPTx, uint16_t GPT_Flag);
void GPT_DeadTimeInit(GPT_TypeDef* GPTx, GPT_DeadTimeInitTypeDef * GPT_DeadTimeInitStruct);
void GPT_DeadTimeValue(GPT_TypeDef* GPTx, uint16_t GTDVU, uint16_t GTDVD);
void GPT_DeadTimeBufferConfig(GPT_TypeDef* GPTx, uint16_t GPT_GTDVUBufCmd, uint16_t GPT_GTDVDBufCmd);
void GPT_DeadTimeBufferValue(GPT_TypeDef* GPTx, uint16_t GTDBU, uint16_t GTDBD);
void GPT_DeadTimeAutoSetRegConfig(GPT_TypeDef* GPTx, FunctionalState NewState);
void GPT_DeadTimeConfig(GPT_TypeDef* GPTx, FunctionalState NewState);
void GPT_OutputShortCircuitConfig(GPT_TypeDef* GPTx, uint8_t GPT_CCRALevel, uint8_t GPT_CCRBLevel);
void GPT_OutputShortCircuitOnOff(GPT_TypeDef* GPTx, CmdState OnOffState);



/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_GPT_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/

