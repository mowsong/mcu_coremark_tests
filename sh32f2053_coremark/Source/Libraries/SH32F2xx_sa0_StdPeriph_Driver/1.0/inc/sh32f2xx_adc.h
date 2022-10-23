/**
  ******************************************************************************
  * @file    sh32f2xx_adc.h
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
#ifndef __SH32F2xx_ADC_H
#define __SH32F2xx_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup ADC_MODULE
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/

/** @defgroup ADC_Exported_Types
  * @{
  */

typedef struct
{
    uint16_t ADC_Resolution;      /* Configure the ADC_Resolution Mode
                                     This parameter can be a value of ADC_Resolution */
    
    uint16_t ADC_Trigger;         /* Configure the ADC external trigger sources
                                     This parameter can be a value of ADC_Trigger */
    
    uint16_t ADC_RefVoltage;      /* Select the ADC reference voltage sources
                                     This parameter can be a value of ADC_Reference_Voltage_Sources */
    
    uint16_t ADC_ConversionMode;  /* Configure the ADC conversion Mode
                                     This parameter can be a value of ADC_Conversion_Mode */
    
    uint32_t ADC_SequencePointer;  /* Set the ADC channel pointer to select which channel to be converted.
                                     This parameter can be a value of ADC_Channel_Pointer */
    
    uint32_t ADC_NumOfConversion; /* Configure the ADC conversion channel numbers
                                     This parameter can be 1 to 8 */
    
    uint32_t ADC_Prescaler;       /* Configure the ADC clock prescaler
                                     This parameter can be a value of ADC_Prescaler */
    
    uint32_t ADC_TwoSampleGap;  /* Configure the ADC two sample gap time
                                     This parameter can be a value of ADC_Two_Sample_Gap */
                                     
    uint32_t ADC_SampleTime;      /* Configure the ADC sample time
                                     This parameter can be a value of ADC_Sample_Times */
}ADC_InitTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup ADC_Exported_Constants
  * @{
  */ 

  

#define IS_ADC_ALL_PERIPH(PERIPH)  ((PERIPH == ADC1) \
                                   || (PERIPH == ADC2) \
                                   || (PERIPH == ADC3))

/** @defgroup ADC_Resolution
  * @{
  */
  
#define ADC_Resolution_12Bit              (uint16_t)0x0000
#define ADC_Resolution_10Bit              (uint16_t)0x0008
#define IS_ADC_RESOLUTION(RESOLUTION)     ((RESOLUTION == ADC_Resolution_12Bit) \
                                          || (RESOLUTION == ADC_Resolution_10Bit))  

/**
  * @}
  */

/** @defgroup ADC_Trigger
  * @{
  */
  
#define ADC_Trigger_Software              (uint16_t)0x0000
#define ADC_Trigger_ADTRG1                (uint16_t)0x0100
#define ADC_Trigger_ADTRG2                (uint16_t)0x0200
#define ADC_Trigger_ADTRG3                (uint16_t)0x0400
#define ADC_Trigger_TIM8                  (uint16_t)0x1000
#define ADC_Trigger_MCM1TRG1              (uint16_t)0x2100
#define ADC_Trigger_MCM1TRG2              (uint16_t)0x2200
#define ADC_Trigger_MCM1TRG3              (uint16_t)0x2400
#define ADC_Trigger_MCM1TRG4              (uint16_t)0x2800
#define ADC_Trigger_MCM2TRG1              (uint16_t)0x3100
#define ADC_Trigger_MCM2TRG2              (uint16_t)0x3200
#define ADC_Trigger_MCM2TRG3              (uint16_t)0x3400
#define ADC_Trigger_MCM2TRG4              (uint16_t)0x3800
#define ADC_Trigger_GPT0ADTRA             (uint16_t)0x4100
#define ADC_Trigger_GPT0ADTRB             (uint16_t)0x4200
#define ADC_Trigger_GPT1ADTRA             (uint16_t)0x5100
#define ADC_Trigger_GPT1ADTRB             (uint16_t)0x5200
#define ADC_Trigger_GPT2ADTRA             (uint16_t)0x6100
#define ADC_Trigger_GPT2ADTRB             (uint16_t)0x6200
#define ADC_Trigger_GPT3ADTRA             (uint16_t)0x7100
#define ADC_Trigger_GPT3ADTRB             (uint16_t)0x7200
#define IS_ADC_TRIGGER(TRIGGER)           ((TRIGGER & (uint16_t)0x80FF) == 0x00)

#define IS_ADC_ADTRG_TRIGGER(TRIGGER)     (((TRIGGER & (uint16_t)0xF8FF) == 0x00) && (TRIGGER != 0x00))
                                          
#define IS_ADC_TIM8_TRIGGER(TRIGGER)       (TRIGGER == ADC_Trigger_TIM8)      

#define IS_ADC_MCM1_TRIGGER(TRIGGER)      (((TRIGGER & (uint16_t)0xD0FF) == 0x00) && (TRIGGER != 0x00))

#define IS_ADC_MCM2_TRIGGER(TRIGGER)      (((TRIGGER & (uint16_t)0xC0FF) == 0x00) && (TRIGGER != 0x00))

#define IS_ADC_GPT0_TRIGGER(TRIGGER)      (((TRIGGER & (uint16_t)0xBCFF) == 0x00) && (TRIGGER != 0x00))

#define IS_ADC_GPT1_TRIGGER(TRIGGER)      (((TRIGGER & (uint16_t)0xACFF) == 0x00) && (TRIGGER != 0x00))

#define IS_ADC_GPT2_TRIGGER(TRIGGER)      (((TRIGGER & (uint16_t)0x9CFF) == 0x00) && (TRIGGER != 0x00))

#define IS_ADC_GPT3_TRIGGER(TRIGGER)      (((TRIGGER & (uint16_t)0x8CFF) == 0x00) && (TRIGGER != 0x00))

/**
  * @}
  */


/** @defgroup ADC_Reference_Voltage_Sources
  * @{
  */

#define ADC_RefVoltage_AVDD             (uint16_t)0x0000
#define ADC_RefVoltage_ExtVref          (uint16_t)0x0010
#define ADC_RefVoltage_InterVref        (uint16_t)0x0020
#define IS_ADC_REF_VOLTAGE(VOLTAGE)     ((VOLTAGE == ADC_RefVoltage_AVDD) \
                                        || (VOLTAGE == ADC_RefVoltage_ExtVref) \
                                        || (VOLTAGE == ADC_RefVoltage_InterVref))  
  
/**
  * @}
  */


/** @defgroup ADC_Conversion_Mode
  * @{
  */
  
#define ADC_ConversionMode_Single            (uint16_t)0x0000
#define ADC_ConversionMode_Discontinuous     (uint16_t)0x0002
#define ADC_ConversionMode_Continuous        (uint16_t)0x0004
#define IS_ADC_CONVERSION_MODE(MODE)         ((MODE == ADC_ConversionMode_Single) \
                                             || (MODE == ADC_ConversionMode_Discontinuous) \
                                             || (MODE == ADC_ConversionMode_Continuous))  
                                             
/**
  * @}
  */


/** @defgroup ADC_Conversion_Numbers
  * @{
  */
  
#define IS_ADC_CONVERSION_NUMBER(NUMBER)   ((NUMBER >= 0x01) && (NUMBER <= 0x08))  
  
/**
  * @}
  */

                                        
/** @defgroup ADC_Prescaler
  * @{
  */
  
#define ADC_Prescaler_1               (uint32_t)0x0000
#define ADC_Prescaler_2               (uint32_t)0x0001
#define ADC_Prescaler_3               (uint32_t)0x0002
#define ADC_Prescaler_4               (uint32_t)0x0003
#define ADC_Prescaler_5               (uint32_t)0x0004
#define ADC_Prescaler_6               (uint32_t)0x0005
#define ADC_Prescaler_8               (uint32_t)0x0006
#define ADC_Prescaler_12               (uint32_t)0x0007
#define ADC_Prescaler_16               (uint32_t)0x0008
#define ADC_Prescaler_24               (uint32_t)0x0009
#define ADC_Prescaler_32               (uint32_t)0x000A
#define ADC_Prescaler_48               (uint32_t)0x000B
#define ADC_Prescaler_64               (uint32_t)0x000C
#define ADC_Prescaler_128               (uint32_t)0x000D
#define ADC_Prescaler_256               (uint32_t)0x000E
#define ADC_Prescaler_320               (uint32_t)0x000F
#define IS_ADC_CLK_PRESCALER(DIV)       ((DIV == ADC_Prescaler_1) \
                                        || (DIV == ADC_Prescaler_2) \
                                        || (DIV == ADC_Prescaler_3) \
                                        || (DIV == ADC_Prescaler_4) \
                                        || (DIV == ADC_Prescaler_5) \
                                        || (DIV == ADC_Prescaler_6) \
                                        || (DIV == ADC_Prescaler_8) \
                                        || (DIV == ADC_Prescaler_12) \
                                        || (DIV == ADC_Prescaler_16) \
                                        || (DIV == ADC_Prescaler_24) \
                                        || (DIV == ADC_Prescaler_32) \
                                        || (DIV == ADC_Prescaler_48) \
                                        || (DIV == ADC_Prescaler_64) \
                                        || (DIV == ADC_Prescaler_128) \
                                        || (DIV == ADC_Prescaler_256) \
                                        || (DIV == ADC_Prescaler_320)) 
  
/**
  * @}
  */

                                        
                                        
/** @defgroup ADC_Two_Sample_Gap
  * @{
  */

#define ADC_TwoSampleGap_0Cycles       (uint32_t)0x00000
#define ADC_TwoSampleGap_2Cycles       (uint32_t)0x10000
#define ADC_TwoSampleGap_4Cycles       (uint32_t)0x20000
#define ADC_TwoSampleGap_8Cycles       (uint32_t)0x30000
#define ADC_TwoSampleGap_16Cycles      (uint32_t)0x40000
#define ADC_TwoSampleGap_32Cycles      (uint32_t)0x50000
#define ADC_TwoSampleGap_64Cycles      (uint32_t)0x60000
#define ADC_TwoSampleGap_128Cycles     (uint32_t)0x70000
#define IS_ADC_TWO_SAMPLE_GAP(GAP)     ((GAP == ADC_TwoSampleGap_0Cycles) \
                                       || (GAP == ADC_TwoSampleGap_2Cycles) \
                                       || (GAP == ADC_TwoSampleGap_4Cycles) \
                                       || (GAP == ADC_TwoSampleGap_8Cycles) \
                                       || (GAP == ADC_TwoSampleGap_16Cycles) \
                                       || (GAP == ADC_TwoSampleGap_32Cycles) \
                                       || (GAP == ADC_TwoSampleGap_64Cycles) \
                                       || (GAP == ADC_TwoSampleGap_128Cycles))

/**
  * @}
  */


/** @defgroup ADC_Sample_Times
  * @{
  */

#define ADC_SampleTime_2Cycles      (uint32_t)0x0100
#define ADC_SampleTime_3Cycles      (uint32_t)0x0200
#define ADC_SampleTime_4Cycles      (uint32_t)0x0300
#define ADC_SampleTime_5Cycles      (uint32_t)0x0400
#define ADC_SampleTime_6Cycles      (uint32_t)0x0500
#define ADC_SampleTime_7Cycles      (uint32_t)0x0600
#define ADC_SampleTime_8Cycles      (uint32_t)0x0700
#define ADC_SampleTime_9Cycles      (uint32_t)0x0800
#define ADC_SampleTime_10Cycles     (uint32_t)0x0900
#define ADC_SampleTime_11Cycles     (uint32_t)0x0A00
#define ADC_SampleTime_12Cycles     (uint32_t)0x0B00
#define ADC_SampleTime_13Cycles     (uint32_t)0x0C00
#define ADC_SampleTime_14Cycles     (uint32_t)0x0D00
#define ADC_SampleTime_15Cycles     (uint32_t)0x0E00
#define IS_ADC_SAMPLE_TIME(TIME)    ((TIME == ADC_SampleTime_2Cycles) \
                                    || (TIME == ADC_SampleTime_2Cycles) \
                                    || (TIME == ADC_SampleTime_3Cycles) \
                                    || (TIME == ADC_SampleTime_4Cycles) \
                                    || (TIME == ADC_SampleTime_5Cycles) \
                                    || (TIME == ADC_SampleTime_6Cycles) \
                                    || (TIME == ADC_SampleTime_7Cycles) \
                                    || (TIME == ADC_SampleTime_8Cycles) \
                                    || (TIME == ADC_SampleTime_9Cycles) \
                                    || (TIME == ADC_SampleTime_10Cycles) \
                                    || (TIME == ADC_SampleTime_11Cycles) \
                                    || (TIME == ADC_SampleTime_12Cycles) \
                                    || (TIME == ADC_SampleTime_13Cycles) \
                                    || (TIME == ADC_SampleTime_14Cycles) \
                                    || (TIME == ADC_SampleTime_15Cycles))  
  
/**
  * @}
  */

                                    
/** @defgroup ADC_Interrupt_Sources
  * @{
  */
  
#define ADC_INT_EOC                 (uint16_t)0x0040
#define ADC_INT_ADG                 (uint16_t)0x0480
#define ADC_INT_ADL                 (uint16_t)0x0420
#define IS_ADC_INT(INT)              (((INT & (uint16_t)0xFB1F) == 0x00) && (INT != 0x00))  
                                   
/**
  * @}
  */



/** @defgroup ADC_DMA_Request_Sources
  * @{
  */
  
#define ADC_DMA_EOC                  (uint16_t)0x0020
#define ADC_DMA_ADG                  (uint16_t)0x0440
#define ADC_DMA_ADL                  (uint16_t)0x0410
#define IS_ADC_DMA_SOURCE(SOURCE)    (((SOURCE & (uint16_t)0xFB8F) == 0x00) && (SOURCE != 0x00))
                                     
/**
  * @}
  */


                                     
/** @defgroup ADC_Flags
  * @{
  */
  
#define ADC_FLAG_EOC          (uint16_t)0x0004
#define ADC_FLAG_ADG          (uint16_t)0x0002
#define ADC_FLAG_ADL          (uint16_t)0x0001

#define IS_ADC_FLAG(FLAG)     (((FLAG & (uint16_t)0xFFF8) == 0x00) && (FLAG != 0x00))

#define IS_ADC_GET_ONE_FLAG(FLAG)   ((FLAG == ADC_FLAG_EOC) \
                                    || (FLAG == ADC_FLAG_ADG) \
                                    || (FLAG == ADC_FLAG_ADL))
                              
/**
  * @}
  */                                     





/** @defgroup ADC_Sample_Sequence
  * @{
  */

#define ADC_Sequence_0              (uint8_t)0x0C
#define ADC_Sequence_1              (uint8_t)0x10
#define ADC_Sequence_2              (uint8_t)0x14
#define ADC_Sequence_3              (uint8_t)0x18
#define ADC_Sequence_4              (uint8_t)0x1C
#define ADC_Sequence_5              (uint8_t)0x20
#define ADC_Sequence_6              (uint8_t)0x24
#define ADC_Sequence_7              (uint8_t)0x28
#define IS_ADC_SEQUENCE(SEQUENCE)   ((SEQUENCE == ADC_Sequence_0) \
                                    || (SEQUENCE == ADC_Sequence_1) \
                                    || (SEQUENCE == ADC_Sequence_2) \
                                    || (SEQUENCE == ADC_Sequence_3) \
                                    || (SEQUENCE == ADC_Sequence_4) \
                                    || (SEQUENCE == ADC_Sequence_5) \
                                    || (SEQUENCE == ADC_Sequence_6) \
                                    || (SEQUENCE == ADC_Sequence_7))

/**
  * @}
  */



/** @defgroup ADC_Two_Sample_Gap_Cmd
  * @{
  */
  
#define ADC_TwoSampleGap_Enable          (uint8_t)0x01
#define ADC_TwoSampleGap_Disable         (uint8_t)0x00
#define IS_ADC_TWO_SAMPLE_GAP_CMD(CMD)   ((CMD == ADC_TwoSampleGap_Enable) \
                                         || (CMD == ADC_TwoSampleGap_Disable))  
  
/**
  * @}
  */


                                    
                                    

/** @defgroup ADC_Channel
  * @{
  */

#define ADC_Channel_0               (uint16_t)0x0000
#define ADC_Channel_1               (uint16_t)0x0001
#define ADC_Channel_2               (uint16_t)0x0002
#define ADC_Channel_3               (uint16_t)0x0003
#define ADC_Channel_4               (uint16_t)0x0004
#define ADC_Channel_5               (uint16_t)0x0005
#define ADC_Channel_6               (uint16_t)0x0006
#define ADC_Channel_7               (uint16_t)0x0007
#define ADC_Channel_8               (uint16_t)0x0008
#define ADC_Channel_9               (uint16_t)0x0009
#define ADC_Channel_Vref            (uint16_t)0x000A
#define ADC_Channel_OP1OUT          (uint16_t)0x000B
#define ADC_Channel_OP2OUT          (uint16_t)0x000C
#define ADC_Channel_OP3OUT          (uint16_t)0x000D
#define ADC_Channel_TPS             (uint16_t)0x000E
#define IS_ADC_CHANNEL(CHANNEL)     ((CHANNEL == ADC_Channel_0) \
                                    || (CHANNEL == ADC_Channel_1) \
                                    || (CHANNEL == ADC_Channel_2) \
                                    || (CHANNEL == ADC_Channel_3) \
                                    || (CHANNEL == ADC_Channel_4) \
                                    || (CHANNEL == ADC_Channel_5) \
                                    || (CHANNEL == ADC_Channel_6) \
                                    || (CHANNEL == ADC_Channel_7) \
                                    || (CHANNEL == ADC_Channel_8) \
                                    || (CHANNEL == ADC_Channel_9) \
                                    || (CHANNEL == ADC_Channel_Vref) \
                                    || (CHANNEL == ADC_Channel_OP1OUT) \
                                    || (CHANNEL == ADC_Channel_OP2OUT) \
                                    || (CHANNEL == ADC_Channel_OP3OUT) \
                                    || (CHANNEL == ADC_Channel_TPS))

/**
  * @}
  */


                               

/**
  * @}
  */ 


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

/** @defgroup ADC_Exported_Functions
  * @{
  */
  
void ADC_Reset(ADC_TypeDef* ADCx);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_OnOff(ADC_TypeDef* ADCx, CmdState OnOffState);
void ADC_SoftwareStartConversion(ADC_TypeDef* ADCx);
void ADC_SoftwareStopConversion(ADC_TypeDef* ADCx);
uint8_t ADC_GetSoftwareStartConversionStatus(ADC_TypeDef* ADCx);
void ADC_ChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t ADC_Sequence, uint8_t ADC_TwoSampleGapCmd);
void ADC_ExternalTriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger);
void ADC_ExtADTRGTriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState);
void ADC_ExtTIM8TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState);
void ADC_ExtMCM1TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState);
void ADC_ExtMCM2TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState);
void ADC_ExtGPT0TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState);
void ADC_ExtGPT1TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState);
void ADC_ExtGPT2TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState);
void ADC_ExtGPT3TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState);
void ADC_INTConfig(ADC_TypeDef* ADCx, uint16_t IT, FunctionalState NewState);
void ADC_DMAConfig(ADC_TypeDef* ADCx, uint16_t ADC_DMASource, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx, uint16_t ADC_Sequence);
void ADC_SequenceResultCompareConfig(ADC_TypeDef* ADCx, uint16_t ADC_UpLimit, uint16_t ADC_LowerLimit, uint8_t ADC_Sequence);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint16_t ADC_Flag);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint16_t ADC_Flag);

  
/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_ADC_H */

/**
  * @}
  */ 

/**
  * @}
  */ 


/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
