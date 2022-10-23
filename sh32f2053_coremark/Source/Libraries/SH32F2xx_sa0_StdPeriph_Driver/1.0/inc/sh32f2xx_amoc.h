/**
  ******************************************************************************
  * @file    sh32f2xx_amoc.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file contains all the functions prototypes for the AMOC firmware
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
#ifndef __SH32F2xx_AMOC_H
#define __SH32F2xx_AMOC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup AMOC_MODULE
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/

/** @defgroup AMOC_Exported_Types
  * @{
  */

/**
  * @brief COMP Init structure definition
  */
  
typedef struct
{
    uint32_t COMP_NRefVoltageSource; /*!< Specifie the comparetor none-inverting input voltage source.
                                          This parameter can be a value of @ref COMP_NRefVoltageSource */
                                          
    uint32_t COMP_NRefVoltageLevel;  /*!< Specifie the comparetor none-inverting input voltage level.
                                          This parameter can be a value of @ref COMP_None-inverting_Reference_Voltage_Level */
                                          
    uint32_t COMP_OutputCmd;         /*!< Specifie the comparetor output function to be enabled or disabled.
                                          This parameter can be a value of @ref COMP_Output */
                                          
    uint32_t COMP_SchmittVoltage;    /*!< Specifie the comparetor schmitt voltage.
                                          This parameter can be a value of @ref COMP_Schmitt_Voltage_Select */
                                          
    uint32_t COMP_NSelect;           /*!< Specifie the comparetor none-inverting input pin source.
                                          This parameter can be a value of @ref COMP_None-inverting_Input_Select */
                                          
    uint32_t COMP_PSelect;           /*!< Specifie the comparetor inverting input pin source.
                                          This parameter can be a value of @ref COMP_Inverting_Input_Select */
                                          
    uint32_t COMP_Filter;            /*!< Specifie the comparetor filter value.
                                          This parameter can be a value of @ref Comp_Filter */
}COMP_InitTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup AMOC_Exported_Constants
  * @{
  */ 


/** @defgroup AMOC_Comp_Peripheral
  * @{
  */

#define COMP1                       (uint8_t)0x00
#define COMP2                       (uint8_t)0x04
#define COMP3                       (uint8_t)0x08
#define IS_COMP_ALL_PERIPH(PERIPH)  ((PERIPH == COMP1) || (PERIPH == COMP2) \
                                    || (PERIPH == COMP3))

/**
  * @}
  */


/** @defgroup AMOC_OP_Peripheral
  * @{
  */

#define OP1                       (uint8_t)0x01
#define OP2                       (uint8_t)0x02
#define OP3                       (uint8_t)0x04
#define IS_OP_ALL_PERIPH(PERIPH)  ((PERIPH == OP1) || (PERIPH == OP2) || (PERIPH == OP3))

/**
  * @}
  */




/** @defgroup Comp_Filter
  * @{
  */

#define COMP_Filter_None          (uint32_t)0x00
#define COMP_Filter_0us5          (uint32_t)0x01
#define COMP_Filter_1us           (uint32_t)0x02
#define COMP_Filter_2us           (uint32_t)0x03
#define COMP_Filter_4us           (uint32_t)0x04
#define COMP_Filter_8us           (uint32_t)0x05
#define COMP_Filter_12us          (uint32_t)0x06
#define COMP_Filter_16us          (uint32_t)0x07
#define IS_COMP_FILTER(FILTER)    ((FILTER == COMP_Filter_None) \
                                  || (FILTER == COMP_Filter_0us5) \
                                  || (FILTER == COMP_Filter_1us) \
                                  || (FILTER == COMP_Filter_2us) \
                                  || (FILTER == COMP_Filter_4us) \
                                  || (FILTER == COMP_Filter_8us) \
                                  || (FILTER == COMP_Filter_12us) \
                                  || (FILTER == COMP_Filter_16us))

/**
  * @}
  */




/** @defgroup COMP_Inverting_Input_Select
  * @{
  */

#define COMP_PSelect_P0             (uint32_t)0x00
#define COMP_PSelect_P1             (uint32_t)0x01
#define COMP_PSelect_P2             (uint32_t)0x02
#define COMP_PSelect_OP1OUT         (uint32_t)0x03
#define COMP_PSelect_OP2OUT         (uint32_t)0x04
#define COMP_PSelect_OP3OUT         (uint32_t)0x05
#define IS_COMP_1_P_SELECT(SELECT)  ((SELECT == COMP_PSelect_P0) || (SELECT == COMP_PSelect_P1))

#define IS_COMP_2_P_SELECT(SELECT)  ((SELECT == COMP_PSelect_P0) \
                                    || (SELECT == COMP_PSelect_P1) \
                                    || (SELECT == COMP_PSelect_P2) \
                                    || (SELECT == COMP_PSelect_OP1OUT) \
                                    || (SELECT == COMP_PSelect_OP2OUT) \
                                    || (SELECT == COMP_PSelect_OP3OUT))

#define IS_COMP_3_P_SELECT(SELECT)  ((SELECT == COMP_PSelect_P0) \
                                    || (SELECT == COMP_PSelect_P1) \
                                    || (SELECT == COMP_PSelect_OP1OUT) \
                                    || (SELECT == COMP_PSelect_OP2OUT) \
                                    || (SELECT == COMP_PSelect_OP3OUT))

/**
  * @}
  */



/** @defgroup COMP_None-inverting_Input_Select
  * @{
  */
  
#define COMP_NSelect_NPin             (uint32_t)0x0000
#define COMP_NSelect_VRef             (uint32_t)0x0040
#define IS_COMP_N_SELECT(SELECT)      ((SELECT == COMP_NSelect_NPin) \
                                      || (SELECT == COMP_NSelect_VRef))

/**
  * @}
  */




/** @defgroup COMP_Schmitt_Voltage_Select
  * @{
  */

#define COMP_SchmittVoltage_None          (uint32_t)0x0000
#define COMP_SchmittVoltage_10mV          (uint32_t)0x0100
#define COMP_SchmittVoltage_20mV          (uint32_t)0x0200
#define COMP_SchmittVoltage_50mV          (uint32_t)0x0300
#define IS_COMP_SCHMITT_VOLTAGE(VOLTAGE)  ((VOLTAGE == COMP_SchmittVoltage_None) \
                                          || (VOLTAGE == COMP_SchmittVoltage_10mV) \
                                          || (VOLTAGE == COMP_SchmittVoltage_20mV) \
                                          || (VOLTAGE == COMP_SchmittVoltage_50mV))

/**
  * @}
  */



/** @defgroup COMP_Output
  * @{
  */

#define COMP_Output_Disable        (uint32_t)0x0000
#define COMP_Output_Enable         (uint32_t)0x0800
#define IS_COMP_OUTPUT_CMD(CMD)    ((CMD == COMP_Output_Disable) || (CMD == COMP_Output_Enable))

/**
  * @}
  */


/** @defgroup COMP_None-inverting_Reference_Voltage_Level
  * @{
  */

#define COMP_NRefVoltageLevel_VRef          (uint32_t)0x00000
#define COMP_NRefVoltageLevel_1_8           (uint32_t)0x10000
#define COMP_NRefVoltageLevel_2_8           (uint32_t)0x20000
#define COMP_NRefVoltageLevel_3_8           (uint32_t)0x30000
#define COMP_NRefVoltageLevel_4_8           (uint32_t)0x40000
#define COMP_NRefVoltageLevel_5_8           (uint32_t)0x50000
#define COMP_NRefVoltageLevel_6_8           (uint32_t)0x60000
#define COMP_NRefVoltageLevel_7_8           (uint32_t)0x70000
#define IS_COMP_N_REF_VOLTAGE_LEVEL(LEVEL)  ((LEVEL == COMP_NRefVoltageLevel_VRef) \
                                                || (LEVEL == COMP_NRefVoltageLevel_1_8) \
                                                || (LEVEL == COMP_NRefVoltageLevel_2_8) \
                                                || (LEVEL == COMP_NRefVoltageLevel_3_8) \
                                                || (LEVEL == COMP_NRefVoltageLevel_4_8) \
                                                || (LEVEL == COMP_NRefVoltageLevel_5_8) \
                                                || (LEVEL == COMP_NRefVoltageLevel_6_8) \
                                                || (LEVEL == COMP_NRefVoltageLevel_7_8))

/**
  * @}
  */


    

/** @defgroup COMP_NRefVoltageSource
  * @{
  */

#define COMP_NRefVoltage_AVDD      (uint32_t)0x00000
#define COMP_NRefVoltage_VREF      (uint32_t)0x80000
#define IS_COMP_N_REF_VOLTAGE_SOURCE(SOURCE)  ((SOURCE == COMP_NRefVoltage_AVDD) \
                                              || (SOURCE == COMP_NRefVoltage_VREF))
                                              
/**
  * @}
  */                                              


/** @defgroup COMP_Interrupt_Sources
  * @{
  */

#define COMP_INT_RISING      (uint16_t)0x4000
#define COMP_INT_FALLING     (uint16_t)0x2000
#define IS_COMP_INT(INT)      (((INT & (uint16_t)0x9FFF) == 0x00) && (INT != 0x00))

/**
  * @}
  */


/** @defgroup AMOC_Internal_Voltage_Reference_CHOP
  * @{
  */

#define AMOC_VrefCHOP_Positive                (uint8_t)0x00
#define AMOC_VrefCHOP_Negative                (uint8_t)0x04
#define IS_AMOC_VREF_CHOP_POLARITY(POLARITY)  ((POLARITY == AMOC_VrefCHOP_Positive) \
                                              || (POLARITY == AMOC_VrefCHOP_Negative))

/**
  * @}
  */ 


/** @defgroup AMOC_Internal_Voltage_Reference
  * @{
  */

#define AMOC_VrefVoltage_2V5           (uint8_t)0x00
#define AMOC_VrefVoltage_1V65          (uint8_t)0x02
#define IS_AMOC_VREF_VOLTAGE(VOLTAGE)  ((VOLTAGE == AMOC_VrefVoltage_2V5) \
                                       || (VOLTAGE == AMOC_VrefVoltage_1V65))

/**
  * @}
  */  



/**
  * @}
  */ 


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

/** @defgroup AMOC_Exported_Functions
  * @{
  */

extern void COMP_Reset(uint8_t COMPx);
extern void COMP_StructInit(COMP_InitTypeDef* COMP_InitStruct);
extern void COMP_Init(uint8_t COMPx, COMP_InitTypeDef* COMP_InitStruct);
extern void COMP_OnOff(uint8_t COMPx, CmdState OnOffState);
extern uint8_t COMP_GetOutputLevel(uint8_t COMPx);
extern void COMP_DMAConfig(uint8_t COMPx, FunctionalState NewState);
extern void COMP_INTConfig(uint8_t COMPx, uint16_t COMP_INT, FunctionalState NewState);
extern FlagStatus COMP_GetFlagStatus(uint8_t COMPx);
extern void COMP_ClearFlag(uint8_t COMPx);
extern void OP_OnOff(uint8_t OPx, CmdState OnOffState);
extern void AMOC_VrefCHOPOnOff(CmdState OnOffState);
extern void AMOC_VrefVoltageConfig(uint8_t AMOC_VrefVoltage);
extern void AMOC_VrefOnOff(CmdState OnOffState);
extern void AMOC_TempSensorCHOPConfig(FunctionalState NewState);
extern void AMOC_TempSensorOnOff(CmdState OnOffState);

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_AMOC_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
