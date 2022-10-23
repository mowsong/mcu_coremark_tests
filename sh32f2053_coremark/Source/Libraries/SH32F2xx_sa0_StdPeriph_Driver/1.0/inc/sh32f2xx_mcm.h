/**
  ******************************************************************************
  * @file    sh32f2xx_mcm.h
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
#ifndef __SH32F2xx_MCM_H
#define __SH32F2xx_MCM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup MCM_MODULE
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/

/** @defgroup MCM_Exported_Types
  * @{
  */


typedef struct
{
	uint16_t CounterMode;  /*!< Specifie the MCMx's Counter mode.
                                This parameter can be a value of @ref MCM_Counter_Mode */
                                    
	uint16_t Prescaler;    /*!< Specifie the MCMx's Prescaler Register value.
                                This parameter can be 0 to 0xFFFF. */
                                    
	uint16_t Period;	   /*!< Specifie the MCMx's Period Register value.
                                This parameter can be 0 to 0xFFFF. */
}MCM_TimeBaseInitTypeDef;


typedef struct
{
	uint16_t PWMMode;        /*!< Specifie the MCMx PWM output mode.
                                  This parameter can be a value of @ref MCM_PWM_Mode */
                                        
	uint16_t PWMSymmetry;    /*!< Specifie the MCMx PWM output wave symmetry or asymmetry with Complementary mode.
                                  This parameter can be a value of @ref MCM_PWM_Symmetry */
                                        
	uint16_t DutyArea;      /*!< Specifie the MCMx PWM duty area.
                                 This parameter can be a value of @ref MCM_Duty_Area */
                                        
	uint16_t DutyPolarity;  /*!< Specifie the MCMx PWM duty polarity.
                                 This parameter can be a value of @ref MCM_Duty_Polarity */
    uint16_t DutyActivePoint; /*!< Speckifie the Duty register value active timing
                                 This parameter can be any combination of @ref MCM_Duty_Active_Point */
    
	uint16_t DutyValue;         /*!< Specifie the MCMx PWM Duty Register value.
                                     This parameter can be 0 to 0xFFFF */
}MCM_PWMInitTypeDef;
	

typedef struct
{
	uint16_t SC_InputLevel;   /*!< Specifie the MCMx Stoppage Check(SC) module input level.
                                   This parameter can be a value of @ref MCM_SC_Input_Level */
                                    
	uint16_t SC_ProtectPWM;   /*!< Specifie the MCMx Stoppage Check(SC) module protect pwm.
                                   This parameter can be any combination of @ref MCM_SC_Protect_PWM */
                                    
	uint16_t SC_Filter;       /*!< Specifie the MCMx Stoppage Check(SC) module filter.
                                   This parameter can be a value of @ref MCM_SC_Filter */
                                    
	uint16_t SC_ProtectTime;  /*!< Specifie the MCMx Stoppage Check(SC) module protect time.
                                   This parameter can be a value of @ref MCM_SC_Protect_Time */
}MCM_SCInitTypeDef;

/**
  * @}
  */



/* Exported constants --------------------------------------------------------*/

/** @defgroup MCM_Exported_Constants
  * @{
  */ 


#define MCM_REG_LOCK(MCMx)        (MCMx->PWMRLDEN = 0xAA) 
#define MCM_REG_UNLOCK(MCMx)      (MCMx->PWMRLDEN = 0x55)

#define MCM_FLT_REG_LOCK(MCMx)    (MCMx->FLTWEN = 0x00)
#define MCM_FLT_REG_UNLOCK(MCMx)  (MCMx->FLTWEN = 0x33CC)

//#define MCM_SET_DUTY0(MCMx,Dutyxy)  (MCMx->PWM0D = Dutyxy)
//#define MCM_SET_DUTY01(MCMx,Duty01)  (MCMx->PWM01D = Duty01)
//#define MCM_SET_DUTY1(MCMx,Duty1)  (MCMx->PWM1D = Duty1)
//#define MCM_SET_DUTY11(MCMx,Duty11)  (MCMx->PWM11D = Duty11)
//#define MCM_SET_DUTY2(MCMx,Duty2)  (MCMx->PWM2D = Duty2)
//#define MCM_SET_DUTY21(MCMx,Duty21)  (MCMx->PWM34D = Duty21)




#define IS_MCM_ALL_PERIPH(PERIPH)  ((PERIPH == MCM1) || (PERIPH == MCM2))


/** @defgroup MCM_Counter_Mode
  * @{
  */

#define MCM_CounterMode_Edge         (uint16_t)0x0000
#define MCM_CounterMode_Center       (uint16_t)0x0400
#define MCM_CounterMode_Single       (uint16_t)0x0800
#define IS_MCM_COUNTER_MODE(MODE)    ((MODE == MCM_CounterMode_Edge) \
	                                 || (MODE == MCM_CounterMode_Center) \
	                                 || (MODE == MCM_CounterMode_Single))

/**
  * @}
  */


/** @defgroup MCM_PWM_Mode
  * @{
  */

#define MCM_PWMMode_Complementary    (uint16_t)0x0000
#define MCM_PWMMode_Standalone       (uint16_t)0x8000
#define IS_MCM_PWM_MODE(MODE)        ((MODE == MCM_PWMMode_Complementary) \
                                     || (MODE == MCM_PWMMode_Standalone))

/**
  * @}
  */


/** @defgroup MCM_PWM_Symmetry
  * @{
  */

#define MCM_PWM_Symmetry                (uint16_t)0x0000
#define MCM_PWM_Asymmetry               (uint16_t)0x0200
#define IS_MCM_PWM_SYMMETRY(SYMMETRY)   ((SYMMETRY == MCM_PWM_Symmetry) \
                                        || (SYMMETRY == MCM_PWM_Asymmetry))

/**
  * @}
  */

/** @defgroup MCM_Duty_Area
  * @{
  */

#define MCM_DutyArea_1     (uint16_t)0x0000
#define MCM_DutyArea_2     (uint16_t)0x0040
#define IS_MCM_DUTY_AREA(AREA)  ((AREA == MCM_DutyArea_1) \
                                || (AREA == MCM_DutyArea_2))

/**
  * @}
  */


/** @defgroup MCM_Duty_Polarity
  * @{
  */

#define MCM_DutyPolarity_High            (uint16_t)0x0002
#define MCM_DutyPolarity_Low             (uint16_t)0x0001
#define IS_MCM_DUTY_POLARITY(POLARITY)   ((POLARITY == MCM_DutyPolarity_High) \
                                         || (POLARITY == MCM_DutyPolarity_Low))
/**
  * @}
  */


/** @defgroup MCM_Duty_Active_Point
  * @{
  */

#define MCM_DutyActivePoint_Now           (uint16_t)0x0200
#define MCM_DutyActivePoint_Zero          (uint16_t)0x0800
#define MCM_DutyActivePoint_Period        (uint16_t)0x1000
#define IS_MCM_DUTY_ACTIVE_POINT(POINT)   (((POINT & (uint16_t)0xE5FF) == 0x00) && (POINT != 0x00))
/**
  * @}
  */

/** @defgroup MCM_Interrupt_Event_Flag_Division
  * @{
  */

#define MCM_INT_EVENT_FLAG_DIV_1         (uint16_t)0x0000
#define MCM_INT_EVENT_FLAG_DIV_2         (uint16_t)0x1000
#define MCM_INT_EVENT_FLAG_DIV_3         (uint16_t)0x2000
#define MCM_INT_EVENT_FLAG_DIV_4         (uint16_t)0x3000
#define MCM_INT_EVENT_FLAG_DIV_5         (uint16_t)0x4000
#define MCM_INT_EVENT_FLAG_DIV_6         (uint16_t)0x5000
#define MCM_INT_EVENT_FLAG_DIV_7         (uint16_t)0x6000
#define MCM_INT_EVENT_FLAG_DIV_8         (uint16_t)0x7000
#define IS_MCM_INT_EVENT_FLAG_DIV(DIV)   ((DIV == MCM_INT_EVENT_FLAG_DIV_1) \
                                         || (DIV == MCM_INT_EVENT_FLAG_DIV_2) \
                                         || (DIV == MCM_INT_EVENT_FLAG_DIV_3) \
                                         || (DIV == MCM_INT_EVENT_FLAG_DIV_4) \
                                         || (DIV == MCM_INT_EVENT_FLAG_DIV_5) \
                                         || (DIV == MCM_INT_EVENT_FLAG_DIV_6) \
                                         || (DIV == MCM_INT_EVENT_FLAG_DIV_7) \
                                         || (DIV == MCM_INT_EVENT_FLAG_DIV_8))

/**
  * @}
  */


/** @defgroup MCM_CMP_Active
  * @{
  */

#define MCM_CMP_Active_Now          (uint16_t)0x0400
#define MCM_CMP_Active_Zero         (uint16_t)0x2000
#define MCM_CMP_Active_Period       (uint16_t)0x4000
#define IS_MCM_CMP_ACTIVE(ACTIVE)   (((ACTIVE & (uint16_t)0x9BFF) == 0x00) && (ACTIVE != 0x00))

/**
  * @}
  */


/** @defgroup MCM_DMA_Request_Sources
  * @{
  */

#define MCM_DMA_PWM0_Up               (uint8_t)0x01
#define MCM_DMA_PWM0_Down             (uint8_t)0x02
#define MCM_DMA_PWM1_Up               (uint8_t)0x04
#define MCM_DMA_PWM1_Down             (uint8_t)0x08
#define MCM_DMA_PWM2_Up               (uint8_t)0x10
#define MCM_DMA_PWM2_Down             (uint8_t)0x20
#define MCM_DMA_ZeroMatch             (uint8_t)0x40
#define MCM_DMA_PeriodMatch           (uint8_t)0x80
#define IS_MCM_DMA_SOURCE(SOURCE)     ((SOURCE != 0x00))

/**
  * @}
  */


/** @defgroup MCM_Interrupt_Sources
  * @{
  */

#define MCM_INT_PWM0_UP           (uint16_t)0x0001
#define MCM_INT_PWM0_DOWN         (uint16_t)0x0002
#define MCM_INT_PWM1_UP           (uint16_t)0x0004
#define MCM_INT_PWM1_DOWN         (uint16_t)0x0008
#define MCM_INT_PWM2_UP           (uint16_t)0x0010
#define MCM_INT_PWM2_DOWN         (uint16_t)0x0020
#define MCM_INT_ZM                (uint16_t)0x0040
#define MCM_INT_PM                (uint16_t)0x0080
#define MCM_INT_FLT               (uint16_t)0x0100
#define MCM_INT_OUT               (uint16_t)0x0200
#define MCM_INT_OSC               (uint16_t)0x0400
#define IS_MCM_INT(INT)           (((INT & (uint16_t)0xF800) == 0x00) && (INT != 0x00))

/**
  * @}
  */


/** @defgroup MCM_Flags
  * @{
  */

#define MCM_FLAG_PWM0_UP           (uint16_t)0x0001
#define MCM_FLAG_PWM0_DOWN         (uint16_t)0x0002
#define MCM_FLAG_PWM1_UP           (uint16_t)0x0004
#define MCM_FLAG_PWM1_DOWN         (uint16_t)0x0008
#define MCM_FLAG_PWM2_UP           (uint16_t)0x0010
#define MCM_FLAG_PWM2_DOWN         (uint16_t)0x0020
#define MCM_FLAG_ZM                (uint16_t)0x0040
#define MCM_FLAG_PM                (uint16_t)0x0080
#define MCM_FLAG_FLT               (uint16_t)0x0100
#define MCM_FLAG_OUT               (uint16_t)0x0200
#define MCM_FLAG_OSC               (uint16_t)0x0400
#define MCM_FLAG_SC1STAT           (uint16_t)0x0800
#define MCM_FLAG_SC2STAT           (uint16_t)0x1000
#define MCM_FLAG_SC3STAT           (uint16_t)0x2000

#define IS_MCM_GET_ONE_FLAG(FLAG)  (((FLAG & (uint16_t)0xC0000) == 0x00) && (FLAG != 0x00))

#define IS_MCM_FLAG(FLAG)          ((FLAG == MCM_FLAG_PWM0_UP)  \
                                   || (FLAG == MCM_FLAG_PWM0_DOWN) \
                                   ||  (FLAG == MCM_FLAG_PWM1_UP)  \
                                   || (FLAG == MCM_FLAG_PWM1_DOWN) \
                                   || (FLAG == MCM_FLAG_PWM2_UP)  \
                                   || (FLAG == MCM_FLAG_PWM2_DOWN) \
                                   || (FLAG == MCM_FLAG_ZM)  \
                                   || (FLAG == MCM_FLAG_PM) \
                                   || (FLAG == MCM_FLAG_FLT)  \
                                   || (FLAG == MCM_FLAG_OUT) \
                                   || (FLAG == MCM_FLAG_OSC) \
                                   || (FLAG == MCM_FLAG_SC1STAT) \
                                   || (FLAG == MCM_FLAG_SC2STAT) \
                                   || (FLAG == MCM_FLAG_SC3STAT))

/**
  * @}
  */

/** @defgroup MCM_Mannual_Active_Timing
  * @{
  */

#define MCM_MannualActiveTiming_Now              (uint16_t)0x0000
#define MCM_MannualActiveTiming_PeriodUpdate     (uint16_t)0x0100
#define IS_MCM_MANUAL_ACTIVE_TIMING(TIMING)      ((TIMING == MCM_MannualActiveTiming_Now) \
                                                 || (TIMING == MCM_MannualActiveTiming_PeriodUpdate))

/**
  * @}
  */


/** @defgroup MCM_Mannual_Output_Port
  * @{
  */

#define MCM_MANUAL_PWM0         (uint8_t)0x01
#define MCM_MANUAL_PWM1         (uint8_t)0x02
#define MCM_MANUAL_PWM2         (uint8_t)0x04
#define MCM_MANUAL_PWM01        (uint8_t)0x08
#define MCM_MANUAL_PWM11        (uint8_t)0x10
#define MCM_MANUAL_PWM21        (uint8_t)0x20
#define IS_MCM_MANUAL_PWM(PWM)  (((PWM & 0xC0) == 0x00) && (PWM != 0x00))

/**
  * @}
  */


/** @defgroup MCM_Mannual_Output_Port
  * @{
  */

#define MCM_OutputLevel_Low           (uint8_t)0x00
#define MCM_OutputLevel_High          (uint8_t)0x01
#define IS_MCM_OUTPUT_LEVEL(LEVEL)    ((LEVEL == MCM_MANUAL_PWM0) \
                                      || (LEVEL == MCM_MANUAL_PWM1))

/**
  * @}
  */


/** @defgroup MCM_Mannual_Output_Port
  * @{
  */

#define MCM_FLT_Pin              (uint16_t)0x0008
#define MCM_FLT_COMP             (uint16_t)0x0400
#define IS_MCM_FLT_INPUT(INPUT)  (((INPUT & (uint16_t)0xFBF7) == 0x00) && (INPUT != 0x00))

/**
  * @}
  */


/** @defgroup MCM_FLT1_Input_Sources
  * @{
  */

#define MCM_FLT1_Source_COMP1       (uint16_t)0x0000
#define MCM_FLT1_Source_COMP2       (uint16_t)0x0100
#define MCM_FLT1_Source_COMP3       (uint16_t)0x0200
#define IS_MCM_FLT1_SOURCE(SOURCE)  ((SOURCE == MCM_FLT1_Source_COMP1) \
	                                || (SOURCE == MCM_FLT1_Source_COMP2) \
	                                || (SOURCE == MCM_FLT1_Source_COMP3))

/**
  * @}
  */


/** @defgroup MCM_FLT2_Filter
  * @{
  */

#define MCM_FLT2_Filter_None         (uint16_t)0x0000
#define MCM_FLT2_Filter_0dot5us      (uint16_t)0x0010
#define MCM_FLT2_Filter_1us          (uint16_t)0x0020
#define MCM_FLT2_Filter_1dot5us      (uint16_t)0x0030
#define MCM_FLT2_Filter_2us          (uint16_t)0x0040
#define MCM_FLT2_Filter_3us          (uint16_t)0x0050
#define MCM_FLT2_Filter_4us          (uint16_t)0x0060
#define MCM_FLT2_Filter_6us          (uint16_t)0x0070
#define MCM_FLT2_Filter_8us          (uint16_t)0x0080
#define MCM_FLT2_Filter_10us         (uint16_t)0x0090
#define MCM_FLT2_Filter_12us         (uint16_t)0x00A0
#define MCM_FLT2_Filter_14us         (uint16_t)0x00B0
#define MCM_FLT2_Filter_16us         (uint16_t)0x00C0
#define MCM_FLT2_Filter_20us         (uint16_t)0x00D0
#define MCM_FLT2_Filter_24us         (uint16_t)0x00E0
#define MCM_FLT2_Filter_32us         (uint16_t)0x00F0
#define IS_MCM_FLT2_FILTER(FILTER)   ((FILTER == MCM_FLT2_Filter_None) \
                                     || (FILTER == MCM_FLT2_Filter_0dot5us) \
                                     || (FILTER == MCM_FLT2_Filter_1us) \
                                     || (FILTER == MCM_FLT2_Filter_1dot5us) \
                                     || (FILTER == MCM_FLT2_Filter_2us) \
                                     || (FILTER == MCM_FLT2_Filter_3us) \
                                     || (FILTER == MCM_FLT2_Filter_4us) \
                                     || (FILTER == MCM_FLT2_Filter_6us) \
                                     || (FILTER == MCM_FLT2_Filter_8us) \
                                     || (FILTER == MCM_FLT2_Filter_10us) \
                                     || (FILTER == MCM_FLT2_Filter_12us) \
                                     || (FILTER == MCM_FLT2_Filter_14us) \
                                     || (FILTER == MCM_FLT2_Filter_16us) \
                                     || (FILTER == MCM_FLT2_Filter_20us) \
                                     || (FILTER == MCM_FLT2_Filter_24us) \
                                     || (FILTER == MCM_FLT2_Filter_32us))
                                     
/**
  * @}
  */




/** @defgroup MCM_FLT2_Active_Level
  * @{
  */

#define MCM_FLT2_ActiveLevel_High        (uint16_t) 0x0000
#define MCM_FLT2_ActiveLevel_Low         (uint16_t) 0x0004
#define IS_MCM_FLT2_ACTIVE_LEVEL(LEVEL)  ((LEVEL == MCM_FLT2_ActiveLevel_High) \
	                                     || (LEVEL == MCM_FLT2_ActiveLevel_Low))

/**
  * @}
  */


/** @defgroup MCM_FLT_Detect_Mode
  * @{
  */

#define MCM_FLT_DETECT_Latch            (uint16_t)0x0000
#define MCM_FLT_DETECT_Each             (uint16_t)0x0002
#define IS_MCM_FLT_DETECT_MODE(MODE)    ((MODE == MCM_FLT_DETECT_Latch) \
	                                    || (MODE == MCM_FLT_DETECT_Each))

/**
  * @}
  */


/** @defgroup MCM_FLT_PWMx_Pin_Output_Status
  * @{
  */

#define MCM_FLT_PWMx_PinStatus_Hiz            (uint16_t)0x0000
#define MCM_FLT_PWMx_PinStatus_Low            (uint16_t)0x2000
#define MCM_FLT_PWMx_PinStatus_High           (uint16_t)0x3000
#define IS_MCM_FLT_PWMx_PIN_STATUS(STATUS)    ((STATUS == MCM_FLT_PWMx_PinStatus_Hiz) \
	                                          || (STATUS == MCM_FLT_PWMx_PinStatus_Low) \
	                                          || (STATUS == MCM_FLT_PWMx_PinStatus_High))

/**
  * @}
  */


/** @defgroup MCM_FLT_PWMx1_Pin_Output_Status
  * @{
  */

#define MCM_FLT_PWMx1_PinStatus_Hiz            (uint16_t)0x0000
#define MCM_FLT_PWMx1_PinStatus_Low            (uint16_t)0x8000
#define MCM_FLT_PWMx1_PinStatus_High           (uint16_t)0xC000
#define IS_MCM_FLT_PWMx1_PIN_STATUS(STATUS)    ((STATUS == MCM_FLT_PWMx1_PinStatus_Hiz) \
	                                           || (STATUS == MCM_FLT_PWMx1_PinStatus_Low) \
	                                           || (STATUS == MCM_FLT_PWMx1_PinStatus_High))

/**
  * @}
  */





/** @defgroup MCM_ADC_Trigger_Source
  * @{
  */

/*
#define MCM_TRIGGER_CMP1_None                 (uint16_t)0x0100
#define MCM_TRIGGER_CMP1_RisingFalling        (uint16_t)0x0101
#define MCM_TRIGGER_CMP1_Rising               (uint16_t)0x0102
#define MCM_TRIGGER_CMP1_Falling              (uint16_t)0x0103

#define MCM_TRIGGER_CMP2_None                 (uint16_t)0x0400
#define MCM_TRIGGER_CMP2_RisingFalling        (uint16_t)0x0404
#define MCM_TRIGGER_CMP2_Rising               (uint16_t)0x0408
#define MCM_TRIGGER_CMP2_Falling              (uint16_t)0x040C

#define MCM_TRIGGER_CMP3_None                 (uint16_t)0x1000
#define MCM_TRIGGER_CMP3_RisingFalling        (uint16_t)0x1010
#define MCM_TRIGGER_CMP3_Rising               (uint16_t)0x1020
#define MCM_TRIGGER_CMP3_Falling              (uint16_t)0x1030

#define MCM_TRIGGER_CMP4_None                 (uint16_t)0x4000
#define MCM_TRIGGER_CMP4_RisingFalling        (uint16_t)0x4040
#define MCM_TRIGGER_CMP4_Rising               (uint16_t)0x4080
#define MCM_TRIGGER_CMP4_Falling              (uint16_t)0x40C0
*/


#define MCM_TRIGGER_CMP1_None                 (uint16_t)0x0000
#define MCM_TRIGGER_CMP1_RisingFalling        (uint16_t)0x0001
#define MCM_TRIGGER_CMP1_Rising               (uint16_t)0x0002
#define MCM_TRIGGER_CMP1_Falling              (uint16_t)0x0003
#define IS_MCM_ADC_TRIGGER1_SOURCE(SOURCE)     ((SOURCE == MCM_TRIGGER_CMP1_None) \
                                              || (SOURCE == MCM_TRIGGER_CMP1_RisingFalling) \
                                              || (SOURCE == MCM_TRIGGER_CMP1_Rising) \
                                              || (SOURCE == MCM_TRIGGER_CMP1_Falling))

#define MCM_TRIGGER_CMP2_None                 (uint16_t)0x0000
#define MCM_TRIGGER_CMP2_RisingFalling        (uint16_t)0x0004
#define MCM_TRIGGER_CMP2_Rising               (uint16_t)0x0008
#define MCM_TRIGGER_CMP2_Falling              (uint16_t)0x000C
#define IS_MCM_ADC_TRIGGER2_SOURCE(SOURCE)     ((SOURCE == MCM_TRIGGER_CMP2_None) \
                                              || (SOURCE == MCM_TRIGGER_CMP2_RisingFalling) \
                                              || (SOURCE == MCM_TRIGGER_CMP2_Rising) \
                                              || (SOURCE == MCM_TRIGGER_CMP2_Falling))


#define MCM_TRIGGER_CMP3_None                 (uint16_t)0x0000
#define MCM_TRIGGER_CMP3_RisingFalling        (uint16_t)0x0010
#define MCM_TRIGGER_CMP3_Rising               (uint16_t)0x0020
#define MCM_TRIGGER_CMP3_Falling              (uint16_t)0x0030
#define IS_MCM_ADC_TRIGGER3_SOURCE(SOURCE)     ((SOURCE == MCM_TRIGGER_CMP3_None) \
                                              || (SOURCE == MCM_TRIGGER_CMP3_RisingFalling) \
                                              || (SOURCE == MCM_TRIGGER_CMP3_Rising) \
                                              || (SOURCE == MCM_TRIGGER_CMP3_Falling))


#define MCM_TRIGGER_CMP4_None                 (uint16_t)0x0000
#define MCM_TRIGGER_CMP4_RisingFalling        (uint16_t)0x0040
#define MCM_TRIGGER_CMP4_Rising               (uint16_t)0x0080
#define MCM_TRIGGER_CMP4_Falling              (uint16_t)0x00C0
#define IS_MCM_ADC_TRIGGER4_SOURCE(SOURCE)     ((SOURCE == MCM_TRIGGER_CMP4_None) \
                                              || (SOURCE == MCM_TRIGGER_CMP4_RisingFalling) \
                                              || (SOURCE == MCM_TRIGGER_CMP4_Rising) \
                                              || (SOURCE == MCM_TRIGGER_CMP4_Falling))


/**
  * @}
  */


/** @defgroup MCM_Duty_Saturation_Maximum_Selection
  * @{
  */

#define MCM_MaxSelect_None          (uint8_t)0x00
#define MCM_MaxSelect_ToMax         (uint8_t)0x10
#define MCM_MaxSelect_ToPeriod      (uint8_t)0x20
#define IS_MCM_MAX_SELECT(SELECT)   ((SELECT == MCM_MaxSelect_None) \
                                    || (SELECT == MCM_MaxSelect_ToMax) \
                                    || (SELECT == MCM_MaxSelect_ToPeriod))

/**
  * @}
  */


/** @defgroup MCM_Duty_Saturation_Minimum_Selection
  * @{
  */

#define MCM_MinSelect_None          (uint8_t)0x00
#define MCM_MinSelect_ToMin         (uint8_t)0x40
#define MCM_MinSelect_ToZero        (uint8_t)0x80
#define IS_MCM_MIN_SELECT(SELECT)   ((SELECT == MCM_MinSelect_None) \
                                    || (SELECT == MCM_MinSelect_ToZero) \
                                    || (SELECT == MCM_MinSelect_ToMin))

/**
  * @}
  */


/** @defgroup MCM_PhaseShift_MaxToMin_Selection
  * @{
  */

#define MCM_PS_MaxToMin_PWMD210             (uint8_t)0x00
#define MCM_PS_MaxToMin_PWMD201             (uint8_t)0x02
#define MCM_PS_MaxToMin_PWMD021             (uint8_t)0x04
#define MCM_PS_MaxToMin_PWMD012             (uint8_t)0x06
#define MCM_PS_MaxToMin_PWMD102             (uint8_t)0x08
#define MCM_PS_MaxToMin_PWMD120             (uint8_t)0x0A
#define IS_MCM_PS_MAXTOMIN_SELECT(SELECT)   ((SELECT == MCM_PS_MaxToMin_PWMD210) \
                                            || (SELECT == MCM_PS_MaxToMin_PWMD201) \
                                            || (SELECT == MCM_PS_MaxToMin_PWMD021) \
                                            || (SELECT == MCM_PS_MaxToMin_PWMD012) \
                                            || (SELECT == MCM_PS_MaxToMin_PWMD102) \
                                            || (SELECT == MCM_PS_MaxToMin_PWMD120))

/**
  * @}
  */


/** @defgroup MCM_Short_Circuit_Pin
  * @{
  */

#define MCM_ShortCircuit_PWM0Pin              (uint8_t)0x01
#define MCM_ShortCircuit_PWM1Pin              (uint8_t)0x02
#define MCM_ShortCircuit_PWM2Pin              (uint8_t)0x04
#define MCM_ShortCircuit_PWM01Pin             (uint8_t)0x08
#define MCM_ShortCircuit_PWM11Pin             (uint8_t)0x10
#define MCM_ShortCircuit_PWM21Pin             (uint8_t)0x20
#define IS_MCM_SHORT_CIRCUIT_PIN(PIN)         (((PIN & 0xC0) == 0x00) && (PIN != 0x00))

/**
  * @}
  */


/** @defgroup MCM_Short_Circuit_Output_Level
  * @{
  */

#define MCM_ShortCircuit_Level_Low              (uint8_t)0x00
#define MCM_ShortCircuit_Level_High             (uint8_t)0x01
#define IS_MCM_SHORT_CIRCUIT_LEVEL(LEVEL)       ((LEVEL == MCM_ShortCircuit_Level_Low) \
                                                || (LEVEL == MCM_ShortCircuit_Level_High))

/**
  * @}
  */




/** @defgroup MCM_SC_Input_Level
  * @{
  */

#define MCM_SC_InputLevel_High        (uint16_t)0x0000
#define MCM_SC_InputLevel_Low         (uint16_t)0x0800
#define IS_MCM_SC_INPUT_LEVEL(LEVEL)  ((LEVEL == MCM_SC_InputLevel_High) \
                                      || (LEVEL == MCM_SC_InputLevel_Low))

/**
  * @}
  */


/** @defgroup MCM_SC_Protect_PWM
  * @{
  */

#define MCM_SC_Protect_PWM0            (uint16_t)0x0100
#define MCM_SC_Protect_PWM1            (uint16_t)0x0200
#define MCM_SC_Protect_PWM2            (uint16_t)0x0400
#define IS_MCM_SC_PROTECT_PWM(PWM)     (((PWM & (uint16_t)0xF800) == 0x00) && (PWM != 0x00))

/**
  * @}
  */


/** @defgroup MCM_SC_Filter
  * @{
  */

#define MCM_SC_Filter_None            (uint16_t)0x0000
#define MCM_SC_Filter_30CLK           (uint16_t)0x0010
#define MCM_SC_Filter_60CLK           (uint16_t)0x0020
#define MCM_SC_Filter_90CLK           (uint16_t)0x0030
#define MCM_SC_Filter_120CLK          (uint16_t)0x0040
#define MCM_SC_Filter_180CLK          (uint16_t)0x0050
#define MCM_SC_Filter_240CLK          (uint16_t)0x0060
#define MCM_SC_Filter_360CLK          (uint16_t)0x0070
#define MCM_SC_Filter_480CLK          (uint16_t)0x0080
#define MCM_SC_Filter_600CLK          (uint16_t)0x0090
#define MCM_SC_Filter_720CLK          (uint16_t)0x00A0
#define MCM_SC_Filter_840CLK          (uint16_t)0x00B0
#define MCM_SC_Filter_960CLK          (uint16_t)0x00C0
#define MCM_SC_Filter_1200CLK         (uint16_t)0x00D0
#define MCM_SC_Filter_1440CLK         (uint16_t)0x00E0
#define MCM_SC_Filter_1920CLK         (uint16_t)0x00F0
#define IS_MCM_SC_FILTER(FILTER)      ((FILTER == MCM_SC_Filter_None) \
                                      || (FILTER == MCM_SC_Filter_30CLK) \
                                      || (FILTER == MCM_SC_Filter_60CLK) \
                                      || (FILTER == MCM_SC_Filter_90CLK) \
                                      || (FILTER == MCM_SC_Filter_120CLK) \
                                      || (FILTER == MCM_SC_Filter_180CLK) \
                                      || (FILTER == MCM_SC_Filter_240CLK) \
                                      || (FILTER == MCM_SC_Filter_360CLK) \
                                      || (FILTER == MCM_SC_Filter_480CLK) \
                                      || (FILTER == MCM_SC_Filter_600CLK) \
                                      || (FILTER == MCM_SC_Filter_720CLK) \
                                      || (FILTER == MCM_SC_Filter_840CLK) \
                                      || (FILTER == MCM_SC_Filter_960CLK) \
                                      || (FILTER == MCM_SC_Filter_1200CLK) \
                                      || (FILTER == MCM_SC_Filter_1440CLK) \
                                      || (FILTER == MCM_SC_Filter_1920CLK))

/**
  * @}
  */

                                     
/** @defgroup MCM_SC_Protect_Time
  * @{
  */

#define MCM_SC_ProtectTime_600CLK     (uint16_t)0x0000
#define MCM_SC_ProtectTime_900CLK     (uint16_t)0x0001
#define MCM_SC_ProtectTime_1200CLK     (uint16_t)0x0002
#define MCM_SC_ProtectTime_1500CLK     (uint16_t)0x0003
#define MCM_SC_ProtectTime_1800CLK     (uint16_t)0x0004
#define MCM_SC_ProtectTime_2100CLK     (uint16_t)0x0005
#define MCM_SC_ProtectTime_2400CLK     (uint16_t)0x0006
#define MCM_SC_ProtectTime_3000CLK     (uint16_t)0x0007
#define MCM_SC_ProtectTime_3600CLK     (uint16_t)0x0008
#define MCM_SC_ProtectTime_4800CLK     (uint16_t)0x0009
#define MCM_SC_ProtectTime_6000CLK     (uint16_t)0x000A
#define MCM_SC_ProtectTime_12000CLK     (uint16_t)0x000B
#define MCM_SC_ProtectTime_24000CLK     (uint16_t)0x000C
#define MCM_SC_ProtectTime_36000CLK     (uint16_t)0x000D
#define MCM_SC_ProtectTime_48000CLK     (uint16_t)0x000E
#define MCM_SC_ProtectTime_60000CLK     (uint16_t)0x000F
#define IS_MCM_SC_PROTECT_TIME(TIME)    ((TIME == MCM_SC_ProtectTime_600CLK) \
                                        || (TIME == MCM_SC_ProtectTime_900CLK) \
                                        || (TIME == MCM_SC_ProtectTime_1200CLK) \
                                        || (TIME == MCM_SC_ProtectTime_1500CLK) \
                                        || (TIME == MCM_SC_ProtectTime_1800CLK) \
                                        || (TIME == MCM_SC_ProtectTime_2100CLK) \
                                        || (TIME == MCM_SC_ProtectTime_2400CLK) \
                                        || (TIME == MCM_SC_ProtectTime_3000CLK) \
                                        || (TIME == MCM_SC_ProtectTime_3600CLK) \
                                        || (TIME == MCM_SC_ProtectTime_4800CLK) \
                                        || (TIME == MCM_SC_ProtectTime_6000CLK) \
                                        || (TIME == MCM_SC_ProtectTime_12000CLK) \
                                        || (TIME == MCM_SC_ProtectTime_24000CLK) \
                                        || (TIME == MCM_SC_ProtectTime_36000CLK) \
                                        || (TIME == MCM_SC_ProtectTime_48000CLK) \
                                        || (TIME == MCM_SC_ProtectTime_60000CLK))

/**
  * @}
  */


/** @defgroup MCM_SC
  * @{
  */

#define MCM_SC1     (uint16_t)0x3000
#define MCM_SC2     (uint16_t)0x5000
#define MCM_SC3     (uint16_t)0x9000
#define IS_MCM_SC(SC)  (((SC & (uint16_t)0x0FFF) == 0x00) && (SC != 0x00))

/**
  * @}
  */


















/**
  * @}
  */ 


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

/** @defgroup MCM_Exported_Functions
  * @{
  */

void MCM_Reset(MCM_TypeDef* MCMx);
void MCM_RegisterWriteLock(MCM_TypeDef* MCMx, FunctionalState NewState);
void MCM_OnOff(MCM_TypeDef* MCMx, CmdState OnOffState);
void MCM_TimeBaseStructInit(MCM_TimeBaseInitTypeDef* MCM_TimeBaseInitStruct);
void MCM_TimeBaseInit(MCM_TypeDef* MCMx, MCM_TimeBaseInitTypeDef* MCM_TimeBaseInitStruct);
void MCM_PWMStructInit(MCM_PWMInitTypeDef* MCM_PWMInitStruct);
void MCM_PWM0Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct);
void MCM_PWM01Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct);
void MCM_PWM1Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct);
void MCM_PWM11Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct);
void MCM_PWM2Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct);
void MCM_PWM21Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct);
void MCM_SetCounter(MCM_TypeDef* MCMx, uint16_t Counter);
void MCM_SetPrescaler(MCM_TypeDef* MCMx, uint16_t Prescaler);
void MCM_SetPeriod(MCM_TypeDef* MCMx, uint16_t Period);
uint16_t MCM_GetCounter(MCM_TypeDef* MCMx);
uint16_t MCM_GetPrescaler(MCM_TypeDef* MCMx);
uint16_t MCM_GetPeriod(MCM_TypeDef* MCMx);
void MCM_SetPWMDuty0(MCM_TypeDef* MCMx, uint16_t Duty0);
void MCM_SetPWMDuty01(MCM_TypeDef* MCMx, uint16_t Duty01);
void MCM_SetPWMDuty1(MCM_TypeDef* MCMx, uint16_t Duty1);
void MCM_SetPWMDuty11(MCM_TypeDef* MCMx, uint16_t Duty11);
void MCM_SetPWMDuty2(MCM_TypeDef* MCMx, uint16_t Duty2);
void MCM_SetPWMDuty21(MCM_TypeDef* MCMx, uint16_t Duty21);
uint16_t MCM_GetPWMDuty0(MCM_TypeDef* MCMx);
uint16_t MCM_GetPWMDuty01(MCM_TypeDef* MCMx);
uint16_t MCM_GetPWMDuty1(MCM_TypeDef* MCMx);
uint16_t MCM_GetPWMDuty11(MCM_TypeDef* MCMx);
uint16_t MCM_GetPWMDuty2(MCM_TypeDef* MCMx);
uint16_t MCM_GetPWMDuty21(MCM_TypeDef* MCMx);
void MCM_IntEventFlagDivConfig(MCM_TypeDef* MCMx, uint16_t MCM_Division);
void MCM_CompareEventActiveConfig(MCM_TypeDef* MCMx, uint16_t MCM_CMP_Active, FunctionalState NewState);
void MCM_SynchronousConfig(FunctionalState NewState);
void MCM_OSCStopDetectOnOff(MCM_TypeDef* MCMx, CmdState OnOffState);
void MCM_DMAConfig(MCM_TypeDef* MCMx, uint8_t DMASource, FunctionalState NewState);
void MCM_INTConfig(MCM_TypeDef* MCMx, uint16_t MCM_INT, FunctionalState NewState);
FlagStatus MCM_GetFlagStatus(MCM_TypeDef* MCMx, uint16_t MCM_Flag);
void MCM_ClearFlag(MCM_TypeDef* MCMx, uint16_t MCM_Flag);
void MCM_ManualPWMOutRegSync(MCM_TypeDef* MCMx, uint16_t MCM_ActiveTiming);
void MCM_ManualPWMOutOnOff(MCM_TypeDef* MCMx, uint8_t MCM_PWMx, CmdState OnOffState);
void MCM_SetManualPWMOut(MCM_TypeDef* MCMx, uint8_t MCM_PWMx);
void MCM_ResetManualPWMOut(MCM_TypeDef* MCMx, uint8_t MCM_PWMx);
void MCM_MannualPWMOutConfig(MCM_TypeDef* MCMx, uint8_t MCM_PWMx, uint8_t MCM_OutputLevel);
void MCM_FLTWriteLock(MCM_TypeDef* MCMx, FunctionalState NewState);
void MCM_FLTOnOff(MCM_TypeDef* MCMx, uint16_t MCM_FLT, CmdState OnOffState);
void MCM_FLT1SourceConfig(MCM_TypeDef* MCMx, uint16_t MCM_FLTSource);
void MCM_FLT2Config(MCM_TypeDef* MCMx, uint16_t MCM_FLT2Filter, uint16_t MCM_FLT2ActiveLevel);
void MCM_FLTConfig(MCM_TypeDef* MCMx, uint16_t MCM_DetectMode, uint16_t MCM_PWMxPinStatus, uint16_t MCM_PWMx1PinStatus);
void MCM_ClearFLTStatusFlag(MCM_TypeDef* MCMx);
FlagStatus MCM_GetFLTStatus(MCM_TypeDef* MCMx);
void MCM_PWM0DeadTimeConfig(MCM_TypeDef* MCMx, uint16_t PWM0DT,uint16_t PWM01DT);
void MCM_PWM1DeadTimeConfig(MCM_TypeDef* MCMx, uint16_t PWM1DT,uint16_t PWM11DT);
void MCM_PWM2DeadTimeConfig(MCM_TypeDef* MCMx, uint16_t PWM2DT,uint16_t PWM21DT);
void MCM_SetADCTriggerValue1(MCM_TypeDef* MCMx, uint16_t Value1);
void MCM_SetADCTriggerValue2(MCM_TypeDef* MCMx, uint16_t Value2);
void MCM_SetADCTriggerValue3(MCM_TypeDef* MCMx, uint16_t Value3);
void MCM_SetADCTriggerValue4(MCM_TypeDef* MCMx, uint16_t Value4);
void MCM_ADCTriggerConfig(MCM_TypeDef* MCMx, uint16_t MCM_ADCTrigger);
void MCM_ADCTrigger1Config(MCM_TypeDef* MCMx, uint16_t MCM_ADCTrigger1);
void MCM_ADCTrigger2Config(MCM_TypeDef* MCMx, uint16_t MCM_ADCTrigger2);
void MCM_ADCTrigger3Config(MCM_TypeDef* MCMx, uint16_t MCM_ADCTrigger3);
void MCM_ADCTrigger4Config(MCM_TypeDef* MCMx, uint16_t MCM_ADCTrigger4);
void MCM_DutySaturationMinMaxValue(MCM_TypeDef* MCMx, uint16_t MCM_PWMMin, uint16_t MCM_PWMMax);
void MCM_DutySaturationConfig(MCM_TypeDef* MCMx, uint8_t MCM_MinSelect, uint8_t MCM_MaxSelect);
void MCM_DutySaturationOnOff(MCM_TypeDef* MCMx, CmdState OnOffState);
void MCM_PhaseShiftConfig(MCM_TypeDef* MCMx, uint8_t MCM_ShiftSelect, uint16_t MCM_Shift1, uint16_t MCM_Shift2);
void MCM_PhaseShiftOnOff(MCM_TypeDef* MCMx, CmdState OnOffState);
void MCM_ShortCircuitProtectOnOff(MCM_TypeDef* MCMx, CmdState OnOffState);
void MCM_ShortCircuitOutPutLevel(MCM_TypeDef* MCMx, uint8_t MCM_PWMxPin, uint8_t MCM_OutputLevel);
void MCM_SCStructInit(MCM_SCInitTypeDef* MCM_SCInitStruct);
void MCM_SC1Init(MCM_TypeDef* MCMx, MCM_SCInitTypeDef* MCM_SCInitStruct);
void MCM_SC2Init(MCM_TypeDef* MCMx, MCM_SCInitTypeDef* MCM_SCInitStruct);
void MCM_SC3Init(MCM_TypeDef* MCMx, MCM_SCInitTypeDef* MCM_SCInitStruct);
void MCM_SCPWMProtectConfig(MCM_TypeDef* MCMx, uint16_t SCx, uint16_t MCM_PWMx, FunctionalState NewState);
void MCM_SCOnOff(MCM_TypeDef* MCMx, uint16_t SCx, CmdState OnOffState);




/**
  * @}
  */








#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_MCM_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/

