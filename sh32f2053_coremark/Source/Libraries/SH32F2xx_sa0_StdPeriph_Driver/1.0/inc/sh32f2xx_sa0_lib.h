/**
  ******************************************************************************
  * @file    sh32f2xx_sa0_lib.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file includes all module's head files
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
#ifndef __SH32F2xx_SA0_LIB_H
#define __SH32F2xx_SA0_LIB_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup LIB_Group Global
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/**
    * @brief  Define Boolean data type
    * @arg  TRUE:   logical  true
    * @arg  FALSE:  logical  false
  */    
typedef enum{
    FALSE=0,
    TRUE =!FALSE,
}bool_t;

/** 
  *@brief  check the input value is a legal boolean data
  */
#define IS_BOOL_TYPE(var) (((var) == FALSE) || ((var) == TRUE))

/**
    * @brief  define  flag's data type
    * @arg  SET:     the flag is setted
    * @arg  RESET: the flag is cleared
  */    
typedef enum 
{ 
    RESET = 0, 
    SET = !RESET, 
}FlagStatus;

/** 
  *@brief  check the input value is a legal flag data
  */
#define IS_FLAG_STATUS(flag) (((flag) == RESET) || ((flag) == SET))

/**
    * @brief  define the functional state type
    * @arg  ENABLE:    enable the feature
    * @arg  DISABLE:  disable the feature
  */    
typedef enum 
{ 
    DISABLE = 0, 
    ENABLE = !DISABLE, 
}FunctionalState; 
/** 
  *@brief  check the input value is a legal function state data
  */
#define IS_FUNCTION_STATE(state) (((state) == DISABLE) || ((state) == ENABLE))

/**
    * @brief  define the peripheral state type
    * @arg  ON:   peripheral is enabled
    * @arg  OFF:  peripheral is disabled
  */    
typedef enum 
{ 
    OFF = 0, 
    ON = !OFF, 
}CmdState; 
/** 
  *@brief  check the input value is a legal peripheral state data
  */
#define IS_CMD_STATE(state) (((state) == ON) || ((state) == OFF))


/**
    * @brief  define the return data type
    * @arg  ERROR:      error occurs
    * @arg  SUCCESS:   operation succeeded
  */   
typedef enum 
{
    ERROR = 0, 
    SUCCESS = !ERROR
} ErrorStatus;

/** 
  *@brief  check the input value is a legal error status data
  */
#define IS_ERROR_STATUS(status) (((status) == ERROR) || ((status) == SUCCESS))


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

#ifdef   USING_USER_CFG
#include "sh32fxxx_libcfg.h"
#endif /*_NO_USER_CONFIG*/

#include "sh32f2xx_sa0.h"
      
#ifndef USING_USER_CFG

#include "sh32f2xx_flash.h"      

#include "sh32f2xx_syscfg.h"      
      
#include "sh32f2xx_rcc.h"

#include "sh32f2xx_iwdt.h"      

#include "sh32f2xx_wwdt.h"      

#include "sh32f2xx_crc.h"      
      
#include "sh32f2xx_rambist.h"

#include "sh32f2xx_gpio.h"

#include "sh32f2xx_exti.h"

#include "sh32f2xx_dma.h"

#include "sh32f2xx_mcm.h"

#include "sh32f2xx_gpt.h"

#include "sh32f2xx_gpt.h"

#include "sh32f2xx_tim.h"
      
#include "sh32f2xx_macp.h"

#include "sh32f2xx_adc.h"
      
#include "sh32f2xx_qei.h"

#include "sh32f2xx_amoc.h"

#include "sh32f2xx_uart.h"

#include "sh32f2xx_spi.h"
      
#include "sh32f2xx_twi.h"

#include "sh32f2xx_mpu.h"

#include "sh32f2xx_nvic.h"

#include "sh32f2xx_cortex.h"
#else

#ifdef _MODULE_FLASH
#include "sh32f2xx_flash.h"      
#endif      

#ifdef _MODULE_SYSCFG
#include "sh32f2xx_syscfg.h"      
#endif      
      
#ifdef _MODULE_RCC
#include "sh32f2xx_rcc.h"
#endif

#ifdef _MODULE_IWDT
#include "sh32f2xx_iwdt.h"      
#endif      

#ifdef _MODULE_WWDT
#include "sh32f2xx_wwdt.h"      
#endif      

#ifdef _MODULE_CRC
#include "sh32f2xx_crc.h"      
#endif      
      
#ifdef _MODULE_RAMBIST
#include "sh32f2xx_rambist.h"
#endif

#ifdef _MODULE_GPIO
#include "sh32f2xx_gpio.h"
#endif

#ifdef _MODULE_EXTI
#include "sh32f2xx_exti.h"
#endif

#ifdef _MODULE_DMA
#include "sh32f2xx_dma.h"
#endif

#ifdef _MODULE_MCM
#include "sh32f2xx_mcm.h"
#endif

#ifdef _MODULE_GPT
#include "sh32f2xx_gpt.h"
#endif      

#ifdef _MODULE_GPT
#include "sh32f2xx_gpt.h"
#endif      

#ifdef _MODULE_TIM
#include "sh32f2xx_tim.h"
#endif      
      
#ifdef _MODULE_MACP
#include "sh32f2xx_macp.h"
#endif      

#ifdef _MODULE_ADC
#include "sh32f2xx_adc.h"
#endif      
      
#ifdef _MODULE_QEI
#include "sh32f2xx_qei.h"
#endif      

#ifdef _MODULE_AMOC
#include "sh32f2xx_amoc.h"
#endif      

#ifdef _MODULE_UART
#include "sh32f2xx_uart.h"
#endif      

#ifdef _MODULE_SPI
#include "sh32f2xx_spi.h"
#endif      
      
#ifdef _MODULE_TWI
#include "sh32f2xx_twi.h"
#endif      

#ifdef _MODULE_MPU
#include "sh32f2xx_mpu.h"
#endif

#ifdef _MODULE_NVIC
#include "sh32f2xx_nvic.h"
#endif

#ifdef _MODULE_CORTEX
#include "sh32f2xx_cortex.h"
#endif

#endif
      
#ifdef _MODULE_DBG_PRINTF
#include <stdio.h>
#else
/** @brief  Bypass the printf function to save code size */
#define printf(...) ((void)0)
#endif

#ifdef DEFAULT_ASSERT_ENABLE
/** @brief  assert for function's input parameters. Enable or disable it by the macro _DEBUG  at compile time*/
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t*)__FILE__,(uint32_t)__LINE__));
/** @brief  break the program at the assert  failed satement . Enable or disable it by the macro _DEBUG at compile time */
#define assert_break(expr) if(!(expr)){ __BKPT(0);}
/** @brief  define the assert failed actions here. Enable or disable it by the macro _DEBUG  at compile time */
void assert_failed(uint8_t* file, uint32_t line);
#else
/** @brief  assert for function's input parameters. Enable or disable it by the macro _DEBUG  at compile time*/
#define assert_param(expr) ((void)0)
/** @brief  break the program at the assert  failed satement . Enable or disable it by the macro _DEBUG at compile time */
#define assert_break(expr) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_LIB_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/










