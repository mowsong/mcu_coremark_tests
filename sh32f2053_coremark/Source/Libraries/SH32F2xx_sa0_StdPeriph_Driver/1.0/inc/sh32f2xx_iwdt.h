/**
  ******************************************************************************
  * @file    sh32f2xx_iwdt.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    29-April-2017 
  * @brief   This file contains all the functions prototypes for the IWDT firmware
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
#ifndef __SH32F2xx_IWDT_H
#define __SH32F2xx_IWDT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup IWDT_MODULE
  * @{
  */ 

/** @defgroup  IWDT_Group_Macro  Public Macros
  * @{
  */

/** 
  *@brief  Lock or Unlock the IWDT module's regsiter. 
  */
/*! The Unlock Key */
#define IWDT_WriteAccess_Enable     ((uint16_t)0x5AA5)
/*! The Lock Key */
#define IWDT_WriteAccess_Disable    ((uint16_t)0x0000)

/*! Check the Key value */
#define IS_IWDT_WRITE_ACCESS(ACCESS) (((ACCESS) == IWDT_WriteAccess_Enable) || \
                                      ((ACCESS) == IWDT_WriteAccess_Disable))

/** 
  *@brief  IWDT prescaler 
  */
/*! IWDT prescaler set to 4 */
#define IWDT_Prescaler_4            ((uint8_t)0x00)
/*! IWDT prescaler set to 8 */
#define IWDT_Prescaler_8            ((uint8_t)0x01)
/*! IWDT prescaler set to 16 */
#define IWDT_Prescaler_16           ((uint8_t)0x02) 
/*! IWDT prescaler set to 32 */
#define IWDT_Prescaler_32           ((uint8_t)0x03)  
/*! IWDT prescaler set to 64 */
#define IWDT_Prescaler_64           ((uint8_t)0x04)  
/*! IWDT prescaler set to 128 */
#define IWDT_Prescaler_128          ((uint8_t)0x05) 
/*! IWDT prescaler set to 256 */
#define IWDT_Prescaler_256          ((uint8_t)0x06) 
/*! IWDT prescaler set to 512 */
#define IWDT_Prescaler_512          ((uint8_t)0x07)  

/*! Check the Prescaler of IWDT clock */
#define IS_IWDT_PRESCALER(PRESCALER) (((PRESCALER) == IWDT_Prescaler_4)      \
                                      || ((PRESCALER) == IWDT_Prescaler_8)   \
                                      || ((PRESCALER) == IWDT_Prescaler_16)  \
                                      || ((PRESCALER) == IWDT_Prescaler_32)  \
                                      || ((PRESCALER) == IWDT_Prescaler_64)  \
                                      || ((PRESCALER) == IWDT_Prescaler_128) \
                                      || ((PRESCALER) == IWDT_Prescaler_256) \
                                      || ((PRESCALER) == IWDT_Prescaler_512))                                     

/** 
  *@brief  Check the reload value 
  */
/*! Check the reload value  */	
#define IS_IWDT_RELOAD(RELOAD) ((RELOAD) <= 0xFFF)

/**
  * @}
  */


/** @addtogroup IWDT_Public_Functions
  * @{
  */

/* IWDT activation function ***************************************************/
void IWDT_Enable(uint8_t IWDT_Prescaler, uint16_t Reload);

/* Flag management function ***************************************************/
FlagStatus IWDT_GetFlagStatus(void);

/**
  * @}
  */ 


#ifdef __cplusplus
}
#endif

#endif /* __SH32F2xx_IWDT_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
