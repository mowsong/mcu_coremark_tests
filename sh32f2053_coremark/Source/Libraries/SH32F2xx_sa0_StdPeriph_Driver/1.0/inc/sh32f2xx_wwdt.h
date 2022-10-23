/**
  ******************************************************************************
  * @file    sh32f2xx_wwdt.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    29-April-2017
  * @brief   This file contains all the functions prototypes for the WWDT firmware
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
#ifndef __SH32F2xx_WWDT_H
#define __SH32F2xx_WWDT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup WWDT_MODULE WWDT
  * @{
  */ 

/** @defgroup  WWDT_Group_Macro Public Macros
  * @{
  */

/** 
  *@brief  WWDT Prescaler 
  */
/*! WWDT counter clock = (PCLK1/256)/1 */
#define WWDT_Prescaler_1    ((uint32_t)0x00000000)
/*! WWDT counter clock = (PCLK1/256)/2 */
#define WWDT_Prescaler_2    ((uint32_t)0x00000100)	
/*! WWDT counter clock = (PCLK1/256)/4 */
#define WWDT_Prescaler_4    ((uint32_t)0x00000200)	
/*! WWDT counter clock = (PCLK1/256)/8 */
#define WWDT_Prescaler_8    ((uint32_t)0x00000300)	
/*! WWDT counter clock = (PCLK1/256)/16 */
#define WWDT_Prescaler_16   ((uint32_t)0x00000400)	
/*! WWDT counter clock = (PCLK1/256)/32 */
#define WWDT_Prescaler_32   ((uint32_t)0x00000500)	
/*! WWDT counter clock = (PCLK1/256)/64 */
#define WWDT_Prescaler_64   ((uint32_t)0x00000600)	
/*! WWDT counter clock = (PCLK1/256)/128 */
#define WWDT_Prescaler_128  ((uint32_t)0x00000700)	

/*! Check the Prescaler of WWDT clock */
#define IS_WWDT_PRESCALER(PRESCALER) (((PRESCALER) == WWDT_Prescaler_1)  		\
                                      || ((PRESCALER) == WWDT_Prescaler_2)	\
                                      || ((PRESCALER) == WWDT_Prescaler_4)	\
                                      || ((PRESCALER) == WWDT_Prescaler_8)  \
                                      || ((PRESCALER) == WWDT_Prescaler_16) \
                                      || ((PRESCALER) == WWDT_Prescaler_32) \
                                      || ((PRESCALER) == WWDT_Prescaler_64) \
                                      || ((PRESCALER) == WWDT_Prescaler_128))
																			
/*! Check the WWDT window value */																			
#define IS_WWDT_WINDOW_VALUE(VALUE) ((VALUE) <= 0xFF)	
/*! Check the WWDT counter value */
#define IS_WWDT_COUNTER(COUNTER) ((COUNTER) <= 0xFF)	
/*! Clear WWDT */
#define CLR_WWDT() WWDT->CLR =0x5555		

/**
  * @}
  */ 



/** @addtogroup WWDT_Public_Functions Public Functions
  * @{
  */  
	

/* Prescaler, Refresh window and Counter configuration functions **************/
void WWDT_SetPrescaler(uint32_t WWDT_Prescaler);
void WWDT_SetWindowValue(uint8_t WindowValue);
void WWDT_EnableIT(bool_t EnableIT);
void WWDT_SetCounter(uint8_t Counter);

/* WWDT activation function ***************************************************/
void WWDT_Enable(uint32_t WWDT_Prescaler, bool_t EnableIT, uint8_t Counter, uint8_t WindowValue);

/* Interrupts and flags management functions **********************************/
FlagStatus WWDT_GetINTFlagStatus(void);
void WWDT_ClearFlag(void);


#ifdef __cplusplus
}
#endif

#endif /* __SH32F2xx_WWDT_H */


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
