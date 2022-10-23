/**
  ******************************************************************************
  * @file    sh32f9xx_retarget.h
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
#ifndef __SH32F9xx_RETARGET_H
#define __SH32F9xx_RETARGET_H

#ifdef __cplusplus
 extern "C" {
#endif


/** @addtogroup SH32F9xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup Retarget_Group
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/


/** @addtogroup Retarget_Group_Pub_Funcs
  * @{
  */
     
     
/*! Retarget Init function */
void RetargetInit(void);

/*! flush printf buffer to output device */
void flush_printfbuffer(void);

/**
  * @}
  */ 
  
  
/**
  * @}
  */ 
  
  
/**
  * @}
  */ 
    
#ifdef __cplusplus
}
#endif

#endif /*__SH32F9xx_RETARGET_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
