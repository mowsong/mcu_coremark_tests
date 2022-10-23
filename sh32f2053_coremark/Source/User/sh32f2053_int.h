/**
  ******************************************************************************
  * @file    sh32f2053_int.h 
  * @author  
  * @version V1.1.0
  * @date    15-April-2016
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SH32F2053_INT_H__
#define __SH32F2053_INT_H__

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "sh32f2053.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __SH32F2053_INT_H__ */

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
