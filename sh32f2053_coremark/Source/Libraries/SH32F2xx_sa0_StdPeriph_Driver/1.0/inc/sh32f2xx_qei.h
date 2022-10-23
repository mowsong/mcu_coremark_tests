/**
  ******************************************************************************
  * @file    sh32f2xx_qei.h
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
#ifndef __SH32F2xx_QEI_H
#define __SH32F2xx_QEI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup QEI_MODULE
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @defgroup QEI_Exported_Constants
  * @{
  */ 


typedef struct
{
    uint16_t QEI_PosCntMode;             /*!< Specifie QEI position counter mode.
                                              This parameter can be a value of @ref QEI_Position_Counter_Mode */
                                                
    uint16_t QEI_Prescaler;              /*!< Specifie QEI position counter prescaler.
                                              This parameter can be 0 to 0xFFFF */
                                                
    uint16_t QEI_PosCntMaxValue;         /*!< Specifie QEI position counter maximum value.
                                              This parameter can be 0 to 0xFFFF */
                                                
    uint16_t QEI_IndexRSTPosConterCmd;   /*!< Specifie QEI index signal reset position counter.
                                              This parameter can be a value of @ref QEI_Index_Reset_Position_Counter */
                                                
    uint16_t QEI_ABSwapCmd;              /*!< Specifie whether QEI A and B signals swap or not.
                                              This parameter can be a value of @ref QEI_AB_Swap */
                                                
    uint16_t QEI_IndexFilterCmd;         /*!< Specifie QEI index signal fliter function to be enabled or disabled.
                                              This parameter can be a value of @ref QEI_Index_Filter */
                                                
    uint16_t QEI_IndexFilterPrescaler;   /*!< Specifie QEI index signal fliter prescaler.
                                              This parameter can be a value of @ref QEI_Index_Filter_Prescaler */
                                                
    uint16_t QEI_QEABFilterCmd;          /*!< Specifie QEI A and B signals filter function to be enabled or disabled.
                                              This parameter can be a value of @ref QEI_AB_Filter */
                                                
    uint16_t QEI_QEABFilterPrescaler;    /*!< Specifie QEI A and B signals filter prescaler.
                                              This parameter can be a value of @ref QEI_QEABFilterPrescaler_1 */
                                                
    uint16_t QEI_QTimerPrescaler;        /*!< Specifie QEI QTimer prescaler.
                                              This parameter can be a value of @ref QEI_QTimer_Prescaler */
                                                
    uint32_t QEI_QTimerPeriod;           /*!< Specifie QEI QTimer period value.
                                              This parameter can be 0 to 0xFFFF */
                                                
    uint16_t QEI_PosCounterMinValue;         /*!< Specifie QEI position counter minimum value.
                                              This parameter can be 0 to 0xFFFF */
}QEI_InitTypeDef;



/** @defgroup QEI_Position_Counter_Mode 
  * @{
  */ 

#define QEI_PosCntMode_2Edge_IndexRST        (uint16_t)0x0000
#define QEI_PosCntMode_2Edge_SelfContinue    (uint16_t)0x0001
#define QEI_PosCntMode_4Edge_IndexRST        (uint16_t)0x0002
#define QEI_PosCntMode_4Edge_SelfContinue    (uint16_t)0x0003
#define IS_QEI_POS_CNT_MODE(MODE)            ((MODE == QEI_PosCntMode_2Edge_IndexRST) \
                                             || (MODE == QEI_PosCntMode_2Edge_SelfContinue) \
                                             || (MODE == QEI_PosCntMode_4Edge_IndexRST) \
                                             || (MODE == QEI_PosCntMode_4Edge_SelfContinue))

/**
  * @}
  */


/** @defgroup QEI_Index_Reset_Position_Counter
  * @{
  */ 

#define QEI_IndexRSTPosConter_Enable       (uint16_t)0x0010
#define QEI_IndexRSTPosConter_Disable      (uint16_t)0x0000
#define IS_QEI_INDEX_RST_POS_COUNTER(CMD)  ((CMD == QEI_IndexRSTPosConter_Enable) \
                                           || (CMD == QEI_IndexRSTPosConter_Disable))

/**
  * @}
  */




/** @defgroup QEI_AB_Swap 
  * @{
  */ 

#define QEI_ABSwap_Enable           (uint16_t)0x0020
#define QEI_ABSwap_Disable          (uint16_t)0x0000
#define IS_QEI_AB_SWAP(CMD)   ((CMD == QEI_ABSwap_Enable) || (CMD == QEI_ABSwap_Disable))

/**
  * @}
  */

 

/** @defgroup QEI_Index_Filter 
  * @{
  */ 

#define QEI_IndexFilter_Enable       (uint16_t)0x0080
#define QEI_IndexFilter_Disable      (uint16_t)0x0000
#define IS_QEI_INDEX_FILTER(CMD)    ((CMD == QEI_IndexFilter_Enable) \
                                    || (CMD == QEI_IndexFilter_Disable))

/**
  * @}
  */



/** @defgroup QEI_Index_Filter_Prescaler 
  * @{
  */ 

#define QEI_IndexFilterPrescaler_1          (uint16_t)0x0000
#define QEI_IndexFilterPrescaler_2          (uint16_t)0x0010
#define QEI_IndexFilterPrescaler_4          (uint16_t)0x0020
#define QEI_IndexFilterPrescaler_16         (uint16_t)0x0030
#define QEI_IndexFilterPrescaler_32         (uint16_t)0x0040
#define QEI_IndexFilterPrescaler_64         (uint16_t)0x0050
#define QEI_IndexFilterPrescaler_128        (uint16_t)0x0060
#define QEI_IndexFilterPrescaler_256        (uint16_t)0x0070
#define IS_QEI_INDEX_FILTER_PRESCALER(DIV)   ((DIV == QEI_IndexFilterPrescaler_1) \
                                             || (DIV == QEI_IndexFilterPrescaler_2) \
                                             || (DIV == QEI_IndexFilterPrescaler_4) \
                                             || (DIV == QEI_IndexFilterPrescaler_16) \
                                             || (DIV == QEI_IndexFilterPrescaler_32) \
                                             || (DIV == QEI_IndexFilterPrescaler_64) \
                                             || (DIV == QEI_IndexFilterPrescaler_128) \
                                             || (DIV == QEI_IndexFilterPrescaler_256))

/**
  * @}
  */


/** @defgroup QEI_AB_Filter 
  * @{
  */ 

#define QEI_QEABFilter_Enable      (uint16_t)0x0008
#define QEI_QEABFilter_Disable     (uint16_t)0x0000
#define IS_QEI_AB_FILTER(CMD)     ((CMD == QEI_QEABFilter_Enable) \
                                  || (CMD == QEI_QEABFilter_Disable))

/**
  * @}
  */                                             



/** @defgroup QEI_AB_Filter_Prescaler 
  * @{
  */ 

#define QEI_QEABFilterPrescaler_1          (uint16_t)0x0000
#define QEI_QEABFilterPrescaler_2          (uint16_t)0x0001
#define QEI_QEABFilterPrescaler_4          (uint16_t)0x0002
#define QEI_QEABFilterPrescaler_16         (uint16_t)0x0003
#define QEI_QEABFilterPrescaler_32         (uint16_t)0x0004
#define QEI_QEABFilterPrescaler_64         (uint16_t)0x0005
#define QEI_QEABFilterPrescaler_128        (uint16_t)0x0006
#define QEI_QEABFilterPrescaler_256        (uint16_t)0x0007
#define IS_QEI_QEAB_FILTER_PRESCALER(DIV)   ((DIV == QEI_QEABFilterPrescaler_1) \
                                             || (DIV == QEI_QEABFilterPrescaler_2) \
                                             || (DIV == QEI_QEABFilterPrescaler_4) \
                                             || (DIV == QEI_QEABFilterPrescaler_16) \
                                             || (DIV == QEI_QEABFilterPrescaler_32) \
                                             || (DIV == QEI_QEABFilterPrescaler_64) \
                                             || (DIV == QEI_QEABFilterPrescaler_128) \
                                             || (DIV == QEI_QEABFilterPrescaler_256))

/**
  * @}
  */



/** @defgroup QEI_QTimer_Prescaler 
  * @{
  */ 

#define QEI_QTimerPrescaler_1          (uint16_t)0x0000
#define QEI_QTimerPrescaler_2          (uint16_t)0x0100
#define QEI_QTimerPrescaler_4          (uint16_t)0x0200
#define QEI_QTimerPrescaler_8          (uint16_t)0x0300
#define QEI_QTimerPrescaler_16        (uint16_t)0x0400
#define QEI_QTimerPrescaler_32        (uint16_t)0x0500
#define QEI_QTimerPrescaler_64        (uint16_t)0x0600
#define QEI_QTimerPrescaler_128        (uint16_t)0x0700
#define QEI_QTimerPrescaler_256        (uint16_t)0x0800
#define QEI_QTimerPrescaler_512        (uint16_t)0x0900
#define QEI_QTimerPrescaler_1024        (uint16_t)0x0A00
#define QEI_QTimerPrescaler_2048        (uint16_t)0x0B00
#define IS_QEI_QTIMER_PRESCALER(DIV)     ((DIV == QEI_QTimerPrescaler_1) \
                                         || (DIV == QEI_QTimerPrescaler_2) \
                                         || (DIV == QEI_QTimerPrescaler_4) \
                                         || (DIV == QEI_QTimerPrescaler_8) \
                                         || (DIV == QEI_QTimerPrescaler_16) \
                                         || (DIV == QEI_QTimerPrescaler_32) \
                                         || (DIV == QEI_QTimerPrescaler_64) \
                                         || (DIV == QEI_QTimerPrescaler_128) \
                                         || (DIV == QEI_QTimerPrescaler_256) \
                                         || (DIV == QEI_QTimerPrescaler_512) \
                                         || (DIV == QEI_QTimerPrescaler_1024) \
                                         || (DIV == QEI_QTimerPrescaler_2048))

/**
  * @}
  */


                                         
                                         
                                         
/** @defgroup QEI_Flag 
  * @{
  */ 

#define QEI_FLAG_PE               (uint16_t)0x0001
#define QEI_FLAG_CE               (uint16_t)0x0002
#define QEI_FLAG_Update           (uint16_t)0x0004
#define QEI_FLAG_CAP              (uint16_t)0x0008
#define QEI_FLAG_OV               (uint16_t)0x0010
#define IS_QEI_FLAG(FLAG)         ((FLAG & (uint16_t)0xFFE0) == (uint16_t)0x00 \
                                   && (FLAG != (uint16_t)0x00))

#define IS_QEI_GET_ONE_FLAG(FLAG)  ((FLAG == QEI_FLAG_PE) \
                                   || (FLAG == QEI_FLAG_CE) \
                                   || (FLAG == QEI_FLAG_Update) \
                                   || (FLAG == QEI_FLAG_CAP) \
                                   || (FLAG == QEI_FLAG_OV))
/**
  * @}
  */                                         


/** @defgroup QEI_Interrupt_Sources
  * @{
  */ 

#define QEI_INT_PE            (uint16_t)0x0001
#define QEI_INT_CE            (uint16_t)0x0002
#define QEI_INT_Update        (uint16_t)0x0004
#define QEI_INT_CAP           (uint16_t)0x0008
#define QEI_INT_OV            (uint16_t)0x0010
#define IS_QEI_INT(INT)       ((INT & (uint16_t)0xFFE0) == (uint16_t)0x00 \
                              && (INT != (uint16_t)0x00))

/**
  * @}
  */



/** @defgroup QEI_DMA_Request_Source 
  * @{
  */ 

#define QEI_DMA_PE                  (uint16_t)0x0020
#define QEI_DMA_CE                  (uint16_t)0x0040
#define QEI_DMA_Update              (uint16_t)0x0080
#define QEI_DMA_CAP                 (uint16_t)0x0100
#define QEI_DMA_OV                  (uint16_t)0x0200
#define IS_QEI_DMA_SOURCE(SOURCE)   (((SOURCE & (uint16_t)0xFC1F) == 0x00) \
                                    && (SOURCE != (uint16_t)0x00))

/**
  * @}
  */


/**
  * @}
  */ 


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

/** @defgroup QEI_Exported_Functions
  * @{
  */
extern void QEI_Reset(void);
extern void QEI_StructInit(QEI_InitTypeDef* QEI_InitStruct);
extern void QEI_Init(QEI_InitTypeDef* QEI_InitStruct);
extern void QEI_OnOff(CmdState OnOffState);
extern void QEI_QTimerOnOff(CmdState OnOffState);
extern void QEI_SetPositionCounter(uint16_t QEI_PosCounter);
extern uint16_t QEI_GetPositonCounter(void);
extern void QEI_SetQTimerCounter(uint32_t QEI_QTimerCounter);
extern uint32_t QEI_GetQTimerCounter(void);
extern uint32_t QEI_GetCaptureLatchValue(void);
extern void QEI_DMAConfig(uint16_t QEI_DMASource, FunctionalState NewState);
extern void QEI_INTConfig(uint16_t QEI_INT, FunctionalState NewState);
extern FlagStatus QEI_GetPosCounterDir(void);
extern FlagStatus QEI_GetFlagStatus(uint16_t QEI_Flag);
extern void QEI_ClearFlag(uint16_t QEI_Flag);

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_QEI_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
