/**
  ******************************************************************************
  * @file    sh32f2xx_crc.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provide CRC module's APIs
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
#ifndef __SH32F2xx_CRC_H
#define __SH32F2xx_CRC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"
     
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup CRC_MODULE
  * @{
  */ 

/** @defgroup CRC_Group_Constant  Public Constants
  * @{
  */ 

/*!  CRC Mode CONSTANT */
typedef enum {
    CRC_MODE_32     =   0,    /*!< 32 bits CRC (POLY: 04C11DB7) */
    CRC_MODE_16     =   1,    /*!< 16 bits CRC (POLY: 8005) */
    CRC_MODE_CITT   =   2,   /*!< 16 bits CRC (POLY: 1021) */
    CRC_MODE_8      =   3,     /*!< 8 bits CRC (POLY: 7) */
}CRC_MODE_Type;
/*!  check CRC mode selection */
#define IS_CRC_MODE(mode)    (((mode) == CRC_MODE_32)   \
                                       || ((mode) == CRC_MODE_16)   \
                                       || ((mode) == CRC_MODE_CITT)   \
                                       || ((mode) == CRC_MODE_8))

/*!  CRC Input Mode CONSTANT */
typedef enum {
    CRC_INPUT_WORD     =   0,  /*!< One Word One Calculate */
    CRC_INPUT_HALFWORD  =   1,  /*!< Half Word One Calculate */
    CRC_INPUT_BYTE      =   2,  /*!< One Byte One Calculate */
}CRC_INPUT_Type;
/*!  check CRC input mode selection*/
#define IS_CRC_INPUTTYPE(type)    (((type) == CRC_INPUT_WORD)   \
                               || ((type) == CRC_INPUT_HALFWORD)   \
                               || ((type) == CRC_INPUT_BYTE))


/**
  * @}
  */ 

/** @defgroup RCC_Group_Types  Public Types
  * @{
  */     

/*! @struct  CRC_InitTypeDef
  *   structure for CRC Options
  */    
typedef struct
{   
    uint32_t Reload           :1;   /*!< Reload the Initial value to calculate unit: ENABLE or DISABLE */
    uint32_t rev1             :7;   /*!< reserved */
    uint32_t Mode             :2;   /*!< POLY mode selection: CRC_MODE_32, CRC_MODE_16,CRC_MODE_CITT,CRC_MODE_8*/
    uint32_t ComplementInput  :1;   /*!< complement the input value  then to calculate : ENABLE or DISABLE*/
    uint32_t BitRevertInput   :1;   /*!< reverse the input value by bit then to calculate: ENABLE or DISABLE */
    uint32_t ByteRevertInput  :1;   /*!< reverse the input value by byte then to calculate: ENABLE or DISABLE */
    uint32_t ComplementOutput :1;   /*!< complement the result value : ENABLE or DISABLE */
    uint32_t BitRevertOutput  :1;   /*!< reverse the result value by bit : ENABLE or DISABLE  */
    uint32_t ByteRevertOutput :1;   /*!< reverse the result value by byte : ENABLE or DISABLE */
    uint32_t rev              :16;  /*!< reserved for address*/
    uint32_t InitialVal;            /*!< Inital Value */
}CRC_InitTypeDef;


/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/** @defgroup CRC_Group_Macro  Public Macros
  * @{
  */ 

/*! get last CRC value */
#define CRC_GetCRC()  (CRC->DR)      

/*! reload CRC init value */
#define CRC_Reload()   (CRC_CR_RELOAD_BIT = SET)

/**
  * @}
  */ 

/* Exported functions --------------------------------------------------------*/ 
/** @addtogroup CRC_Group_Pub_Funcs
  * @{
  */     

/* CRC Module Initialization */
void CRC_Init(const CRC_InitTypeDef* crcInit);

/* CRC fill the initial Structure */
void CRC_StructInit(CRC_InitTypeDef* StructInit);

/* Deinitializes the CRC  registers to their default reset */
void CRC_Reset(void);

/* Calcuate one data's CRC value */
uint32_t CRC_CalcCRC(uint32_t Data, CRC_INPUT_Type InputType);

/* Calcuate one block data's CRC value */
uint32_t CRC_CalcBlockCRC(const uint8_t* InBuffer, uint32_t Count, CRC_INPUT_Type InputType);


/**
  * @}
  */ 


#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_CRC_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
