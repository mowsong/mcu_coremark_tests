/**
  ******************************************************************************
  * @file    sh32f2xx_rambist.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provides RAMBist APIs
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
#ifndef __SH32F2xx_RAMBIST_H
#define __SH32F2xx_RAMBIST_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"
     
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup RAMBIST_MODULE
  * @{
  */ 

/** @defgroup RAMBIST_Group_Constant  Public Constants
  * @{
  */ 

/**@enum RAMBIST_Algorithm_Type
   @brief RAMBIST check algorithm type
  */
typedef enum{
    RAMBIST_MARCH_C = 0,  /*!< Checking algorithm is March-C */
    RAMBIST_MARCH_X = 1,  /*!< Checking algorithm is March-X */
}RAMBIST_Algorithm_Type;

/*!  check rambist algorithm */
#define IS_RAMBIST_ALGORITHM(alg) (((alg)==RAMBIST_MARCH_C) || ((alg)==RAMBIST_MARCH_X))


/**@enum RAMBIST_TESTBLK_Type
   @brief RAMBIST test block size type
  */
typedef enum{
    RAMBIST_TESTBLK_16   = 0, /*!< Checking block is 16 bytes */
    RAMBIST_TESTBLK_32   = 1, /*!< Checking block is 32 bytes */
    RAMBIST_TESTBLK_64   = 2, /*!< Checking block is 64 bytes */
    RAMBIST_TESTBLK_128  = 3, /*!< Checking block is 128 bytes */
    RAMBIST_TESTBLK_256  = 4, /*!< Checking block is 256 bytes */
    RAMBIST_TESTBLK_512  = 5, /*!< Checking block is 512 bytes */
    RAMBIST_TESTBLK_1024 = 6, /*!< Checking block is 1024 bytes */
    RAMBIST_TESTBLK_2048 = 7, /*!< Checking block is 2048 bytes */
}RAMBIST_TESTBLK_Type;
/*!  check rambist test block size */
#define IS_RAMBIST_TESTBLK(blk) (((blk)==RAMBIST_TESTBLK_16) \
                                                 || ((blk)==RAMBIST_TESTBLK_32) \
                                                 || ((blk)==RAMBIST_TESTBLK_64) \
                                                 || ((blk)==RAMBIST_TESTBLK_128) \
                                                 || ((blk)==RAMBIST_TESTBLK_256) \
                                                 || ((blk)==RAMBIST_TESTBLK_512) \
                                                 || ((blk)==RAMBIST_TESTBLK_1024) \
                                                 || ((blk)==RAMBIST_TESTBLK_2048))
/**@enum RAMBIST_CheckArea_Type
   @brief RAMBIST check area type
  */
typedef enum{
    RAMBIST_CHKAREA_DATA   = 0, /*!< Check data area */
    RAMBIST_CHKAREA_BACKUP = 1, /*!< Check backup area */
}RAMBIST_CheckArea_Type;
/*!  check rambist check area */
#define IS_RAMBIST_CHECKAREA(area) (((area)==RAMBIST_CHKAREA_DATA) || ((area)==RAMBIST_CHKAREA_BACKUP))

/**
  * @}
  */ 

/** @defgroup RAMBIST_Group_Types  Public Types
  * @{
  */  

/** @struct  RAMBIST_InitTypeDef
  *   Initializes the RAMBIST structure
  */  
typedef struct{
    RAMBIST_Algorithm_Type AlgmSel; /*!< RAMBIST Alogorithm selection March-C or March-X */
    RAMBIST_TESTBLK_Type   BlkSize; /*!< RAMBIST test block size */
}RAMBIST_InitTypeDef;

/**
  * @}
  */ 


/* Exported macro ------------------------------------------------------------*/
/** @defgroup RAMBIST_Group_Macro  Public Macros
  * @{
  */ 



/** 
  *@brief  Get rambist check size from rambist block type
  *@param blktype rambist block type
  *     @arg  RAMBIST_TESTBLK_16
  *     @arg  RAMBIST_TESTBLK_32
  *     @arg  RAMBIST_TESTBLK_64
  *     @arg  RAMBIST_TESTBLK_128
  *     @arg  RAMBIST_TESTBLK_256
  *     @arg  RAMBIST_TESTBLK_512
  *     @arg  RAMBIST_TESTBLK_1024
  *     @arg  RAMBIST_TESTBLK_2048
  */
#define RAMBIST_TESTBLK_SIZE(blktype)    (1<<((blktype)+4))      
      
/**
  * @brief  Get rambist check size from rambist block type  
*/
#define RAMBIST_IS_BUSY()   (RAMBIST_CSR_BSY_BIT ? TRUE : FALSE)

/**
  * @brief  Check ram test result
  * @retval bool_t   
  *    @arg TRUE   some errors found
  *    @arg FALSE  no error    
  */
#define RAMBIST_IS_ERROR()  (RAMBIST_CSR_ERR_BIT ? TRUE : FALSE)

/**
  * @brief  Check ram test result
  * @retval bool_t   
  *    @arg TRUE   some errors found
  *    @arg FALSE  no error    
  */
#define RAMBIST_CLEAR_ERROR()  RAMBIST_CSR_ERR_BIT = 0;


/**
  * @}
  */ 

/* Exported functions --------------------------------------------------------*/ 
/** @addtogroup RAMBIST_Group_Pub_Funcs
  * @{
  */     

/* Initialize RAMBIST module */
void RAMBIST_Init(const RAMBIST_InitTypeDef* InitCfg);

/* Fills each InitV member with its default value.*/
void RAMBIST_StructInit(RAMBIST_InitTypeDef* InitStruct);
    
/* Deinitialize RAMBIST module */
void RAMBIST_Reset(void);

/* Do do rambist checking for assigned area */
void RAMBIST_Run(RAMBIST_CheckArea_Type AreaType, uint32_t Addr);


/**
  * @}
  */ 


#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_RAMBIST_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
