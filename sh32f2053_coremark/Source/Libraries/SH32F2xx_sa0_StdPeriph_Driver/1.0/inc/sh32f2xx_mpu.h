/**
  ******************************************************************************
  * @file    sh32f2xx_mpu.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file contains all the functions prototypes for the MPU firmware
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
#ifndef __SH32F2xx_MPU_H
#define __SH32F2xx_MPU_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup MPU_MODULE
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @defgroup MPU_Exported_Constants
  * @{
  */ 

typedef struct
{
    uint16_t MPU_Region;            /*!< Specifie the MPU region number.
                                         This parameter can be 0 to 7 */
                                            
    uint16_t MPU_RegionSize;        /*!< Specifie the MPU region size.
                                         This parameter can be a value of @ref MPU_Region_Size */
                                            
    uint32_t MPU_Address;           /*!< Specifie the MPU region address. */
                                         
    uint32_t MPU_AccessPermission;  /*!< Specifie the MPU region address. 
                                         This parameter can be a value of @ref MPU_Access_Permission_Mode */
                                            
    uint32_t MPU_Execute;           /*!< Specifie the MPU region execute function enable or disable. 
                                         This parameter can be a value of @ref MPU_Execute_Cmd */
                                            
    uint32_t MPU_Sharable;          /*!< Specifie the MPU region sharable function enable or disable. 
                                         This parameter can be a value of @ref MPU_Sharable_Cmd */
                                            
    uint32_t MPU_Buffable;          /*!< Specifie the MPU region buffable function enable or disable. 
                                         This parameter can be a value of @ref MPU_Buffable_Cmd */
                                         
    uint16_t MPU_SubregionDisable;  /*!< Specifie the MPU disable subregion number. 
                                         This parameter can be any combination of @ref MPU_Disable_Subregion */
    
    uint16_t MPU_RegionCmd;         /*!< Specifie the MPU region to be enabled or disabled. 
                                         This parameter can be ENABLE or DISABLE */
                                            
}MPU_InitTypeDef;




 

/** @defgroup MPU_Region_Size 
  * @{
  */ 

#define MPU_RegionSize_32B              (uint16_t)0x0008
#define MPU_RegionSize_64B              (uint16_t)0x000A
#define MPU_RegionSize_128B             (uint16_t)0x000C
#define MPU_RegionSize_256B             (uint16_t)0x000E
#define MPU_RegionSize_512B             (uint16_t)0x0010
#define MPU_RegionSize_1KB              (uint16_t)0x0012
#define MPU_RegionSize_2KB              (uint16_t)0x0014
#define MPU_RegionSize_4KB              (uint16_t)0x0016
#define MPU_RegionSize_8KB              (uint16_t)0x0018
#define MPU_RegionSize_16KB             (uint16_t)0x001A
#define MPU_RegionSize_32KB             (uint16_t)0x001C
#define MPU_RegionSize_64KB             (uint16_t)0x001E
#define MPU_RegionSize_128KB            (uint16_t)0x0020
#define MPU_RegionSize_256KB            (uint16_t)0x0022
#define MPU_RegionSize_512KB            (uint16_t)0x0024
#define MPU_RegionSize_1MB              (uint16_t)0x0026
#define MPU_RegionSize_2MB              (uint16_t)0x0028
#define MPU_RegionSize_4MB              (uint16_t)0x002A
#define MPU_RegionSize_8MB              (uint16_t)0x002C
#define MPU_RegionSize_16MB             (uint16_t)0x002E
#define MPU_RegionSize_32MB             (uint16_t)0x0030
#define MPU_RegionSize_64MB             (uint16_t)0x0032
#define MPU_RegionSize_128MB            (uint16_t)0x0034
#define MPU_RegionSize_256MB            (uint16_t)0x0036
#define MPU_RegionSize_512MB            (uint16_t)0x0038
#define MPU_RegionSize_1GB              (uint16_t)0x003A
#define MPU_RegionSize_2GB              (uint16_t)0x003C
#define MPU_RegionSize_4GB              (uint16_t)0x003E
#define IS_MPU_REGION_SIZE(SIZE)        (((SIZE >> 1) >= (uint16_t)0x0004) \
                                        && ((SIZE >> 1) <= (uint16_t)0x001F))

/**
  * @}
  */

/** @defgroup MPU_Access_Permission_Mode 
  * @{
  */ 

#define MPU_AP_PRI_NO_USER_NO        (uint32_t)0x0000000
#define MPU_AP_PRI_RW_USER_NO        (uint32_t)0x1000000
#define MPU_AP_PRI_RW_USER_RO        (uint32_t)0x2000000
#define MPU_AP_PRI_RW_USER_RW        (uint32_t)0x3000000
#define MPU_AP_PRI_RO_USER_NO        (uint32_t)0x5000000
#define MPU_AP_PRI_RO_USER_RO        (uint32_t)0x6000000
#define IS_MPU_AP_MODE(MODE)         ((MODE == MPU_AP_PRI_NO_USER_NO) \
                                     || (MODE == MPU_AP_PRI_RW_USER_NO) \
                                     || (MODE == MPU_AP_PRI_RW_USER_RO) \
                                     || (MODE == MPU_AP_PRI_RW_USER_RW) \
                                     || (MODE == MPU_AP_PRI_RO_USER_NO) \
                                     || (MODE == MPU_AP_PRI_RO_USER_RO))

/**
  * @}
  */

/** @defgroup MPU_Execute_Cmd 
  * @{
  */ 

#define MPU_Execute_Enable        (uint32_t)0x00000000
#define MPU_Execute_Disable       (uint32_t)0x10000000
#define IS_MPU_EXECUTE_CMD(CMD)   ((CMD == MPU_Execute_Enable) \
                                  || (CMD == MPU_Execute_Disable))

/**
  * @}
  */
  
/** @defgroup MPU_Sharable_Cmd 
  * @{
  */ 

#define MPU_Sharable_Enable        (uint32_t)0x40000
#define MPU_Sharable_Disable       (uint32_t)0x00000
#define IS_MPU_SHARABLE_CMD(CMD)   ((CMD == MPU_Sharable_Enable) \
                                   || (CMD == MPU_Sharable_Disable))

/**
  * @}
  */

/** @defgroup MPU_Buffable_Cmd 
  * @{
  */ 

#define MPU_Buffable_Enable        (uint32_t)0x10000
#define MPU_Buffable_Disable       (uint32_t)0x00000
#define IS_MPU_BUFFABLE_CMD(CMD)   ((CMD == MPU_Buffable_Enable) \
                                   || (CMD == MPU_Buffable_Disable))

/**
  * @}
  */

/** @defgroup MPU_Disable_Subregion 
  * @{
  */ 
  
#define MPU_Subregion_Disable_None         (uint16_t)0x0000
#define MPU_Subregion_Disable_0            (uint16_t)0x0100
#define MPU_Subregion_Disable_1            (uint16_t)0x0200
#define MPU_Subregion_Disable_2            (uint16_t)0x0400
#define MPU_Subregion_Disable_3            (uint16_t)0x0800
#define MPU_Subregion_Disable_4            (uint16_t)0x1000
#define MPU_Subregion_Disable_5            (uint16_t)0x2000
#define MPU_Subregion_Disable_6            (uint16_t)0x4000
#define MPU_Subregion_Disable_7            (uint16_t)0x8000
#define IS_MPU_SUBREGION_DISABLE(REGION)   (((REGION & (uint16_t)0x00FF) == (uint16_t)0x00) \
                                           || (REGION == (uint16_t)0x00))

/**
  * @}
  */


/**
  * @}
  */ 


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

/** @defgroup MPU_Exported_Functions 
  * @{
  */

void MPU_Reset(void);
void MPU_StructInit(MPU_InitTypeDef* MPU_InitStruct);
void MPU_Init(MPU_InitTypeDef* MPU_InitStruct);
void MPU_OnOff(CmdState OnOffState);
uint8_t MPU_GetCurrentRegionNumber(void);

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_MPU_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
