/**
  ******************************************************************************
  * @file    sh32f2xx_cortex.h
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
#ifndef __SH32F2xx_CORTEX_H
#define __SH32F2xx_CORTEX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup CORTEX_MODULE
  * @{
  */
     
     
/** @defgroup  CORTEX_Group_Constant  Public Constants
  * @{
  */  
/**@enum SAFE_OPMODE_Type
  *@brief variable operation mode for safe readwrite   
  */     
typedef enum{
    SAFE_OP_ADD = 0,          /*!< do add operation */
    SAFE_OP_AND = 1,          /*!< do bit AND operation */ 
    SAFE_OP_OR  = 2,          /*!< do bit OR operation */   
    SAFE_OP_NOT = 3,          /*!< do bit NOT operation */
    SAFE_OP_XOR = 4,          /*!< do bit XOR operation */ 
    SAFE_OP_SHIFT_LEFT  = 5,  /*!< do logic shift left operation */
    SAFE_OP_SHIFT_RIGHT = 6   /*!< do logic shift right operation */ 
}SAFE_OPMODE_Type;
    
/*! check safe operation mode */
#define IS_SAFE_OPMODE_Type(type) ((type) <= SAFE_OP_SHIFT_RIGHT)

/**@enum THREAD_MODE_Type
  *@brief Operation rights in thread mode
  */
typedef enum{
    THREAD_PRIVILEGED  = 0, /*!< Privileged rights in thread mode */
    THREAD_USER        = 1, /*!< User rights in thread mode */
}THREAD_MODE_Type;

/*! Check thread mode type */
#define IS_THREAD_MODE_Type(type) ((type) == THREAD_PRIVILEGED || (type) == THREAD_USER)

/**@enum STACK_MODE_Type
  *@brief stack mode
  */
typedef enum{
    STACK_MSP        = 0, /*!<default MSP stack used. If in handler mode must 0 */
    STACK_ALTERNATE  = 1, /*!<if in thread mode use PSP */
}STACK_MODE_Type;

/*! Check stack mode type */
#define IS_STACK_MODE_Type(type) ((type) == STACK_MSP || (type) == STACK_ALTERNATE)

/**
  * @}
  */ 

/** @defgroup CORTEX_Group_Types  Public Types
  * @{
  */ 

/**@struct INTStack_Typedef
  *@brief Structure for interrupt auto saved stack information
  */
typedef struct{
    uint32_t R0;   /*!< general register R0 */
    uint32_t R1;   /*!< general register R1 */
    uint32_t R2;   /*!< general register R2 */
    uint32_t R3;   /*!< general register R3 */
    uint32_t R12;  /*!< general register R12 */
    uint32_t LR;   /*!< general register LR:R14*/
    uint32_t PC;   /*!< general register PC:R15*/
    uint32_t xPSR; /*!< status register */
}INTStack_Typedef;


  
/**
  * @}
  */ 
     
     
/** @addtogroup CORTEX_Group_Pub_Funcs
  * @{
  */   

/* 32 bit variable for safe read write */
void safe_readwrite32(uint32_t* pVar, SAFE_OPMODE_Type OPType, uint32_t OPData);

/* 16 bit variable for safe read write */
void safe_readwrite16(uint16_t* pVar, SAFE_OPMODE_Type OPType, uint16_t OPData);

/* 8 bit variable for safe read write */
void safe_readwrite8(uint8_t* pVar, SAFE_OPMODE_Type OPType, uint8_t OPData);

/* enter critical program code area */
uint32_t enter_critical_section(void);

/* leave critical program code area */
void leave_critical_section(uint32_t saveSR);
 
/* switch run rights in thread mode */ 
void switch_runmode(THREAD_MODE_Type modeType);
 
 
/* Get Interrupt Stack information must call it in ISR direct */
INTStack_Typedef* INT_GetStackInfo(void); 

/* Get SVCall instruction information : must call it in ISR direct */
uint8_t SVCall_GetCallNO(INTStack_Typedef* pStackInfo); 


/** @brief switch stack mode
   * @param modeType STACK mode selected
   *     @arg STACK_MSP: use MSP as stack pointer
   *     @arg STACK_ALTERNATE: use PSP in thread mode and MSP in handler mode
   * @retval None
   * @note 
   * 1. Must implement this function as inline otherwise return failed   
   * 2. Run this functin in privileged mode or handler mode
 */ 
__STATIC_INLINE void switch_stackmode(STACK_MODE_Type modeType)
{
    uint32_t control;     
    control = __get_CONTROL();
    if(modeType == STACK_MSP)
    {
        control &= ~(1<<1);
    }
    else if(modeType == STACK_ALTERNATE)
    {
        control |= (1<<1);
    }
    __set_CONTROL(control);        
}

  
/**
  * @}
  */ 



#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_CORTEX_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/










