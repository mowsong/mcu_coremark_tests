/**
  ******************************************************************************
  * @file    sh32f2xx_twi.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provide APIs for using TWI module
  *         
  * @verbatim
  *
  *          ===================================================================
  *                                  How to use this driver
  *          ===================================================================

  *  @endverbatim
  *
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
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_cortex.h"
     
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/* TWI  Module----------------------------------------------------------*/
/** @defgroup CORTEX_MODULE  Cortex-M3 
   *  TWI Calculate Mode
  * @{
  */ 
        
/** @defgroup  CORTEX_Group_Pub_Funcs  Public Functions
 *  @brief   Cortex-M3 Public Functions
 *
  * @{
  */
 
/** @brief 32 bit variable for safe read write
  * @param pVar   pointer to variable which need modify
  * @param OPType variable operation mode
  *      @arg @b SAFE_OP_ADD  *pVar += OPData
  *      @arg @b SAFE_OP_AND  *pVar &= OPData
  *      @arg @b SAFE_OP_OR   *pVar |= OPData
  *      @arg @b SAFE_OP_NOT  *pVar = ~(*pVar)
  *      @arg @b SAFE_OP_XOR  *pVar ^= OPData
  *      @arg @b SAFE_OP_SHIFT_LEFT  *pVar <<= OPData
  *      @arg @b SAFE_OP_SHIFT_RIGHT *pVar >>= OPData
  * @param OPData operation data
  * @retval None
  * @note This function used for protect read then write operation if it be interrupted
  */ 
#if defined(__ARMCC_VERSION)
#if (__ARMCC_VERSION >= 5060020)
#define __LDREXW_S(ptr)          _Pragma("push") _Pragma("diag_suppress 3731") ((uint32_t ) __ldrex(ptr))  _Pragma("pop")
#define __STREXW_S(value, ptr)   _Pragma("push") _Pragma("diag_suppress 3731") __strex(value, ptr)        _Pragma("pop")
#define __LDREXH_S(ptr)          _Pragma("push") _Pragma("diag_suppress 3731") ((uint16_t ) __ldrex(ptr))  _Pragma("pop")
#define __STREXH_S(value, ptr)   _Pragma("push") _Pragma("diag_suppress 3731") __strex(value, ptr)        _Pragma("pop")
#define __LDREXB_S(ptr)          _Pragma("push") _Pragma("diag_suppress 3731") ((uint8_t ) __ldrex(ptr))  _Pragma("pop")
#define __STREXB_S(value, ptr)   _Pragma("push") _Pragma("diag_suppress 3731") __strex(value, ptr)        _Pragma("pop")
#else
#define __LDREXW_S(ptr)                                                        ((uint32_t ) __ldrex(ptr))
#define __STREXW_S(value, ptr)                                                 __strex(value, ptr)
#define __LDREXH_S(ptr)                                                        ((uint16_t ) __ldrex(ptr))
#define __STREXH_S(value, ptr)                                                 __strex(value, ptr)
#define __LDREXB_S(ptr)                                                        ((uint8_t ) __ldrex(ptr))
#define __STREXB_S(value, ptr)                                                 __strex(value, ptr)
#endif
#else
#define __LDREXW_S   __LDREXW
#define __STREXW_S   __STREXW
#define __LDREXH_S   __LDREXH
#define __STREXH_S   __STREXH
#define __LDREXB_S   __LDREXB
#define __STREXB_S   __STREXB
#endif

void safe_readwrite32(uint32_t* pVar, SAFE_OPMODE_Type OPType, uint32_t OPData)
{
    __IO uint32_t NewValue;    
    
    assert_param(IS_SAFE_OPMODE_Type(OPType));
    
    do{
        NewValue = __LDREXW_S((unsigned long volatile*)pVar);
        switch(OPType)
        {
            case SAFE_OP_ADD:
                NewValue += (int32_t)OPData;
                break;
            case SAFE_OP_AND:
                NewValue &= OPData;
                break;
            case SAFE_OP_OR:
                NewValue |= OPData;
                break;
            case SAFE_OP_NOT:
                NewValue = ~NewValue;
                break;
            case SAFE_OP_XOR:
                NewValue ^= OPData;
                break;
            case SAFE_OP_SHIFT_LEFT:
                NewValue <<= OPData;
                break;
            case SAFE_OP_SHIFT_RIGHT:
                NewValue >>= OPData;
                break;            
            default:
                assert_param(0);
                break;
        }   
        
    }while(__STREXW_S(NewValue, (unsigned long volatile*)pVar) == 1);    
}

/** @brief 16 bit variable for safe read write
  * @param pVar   pointer to variable which need modify
  * @param OPType variable operation mode
  *      @arg @b SAFE_OP_ADD  *pVar += OPData
  *      @arg @b SAFE_OP_AND  *pVar &= OPData
  *      @arg @b SAFE_OP_OR   *pVar |= OPData
  *      @arg @b SAFE_OP_NOT  *pVar = ~(*pVar)
  *      @arg @b SAFE_OP_XOR  *pVar ^= OPData
  *      @arg @b SAFE_OP_SHIFT_LEFT  *pVar <<= OPData
  *      @arg @b SAFE_OP_SHIFT_RIGHT *pVar >>= OPData
  * @param OPData operation data
  * @retval None
  * @note This function used for protect read then write operation if it be interrupted
  */ 
void safe_readwrite16(uint16_t* pVar, SAFE_OPMODE_Type OPType, uint16_t OPData)
{
    __IO uint16_t NewValue;    
    
    assert_param(IS_SAFE_OPMODE_Type(OPType));
    
    do{
        NewValue = __LDREXH_S(pVar);
        switch(OPType)
        {
            case SAFE_OP_ADD:
                NewValue += (int16_t)OPData;
                break;
            case SAFE_OP_AND:
                NewValue &= OPData;
                break;
            case SAFE_OP_OR:
                NewValue |= OPData;
                break;
            case SAFE_OP_NOT:
                NewValue = ~NewValue;
                break;
            case SAFE_OP_XOR:
                NewValue ^= OPData;
                break;
            case SAFE_OP_SHIFT_LEFT:
                NewValue <<= OPData;
                break;
            case SAFE_OP_SHIFT_RIGHT:
                NewValue >>= OPData;
                break;            
            default:
                assert_param(0);
                break;
        }   
        
    }while(__STREXH_S(NewValue, pVar) == 1);    
}

/** @brief 8 bit variable for safe read write
  * @param pVar   pointer to variable which need modify
  * @param OPType variable operation mode
  *      @arg @b SAFE_OP_ADD  *pVar += OPData
  *      @arg @b SAFE_OP_AND  *pVar &= OPData
  *      @arg @b SAFE_OP_OR   *pVar |= OPData
  *      @arg @b SAFE_OP_NOT  *pVar = ~(*pVar)
  *      @arg @b SAFE_OP_XOR  *pVar ^= OPData
  *      @arg @b SAFE_OP_SHIFT_LEFT  *pVar <<= OPData
  *      @arg @b SAFE_OP_SHIFT_RIGHT *pVar >>= OPData
  * @param OPData operation data
  * @retval None
  * @note This function used for protect read then write operation if it be interrupted
  */ 
void safe_readwrite8(uint8_t* pVar, SAFE_OPMODE_Type OPType, uint8_t OPData)
{
    __IO uint8_t NewValue;    
    
    assert_param(IS_SAFE_OPMODE_Type(OPType));
    
    do{
        NewValue = __LDREXB_S(pVar);
        switch(OPType)
        {
            case SAFE_OP_ADD:
                NewValue += (int8_t)OPData;
                break;
            case SAFE_OP_AND:
                NewValue &= OPData;
                break;
            case SAFE_OP_OR:
                NewValue |= OPData;
                break;
            case SAFE_OP_NOT:
                NewValue = ~NewValue;
                break;
            case SAFE_OP_XOR:
                NewValue ^= OPData;
                break;
            case SAFE_OP_SHIFT_LEFT:
                NewValue <<= OPData;
                break;
            case SAFE_OP_SHIFT_RIGHT:
                NewValue >>= OPData;
                break;            
            default:
                assert_param(0);
                break;
        }   
        
    }while(__STREXB_S(NewValue, pVar) == 1);        
}

/** @brief Start protect the critical codes by disable interrupts
  * @retval uint32_t register PRIMASK value before disable interrupts
  *         This value needed while leave critical section
  **/
uint32_t enter_critical_section(void)
{
    uint32_t sr = __get_PRIMASK();
    
	__disable_irq();
    
    return (uint32_t)(sr);
}

/** @brief Stop protect the critical codes by enable interrupts
  * @param saveSR PRIMASK value need to recovery
  * @retval None
  **/
void leave_critical_section(uint32_t saveSR)
{
    __set_PRIMASK(saveSR);
}



/** @brief switch run rights in thread mode
  * @param modeType which mode need switch to 
  *    @arg @b THREAD_PRIVILEGED  run in Privileged Mode
  *    @arg @b THREAD_USER        run in User Mode
  * @retval None
  * @note  This function must run in Handler Mode or Privileged Mode.
  *        If run in user mode to switch to Privileged mode must switch to Handler Mode first(by interrupt)
  **/
void switch_runmode(THREAD_MODE_Type modeType)
{
    uint32_t control;
    
    assert_param(IS_THREAD_MODE_Type(modeType));
    
    control = __get_CONTROL();    
    
    if(modeType == THREAD_PRIVILEGED)
    {
        control &= ~(1<<0);
    }
    else if(modeType == THREAD_USER)
    {
        control |= (1<<0);
    }
    __set_CONTROL(control);    
}



#if defined ( __CC_ARM )
/** @brief Get Interrupted LR register 
  * @param item operated as R0 register.
  * @retval uint32_t LR register value
  */
static uint32_t get_LR(uint32_t item) __attribute__((noinline));
static __ASM uint32_t get_LR(uint32_t item)
{
    ldr r0,[sp,#12];
    bx lr
}
#elif defined ( __GNUC__ )
static uint32_t get_LR(uint32_t item) __attribute__((noinline));
static uint32_t get_LR(uint32_t item)
{
    __ASM volatile ("ldr r0,[sp,#12]");
    return item;
}

#elif defined ( __ICCARM__ )
static uint32_t get_LR(uint32_t item)
{
    __ASM volatile ("ldr r0,[sp,#12]");
    return item;
}
#endif


/** @brief Get Interrupt Stack information must call it in ISR direct
  * @retval INTStack_Typedef  struct pointer to the interrupt saved stack
  * @note This function must called in Interrupt ISR direct and the ISR no other variables
  */
INTStack_Typedef* INT_GetStackInfo(void)
{
    uint32_t stkAddr;
    uint32_t __regLR ;

    /* Get Interrupt LR register : must be 0xFFFFxxx*/
    __regLR = get_LR(0);
    
    if(__regLR & 4) 
    {/* Bit2 == 1 : PSP (Caller in user mode)*/
        stkAddr = __get_PSP();
    }
    else
    {/* Bit2 == 0 : MSP(Caller in hander mode)*/
        stkAddr = __get_MSP() + 16;        
    }

    return (INTStack_Typedef*)stkAddr;       
}

/** @brief Parse the Interrupt Stack and get the SVCall Number
  * @param pStackInfo  Interrupt Saved stack pointer
  * @retval uint8_t SVCall Number
  * @note This function must called in Interrupt ISR direct and the ISR no other variables
  */
uint8_t SVCall_GetCallNO(INTStack_Typedef* pStackInfo)
{
   return *((uint8_t*)(pStackInfo->PC - 2));
}


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


