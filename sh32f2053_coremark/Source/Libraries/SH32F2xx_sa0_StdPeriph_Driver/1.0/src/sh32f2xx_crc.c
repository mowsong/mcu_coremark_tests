/**
  ******************************************************************************
  * @file    sh32f2xx_crc.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provide APIs for using CRC module
  *         
  * @verbatim
  *
  *          ===================================================================
  *                                  How to use this driver
  *          ===================================================================
  *          
  *
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
#include "sh32f2xx_crc.h"
   
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/* CRC  Module----------------------------------------------------------*/
/** @defgroup CRC_MODULE  CRC 
   *  CRC Calculate Mode
  * @{
  */ 
        
/** @defgroup  CRC_Group_Pub_Funcs  Public Functions
 *  @brief   CRC Public Functions
 *
  * @{
  */

/**
  *@code   Example
      // Please make sure compiler option --c99 is valied
      // Otherwise this method cannot supported
      #if defined (USE_C99_COMPILER_OPTION)
      CRC_InitTypeDef InitCfg = {  
                               .Reload            = ENABLE,      // Reload the Initial value to calculate unit: ENABLE or DISABLE
                               .Mode              = CRC_MODE_32, // POLY mode selection: CRC_MODE_32, CRC_MODE_16,CRC_MODE_CITT,CRC_MODE_8
                               .ComplementInput   = DISABLE,     // Complement the input value  then to calculate : ENABLE or DISABLE
                               .BitRevertInput    = DISABLE,     // Reverse the input value by bit then to calculate: ENABLE or DISABLE
                               .ByteRevertInput   = DISABLE,     // Reverse the input value by byte then to calculate: ENABLE or DISABLE
                               .ComplementOutput  = DISABLE,     // Complement the result value : ENABLE or DISABLE
                               .BitRevertOutput   = DISABLE,     // Reverse the result value by bit : ENABLE or DISABLE 
                               .ByteRevertOutput  = DISABLE,     // Reverse the result value by byte : ENABLE or DISABLE
                               .InitialVal        = 0xFFFFFFFF,  // Inital Value
                                };
       #else
       CRC_InitTypeDef InitCfg = {  
                               ENABLE,      // Reload the Initial value to calculate unit: ENABLE or DISABLE
                               0,           // Reserved
                               CRC_MODE_32, // POLY mode selection: CRC_MODE_32, CRC_MODE_16,CRC_MODE_CITT,CRC_MODE_8
                               DISABLE,     // Complement the input value  then to calculate : ENABLE or DISABLE
                               DISABLE,     // Reverse the input value by bit then to calculate: ENABLE or DISABLE
                               DISABLE,     // Reverse the input value by byte then to calculate: ENABLE or DISABLE
                               DISABLE,     // Complement the result value : ENABLE or DISABLE
                               DISABLE,     // Reverse the result value by bit : ENABLE or DISABLE 
                               DISABLE,     // Reverse the result value by byte : ENABLE or DISABLE
                               0,           // Reserved
                               0xFFFFFFFF,  // Inital Value
                                };       
       #endif       
       CRC_Init(&InitCfg);
       
   *@endcode
   */
/**
  * @brief  CRC Module Initialization  
  * @param  InitCfg: Input CRC Configuration
  * @retval None
  */
void CRC_Init(const CRC_InitTypeDef* InitCfg)
{
    assert_param(InitCfg);

    /* set initial value */
    CRC->INIT = InitCfg->InitialVal;

    /* Set configuration and reload */
    CRC->CR.V32 = *((uint32_t*)InitCfg);

}

/* CRC fill the initial Structure */
/**
  * @brief  Fill the initial Structure  
  * @param  InitStruct : structure which need filled
  * @retval None
  */
void CRC_StructInit(CRC_InitTypeDef* InitStruct)
{
    CRC_InitTypeDef CRCDefault={0};
    *InitStruct = CRCDefault;
}




/**
  * @brief  Deinitializes the CRC registers to their default reset 
  * @retval None
  */
void CRC_Reset(void)
{
   RCC_AHBPeriphReset(RCC_AHB_CRC);
}


/**
  * @brief  Calcuate one data's CRC value
  * @param  Data Input data to calculate.
  * @param  InputType  Input data's format.Can be of below
  *    @arg @b CRC_INPUT_WORD       one word data to calculate
  *    @arg @b CRC_INPUT_HALFWORD   half word to calculate
  *    @arg @b CRC_INPUT_BYTE       one byte to calculate     
  * @retval uint32_t  one block data's CRC value
  */
uint32_t CRC_CalcCRC(uint32_t Data, CRC_INPUT_Type InputType)
{
    assert_param(IS_CRC_INPUTTYPE(InputType));
    
    switch(InputType)
    {
        case CRC_INPUT_WORD:
            CRC->DR = Data;
            break;
        case CRC_INPUT_HALFWORD:
            *((uint16_t*)&(CRC->DR)) = ((uint16_t)(Data&0xFFFF));
            break;
        case CRC_INPUT_BYTE:
            *((uint8_t*)&(CRC->DR)) = ((uint8_t)(Data&0xFF));
            break;
        default:
            assert_break(0);
            break;
    }
    
    /* return the CRC result */
    return (uint32_t)(CRC->DR);
}

/**
  * @brief  Calcuate one block data's CRC value
  * @param  InBuffer Input data buffer to calculate.
  * @param  Count  size of input buffer. data unit is assigned by InputType
  * @param  InputType  Input data's format.Can be of below
  *    @arg @b CRC_INPUT_WORD       one word data to calculate
  *    @arg @b CRC_INPUT_HALFWORD   half word to calculate
  *    @arg @b CRC_INPUT_BYTE       one byte to calculate     
  * @retval uint32_t one block data's CRC value
  */
uint32_t CRC_CalcBlockCRC(const uint8_t* InBuffer, uint32_t Count, CRC_INPUT_Type InputType)
{
    uint32_t i;
    const uint8_t* p;
    assert_param(Count > 0);
    assert_param(IS_CRC_INPUTTYPE(InputType));
    
    p = InBuffer;
    for(i = 0; i < Count; i++)
    {
         switch(InputType)
        {
            case CRC_INPUT_WORD:
                CRC->DR = *((uint32_t*)p);
                p += 4;
                break;
            case CRC_INPUT_HALFWORD:
                *((uint16_t*)&(CRC->DR)) = *((uint16_t*)p);
                p += 2;
                break;
            case CRC_INPUT_BYTE:
                *((uint8_t*)&(CRC->DR)) = *((uint8_t*)p);
                p++;
                break;
            default:
                assert_break(0);
                p++;
                break;
        }
    }

    /* return the CRC result */
    return (uint32_t)(CRC->DR);
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


