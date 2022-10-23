/**
  ******************************************************************************
  * @file    sh32f2xx_uart.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provides firmware functions to manage the following 
  *         
  *  @verbatim
  *
  *          ===================================================================
  *                                   How to use this driver
  *          ===================================================================
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
#include "sh32f2xx_uart.h"
#include "sh32f2xx_rcc.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup UART 
  * @brief UART driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup UART_Private_Functions
  * @{
  */ 

/** @defgroup UART_Group1 Initialization and Configuration
 *  @brief    Initialization and Configuration
 *
@verbatim    
 ===============================================================================
                      Initialization and Configuration
 ===============================================================================  
   
@endverbatim
  * @{
  */

/**
  * @brief  Reset the UARTx peripheral registers to their default reset values.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @retval None
  */
void UART_Reset(UART_TypeDef* UARTx)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));
        
    if (UARTx == UART1)
    {
        RCC_APB1PeriphReset(RCC_APB1_UART1);
    }
    else if (UARTx == UART2)
    {
        RCC_APB1PeriphReset(RCC_APB1_UART2);
    }
    else
    {
        if (UARTx == UART3)
        {
            RCC_APB1PeriphReset(RCC_APB1_UART3);
        }
    }
}

/**
  * @brief  Fills each UART_InitStruct member with its default value.
  * @param  UART_InitStruct : pointer to a @ref UART_InitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
void UART_StructInit(UART_InitTypeDef* UART_InitStruct)
{
    UART_InitStruct->UART_Mode = UART_Mode_0;
    UART_InitStruct->UART_BaudRate = 0x00;
    UART_InitStruct->UART_DataLength = UART_DataLength_8Bit;
    UART_InitStruct->UART_StopBits = UART_StopBits_None;
    UART_InitStruct->UART_Parity = UART_Parity_None;
    UART_InitStruct->UART_Enable = 0x00;
}

/**
  * @brief  Initialize the UART peripheral according to the specified parameters
  *         in the UART_InitStruct.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @param  UART_InitStruct: pointer to a @ref UART_InitTypeDef structure that contains
  *         the configuration information for the specified UART peripheral.
  * @retval None
  */
void UART_Init(UART_TypeDef* UARTx,UART_InitTypeDef* UART_InitStruct)
{
    uint32_t temp = 0x00;
    uint32_t baudRate = 0;
    RCC_Clocks_TypeDef RCC_ClocksStatus;

    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));
    assert_param(IS_UART_MODE(UART_InitStruct->UART_Mode));

    /* Set UART mode */
    UARTx->CR.V32 &= ~UART_CR_SM_Msk;
    UARTx->CR.V32 |= UART_InitStruct->UART_Mode;

    /* UART baud rate configuration */
    if ((UART_InitStruct->UART_Mode == UART_Mode_1) \
        || (UART_InitStruct->UART_Mode == UART_Mode_3))
    {
        RCC_GetClocksFreq(&RCC_ClocksStatus);
        baudRate = RCC_ClocksStatus.pclk1Freq / 16 / UART_InitStruct->UART_BaudRate - 1;
        temp = RCC_ClocksStatus.pclk1Freq / UART_InitStruct->UART_BaudRate \
                 - 16 * (baudRate + 1);
        baudRate += temp << 16;
    }

    if (UART_InitStruct->UART_Mode == UART_Mode_0)
    {
        assert_param(IS_UART_MODE_0_BAUDRATE(UART_InitStruct->UART_BaudRate));
        assert_param(IS_UART_MODE_0_STOP_BITS(UART_InitStruct->UART_StopBits));
        assert_param(IS_UART_MODE_01_DATALENGTH(UART_InitStruct->UART_DataLength));
        assert_param(IS_UART_MODE_01_PARITY(UART_InitStruct->UART_Parity));

        /* Set the UART baud rate */
        UARTx->CR.V32 &= ~UART_CR_SM2_Msk;
        UARTx->CR.V32 |= ((UART_InitStruct->UART_BaudRate >> 19) & UART_CR_SM2_Msk); 

        /* Set the Parity bit */
        UARTx->CR.V32 &= (~(UART_CR_PCE_Msk | UART_CR_PS_Msk));
        UARTx->CR.V32 |= UART_InitStruct->UART_Parity;
    }
    else if (UART_InitStruct->UART_Mode == UART_Mode_1)
    {
        assert_param(IS_UART_MODE_13_BAUDRATE(UART_InitStruct->UART_BaudRate));
        assert_param(IS_UART_MODE_123_STOP_BITS(UART_InitStruct->UART_StopBits));
        assert_param(IS_UART_MODE_01_DATALENGTH(UART_InitStruct->UART_DataLength));
        assert_param(IS_UART_MODE_01_PARITY(UART_InitStruct->UART_Parity));

        UARTx->BRT.V32 = baudRate;
        UARTx->CR.V32 |= UART_CR_SBRTEN_Msk;

        UARTx->CR.V32 &= ~UART_CR_STOP_Msk;
        UARTx->CR.V32 |= UART_InitStruct->UART_StopBits - 1;
        
        /* Set the Parity bit */
        UARTx->CR.V32 &= (~(UART_CR_PCE_Msk | UART_CR_PS_Msk));
        UARTx->CR.V32 |= UART_InitStruct->UART_Parity;        
    }
    else if (UART_InitStruct->UART_Mode == UART_Mode_2)
    {
        assert_param(IS_UART_MODE_2_BAUDRATE(UART_InitStruct->UART_BaudRate));
        assert_param(IS_UART_MODE_123_STOP_BITS(UART_InitStruct->UART_StopBits));
        assert_param(IS_UART_MODE_23_DATALENGTH(UART_InitStruct->UART_DataLength));
        assert_param(IS_UART_MODE_23_PARITY(UART_InitStruct->UART_Parity));

        UARTx->CR.V32 &= ~UART_CR_SMOD_Msk;
        UARTx->CR.V32 |= UART_InitStruct->UART_BaudRate;
        
        UARTx->CR.V32 &= ~UART_CR_STOP_Msk;
        UARTx->CR.V32 |= UART_InitStruct->UART_StopBits - 1;

        /* Set the Parity bit */
        UARTx->CR.V32 &= (~(UART_CR_PCE_Msk | UART_CR_PS_Msk));
        UARTx->CR.V32 |= UART_InitStruct->UART_Parity;         
    }
    else
    {
        assert_param(IS_UART_MODE_13_BAUDRATE(UART_InitStruct->UART_BaudRate));
        assert_param(IS_UART_MODE_123_STOP_BITS(UART_InitStruct->UART_StopBits));
        assert_param(IS_UART_MODE_23_DATALENGTH(UART_InitStruct->UART_DataLength));
        assert_param(IS_UART_MODE_23_PARITY(UART_InitStruct->UART_Parity));

        UARTx->BRT.V32 = baudRate;

        UARTx->CR.V32 |= UART_CR_SBRTEN_Msk;
        
        UARTx->CR.V32 &= ~UART_CR_STOP_Msk;
        UARTx->CR.V32 |= UART_InitStruct->UART_StopBits - 1;

        /* Set the Parity bit */
        UARTx->CR.V32 &= (~(UART_CR_PCE_Msk | UART_CR_PS_Msk));
        UARTx->CR.V32 |= UART_InitStruct->UART_Parity;         
    }
        


    UARTx->CR.V32 &= ~(UART_CR_TEN_Msk | UART_CR_REN_Msk);
    /* Enable UART transmit or receive */
    UARTx->CR.V32 |= UART_InitStruct->UART_Enable; 
}

/**
  * @brief  Check whether the specified UART flag is set or not.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @param  UART_Flag: specifie the flag to check.
  *          This parameter can be one of the following values:
  *            @arg UART_FLAG_RI
  *            @arg UART_FLAG_TI
  *            @arg UART_FLAG_TC
  *            @arg UART_FLAG_TXCOL
  *            @arg UART_FLAG_RXOV
  *            @arg UART_FLAG_FE
  *            @arg UART_FLAG_PE
  *            @arg UART_FLAG_LBD  
  * @retval The new state of UART_Flag(SET or RESET).
  */
FlagStatus UART_GetFlagStatus(UART_TypeDef* UARTx, uint8_t UART_Flag)
{
    FlagStatus bitStatus;
    
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));    
    assert_param(IS_UART_GET_ONE_FLAG(UART_Flag));
    
    if ((UARTx->FR.V32 & UART_Flag) != RESET)
    {
        bitStatus = SET;
    }
    else
    {
        bitStatus = RESET;
    }
    return bitStatus;
}


/**
  * @brief  Clear the UART's pending flags.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @param  UART_Flag: Specifie the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg UART_FLAG_RI
  *            @arg UART_FLAG_TI
  *            @arg UART_FLAG_TC
  *            @arg UART_FLAG_TXCOL
  *            @arg UART_FLAG_RXOV
  *            @arg UART_FLAG_FE
  *            @arg UART_FLAG_PE
  *            @arg UART_FLAG_LBD  
  * @retval None
  */
void UART_ClearFlag(UART_TypeDef* UARTx, uint8_t UART_Flag)
{    
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));
    assert_param(IS_UART_FLAG(UART_Flag));
    
    UARTx->FR.V32 = ((uint32_t)UART_Flag << 16);
}

/**
  * @brief  Enable or disable the specified UART's interrupts.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @param  UART_INT: specifie the UART interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg UART_INT_RX
  *            @arg UART_INT_TX
  *            @arg UART_INT_TC
  *            @arg UART_INT_LBD  
  * @param  NewState: New state of UART interrupt.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void UART_INTConfig(UART_TypeDef* UARTx, uint8_t UART_INT, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));
    assert_param(IS_UART_INT(UART_INT));
    assert_param(IS_FUNCTION_STATE(NewState));
    
    if (NewState != DISABLE)
    {
        UARTx->CR.V32 |= UART_INT;
    }
    else
    {
        UARTx->CR.V32 &= ~UART_INT;
    }
}

/**
  * @brief  Enable or disable the UART's DMA request sources.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @param  UART_DMASource: specifies the DMA request.
  *          This parameter can be any combination of the following values:
  *            @arg UART_DMA_RX
  *            @arg UART_DMA_TX
  * @param  NewState: new state of the DMA Request sources.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void UART_DMAConfig(UART_TypeDef* UARTx, uint32_t UART_DMASource, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));
    assert_param(IS_UART_DMA_SOURCE(UART_DMASource));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the UART's DMA request sources */
        UARTx->CR.V32 |= UART_DMASource;
    }
    else
    {
        /* Disable the UART's DMA request sources */
        UARTx->CR.V32 &= ~UART_DMASource;
    }
}

/**
  * @brief  Transmit single data through the UARTx peripheral.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @param  SendData: the data to transmit.
  * @retval None
  */
void UART_SendData(UART_TypeDef* UARTx, uint16_t SendData)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));

    /* Transmit data */
    UARTx->TDR = SendData;   
}

/**
  * @brief  Return the most recent received data by the UARTx peripheral.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @retval The received data.
  */
uint16_t UART_ReceiveData(UART_TypeDef* UARTx)
{
    uint16_t tempData = 0;
    uint16_t tempReg = 0;

    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));

    tempReg = (uint16_t)(UARTx->CR.V32 & UART_CR_SM_Msk);
    
    if ((tempReg == UART_Mode_0) || (tempReg == UART_Mode_1))
    {
        tempData = UARTx->RDR;
    }
    else
    {
        tempData = (UARTx->CR.V32 & UART_CR_RB8_Msk);
        tempData |= UARTx->RDR;
    }
    return tempData;
}
 

/**
  * @brief  Return the most recent received 8bits data by the UATTx peripheral.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @retval The received 8bits data.
  */
uint16_t UART_ReceiveData8(UART_TypeDef* UARTx)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));

    return UARTx->RDR;
}

/**
  * @brief  Return the most recent received 9bits data by the UATTx peripheral.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @retval The received 9bits data.
  */
uint16_t UART_ReceiveData9(UART_TypeDef* UARTx)
{
    uint16_t tempData = 0;

    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));

    tempData = (UARTx->CR.V32 & UART_CR_RB8_Msk);
    tempData |= UARTx->RDR;

    return tempData;
}

/**
  * @brief  Set the UARTx address for master-slave communication.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @param  UART_Address: specifie the address to set.
  *          This parameter can be 0 to 0xFF.
  * @retval None
  */
void UART_SetAddress(UART_TypeDef* UARTx, uint8_t UART_Address)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));

    UARTx->ADDR.V32 &= ~UART_ADDR_SADDR_Msk;
    UARTx->ADDR.V32 |= UART_Address; 
}

/**
  * @brief  Configure the UARTx address shield bits for master-slave communication.
  * @param  UARTx: where x can be 1,2 or 3 to select the UART peripheral.
  * @param  UART_AddressShield: specifie the address shield bits.
  *          This parameter can be 0 to 0xFF.
  * @retval None
  */
void UART_AddressShieldConfig(UART_TypeDef* UARTx, uint8_t UART_AddressShield)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));

    UARTx->ADDR.V32 &= ~UART_ADDR_SMAR_Msk;
    UARTx->ADDR.V32 |= (uint16_t)UART_AddressShield << 8; 
}

/**
  * @brief  Open or close the UART's LIN mode.
  * @param  UARTx: where x can be 1, 2, 3 to select the UART peripheral.
  * @param  OnOffState: state of the UART LIN mode.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void UART_LINOnOff(UART_TypeDef* UARTx, CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));
    assert_param(IS_CMD_STATE(OnOffState));

    if (OnOffState != OFF)
    {
        UARTx->CR.V32 |= UART_CR_LINEN_Msk; 
    }
    else
    {
        UARTx->CR.V32 &= (~UART_CR_LINEN_Msk);
    }
}

/**
  * @brief  Open or close the UART's send module.
  * @param  UARTx: where x can be 1, 2, 3 to select the UART peripheral.
  * @param  OnOffState: state of the UART send module.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void UART_SendOnOff(UART_TypeDef* UARTx, CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));
    assert_param(IS_CMD_STATE(OnOffState));

    if (OnOffState != OFF)
    {
        UARTx->CR.V32 |= UART_CR_TEN_Msk; 
    }
    else
    {
        UARTx->CR.V32 &= (~UART_CR_TEN_Msk);
    }
}

/**
  * @brief  Open or close the UART's receive module.
  * @param  UARTx: where x can be 1, 2, 3 to select the UART peripheral.
  * @param  OnOffState: state of the UART receive module.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void UART_ReceiveOnOff(UART_TypeDef* UARTx, CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));
    assert_param(IS_CMD_STATE(OnOffState));

    if (OnOffState != OFF)
    {
        UARTx->CR.V32 |= UART_CR_REN_Msk; 
    }
    else
    {
        UARTx->CR.V32 &= (~UART_CR_REN_Msk);
    }
}

/**
  * @brief  Enable or Disable the UART's LIN mode.
  * @param  UARTx: where x can be 1, 2, 3 to select the UART peripheral.
  * @param  UART_LINDetectLength: specifie the LIN break detect length.
  *          This parameter can be one of the following values:
  *            @arg UART_LINDetectLength_10b
  *            @arg UART_LINDetectLength_11b
  * @retval None
  */
void UART_LINDetectLengthConfig(UART_TypeDef* UARTx, uint16_t UART_LINDetectLength)
{
    uint32_t temp = 0;
    
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));
    assert_param(IS_UART_LIN_DETECT_LENGTH(UART_LINDetectLength));

    temp = UARTx->CR.V32;
    temp &= ~UART_CR_LBDL_Msk;
    temp |= UART_LINDetectLength;
    UARTx->CR.V32 = temp;
}


/**
  * @brief  Transmits break characters.
  * @param  UARTx: where x can be 1, 2, 3 to select the UART peripheral.
  * @retval None
  */
void UART_SendBreak(UART_TypeDef* UARTx)
{
    /* Check the parameters */
    assert_param(IS_UART_ALL_PERIPH(UARTx));

    /* Send break characters */
    UARTx->CR.V32 |= UART_CR_SBK_Msk;
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

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/


