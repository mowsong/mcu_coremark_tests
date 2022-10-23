/**
  ******************************************************************************
  * @file    sh32f2xx_spi.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provide APIs for using SPI module
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
#include "sh32f2xx_spi.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/* SPI  Module----------------------------------------------------------*/
/** @defgroup SPI_MODULE  SPI 
   *  SPI Calculate Mode
  * @{
  */ 
        
/** @defgroup  SPI_Group_Pub_Funcs  Public Functions
 *  @brief   SPI Public Functions
 *
  * @{
  */
      

/**@code Example
    #if defined ( USE_C99_COMPILER_MODE )
    // Please make sure compiler option --c99 is valied
    // Otherwise this method cannot supported
    const SPI_InitTypeDef SPI_FlashCfg = {
                                        .Clock          = SPR_PCKL1_DIV_8,   // SPI clock option @ref SPI_SPR_Type
                                        .SSPin          = PIN_SS_DISABLE,    // SS Pin in master mode= @ref SPI_SSDIS_Type= PIN_SS_ENABLE or PIN_SS_DISABLE
                                        .ClkIdleState   = CLK_IDLE_HIGH,     // CLK Pin status while in IDLE= CLK_IDLE_LOW or CLK_IDLE_HIGH 
                                        .CaptureEdge    = CAP_CLK_EDGE2,     // capture signal time= CAP_CLK_EDGE1 or CAP_CLK_EDGE2 
                                        .MasterOrSlave  = SPI_MASTER,        // SPI mode = SPI_SLAVE or SPI_MASTER
                                        .Direction      = SPI_TRANS_MSB,     // SPI transmit direction type:SPI_TRANS_MSB or SPI_TRANS_LSB  
                                        .DataSize       = SPI_DATASIZE_8b,   // SPI transmit data length= SPI_DATASIZE_8b or SPI_DATASIZE_16b
                                        .ReceiveIE      = ENABLE,            // SPI receive interrupt= ENABLE or DISABLE 
                                        .SendIE         = ENABLE,            // SPI send interrupt enable bit 
                                        .ReceiveDMA     = DISABLE,           // SPI receive DMA enable bit 
                                        .SendDMA        = DISABLE,           // SPI send DMA enable bit 
                                        .Enable         = OFF,           // SPI enable bit 
                                        .SlaveTransMode = SPI_SLAVE_NORMAL,  // SPI transmit mode for slave mode:SPI_SLAVE_NORMAL or SPI_SLAVE_FAST 
                                        };


    #else          
    const SPI_InitTypeDef SPI_FlashCfg = {
                                        SPR_PCKL1_DIV_8,   // SPI clock option @ref SPI_SPR_Type
                                        PIN_SS_DISABLE,    // SS Pin in master mode= @ref SPI_SSDIS_Type= PIN_SS_ENABLE or PIN_SS_DISABLE
                                        CLK_IDLE_HIGH,     // CLK Pin status while in IDLE= CLK_IDLE_LOW or CLK_IDLE_HIGH 
                                        CAP_CLK_EDGE2,     // capture signal time= CAP_CLK_EDGE1 or CAP_CLK_EDGE2 
                                        SPI_MASTER,        // SPI mode = SPI_SLAVE or SPI_MASTER
                                        SPI_TRANS_MSB,     // SPI transmit direction type:SPI_TRANS_MSB or SPI_TRANS_LSB  
                                        SPI_DATASIZE_8b,   // SPI transmit data length= SPI_DATASIZE_8b or SPI_DATASIZE_16b
                                        ENABLE,            // SPI receive interrupt= ENABLE or DISABLE 
                                        ENABLE,            // SPI send interrupt enable bit 
                                        DISABLE,           // SPI receive DMA enable bit 
                                        DISABLE,           // SPI send DMA enable bit 
                                        OFF,               // SPI enable bit 
                                        SPI_SLAVE_NORMAL,  // SPI transmit mode for slave mode:SPI_SLAVE_NORMAL or SPI_SLAVE_FAST 
                                        };

    #endif     
    RCC_APB1PeriphClockOnOff(RCC_APB1_SPI1,ON);    
    SPI_Init(SPI1,&SPI_CFG);
  *@endcode
  * @brief  Initializes the SPIx peripheral according to the specified 
  *   parameters in the InitCfg.
  * @param  SPIx SPI device: SPI1 or SPI2
  * @param  InitCfg  pointer to a SPI_InitTypeDef structure that
  *   contains the configuration information for the specified SPI peripheral.
  * @retval None
  */
  
void SPI_Init(SPI_TypeDef* SPIx, const SPI_InitTypeDef* InitCfg)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_SPR_Type(InitCfg->Clock));
    
    SPIx->CR.V32 = *((uint32_t*)InitCfg);    
}

/**
  * @brief  Deinitializes the SPIx peripheral registers to their default
  *   reset values 
  * @param  SPIx: where x can be 1, 2 to select the SPI peripheral.
  * @retval None
  */
void SPI_Reset(SPI_TypeDef* SPIx)
{
  /* Check the parameters */
  assert_param(IS_SPI_MODULE(SPIx));

  if (SPIx == SPI1)
  {
    /* Reset SPI1 module */
    RCC_APB1PeriphReset(RCC_APB1_SPI1);

  }
  else if (SPIx == SPI2)
  {
    /* Reset SPI2 module */
    RCC_APB1PeriphReset(RCC_APB1_SPI2);
  }
}

/**
  * @brief  Fills each InitStruct member with its default value.
  * @param  InitStruct : pointer to a SPI_InitTypeDef structure which will be initialized.
  * @retval None
  */
void SPI_StructInit(SPI_InitTypeDef* InitStruct)
{
    const SPI_InitTypeDef defaultSPICfg = {0};
   *InitStruct = defaultSPICfg;
}


/**
  * @brief  Enables or disables the specified SPI peripheral.
  * @param  SPIx: where x can be 1, 2 to select the SPI peripheral.
  * @param  OnOffState: new state of the SPIx peripheral. 
  *   This parameter can be: ON or OFF.
  * @retval None
  */
void SPI_OnOff(SPI_TypeDef* SPIx, CmdState OnOffState)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_CMD_STATE(OnOffState));
    
    SPIx->CR.BIT.SPIEN = OnOffState;
}


/**
  * @brief  Selects the SPI mode for the specified SPI.
  * @param  SPIx: where x can be 1, 2 to select the SPI peripheral.
  * @param  SPIMode: new mode of the SPIx peripheral. 
  *   This parameter can be: SPI_MASTER or SPI_SLAVE
  * @retval None
  */
void SPI_ModeConfig(SPI_TypeDef* SPIx, SPI_MSTR_Type SPIMode)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_MSTR_Type(SPIMode));

    SPIx->CR.BIT.MSTR = SPIMode;
}


/**
  * @brief  Enables or disables the specified SPI interrupts.
  * @param  SPIx: where x can be
  *   - 1, 2 in SPI mode 
  * @param  SPIFunc: specifies the SPI interrupt source to be enabled or disabled. 
  *   This parameter can be any combination of following values:
  *     @arg @b SPI_FUNC_SEND:    Tx buffer empty interrupt mask
  *     @arg @b SPI_FUNC_RECEIVE: Rx buffer not empty interrupt mask
  * @param  NewState: new state of the specified SPI/I2S interrupt.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPI_INTConfig(SPI_TypeDef* SPIx, uint32_t SPIFunc, FunctionalState NewState)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_FUNC_COMBType(SPIFunc));
    assert_param(IS_FUNCTION_STATE(NewState));

    if((SPIFunc & SPI_FUNC_SEND) == SPI_FUNC_SEND)
        SPIx->CR.BIT.SPTIE = NewState;
        
    if((SPIFunc & SPI_FUNC_RECEIVE) == SPI_FUNC_RECEIVE)
        SPIx->CR.BIT.SPRIE = NewState;
    
}

/**
  * @brief  Enables or disables the SPIx DMA interface.
  * @param  SPIx: where x can be
  *   - 1, 2 in SPI mode 
  * @param  SPIFunc: specifies the SPI DMA transfer request to be enabled or disabled. 
  *   This parameter can be any combination of following values:
  *     @arg @b SPI_FUNC_SEND:    Tx buffer DMA transfer request
  *     @arg @b SPI_FUNC_RECEIVE: Rx buffer DMA transfer request
  * @param  NewState: new state of the selected SPI DMA transfer request.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPI_DMAConfig(SPI_TypeDef* SPIx, uint32_t SPIFunc, FunctionalState NewState)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_FUNC_COMBType(SPIFunc));
    assert_param(IS_FUNCTION_STATE(NewState));

    if((SPIFunc & SPI_FUNC_SEND) == SPI_FUNC_SEND)
        SPIx->CR.BIT.SPDMAT = NewState;
        
    if((SPIFunc & SPI_FUNC_RECEIVE) == SPI_FUNC_RECEIVE)
        SPIx->CR.BIT.SPDMAR = NewState;

}

/**
  * @brief  Enables or disables the NSS Pin for the selected SPI.
  * @param  SPIx: where x can be 1, 2 to select the SPI peripheral.
  * @param  SSState: new state of the SPIx NSS Pin
  *   This parameter can be: PIN_SS_ENABLE or PIN_SS_DISABLE.
  * @retval None
  */
void SPI_NSSConfig(SPI_TypeDef* SPIx, SPI_SSDIS_Type SSState)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_SSDIS_Type(SSState));
    
    SPIx->CR.BIT.SSDIS = SSState;
}

/**
  * @brief  Configures the data size for the selected SPI.
  * @param  SPIx: where x can be 1, 2 to select the SPI peripheral.
  * @param  DataSize: specifies the SPI data size.
  *   This parameter can be one of the following values:
  *     @arg @b SPI_DATASIZE_16b: Set data frame format to 16bit
  *     @arg @b SPI_DATASIZE_8b: Set data frame format to 8bit
  * @retval None
  */
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, SPI_SPDATL_Type DataSize)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_SPDATL_Type(DataSize));
    
    SPIx->CR.BIT.SPDATL = DataSize;
}

/**
  * @brief  Selects the data transfer direction  for the specified SPI.
  * @param  SPIx: where x can be 1, 2 to select the SPI peripheral.
  * @param  Direction: specifies the data transfer direction . 
  *   This parameter can be one of the following values:
  *     @arg @b SPI_TRANS_MSB: Selects Tx transmission direction
  *     @arg @b SPI_TRANS_LSB: Selects Rx receive direction
  * @retval None
  */
void SPI_DIRConfig(SPI_TypeDef* SPIx, SPI_DIR_Type Direction)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_DIR_Type(Direction));
    
    SPIx->CR.BIT.DIR = Direction;
}

/**
  * @brief  Selects the data transfer baudrate for the specified SPI.
  * @param  SPIx: where x can be 1, 2 to select the SPI peripheral.
  * @param  Baudrate: specifies the data transfer baudrate . 
  *   This parameter can be one of the following values:
  *     @arg @b SPR_PCKL1_DIV_2   : APB1 Clock / 2
  *     @arg @b SPR_PCKL1_DIV_4   : APB1 Clock / 4
  *     @arg @b SPR_PCKL1_DIV_8   : APB1 Clock / 8
  *     @arg @b SPR_PCKL1_DIV_16  : APB1 Clock / 16
  *     @arg @b SPR_PCKL1_DIV_32  : APB1 Clock / 32
  *     @arg @b SPR_PCKL1_DIV_64  : APB1 Clock / 64
  *     @arg @b SPR_PCKL1_DIV_128 : APB1 Clock / 128
  *     @arg @b SPR_PCKL1_DIV_256 : APB1 Clock / 256
  *     @arg @b SPR_PCKL1_DIV_512 : APB1 Clock / 512
  *     @arg @b SPR_PCKL1_DIV_1024: APB1 Clock / 1024
  * @retval None
  */
void SPI_SPRConfig(SPI_TypeDef* SPIx, SPI_SPR_Type Baudrate)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_SPR_Type(Baudrate));
    
    SPIx->CR.BIT.SPR = Baudrate;
}

/**
  * @brief  Selects the data transfer method in slave mode for the specified SPI.
  * @param  SPIx: where x can be 1, 2 to select the SPI peripheral.
  * @param  SlaveMode: specifies the data transfer method in slave mode. 
  *   This parameter can be one of the following values:
  *     @arg @b SPI_SLAVE_NORMAL   : Normal Mode
  *     @arg @b SPI_SLAVE_FAST     : Fast Mode
  * @retval None
  */
void SPI_SlaveModeConfig(SPI_TypeDef* SPIx, SPI_SPSFF_Type SlaveMode)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_SPSFF_Type(SlaveMode));

    SPIx->CR.BIT.SPSFF = SlaveMode;
}



/**
  * @brief  Checks whether the specified SPI interrupt has occurred or not.
  * @param  SPIx: where x can be
  *   - 1, 2 in SPI mode 
  * @param  SPIFunc: specifies the SPI interrupt source to check. 
  *   This parameter can be one of the following values:
  *     @arg @b SPI_FUNC_SEND: Transmit buffer empty interrupt.
  *     @arg @b SPI_FUNC_RECEIVE: Receive buffer not empty interrupt.
  * @retval FlagStatus The new state interrupt (SET or RESET).
  */
FlagStatus SPI_GetINTStatus(SPI_TypeDef* SPIx, SPI_FUNC_Type SPIFunc)
{
    FlagStatus status = RESET;
    uint32_t tmpVal;
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_FUNC_Type(SPIFunc));
    
    if(SPIFunc == SPI_FUNC_SEND)
    {
        tmpVal = SPIx->FR.BIT.SPTI;
        tmpVal &= SPIx->CR.BIT.SPTIE;
        status = (tmpVal ? SET : RESET); 
    }
    else if(SPIFunc == SPI_FUNC_RECEIVE)
    {
        tmpVal = SPIx->FR.BIT.SPRI;
        tmpVal &= SPIx->CR.BIT.SPRIE;
        status = (tmpVal ? SET : RESET); 
    }
    
    return status;
}

/**
  * @brief  Clears the SPIx interrupt pending bit.
  * @param  SPIx: where x can be
  *   - 1, 2 in SPI mode 
  * @param  SPIFunc: specifies the SPI interrupt pending bit to clear.
  *   This parameter can be any combination of following values:  
  *    @arg @b SPI_FUNC_SEND:    Transmit buffer empty interrupt.
  *    @arg @b SPI_FUNC_RECEIVE: Receive buffer not empty interrupt.
  * @retval None
  */
void SPI_ClearINTStatus(SPI_TypeDef* SPIx, uint32_t SPIFunc)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_FUNC_COMBType(SPIFunc));
    
    if((SPIFunc & SPI_FUNC_SEND) == SPI_FUNC_SEND)
        SPIx->FR.BIT.SPTIC = 1;
    
    if((SPIFunc & SPI_FUNC_RECEIVE) == SPI_FUNC_RECEIVE)
        SPIx->FR.BIT.SPRIC = 1;
}



/**
  * @brief  get SPI transmit flag
  * @param  SPIx SPI device: SPI1 or SPI2
  * @param  Flag  SPI flag mask
  *   This parameter can be one of following values:  
  *     @arg @b  SPI_FLAG_RECEIVE_INT: Interrupt flag for receive buffer ready
  *     @arg @b  SPI_FLAG_SEND_INT: Interrupt flag for send buffer empty
  *     @arg @b  SPI_FLAG_MODE_ERROR: More than one master in the transmit net
  *     @arg @b  SPI_FLAG_RECIEVE_OVER: Receive overflag
  *     @arg @b  SPI_FLAG_SEND_CONFLICT: Write conflict flag
  * @retval  FlagStatus RCC reset flag
  *     @arg @b  SET    the flag is setted
  *     @arg @b  RESET  the flag is cleared
  */
FlagStatus SPI_GetFlagStatus(SPI_TypeDef* SPIx, SPI_FLAG_Type Flag)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_FLAG_Type(Flag));
    
    return ((SPIx->FR.V32 & Flag) ? SET : RESET);
}

/**
  * @brief  clear SPI transmit flags
  * @param  SPIx SPI device: SPI1 or SPI2
  * @param  Flags   SPI flags clear bit mask
  *   This parameter can be any combination of following values:  
  *     @arg @b  SPI_FLAG_RECEIVE_INT: Clear interrupt flag for receive buffer ready
  *     @arg @b  SPI_FLAG_SEND_INT: Clear interrupt flag for send buffer empty
  *     @arg @b  SPI_FLAG_MODE_ERROR: Clear more than one master in the transmit net 
  *     @arg @b  SPI_FLAG_RECIEVE_OVER: Clear receive overflag 
  *     @arg @b  SPI_FLAG_SEND_CONFLICT: Clear write conflict flag
  * @retval None
  */
void SPI_ClearFlag(SPI_TypeDef* SPIx,uint32_t Flags)
{
    assert_param(IS_SPI_MODULE(SPIx));
    assert_param(IS_SPI_FLAG_Types(Flags));

    SPIx->FR.V32 = (Flags<<16);
}

/**
  * @brief  SPI send data
  * @param  SPIx SPI device: SPI1 or SPI2
  * @param  Data send data
  * @retval None
  */
void SPI_SendData(SPI_TypeDef* SPIx, uint16_t Data)
{
    assert_param(IS_SPI_MODULE(SPIx));

    SPIx->TDR = Data;
}

/**
  * @brief  SPI receive data
  * @param  SPIx SPI device: SPI1 or SPI2
  * @retval uint16_t received data
  */
uint16_t SPI_ReceiveData(SPI_TypeDef* SPIx)
{
    assert_param(IS_SPI_MODULE(SPIx));
        
    return SPIx->RDR;
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


