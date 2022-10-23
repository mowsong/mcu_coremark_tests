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
#include "sh32f2xx_twi.h"
     
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/* TWI  Module----------------------------------------------------------*/
/** @defgroup TWI_MODULE  TWI 
   *  TWI Calculate Mode
  * @{
  */ 
        
/** @defgroup  TWI_Group_Pub_Funcs  Public Functions
 *  @brief   TWI Public Functions
 *
  * @{
  */
      

/**@code Example:        
        #if defined ( USE_C99_COMPILER_MODE )
        // Please make sure compiler option --c99 is valied
        // Otherwise this method cannot supported
        const TWI_InitTypeDef TWIEECfg = { 
                                        .ZeroAddress   = DISABLE,          // Enable ZERO Address in Slave Mode: ENABLE or DISABLE 
                                        .Address7      = 0,                // 7 bits address                                
                                        .AddressMask   = 0,                // TWI address mask bit 0:Must match 1:Ignore                                
                                        .ACKorNACK     = TWI_AA_ACK,       // Response ACK or NACK 
                                        .SendStop      = DISABLE,          // Send stop signal: ENABLE or DISABLE 
                                        .SendStart     = DISABLE,          // Send start signal: ENABLE or DISABLE                                 
                                        .Clock         = TWI_CR_DIV_16,    // Clock prescale: @ref TWI_CLKRATE_Type 
                                        .BusTimeOut    = TWI_CNT_50000,    // Bus timeout count 
                                        .HighLevelTimeOutEnable = DISABLE, // High level timeout enable bit 
                                        .BusTimeOutEnable = DISABLE,       // Bus timeout enable bit 
                                        .IE               = DISABLE,       // Interrupt enable bit                                
                                        .Enable           = ENABLE,       // TWI Enalbe bit                                
                                        .BaudRate         = 32,            // Baudrate 
                                        .HighLevelTimeOut = 10,            // High level timeout count    
                                    };
        #else
        const TWI_InitTypeDef TWIEECfg = {
                                        DISABLE,        // Enable ZERO Address in Slave Mode: ENABLE or DISABLE 
                                        0,              // 7 bits address 
                                        0,              // reserved 
                                        0x0,            // TWI address mask bit 0:Must match 1:Ignore                                 
                                        0,              // reserved                                                                           
                                        TWI_AA_ACK,     // Response TWI_AA_ACK or TWI_AA_NACK 
                                        DISABLE,        // Send stop signal: ENABLE or DISABLE 
                                        DISABLE,        // Send start signal: ENABLE or DISABLE 
                                        0,              // reserved 
                                        TWI_CR_DIV_16,  // Clock prescale: @ref TWI_CLKRATE_Type 
                                        TWI_CNT_50000,  // Bus timeout count 
                                        DISABLE,        // High level timeout enable bit   
                                        DISABLE,        // Bus timeout enable bit 
                                        DISABLE,         // Interrupt enable bit    
                                        0,              // Reseved 
                                        ENABLE,        // TWI Enalbe bit 
                                        32,             // Baudrate 
                                        10              // High level timeout count 
                                     };
        #endif
        TWI_Init(&TWIEECfg);        
  * @endcode      
  * @brief  Initializes the TWIx peripheral according to the specified 
  *   parameters in the InitCfg.
  * @param  TWIx  TWI device: TWI1
  * @param  InitCfg  pointer to a TWI_InitTypeDef structure that
  *   contains the configuration information for the specified TWI peripheral.
  * @retval None
  */
  
void TWI_Init(TWI_TypeDef* TWIx, const TWI_InitTypeDef* InitCfg)
{
    uint8_t* pv;
    assert_param(IS_TWI_MODULE(TWIx));
    assert_param((InitCfg->BaudRate&0x80) == 0);
    pv = (uint8_t*)InitCfg;
    TWIx->BRT = InitCfg->BaudRate;    
    TWIx->HOC = InitCfg->HighLevelTimeOut;
    TWIx->ADDR.V32 = *((uint32_t*)pv);    
    TWIx->CR.V32 = (uint32_t)(*((uint16_t*)(pv+4)));  
}

/**
  * @brief  Deinitializes the TWIx peripheral registers to their default
  *   reset values 
  * @param  TWIx  TWI device: TWI1
  * @retval None
  */
void TWI_Reset(TWI_TypeDef* TWIx)
{
  /* Check the parameters */
  assert_param(IS_TWI_MODULE(TWIx));

  if (TWIx == TWI1)
  {
    /* Reset TWI1 module */
    RCC_APB1PeriphReset(RCC_APB1_TWI1);

  }
}

/**
  * @brief  Fills each InitStruct member with its default value.
  * @param  InitStruct : pointer to a TWI_InitTypeDef structure which will be initialized.
  * @retval None
  */
void TWI_StructInit(TWI_InitTypeDef* InitStruct)
{
    const TWI_InitTypeDef defaultTWICfg = {0};
    *InitStruct = defaultTWICfg;
}


/**
  * @brief  Enables or disables the specified TWI peripheral.
  * @param  TWIx  TWI device: TWI1
  * @param  OnOffState: new state of the TWIx peripheral. 
  *   This parameter can be: ON or OFF.
  * @retval None
  */
void TWI_OnOff(TWI_TypeDef* TWIx, CmdState OnOffState)
{
    assert_param(IS_TWI_MODULE(TWIx));
    assert_param(IS_CMD_STATE(OnOffState));
    
    TWIx->CR.BIT.TWIEN = OnOffState;
}


/**
  * @brief  Selects the TWI mode for the specified TWI.
  * @param  TWIx  TWI device: TWI1
  * @param  TWIMode: new mode of the TWIx peripheral. 
  *   This parameter can be: TWI_MASTER or TWI_SLAVE
  * @retval None
  */
void TWI_ModeConfig(TWI_TypeDef* TWIx, TWI_MODE_Type TWIMode)
{
    assert_param(IS_TWI_MODULE(TWIx));
    assert_param(IS_TWI_MODE_Type(TWIMode));

    TWIx->ADDR.BIT.GC = TWIMode;
}


/**
  * @brief  Enables or disables the specified TWI interrupts.
  * @param  TWIx  TWI device: TWI1
  * @param  NewState: new state of the specified TWI/I2S interrupt.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void TWI_INTConfig(TWI_TypeDef* TWIx, FunctionalState NewState)
{
    assert_param(IS_TWI_MODULE(TWIx));
    assert_param(IS_FUNCTION_STATE(NewState));

    TWIx->CR.BIT.TWINTE = NewState;
}


/**
  * @brief  Checks whether the specified TWI interrupt has occurred or not.
  * @param  TWIx device: TWI1
   * @retval FlagStatus The new state interrupt (SET or RESET).
  */
FlagStatus TWI_GetINTStatus(TWI_TypeDef* TWIx)
{
    uint32_t tmpV;
    assert_param(IS_TWI_MODULE(TWIx));
    tmpV  = TWIx->FR.BIT.TWINT;
    tmpV &= TWIx->CR.BIT.TWINTE;
    return ((tmpV) ? SET : RESET);
}

/**
  * @brief  Clears the TWIx interrupt pending bit.
  * @param  TWIx  TWI device: TWI1
  * @retval None
  */
void TWI_ClearINTStatus(TWI_TypeDef* TWIx)
{
    assert_param(IS_TWI_MODULE(TWIx));
    TWIx->FR.BIT.TWINTC = 1;
}

/**
  * @brief  get TWI transmit flag
  * @param  TWIx  TWI device: TWI1
  * @param  Flag  TWI flag mask
  *   This parameter can be one of following values:
  *     @arg @b  TWI_FLAG_INT Interrupt flag
  *     @arg @b  TWI_FLAG_SCL_TIMEOUT SCL high level timeout flag
  *     @arg @b  TWI_FLAG_BUS_TIMEOUT  Bus timeout flag
  * @retval  FlagStatus RCC reset flag
  *     @arg @b  SET    the flag is setted
  *     @arg @b  RESET  the flag is cleared
  */
FlagStatus TWI_GetFlagStatus(TWI_TypeDef* TWIx,TWI_FLAG_Type Flag)
{
    assert_param(IS_TWI_MODULE(TWIx));
    assert_param(IS_TWI_FLAG_Type(Flag));    
    
    return ((TWIx->FR.V32 & Flag) ? SET : RESET);
}

/**
  * @brief  clear TWI transmit flags
  * @param  TWIx  TWI device: TWI1
  * @param  Flags   TWI flags clear bit mask
  *   This parameter can be any combination of following values:
  *     @arg @b  TWI_FLAG_INT Interrupt flag
  *     @arg @b  TWI_FLAG_SCL_TIMEOUT SCL high level timeout flag
  *     @arg @b  TWI_FLAG_BUS_TIMEOUT  Bus timeout flag
  * @retval None
  */
void TWI_ClearFlag(TWI_TypeDef* TWIx,uint32_t Flags)
{
    assert_param(IS_TWI_MODULE(TWIx));
    assert_param(IS_TWI_FLAG_Types(Flags));    
 
    TWIx->FR.V32 = (Flags<<16);
}

/**
  * @brief  TWI send data
  * @param  TWIx  TWI device: TWI1
  * @param  Data send data
  * @retval None
  */
void TWI_SendData(TWI_TypeDef* TWIx, uint8_t Data)
{
    assert_param(IS_TWI_MODULE(TWIx));

    TWIx->DR = Data;
}


/**
  * @brief  TWI receive data
  * @param  TWIx  TWI device: TWI1
  * @retval uint16_t received data
  */
uint8_t TWI_ReceiveData(TWI_TypeDef* TWIx)
{
    assert_param(IS_TWI_MODULE(TWIx));
        
    return TWIx->DR;
}

/* get TWI transmit event */
/**
  * @brief  get TWI transmit event
  * @param  TWIx  TWI device: TWI1
  * @retval TWI_EVENT_Type TWI transmit event flag
  */
TWI_EVENT_Type TWI_GetEvent(TWI_TypeDef* TWIx)
{
    assert_param(IS_TWI_MODULE(TWIx));
        
    return (TWI_EVENT_Type)(TWIx->STAT.V32);    
}

/**
  * @brief  Generates TWI communication START condition.
  * @param  TWIx  TWI device: TWI1
  * @param  NewState: new state of the I2C START condition generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void TWI_GenerateSTART(TWI_TypeDef* TWIx, FunctionalState NewState)
{
    assert_param(IS_TWI_MODULE(TWIx));
    assert_param(IS_FUNCTION_STATE(NewState));
    
    TWIx->CR.BIT.STA = NewState;    
}

/**
  * @brief  Generates TWI communication STOP condition.
  * @param  TWIx  TWI device: TWI1
  * @param  NewState: new state of the I2C STOP condition generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void TWI_GenerateSTOP(TWI_TypeDef* TWIx, FunctionalState NewState)
{
    assert_param(IS_TWI_MODULE(TWIx));
    assert_param(IS_FUNCTION_STATE(NewState));

    TWIx->CR.BIT.STO = NewState;  
}

/**
  * @brief  Enables or disables the specified TWI acknowledge feature.
  * @param  TWIx  TWI device: TWI1
  * @param  NewAck: new state of the TWI Acknowledgement.
  *   This parameter can be: TWI_AA_NACK or TWI_AA_ACK.
  * @retval None.
  */
void TWI_AcknowledgeConfig(TWI_TypeDef* TWIx, TWI_AA_Type NewAck)
{
  /* Check the parameters */  
  assert_param(IS_TWI_MODULE(TWIx));
  assert_param(IS_TWI_AA_Type(NewAck));
    
  TWIx->CR.BIT.AA = NewAck;
}
		

/**
  * @brief  Send 7bit address and read/write flag
  * @param  TWIx  TWI device: TWI1
  * @param  Address  device address
  * @param  TWI_Direction  transmit direction 
  *   This parameter can be: TWI_DIR_TRANSMITTER or TWI_DIR_RECEIVER.
  * @retval None.
  */
void TWI_Send7bitAddress(TWI_TypeDef* TWIx, uint8_t Address, TWI_DIR_Type TWI_Direction)
{
  assert_param(IS_TWI_MODULE(TWIx));
  assert_param(IS_TWI_DIR_Type(TWI_Direction));
    
  if(TWI_Direction == TWI_DIR_TRANSMITTER)
    TWIx->DR = Address & (~1);
  else
    TWIx->DR = Address | 1;    
}

/**
  * @brief  Configures the specified TWI own address.
  * @param  TWIx  TWI device: TWI1
  * @param  Address:    specifies the own address.
  * @param  AddressMsk: specifies the own address mask bits.
  * @param  NewState:   Enable/Disable 
  * @retval None.
  */
void TWI_OwnAddressConfig(TWI_TypeDef* TWIx, uint8_t Address, uint8_t AddressMsk, FunctionalState NewState)
{
   assert_param(IS_TWI_MODULE(TWIx));
   assert_param(Address < 0x80);
   assert_param(AddressMsk < 0x80);
   assert_param(IS_FUNCTION_STATE(NewState));
    
   TWIx->ADDR.V32 = (Address << TWI_ADDR_ADDR_Pos) 
                  | (AddressMsk << TWI_ADDR_TWIAMR_Pos) 
                  | (NewState << TWI_ADDR_GC_Pos) ;

}


/**
  * @brief  Send TWI commands
  * @param  TWIx  TWI device: TWI1
  * @retval TWI_EVENT_Type TWI event flags.
  */
TWI_EVENT_Type TWI_SendWaitDone(TWI_TypeDef* TWIx)
{
   uint8_t lastEvent;
   assert_param(IS_TWI_MODULE(TWIx));
    
   lastEvent = (TWI_EVENT_Type)(TWIx->STAT.V32);
   /* clear Interrupt Flag to start transmit */
   TWIx->FR.V32 = (TWI_FR_TOUTC_Msk | TWI_FR_TFREEC_Msk | TWI_FR_TWINTC_Msk);    
   
   while(TWI_GetFlagStatus(TWI1,TWI_FLAG_INT) != SET)
   {
       /* STOP Event No INT Flag Setted */
       if(TWIx->CR.BIT.STO && TWIx->CR.BIT.STA == 0)
       {
           if((TWI_EVENT_Type)(TWIx->STAT.V32) != lastEvent)
                break;
       }
   }
   
   /* Clear STA / STO */    
   TWIx->CR.BIT.STA = RESET;
   TWIx->CR.BIT.STO = RESET;
   return (TWI_EVENT_Type)(TWIx->STAT.V32);   
}

/**
  * @brief  Send TWI commands
  * @param  TWIx  TWI device: TWI1
  * @param  CmdType:    TWI Commands as below
  *    @arg @b TWI_CMD_START  Send START Condition 
  *    @arg @b TWI_CMD_STOP   Send STOP Condition
  *    @arg @b TWI_CMD_ACK    Response ACK 
  *    @arg @b TWI_CMD_NACK   Response NACK
  *    @arg @b TWI_CMD_STARTSTOP  Send STOP condition then send START condition
  * @retval TWI_EVENT_Type TWI event flags.
  */
TWI_EVENT_Type TWI_SendCommand(TWI_TypeDef* TWIx, TWI_COMMAND_Type CmdType)
{
   assert_param(IS_TWI_MODULE(TWIx));
   assert_param(IS_TWI_COMMAND_Type(CmdType));
    
   switch(CmdType)
   {
       case TWI_CMD_START:
            TWIx->CR.BIT.STA = SET;
            break;
       case TWI_CMD_STOP:
         //   TWIx->FR.V32 = (TWI_FR_TOUTC_Msk | TWI_FR_TFREEC_Msk | TWI_FR_TWINTC_Msk);    
            TWIx->CR.BIT.STO = SET;
            break;
       case TWI_CMD_ACK:
            TWIx->CR.BIT.AA = TWI_AA_ACK;
            break;
       case TWI_CMD_NACK:
            TWIx->CR.BIT.AA = TWI_AA_NACK;
            break;
       case TWI_CMD_STOPSTART:
            TWIx->CR.BIT.STA = SET;
            TWIx->CR.BIT.STO = SET;       
            break;
       default:
           break;
   }
   
    return TWI_SendWaitDone(TWIx);
}

/** @code Example
        #if defined ( USE_C99_COMPILER_MODE )
        // Please make sure compiler option --c99 is valied
        // Otherwise this method cannot supported
        const TWI_InitTypeDef TWIEECfg = { 
                                        .ZeroAddress   = DISABLE,          // Enable ZERO Address in Slave Mode: ENABLE or DISABLE 
                                        .Address7      = 0,                // 7 bits address                                
                                        .AddressMask   = 0,                // TWI address mask bit 0:Must match 1:Ignore                                
                                        .ACKorNACK     = TWI_AA_ACK,       // Response ACK or NACK 
                                        .SendStop      = DISABLE,          // Send stop signal: ENABLE or DISABLE 
                                        .SendStart     = DISABLE,          // Send start signal: ENABLE or DISABLE                                 
                                        .Clock         = TWI_CR_DIV_16,    // Clock prescale: @ref TWI_CLKRATE_Type 
                                        .BusTimeOut    = TWI_CNT_50000,    // Bus timeout count 
                                        .HighLevelTimeOutEnable = DISABLE, // High level timeout enable bit 
                                        .BusTimeOutEnable = DISABLE,       // Bus timeout enable bit 
                                        .IE               = DISABLE,       // Interrupt enable bit                                
                                        .Enable           = DISABLE,       // TWI Enalbe bit                                
                                        .BaudRate         = 32,            // Baudrate 
                                        .HighLevelTimeOut = 10,            // High level timeout count    
                                    };
        const TWI_EETypeDef EECfg = { .EEHWAddr = EEPROM_HW_ADDR,  // EEPROM device Address, like 0xA0 
                                      .AddrByte = 1,               // EEPROM address length, AT24C02:1 Byte, M24C64: 2 Bytes
                                      .WritePageByte = 8           // EEPROM write page size Default is 8 bytes
                                     };
                                     
        #else
        const TWI_InitTypeDef TWIEECfg = {
                                        DISABLE,        // Enable ZERO Address in Slave Mode: ENABLE or DISABLE 
                                        0,              // 7 bits address 
                                        0,              // reserved 
                                        0x0,            // TWI address mask bit 0:Must match 1:Ignore                                 
                                        0,              // reserved                                                                           
                                        TWI_AA_ACK,     // Response ACK or NACK 
                                        DISABLE,        // Send stop signal: ENABLE or DISABLE 
                                        DISABLE,        // Send start signal: ENABLE or DISABLE 
                                        0,              // reserved 
                                        TWI_CR_DIV_16,  // Clock prescale: @ref TWI_CLKRATE_Type 
                                        TWI_CNT_50000,  // Bus timeout count 
                                        DISABLE,        // High level timeout enable bit   
                                        DISABLE,        // Bus timeout enable bit 
                                        DISABLE,         // Interrupt enable bit    
                                        0,              // Reseved 
                                        DISABLE,        // TWI Enalbe bit 
                                        32,             // Baudrate 
                                        10              // High level timeout count 
                                     };
        const TWI_EETypeDef EECfg = { EEPROM_HW_ADDR,  // EEPROM device Address, like 0xA0 
                                      1,               // EEPROM address length, AT24C02:1 Byte, M24C64: 2 Bytes
                                      8                // EEPROM write page size Default is 8 bytes
                                     }; 
       #endif
     uint8_t read_data;
     uint16_t read_addr = 0x0;   
     TWI_Init(TWI1,&TWIEECfg);    
     TWI_OnOff(TWI1,ON);
     read_data = TWI_EEReadByte(TWI1, &EECfg, read_addr); 
     printf("read data:%X\n",(unsigned int)read_data);
     
  * @endcode
  * @brief  read EEPROM one byte
  * @param  TWIx:  TWI device: TWI1
  * @param  EECfg: EEPROM TWI parameters
  * @param  ReadAddr: data address for read
  * @retval uint8_t read data, if read fail the return value always 0
  */
uint8_t TWI_EEReadByte(TWI_TypeDef* TWIx, const TWI_EETypeDef* EECfg, uint16_t ReadAddr)
{
    __IO uint8_t event;
    uint8_t readData = 0;
    
    assert_param(IS_TWI_MODULE(TWIx));

    /* 1. Send START condition */
   event = TWI_SendCommand(TWIx, TWI_CMD_START);   
   if(event != TWI_MASTER_START_SEND && event != TWI_MASTER_RESTART_SEND)
       return readData;
   
   /* 2. Send Hardware address */
   TWI_Send7bitAddress(TWIx,EECfg->EEHWAddr, TWI_DIR_TRANSMITTER);
   event = TWI_SendWaitDone(TWIx);   
   if(event != TWI_MASTER_SLA_W_ACK)
       return readData;
         
   /* 3. send read address */   
   if(EECfg->AddrBytes == 2)
   {
       TWI_SendData(TWIx,(uint8_t)(ReadAddr>>8));    
       event = TWI_SendWaitDone(TWIx);   
       if(event != TWI_MASTER_DATA_SEND_ACK)
           return readData;
   }   
   TWI_SendData(TWIx,(uint8_t)(ReadAddr&0XFF));    
   event = TWI_SendWaitDone(TWIx);   
   if(event != TWI_MASTER_DATA_SEND_ACK)
       return readData;

   /* 4. Send RESTART condition */
   event = TWI_SendCommand(TWIx, TWI_CMD_START);   
   if(event != TWI_MASTER_START_SEND && event != TWI_MASTER_RESTART_SEND)
       return readData;
  
   /* 5. Send Hardware address */
   TWI_Send7bitAddress(TWIx,EECfg->EEHWAddr, TWI_DIR_RECEIVER);
   event = TWI_SendWaitDone(TWIx);   
   if(event != TWI_MASTER_SLA_R_ACK)
       return readData;
   
    /* 6. Response ACK */
   event = TWI_SendCommand(TWIx, TWI_CMD_ACK);   
   if(event != TWI_MASTER_DATA_RECEIVE_ACK)
       return readData;
   
   /* 7. Read data */
   readData = TWIx->DR;
   
   /* 8. Response NACK to inform EEPROM stop transmit */
   event = TWI_SendCommand(TWIx, TWI_CMD_NACK);   
   assert_param(event == TWI_MASTER_DATA_RECEIVE_NACK);
   
   /* 9. Send STOP condition */
   TWI_SendCommand(TWIx, TWI_CMD_STOP);     
   
   return readData;
}



/** @brief   write one byte data to EEPROM 
  * @param  TWIx:  TWI device: TWI1
  * @param  EECfg: EEPROM TWI parameters
  * @param  WriteAddr: data address for write
  * @param  Data: data for write
  * @retval bool_t TRUE for write success and FALSE for write fail
  */
bool_t  TWI_EEWriteByte(TWI_TypeDef* TWIx,  const TWI_EETypeDef* EECfg, uint16_t WriteAddr,uint8_t Data)
{
   __IO uint8_t event;

    assert_param(IS_TWI_MODULE(TWIx));

    /* 1. Send START condition */
   event = TWI_SendCommand(TWIx, TWI_CMD_START);   
   if(event != TWI_MASTER_START_SEND && event != TWI_MASTER_RESTART_SEND)
       return FALSE;
   
   /* 2. Send Hardware address */
   TWI_Send7bitAddress(TWIx,EECfg->EEHWAddr, TWI_DIR_TRANSMITTER);
   event = TWI_SendWaitDone(TWIx);   
   if(event != TWI_MASTER_SLA_W_ACK)
       return FALSE;
         
   /* 3. send read address */   
   if(EECfg->AddrBytes == 2)
   {
       TWI_SendData(TWIx,(uint8_t)(WriteAddr>>8));    
       event = TWI_SendWaitDone(TWIx);   
       if(event != TWI_MASTER_DATA_SEND_ACK)
           return FALSE;
   }
   
   TWI_SendData(TWIx,(uint8_t)(WriteAddr&0XFF));    
   event = TWI_SendWaitDone(TWIx);   
   if(event != TWI_MASTER_DATA_SEND_ACK)
       return FALSE;

   /* 4. send data to write */
   TWI_SendData(TWIx,Data);    
   event = TWI_SendWaitDone(TWIx);   
   if(event != TWI_MASTER_DATA_SEND_ACK)
        return FALSE;
   
   /* 5. Send STOP condition */
   TWI_SendCommand(TWIx, TWI_CMD_STOP);    
      
   /* 6. Restart condition to wait EEPROM write done. No response if EEPROM is busy */
   do{
        event = TWI_SendCommand(TWIx, TWI_CMD_START);   
        if(event != TWI_MASTER_START_SEND && event != TWI_MASTER_RESTART_SEND)
            continue;
        
        TWI_Send7bitAddress(TWIx,EECfg->EEHWAddr, TWI_DIR_TRANSMITTER);
        event = TWI_SendWaitDone(TWIx);              
   }while(event != TWI_MASTER_SLA_W_ACK);
   
   /* 7. Send STOP condition */
   TWI_SendCommand(TWIx, TWI_CMD_STOP);     
      
   return TRUE;
}


/** @brief  Read EEPROM data to buffer
  * @param  TWIx:  TWI device: TWI1
  * @param  EECfg: EEPROM TWI parameters
  * @param  ReadAddr: data address for write
  * @param  ReadBuffer: buffer for read data
  * @param  ReadBytes: bytes count need read
  * @retval bool_t TRUE for read success and FALSE for read fail
  */
bool_t TWI_EEReadBuffer(TWI_TypeDef* TWIx, const TWI_EETypeDef* EECfg, uint16_t ReadAddr, uint8_t* ReadBuffer, uint16_t ReadBytes)
{
    __IO uint8_t event;
    assert_param(IS_TWI_MODULE(TWIx));
    
   /* 1. Send START condition */
   event = TWI_SendCommand(TWIx, TWI_CMD_START);   
   if(event != TWI_MASTER_START_SEND && event != TWI_MASTER_RESTART_SEND)
       return FALSE;
   
   /* 2. Send Hardware address */
   TWI_Send7bitAddress(TWIx,EECfg->EEHWAddr, TWI_DIR_TRANSMITTER);
   event = TWI_SendWaitDone(TWIx);   
   if(event != TWI_MASTER_SLA_W_ACK)
       return FALSE;
         
   /* 3. send read address */   
   if(EECfg->AddrBytes == 2)
   {
       TWI_SendData(TWIx,(uint8_t)(ReadAddr>>8));    
       event = TWI_SendWaitDone(TWIx);   
       if(event != TWI_MASTER_DATA_SEND_ACK)
           return FALSE;
   }   
   TWI_SendData(TWIx,(uint8_t)(ReadAddr&0XFF));    
   event = TWI_SendWaitDone(TWIx);   
   if(event != TWI_MASTER_DATA_SEND_ACK)
       return FALSE;

   /* 4. Send RESTART condition */
   event = TWI_SendCommand(TWIx, TWI_CMD_START);   
   if(event != TWI_MASTER_START_SEND && event != TWI_MASTER_RESTART_SEND)
       return FALSE;
  
   /* 5. Send Hardware address */
   TWI_Send7bitAddress(TWIx,EECfg->EEHWAddr, TWI_DIR_RECEIVER);
   event = TWI_SendWaitDone(TWIx);   
   if(event != TWI_MASTER_SLA_R_ACK)
       return FALSE;
   
   /* 6. Response ACK */
   event = TWI_SendCommand(TWIx, TWI_CMD_ACK);   
   
   /* 7. Read data and response ACK or NACK */
   while(event == TWI_MASTER_DATA_RECEIVE_ACK && ReadBytes > 0)
   {
       *ReadBuffer = TWIx->DR;
       ReadBuffer++;
       ReadBytes--;
      if(ReadBytes > 0)          
        event = TWI_SendCommand(TWIx, TWI_CMD_ACK); 
      else
        event = TWI_SendCommand(TWIx, TWI_CMD_NACK);
   }
   
   /* 8. Send STOP condition */
   TWI_SendCommand(TWIx, TWI_CMD_STOP);     
       
   return (ReadBytes == 0 ? TRUE : FALSE);
}

/** @brief  Write EEPROM data from buffer
  * @param  TWIx:  TWI device: TWI1
  * @param  EECfg: EEPROM TWI parameters
  * @param  WriteAddr: data address for write
  * @param  WriteBuffer: buffer for write data
  * @param  WriteBytes: bytes count need write
  * @retval bool_t TRUE for write success and FALSE for write fail
  */
bool_t  TWI_EEWriteBuffer(TWI_TypeDef* TWIx, const TWI_EETypeDef* EECfg, uint16_t WriteAddr, uint8_t* WriteBuffer, uint16_t WriteBytes)
{
   __IO uint8_t event;
   uint16_t addr = WriteAddr;
   int8_t oneWriteBytes;
   int i;
    assert_param(IS_TWI_MODULE(TWIx));
   
    while( WriteBytes > 0)
    {
	    oneWriteBytes =EECfg->WritePageBytes - (addr % EECfg->WritePageBytes);
	    if(oneWriteBytes > WriteBytes)
	        oneWriteBytes = WriteBytes;
        
       /* 1. Send START condition */
       event = TWI_SendCommand(TWIx, TWI_CMD_START);   
       if(event != TWI_MASTER_START_SEND && event != TWI_MASTER_RESTART_SEND)
           break;
       
           
       /* 2. Send Hardware address */
       TWI_Send7bitAddress(TWIx,EECfg->EEHWAddr, TWI_DIR_TRANSMITTER);
       event = TWI_SendWaitDone(TWIx);   
       if(event != TWI_MASTER_SLA_W_ACK)
           break;
             
       /* 3. send read address */   
       if(EECfg->AddrBytes == 2)
       {
           TWI_SendData(TWIx,(uint8_t)(addr>>8));    
           event = TWI_SendWaitDone(TWIx);   
           if(event != TWI_MASTER_DATA_SEND_ACK)
               break;
       }
       
       TWI_SendData(TWIx,(uint8_t)(addr&0XFF));    
       event = TWI_SendWaitDone(TWIx);   
       if(event != TWI_MASTER_DATA_SEND_ACK)
           break;

       for(i = 0; i < oneWriteBytes; i++)
       {
           /* 4. send data to write */
           TWI_SendData(TWIx,*WriteBuffer);
           WriteBuffer++;
           event = TWI_SendWaitDone(TWIx);   
           if(event != TWI_MASTER_DATA_SEND_ACK)
                return FALSE;
       }

       /* 5. Send STOP condition */
       TWI_SendCommand(TWIx, TWI_CMD_STOP);    
          
       /* 6. Restart condition to wait EEPROM write done. No response if EEPROM is busy */
       do{
            event = TWI_SendCommand(TWIx, TWI_CMD_START);   
            if(event != TWI_MASTER_START_SEND && event != TWI_MASTER_RESTART_SEND)
                continue;
            
            TWI_Send7bitAddress(TWIx,EECfg->EEHWAddr, TWI_DIR_TRANSMITTER);
            event = TWI_SendWaitDone(TWIx);              
       }while(event != TWI_MASTER_SLA_W_ACK);
       
       /* 7. Send STOP condition */
       TWI_SendCommand(TWIx, TWI_CMD_STOP);

       addr += oneWriteBytes;
       WriteBytes -= oneWriteBytes;
       oneWriteBytes = EECfg->WritePageBytes;
       if(WriteBytes < oneWriteBytes)
           oneWriteBytes = WriteBytes;
   }
      
   return (WriteBytes == 0 ? TRUE : FALSE);
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


