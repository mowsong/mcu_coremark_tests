/**
  ******************************************************************************
  * @file    sh32f2xx_twi.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provide TWI module's APIs
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
#ifndef __SH32F2xx_TWI_H
#define __SH32F2xx_TWI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"
     
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup TWI_MODULE
  * @{
  */ 

/** @defgroup TWI_Group_Constant  Public Constants
  * @{
  */ 
/**@enum TWI_CLKRATE_Type
   @brief TWI Clock Rate Control Type
*/     
typedef enum{
    TWI_CR_DIV_64 = 0, /*!< PCLK1 / 64 */
    TWI_CR_DIV_16 = 1, /*!< PCLK1 / 16 */
    TWI_CR_DIV_4  = 2, /*!< PCLK1 / 4 */
    TWI_CR_DIV_1  = 3, /*!< PCLK1 / 1 */
}TWI_CLKRATE_Type;

/*! Check TWI Clock Rate Control Type */
#define IS_TWI_CR_Type(cr)  (((cr) == TWI_CR_DIV_64) \
                          || ((cr) == TWI_CR_DIV_16) \
                          || ((cr) == TWI_CR_DIV_4)  \
                          || ((cr) == TWI_CR_DIV_1))

/**@enum TWI_CNT_Type
   @brief TWI bus timeout count options
*/     
typedef enum{
    TWI_CNT_25000   = 0, /*!< 25000 counter for bus timeout */
    TWI_CNT_50000   = 1, /*!< 50000 counter for bus timeout */
    TWI_CNT_100000  = 2, /*!< 100000 counter for bus timeout */
    TWI_CNT_200000  = 3, /*!< 200000 counter for bus timeout */
}TWI_CNT_Type;


/*! Check TWI Clock Rate Control Type */
#define TWI_CNT_Type(cnt)  (((cnt) == TWI_CNT_25000) \
                          ||((cnt) == TWI_CNT_50000) \
                          ||((cnt) == TWI_CNT_100000)  \
                          ||((cnt) == TWI_CNT_200000))


/**emum TWI_AA_Type
  *@brief TWI Response ACK or NACK control Type;
*/
typedef enum{
    TWI_AA_NACK = 0, /*!< Response NACK */
    TWI_AA_ACK  = 1,  /*!< Response ACK */
}TWI_AA_Type;

/*! Check TWI Response Type */
#define IS_TWI_AA_Type(aa)  (((aa) == TWI_AA_NACK) || ((aa) == TWI_AA_ACK))

/**emum TWI_MODE_Type
  *@brief TWI mode type
*/
typedef enum{
    TWI_MASTER        = 0,  /*!< TWI as master */
    TWI_SLAVE         = 1,  /*!< TWI as slave */
    TWI_MASTER_SLAVE  = 3,  /*!< TWI as master and slave */
}TWI_MODE_Type;

/*! Check TWI Response Type */
#define IS_TWI_MODE_Type(type)  (((type) == TWI_MASTER) || ((type) == TWI_SLAVE) || ((type) == TWI_MASTER_SLAVE))

/**emum TWI_EVENT_Type
  *@brief TWI transmit event type
*/
typedef enum{
    TWI_MASTER_START_SEND                = 0X08,  /*!< 0x08: TWI Master Send/Receive Mode: Start condition has sent */
    TWI_MASTER_RESTART_SEND              = 0x10,  /*!< 0x10: TWI Master Send/Receive Mode: Restart condition has sent*/
    TWI_MASTER_SLA_W_ACK                 = 0x18,  /*!< 0x18: TWI Master Send Mode: SLA+W sent; ACK received*/
    TWI_MASTER_SLA_W_NACK                = 0x20,  /*!< 0x20: TWI Master Send Mode: SLA+W sent; NACK received*/
    TWI_MASTER_DATA_SEND_ACK             = 0x28,  /*!< 0x28: TWI Master Send Mode: TWI_DR sent; ACK received*/
    TWI_MASTER_DATA_SEND_NACK            = 0x30,  /*!< 0x30: TWI Master Send Mode: TWI_DR sent; NACK received*/
    TWI_MASTER_LOST_ARBITRATION          = 0x38,  /*!< 0x38: TWI Master Send/receive Mode: Lost arbitration bit while SLA+W/R or data transmit*/
    TWI_MASTER_SLA_R_ACK                 = 0x40,  /*!< 0x40: TWI Master Receive Mode: SLA+R sent; ACK received*/
    TWI_MASTER_SLA_R_NACK                = 0x48,  /*!< 0x48: TWI Master Receive Mode: SLA+R sent; NACK received*/
    TWI_MASTER_DATA_RECEIVE_ACK          = 0x50,  /*!< 0x50: TWI Master Receive Mode: SLA+R received; ACK sent*/
    TWI_MASTER_DATA_RECEIVE_NACK         = 0x58,  /*!< 0x58: TWI Master Receive Mode: SLA+R received; NACK sent*/
    TWI_SLAVE_SLA_R_ACK                  = 0xA8,  /*!< 0xA8: TWI Slave Send Mode: SLA+R self received; ACK sent*/
    TWI_SLAVE_LOST_ARBITRATION           = 0xB0,  /*!< 0xB0: TWI Slave Send Mode: Lost arbitration bit;SLA+R received; ACK sent*/
    TWI_SLAVE_DATA_SEND_ACK              = 0xB8,  /*!< 0xB8: TWI Slave Send Mode: TWI_DR sent; ACK received*/
    TWI_SLAVE_DATA_SEND_NACK             = 0xC0,  /*!< 0xC0: TWI Slave Send Mode: TWI_DR sent; NACK received*/
    TWI_SLAVE_LAST_SEND_ACK              = 0xC8,  /*!< 0xC8: TWI Slave Send Mode: Last TWI_DR SENT; ACK received*/
    TWI_SLAVE_SLA_W_ACK                  = 0x60,  /*!< 0x60: TWI Slave Receive Mode:SLA+W self received; ACK sent */
    TWI_SLAVE_LOST_SLA_W_ACK             = 0x68,  /*!< 0x68: TWI Slave Receive Mode:Lost arbitration while SLA+R/W sending; SLA+W received; ACK sent */
    TWI_SLAVE_ADDR_RECEIVE_ACK           = 0x70,  /*!< 0x70: TWI Slave Receive Mode:General address received; ACK sent */
    TWI_SLAVE_LOST_ADDR_ACK              = 0x78,  /*!< 0x78: TWI Slave Receive Mode:Lost arbitration while SLA+R/W sending; addr received; ACK sent  */
    TWI_SLAVE_ADDR_DATA_RECEIVE_ACK      = 0x80,  /*!< 0x80: TWI Slave Receive Mode: In address valid mode; data received; ACK sent*/ 
    TWI_SLAVE_ADDR_DATA_RECEIVE_NACK     = 0x88,  /*!< 0x88: TWI Slave Receive Mode: In address valid mode; data received; NACK sent*/ 
    TWI_SLAVE_GADDR_DATA_RECEIVE_ACK     = 0x90,  /*!< 0x90: TWI Slave Receive Mode: In general address valid mode,data received; ACK sent*/ 
    TWI_SLAVE_GADDR_DATA_RECEIVE_NACK    = 0x98,  /*!< 0x98: TWI Slave Receive Mode: In general address valid mode,data received; NACK sent*/ 
    TWI_SLAVE_STOP_RESTART_RECEIVE       = 0xA0,  /*!< 0xA0: TWI Slave Receive Mode: Stop condition or Restart condition received*/
    TWI_NO_VALID_EVENT                   = 0xF8,  /*!< 0xF8: TWI other Mode: no valid event code found. TWINT=0*/
    TWI_INVALID_EVENT                    = 0x0,   /*!< 0x00: TWI other Mode: Invalid start or stop sent; internal logic error*/
}TWI_EVENT_Type;

/**emum TWI_DIR_Type
  *@brief TWI transmit direction type
*/
typedef enum{
    TWI_DIR_TRANSMITTER = 0,  /*!< Transmitter mode */
    TWI_DIR_RECEIVER    = 1,  /*!< Receiver mode */
}TWI_DIR_Type;

/*! check TWI direction type */
#define IS_TWI_DIR_Type(dir) (((dir) == TWI_DIR_TRANSMITTER) || ((dir) == TWI_DIR_RECEIVER))


/**emum TWI_COMMAND_Type
  *@brief TWI transmit direction type
*/
typedef enum{
    TWI_CMD_START = 0,  /*!< Send START Condition */
    TWI_CMD_STOP  = 1,  /*!< Send STOP Condition */
    TWI_CMD_ACK   = 2,  /*!< Response ACK */
    TWI_CMD_NACK  = 3,  /*!< Response NACK */
    TWI_CMD_STOPSTART = 4,  /*!< Send STOP condition then send START condition */    
}TWI_COMMAND_Type;

/*! check TWI command type */
#define IS_TWI_COMMAND_Type(cmd) (((cmd) == TWI_CMD_START)  \
                               || ((cmd) == TWI_CMD_STOP)    \
                               || ((cmd) == TWI_CMD_ACK)     \
                               || ((cmd) == TWI_CMD_NACK)    \
                               || ((cmd) == TWI_CMD_STOPSTART))
/**emum TWI_FLAG_Type
  *@brief TWI Flags
*/
typedef enum{
    TWI_FLAG_INT          = TWI_FR_TWINT_Msk, /*!< TWI Interrupt Flag */
    TWI_FLAG_SCL_TIMEOUT  = TWI_FR_TFREE_Msk, /*!< SCL high level timeout flag */
    TWI_FLAG_BUS_TIMEOUT  = TWI_FR_TOUT_Msk, /*!< Bus timeout flag */
}TWI_FLAG_Type;

/*! check TWI Flag */
#define IS_TWI_FLAG_Type(t) (((t) == TWI_FLAG_INT) || ((t) == TWI_FLAG_SCL_TIMEOUT) || ((t) == TWI_FLAG_BUS_TIMEOUT))
/*! All TWI Flags */
#define TWI_FLAG_ALL        (TWI_FLAG_INT|TWI_FLAG_SCL_TIMEOUT|TWI_FLAG_BUS_TIMEOUT)
/*! check TWI Flag */
#define IS_TWI_FLAG_Types(f) ((((f) & TWI_FLAG_ALL) != 0) && (((f) & (~TWI_FLAG_ALL)) == 0))

/**
  * @}
  */ 

/** @defgroup TWI_Group_Types  Public Types
  * @{
  */     

/*! @struct  TWI_InitTypeDef
  * @brief structure for TWI initial
  */ 
typedef struct
{   /* for address register */
    uint32_t  ZeroAddress   : 1;  /*!< Enable ZERO Address in Slave Mode: ENABLE or DISABLE */
    uint32_t  Address7      : 7;  /*!< 7 bits address*/
    uint32_t  rev0          : 9;  /*!< reserved*/
    uint32_t  AddressMask   : 7;  /*!< TWI address mask bit 0:Must match 1:Ignore*/
    uint32_t  rev1          : 8;  /*!< [b31~b24]*/
    /* for CR register */
    uint16_t  ACKorNACK     : 1;  /*!< Response TWI_AA_ACK or TWI_AA_NACK */
    uint16_t  SendStop      : 1;  /*!< Send stop signal: ENABLE or DISABLE */
    uint16_t  SendStart     : 1;  /*!< Send start signal: ENABLE or DISABLE */
    uint16_t  rev2          : 1;  /*!< reserved*/
    uint16_t  Clock          : 2;  /*!< Clock prescale: @ref TWI_CLKRATE_Type */
    uint16_t  BusTimeOut     : 2;  /*!< Bus timeout count */
    uint16_t  HighLevelTimeOutEnable : 1;  /*!< High level timeout enable bit */
    uint16_t  BusTimeOutEnable : 1;  /*!< Bus timeout enable bit */
    uint16_t  IE               : 1;  /*!< Interrupt enable bit*/
    uint16_t  rev3             : 4;  /*!< Reseved*/
    uint16_t  Enable           : 1;  /*!< TWI Enalbe bit*/    
    uint8_t   BaudRate;              /*!< Baudrate */
    uint8_t   HighLevelTimeOut;      /*!< High level timeout count*/    
}TWI_InitTypeDef;

/** struct TWI_EETypeDef
  *@brief TWI operate EEPROM options
  */
typedef struct {
    uint8_t      EEHWAddr;         /*!< EEPROM TWI Address, like 0xA0 */
    uint8_t      AddrBytes;        /*!< EEPROM data address lenggh. AT24C02:1 Byte, M24C64: 2 Bytes */
    uint8_t      WritePageBytes;   /*!< EEPROM page size for write. Default is 8 bytes*/
}TWI_EETypeDef;

/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/** @defgroup TWI_Group_Macro  Public Macros
  * @{
  */ 

/*! check TWI module pointer */
#define IS_TWI_MODULE(m) (m == TWI1)

/**
  * @}
  */ 

/* Exported functions --------------------------------------------------------*/ 
/** @addtogroup TWI_Group_Pub_Funcs
  * @{
  */     

/* TWI Initial function */
void TWI_Init(TWI_TypeDef* TWIx, const TWI_InitTypeDef* InitCfg);

/*Deinitializes the TWIx peripheral registers to their default*/
void TWI_Reset(TWI_TypeDef* TWIx);

/*Deinitializes the TWIx peripheral registers to their default*/
void TWI_StructInit(TWI_InitTypeDef* InitStruct);

/* Enables or disables the specified TWI peripheral. */
void TWI_OnOff(TWI_TypeDef* TWIx, CmdState OnOffState);

/* Selects the TWI mode for the specified TWI */
void TWI_ModeConfig(TWI_TypeDef* TWIx, TWI_MODE_Type TWIMode);

/* Enables or disables the specified TWI interrupts. */
void TWI_INTConfig(TWI_TypeDef* TWIx, FunctionalState NewState);

/* Checks whether the specified TWI interrupt has occurred or not.*/
FlagStatus TWI_GetINTStatus(TWI_TypeDef* TWIx);

/* Clears the TWIx interrupt pending bit. */
void TWI_ClearINTStatus(TWI_TypeDef* TWIx);

/* get TWI transmit flag */
FlagStatus TWI_GetFlagStatus(TWI_TypeDef* TWIx,TWI_FLAG_Type Flag);

/* clear TWI transmit flags */
void TWI_ClearFlag(TWI_TypeDef* TWIx,uint32_t Flags);

/* TWI send data */
void TWI_SendData(TWI_TypeDef* TWIx, uint8_t Data);

/* TWI receive data */
uint8_t TWI_ReceiveData(TWI_TypeDef* TWIx);

/* get TWI transmit event */
TWI_EVENT_Type TWI_GetEvent(TWI_TypeDef* TWIx);

/* generate TWI start condition or not*/
void TWI_GenerateSTART(TWI_TypeDef* TWIx, FunctionalState NewState);

/* generate TWI stop condition or not*/
void TWI_GenerateSTOP(TWI_TypeDef* TWIx, FunctionalState NewState);

/* Response ACK/NACK */
void TWI_AcknowledgeConfig(TWI_TypeDef* TWIx, TWI_AA_Type NewAck);
    
/* Send 7bit address and read/write flag */
void TWI_Send7bitAddress(TWI_TypeDef* TWIx, uint8_t Address, TWI_DIR_Type TWI_Direction);

/* Configures the specified TWI own address. */
void TWI_OwnAddressConfig(TWI_TypeDef* TWIx, uint8_t Address, uint8_t AddressMsk, FunctionalState NewState);

/* Send TWI commands */
TWI_EVENT_Type TWI_SendCommand(TWI_TypeDef* TWIx, TWI_COMMAND_Type CmdType);

/* Start send data and wait it done */
TWI_EVENT_Type TWI_SendWaitDone(TWI_TypeDef* TWIx);

/* read EEPROM one byte*/
uint8_t TWI_EEReadByte(TWI_TypeDef* TWIx, const TWI_EETypeDef* EECfg, uint16_t ReadAddr);

/* write one byte data to EEPROM */
bool_t TWI_EEWriteByte(TWI_TypeDef* TWIx, const TWI_EETypeDef* EECfg, uint16_t WriteAddr, uint8_t Data);

/* read EEPROM to buffer*/
bool_t TWI_EEReadBuffer(TWI_TypeDef* TWIx, const TWI_EETypeDef* EECfg, uint16_t ReadAddr, uint8_t* ReadBuffer, uint16_t ReadBytes);

/* write EEPROM from buffer */
bool_t TWI_EEWriteBuffer(TWI_TypeDef* TWIx, const TWI_EETypeDef* EECfg, uint16_t WriteAddr, uint8_t* WriteBuffer, uint16_t WriteBytes);


/**
  * @}
  */ 


#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_TWI_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
