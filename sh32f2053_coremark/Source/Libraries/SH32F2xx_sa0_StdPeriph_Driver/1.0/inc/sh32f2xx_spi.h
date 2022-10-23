/**
  ******************************************************************************
  * @file    sh32f2xx_spi.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provide SPI module's APIs
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
#ifndef __SH32F2xx_SPI_H
#define __SH32F2xx_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"
     
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup SPI_MODULE
  * @{
  */ 

/** @defgroup SPI_Group_Constant  Public Constants
  * @{
  */ 

/** @enum SPI_SPR_Type
    @brief SPI clock options
     */    
typedef enum{
    SPR_PCKL1_DIV_2    = 0, /*!< PCK1 / 2 */
    SPR_PCKL1_DIV_4    = 1, /*!< PCK1 / 4 */
    SPR_PCKL1_DIV_8    = 2, /*!< PCK1 / 8 */
    SPR_PCKL1_DIV_16   = 3, /*!< PCK1 / 16 */
    SPR_PCKL1_DIV_32   = 4, /*!< PCK1 / 32 */
    SPR_PCKL1_DIV_64   = 5, /*!< PCK1 / 64 */
    SPR_PCKL1_DIV_128  = 6, /*!< PCK1 / 128 */
    SPR_PCKL1_DIV_256  = 7, /*!< PCK1 / 256 */
    SPR_PCKL1_DIV_512  = 8, /*!< PCK1 / 512 */
    SPR_PCKL1_DIV_1024 = 9, /*!< PCK1 / 1024 */    
}SPI_SPR_Type;

/*! check SPI SPR type */
#define IS_SPI_SPR_Type(spr) ((spr) <= SPR_PCKL1_DIV_1024)


/** @enum SPI_SSDIS_Type
    *@brief SS Pin Enalbe or Disable
    *@details If in master slave mode disable NSS pin, no MODEF interrupt require occurs.
              If in slave mode and CPHA is 0, bypass this option bit.
     */    
typedef enum{
    PIN_SS_ENABLE  = 0, /*!< SS pin used */
    PIN_SS_DISABLE = 1, /*!< SS pin no used */
}SPI_SSDIS_Type;

/*! check SPI SPR type */
#define IS_SPI_SSDIS_Type(ss) (((ss) == PIN_SS_ENABLE) || ((ss) == PIN_SS_DISABLE))


/** @enum SPI_CPOL_Type
    *@brief CLK Pin status while in IDLE
    */    
typedef enum{
    CLK_IDLE_LOW  = 0, /*!< CLK pin is low while in IDLE */
    CLK_IDLE_HIGH = 1, /*!< CLK pin is high while in IDLE */
}SPI_CPOL_Type;

/*! check SPI CLK pin status in IDLE mode */
#define IS_SPI_CPOL_Type(clk) (((clk) == CLK_IDLE_LOW) || ((clk) == CLK_IDLE_HIGH))


/** @enum SPI_CPHA_Type
    *@brief capture signal time
    */    
typedef enum{
    CAP_CLK_EDGE1 = 0, /*!< capture signal at clock's first edge */
    CAP_CLK_EDGE2 = 1, /*!< capture signal at clock's second edge */
}SPI_CPHA_Type;

/*! check SPI Capture Phase type */
#define IS_SPI_CPHA_Type(clk) (((clk) == CAP_CLK_EDGE1) || ((clk) == CAP_CLK_EDGE2))


/** @enum SPI_MSTR_Type
    *@brief SPI mode type
    */    
typedef enum{
    SPI_SLAVE  = 0, /*!< SPI slave mode */
    SPI_MASTER = 1, /*!< SPI master mode */
}SPI_MSTR_Type;

/*! check SPI Master / Slave mode type */
#define IS_SPI_MSTR_Type(mode) (((mode) == SPI_SLAVE) || ((mode) == SPI_MASTER))


/** @enum SPI_DIR_Type
    *@brief SPI transmit direction type
    */    
typedef enum{
    SPI_TRANS_MSB = 0, /*!< Most Significant Bit first transmit*/
    SPI_TRANS_LSB = 1, /*!< Least Significant Bit first transmit */
}SPI_DIR_Type;

/*! check SPI transmit direction type */
#define IS_SPI_DIR_Type(mode) (((mode) == SPI_TRANS_MSB) || ((mode) == SPI_TRANS_LSB))


/** @enum SPI_SPDATL_Type
    *@brief SPI transmit data length
    */    
typedef enum{
    SPI_DATASIZE_8b  = 0, /*!< 8 bits one transmit */
    SPI_DATASIZE_16b = 1, /*!< 16 bits one transmit */
}SPI_SPDATL_Type;

/*! check SPI transmit direction type */
#define IS_SPI_SPDATL_Type(mode) (((mode) == SPI_DATASIZE_8b) || ((mode) == SPI_DATASIZE_16b))

/** @enum SPI_SPSFF_Type
    *@brief SPI transmit mode for slave mode
    */    
typedef enum{
    SPI_SLAVE_NORMAL = 0, /*!< SS must high after one data transmitted */
    SPI_SLAVE_FAST   = 1, /*!< no start bit after first data while continuous transmit */
}SPI_SPSFF_Type;

/*! check SPI slave transmit mode*/
#define IS_SPI_SPSFF_Type(mode) (((mode) == SPI_SLAVE_NORMAL) || ((mode) == SPI_SLAVE_FAST))



/** @enum SPI_FUNC_Type
    *@brief SPI transmit function type
    */    
typedef enum{
    SPI_FUNC_SEND    = 1, /*!< SPI send function */
    SPI_FUNC_RECEIVE = 2, /*!< SPI receive function */
}SPI_FUNC_Type;

/*! check SPI transmit mode*/
#define IS_SPI_FUNC_Type(func) (((func) == SPI_FUNC_SEND) ||((func) == SPI_FUNC_RECEIVE))

/*! check SPI transmit mode can be combinated */
#define IS_SPI_FUNC_COMBType(func) (((func) == SPI_FUNC_SEND) \
                                ||((func) == SPI_FUNC_RECEIVE) \
                                ||((func) == (SPI_FUNC_SEND | SPI_FUNC_RECEIVE)))
                                


/** @enum SPI_FLAG_Type
    *@brief SPI transmit function type
    */  
typedef enum{
    SPI_FLAG_RECEIVE_INT   = SPI_FR_SPRI_Msk, /*!< Interrupt flag for receive buffer ready */
    SPI_FLAG_SEND_INT      = SPI_FR_SPTI_Msk, /*!< Interrupt flag for send buffer empty */
    SPI_FLAG_MODE_ERROR    = SPI_FR_MODF_Msk, /*!< More than one master in the transmit net */
    SPI_FLAG_RECIEVE_OVER  = SPI_FR_RXOV_Msk, /*!< Receive overflag */
    SPI_FLAG_SEND_CONFLICT = SPI_FR_WCOL_Msk /*!< Write conflict flag */  
}SPI_FLAG_Type;

/*! check SPI transmit mode*/
#define IS_SPI_FLAG_Type(f)      (((f) == SPI_FLAG_RECEIVE_INT) \
                                ||((f) == SPI_FLAG_SEND_INT) \
                                ||((f) == SPI_FLAG_MODE_ERROR) \
                                ||((f) == SPI_FLAG_RECIEVE_OVER) \
                                ||((f) == SPI_FLAG_SEND_CONFLICT) \
                                )
/*! All SPI Flags */                                
#define SPI_FLAG_ALL   (  SPI_FLAG_RECEIVE_INT \
                        | SPI_FLAG_SEND_INT \
                        | SPI_FLAG_MODE_ERROR \
                        | SPI_FLAG_RECIEVE_OVER \
                        | SPI_FLAG_SEND_CONFLICT)
/*! check SPI transmit mode can be combinated */
#define IS_SPI_FLAG_Types(f) ((((f) & SPI_FLAG_ALL) != 0) && (((f) & (~SPI_FLAG_ALL)) == 0))

/**
  * @}
  */ 

/** @defgroup SPI_Group_Types  Public Types
  * @{
  */     

/*! @struct  SPI_InitTypeDef
  * @brief structure for SPI initial
  */ 
typedef struct
{
    uint32_t  Clock          : 4;  /*!< SPI clock option @ref SPI_SPR_Type*/
    uint32_t  SSPin          : 1;  /*!< SS Pin in master mode: @ref SPI_SSDIS_Type : PIN_SS_ENABLE or PIN_SS_DISABLE*/
    uint32_t  ClkIdleState   : 1;  /*!< CLK Pin status while in IDLE: CLK_IDLE_LOW or CLK_IDLE_HIGH */
    uint32_t  CaptureEdge    : 1;  /*!< capture signal time: CAP_CLK_EDGE1 or CAP_CLK_EDGE2 */
    uint32_t  MasterOrSlave  : 1;  /*!< SPI mode : SPI_SLAVE or SPI_MASTER*/
    uint32_t  Direction      : 1;  /*!< SPI transmit direction type:SPI_TRANS_MSB or SPI_TRANS_LSB  */
    uint32_t  DataSize       : 1;  /*!< SPI transmit data length: SPI_DATASIZE_8b or SPI_DATASIZE_16b*/
    uint32_t  ReceiveIE      : 1;  /*!< SPI receive interrupt: ENABLE or DISABLE */
    uint32_t  SendIE         : 1;  /*!< SPI send interrupt enable bit */
    uint32_t  ReceiveDMA     : 1;  /*!< SPI receive DMA enable bit */
    uint32_t  SendDMA        : 1;  /*!< SPI send DMA enable bit */
    uint32_t  Enable         : 1;  /*!< SPI enable bit */
    uint32_t  SlaveTransMode : 1;  /*!< SPI transmit mode for slave mode:SPI_SLAVE_NORMAL or SPI_SLAVE_FAST */
}SPI_InitTypeDef;


/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/** @defgroup SPI_Group_Macro  Public Macros
  * @{
  */ 

/*! check SPI module pointer */
#define IS_SPI_MODULE(m) (m == SPI1 || m == SPI2)


/**
  * @}
  */ 

/* Exported functions --------------------------------------------------------*/ 
/** @addtogroup SPI_Group_Pub_Funcs
  * @{
  */     

/* SPI Initial function */
void SPI_Init(SPI_TypeDef* SPIx, const SPI_InitTypeDef* InitCfg);

/*Deinitializes the SPIx peripheral registers to their default*/
void SPI_Reset(SPI_TypeDef* SPIx);

/*Deinitializes the SPIx peripheral registers to their default*/
void SPI_StructInit(SPI_InitTypeDef* InitStruct);

/* Selects the SPI mode for the specified SPI. */
void SPI_ModeConfig(SPI_TypeDef* SPIx, SPI_MSTR_Type SPIMode);

/* Enables or disables the specified SPI peripheral. */
void SPI_OnOff(SPI_TypeDef* SPIx, CmdState OnOffState);

/* Enables or disables the specified SPI interrupts. */
void SPI_INTConfig(SPI_TypeDef* SPIx, uint32_t SPIFunc, FunctionalState NewState);

/* Enables or disables the SPIx DMA interface. */
void SPI_DMAConfig(SPI_TypeDef* SPIx, uint32_t SPIFunc, FunctionalState NewState);

/* Enables or disables the NSS Pin for the selected SPI.*/
void SPI_NSSConfig(SPI_TypeDef* SPIx, SPI_SSDIS_Type SSState);

/* Configures the data size for the selected SPI. */
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, SPI_SPDATL_Type DataSize);

/* Selects the data transfer direction for the specified SPI. */
void SPI_DIRConfig(SPI_TypeDef* SPIx, SPI_DIR_Type Direction);

/* Selects the data transfer baudrate for the specified SPI. */
void SPI_SPRConfig(SPI_TypeDef* SPIx, SPI_SPR_Type Baudrate);

/* Selects the data transfer method in slave mode for the specified SPI. */
void SPI_SlaveModeConfig(SPI_TypeDef* SPIx, SPI_SPSFF_Type SlaveMode);

/* Checks whether the specified SPI interrupt has occurred or not.*/
FlagStatus SPI_GetINTStatus(SPI_TypeDef* SPIx, SPI_FUNC_Type SPIFunc);

/* Clears the SPIx interrupt pending bit. */
void SPI_ClearINTStatus(SPI_TypeDef* SPIx, uint32_t SPIFunc);

/* get SPI transmit flag */
FlagStatus SPI_GetFlagStatus(SPI_TypeDef* SPIx,SPI_FLAG_Type Flag);

/* clear SPI transmit flags */
void SPI_ClearFlag(SPI_TypeDef* SPIx,uint32_t Flags);

/* SPI send data */
void SPI_SendData(SPI_TypeDef* SPIx, uint16_t Data);

/* SPI receive data */
uint16_t SPI_ReceiveData(SPI_TypeDef* SPIx);


/**
  * @}
  */ 


#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_SPI_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
