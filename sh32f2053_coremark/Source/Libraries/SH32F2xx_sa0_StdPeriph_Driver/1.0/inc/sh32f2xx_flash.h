/**
  ******************************************************************************
  * @file    sh32f2xx_flash.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    29-April-2017
  * @brief   This file contains all the functions prototypes for the FLASH 
  *          firmware library.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SH32F2xx_FLASH_H
#define __SH32F2xx_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"
#include <stdint.h>
	 
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */ 

/** @addtogroup FLASH_MODULE
  * @{
  */
	 
/** @defgroup FLASH_Group_Constant  Public Constants
  * @{
  */ 
 
/** @enum FLASH_Status
    @brief FLASH Operation Status  
     */   
/*! Flash Operation Status */
typedef enum
{ 
  FLASH_BUSY 							= 1,			/*!< Flash is busy */
  FLASH_ERROR_STA					= 2,			/*!< Flash State Error */	
  FLASH_ERROR_PGW					= 3,			/*!< Flash Program Window Error */
  FLASH_ERROR_PGP					= 4,			/*!< Flash Write Error */
  FLASH_ERROR_WRP					= 5,			/*!< Flash Write Protect Error */
  FLASH_ERROR_FLS					= 6,			/*!< Flash Hardware Error */	
  FLASH_ERROR_OPERATION		= 7,			/*!< Flash Opertation Error */
  FLASH_ERROR_ERASE				= 8,			/*!< Flash Erase Error */
  FLASH_ERROR_PROGRAM			= 9,			/*!< Flash Program Error */
  FLASH_ERROR_BLANK				= 10,			/*!< Flash Blank Error */
  FLASH_ERROR_VERIFY			= 11,			/*!< Flash Verify Error */
  FLASH_COMPLETE					= 12,			/*!< Flash Operation Complete*/
}FLASH_Status;                 	


/**
  * @}
  */ 


/** @defgroup FLASH_Group_Macro  Public Macros
  * @{
  */ 
  
/** 
	*@defgroup Group1 Flash Latency
  * @{
  */
#define FLASH_Latency_0                ((uint8_t)0x0000)						  /*!< FLASH Zero Latency cycle */
#define FLASH_Latency_1                ((uint8_t)0x0001)						  /*!< FLASH One Latency cycle */
#define FLASH_Latency_2                ((uint8_t)0x0002)						  /*!< FLASH Two Latency cycles */
#define FLASH_Latency_3                ((uint8_t)0x0003)						  /*!< FLASH Three Latency cycles */
#define FLASH_Latency_4                ((uint8_t)0x0004)						  /*!< FLASH Four Latency cycles */
#define FLASH_Latency_5                ((uint8_t)0x0005)						  /*!< FLASH Five Latency cycles */
#define FLASH_Latency_6                ((uint8_t)0x0006)						  /*!< FLASH Six Latency cycles */
#define FLASH_Latency_7                ((uint8_t)0x0007)						  /*!< FLASH Seven Latency cycles */

/*! Check the Latency value */
#define IS_FLASH_LATENCY(LATENCY) (((LATENCY) == FLASH_Latency_0) || \
((LATENCY) == FLASH_Latency_1) || \
((LATENCY) == FLASH_Latency_2) || \
((LATENCY) == FLASH_Latency_3) || \
((LATENCY) == FLASH_Latency_4) || \
((LATENCY) == FLASH_Latency_5) || \
((LATENCY) == FLASH_Latency_6) || \
((LATENCY) == FLASH_Latency_7))

/**
  * @}
  */ 


/** 
	*@defgroup Group2 FLASH_CR register
  * @{
  */

#define FLASH_CR_CMD_ME0			               ((uint32_t)0xAA55)/*!< Flash ME0 Opteration Command */   
#define FLASH_CR_CMD_MSE  					         ((uint32_t)0xE619)/*!< Flash Main Block Sector Erase Opteration Command */ 
#define FLASH_CR_CMD_MPG  					         ((uint32_t)0x6E91)/*!< Flash Main Block Programming Opteration Command */ 
#define FLASH_CR_CMD_E2PG  					         ((uint32_t)0xB44B)/*!< Flash EEpRom Block Programming Opteration Command */ 
#define FLASH_CR_CMD_E2SE    			           ((uint32_t)0x4BB4)/*!< Flash EEpRom Block Sector Erase Opteration Command */ 
#define FLASH_CR_CMD_OPG    				         ((uint32_t)0xF00F)/*!< Flash OTP Block Programming Opteration Command */
#define FLASH_CR_CMD_CBPG    			           ((uint32_t)0xC33C)/*!< Flash Customer Block Programming Opteration Command */
#define FLASH_CR_CMD_CBSE    			           ((uint32_t)0x3CC3)/*!< Flash Customer Block Sector Erase Opteration Command */ 
#define FLASH_CR_CMD_SPG    				         ((uint32_t)0xD22D)/*!< Flash Protect Block Programming Opteration Command */





/*! Programming a Half-word one time */	
#define FLASH_CR_PSIZE_HALFWD                1
/*! Programming a word one time */	
#define FLASH_CR_PSIZE_WD										 0

/**
  * @}
  */ 


/** 
	*@defgroup Group3 FLASH Sectors Parameters
  * @{
  */ 
#define FLASH_Sector_0     													((uint8_t)0x0000) /*!< Sector Number 0 */
#define FLASH_Sector_5     													((uint8_t)0x0005) /*!< Sector Number 5 */
#define FLASH_Sector_127   													((uint8_t)0x007F) /*!< Sector Number 127 */

#define IS_E2_BLOCK_SECTOR(SECTOR) 									((SECTOR) <= FLASH_Sector_5)/*!< Check the sector parameters */
#define IS_FLASH_SECTOR(SECTOR) 										((SECTOR) <= FLASH_Sector_127)/*!< Check the sector parameters */

/**
  * @}
  */
  
  
/** 
	* @defgroup Group4 FLASH Read Protection 
  * @{
  */
#define RD_PRT_NO_PROTECT			((uint16_t)0xA55A)/*!< The value of no read protection  */	
#define RD_PRT_LOW_PROTECT		((uint16_t)0xAAAA)/*!< The value of the low level read protection */	
#define RD_PRT_HIGH_PROTECT   ((uint16_t)0xC33C)/*!< The value of the high level read protection  */	
/*! Check the value of read protection */												
#define IS_RD_PRT(LEVEL) (((LEVEL) == RD_PRT_NO_PROTECT)||\
((LEVEL) == RD_PRT_LOW_PROTECT)||\
((LEVEL) == RD_PRT_HIGH_PROTECT))
/**
  * @}
  */


/** 
	*@defgroup Group5 Flash Write Protection
  * @{
  */ 
#define WRT_PRT_S0_S3       	((uint32_t)0x00000001) /*!< Write protection of Sector0 - Sector3 */
#define WRT_PRT_S4_S7      	  ((uint32_t)0x00000002) /*!< Write protection of Sector4 - Sector7 */
#define WRT_PRT_S8_S11     	  ((uint32_t)0x00000004) /*!< Write protection of Sector8 - Sector11 */
#define WRT_PRT_S12_S15       ((uint32_t)0x00000008) /*!< Write protection of Sector12 - Sector15 */
#define WRT_PRT_S16_S19       ((uint32_t)0x00000010) /*!< Write protection of Sector16 - Sector19 */
#define WRT_PRT_S20_S23       ((uint32_t)0x00000020) /*!< Write protection of Sector20 - Sector23 */
#define WRT_PRT_S24_S27       ((uint32_t)0x00000040) /*!< Write protection of Sector24 - Sector27 */
#define WRT_PRT_S28_S31       ((uint32_t)0x00000080) /*!< Write protection of Sector28 - Sector31 */
#define WRT_PRT_S32_S35       ((uint32_t)0x00000100) /*!< Write protection of Sector32 - Sector35 */
#define WRT_PRT_S36_S39       ((uint32_t)0x00000200) /*!< Write protection of Sector36 - Sector39 */
#define WRT_PRT_S40_S43       ((uint32_t)0x00000400) /*!< Write protection of Sector40 - Sector43 */
#define WRT_PRT_S44_S47       ((uint32_t)0x00000800) /*!< Write protection of Sector44 - Sector47 */
#define WRT_PRT_S48_S51       ((uint32_t)0x00001000) /*!< Write protection of Sector48 - Sector51 */
#define WRT_PRT_S52_S55       ((uint32_t)0x00002000) /*!< Write protection of Sector52 - Sector55 */
#define WRT_PRT_S56_S59       ((uint32_t)0x00004000) /*!< Write protection of Sector56 - Sector59 */
#define WRT_PRT_S60_S63       ((uint32_t)0x00008000) /*!< Write protection of Sector60 - Sector63 */
#define WRT_PRT_S64_S67       ((uint32_t)0x00010000) /*!< Write protection of Sector64 - Sector67 */
#define WRT_PRT_S68_S71       ((uint32_t)0x00020000) /*!< Write protection of Sector68 - Sector71 */
#define WRT_PRT_S72_S75       ((uint32_t)0x00040000) /*!< Write protection of Sector72 - Sector75 */
#define WRT_PRT_S76_S79       ((uint32_t)0x00080000) /*!< Write protection of Sector76 - Sector79 */
#define WRT_PRT_S80_S83       ((uint32_t)0x00100000) /*!< Write protection of Sector80 - Sector83 */
#define WRT_PRT_S84_S87       ((uint32_t)0x00200000) /*!< Write protection of Sector84 - Sector87 */
#define WRT_PRT_S88_S91       ((uint32_t)0x00400000) /*!< Write protection of Sector88 - Sector91 */
#define WRT_PRT_S92_S95       ((uint32_t)0x00800000) /*!< Write protection of Sector92 - Sector95 */
#define WRT_PRT_S96_S99       ((uint32_t)0x01000000) /*!< Write protection of Sector96 - Sector99 */
#define WRT_PRT_S100_S103     ((uint32_t)0x02000000) /*!< Write protection of Sector100 - Sector103 */
#define WRT_PRT_S104_S107     ((uint32_t)0x04000000) /*!< Write protection of Sector104 - Sector107 */
#define WRT_PRT_S108_S111     ((uint32_t)0x08000000) /*!< Write protection of Sector108 - Sector111 */
#define WRT_PRT_S112_S115     ((uint32_t)0x10000000) /*!< Write protection of Sector112 - Sector115 */
#define WRT_PRT_S116_S119     ((uint32_t)0x20000000) /*!< Write protection of Sector116 - Sector119 */
#define WRT_PRT_S120_S123     ((uint32_t)0x40000000) /*!< Write protection of Sector120 - Sector123 */
#define WRT_PRT_S124_S127     ((uint32_t)0x80000000) /*!< Write protection of Sector124 - Sector127 */

#define WRT_PRT_Sector_All    ((uint32_t)0xFFFFFFFF) /*!< Write protection of all Sectors */
/**
  * @}
  */

/** 
	*@defgroup Group6 FLASH Program Window Parameters 
  * @{
  */	
#define PGM_WORD_TIME  		70  		/*!< The time of programming a word data (Uint:100us) */
#define PGM_HFWORD_TIME  	35  		/*!< The time of programming a half-word data (Uint:100us) */
#define SE_ERS_TIME  			10000  	/*!< The time of erasing a sector (Uint:100us)*/
#define ME_ERS_TIME  			20000  	/*!< The time of erasing the whole flash (Uint:100us)*/
/**
  * @}
  */


/** 
	* @defgroup Group7 FLASH Operation Error Flags 
  * @{
  */ 
#define FLASH_FLAG_EOP                 ((uint32_t)0x00000001)  /*!< FLASH End of Operation flag */
#define FLASH_FLAG_OPERR               ((uint32_t)0x00000002)  /*!< FLASH Operation Error flag */
#define FLASH_FLAG_FLSERR            	 ((uint32_t)0x00000008)  /*!< FLASH Hardware Error flag */
#define FLASH_FLAG_WRPERR              ((uint32_t)0x00000010)  /*!< FLASH Write protected error flag */
#define FLASH_FLAG_PGPERR              ((uint32_t)0x00000020)  /*!< FLASH Programming One time function error flag */
#define FLASH_FLAG_PGWERR              ((uint32_t)0x00000040)  /*!< FLASH Programming Window error flag  */
#define FLASH_FLAG_STAERR              ((uint32_t)0x00000080)  /*!< FLASH Programming State error flag  */
#define FLASH_FLAG_BSY                 ((uint32_t)0x00008000)  /*!< FLASH Busy flag */ 

/*! Check the error flag */	
#define IS_FLASH_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFF00FFFF) == 0x00000000) && ((FLAG) != 0x00000000))
/*! Check the error flag */	
#define IS_FLASH_GET_FLAG(FLAG)  (((FLAG) == FLASH_FLAG_EOP) || ((FLAG) == FLASH_FLAG_OPERR) || \
((FLAG) == FLASH_FLAG_WRPERR) || ((FLAG) == FLASH_FLAG_PGPERR) || \
((FLAG) == FLASH_FLAG_FLSERR) || ((FLAG) == FLASH_FLAG_PGWERR) || \
((FLAG) == FLASH_FLAG_STAERR) || ((FLAG) == FLASH_FLAG_BSY))
/**
  * @}
  */
  
  
/** 
	* @defgroup Group8 The operation keys
  * @{
  */
 
#define FLASH_MAIN_KEY           ((uint32_t)0x8ACE0246)/*!< The key value of unlocking the Main block  */	
#define FLASH_E2_KEY             ((uint32_t)0x9BDF1357)/*!< The key value of unlocking the EEpRom block  */	
#define FLASH_INFO_KEY           ((uint32_t)0xABCD5678)/*!< The key value of unlocking the Info. block  */	
#define FLASH_SINGLE_OP_KEY      ((uint32_t)0xC3C3C3C3)/*!< The key value of unlocking single operation  */	
#define FLASH_MULTI_OP_KEY       ((uint32_t)0xB4B4B4B4)/*!< The key value of unlocking multiple operations  */	

#define SINGLE_OPERATION               ((uint32_t)0x00)/*!< The flag of single operation  */	
#define MULTI_OPERATION                ((uint32_t)0x01)/*!< The flag of multiple operations  */	
/**
  * @}
  */

/** 
	* @defgroup Group9 The address of blocks
  * @{
  */
#define E2_BLOCK_ADDRESS						0x0FFF0000/*!< The start address of the EEpRom block */	
#define PROTECT_BLOCK_ADDRESS				0x0FFFE000/*!< The start address of the Protect block */	
#define CUSTOMER_BLOCK_ADDRESS			0x0FFFE800/*!< The start address of the Customer block */	
#define PRODUCT_BLOCK_ADDRESS				0x0FFFF000/*!< The start address of the Product block */	
#define OTP_BLOCK_ADDRESS						0x0FFFF800/*!< The start address of the OTP block */	
#define MAIN_BLOCK_ADDRESS					0x00000000/*!< The start address of the Main block */	

#define READ_PROTECT_ADDRESS				0x0FFFE000/*!< The start address of the read protection bits */	
#define WRITE_PROTECT_ADDRESS				0x0FFFE010/*!< The start address of the write protection bits */	

#define OP_CUST1_ADDRESS						0x0FFFE808/*!< The start address of the customer option */	
#define CUSTOM_SECURITY_ADDRESS			0x0FFFE820/*!< The start address of the customer security */	
#define UNIQUE_ID_ADDRESS						0x0FFFF080/*!< The start address of the unique device ID */	
#define AGENT_ID_ADDRESS						0x0FFFF910/*!< The start address of the agent ID */

/*! Check the address is reasonable or not */	
#define IS_FLASH_MAIN_BLOCK_ADDRESS(ADDRESS) 				((ADDRESS) <= 0x0003FFFF) 
/*! Check the address is reasonable or not */	  
#define IS_E2_BLOCK_ADDRESS(ADDRESS)								(((ADDRESS) >= 0x0FFF0000) && ((ADDRESS) <= 0x0FFF2FFF)) 
/*! Check the address is reasonable or not */	
#define IS_PROTECT_BLOCK_ADDRESS(ADDRESS)						(((ADDRESS) >= 0x0FFFE000) && ((ADDRESS) <= 0x0FFFE7FF)) 
/*! Check the address is reasonable or not */	
#define IS_CUSTOMER_BLOCK_ADDRESS(ADDRESS)					(((ADDRESS) >= 0x0FFFE800) && ((ADDRESS) <= 0x0FFFEFFF)) 
/*! Check the address is reasonable or not */	
#define IS_OTP_BLOCK_ADDRESS(ADDRESS)								(((ADDRESS) >= 0x0FFFF800) && ((ADDRESS) <= 0x0FFFFFFF)) 
/*! Check the Length is reasonable or not */	
#define IS_LENGTH_OK(LENGTH)												(((LENGTH) >= 0x4) && ((LENGTH%4) == 0x0)) 
/**
  * @}
  */

/**
  * @}
  */ 



/** @addtogroup FLASH_Public_Functions Public Functions
  * @{
  */ 
 
/** @defgroup FLASH_Group1 FLASH Interface configuration functions
  * @{
  */  
/* FLASH Interface configuration functions ************************************/
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_InstructionCacheCmd(FunctionalState NewState);
void FLASH_DataCacheCmd(FunctionalState NewState);
void FLASH_PrefetchCmd(FunctionalState NewState);
void FLASH_CacheReset(void);
/**
  * @}
  */ 
  
/** @defgroup FLASH_Group2 FLASH Memory Programming Settings
  * @{
  */  
/* FLASH Memory Programming Settings functions *****************************************/   
uint32_t FLASH_Get_WinCounterVal(uint32_t usDelay);
void FLASH_Set_PGMWindow(uint32_t CounterInitValue, uint32_t UpperCounterValue);
void FLASH_Clear_PGMWindow(void);
void FLASH_Main_Unlock(uint8_t OperationType);
void FLASH_Main_Lock(void);void FLASH_E2_Unlock(uint8_t OperationType);
void FLASH_E2_Lock(void);
void FLASH_Info_Unlock(uint8_t OperationType);
void FLASH_Info_Lock(void);
/**
  * @}
  */
  
/** @defgroup FLASH_Group3 Interrupts and flags management functions  
  * @{
  */  
/* Interrupts and flags management functions **********************************/
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(void);
/**
  * @}
  */
  
/** @defgroup FLASH_Group4 FLASH Memory Programming basic functions
  * @{
  */  
  
/* FLASH Memory Programming Basic functions *****************************************/   
FLASH_Status FLASH_Main_EraseAllSectors(void);
FLASH_Status FLASH_Main_EraseSector(uint32_t FLASH_Sector);
FLASH_Status FLASH_Main_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_Main_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_E2_EraseSector(uint32_t E2_Sector);
FLASH_Status FLASH_E2_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_E2_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_CB_EraseSector(void);
FLASH_Status FLASH_CB_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_CB_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_PRTB_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_PRTB_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_OTP_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_OTP_ProgramHalfWord(uint32_t Address, uint16_t Data);


/* FLASH Memory Programming functions *****************************************/   

FLASH_Status FLASH_Main_ProgramPage(uint32_t StartAddr, uint32_t Length, uint8_t *Buf);
FLASH_Status FLASH_E2_ProgramPage(uint32_t StartAddr, uint32_t Length, uint8_t *Buf);
FLASH_Status FLASH_OTP_ProgramPage(uint32_t StartAddr, uint32_t Length, uint8_t *Buf);
FLASH_Status FLASH_VerifyPage(uint32_t StartAddr, uint32_t Length, uint8_t *Buf);
FLASH_Status FLASH_SetWriteProtect(uint32_t WRT_PRT);
FLASH_Status FLASH_SetReadProtect(uint16_t RD_PRT);
uint32_t FLASH_GetWriteProtect(void);
FlagStatus FLASH_GetReadProtect(void);
FLASH_Status FLASH_SetCustomerOption(uint32_t OPT1,uint32_t OPT2);
uint32_t FLASH_GetCustomerOPT0(void);
uint32_t FLASH_GetCustomerOPT1(void);
FLASH_Status FLASH_SetCustomerSecurity(uint32_t CS0,uint32_t CS1);
void FLASH_GetUniqueID(uint8_t *Buf);
void FLASH_GetAgentID(uint8_t *Buf);

/**
  * @}
  */

/**
  * @}
  */ 




#ifdef __cplusplus
}
#endif

#endif /* __SH32F2xx_FLASH_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
