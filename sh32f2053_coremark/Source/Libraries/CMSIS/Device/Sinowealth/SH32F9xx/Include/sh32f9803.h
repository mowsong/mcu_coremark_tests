/**
  ******************************************************************************
  * @file    sh32f9803.h
  * @version V1.0
  * @date    2018-05-30
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer Header File.
  *          This file contains all the peripheral register's definitions, bits
  *          definitions and memory mapping for Sinowealth M3 based devices.
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

#ifndef __SH32F9803_H__
#define __SH32F9803_H__

#define SH32F9803
#include "sh32f9xx_sa0.h"

#ifndef _USING_MDK_PACK
#ifdef   USING_USER_CFG
#include "sh32f9803_config.h"
#endif /*_NO_USER_CONFIG*/
#else
#include "RTE_Components.h"
#endif


#if     USE_STD_LIBRARY
#include "sh32f9xx_sa0_lib.h"
#endif /*USE_STD_LIBRARY*/

#if defined(_MODULE_DBG_PRINTF)
#include "sh32f9xx_sa0_retarget.h"
#endif /*_MODULE_DBG_PRINTF*/

#define CLR_WDT() IWDT->CLR=0xAAAA

/***************************************/
#define ROM_SIZE    0x20000
#define SRAM1_SIZE  0x4000
#define SRAM2_SIZE  0x2000
#define ROM_BASE    0x0
#define SRAM1_BASE  0x10000000
#define SRAM2_BASE  0x20000000
/***************************************/

#define IS_IN_SRAM1(addr) ((((uint32_t)(addr)) >= SRAM1_BASE) && (((uint32_t)(addr)) <= (SRAM1_BASE+SRAM1_SIZE)))
#define IS_IN_SRAM2(addr) ((((uint32_t)(addr)) >= SRAM2_BASE) && (((uint32_t)(addr)) <= (SRAM2_BASE+SRAM2_SIZE)))
#define IS_IN_ROM(addr)  ((((uint32_t)(addr)) >= ROM_BASE)  && (((uint32_t)(addr)) <= (ROM_BASE+ROM_SIZE)))
#endif /*__SH32F9803_H__*/
