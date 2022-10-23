/**
  ******************************************************************************
  * @file    sh32f2xx_rambist.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provide APIs for using RAM Bist module
  *         
  *  @verbatim
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
#include "sh32f2xx_rambist.h"


/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/* CRC  Module----------------------------------------------------------*/
/** @defgroup RAMBIST_MODULE  Ram Bist
   *  CRC Calculate Mode
  * @{
  */ 
        
/** @defgroup  RAMBIST_Group_Pub_Funcs  Public Functions
 *  @brief   Ram Bist  Public Functions
 *
  * @{
  */

/**
  * @brief  Initialize RAMBIST registers   
  * @param  InitCfg Inital data to RAMBIST module
  *      @arg @b AlgmSel: RAMBIST_MARCH_C or RAMBIST_MARCH_X
  *      @arg @b BlkSize: RAMBIST_TESTBLK_16 ,RAMBIST_TESTBLK_32 ~ RAMBIST_TESTBLK_2048    
  * @retval None
  */
void RAMBIST_Init(const RAMBIST_InitTypeDef* InitCfg)
{
    assert_param(IS_RAMBIST_ALGORITHM(InitCfg->AlgmSel));
    assert_param(IS_RAMBIST_TESTBLK(InitCfg->BlkSize));
    
    RAMBIST->CFG.BIT.SEL   = InitCfg->AlgmSel;
    RAMBIST->CFG.BIT.BLKSZ = InitCfg->BlkSize;
   
}

/**
  * @brief  Fills each InitV member with its default value.
  * @param  InitStruct : pointer to a RAMBIST_InitTypeDef structure which will be initialized.
  * @retval None
  */
void RAMBIST_StructInit(RAMBIST_InitTypeDef* InitStruct)
{
    assert_param(IS_IN_SRAM(InitStruct) || IS_IN_CRAM(InitStruct));
    
    InitStruct->AlgmSel = RAMBIST_MARCH_C;
    InitStruct->BlkSize = RAMBIST_TESTBLK_16;    
}

/**
  * @brief  Deinitializes the RAMBIST peripheral registers to their default reset values 
  * @retval None
  */
void RAMBIST_Reset(void)
{
    RAMBIST->ADDR.V32 = 0;
    RAMBIST->CFG.V32 = 0;
    RAMBIST->CSR.V32 = 0;
}

/**
  *@code   Example : checking CRAM area
        uint32_t addr;
        uint32_t size;
        uint32_t blksize;
        RAMBIST_InitTypeDef InitCfg;
        
        RAMBIST_StructInit(&InitCfg);    
        RAMBIST_Init(&InitCfg);
        
        addr = CRAM_BASE;
        blksize = RAMBIST_TESTBLK_SIZE(InitCfg.BlkSize);
        size = CRAM_SIZE - blksize;
        
        RAMBIST_Run(RAMBIST_CHKAREA_BACKUP,addr);
        while(RAMBIST_IS_BUSY()){CLR_WDT();}
        if(RAMBIST_IS_ERROR())
            printf("cram backup area error.\n");
        else
        {
             while(addr < (CRAM_BASE + size))
             {
                 RAMBIST_Run(RAMBIST_CHKAREA_DATA,addr);
                 while(RAMBIST_IS_BUSY()){CLR_WDT();}
                 if(RAMBIST_IS_ERROR())
                 {   
                     printf("cram area %X error.\n",(unsigned int)addr);
                     break;
                 }
                 addr += blksize;
             }        
        }    
   *@endcode
   */      
/**
  * @brief  Run RAMBIST to do checking
  * @param  AreaType
  *      @arg @b RAMBIST_CHKAREA_DATA
  *      @arg @b RAMBIST_CHKAREA_BACKUP      
  * @param  Addr: checking area, cram address for cram backup area 
                  and sram address for sram backup area. Backup area is the last testblock    
  * @retval None
  */
void RAMBIST_Run(RAMBIST_CheckArea_Type AreaType, uint32_t Addr)
{
    RAMBIST->ADDR.V32 = Addr;
    RAMBIST->CSR.V32 = (AreaType << RAMBIST_CSR_MOD_Pos)|(0x59A6);        
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


