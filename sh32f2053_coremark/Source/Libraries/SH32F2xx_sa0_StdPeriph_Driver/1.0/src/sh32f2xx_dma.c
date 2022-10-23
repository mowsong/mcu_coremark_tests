/**
  ******************************************************************************
  * @file    sh32f2xx_dma.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provide APIs for using DMA module
  *         
  * @verbatim
  *
  *          ===================================================================
  *                                  How to use this driver
  *          ===================================================================
    int main(void)
    {	
        //calculate CRC for Flash ROM 
        // from address = 0
        // size = unit(4) * burstlen(16) * burstcount(2048) = 0x20000 (128K Bytes)
        //
        CRC_Init_TypeDef initCRC;               
        const DMA_InitTypeDef DMA_FLASHCRC_CFG = {DISABLE, // Reload
                                                BURST_IDLE_1CYCLE, //Burst IDLE cycle
                                                {{ENABLE,  // ENALBE/DISABLE DMA ChannelNO
                                                 ENABLE,  // transmit complete interrupt
                                                 DISABLE, // Half transmit complete interrupt
                                                 DISABLE, // Block transmit complete interrupt
                                                 ENABLE, // transmit Error Interrupt
                                                 0,   // reserved always 0
                                                 DMA_ADDR_FIXED,   // Destination Mode
                                                 DMA_ADDR_INC,     // Source Mode
                                                 DMA_UNIT_WORD,    // Destination unit size 
                                                 DMA_UNIT_WORD,    // Destination unit size 
                                                 DMA_PRIOR_LOW,    // Priority 
                                                 BURST_LEN_16UNIT, // burst block length 
                                                 DMA_STRMSEL_UART, // stream select: here no care 
                                                 DMA_TRGMODE_TOEND, // DMA Trigger mode 
                                                 0 }},                 // reserved. always 0 
                                                 0X1FF, // 512 BURST LENGTH 
                                                 (uint32_t)&CRC->DR, // destination address 
                                                 0x0 }; //source address
        
        RetargetInit();
              
        RCC_AHBPeriphClockOnOff(RCC_AHB_DMA | RCC_AHB_CRC,ON);
        initCRC.option.V32 = 0;
        initCRC.option.BIT.reload = ENABLE;
        initCRC.option.BIT.mode = CRC_MODE_32;
        initCRC.InitCfg = 0xFFFFFFFF; 
        
        CRC_Init(&initCRC);
        
        DMA_Init(DMA_CH_NO_0,&DMA_FLASHCRC_CFG);
        
        NVIC_EnableIRQ(DMA_CH0_IRQn);
        
        DMA_SWTrigger(DMA_CH_NO_0);
        
        
        // Infinite loop 
        while(1)
        {
            CLR_WDT();
        }
    }

    void DMA_CH0_Handler()
    {
        DMA_INTStatus_TypeDef ITFlags;
        
        DMA_GetINTStatus(DMA_CH_NO_0,&ITFlags);
        if(ITFlags.flag.TCIF)
            printf("CRC = %X\n",(unsigned int)CRC->DR);
        else
            printf("CRC DMA Error (%X)\n",(unsigned int)ITFlags.V32);
        
        DMA_ClearINTStatus(DMA_CH_NO_0,&ITFlags);
        
    }
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
#include "sh32f2xx_dma.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/* DMA  Module----------------------------------------------------------*/
/** @defgroup DMA_MODULE  DMA 
   *  DMA Calculate Mode
  * @{
  */ 
        
/** @defgroup  DMA_Group_Pub_Funcs  Public Functions
 *  @brief   DMA Public Functions
 *
  * @{
  */
      
/**@code Example
    #if defined (USE_C99_COMPILE_OPTION)
    // Please make sure compiler option --c99 is valied
    // Otherwise this method cannot supported
    DMA_InitTypeDef DMA_FLASHCRC_CFG = {
                                        .Enable            = ON,            // Channel ENABLE or DISABLE
                                        .TransCompleteIE   = ENABLE,            // Transmit complete interrupt ENABLE or DISABLE 
                                        .HalfCompleteIE    = DISABLE,           // Half transmit complete interrupt ENABLE or DISABLE 
                                        .BurstCompleteIE   = DISABLE,           // One burst transmit complete interrupt ENABLE or DISABLE 
                                        .TransErrorIE      = ENABLE,            // Transmit error interrupt ENABLE or DISABLE
                                        .DestAddrType      = DMA_ADDR_FIXED,    // Destination address mode= DMA_ADDR_INC,DMA_ADDR_DEC,DMA_ADDR_FIXED,DMA_ADDR_LOOP 
                                        .SourceAddrType    = DMA_ADDR_INC,      // Source address mode= DMA_ADDR_INC,DMA_ADDR_DEC,DMA_ADDR_FIXED,DMA_ADDR_LOOP 
                                        .DestUnitSize      = DMA_UNIT_WORD,     // Destination unit size= DMA_UNIT_BYTE,DMA_UNIT_HALFWORD,DMA_UNIT_WORD 
                                        .SourceUnitSize    = DMA_UNIT_WORD,     // Source unit size= DMA_UNIT_BYTE,DMA_UNIT_HALFWORD,DMA_UNIT_WORD 
                                        .Priority          = DMA_PRIOR_LOW,     // DMA ChannelNO's priority = DMA_PRIOR_LOW, DMA_PRIOR_MIDDLE, DMA_PRIOR_HIGH,DMA_PRIOR_HIGHEST
                                        .OneBurstLen       = BURST_LEN_16UNIT,  // One burst transmit length : @ref DMA_BURSTLEN_Type 
                                        .StreamSelect      = DMA_STRMSEL_FUNC0, // DMA stream contrl source select 
                                        .TriggerMode       = DMA_TRGMODE_TOEND, // DMA tragle mode = DMA_TRGMODE_ONESHOT,DMA_TRGMODE_TOEND
                                        .Reload            = DISABLE,           // Autoreload ChannelNO's settings 
                                        .BurstIdle         = BURST_IDLE_1CYCLE, // Set burst idle cycles BURST_IDLE_1CYCLE ~ BURST_IDLE_32CYCLE
                                        .BurstCount        = 0x1FF,             // Burst Count to transmit value(0~0x1FF) means (1~0x200) burst count
                                        .DestAddr          = (uint32_t)CRC_BASE,// Destination address 
                                        .SourceAddr        = (uint32_t)0,       // Source Address 
                                        };
    #else
    DMA_InitTypeDef DMA_FLASHCRC_CFG = {
                                        ON,            // Channel ENABLE or DISABLE
                                        ENABLE,            // Transmit complete interrupt ENABLE or DISABLE 
                                        DISABLE,           // Half transmit complete interrupt ENABLE or DISABLE 
                                        DISABLE,           // One burst transmit complete interrupt ENABLE or DISABLE 
                                        ENABLE,            // Transmit error interrupt ENABLE or DISABLE
                                        1,                 // revserved 
                                        DMA_ADDR_FIXED,    // Destination address mode= DMA_ADDR_INC,DMA_ADDR_DEC,DMA_ADDR_FIXED,DMA_ADDR_LOOP 
                                        DMA_ADDR_INC,      // Source address mode= DMA_ADDR_INC,DMA_ADDR_DEC,DMA_ADDR_FIXED,DMA_ADDR_LOOP 
                                        DMA_UNIT_WORD,     // Destination unit size= DMA_UNIT_BYTE,DMA_UNIT_HALFWORD,DMA_UNIT_WORD 
                                        DMA_UNIT_WORD,     // Source unit size= DMA_UNIT_BYTE,DMA_UNIT_HALFWORD,DMA_UNIT_WORD 
                                        DMA_PRIOR_LOW,     // DMA ChannelNO's priority = DMA_PRIOR_LOW, DMA_PRIOR_MIDDLE, DMA_PRIOR_HIGH,DMA_PRIOR_HIGHEST
                                        BURST_LEN_16UNIT,  // One burst transmit length : @ref DMA_BURSTLEN_Type 
                                        DMA_STRMSEL_FUNC0, // DMA stream contrl source select 
                                        DMA_TRGMODE_TOEND, // DMA tragle mode = DMA_TRGMODE_ONESHOT,DMA_TRGMODE_TOEND
                                        8,                 // reserved                                     
                                        DISABLE,           // Autoreload ChannelNO's settings 
                                        BURST_IDLE_1CYCLE, // Set burst idle cycles BURST_IDLE_1CYCLE ~ BURST_IDLE_32CYCLE
                                        0x1FF,             // Burst Count to transmit value(0~0x1FF) means (1~0x200) burst count
                                        (uint32_t)CRC_BASE,// Destination address 
                                        (uint32_t)0,       // Source Address 
                                        };
    #endif 
    RCC_AHBPeriphClockOnOff(RCC_AHB_DMA,ON);    
    DMA_Init(DMA_CH_NO_0,&DMA_FLASHCRC_CFG);

      *@endcode
  * @brief  Initializes the DMA Channelx according to the specified
  * @param  ChannelNO ChannelNO number,can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @param  InitCfg  initialize data
  * @retval None
  */
void DMA_Init(const DMA_CHNO_Type ChannelNO, const DMA_InitTypeDef* InitCfg)
{
    DMA_CH_TypeDef* pDMAChannel;
    assert_param(IS_DMA_CHNO_Type(ChannelNO));
    assert_param(IS_DMA_BURSTIDLE_Type(InitCfg->BurstIdle));
    assert_param(IS_DMA_BURSTLEN_Type(InitCfg->OneBurstLen));
    assert_param(IS_FUNCTION_STATE(InitCfg->Reload));
    assert_param(InitCfg->BurstCount < 0x200);
    assert_param(IS_DMA_ADDR_Type(InitCfg->DestAddrType));
    assert_param(IS_DMA_ADDR_Type(InitCfg->SourceAddrType));
    assert_param(IS_DMA_UNIT_Type(InitCfg->DestUnitSize));
    assert_param(IS_DMA_UNIT_Type(InitCfg->SourceUnitSize));
    
    assert_param(IS_DMA_PRIORITY_Type(InitCfg->Priority));
    assert_param(IS_DMA_TRGMODE_Type(InitCfg->TriggerMode));
    assert_param(IS_DMA_STRMSEL_Type(InitCfg->StreamSelect));
        
    
    /* config DMA ChannelNO */
    pDMAChannel = DMA_CHx(ChannelNO);
    
    /* disable DMA EN first */
    pDMAChannel->CCR.V32 &= ~(DMA_CCR0_EN_Msk);
    
    pDMAChannel->NPKT = InitCfg->BurstCount;
    pDMAChannel->DAR  = InitCfg->DestAddr;
    pDMAChannel->SAR  = InitCfg->SourceAddr;

    /* enable / disable reload function */
    BITBAND((uint32_t)&(DMA->CSR.V32),DMA_CSR_RELOAD_Pos+ChannelNO) = InitCfg->Reload;

    /* set burst idle cycles */
    DMA->CSR.BIT.BURSTIDLE = InitCfg->BurstIdle;    
    
    pDMAChannel->CCR.V32 = *((uint32_t*)InitCfg);
}

/**
  * @brief  Fills each InitStruct member with its default value.
  * @param  InitStruct : pointer to a DMA_InitTypeDef structure which will
  *   be initialized.
  * @retval None
  */
void DMA_StructInit(DMA_InitTypeDef* InitStruct)
{        
    const DMA_InitTypeDef DMA_DEFAULT_CFG = {0};
    *InitStruct = DMA_DEFAULT_CFG;
}


/**
  * @brief  Deinitializes the DMA Channelx registers to their default reset
  *   values.
  * @param  ChannelNO ChannelNO number,can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @retval None
  */
void DMA_Reset(const DMA_CHNO_Type ChannelNO)
{
   DMA_CH_TypeDef* pDMAChannel;
   assert_param(IS_DMA_CHNO_Type(ChannelNO));
    
    pDMAChannel = DMA_CHx(ChannelNO);

    /* disable DMA ChannelNO first */
    pDMAChannel->CCR.V32 = 0;
    
    /* config DMA ChannelNO */

    pDMAChannel->NPKT = 0;
    pDMAChannel->DAR  = 0;
    pDMAChannel->SAR  = 0;
    
    DMA->CSR.V32 &= ~(1 << (ChannelNO + DMA_CSR_RELOAD_Pos));
    
    /* clear interrupt flags */
    DMA->IFCR.V32 = (((1 << DMA_IFCR_CBEIF_Pos) 
                     |(1 << DMA_IFCR_CTCIF_Pos) 
                     |(1 << DMA_IFCR_CHTIF_Pos) 
                     |(1 << DMA_IFCR_CTEIF_Pos))
                     << ChannelNO);    
    /* clear again if first is running */                     
    pDMAChannel->CCR.V32 = 0;                     
}


/** @brief  DMA Channel Enable / Disable
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @param  OnOffState control DMA ChannelNO
  *     @arg @b ON disable the DMA ChannelNO
  *     @arg @b OFF enable the DMA ChannelNO
  * @retval None
  */
void DMA_OnOff(const DMA_CHNO_Type ChannelNO, CmdState OnOffState)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));
    assert_param(IS_CMD_STATE(OnOffState));
    
    DMA_CHx(ChannelNO)->CCR.BIT.EN = OnOffState;
}
    
/** @brief  DMA Channel Software Trigger 
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @retval None
  */    
void DMA_SWTrigger(const DMA_CHNO_Type ChannelNO)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));
    
    /* clear interrupt flags */
    DMA->IFCR.V32 = (((1 << DMA_IFCR_CBEIF_Pos) 
                     |(1 << DMA_IFCR_CTCIF_Pos) 
                     |(1 << DMA_IFCR_CHTIF_Pos) 
                     |(1 << DMA_IFCR_CTEIF_Pos))
                     << ChannelNO);
   
    /* software trigger to start DMA */
    DMA->CSR.BIT.SWTRG |= (1<<ChannelNO);
    
}

/**
  * @brief  Sets the number of data units in the current DMA Channelx transfer.
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
   * @param  DataNumber: The number of data units in the current DMA Channelx
  *         transfer. must be (1~512)
  * @note   This function can only be used when the DMA Channelx is disabled.                 
  * @retval None.
  */
void DMA_SetDataCounter(const DMA_CHNO_Type ChannelNO, uint16_t DataNumber)
{
  uint32_t bkCCR;
  assert_param(IS_DMA_CHNO_Type(ChannelNO));
  assert_param(DataNumber >= 1 && DataNumber <= 512);
  
  /* backup CCR */
  bkCCR = DMA_CHx(ChannelNO)->CCR.V32;
  /* disable DMA ChannelNO */
  DMA_CHx(ChannelNO)->CCR.V32 &= ~DMA_CCR0_EN_Msk;
    
  /* Write to DMA Channelx CNDTR */
  DMA_CHx(ChannelNO)->NPKT = DataNumber-1;  
  
  /* restore CCR */
  DMA_CHx(ChannelNO)->CCR.V32 = bkCCR; 
}

/**
  * @brief  Returns the number of remaining data units in the current DMA Channelx transfer.
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @retval uint16_t The number of remaining data units in the current DMAy Channelx
  *   transfer.
  */
uint16_t DMA_GetCurrDataCounter(const DMA_CHNO_Type ChannelNO)
{
  assert_param(IS_DMA_CHNO_Type(ChannelNO));
  
  return (DMA_CHx(ChannelNO)->CPKT);
}


/**
  * @brief  Enables or disables the specified DMAy Channelx interrupts.
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @param  INTType: specifies the DMA interrupts sources to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg @b DMA_INT_COMPLETE:  Transfer complete interrupt mask
  *     @arg @b DMA_INT_HALF:  Half transfer interrupt mask
  *     @arg @b DMA_INT_ERROR:  Transfer error interrupt mask
  *     @arg @b DMA_INT_BURST:  Burst block transmit complete interrupt mask
  * @param  NewState: new state of the specified DMA interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DMA_INTConfig(const DMA_CHNO_Type ChannelNO, uint32_t INTType, FunctionalState NewState)
{
  uint32_t ccr;
  assert_param(IS_DMA_CHNO_Type(ChannelNO));
  assert_param(IS_DMA_INT_Types(INTType));
  assert_param(IS_FUNCTION_STATE(NewState));
  ccr = DMA_CHx(ChannelNO)->CCR.V32;
  if (NewState != DISABLE)
  {
    /* Enable the selected DMA interrupts */
    ccr |= INTType;
  }
  else
  {
    /* Disable the selected DMA interrupts */
    ccr &= ~INTType;
  }
  /* disable enable bit first */
  DMA_CHx(ChannelNO)->CCR.V32 &= ~(DMA_CCR0_EN_Msk);
  DMA_CHx(ChannelNO)->CCR.V32 = ccr;

}

/** @brief  Get DMA ChannelNO's interrupt status 
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @param  INTStatus output this ChannelNO's interrupt flags
  * @retval None
  */
void DMA_GetINTStatus(const DMA_CHNO_Type ChannelNO, DMA_INTStatus_TypeDef* INTStatus)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));
    assert_param(IS_IN_CRAM(INTStatus) || IS_IN_SRAM(INTStatus));
    
    INTStatus->AllFlags = (DMA->IFSR.V32 >> ChannelNO);
    
    /* clear other bits */
    INTStatus->AllFlags &= (1 << DMA_IFSR_BEIF_Pos) | (1 << DMA_IFSR_TCIF_Pos) |(1 << DMA_IFSR_HTIF_Pos) |(1 << DMA_IFSR_TEIF_Pos);
}

/** @brief  clear interrupt flags 
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @param  INTStatus this ChannelNO's interrupt flags which need clear
  * @retval None
  */   
void DMA_ClearINTStatus(const DMA_CHNO_Type ChannelNO, const DMA_INTStatus_TypeDef* INTStatus)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));

    DMA->IFCR.V32 = (INTStatus->AllFlags << ChannelNO);    
}


/**@brief check DMA ChannelNO's busy flag 
  *@param ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  *@retval bool_t transmit complete flag
  *    @arg @b TRUE   DMA Channel is busy
  *    @arg @b FALSE  DMA Channel is free
  */  
bool_t DMA_IsBusy(const DMA_CHNO_Type ChannelNO)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));
    
    return ((DMA->CSR.BIT.DBUSY&(1<<ChannelNO)) ? TRUE : FALSE);
}



/** @brief  DMA Channel transmit complete 
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @retval bool_t transmit complete flag
  *    @arg @b TRUE   transmit complete
  *    @arg @b FALSE  transmitting
  */  
bool_t DMA_IsComplete(const DMA_CHNO_Type ChannelNO)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));

    return ((DMA->IFSR.V32 & (DMA_TRANS_COMPLETE << ChannelNO)) ? TRUE : FALSE);
}


/** @brief  DMA Channel transmit half complete 
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @retval bool_t transmit half complete flag
  *    @arg @b TRUE   has transmit half
  *    @arg @b FALSE  has not transmit half yet
  */  
bool_t DMA_IsHalfComplete(const DMA_CHNO_Type ChannelNO)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));

    return ((DMA->IFSR.V32 & (DMA_TRANS_HALF << ChannelNO)) ? TRUE : FALSE);    
}

/** @brief  DMA Channel one burst block transmit complete 
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @retval bool_t transmit half complete flag
  *    @arg @b TRUE   has transmit half
  *    @arg @b FALSE  has not transmit half yet
  */  
bool_t DMA_IsBlockComplete(const DMA_CHNO_Type ChannelNO)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));

    return ((DMA->IFSR.V32 & (DMA_TRANS_BURST << ChannelNO)) ? TRUE : FALSE);    
}
    


/** @brief  DMA Channel transmit error 
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @retval bool_t transmit half complete flag
  *    @arg @b TRUE   transmit error
  *    @arg @b FALSE  transmit succeed
  */  
bool_t DMA_IsError(const DMA_CHNO_Type ChannelNO)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));

    return ((DMA->IFSR.V32 & (DMA_TRANS_ERROR << ChannelNO)) ? TRUE : FALSE);    
}

/** @brief  Clear DMA Channel's transmit flags 
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @param  Flags DMA Transmit flags
  *   This parameter can be any combination of following values:
  *     @arg @b DMA_TRANS_BURST           One burst block transmit complete
  *     @arg @b DMA_TRANS_COMPLETE        All data transmit complete
  *     @arg @b DMA_TRANS_HALF            Half data transmit complete
  *     @arg @b DMA_TRANS_ERROR           Transmit Error
  * @retval None
  */  
void DMA_ClearFlag(const DMA_CHNO_Type ChannelNO, const uint32_t Flags)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));
    assert_param(IS_DMA_FLAG_Types(Flags));

    DMA->IFCR.V32 = (Flags << ChannelNO);    
}
    
/** @brief  Get DMA Channel's transmit flags 
  * @param  ChannelNO DMA ChannelNO number, can be DMA_CH_NO_0 ~ DMA_CH_NO_7
  * @param  Flag DMA Transmit flags
   *   This parameter can be one of following values: 
  *     @arg @b DMA_TRANS_BURST           One burst block transmit complete
  *     @arg @b DMA_TRANS_COMPLETE        All data transmit complete
  *     @arg @b DMA_TRANS_HALF            Half data transmit complete
  *     @arg @b DMA_TRANS_ERROR           Transmit Error
  * @retval FlagStatus
        @arg @b SET:   Flag is setted
        @arg @b RESET: Flag is not setted
  */  
FlagStatus DMA_GetFlagStatus(const DMA_CHNO_Type ChannelNO, const DMA_FLAG_Type Flag)
{
    assert_param(IS_DMA_CHNO_Type(ChannelNO));
    assert_param(IS_DMA_FLAG_Type(Flag));
    if(DMA->IFSR.V32 & (Flag << ChannelNO))
        return SET;
    else
        return RESET;
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


