/**
  ******************************************************************************
  * @file    sh32f2xx_qei.c
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
#include "sh32f2xx_qei.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup QEI_MODULE QEI 
  * @brief QEI driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup QEI_Private_Functions
  * @{
  */ 


/**
  * @brief  Reset the QEI peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
void QEI_Reset(void)
{
    RCC_APB1PeriphReset(RCC_APB1_QEI);
}

/**
  * @brief  Fills each QEI_InitStruct member with its default value.
  * @param  QEI_InitStruct : pointer to a @ref QEI_InitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
void QEI_StructInit(QEI_InitTypeDef* QEI_InitStruct)
{
    QEI_InitStruct->QEI_PosCntMode = QEI_PosCntMode_2Edge_IndexRST;
    QEI_InitStruct->QEI_Prescaler = 0x00;
    QEI_InitStruct->QEI_PosCntMaxValue = 0x00;
    QEI_InitStruct->QEI_IndexRSTPosConterCmd = QEI_IndexRSTPosConter_Disable;
    QEI_InitStruct->QEI_ABSwapCmd = QEI_ABSwap_Disable;
    QEI_InitStruct->QEI_IndexFilterCmd = QEI_IndexFilter_Disable;
    QEI_InitStruct->QEI_IndexFilterPrescaler = QEI_IndexFilterPrescaler_1;
    QEI_InitStruct->QEI_QEABFilterCmd = QEI_QEABFilter_Disable;
    QEI_InitStruct->QEI_QEABFilterPrescaler = QEI_QEABFilterPrescaler_1;
    QEI_InitStruct->QEI_QTimerPrescaler = QEI_QTimerPrescaler_1;
    QEI_InitStruct->QEI_QTimerPeriod = 0x00;
    QEI_InitStruct->QEI_PosCounterMinValue = 0x00;
}

/**
  * @brief  Initialize the QEI peripheral according to the specified parameters in the QEI_InitStruct.  
  * @param  QEI_InitStruct: pointer to a QEI_InitTypeDef structure that contains
  *         the configuration information for the QEI peripheral.
  * @retval None
  */
void QEI_Init(QEI_InitTypeDef* QEI_InitStruct)
{
    uint16_t tempReg1 = 0;
    uint16_t tempReg2 = 0;

    /* Check the parameters */
    assert_param(IS_QEI_POS_CNT_MODE(QEI_InitStruct->QEI_PosCntMode));
    assert_param(IS_QEI_INDEX_RST_POS_COUNTER(QEI_InitStruct->QEI_IndexRSTPosConterCmd));
    assert_param(IS_QEI_AB_SWAP(QEI_InitStruct->QEI_ABSwapCmd));
    assert_param(IS_QEI_INDEX_FILTER(QEI_InitStruct->QEI_IndexFilterCmd));
    assert_param(IS_QEI_INDEX_FILTER_PRESCALER(QEI_InitStruct->QEI_IndexFilterPrescaler));
    assert_param(IS_QEI_AB_FILTER(QEI_InitStruct->QEI_QEABFilterCmd));
    assert_param(IS_QEI_QEAB_FILTER_PRESCALER(QEI_InitStruct->QEI_QEABFilterPrescaler));
    assert_param(IS_QEI_QTIMER_PRESCALER(QEI_InitStruct->QEI_QTimerPrescaler));
    
    tempReg1 = QEI->QEICON.V32;
    tempReg2 = QEI->QFLTCON.V32;

    tempReg1 &= ~(QEI_QEICON_QPMOD_Msk | QEI_QEICON_QIDXEN_Msk | QEI_QEICON_QSWAP_Msk \
                 | QEI_QEICON_QTEPS_Msk);
    tempReg1 |= QEI_InitStruct->QEI_PosCntMode | QEI_InitStruct->QEI_IndexRSTPosConterCmd \
                | QEI_InitStruct->QEI_ABSwapCmd | QEI_InitStruct->QEI_QTimerPrescaler;
    QEI->QEICON.V32 = tempReg1;

    tempReg2 &= ~(QEI_QFLTCON_QABCPS_Msk | QEI_QFLTCON_QABFEN_Msk | QEI_QFLTCON_QIDXCPS_Msk \
                | QEI_QFLTCON_QIDXFEN_Msk);
    tempReg2 |= QEI_InitStruct->QEI_IndexFilterCmd | QEI_InitStruct->QEI_IndexFilterPrescaler \
                | QEI_InitStruct->QEI_QEABFilterCmd | QEI_InitStruct->QEI_QEABFilterPrescaler;
    QEI->QFLTCON.V32 = tempReg2;
    
    QEI->QTPSQ = QEI_InitStruct->QEI_Prescaler;
    QEI->QPOSMAX = QEI_InitStruct->QEI_PosCntMaxValue;
    QEI->QTPR = QEI_InitStruct->QEI_QTimerPeriod;
    QEI->QCNTMIN = QEI_InitStruct->QEI_PosCounterMinValue;
                
}

/**
  * @brief  Open or close the QEI peripheral.  
  * @param  OnOffState: state of the QEI peripheral.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void QEI_OnOff(CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_CMD_STATE(OnOffState));

    if (OnOffState != OFF)
    {
        /* Enable the QEI peripheral */
        QEI->QEICON.V32 |= QEI_QEICON_QEIEN_Msk; 
    }
    else
    {
        /* Disable the QEI peripheral */
        QEI->QEICON.V32 &= ~QEI_QEICON_QEIEN_Msk;
    }
}

/**
  * @brief  Open or close the QTimer function.  
  * @param  OnOffState: state of the QTimer function.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void QEI_QTimerOnOff(CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_CMD_STATE(OnOffState));

    if (OnOffState != OFF)
    {
        /* Enable the QTimer */
        QEI->QEICON.V32 |= QEI_QEICON_QTSR_Msk; 
    }
    else
    {
        /* Disable the QTimer */
        QEI->QEICON.V32 &= ~QEI_QEICON_QTSR_Msk;
    }
}

/**
  * @brief  Set the QEI's position counter register new value.  
  * @param  QEI_PosCounter: new value of the Position Register.
  *          This parameter can be 0 to 0xFFFF.
  * @retval None
  */
void QEI_SetPositionCounter(uint16_t QEI_PosCounter)
{
    QEI->QPOSCNT = QEI_PosCounter;
}

/**
  * @brief  Get the QEI's position counter register value.  
  * @param  None
  * @retval The Position Register value.
  */
uint16_t QEI_GetPositonCounter(void)
{
    return QEI->QPOSCNT;
}

/**
  * @brief  Set the QEI's QTimer counter register new value.  
  * @param  QEI_QTimerCounter: new value of the QTimer Counter Register.
  *          This parameter can be 0 to 0xFFFFFFFF.
  * @retval None
  */
void QEI_SetQTimerCounter(uint32_t QEI_QTimerCounter)
{
    QEI->QCNT = QEI_QTimerCounter;
}

/**
  * @brief  Get the QEI's QTimer counter register value.  
  * @param  None
  * @retval The QTimer Counter Register value.
  */
uint32_t QEI_GetQTimerCounter(void)
{
    return QEI->QCNT;
}


/**
  * @brief  Get latch register value of the QTimer's Counter.  
  * @param  None
  * @retval Value of QTimer's Counter.
  */
uint32_t QEI_GetCaptureLatchValue(void)
{
    return QEI->QTMLAT;
}

/**
  * @brief  Enable or disable the QEI's DMA requests.
  * @param  QEI_DMASource: specifie the DMA request sources.
  *          This parameter can be any combination of the following values:
  *            @arg QEI_DMA_PE: QEI phase error DMA source
  *            @arg QEI_DMA_CE: 
  *            @arg QEI_DMA_Update:
  *            @arg QEI_DMA_CAP: 
  *            @arg QEI_DMA_OV:   
  * @param  NewState: new state of the DMA request sources.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void QEI_DMAConfig(uint16_t QEI_DMASource, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_QEI_DMA_SOURCE(QEI_DMASource));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        QEI->QEIINT.V32 |= QEI_DMASource;
    }
    else
    {
        QEI->QEIINT.V32 &= ~QEI_DMASource;
    }
}

/**
  * @brief  Enable or disable the QEI's interrupts.
  * @param  QEI_IT: specifie the QEI interrupts to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg QEI_INT_PE: QEI phase error interrupt source
  *            @arg QEI_INT_CE: 
  *            @arg QEI_INT_Update:
  *            @arg QEI_INT_CAP: 
  *            @arg QEI_INT_OV:   
  * @param  NewState: new state of the interrupts.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void QEI_INTConfig(uint16_t QEI_INT, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_QEI_INT(QEI_INT));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        QEI->QEIINT.V32 |= QEI_INT;
    }
    else
    {
        QEI->QEIINT.V32 &= ~QEI_INT;
    }
}

/**
  * @brief    
  * @param  None
  * @retval None
  */
FlagStatus QEI_GetPosCounterDir(void)
{
    FlagStatus bitStatus;

    if ((QEI->QEICON.V32 & QEI_QEICON_QPDIR_Msk) != (uint32_t)RESET)
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
  * @brief  Check whether the specified QEI flag is set or not.
  * @param  QEI_Flag: specifie the QEI flag to check.
  *          This parameter can be one of the following values:
  *            @arg QEI_FLAG_PE: 
  *            @arg QEI_FLAG_CE: 
  *            @arg QEI_FLAG_Update:
  *            @arg QEI_FLAG_CAP: 
  *            @arg QEI_FLAG_OV:   
  * @retval None
  */
FlagStatus QEI_GetFlagStatus(uint16_t QEI_Flag)
{
    FlagStatus bitStatus;
    
    /* Check the parameters */
    assert_param(IS_QEI_GET_ONE_FLAG(QEI_Flag));   

    if ((QEI->QTINTF.V32 & QEI_Flag) != (uint32_t)RESET)
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
  * @brief  Clear the QEI's pending flags.
  * @param  QEI_Flag: specifie the flags to be clear.
  *          This parameter can be any combination of the following values:
  *            @arg QEI_FLAG_PE: 
  *            @arg QEI_FLAG_CE: 
  *            @arg QEI_FLAG_Update:
  *            @arg QEI_FLAG_CAP: 
  *            @arg QEI_FLAG_OV:      
  * @retval None
  */
void QEI_ClearFlag(uint16_t QEI_Flag)
{
    /* Check the parameters */
    assert_param(IS_QEI_FLAG(QEI_Flag));

    QEI->QTINTF.V32 = QEI_Flag << 16;
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


