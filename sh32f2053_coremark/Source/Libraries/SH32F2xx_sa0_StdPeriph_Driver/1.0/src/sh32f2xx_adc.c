/**
  ******************************************************************************
  * @file    sh32f2xx_adc.c
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
#include "sh32f2xx_adc.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup ADC_MODULE ADC
  * @brief ADC driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup ADC_Private_Functions
  * @{
  */ 

/** @defgroup ADC_Group1 Initialization and Configuration
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
  * @brief  Reset the ADC peripherals registers to their default reset values.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval None
  */
void ADC_Reset(ADC_TypeDef* ADCx)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    
    if (ADCx == ADC1)
    {
        RCC_AHBPeriphReset(RCC_AHB_ADC1);
    }
    else if (ADCx == ADC2)
    {
        RCC_AHBPeriphReset(RCC_AHB_ADC2);
    }
    else
    {
        if (ADCx == ADC3)
        {
            RCC_AHBPeriphReset(RCC_AHB_ADC3);
        }
    }
}

/**
  * @brief  Fills each ADC_InitStruct member with its default value. 
  * @param  ADC_InitStruct: pointer to a @ref ADC_InitTypeDef structure which will 
  *         be initialized.
  * @retval None
  */
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
{
    /* Initialize the ADC_Resolution member */
    ADC_InitStruct->ADC_Resolution = ADC_Resolution_12Bit;

    /* Initialize the ADC_Trigger member */
    ADC_InitStruct->ADC_Trigger = ADC_Trigger_Software;

    /* Initialize the ADC_RefVoltage member */
    ADC_InitStruct->ADC_RefVoltage = ADC_RefVoltage_AVDD;

    /* Initialize the ADC_ConversionMode member */
    ADC_InitStruct->ADC_ConversionMode = ADC_ConversionMode_Single;

    /* Initialize the ADC_ChannelPointer member */
    ADC_InitStruct->ADC_SequencePointer = ADC_Sequence_0;

    /* Initialize the ADC_NumOfConversion member */
    ADC_InitStruct->ADC_NumOfConversion = 1;

    /* Initialize the ADC_ClkPrescaler member */
    ADC_InitStruct->ADC_Prescaler = ADC_Prescaler_1;

    /* Initialize the ADC_TwoSampleGap member */
    ADC_InitStruct->ADC_TwoSampleGap = ADC_TwoSampleGap_0Cycles;

    /* Initialize the ADC_SampleTime member */
    ADC_InitStruct->ADC_SampleTime = ADC_SampleTime_2Cycles;
}

/**
  * @brief  Initializes the ADCx peripheral according to the specified parameters 
  *         in the ADC_InitStruct.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_InitStruct: pointer to a @ref ADC_InitTypeDef structure that contains
  *         the configuration information for the specified ADC peripheral.
  * @retval None
  */
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct)
{
    uint8_t pos = 0;
    uint16_t tempReg1 = 0;
    uint32_t tempReg2 = 0;

    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_RESOLUTION(ADC_InitStruct->ADC_Resolution));
    assert_param(IS_ADC_TRIGGER(ADC_InitStruct->ADC_Trigger));
    assert_param(IS_ADC_REF_VOLTAGE(ADC_InitStruct->ADC_RefVoltage));
    assert_param(IS_ADC_CONVERSION_MODE(ADC_InitStruct->ADC_ConversionMode));
    assert_param(IS_ADC_CONVERSION_NUMBER(ADC_InitStruct->ADC_NumOfConversion));
    assert_param(IS_ADC_CLK_PRESCALER(ADC_InitStruct->ADC_Prescaler));
    assert_param(IS_ADC_TWO_SAMPLE_GAP(ADC_InitStruct->ADC_TwoSampleGap));
    assert_param(IS_ADC_SAMPLE_TIME(ADC_InitStruct->ADC_SampleTime));
    

    tempReg1 = ADCx->ADCON1.V32;
    tempReg2 = ADCx->ADCON2.V32;

    /* Clear the ADCTU/MOD/REFC/ADSTRS bits */
    tempReg1 &= ~(ADC_ADCON1_ADCTU_Msk | ADC_ADCON1_MOD_Msk | ADC_ADCON1_REFC_Msk | ADC_ADCON1_ADSTRS_Msk);
    tempReg1 |= ADC_InitStruct->ADC_ConversionMode | ADC_InitStruct->ADC_Resolution \
                | ADC_InitStruct->ADC_RefVoltage | ADC_InitStruct->ADC_Trigger;
    ADCx->ADCON1.V32 = tempReg1;

    /* Clear the TADC/ADMAXCH/TS/TGAP bits */
    tempReg2 &= ~(ADC_ADCON2_TADC_Msk | ADC_ADCON2_ADMAXCH_Msk | ADC_ADCON2_TS_Msk | ADC_ADCON2_TGAP_Msk);
    tempReg2 |= ADC_InitStruct->ADC_Prescaler| ((ADC_InitStruct->ADC_NumOfConversion - 1) << 4) \
                | ADC_InitStruct->ADC_SampleTime | ADC_InitStruct->ADC_TwoSampleGap;
    
    if (ADC_InitStruct->ADC_ConversionMode == ADC_ConversionMode_Discontinuous)
    {
        /* Check the channel point register */
        assert_param(IS_ADC_SEQUENCE(ADC_InitStruct->ADC_SequencePointer));

        pos = (ADC_InitStruct->ADC_SequencePointer >> 2) - 3;
        
        ADCx->ADPCH &= ~ADC_ADPCH_ADPCH_Msk;
        
        /* To change this bit, ADSOC must be 0 */
        ADCx->ADPCH |= pos;
    }
    
    ADCx->ADCON2.V32 = tempReg2;
}

/**
  * @brief  Open or close the specified ADC peripheral.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  OnOffState: state of the ADC peripheral.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void ADC_OnOff(ADC_TypeDef* ADCx, CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_CMD_STATE(OnOffState));

    if (OnOffState != OFF)
    {
        /* Enable the ADC peripheral */
        ADCx->ADCON1.V32 |= ADC_ADCON1_ADON_Msk;
    }
    else
    {
        /* Disable the ADC peripheral */
        ADCx->ADCON1.V32 &= ~ADC_ADCON1_ADON_Msk;
    }
}

/**
  * @brief  Start conversion of the specified ADC peripheral.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval None
  */
void ADC_SoftwareStartConversion(ADC_TypeDef* ADCx)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));

    ADCx->ADCON1.V32 |= ADC_ADCON1_ADSOC_Msk;
}

/**
  * @brief  Stop conversion of the specified ADC peripheral.  
  * @note   This function only can be used in ADC Continuous Mode.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval None
  */
void ADC_SoftwareStopConversion(ADC_TypeDef* ADCx)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));

    ADCx->ADCON1.V32 &= ~ADC_ADCON1_ADSOC_Msk;
}


/**
  * @brief  Get start conversion of the specified ADC peripheral.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval ADC start status
  */
uint8_t ADC_GetSoftwareStartConversionStatus(ADC_TypeDef* ADCx)
{
    uint8_t ADC_StartStatus = 0;
    
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));

    if ((ADCx->ADCON1.V32 & ADC_ADCON1_ADSOC_Msk) != (uint32_t)RESET)
    {
        ADC_StartStatus = (uint8_t)SET;
    }
    else
    {
        ADC_StartStatus = (uint8_t)RESET;
    }

    return ADC_StartStatus;
}


/**
  * @brief  Configure the selected ADC channel for its sample sequence and whether enable or
  *         disable two sample gap function.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Channel: specifie ADC channel which to be configured.
  *          This parameter can be one of the following values:
  *           @arg ADC_Channel_0 
  *           @arg ADC_Channel_1
  *           @arg ADC_Channel_2
  *           @arg ADC_Channel_3
  *           @arg ADC_Channel_4
  *           @arg ADC_Channel_5
  *           @arg ADC_Channel_6
  *           @arg ADC_Channel_7
  *           @arg ADC_Channel_8
  *           @arg ADC_Channel_9
  *           @arg ADC_Channel_Vref
  *           @arg ADC_Channel_OP1OUT
  *           @arg ADC_Channel_OP2OUT
  *           @arg ADC_Channel_OP3OUT
  *           @arg ADC_Channel_TPS  
  * @param  ADC_Sequence: specifie sample sequence in sample group. 
  *          This parameter can be one of the following values:
  *            @arg ADC_Sequence_0
  *            @arg ADC_Sequence_1
  *            @arg ADC_Sequence_2
  *            @arg ADC_Sequence_3
  *            @arg ADC_Sequence_4
  *            @arg ADC_Sequence_5
  *            @arg ADC_Sequence_6
  *            @arg ADC_Sequence_7  
  * @param  ADC_TwoSampleGapCmd: enable or disable ADC two sample gap function.
  *          This parameter can be one of the following values:
  *            @arg ADC_TwoSampleGap_Enable
  *            @arg ADC_TwoSampleGap_Disable
  * @retval None
  */
void ADC_ChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t ADC_Sequence, uint8_t ADC_TwoSampleGapCmd)
{
    uint8_t pos = 0;
    uint32_t temp = 0;
    
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_CHANNEL(ADC_Channel));
    assert_param(IS_ADC_SEQUENCE(ADC_Sequence));
    assert_param(IS_ADC_TWO_SAMPLE_GAP_CMD(ADC_TwoSampleGapCmd));

    pos = (ADC_Sequence >> 2) - 3;

    ADC1->ADGAPON &= ~(0x01 << pos);
    ADC1->ADGAPON |= (ADC_TwoSampleGapCmd << pos);
    
    temp = ADCx->SEQCHSEL.V32;
    temp &= ~(ADC_SEQCHSEL_SEQCH0_Msk << (pos << 2));
    temp |= ADC_Channel << (pos << 2);
    ADCx->SEQCHSEL.V32 = temp;
}

/**
  * @brief  Configure external peripherals trigger ADC start to convert.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Trigger: specifie the ADC trigger to start conversion.
  *          This parameter can be one of the following values:
  *            @arg ADC_Trigger_Software
  *            @arg ADC_Trigger_ADTRG1
  *            @arg ADC_Trigger_ADTRG2
  *            @arg ADC_Trigger_ADTRG3 
  *            @arg ADC_Trigger_TIM8
  *            @arg ADC_Trigger_MCM1TRG1
  *            @arg ADC_Trigger_MCM1TRG2
  *            @arg ADC_Trigger_MCM1TRG3
  *            @arg ADC_Trigger_MCM1TRG4
  *            @arg ADC_Trigger_MCM2TRG1
  *            @arg ADC_Trigger_MCM2TRG2
  *            @arg ADC_Trigger_MCM2TRG3
  *            @arg ADC_Trigger_MCM2TRG4
  *            @arg ADC_Trigger_GPT0ADTRA
  *            @arg ADC_Trigger_GPT0ADTRB
  *            @arg ADC_Trigger_GPT1ADTRA  
  *            @arg ADC_Trigger_GPT1ADTRB
  *            @arg ADC_Trigger_GPT2ADTRA
  *            @arg ADC_Trigger_GPT2ADTRB
  *            @arg ADC_Trigger_GPT3ADTRA
  *            @arg ADC_Trigger_GPT3ADTRB  
  * @retval None
  */
void ADC_ExternalTriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_TRIGGER(ADC_Trigger));

    ADCx->ADCON1.V32 &= ~ADC_ADCON1_ADSTRS_Msk;
    ADCx->ADCON1.V32 |= ADC_Trigger;
}

/**
  * @brief  Configure external ADTRG pins trigger ADC start to convert.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Trigger: specifie the ADC trigger to start conversion.
  *          This parameter can any combination of the following values:
  *            @arg ADC_Trigger_ADTRG1
  *            @arg ADC_Trigger_ADTRG2
  *            @arg ADC_Trigger_ADTRG3  
  * @retval None
  */
void ADC_ExtADTRGTriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_ADTRG_TRIGGER(ADC_Trigger));

    if (NewState != DISABLE)
    {
        ADCx->ADCON1.V32 |= ADC_Trigger;
    }
    else
    {
        ADCx->ADCON1.V32 &= ~ADC_Trigger;
    }
}

/**
  * @brief  Configure TIM8 overflow trigger ADC start to convert.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Trigger: specifie the ADC trigger to start conversion.
  *          This parameter can be the following value:
  *            @arg ADC_Trigger_TIM8 
  * @retval None
  */
void ADC_ExtTIM8TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_TIM8_TRIGGER(ADC_Trigger));

    if (NewState != DISABLE)
    {
        ADCx->ADCON1.V32 |= ADC_Trigger;
    }
    else
    {
        ADCx->ADCON1.V32 &= ~ADC_Trigger;
    }
}

/**
  * @brief  Configure MCM1 trigger ADC start to convert.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Trigger: specifie the ADC trigger to start conversion.
  *          This parameter can be any combination of the following values:
  *            @arg ADC_Trigger_MCM1TRG1
  *            @arg ADC_Trigger_MCM1TRG2
  *            @arg ADC_Trigger_MCM1TRG3
  *            @arg ADC_Trigger_MCM1TRG4 
  * @retval None
  */
void ADC_ExtMCM1TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_MCM1_TRIGGER(ADC_Trigger));

    if (NewState != DISABLE)
    {
        ADCx->ADCON1.V32 |= ADC_Trigger;
    }
    else
    {
        ADCx->ADCON1.V32 &= ~ADC_Trigger;
    }

}

/**
  * @brief  Configure MCM2 trigger ADC start to convert.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Trigger: specifie the ADC trigger to start conversion.
  *          This parameter can be any combination of the following values:
  *            @arg ADC_Trigger_MCM2TRG1
  *            @arg ADC_Trigger_MCM2TRG2
  *            @arg ADC_Trigger_MCM2TRG3
  *            @arg ADC_Trigger_MCM2TRG4 
  * @retval None
  */
void ADC_ExtMCM2TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_MCM2_TRIGGER(ADC_Trigger));

    if (NewState != DISABLE)
    {
        ADCx->ADCON1.V32 |= ADC_Trigger;
    }
    else
    {
        ADCx->ADCON1.V32 &= ~ADC_Trigger;
    }

}

/**
  * @brief  Configure GPT0 trigger ADC start to convert.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Trigger: specifie the ADC trigger to start conversion.
  *          This parameter can be any combination of the following values:
  *            @arg ADC_Trigger_GPT0ADTRA
  *            @arg ADC_Trigger_GPT0ADTRB
  * @retval None
  */
void ADC_ExtGPT0TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_GPT0_TRIGGER(ADC_Trigger));

    if (NewState != DISABLE)
    {
        ADCx->ADCON1.V32 |= ADC_Trigger;
    }
    else
    {
        ADCx->ADCON1.V32 &= ~ADC_Trigger;
    }

}

/**
  * @brief  Configure GPT1 trigger ADC start to convert.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Trigger: specifie the ADC trigger to start conversion.
  *          This parameter can be any combination of the following values:
  *            @arg ADC_Trigger_GPT1ADTRA
  *            @arg ADC_Trigger_GPT1ADTRB
  * @retval None
  */
void ADC_ExtGPT1TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_GPT1_TRIGGER(ADC_Trigger));

    if (NewState != DISABLE)
    {
        ADCx->ADCON1.V32 |= ADC_Trigger;
    }
    else
    {
        ADCx->ADCON1.V32 &= ~ADC_Trigger;
    }

}

/**
  * @brief  Configure GPT2 trigger ADC start to convert.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Trigger: specifie the ADC trigger to start conversion.
  *          This parameter can be any combination of the following values:
  *            @arg ADC_Trigger_GPT2ADTRA
  *            @arg ADC_Trigger_GPT2ADTRB
  * @retval None
  */
void ADC_ExtGPT2TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_GPT2_TRIGGER(ADC_Trigger));

    if (NewState != DISABLE)
    {
        ADCx->ADCON1.V32 |= ADC_Trigger;
    }
    else
    {
        ADCx->ADCON1.V32 &= ~ADC_Trigger;
    }
}

/**
  * @brief  Configure GPT3 trigger ADC start to convert.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Trigger: specifie the ADC trigger to start conversion.
  *          This parameter can be any combination of the following values:
  *            @arg ADC_Trigger_GPT3ADTRA
  *            @arg ADC_Trigger_GPT3ADTRB
  * @retval None
  */
void ADC_ExtGPT3TriggerConfig(ADC_TypeDef* ADCx, uint16_t ADC_Trigger, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_GPT3_TRIGGER(ADC_Trigger));

    if (NewState != DISABLE)
    {
        ADCx->ADCON1.V32 |= ADC_Trigger;
    }
    else
    {
        ADCx->ADCON1.V32 &= ~ADC_Trigger;
    }
}



/**
  * @brief  Enable or disable the specified ADC interrupts.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_INT: specifie the ADC interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg ADC_INT_EOC
  *            @arg ADC_INT_ADG
  *            @arg ADC_INT_ADL
  * @param  NewState: new state of ADC's interrupts.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void ADC_INTConfig(ADC_TypeDef* ADCx, uint16_t ADC_INT, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_INT(ADC_INT));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        if ((ADC_INT & ADC_INT_EOC) == ADC_INT_EOC)
        {
            ADCx->ADCON1.V32 |= ADC_ADCON1_ADIE_Msk;
        }
        if ((ADC_INT & (uint16_t)0x0400) == (uint16_t)0x0400)
        {
            ADCx->ADCMPCON.V32 |= (ADC_INT & (uint16_t)0x00A0);
        }
    }
    else
    {
        if ((ADC_INT & ADC_INT_EOC) == ADC_INT_EOC)
        {
            ADCx->ADCON1.V32 &= ~ADC_ADCON1_ADIE_Msk;
        }
        if ((ADC_INT & (uint16_t)0x0400) == (uint16_t)0x0400)
        {
            ADCx->ADCMPCON.V32 &= ~(ADC_INT & (uint16_t)0x00A0);
        }

    }   
}

/**
  * @brief  Enable or disable the specified ADC DMA requests.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_DMASource: sepcifie the ADC's DMA request sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg ADC_DMA_EOC
  *            @arg ADC_DMA_ADG
  *            @arg ADC_DMA_ADL
  * @param  NewState: new state of ADC DMA request sources.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE  
  * @retval None
  */
void ADC_DMAConfig(ADC_TypeDef* ADCx, uint16_t ADC_DMASource, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_DMA_SOURCE(ADC_DMASource));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        if ((ADC_DMASource & ADC_DMA_EOC) == ADC_DMA_EOC)
        {
            ADCx->ADCON1.V32 |= ADC_ADCON1_ADDE_Msk;
        }
        if ((ADC_DMASource & (uint16_t)0x0400) == (uint16_t)0x0400)
        {
            ADCx->ADCMPCON.V32 |= (ADC_DMASource & (uint16_t)0x0050);
        }
    }
    else
    {
        if ((ADC_DMASource & ADC_DMA_EOC) == ADC_DMA_EOC)
        {
            ADCx->ADCON1.V32 &= ~ADC_ADCON1_ADDE_Msk;
        }
        if ((ADC_DMASource & (uint16_t)0x0400) == (uint16_t)0x0400)
        {
            ADCx->ADCMPCON.V32 &= ~(ADC_DMASource & (uint16_t)0x0050);
        }
    }
    

}

/**
  * @brief  Get the specified ADC's channel data.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Sequence: specifie the ADC's Sample Sequence to get data.
  *          This parameter can be one of the following values:
  *            @arg ADC_Sequence_0
  *            @arg ADC_Sequence_1
  *            @arg ADC_Sequence_2
  *            @arg ADC_Sequence_3  
  *            @arg ADC_Sequence_4
  *            @arg ADC_Sequence_5
  *            @arg ADC_Sequence_6
  *            @arg ADC_Sequence_7    
  * @retval The specified Channel data.
  */
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx, uint16_t ADC_Sequence)
{
    uint16_t temp = 0;
    
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));

    assert_param(IS_ADC_SEQUENCE(ADC_Sequence));

    temp = (*(volatile unsigned long *)((uint32_t)ADCx + ADC_Sequence));
    return temp;
}

/**
  * @brief  Compare the ADC conversion value with the Max and Min data set in registers.  
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_UpLimit: specifie the Max value to be compared with ADC conversion value.
  *          This parameter can be 0 to 0xFFFF.
  * @param  ADC_LowerLimit: specifie the Min value to be compared with ADC conversion value.
  *          This parameter can be 0 to 0xFFFF.  
  * @param  ADC_Sequence: specifie the ADC sample sequence to be compared.
  *          This parameter can be one of the following values:
  *            @arg ADC_Sequence_0
  *            @arg ADC_Sequence_1
  *            @arg ADC_Sequence_2
  *            @arg ADC_Sequence_3
  *            @arg ADC_Sequence_4
  *            @arg ADC_Sequence_5
  *            @arg ADC_Sequence_6
  *            @arg ADC_Sequence_7  
  * @retval None
  */
void ADC_SequenceResultCompareConfig(ADC_TypeDef* ADCx, uint16_t ADC_UpLimit, uint16_t ADC_LowerLimit, uint8_t ADC_Sequence)
{
    uint8_t pos = 0;
    uint16_t temp = 0;
    
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_SEQUENCE(ADC_Sequence));

    pos = (ADC_Sequence >> 2) - 3;

    temp = ADCx->ADCMPCON.V32;
    temp &= ~ADC_ADCMPCON_CSEL_Msk;
    temp |= pos;
    ADCx->ADCMPCON.V32 = temp;

    ADCx->ADDGT = ADC_UpLimit;
    ADCx->ADDLT = ADC_LowerLimit;
}

/**
  * @brief  Check whether the specified ADC flag is set or not
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Flag: specifie the flag to check.
  *          This parameter can be one of the following values:
  *            @arg ADC_FLAG_EOC
  *            @arg ADC_FLAG_ADG
  *            @arg ADC_FLAG_ADL  
  * @retval None
  */
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint16_t ADC_Flag)
{
    FlagStatus bitStatus;
    
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_GET_ONE_FLAG(ADC_Flag));

    if ((ADCx->ADINTF.V32 & ADC_Flag) != (uint32_t)RESET)
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
  * @brief  Clear the ADC's pending flags.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Flag: specifie the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg ADC_FLAG_EOC
  *            @arg ADC_FLAG_ADG
  *            @arg ADC_FLAG_ADL  
  * @retval None
  */
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint16_t ADC_Flag)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_FLAG(ADC_Flag));

    ADCx->ADINTF.V32 = ADC_Flag << 16;
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


