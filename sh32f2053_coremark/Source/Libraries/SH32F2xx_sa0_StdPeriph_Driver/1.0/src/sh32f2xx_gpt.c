/**
  ******************************************************************************
  * @file    sh32f2xx_gpt.c
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
#include "sh32f2xx_gpt.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup GPT_MODULE GPT 
  * @brief GPT driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup GPT_Private_Functions
  * @{
  */ 

/** @defgroup GPT_Group1 Initialization and Configuration
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
  * @brief  Enable or disable the GPT Common registers write protection.
  * @param  NewState: new state of GPT Common registers protection.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void GPT_CommonWriteProtect(FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the GPT common registers write protection */
        GPT->GTPRWEN = 0x00;
    }
    else
    {
        /* Disable the GPT common registers write protection */
        GPT->GTPRWEN = (uint16_t)0x33CC;
    }
}



/**
  * @brief  Configure exteral interrupt filter parameters.
  * @param  ClockDivision: specifie clock division value.
  *          This parameter can be one of the following values:
  *            @arg GPT_CLK_DIV_1
  *            @arg GPT_CLK_DIV_4
  *            @arg GPT_CLK_DIV_16
  *            @arg GPT_CLK_DIV_128  
  * @param  SampleNum: specifie sample number value.
  *          This parameter can be one of the following values:
  *            @arg GPT_SAMPLE_NUM_1
  *            @arg GPT_SAMPLE_NUM_2
  *            @arg GPT_SAMPLE_NUM_3
  *            @arg GPT_SAMPLE_NUM_4  
  * @retval None
  */
void GPT_CommonExternalInputLineFilter(uint8_t GPT_CLKDiv, uint8_t GPT_SampleNum)
{
    /* Check the parameters */
    assert_param(IS_GPT_CLK_DIV(GPT_CLKDiv));
    assert_param(IS_GPT_SAMPLE_NUM(GPT_SampleNum));

    /* Reset the PS and SN bit */
    GPT->GTETINT.V32 &= ~(GPT_GTETINT_PS_Msk | GPT_GTETINT_SN_Msk);

    GPT->GTETINT.V32 |= (GPT_CLKDiv | GPT_SampleNum);
}

/**
  * @brief  configure the Port Control line filter mode.
  * @param  GPT_CommonFilter: specifie filter parameter for common port control(POE) line.
  *          This parameter can be one of the following values:
  *            @arg GPT_POE_Filter_None
  *            @arg GPT_POE_Filter_8
  *            @arg GPT_POE_Filter_16
  *            @arg GPT_POE_Filter_128  
  * @retval None
  */
void GPT_CommonPortCtrlLineFilter(uint8_t GPT_CommonFilter)
{
    /* Check the parameters */
    assert_param(IS_GPT_POE_FILTER(GPT_CommonFilter));
    
    GPT->GTPOECR.V32 &= ~GPT_GTPOECR_POEM_Msk;
    GPT->GTPOECR.V32 |= GPT_CommonFilter;
}

/**
  * @brief  Enable or disable GPT port control register Hiz bits.
  * @param  GPT_CommonHiz: specifie which Hiz bits to be enabed or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_HIZ_GPT0AB
  *            @arg GPT_HIZ_GPT1AB
  *            @arg GPT_HIZ_GPT2AB
  *            @arg GPT_HIZ_GPT3AB  
  * @param  NewState: new state of GPT port control line Hiz function.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE  
  * @retval None
  */
void GPT_CommonPortCtrlHizConfig(uint8_t GPT_CommonHiz, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_GPT_HIZ(GPT_CommonHiz));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable Hiz output of the GPT common port control register */
        GPT->GTPOECR.V32 |= GPT_CommonHiz;
    }
    else
    {
        /* Disable Hiz output of the GPT common port control register */
        GPT->GTPOECR.V32 &= ~GPT_CommonHiz;
    }       
}

/**
  * @brief  Enable or disable GPT common interrupts.
  * @param  GPT_CommonINT: specifie the GPT common interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_COMMON_INT_POE 
  *            @arg GPT_COMMON_INT_ETIP
  *            @arg GPT_COMMON_INT_ETIN  
  * @param  NewState: new state of GPT common interrupt sources.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void GPT_CommonINTConfig(uint8_t GPT_CommonINT, FunctionalState NewState)
{     
    /* Check the parameters */
    assert_param(IS_GPT_COMMON_INT(GPT_CommonINT));
    assert_param(IS_FUNCTION_STATE(NewState));
    
    if ((GPT_CommonINT & GPT_COMMON_INT_POE) == GPT_COMMON_INT_POE)
    {
        GPT_GTPOECR_POEIE_BIT = (uint32_t)NewState; 
    }

    if ((GPT_CommonINT & GPT_COMMON_INT_ETIP) == GPT_COMMON_INT_ETIP)
    {
        GPT_GTETINT_ETIPEN_BIT = (uint32_t)NewState;
    }

    if ((GPT_CommonINT & GPT_COMMON_INT_ETIN) == GPT_COMMON_INT_ETIN)
    {
        GPT_GTETINT_ETINEN_BIT = (uint32_t)NewState;
    }    
}

/**
  * @brief  Enable or disable the GPT common DMA requests.
  * @param  GPT_CommonDMASource: specifie the GPT common DMA request sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_COMMON_DMA_ETIP
  *            @arg GPT_COMMON_DMA_ETIN  
  * @param  NewState: new state of GPT common DMA request sources.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE  
  * @retval None
  */
void GPT_CommonDMAConfig(uint8_t GPT_CommonDMASource, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_GPT_COMMON_DMA_SOURCE(GPT_CommonDMASource));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable GPT common DMA Requests sources */
        GPT->GTETINT.V32 |= GPT_CommonDMASource;
    }
    else
    {
        /* Disable GPT common DMA Requests sources */
        GPT->GTETINT.V32 &= ~GPT_CommonDMASource;
    }    
}

/**
  * @brief  Get GPT global flag status.
  * @param  GPT_GlobalFlag: specifie interrupt flag source to be check.
  *          This parameter can be one of the following values:
  *            @arg GPT_COMMON_FLAG_ETIP
  *            @arg GPT_COMMON_FLAG_ETIN
  *            @arg GPT_COMMON_FLAG_GTPOE
  * @retval The global interrupt flag status
  */
FlagStatus GPT_CommonGetFlagStatus(uint8_t GPT_CommonFlag)
{
    FlagStatus bitStatus;

    /* Check the parameters */
    assert_param(IS_GPT_COMMON_GET_ONE_FLAG(GPT_CommonFlag));

    if ((GPT->GTINTF.V32 & GPT_CommonFlag) != (uint32_t)RESET)
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
  * @brief  Clear GPT global flag.
  * @param  GPT_GlobalFlag: specifie interrupt flag source to be cleared.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_GLOBAL_FLAG_ETIP
  *            @arg GPT_GLOBAL_FLAG_ETIN
  *            @arg GPT_GLOBAL_FLAG_GTPOE
  * @retval None
  */
void GPT_CommonClearFlag(uint8_t GPT_CommonFlag)
{
    /* Check the parameters */
    assert_param(IS_GPT_COMMON_FLAG(GPT_CommonFlag));

    GPT->GTINTF.V32 = (uint32_t)GPT_CommonFlag << 16;
}


/**
  * @brief  Enable or disable the GPT peripheral write protection.
  * @param  GPTx: where x can be 0 to 3 to select the GPT peripheral.
  * @param  NewState: new state of the GPT write protection.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void GPT_WriteProtect(GPT_TypeDef* GPTx, FunctionalState NewState)
{
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        GPTx->GTWP = 0x00;
    }
    else
    {
        GPTx->GTWP = 0x55;
    }
}

/**
  * @brief  On or off specified GPT peripheral.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  OnOffState: state of the GPT peripheral.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void GPT_OnOff(GPT_TypeDef* GPTx, CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_CMD_STATE(OnOffState));
    
    if (GPTx == GPT0)
    {
        GPT_GTSTR_CST0_BIT = (uint32_t)OnOffState;
    }
    else if (GPTx == GPT1)
    {
        GPT_GTSTR_CST1_BIT = (uint32_t)OnOffState;
    }
    else if (GPTx == GPT2)
    {
        GPT_GTSTR_CST2_BIT = (uint32_t)OnOffState;
    }
    else
    {
        GPT_GTSTR_CST3_BIT = (uint32_t)OnOffState;
    }
}


/*bug: below function may modify GTUDC and GTCR register */


/**
  * @brief  Initialize the GPTx Time Base Unit peripheral according to 
  *         the specified parameters in the GPT_TimeBaseInitStruct.
  * @param  GPTx: where x can be 0 to 3 to select the GPT peripheral.
  * @param  GPT_TimeBaseInitStruct: pointer to a @ref GPT_TimeBaseInitTypeDef structure
  *         that contains the configuration information for the specified GPT peripheral.
  * @retval None
  */
void GPT_TimeBaseInit(GPT_TypeDef* GPTx, GPT_TimeBaseInitTypeDef* GPT_TimeBaseInitStruct)
{
	/* Check the parameters */
	assert_param(IS_GPT_ALL_PERIPH(GPTx));
	assert_param(IS_GPT_COUNTER_MODE(GPT_TimeBaseInitStruct->CounterMode));

	/* Reset the MD bits of GTCR register */
	GPTx->GTCR.V32 &= ~GPT_GTCR_MD_Msk;

	if (GPT_TimeBaseInitStruct->CounterMode == GPT_CounterMode_Center)
	{
		GPTx->GTUDC.V32 = 0x02;
		GPTx->GTCR.V32 |= 0x04;
	}
	else
	{
		GPTx->GTUDC.V32 = GPT_TimeBaseInitStruct->CounterMode;
	}

    /* Configure the Prescaler and Period value */
	GPTx->GTPSQ = GPT_TimeBaseInitStruct->Prescaler;
	GPTx->GTPR = GPT_TimeBaseInitStruct->Period;
}



/**
  * @brief  Initialize the GPTx Channel_A according to the specified parameters in
  *         the TIM_OCInitStruct.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_OCInitStruct: pointer to a @ref GPT_OCInitTypeDef structure than contains
  *         the configuration information for the specified GPT peripheral.
  * @retval None
  */
void GPT_OCAInit(GPT_TypeDef* GPTx, GPT_OCInitTypeDef* GPT_OCInitStruct)
{
	/* Check the parameters */
	assert_param(IS_GPT_ALL_PERIPH(GPTx));
	assert_param(IS_GPT_WAVE_MODE(GPT_OCInitStruct->WaveMode));
	assert_param(IS_GPT_START_LEVEL(GPT_OCInitStruct->StartLevel));
	assert_param(IS_GPT_PERIOD_STATUS(GPT_OCInitStruct->PeriodStatus));
	assert_param(IS_GPT_COMPARE_STATUS(GPT_OCInitStruct->CompareStatus));

    GPTx->GTCR.V32 &= ~GPT_GTCR_MD_Msk;
    GPTx->GTCR.V32 |= GPT_OCInitStruct->WaveMode;
    
    GPTx->GTIOR.V32 &= ~GPT_GTIOR_GTIOA_Msk;

    GPTx->GTIOR.V32 |= GPT_OCInitStruct->StartLevel | GPT_OCInitStruct->PeriodStatus \
                       | GPT_OCInitStruct->CompareStatus;

    GPTx->GTCCRA = GPT_OCInitStruct->Pulse;   
}

/**
  * @brief  Initialize the GPTx Channel_B according to the specified parameters in
  *         the TIM_OCInitStruct.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_OCInitStruct: pointer to a @ref GPT_OCInitTypeDef structure than contains
  *         the configuration information for the specified GPT peripheral.
  * @retval None
  */
void GPT_OCBInit(GPT_TypeDef* GPTx, GPT_OCInitTypeDef* GPT_OCInitStruct)
{
	/* Check the parameters */
	assert_param(IS_GPT_ALL_PERIPH(GPTx));
	assert_param(IS_GPT_WAVE_MODE(GPT_OCInitStruct->WaveMode));
	assert_param(IS_GPT_START_LEVEL(GPT_OCInitStruct->StartLevel));
	assert_param(IS_GPT_PERIOD_STATUS(GPT_OCInitStruct->PeriodStatus));
	assert_param(IS_GPT_COMPARE_STATUS(GPT_OCInitStruct->CompareStatus));

    GPTx->GTCR.V32 &= ~GPT_GTCR_MD_Msk;
    GPTx->GTCR.V32 |= GPT_OCInitStruct->WaveMode;

    GPTx->GTIOR.V32 &= ~GPT_GTIOR_GTIOB_Msk;

    GPTx->GTIOR.V32 |= (GPT_OCInitStruct->StartLevel | GPT_OCInitStruct->PeriodStatus \
                       | GPT_OCInitStruct->CompareStatus) << 8;

    GPTx->GTCCRB = GPT_OCInitStruct->Pulse;	
}


/**
  * @brief  Configure the GPTx start stop level retain to register or keep.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_Channel: specifie whether A or B channel to set.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_Channel_A
  *            @arg GPT_Channel_B
  * @param  GPT_Retain: specifie output start stop level retain register or keep.
  *          This parameter can be one of the following values:
  *            @arg GPT_Retain_Register
  *            @arg GPT_Retain_Keep
  * @retval None
  */
void GPT_OCStartStopRetainConfig(GPT_TypeDef* GPTx, uint16_t GPT_Channel, uint16_t GPT_Retain)
{
    /* Check the parameters */
	assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_CHANNEL(GPT_Channel));
    assert_param(IS_GPT_RETAIN(GPT_Retain));

    if ((GPT_Channel & GPT_Channel_A) == GPT_Channel_A)
    {
        GPTx->GTIOR.V32 &= ~GPT_GTIOR_OAHLD_Msk;
        GPTx->GTIOR.V32 |= GPT_Retain;
    }

    if ((GPT_Channel & GPT_Channel_B) == GPT_Channel_B)
    {
        GPTx->GTIOR.V32 &= ~GPT_GTIOR_OBHLD_Msk;
        GPTx->GTIOR.V32 |= GPT_Retain << 8;
    }
}


/**
  * @brief  Configure the GPTx stop level which can be set to low or high.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_Channel: specifie whether A or B channel to set.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_Channel_A
  *            @arg GPT_Channel_B
  * @param  GPT_StopLevel: specifie the low or high of the stop level.
  *          This parameter can be one of the following values:
  *            @arg GPT_StopLevel_Low
  *            @arg GPT_StopLevel_High
  * @retval None
  */
void GPT_OCStopLevelConfig(GPT_TypeDef* GPTx, uint16_t GPT_Channel, uint16_t GPT_StopLevel)
{
    /* Check the parameters */
	assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_CHANNEL(GPT_Channel));
    assert_param(IS_GPT_STOP_LEVEL(GPT_StopLevel));

    if ((GPT_Channel & GPT_Channel_A) == GPT_Channel_A)
    {
        GPTx->GTIOR.V32 &= ~GPT_GTIOR_OADFLT_Msk;
        GPTx->GTIOR.V32 |= GPT_StopLevel;
    }

    if ((GPT_Channel & GPT_Channel_B) == GPT_Channel_B)
    {
        GPTx->GTIOR.V32 &= ~GPT_GTIOR_OBDFLT_Msk;
        GPTx->GTIOR.V32 |= GPT_StopLevel << 8;
    }
}


/**
  * @brief  Initialize the GPT peripheral according to the specified parameters
  *         in the GPT_ICInitStruct.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_ICInitStruct: pointer to a @ref GPT_ICInitTypeDef structure that contains
  *         the configuration information for the specified GPT peripheral.
  * @retval None
  */
void GPT_ICInit(GPT_TypeDef* GPTx, GPT_ICInitTypeDef* GPT_ICInitStruct)
{
	/* Check the parameters */
	assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_CHANNEL(GPT_ICInitStruct->GPT_Channel));
    assert_param(IS_GPT_IC_POLARITY(GPT_ICInitStruct->GPT_ICPolarity));
    assert_param(IS_GPT_IC_SELECT(GPT_ICInitStruct->GPT_ICSelection));
    assert_param(IS_GPT_IC_Filter(GPT_ICInitStruct->GPT_ICFilter));

    if ((GPT_ICInitStruct->GPT_Channel & GPT_Channel_A) == GPT_Channel_A)
    {
        GPTx->GTIOR.V32 &= ~GPT_GTIOR_GTIOA_Msk;
        GPTx->GTIOR.V32 |= GPT_ICInitStruct->GPT_ICSelection | GPT_ICInitStruct->GPT_ICPolarity \
                           | (uint16_t)0x0020; 
    }
    
    if ((GPT_ICInitStruct->GPT_Channel & GPT_Channel_B) == GPT_Channel_B)
    {
        GPTx->GTIOR.V32 &= ~GPT_GTIOR_GTIOB_Msk;
        GPTx->GTIOR.V32 |= (GPT_ICInitStruct->GPT_ICSelection | GPT_ICInitStruct->GPT_ICPolarity \
                           | (uint16_t)0x0020) << 8;
    }

    GPTx->GTDEB.V32 &= ~GPT_GTDEB_DEB_Msk;
    GPTx->GTDEB.V32 |= GPT_ICInitStruct->GPT_ICFilter;
}

/**
  * @brief  Enable or disable the GPTx master slave function.
  * @note   This function only can be used for GPT0 and GPT2.
  * @param  GPTx: where x can be 0 or 2 to select the GPT peripheral.
  * @param  NewState: new state of the master slave function.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void GPT_CascadeConfig(GPT_TypeDef* GPTx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        GPTx->GTCR.V32 |= GPT_GTCR_TPSC_Msk;
    }
    else
    {
        GPTx->GTCR.V32 &= ~GPT_GTCR_TPSC_Msk;
    }
}

/**
  * @brief  Enable or disable the GPTx output Hiz.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  NewState: new state of the output Hiz.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void GPT_OutputHizConfig(GPT_TypeDef* GPTx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        GPTx->GTCR.V32 |= GPT_GTCR_HIZ_Msk;
    }
    else
    {
        GPTx->GTCR.V32 &= ~GPT_GTCR_HIZ_Msk;
    }
}

/**
  * @brief  Enable or disable the GPTx master slave pin association function.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_Channel: specifie GPT Channel to set. 
  *          This parameter can be one of the following values:
  *            @arg GPT_Channel_A
  *            @arg GPT_Channel_B
  * @param  NewState: new state of the master slave pin association.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE  
  * @retval None
  */
void GPT_CascadePinAssociation(GPT_TypeDef* GPTx, uint16_t GPT_Channel, FunctionalState NewState)
{
    /* Check the parameters */
	assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_CHANNEL(GPT_Channel));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        GPTx->GTDEB.V32 |= GPT_Channel << 8;
    }
    else
    {
        GPTx->GTDEB.V32 &= ~(GPT_Channel << 8);
    }
}

/**
  * @brief  Configure the GPTx's Counter start with the hardware(GTETRG) action.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_HardwareAction: specifie the Hardware(GTETRG) action to start the counter. 
  *          This parameter can be one of the following values:
  *            @arg GPT_HardwareAction_None
  *            @arg GPT_HardwareAction_Rising
  *            @arg GPT_HardwareAction_Falling
  *            @arg GPT_HardwareAction_RisingFalling
  * @retval None
  */
void GPT_StartCounterWithHardware(GPT_TypeDef* GPTx, uint16_t GPT_HardwareAction)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_HARDWARE_ACTION(GPT_HardwareAction));

    GPTx->GTHCR.V32 &= ~GPT_GTHCR_CSHW_Msk;
    GPTx->GTHCR.V32 |= GPT_HardwareAction;
}


/**
  * @brief  Configure the GPTx's Counter stop with the hardware(GTETRG) action.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_HardwareAction: specifie the Hardware(GTETRG) action to stop the counter. 
  *          This parameter can be one of the following values:
  *            @arg GPT_HardwareAction_None
  *            @arg GPT_HardwareAction_Rising
  *            @arg GPT_HardwareAction_Falling
  *            @arg GPT_HardwareAction_RisingFalling
  * @retval None
  */
void GPT_StopCounterWithHardware(GPT_TypeDef* GPTx, uint16_t GPT_HardwareAction)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_HARDWARE_ACTION(GPT_HardwareAction));

    GPTx->GTHCR.V32 &= ~GPT_GTHCR_CPHW_Msk;
    GPTx->GTHCR.V32 |= GPT_HardwareAction << 2;
}


/**
  * @brief  Configure the GPTx's Counter clear with the hardware(GTETRG) action.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_HardwareAction: specifie the Hardware(GTETRG) action to clear the counter. 
  *          This parameter can be one of the following values:
  *            @arg GPT_HardwareAction_None
  *            @arg GPT_HardwareAction_Rising
  *            @arg GPT_HardwareAction_Falling
  *            @arg GPT_HardwareAction_RisingFalling
  * @retval None
  */
void GPT_ClearCounterWithHardware(GPT_TypeDef* GPTx, uint16_t GPT_HardwareAction)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_HARDWARE_ACTION(GPT_HardwareAction));

    GPTx->GTHCR.V32 &= ~GPT_GTHCR_CCHW_Msk;
    GPTx->GTHCR.V32 |= GPT_HardwareAction << 4;
}

/**
  * @brief  Clear the GPTx's Counter by software.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval None
  */
void GPT_SoftwareClearCounter(GPT_TypeDef* GPTx)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    GPTx->GTHCR.V32 |= GPT_GTHCR_CCSW_Msk;
}

/**
  * @brief  Configure the GPTx's Counter clear sources.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_CounterClearSource: specifie the Counter Clear sources to be selected. 
  *          This parameter can be one of the following values:
  *            @arg GPT_CounterClearSource_None
  *            @arg GPT_CounterClearSource_CCRA
  *            @arg GPT_CounterClearSource_CCRB 
  *            @arg GPT_CounterClearSource_GPT0
  *            @arg GPT_CounterClearSource_GPT1
  *            @arg GPT_CounterClearSource_GPT2 
  *            @arg GPT_CounterClearSource_GPT3 
  * @retval None
  */
void GPT_SelectCounterClearSource(GPT_TypeDef* GPTx, uint16_t GPT_CounterClearSource)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_COUNTER_CLEAR_SOURCE(GPT_CounterClearSource));

    GPTx->GTCR.V32 &= ~GPT_GTCR_CCLR_Msk;
    if (GPT_CounterClearSource < GPT_CounterClearSource_GPT0)
    {
        GPTx->GTCR.V32 |= GPT_CounterClearSource;
    }
    else
    {
        GPTx->GTHCR.V32 &= ~GPT_GTHCR_SYNC_Msk;
        
        GPTx->GTCR.V32 |= GPT_GTCR_CCLR_Msk;
        GPTx->GTHCR.V32 |= (GPT_CounterClearSource & GPT_GTHCR_SYNC_Msk);
    }
}

/**
  * @brief  Configure the GPTx's Period Buffer function and set the Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_PeriodBufCmd: Period buffer mode. 
  *          This parameter can be one of the following values:
  *            @arg GPT_PeriodBuf_None
  *            @arg GPT_PeriodBuf_Single
  *            @arg GPT_PeriodBuf_Double 
  * @param  GPT_PeriodSingleBuf: specifie the Period Single Buffer Register value.
  *          This parameter can be 0 to 0xFFFF.
  * @param  GPT_PeriodDoubleBuf: specifie the Period Double Buffer Register value.
  *          This parameter can be 0 to 0xFFFF.  
  * @retval None
  */
void GPT_PeriodBufConfig(GPT_TypeDef* GPTx, uint16_t GPT_PeriodBufCmd, uint16_t GPT_PeriodSingleBuf, uint16_t GPT_PeriodDoubleBuf)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_PERIOD_BUF(GPT_PeriodBufCmd));

    if (GPT_PeriodBufCmd != GPT_PeriodBuf_None)
    {
        /* Enable the Period Buffer function */
        GPTx->GTBDR.V32 |= GPT_GTBDR_BDPR_Msk;
    }
    else
    {
        /* Disable the Period Buffer function */
        GPTx->GTBDR.V32 &= ~GPT_GTBDR_BDPR_Msk;
    }

    /* Clear the PR bits of GTBER register */
    GPTx->GTBER.V32 &= ~GPT_GTBER_PR_Msk;
    GPTx->GTBER.V32 |= GPT_PeriodBufCmd;

    GPTx->GTPBR = GPT_PeriodSingleBuf;
    GPTx->GTPDBR = GPT_PeriodDoubleBuf;
}

/**
  * @brief  Get the GPTx's Period Single Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval Period Single Buffer Register value.
  */
uint16_t GPT_GetPeriodSingleBuffer(GPT_TypeDef* GPTx)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    return GPTx->GTPBR;
}

/**
  * @brief  Get the GPTx's Period Double Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval Period Double Buffer Register value.
  */
uint16_t GPT_GetPeriodDoubleBuffer(GPT_TypeDef* GPTx)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    return GPTx->GTPDBR;
}


/**
  * @brief  Configure the GPTx compare capture buffer function.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_CCRABufCmd: specifie the GPTx Channel A buffer mode.
  *          This parameter can be one of the following values:
  *            @arg GPT_CCRABuf_None
  *            @arg GPT_CCRABuf_Single
  *            @arg GPT_CCRABuf_Double 
  * @param  GPT_CCRBBufCmd: specifie the GPTx Channel B buffer mode.
  *          This parameter can be one of the following values:
  *            @arg GPT_CCRBBuf_None
  *            @arg GPT_CCRBBuf_Single
  *            @arg GPT_CCRBBuf_Double
  * @retval None
  */
void GPT_CCBufConfig(GPT_TypeDef* GPTx,        uint16_t GPT_CCRABufCmd, uint16_t GPT_CCRBBufCmd)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_CC_CHANNEL_A_BUF(GPT_CCRABufCmd));
    assert_param(IS_GPT_CC_CHANNEL_B_BUF(GPT_CCRBBufCmd));

    if ((GPT_CCRABufCmd != GPT_CCRABuf_None) || (GPT_CCRBBufCmd != GPT_CCRBBuf_None))
    {
        /* Enable the CCR Buffer function */
        GPTx->GTBDR.V32 |= GPT_GTBDR_BDCCR_Msk;
    }
    else
    {
        /* Disable the CCR Buffer function */
        GPTx->GTBDR.V32 &= ~GPT_GTBDR_BDCCR_Msk;
    }

    /* Clear the CCRA/CCRB bits of GTBER register */
    GPTx->GTBER.V32 &= ~(GPT_GTBER_CCRA_Msk | GPT_GTBER_CCRB_Msk);
    GPTx->GTBER.V32 |= GPT_CCRABufCmd | GPT_CCRBBufCmd;
}

/**
  * @brief  Set the GPTx Compare Single Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  CompareASingleBuf: specifie the GPTx Channel A Compare single Buffer value.
  * @retval None
  */
void GPT_SetCompareASingleBuf(GPT_TypeDef* GPTx, uint16_t CompareASingleBuf)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    GPTx->GTCCRC = CompareASingleBuf;
}

/**
  * @brief  Set the GPTx Compare Double Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  CompareADoubleBuf: specifie the GPTx Channel A Compare Double Buffer value.
  * @retval None
  */
void GPT_SetCompareADoubleBuf(GPT_TypeDef* GPTx, uint16_t CompareADoubleBuf)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    GPTx->GTCCRD = CompareADoubleBuf;
}

/**
  * @brief  Set the GPTx Compare Single Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  CompareBSingleBuf: specifie the GPTx Channel B Compare single Buffer value.
  * @retval None
  */
void GPT_SetCompareBSingleBuf(GPT_TypeDef* GPTx, uint16_t CompareBSingleBuf)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    GPTx->GTCCRE = CompareBSingleBuf;
}

/**
  * @brief  Set the GPTx Compare Double Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  CompareBDoubleBuf: specifie the GPTx Channel B Compare Double Buffer value.
  * @retval None
  */
void GPT_SetCompareBDoubleBuf(GPT_TypeDef* GPTx, uint16_t CompareBDoubleBuf)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    GPTx->GTCCRF = CompareBDoubleBuf;
}

/**
  * @brief  Get the GPTx Capture A value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval The Capture A Register value.
  */
uint16_t GPT_GetCaptureA(GPT_TypeDef* GPTx)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    return GPTx->GTCCRA;
}

/**
  * @brief  Get the GPTx Capture A Single Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval The Capture A Single Buffer register value.
  */
uint16_t GPT_GetCaptureASingleBuf(GPT_TypeDef* GPTx)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    return GPTx->GTCCRC;
}

/**
  * @brief  Get the GPTx Capture A Double Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval The Capture A Double Buffer register value.
  */
uint16_t GPT_GetCaptureADoubleBuf(GPT_TypeDef* GPTx)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    return GPTx->GTCCRD;
}

/**
  * @brief  Get the GPTx Capture B value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval The Capture B Register Value.
  */
uint16_t GPT_GetCompareB(GPT_TypeDef* GPTx)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    return GPTx->GTCCRB;
}

/**
  * @brief  Get the GPTx Capture B Single Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval The Capture B Single Buffer Register Value.
  */
uint16_t GPT_GetCompareBSingleBuf(GPT_TypeDef* GPTx)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    return GPTx->GTCCRE;
}

/**
  * @brief  Get the GPTx Capture B Double Buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval The Capture B Double Buffer Register Value.
  */
uint16_t GPT_GetCompareBDoubleBuf(GPT_TypeDef* GPTx)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    return GPTx->GTCCRF;
}

/**
  * @brief  Start the GPTx forcible buffer operation.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval None
  */
void GPT_ForceBufferOperation(GPT_TypeDef* GPTx)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    GPTx->GTBER.V32 |= GPT_GTBER_CCRSWT_Msk;
}

/**
  * @brief  Enable or disable the GPTx trigger ADC function.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_TriggerSource: specifie the ADC trigger sources.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_TriggerADCSourceA_Rising
  *            @arg GPT_TriggerADCSourceA_Falling
  *            @arg GPT_TriggerADCSourceB_Rising
  *            @arg GPT_TriggerADCSourceB_Falling
  * @param  NewState: new state of the ADC trigger sources.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE  
  * @retval None
  */
void GPT_TriggerADCConfig(GPT_TypeDef* GPTx, uint16_t GPT_TriggerSource, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_TRIGGER_ADC_SOURCE(GPT_TriggerSource));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable trigger ADC sources */
        GPTx->GTINTAD.V32 |= GPT_TriggerSource;
    }
    else
    {
        /* Disable trigger ADC sources */
        GPTx->GTINTAD.V32 &= ~GPT_TriggerSource;
    }
}

/**
  * @brief  
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_TriggerABufCmd:
  *          This parameter can be one of the following values:
  *            @arg GPT_TriggerADCBuf_None
  *            @arg GPT_TriggerADCBuf_Single
  *            @arg GPT_TriggerADCBuf_Double  
  * @param  GPT_TriggerATiming:
  *          This parameter can be one of the following values:
  *            @arg GPT_TriggerADCTiming_None
  *            @arg GPT_TriggerADCTiming_Peak
  *            @arg GPT_TriggerADCTiming_Trough
  *            @arg GPT_TriggerADCTiming_Both  
  * @retval None
  */
void GPT_TriggerADCSourceABufConfig(GPT_TypeDef* GPTx, uint16_t GPT_TriggerABufCmd, uint16_t GPT_TriggerATiming)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx)); 
    assert_param(IS_GPT_TRIGGER_ADC_BUF(GPT_TriggerABufCmd));
    assert_param(IS_GPT_TRIGGER_ADC_TIMING(GPT_TriggerATiming));

    if (GPT_TriggerABufCmd == GPT_TriggerADCBuf_None)
    {
        GPTx->GTBDR.V32 &= ~GPT_GTBDR_BDADTR_Msk;
    }
    else
    {
        GPTx->GTBDR.V32 |= GPT_GTBDR_BDADTR_Msk;
    }

    GPTx->GTBER.V32 &= ~(GPT_GTBER_ADTDA_Msk | GPT_GTBER_ADTTA_Msk);
    GPTx->GTBER.V32 |= GPT_TriggerABufCmd | GPT_TriggerATiming;
}

/**
  * @brief  Configure the GPTx trigger ADC buffer mode and trigger timing.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_TriggerABufCmd: specifie the ADC trigger buffer mode.
  *          This parameter can be one of the following values:
  *            @arg GPT_TriggerADCBuf_None
  *            @arg GPT_TriggerADCBuf_Single
  *            @arg GPT_TriggerADCBuf_Double  
  * @param  GPT_TriggerATiming: specifie the ADC trigger timing.
  *          This parameter can be one of the following values:
  *            @arg GPT_TriggerADCTiming_None
  *            @arg GPT_TriggerADCTiming_Peak
  *            @arg GPT_TriggerADCTiming_Trough
  *            @arg GPT_TriggerADCTiming_Both  
  * @retval None
  */
void GPT_TriggerADCSourceBBufConfig(GPT_TypeDef* GPTx, uint16_t GPT_TriggerBBufCmd, uint16_t GPT_TriggerBTiming)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_TRIGGER_ADC_BUF(GPT_TriggerBBufCmd));
    assert_param(IS_GPT_TRIGGER_ADC_TIMING(GPT_TriggerBTiming));

    if (GPT_TriggerBBufCmd == GPT_TriggerADCBuf_None)
    {
        GPTx->GTBDR.V32 &= ~GPT_GTBDR_BDADTR_Msk;
    }
    else
    {
        GPTx->GTBDR.V32 |= GPT_GTBDR_BDADTR_Msk;
    }

    GPTx->GTBER.V32 &= ~(GPT_GTBER_ADTDB_Msk | GPT_GTBER_ADTTB_Msk);
    GPTx->GTBER.V32 |= ((GPT_TriggerBBufCmd | GPT_TriggerBTiming) << 4);    
}


/**
  * @brief  Set the GPTx's trigger ADC register value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_Channel: specifie the GPT Channel to be set.
  *          This parameter can be one of the following values:
  *            @arg GPT_Channel_A
  *            @arg GPT_Channel_B
  * @param  Value: specifie the GPTx's trigger ADC register new value.
  *          This parameter can be 0 to 0xFFFF. 
  * @retval None
  */
void GPT_SetTriggerADCValue(GPT_TypeDef* GPTx, uint16_t GPT_Channel, uint16_t Value)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_GET_ONE_CHANNEL(GPT_Channel));

    if ((GPT_Channel & GPT_Channel_A) == GPT_Channel_A)
    {
        GPTx->GTADTRA = Value;
    }
    else
    {
        GPTx->GTADTRB = Value;
    }
}

/**
  * @brief  Set the GPTx's trigger ADC single buffer register value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_Channel: specifie the GPT Channel to be set.
  *          This parameter can be one of the following values:
  *            @arg GPT_Channel_A
  *            @arg GPT_Channel_B
  * @param  SingleBufValue: specifie the GPTx's trigger ADC single buffer register new value.
  *          This parameter can be 0 to 0xFFFF. 
  * @retval None
  */
void GPT_SetTriggerADCSingleBufValue(GPT_TypeDef* GPTx, uint16_t GPT_Channel, uint16_t SingleBufValue)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_GET_ONE_CHANNEL(GPT_Channel));
        
    if ((GPT_Channel & GPT_Channel_A) == GPT_Channel_A)
    {
        GPTx->GTADTBRA = SingleBufValue;
    }
    else
    {
        GPTx->GTADTBRB = SingleBufValue;
    }
}

/**
  * @brief  Set the GPTx's trigger ADC double buffer register value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_Channel: specifie the GPT Channel to be set.
  *          This parameter can be one of the following values:
  *            @arg GPT_Channel_A
  *            @arg GPT_Channel_B
  * @param  DoubleBufValue: specifie the GPTx's trigger ADC double buffer register new value.
  *          This parameter can be 0 to 0xFFFF. 
  * @retval None
  */
void GPT_SetTriggerADCDoubleBufValue(GPT_TypeDef* GPTx, uint16_t GPT_Channel, uint16_t DoubleBufValue)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_GET_ONE_CHANNEL(GPT_Channel));
    
    if ((GPT_Channel & GPT_Channel_A) == GPT_Channel_A)
    {
        GPTx->GTADTDBRA = DoubleBufValue;
    }
    else
    {
        GPTx->GTADTDBRB = DoubleBufValue;
    }
}

/**
  * @brief  Configure the GPTx interrupt reduction function.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_ReductionMode: specifie the interrupt reduction mode.
  *          This parameter can be one of the following values:
  *            @arg GPT_IntReductionMode_None
  *            @arg GPT_IntReductionMode_Peak
  *            @arg GPT_IntReductionMode_Trough  
  *            @arg GPT_IntReductionMode_Both
  * @param  GPT_ReductionTimes: specifie the interrupt reduction times.
  *          This parameter can be one of the following values:
  *            @arg GPT_IntReductionTimes_0
  *            @arg GPT_IntReductionTimes_1
  *            @arg GPT_IntReductionTimes_2
  *            @arg GPT_IntReductionTimes_3
  *            @arg GPT_IntReductionTimes_4
  *            @arg GPT_IntReductionTimes_5
  *            @arg GPT_IntReductionTimes_6
  *            @arg GPT_IntReductionTimes_7 
  * @retval None
  */
void GPT_IntReductionConfig(GPT_TypeDef* GPTx, uint16_t GPT_ReductionMode, uint16_t GPT_ReductionTimes)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_INT_REDUCTION_MODE(GPT_ReductionMode));
    assert_param(IS_GPT_INT_REDUCTION_TIMES(GPT_ReductionTimes));

    GPTx->GTITC.V32 &= ~(GPT_GTITC_IVTC_Msk | GPT_GTITC_IVTT_Msk);
    GPTx->GTITC.V32 |= GPT_ReductionMode | GPT_ReductionTimes;
}

/**
  * @brief  Enable or disable the GPTx interrupt reduction association function.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_AssociateFunc: specifie which function can be associated to interrupt reduction.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_AssociateInt_CCRA
  *            @arg GPT_AssociateInt_CCRB
  *            @arg GPT_AssociateInt_TriggerADC_A  
  *            @arg GPT_AssociateInt_TriggerADC_B
  * @param  NewState: new state of the association function.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE 
  * @retval None
  */
void GPT_AssociateIntReductionConfig(GPT_TypeDef* GPTx, uint16_t GPT_AssociateFunc, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_ASSOCIATE_INT_SOURCE(GPT_AssociateFunc));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the associate interrupt reduction function */
        GPTx->GTITC.V32 |= GPT_AssociateFunc;
    }
    else
    {
        /* Disable the associate interrupt reduction function */
        GPTx->GTITC.V32 &= ~GPT_AssociateFunc;    
    }
}

/**
  * @brief  Get the GPTx interrupt reduction times.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @retval Interrupt reduction times.
  */
uint8_t GPT_GetIntReductionTimes(GPT_TypeDef* GPTx)
{
    uint8_t temp = 0;
    
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    temp = (GPTx->GTST.V32 & GPT_GTST_ITCNT_Msk) >> 8;

    return temp;
}

/**
  * @brief  Enable or disable the GPTx's interrupts.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_INT: specifie the GPTx interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_INT_CCRA
  *            @arg GPT_INT_CCRB
  *            @arg GPT_INT_OV
  *            @arg GPT_INT_UD
  *            @arg GPT_INT_OVUD
  *            @arg GPT_INT_DTE
  *            @arg GPT_INT_OS  
  * @param  NewState: new state of the GPT interrupt sources.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void GPT_INTConfig(GPT_TypeDef* GPTx, uint16_t GPT_INT, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_INT(GPT_INT));    
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        GPTx->GTINTAD.V32 |= GPT_INT;
    }
    else
    {
        GPTx->GTINTAD.V32 &= ~GPT_INT;
    }
}

/**
  * @brief  Enable or disable the GPTx's DMA requests.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_DMASource: specifie the DMA Request sources.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_DMA_CCRA
  *            @arg GPT_DMA_CCRB
  *            @arg GPT_DMA_OV
  *            @arg GPT_DMA_UD
  *            @arg GPT_DMA_OVUD  
  * @param  NewState: new state of the GPT's DMA request sources.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE  
  * @retval None
  */
void GPT_DMAConfig(GPT_TypeDef* GPTx, uint16_t GPT_DMASource, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_DMA_SOURCE(GPT_DMASource));    
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        GPTx->GTDMA.V32 |= GPT_DMASource;
    }
    else
    {
        GPTx->GTDMA.V32 &= ~GPT_DMASource;
    }
}

/**
  * @brief  Check whether the specified GPT flag is set or not
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_Flag: specifie the flag to check.
  *          This parameter can be one of the following values:
  *            @arg GPT_FLAG_CCRA
  *            @arg GPT_FLAG_CCRB
  *            @arg GPT_FLAG_OV
  *            @arg GPT_FLAG_UD
  *            @arg GPT_FLAG_DTE
  *            @arg GPT_FLAG_OS  
  * @retval None
  */
FlagStatus GPT_GetFlagStatus(GPT_TypeDef* GPTx, uint16_t GPT_Flag)
{
    FlagStatus bitStatus;
    
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_GET_ONE_FLAG(GPT_Flag));

    if ((GPTx->GTST.V32 & GPT_Flag) != (uint32_t)RESET)
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
  * @brief  Clear the GPTx's pending flags.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_Flag: specifie the flag bit to claer.
  *          This parameter can be any combination of the following values:
  *            @arg GPT_FLAG_CCRA
  *            @arg GPT_FLAG_CCRB
  *            @arg GPT_FLAG_OV
  *            @arg GPT_FLAG_UD
  *            @arg GPT_FLAG_DTE
  *            @arg GPT_FLAG_OS
  * @retval None
  */
void GPT_ClearFlag(GPT_TypeDef* GPTx, uint16_t GPT_Flag)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_FLAG(GPT_Flag));

    GPTx->GTST.V32 = GPT_Flag << 16;
}

/**
  * @brief  Initialize the GPTx Dead Time function according to 
  *         the specified parameters in the GPT_DeadTimeInitStruct.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_DeadTimeInitStruct: pointer to a @ref GPT_DeadTimeInitTypeDef structure
  *         that contains the configuration information for the Dead Time function.
  * @retval None
  */
void GPT_DeadTimeInit(GPT_TypeDef* GPTx, GPT_DeadTimeInitTypeDef * GPT_DeadTimeInitStruct)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    GPTx->GTDTCR.V32 &= ~(GPT_GTDTCR_TDFER_Msk | GPT_GTDTCR_TDBUE_Msk | GPT_GTDTCR_TDBDE_Msk);

    if ((GPT_DeadTimeInitStruct->GPT_GTDVDBufCmd \
        | GPT_DeadTimeInitStruct->GPT_GTDVUBufCmd) != (uint16_t)RESET)
    {
        GPTx->GTBDR.V32 |= GPT_GTBDR_BDDV_Msk;
    }
    else
    {
        GPTx->GTBDR.V32 &= ~GPT_GTBDR_BDDV_Msk;
    }
    
    GPTx->GTDTCR.V32 |= GPT_DeadTimeInitStruct->GPT_AutoSetDVDCmd \
                        | GPT_DeadTimeInitStruct->GPT_GTDVDBufCmd \
                        | GPT_DeadTimeInitStruct->GPT_GTDVUBufCmd;
    GPTx->GTDVU = GPT_DeadTimeInitStruct->GPT_GTDVU;
    GPTx->GTDVD = GPT_DeadTimeInitStruct->GPT_GTDVD;
    GPTx->GTDBU = GPT_DeadTimeInitStruct->GPT_GTDBU;
    GPTx->GTDBD = GPT_DeadTimeInitStruct->GPT_GTDBD;
}

/**
  * @brief  Set the GPTx's dead time register GTDVU and GTDVD value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GTDVU: specifie dead time DVU register value.
  *          This parameter can be 0 to 0xFFFF.
  * @param  GTDVD: specifie dead time DVD register value.
  *          This parameter can be 0 to 0xFFFF.  
  * @retval None
  */
void GPT_DeadTimeValue(GPT_TypeDef* GPTx, uint16_t GTDVU, uint16_t GTDVD)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    GPTx->GTDVU = GTDVU;
    GPTx->GTDVD = GTDVD;
}

/**
  * @brief  Enable or disable the GPTx dead time buffer function.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_GTDVUBufCmd: specifie the dead time GTDVU buffer function.
  *          This parameter can be one of the following values:
  *            @arg GPT_GTDVUBuf_Enable
  *            @arg GPT_GTDVUBuf_Disable
  * @param  GPT_GTDVDBufCmd: specifie the dead time GTDVD buffer function.
  *          This parameter can be one of the following values:
  *            @arg GPT_GTDVDBuf_Enable
  *            @arg GPT_GTDVDBuf_Disable 
  * @retval None
  */
void GPT_DeadTimeBufferConfig(GPT_TypeDef* GPTx, uint16_t GPT_GTDVUBufCmd, uint16_t GPT_GTDVDBufCmd)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_GTDVU_BUFFER_CMD(GPT_GTDVUBufCmd));
    assert_param(IS_GPT_GTDVD_BUFFER_CMD(GPT_GTDVDBufCmd));

    GPTx->GTDTCR.V32 &= ~(GPT_GTDTCR_TDBUE_Msk | GPT_GTDTCR_TDBDE_Msk);
    GPTx->GTDTCR.V32 |= GPT_GTDVUBufCmd | GPT_GTDVDBufCmd;

    if ((GPTx->GTDTCR.V32 & (GPT_GTDTCR_TDBDE_Msk | GPT_GTDTCR_TDBUE_Msk)) == (uint32_t)RESET)
    {
        GPTx->GTBDR.V32 &= ~GPT_GTBDR_BDDV_Msk;
    }
    else
    {
        GPTx->GTBDR.V32 |= GPT_GTBDR_BDDV_Msk;
    }
}

/**
  * @brief  Set the GPTx dead time buffer value.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GTDBU: specifie the GPT dead time buffer GTDBU register value.
  *          This parameter can be 0 to 0xFFFF.
  * @param  GTDBD: specifie the GPT dead time buffer GTDBD register value.
  *          This parameter can be 0 to 0xFFFF.  
  * @retval None
  */
void GPT_DeadTimeBufferValue(GPT_TypeDef* GPTx, uint16_t GTDBU, uint16_t GTDBD)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));

    GPTx->GTDBU = GTDBU;
    GPTx->GTDBD = GTDBD;
}


/**
  * @brief  Enable or disable the GPTx dead time auto set DVD register.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  NewState: new state of the dead time auto set DVD register.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void GPT_DeadTimeAutoSetRegConfig(GPT_TypeDef* GPTx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_FUNCTION_STATE(NewState));    

    if (NewState != DISABLE)

    {
        GPTx->GTDTCR.V32 |= GPT_GTDTCR_TDFER_Msk;
    }
    else
    {
        GPTx->GTDTCR.V32 &= ~GPT_GTDTCR_TDFER_Msk;
    }
}

/**
  * @brief  Enable or disable the GPTx's Dead Time function.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  NewState: new state of the GPTx Dead Time function.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void GPT_DeadTimeConfig(GPT_TypeDef* GPTx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        GPTx->GTDTCR.V32 |= GPT_GTDTCR_TDE_Msk;
    }
    else
    {
        GPTx->GTDTCR.V32 &= ~GPT_GTDTCR_TDE_Msk;
    }
}

/**
  * @brief  Configure the Channel Level of output short circuit protection.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  GPT_CCRALevel: specifie the Channel A level.
  *          This parameter can be one of the following values:
  *            @arg GPT_CCRA_OS_Low
  *            @arg GPT_CCRA_OS_High
  * @param  GPT_CCRBLevel: specifie the Channel B level.
  *          This parameter can be one of the following values:
  *            @arg GPT_CCRB_OS_Low
  *            @arg GPT_CCRB_OS_High  
  * @retval None
  */
void GPT_OutputShortCircuitConfig(GPT_TypeDef* GPTx, uint8_t GPT_CCRALevel, uint8_t GPT_CCRBLevel)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_GPT_CCRA_OS_LEVEL(GPT_CCRALevel));
    assert_param(IS_GPT_CCRB_OS_LEVEL(GPT_CCRBLevel));

    GPTx->GTOSCR.V32 &= ~(GPT_GTOSCR_OLSGA_Msk | GPT_GTOSCR_OLSGB_Msk);
    GPTx->GTOSCR.V32 |= GPT_CCRALevel | GPT_CCRBLevel;
}

/**
  * @brief  On or off the GPTx output short circuit protection.
  * @param  GPTx: where x can be 0,1,2 or 3 to select the GPT peripheral.
  * @param  OnOffState: state of the output short circuit protection.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void GPT_OutputShortCircuitOnOff(GPT_TypeDef* GPTx, CmdState OnOffState)
{
    /* Check the parameters */
    assert_param(IS_GPT_ALL_PERIPH(GPTx));
    assert_param(IS_CMD_STATE(OnOffState));

    if (OnOffState != OFF)
    {
        GPTx->GTOSCR.V32 |= GPT_GTOSCR_OLSEN_Msk;
    }
    else
    {
        GPTx->GTOSCR.V32 &= ~GPT_GTOSCR_OLSEN_Msk;
    }
}



/** end of GPT_Group1
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


