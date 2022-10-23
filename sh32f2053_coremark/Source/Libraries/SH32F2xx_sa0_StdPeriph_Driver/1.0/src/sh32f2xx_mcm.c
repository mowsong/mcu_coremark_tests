/**
  ******************************************************************************
  * @file    sh32f2xx_mcm.c
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
#include "sh32f2xx_mcm.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup MCM_MODULE MCM 
  * @brief MCM driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup MCM_Private_Functions
  * @{
  */ 

/** @defgroup MCM_Group1 Initialization and Configuration
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
  * @brief  Reset the MCMx peripheral registers to their default reset values.
  * @param  MCMx: Where x can be 1 or 2 to select the MCM peripheral.
  * @retval None
  */
void MCM_Reset(MCM_TypeDef* MCMx)
{
    if (MCMx == MCM1)
    {
        RCC_APB2PeriphReset(RCC_APB2_MCM1);
    }
    else
    {
        RCC_APB2PeriphReset(RCC_APB2_MCM2);
    }
}


/**
  * @brief  Enables or disables the MCM's registers protection.
  * @param  MCMx: where x can be 1 or 2 to select the MCMx peripheral.
  * @param  NewState: new state of the MCM's registers protection.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void MCM_RegisterWriteLock(MCM_TypeDef* MCMx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
	assert_param(IS_FUNCTION_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable MCM registers protection */
        MCMx->PWMRLDEN = 0xAA;
    }
    else
    {
        /* Disable MCM registers protection */
        MCMx->PWMRLDEN = 0x55;
    }
}

/**
  * @brief  On or off the specified MCM peripheral.
  * @param  MCMx: where x can be 1 or 2 to select the MCMx peripheral.
  * @param  OnOffState: state of the MCMx peripheral.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void MCM_OnOff(MCM_TypeDef* MCMx, CmdState OnOffState)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
	assert_param(IS_CMD_STATE(OnOffState));

    MCM_REG_UNLOCK(MCMx);

	if (OnOffState != OFF)
	{
		MCMx->PWMOE |= MCM_PWMOE_PWMOE_Msk;
	}
	else
	{
		MCMx->PWMOE &= ~MCM_PWMOE_PWMOE_Msk;
	} 

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Fills each MCM_TimeBaseInitStruct member with its default value.
  * @param  MCM_TimeBaseInitStruct : pointer to a @ref MCM_TimeBaseInitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
void MCM_TimeBaseStructInit(MCM_TimeBaseInitTypeDef* MCM_TimeBaseInitStruct)
{
    MCM_TimeBaseInitStruct->CounterMode = MCM_CounterMode_Edge;
    MCM_TimeBaseInitStruct->Prescaler = 0x00;
    MCM_TimeBaseInitStruct->Period = 0x00;
}

/**
  * @brief  Initializes the MCMx Time Base Unit peripheral according to 
  *         the specified parameters in the MCM_TimeBaseInitStruct.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_TimeBaseInitStruct: pointer to a @ref MCM_TimeBaseInitTypeDef structure
  *         that contains the configuration information for the specified MCM peripheral.
  * @retval None
  */
void MCM_TimeBaseInit(MCM_TypeDef* MCMx, MCM_TimeBaseInitTypeDef* MCM_TimeBaseInitStruct)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
	assert_param(IS_MCM_COUNTER_MODE(MCM_TimeBaseInitStruct->CounterMode));

    MCM_REG_UNLOCK(MCMx);
	/* Reset the PTMOD bits */
	MCMx->PWMCON1.V32 &= ~MCM_PWMCON1_PTMOD_Msk; 
	MCMx->PWMCON1.V32 |= MCM_TimeBaseInitStruct->CounterMode;

    /* Set MCM Prescaler and Period register value */
	MCMx->PWMPSQ = MCM_TimeBaseInitStruct->Prescaler;
	MCMx->PWMP = MCM_TimeBaseInitStruct->Period;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Fills each MCM_PWMInitStruct member with its default value.
  * @param  MCM_PWMInitStruct : pointer to a @ref MCM_PWMInitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
void MCM_PWMStructInit(MCM_PWMInitTypeDef* MCM_PWMInitStruct)
{
    MCM_PWMInitStruct->PWMMode = MCM_PWMMode_Complementary;
    MCM_PWMInitStruct->PWMSymmetry = MCM_PWM_Symmetry;
    MCM_PWMInitStruct->DutyArea = MCM_DutyArea_1;
    MCM_PWMInitStruct->DutyPolarity = MCM_DutyPolarity_High;
    MCM_PWMInitStruct->DutyValue = 0x00;
}


/**
  * @brief  Initialize the MCMx PWM0 function according to 
  *         the specified parameters in the MCM_PWMInitStruct.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMInitStruct: pointer to a @ref MCM_PWMInitTypeDef structure
  *         that contains the configuration information for the PWM0 function.
  * @retval None
  */
void MCM_PWM0Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct)
{
	uint16_t temp = 0;

    assert_param(IS_MCM_PWM_MODE(MCM_PWMInitStruct->PWMMode));
    assert_param(IS_MCM_DUTY_AREA(MCM_PWMInitStruct->DutyArea));
    assert_param(IS_MCM_DUTY_POLARITY(MCM_PWMInitStruct->DutyPolarity));

    MCM_REG_UNLOCK(MCMx);

    temp = MCMx->PWMCON1.V32;
    
	/* Reset PUTMOD/PDCON0/PWM0S bits */
	temp &= ~(MCM_PWMCON1_POUTMOD_Msk | MCM_PWMCON1_PDCON0_Msk | MCM_PWMCON1_PWM0S_Msk);

	temp |= MCM_PWMInitStruct->PWMMode;
	temp |= MCM_PWMInitStruct->DutyArea;
	temp |= (MCM_PWMInitStruct->DutyPolarity & (uint16_t)0x0001);

    if (MCM_PWMInitStruct->PWMMode == MCM_PWMMode_Complementary)
    {
        assert_param(IS_MCM_PWM_SYMMETRY(MCM_PWMInitStruct->PWMSymmetry));
        
        /* Reset PWMSYM bit */
        temp &= ~MCM_PWMCON1_PWMSYM_Msk;
        temp |= MCM_PWMInitStruct->PWMSymmetry;
    }

	MCMx->PWMCON1.V32 = temp;
    
    temp = MCMx->PWMCON2.V32;
    temp &= ~(MCM_DutyActivePoint_Now | MCM_DutyActivePoint_Zero | MCM_DutyActivePoint_Period);
	temp |=  MCM_PWMInitStruct->DutyActivePoint;
    MCMx->PWMCON2.V32 = temp;

	MCMx->PWM0D = MCM_PWMInitStruct->DutyValue;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Initialize the MCMx PWM01 function according to 
  *         the specified parameters in the MCM_PWMInitStruct.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMInitStruct: pointer to a @ref MCM_PWMInitTypeDef structure
  *         that contains the configuration information for the PWM01 function.
  * @retval None
  */
void MCM_PWM01Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct)
{
	uint16_t temp = 0;

    assert_param(IS_MCM_PWM_MODE(MCM_PWMInitStruct->PWMMode));
    assert_param(IS_MCM_DUTY_AREA(MCM_PWMInitStruct->DutyArea));
    assert_param(IS_MCM_DUTY_POLARITY(MCM_PWMInitStruct->DutyPolarity));

    MCM_REG_UNLOCK(MCMx);

    temp = MCMx->PWMCON1.V32;
    
	/* Reset PUTMOD/PDCON0/PWM01S bits */
	temp &= ~(MCM_PWMCON1_POUTMOD_Msk | MCM_PWMCON1_PDCON0_Msk | MCM_PWMCON1_PWM01S_Msk);

	temp |= MCM_PWMInitStruct->PWMMode;
	temp |= MCM_PWMInitStruct->DutyArea;
    temp |= ((MCM_PWMInitStruct->DutyPolarity >> 1) & (uint16_t)0x0001) << 3;
    
    if (MCM_PWMInitStruct->PWMMode == MCM_PWMMode_Complementary)
    {
        assert_param(IS_MCM_PWM_SYMMETRY(MCM_PWMInitStruct->PWMSymmetry));
        
        /* Reset PWMSYM bit */
        temp &= ~MCM_PWMCON1_PWMSYM_Msk;
        temp |= MCM_PWMInitStruct->PWMSymmetry;
    }
    
	MCMx->PWMCON1.V32 = temp;

    temp = MCMx->PWMCON2.V32;
    temp &= ~(MCM_DutyActivePoint_Now | MCM_DutyActivePoint_Zero | MCM_DutyActivePoint_Period);
	temp |=  MCM_PWMInitStruct->DutyActivePoint;
    MCMx->PWMCON2.V32 = temp;
    
    MCMx->PWM01D = MCM_PWMInitStruct->DutyValue;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Initialize the MCMx PWM1 function according to 
  *         the specified parameters in the MCM_PWMInitStruct.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMInitStruct: pointer to a @ref MCM_PWMInitTypeDef structure
  *         that contains the configuration information for the PWM1 function.
  * @retval None
  */
void MCM_PWM1Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct)
{
	uint16_t temp = 0;

    assert_param(IS_MCM_PWM_MODE(MCM_PWMInitStruct->PWMMode));
    assert_param(IS_MCM_DUTY_AREA(MCM_PWMInitStruct->DutyArea));
    assert_param(IS_MCM_DUTY_POLARITY(MCM_PWMInitStruct->DutyPolarity));

    MCM_REG_UNLOCK(MCMx);
    
    temp = MCMx->PWMCON1.V32;
    
	/* Reset PUTMOD/PDCON2/PWM0S bits */
	temp &= ~(MCM_PWMCON1_POUTMOD_Msk | MCM_PWMCON1_PDCON1_Msk | MCM_PWMCON1_PWM1S_Msk);

	temp |= MCM_PWMInitStruct->PWMMode;
	temp |= MCM_PWMInitStruct->DutyArea << 1;
	temp |= ((MCM_PWMInitStruct->DutyPolarity  & (uint16_t)0x0001) << 1);

    if (MCM_PWMInitStruct->PWMMode == MCM_PWMMode_Complementary)
    {
        assert_param(IS_MCM_PWM_SYMMETRY(MCM_PWMInitStruct->PWMSymmetry));
        
        /* Reset PWMSYM bit */
        temp &= ~MCM_PWMCON1_PWMSYM_Msk;
        temp |= MCM_PWMInitStruct->PWMSymmetry;
    }

	MCMx->PWMCON1.V32 = temp;
    
    temp = MCMx->PWMCON2.V32;
    temp &= ~(MCM_DutyActivePoint_Now | MCM_DutyActivePoint_Zero | MCM_DutyActivePoint_Period);
	temp |=  MCM_PWMInitStruct->DutyActivePoint;
    MCMx->PWMCON2.V32 = temp;

	MCMx->PWM1D = MCM_PWMInitStruct->DutyValue;

    MCM_REG_LOCK(MCMx);

}

/**
  * @brief  Initialize the MCMx PWM11 function according to 
  *         the specified parameters in the MCM_PWMInitStruct.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMInitStruct: pointer to a @ref MCM_PWMInitTypeDef structure
  *         that contains the configuration information for the PWM11 function.
  * @retval None
  */
void MCM_PWM11Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct)
{
	uint16_t temp = 0;

    assert_param(IS_MCM_PWM_MODE(MCM_PWMInitStruct->PWMMode));
    assert_param(IS_MCM_DUTY_AREA(MCM_PWMInitStruct->DutyArea));
    assert_param(IS_MCM_DUTY_POLARITY(MCM_PWMInitStruct->DutyPolarity));

    MCM_REG_UNLOCK(MCMx);

    temp = MCMx->PWMCON1.V32;
    
	/* Reset PUTMOD/PDCON2/PWM0S bits */
	temp &= ~(MCM_PWMCON1_POUTMOD_Msk | MCM_PWMCON1_PDCON1_Msk | MCM_PWMCON1_PWM11S_Msk);

	temp |= MCM_PWMInitStruct->PWMMode;
	temp |= MCM_PWMInitStruct->DutyArea << 1;
    temp |= (((MCM_PWMInitStruct->DutyPolarity >> 1) & (uint16_t)0x0001) << 4);

    if (MCM_PWMInitStruct->PWMMode == MCM_PWMMode_Complementary)
    {
        assert_param(IS_MCM_PWM_SYMMETRY(MCM_PWMInitStruct->PWMSymmetry));
        
        /* Reset PWMSYM bit */
        temp &= ~MCM_PWMCON1_PWMSYM_Msk;
        temp |= MCM_PWMInitStruct->PWMSymmetry;
    }

	MCMx->PWMCON1.V32 = temp;
    
    temp = MCMx->PWMCON2.V32;
    temp &= ~(MCM_DutyActivePoint_Now | MCM_DutyActivePoint_Zero | MCM_DutyActivePoint_Period);
	temp |=  MCM_PWMInitStruct->DutyActivePoint;
    MCMx->PWMCON2.V32 = temp;

	MCMx->PWM11D = MCM_PWMInitStruct->DutyValue;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Initialize the MCMx PWM2 function according to 
  *         the specified parameters in the MCM_PWMInitStruct.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMInitStruct: pointer to a @ref MCM_PWMInitTypeDef structure
  *         that contains the configuration information for the PWM2 function.
  * @retval None
  */
void MCM_PWM2Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct)
{
	uint16_t temp = 0;

    assert_param(IS_MCM_PWM_MODE(MCM_PWMInitStruct->PWMMode));
    assert_param(IS_MCM_DUTY_AREA(MCM_PWMInitStruct->DutyArea));
    assert_param(IS_MCM_DUTY_POLARITY(MCM_PWMInitStruct->DutyPolarity));

    MCM_REG_UNLOCK(MCMx);

    temp = MCMx->PWMCON1.V32;
    
	/* Reset PUTMOD/PDCON2/PWM0S bits */
	temp &= ~(MCM_PWMCON1_POUTMOD_Msk | MCM_PWMCON1_PDCON2_Msk | MCM_PWMCON1_PWM2S_Msk);

	temp |= MCM_PWMInitStruct->PWMMode;
	temp |= MCM_PWMInitStruct->DutyArea << 2;
	temp |= ((MCM_PWMInitStruct->DutyPolarity & (uint16_t)0x0001) << 2);

    if (MCM_PWMInitStruct->PWMMode == MCM_PWMMode_Complementary)
    {
        assert_param(IS_MCM_PWM_SYMMETRY(MCM_PWMInitStruct->PWMSymmetry));
        
        /* Reset PWMSYM bit */
        temp &= ~MCM_PWMCON1_PWMSYM_Msk;
        temp |= MCM_PWMInitStruct->PWMSymmetry;
    }

	MCMx->PWMCON1.V32 = temp;
    
    temp = MCMx->PWMCON2.V32;
    temp &= ~(MCM_DutyActivePoint_Now | MCM_DutyActivePoint_Zero | MCM_DutyActivePoint_Period);
	temp |=  MCM_PWMInitStruct->DutyActivePoint;
    MCMx->PWMCON2.V32 = temp;

	MCMx->PWM2D = MCM_PWMInitStruct->DutyValue;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Initialize the MCMx PWM21 function according to 
  *         the specified parameters in the MCM_PWMInitStruct.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMInitStruct: pointer to a @ref MCM_PWMInitTypeDef structure
  *         that contains the configuration information for the PWM21 function.
  * @retval None
  */
void MCM_PWM21Init(MCM_TypeDef* MCMx, MCM_PWMInitTypeDef* MCM_PWMInitStruct)
{
	uint16_t temp = 0;

    assert_param(IS_MCM_PWM_MODE(MCM_PWMInitStruct->PWMMode));
    assert_param(IS_MCM_DUTY_AREA(MCM_PWMInitStruct->DutyArea));
    assert_param(IS_MCM_DUTY_POLARITY(MCM_PWMInitStruct->DutyPolarity));

    MCM_REG_UNLOCK(MCMx);

    temp = MCMx->PWMCON1.V32;
    
	/* Reset PUTMOD/PDCON2/PWM0S bits */
	temp &= ~(MCM_PWMCON1_POUTMOD_Msk | MCM_PWMCON1_PDCON2_Msk | MCM_PWMCON1_PWM21S_Msk);

	temp |= MCM_PWMInitStruct->PWMMode;
	temp |= MCM_PWMInitStruct->DutyArea << 2;
    temp |= (((MCM_PWMInitStruct->DutyPolarity >> 1) & (uint16_t)0x0001) << 5);

    if (MCM_PWMInitStruct->PWMMode == MCM_PWMMode_Complementary)
    {
        assert_param(IS_MCM_PWM_SYMMETRY(MCM_PWMInitStruct->PWMSymmetry));
        
        /* Reset PWMSYM bit */
        temp &= ~MCM_PWMCON1_PWMSYM_Msk;
        temp |= MCM_PWMInitStruct->PWMSymmetry;
    }

	MCMx->PWMCON1.V32 = temp;
    
    temp = MCMx->PWMCON2.V32;
    temp &= ~(MCM_DutyActivePoint_Now | MCM_DutyActivePoint_Zero | MCM_DutyActivePoint_Period);
	temp |=  MCM_PWMInitStruct->DutyActivePoint;
    MCMx->PWMCON2.V32 = temp;

	MCMx->PWM21D = MCM_PWMInitStruct->DutyValue;
    
    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Sets the MCMx Counter Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  Counter: specifies the Counter register new value.
  * @retval None
  */
void MCM_SetCounter(MCM_TypeDef* MCMx, uint16_t Counter)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMC = Counter;	

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Sets the MCMx Prescaler Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  Prescaler: specifies the Prescaler register new value.
  * @retval None
  */
void MCM_SetPrescaler(MCM_TypeDef* MCMx, uint16_t Prescaler)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMPSQ = Prescaler;	

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Sets the MCMx Period Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  Period: specifies the Period register new value.
  * @retval None
  */
void MCM_SetPeriod(MCM_TypeDef* MCMx, uint16_t Period)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMP = Period;
    
    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Get the MCMx Counter Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @retval The Counter Register value.
  */
uint16_t MCM_GetCounter(MCM_TypeDef* MCMx)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

	return MCMx->PWMC;	
}

/**
  * @brief  Get the MCMx Prescaler Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @retval The Prescaler Register value.
  */
uint16_t MCM_GetPrescaler(MCM_TypeDef* MCMx)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

	return MCMx->PWMPSQ;	
}

/**
  * @brief  Get the MCMx Period Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @retval The Period Register value.
  */
uint16_t MCM_GetPeriod(MCM_TypeDef* MCMx)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

	return MCMx->PWMP;	
}


/**
  * @brief  Set the MCMx PWM0 Duty Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  Duty0: specifies the PWM0 Duty Register new value.
  * @retval None
  */
void MCM_SetPWMDuty0(MCM_TypeDef* MCMx, uint16_t Duty0)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWM0D = Duty0;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx PWM01 Duty Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  Duty01: specifies the PWM01 Duty Register new value.
  * @retval None
  */
void MCM_SetPWMDuty01(MCM_TypeDef* MCMx, uint16_t Duty01)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWM01D = Duty01;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx PWM1 Duty Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  Duty1: specifies the PWM1 Duty Register new value.
  * @retval None
  */
void MCM_SetPWMDuty1(MCM_TypeDef* MCMx, uint16_t Duty1)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWM1D = Duty1;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx PWM11 Duty Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  Duty11: specifies the PWM11 Duty Register new value.
  * @retval None
  */
void MCM_SetPWMDuty11(MCM_TypeDef* MCMx, uint16_t Duty11)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWM11D = Duty11;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx PWM2 Duty Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  Duty2: specifies the PWM2 Duty Register new value.
  * @retval None
  */
void MCM_SetPWMDuty2(MCM_TypeDef* MCMx, uint16_t Duty2)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWM2D = Duty2;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx PWM01 Duty Register value
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  Duty21: specifies the PWM21 Duty Register new value.
  * @retval None
  */
void MCM_SetPWMDuty21(MCM_TypeDef* MCMx, uint16_t Duty21)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWM21D = Duty21;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Get the MCMx PWM0 Duty Register value.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @retval The PWM0 Duty Register value.
  */
uint16_t MCM_GetPWMDuty0(MCM_TypeDef* MCMx)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

	return MCMx->PWM0D;
}

/**
  * @brief  Set the MCMx PWM01 Duty Register value.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @retval The PWM01 Duty Register value.
  */
uint16_t MCM_GetPWMDuty01(MCM_TypeDef* MCMx)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

	return MCMx->PWM01D;
}

/**
  * @brief  Get the MCMx PWM1 Duty Register value.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @retval The PWM1 Duty Register value.
  */
uint16_t MCM_GetPWMDuty1(MCM_TypeDef* MCMx)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

	return MCMx->PWM1D;
}

/**
  * @brief  Set the MCMx PWM11 Duty Register value.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @retval The PWM11 Duty Register value.
  */
uint16_t MCM_GetPWMDuty11(MCM_TypeDef* MCMx)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

	return MCMx->PWM11D;
}

/**
  * @brief  Get the MCMx PWM2 Duty Register value.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @retval The PWM2 Duty Register value.
  */
uint16_t MCM_GetPWMDuty2(MCM_TypeDef* MCMx)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

	return MCMx->PWM2D;
}

/**
  * @brief  Set the MCMx PWM21 Duty Register value.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @retval The PWM21 Duty Register value.
  */
uint16_t MCM_GetPWMDuty21(MCM_TypeDef* MCMx)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

	return MCMx->PWM21D;
}

/**
  * @brief  configure interrupt or event flag division parameter.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  MCM_Division: specifies the value of division parameter.
  *          This parameter can be one of the following values:
  *            @arg MCM_INT_EVENT_FLAG_DIV_1
  *            @arg MCM_INT_EVENT_FLAG_DIV_2
  *            @arg MCM_INT_EVENT_FLAG_DIV_3
  *            @arg MCM_INT_EVENT_FLAG_DIV_4
  *            @arg MCM_INT_EVENT_FLAG_DIV_5
  *            @arg MCM_INT_EVENT_FLAG_DIV_6
  *            @arg MCM_INT_EVENT_FLAG_DIV_7
  *            @arg MCM_INT_EVENT_FLAG_DIV_8  
  * @retval None
  */
void MCM_IntEventFlagDivConfig(MCM_TypeDef* MCMx, uint16_t MCM_Division)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_INT_EVENT_FLAG_DIV(MCM_Division));

    MCM_REG_UNLOCK(MCMx);

    MCMx->PWMCON1.V32 &= ~MCM_PWMCON1_POSTPS_Msk;
    MCMx->PWMCON1.V32 |= MCM_Division;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Enable or disable the MCMx Duty and Event auto reload.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_AutoReload: specifie the auto reload register to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg MCM_CMP_Active_Now
  *            @arg MCM_CMP_Active_Zero
  *            @arg MCM_CMP_Active_Period
  * @param  NewState: new state of synchronous function.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE  
  * @retval None
  */
void MCM_CompareEventActiveConfig(MCM_TypeDef* MCMx, uint16_t MCM_CMP_Active, FunctionalState NewState)
{
    /* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_AUTO_RELOAD_REG(MCM_CMP_Active));
    assert_param(IS_FUNCTION_STATE(NewState));

    MCM_REG_UNLOCK(MCMx);

    if (NewState != DISABLE)
    {
        MCMx->PWMCON2.V32 |= MCM_CMP_Active;
    }
    else
    {
        MCMx->PWMCON2.V32 &= ~MCM_CMP_Active;
    }

    MCM_REG_LOCK(MCMx);
}


/**
  * @brief  Enable or Disable MCM1 and MCM2 synchronous function.
  * @param  NewState: new state of synchronous function.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void MCM_SynchronousConfig(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTION_STATE(NewState));

    MCM_REG_UNLOCK(MCM1);
    MCM_REG_UNLOCK(MCM2);

    /* enable or disable MCMSYNEN bit of MCM1 and MCM2 */
    MCM1_PWMCON2_MCMSYNEN_BIT = (uint32_t)NewState;
    MCM2_PWMCON2_MCMSYNEN_BIT = (uint32_t)NewState;

    MCM_REG_LOCK(MCM1);
    MCM_REG_LOCK(MCM2);
}

/**
  * @brief  Open or close oscillator stop detection.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  OnOffState: state of oscillator stop detection.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void MCM_OSCStopDetectOnOff(MCM_TypeDef* MCMx, CmdState OnOffState)
{
	/* Check the parameters */
    assert_param(IS_MCM_ALL_PERIPH(MCMx));
	assert_param(IS_CMD_STATE(OnOffState));

    MCM_FLT_REG_UNLOCK(MCMx);

    if (OnOffState != OFF)
    {
        MCMx->POSTDCR.V32 = MCM_POSTDCR_OSTDEN_Msk;
    }
    else
    {
        MCMx->POSTDCR.V32 = ~MCM_POSTDCR_OSTDEN_Msk;
    }

    MCM_FLT_REG_LOCK(MCMx);
}

/**
  * @brief  Enable or Disable MCM's DMA requests.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  DMASource: specifie the DMA request sources.
  *          This parameter can be any combination of following values:
  *            @arg MCM_DMA_PWM0_Up
  *            @arg MCM_DMA_PWM0_Down
  *            @arg MCM_DMA_PWM1_Up
  *            @arg MCM_DMA_PWM1_Down
  *            @arg MCM_DMA_PWM2_Up
  *            @arg MCM_DMA_PWM2_Down
  *            @arg MCM_DMA_ZeroMatch
  *            @arg MCM_DMA_PeriodMatch  
  * @param  NewState: new state of DMA Request sources.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void MCM_DMAConfig(MCM_TypeDef* MCMx, uint8_t DMASource, FunctionalState NewState)
{
	/* Check the parameters */
    assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_DMA_SOURCE(DMASource));
	assert_param(IS_FUNCTION_STATE(NewState));

    MCM_REG_UNLOCK(MCMx);

    if (NewState != DISABLE)
    {
        /* Enable the DMA sources */
        MCMx->PWMDMAEN.V32 |= DMASource; 
    }
    else
    {
        /* Disable the DMA sources */
        MCMx->PWMDMAEN.V32 &= ~DMASource;
    }

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Enable or Disable specified MCM interrupts.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  MCM_INT: specifie the MCM interrupts sources to be enabled or disabled.
  *          This parameter can be any combination of following values:
  *            @arg MCM_INT_PWM0_UP
  *            @arg MCM_INT_PWM0_DOWN
  *            @arg MCM_INT_PWM1_UP
  *            @arg MCM_INT_PWM1_DOWN
  *            @arg MCM_INT_PWM2_UP
  *            @arg MCM_INT_PWM2_DOWN
  *            @arg MCM_INT_ZM
  *            @arg MCM_INT_PM
  *            @arg MCM_INT_FLT
  *            @arg MCM_INT_OUT
  *            @arg MCM_INT_OSC 
  * @param  NewState: new state of interrupt sources.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void MCM_INTConfig(MCM_TypeDef* MCMx, uint16_t MCM_INT, FunctionalState NewState)
{
	/* Check the parameters */
    assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_INT(MCM_INT));
	assert_param(IS_FUNCTION_STATE(NewState));  

    MCM_REG_UNLOCK(MCMx);

    if (NewState != DISABLE)
    {
        /* Enable the interrupt sources */
        MCMx->PWMINTEN.V32 |= MCM_INT;
    }
    else
    {
        /* Disable the interrupt sources */
        MCMx->PWMINTEN.V32 &= ~MCM_INT;
    }

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Check whether the specified MCM flag is set or not
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  MCM_Flag: specifie the flag to check.
  *          This parameter can be one of the following values:
  *            @arg MCM_FLAG_PWM0_UP
  *            @arg MCM_FLAG_PWM0_DOWN
  *            @arg MCM_FLAG_PWM1_UP
  *            @arg MCM_FLAG_PWM1_DOWN
  *            @arg MCM_FLAG_PWM2_UP
  *            @arg MCM_FLAG_PWM2_DOWN
  *            @arg MCM_FLAG_ZM
  *            @arg MCM_FLAG_PM
  *            @arg MCM_FLAG_FLT
  *            @arg MCM_FLAG_OUT
  *            @arg MCM_FLAG_OSC
  *            @arg MCM_FLAG_SC1STAT
  *            @arg MCM_FLAG_SC2STAT
  *            @arg MCM_FLAG_SC3STAT  
  * @retval None
  */
FlagStatus MCM_GetFlagStatus(MCM_TypeDef* MCMx, uint16_t MCM_Flag)
{
    FlagStatus bitStatus;
    
	/* Check the parameters */
    assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_GET_ONE_FLAG(MCM_Flag));

    if ((MCMx->PWMINTF.V32 & MCM_Flag) != (uint16_t)RESET)
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
  * @brief  Clear the MCMx's pending flags.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_Flag: specifie the flag bit to clear.
  *          This parameter can be any combination of the following values:
  *            @arg MCM_FLAG_PWM0_UP
  *            @arg MCM_FLAG_PWM0_DOWN
  *            @arg MCM_FLAG_PWM1_UP
  *            @arg MCM_FLAG_PWM1_DOWN
  *            @arg MCM_FLAG_PWM2_UP
  *            @arg MCM_FLAG_PWM2_DOWN
  *            @arg MCM_FLAG_ZM
  *            @arg MCM_FLAG_PM
  *            @arg MCM_FLAG_FLT
  *            @arg MCM_FLAG_OUT
  *            @arg MCM_FLAG_OSC
  *            @arg MCM_FLAG_SC1STAT
  *            @arg MCM_FLAG_SC2STAT
  *            @arg MCM_FLAG_SC3STAT  
  * @retval None
  */
void MCM_ClearFlag(MCM_TypeDef* MCMx, uint16_t MCM_Flag)
{
	/* Check the parameters */
    assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_GET_ONE_FLAG(MCM_Flag));

    MCMx->PWMINTF.V32 = (uint32_t)MCM_Flag << 16;
}

/**
  * @brief  Configure the MCMx manual PWM Out Register active timing.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_ActiveTiming: specifie the timing of the manual register active.
  *          This parameter can be one of the following values:
  *            @arg MCM_MannualActiveTiming_Now
  *            @arg MCM_MannualActiveTiming_PeriodUpdate  
  * @retval None
  */
void MCM_ManualPWMOutRegSync(MCM_TypeDef* MCMx, uint16_t MCM_ActiveTiming)
{    
	/* Check the parameters */
    assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_MANUAL_ACTIVE_TIMING(MCM_ActiveTiming));

    MCM_REG_UNLOCK(MCMx);

    MCMx->PWMCON2.V32 &= ~MCM_PWMCON2_OSYNC_Msk;
    MCMx->PWMCON2.V32 |= MCM_ActiveTiming;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Turn On or turn off manual output mode of the specified PWM port.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  PWMxPin: specifie PWM port to be enabled or disabled.
  *          This parameter can be any combination of following values:
  *            @arg MCM_MANUAL_PWM0
  *            @arg MCM_MANUAL_PWM1
  *            @arg MCM_MANUAL_PWM2
  *            @arg MCM_MANUAL_PWM01
  *            @arg MCM_MANUAL_PWM11
  *            @arg MCM_MANUAL_PWM21
  * @param  OnOffState: state of synchronous function.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void MCM_ManualPWMOutOnOff(MCM_TypeDef* MCMx, uint8_t MCM_PWMx, CmdState OnOffState)
{
	/* Check the parameters */
    assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_MANUAL_PWM(MCM_PWMx));
	assert_param(IS_CMD_STATE(OnOffState)); 

    MCM_REG_UNLOCK(MCMx);

    if (OnOffState != OFF)
    {
        /* Enable manual output mode of specified PWM port */
        MCMx->PMANUALCON1.V32 |= MCM_PWMx; 
    }
    else
    {
        /* Disable manual output mode of specified PWM port */
        MCMx->PMANUALCON1.V32 &= ~MCM_PWMx;
    }

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the specified PWM port output level through the PWM Mannual Register.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMx: specifie PWM port to set.
  *          This parameter can be any combination of following values:
  *            @arg MCM_MANUAL_PWM0
  *            @arg MCM_MANUAL_PWM1
  *            @arg MCM_MANUAL_PWM2
  *            @arg MCM_MANUAL_PWM01
  *            @arg MCM_MANUAL_PWM11
  *            @arg MCM_MANUAL_PWM21
  * @retval None
  */
void MCM_SetManualPWMOut(MCM_TypeDef* MCMx, uint8_t MCM_PWMx)
{
	/* Check the parameters */
    assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_MANUAL_PWM(MCM_PWMx));

    MCM_REG_UNLOCK(MCMx);
    
    /* Set the specified PWM port */
    MCMx->PMANUALCON2.V32 |= MCM_PWMx;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Reset the specified PWM port output level through the PWM Mannual Register.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMx: specifie PWM port to reset.
  *          This parameter can be any combination of following values:
  *            @arg MCM_MANUAL_PWM0
  *            @arg MCM_MANUAL_PWM1
  *            @arg MCM_MANUAL_PWM2
  *            @arg MCM_MANUAL_PWM01
  *            @arg MCM_MANUAL_PWM11
  *            @arg MCM_MANUAL_PWM21
  * @retval None
  */
void MCM_ResetManualPWMOut(MCM_TypeDef* MCMx, uint8_t MCM_PWMx)
{
	/* Check the parameters */
    assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_MANUAL_PWM(MCM_PWMx));

    MCM_REG_UNLOCK(MCMx);
    
    /* Reset the specified PWM port */
    MCMx->PMANUALCON2.V32 &= ~MCM_PWMx;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Configure the specified PWM port output level through the PWM Mannual Register.
  * @param  MCMx: where x can be 1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMx: specifie PWM port to reset.
  *          This parameter can be any combination of following values:
  *            @arg MCM_MANUAL_PWM0
  *            @arg MCM_MANUAL_PWM1
  *            @arg MCM_MANUAL_PWM2
  *            @arg MCM_MANUAL_PWM01
  *            @arg MCM_MANUAL_PWM11
  *            @arg MCM_MANUAL_PWM21
  * @param  MCM_OutputLevel: specifie the PWM port output level.
  *          This parameter can be one of the following values:
  *            @arg MCM_OutputLevel_Low
  *            @arg MCM_OutputLevel_High
  * @retval None
  */
void MCM_MannualPWMOutConfig(MCM_TypeDef* MCMx, uint8_t MCM_PWMx, uint8_t MCM_OutputLevel)
{
	/* Check the parameters */
    assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_MANUAL_PWM(MCM_PWMx));

    MCM_REG_UNLOCK(MCMx);

    if (MCM_OutputLevel != MCM_OutputLevel_Low)
    {
        MCMx->PMANUALCON2.V32 |= MCM_PWMx;
    }
    else
    {
        MCMx->PMANUALCON2.V32 &= ~MCM_PWMx;
    }

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Enable or disable FLT module registers write protection.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  NewState: new state of FLT write protection.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */
void MCM_FLTWriteLock(MCM_TypeDef* MCMx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
	assert_param(IS_FUNCTION_STATE(NewState));
	
	if (NewState != DISABLE)
	{
        /* Enable the FLT register write protection */
		MCMx->FLTWEN = 0x0000;
	}
	else
	{
        /* Disable the FLT register write protection */
		MCMx->FLTWEN = 0x33CC;
	}
}

/**
  * @brief  Enable or disable Fault Detect Module(FLT).  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  FLT: specifie which FLT module to be enable or disable.
  *          This parameter can be any combination of the following values:
  *            @arg MCM_FLT_Pin
  *            @arg MCM_FLT_COMP
  * @param  OnOffState: state of FLT module.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void MCM_FLTOnOff(MCM_TypeDef* MCMx, uint16_t MCM_FLT, CmdState OnOffState)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
	assert_param(IS_MCM_FLT_INPUT(MCM_FLT));
	assert_param(IS_CMD_STATE(OnOffState));

    MCM_FLT_REG_UNLOCK(MCMx);

	if (OnOffState != OFF)
	{
		MCMx->FLTCON.V32 |= MCM_FLT;
	}
	else
	{
		MCMx->FLTCON.V32 &= ~MCM_FLT;
	}

    MCM_FLT_REG_LOCK(MCMx);
}

/**
  * @brief  Configure input source of the MCMx FLT1 module.   
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_FLTSource: specifie the FLT1 input sources.
  *          This parameter can be one of the following values:
  *            @arg MCM_FLT1_Source_COMP1
  *            @arg MCM_FLT1_Source_COMP2
  *            @arg MCM_FLT1_Source_COMP3  
  * @retval None
  */
void MCM_FLT1SourceConfig(MCM_TypeDef* MCMx, uint16_t MCM_FLTSource)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
	assert_param(IS_MCM_FLT1_SOURCE(MCM_FLTSource));

    MCM_FLT_REG_UNLOCK(MCMx);

	/* Reset the FLT1SEL bits*/
	MCMx->FLTCON.V32 &= ~MCM_FLTCON_FLT1SEL_Msk;
	MCMx->FLTCON.V32 |= MCM_FLTSource;

    MCM_FLT_REG_LOCK(MCMx);
}

/**
  * @brief  Configure filter and active level of the MCMx FLT2 module.   
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_FLT2Filter: specifie the FLT2 filter value.
  *          This parameter can be one of the following values:
  *            @arg MCM_FLT2_Filter_None         
  *            @arg MCM_FLT2_Filter_0dot5us      
  *            @arg MCM_FLT2_Filter_1us          
  *            @arg MCM_FLT2_Filter_1dot5us      
  *            @arg MCM_FLT2_Filter_2us          
  *            @arg MCM_FLT2_Filter_3us          
  *            @arg MCM_FLT2_Filter_4us          
  *            @arg MCM_FLT2_Filter_6us          
  *            @arg MCM_FLT2_Filter_8us          
  *            @arg MCM_FLT2_Filter_10us         
  *            @arg MCM_FLT2_Filter_12us         
  *            @arg MCM_FLT2_Filter_14us         
  *            @arg MCM_FLT2_Filter_16us         
  *            @arg MCM_FLT2_Filter_20us         
  *            @arg MCM_FLT2_Filter_24us         
  *            @arg MCM_FLT2_Filter_32us  
  * @param  MCM_FLT2ActiveLevel: specifie the FLT2 active level.
  *          This parameter can be one of the following values:
  *            @arg MCM_FLT2_ActiveLevel_High
  *            @arg MCM_FLT2_ActiveLevel_Low
  * @retval None
  */
void MCM_FLT2Config(MCM_TypeDef* MCMx, uint16_t MCM_FLT2Filter, uint16_t MCM_FLT2ActiveLevel)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
	assert_param(IS_MCM_FLT2_FILTER(MCM_FLT2Filter));	
	assert_param(IS_MCM_FLT2_ACTIVE_LEVEL(MCM_FLT2ActiveLevel));

    MCM_FLT_REG_UNLOCK(MCMx);

	MCMx->FLTCON.V32 &= ~(MCM_FLTCON_FLT2DEB_Msk | MCM_FLTCON_FLT2S_Msk);
	MCMx->FLTCON.V32 |= (MCM_FLT2Filter | MCM_FLT2ActiveLevel);

    MCM_FLT_REG_LOCK(MCMx);
}

/**
  * @brief  Configure the MCMx FLT mode and output pin status.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_DetectMode: specifie the FLT detect mode to set.
  *          This parameter can be one of the following values:
  *            @arg MCM_FLT_DETECT_Latch
  *            @arg MCM_FLT_DETECT_Each
  * @param  MCM_PWMxPinStatus: specifie the output status of the FLT PWMx pin.
  *          This parameter can be one of the following values:
  *            @arg MCM_FLT_PWMx_PinStatus_Hiz
  *            @arg MCM_FLT_PWMx_PinStatus_Low
  *            @arg MCM_FLT_PWMx_PinStatus_High  
  * @param  MCM_PWMx1PinStatus: specifie the output status of the FLT PWMx1 pin.
  *          This parameter can be one of the following values:
  *            @arg MCM_FLT_PWMx1_PinStatus_Hiz
  *            @arg MCM_FLT_PWMx1_PinStatus_Low
  *            @arg MCM_FLT_PWMx1_PinStatus_High   
  * @retval None
  */
void MCM_FLTConfig(MCM_TypeDef* MCMx, uint16_t MCM_DetectMode, uint16_t MCM_PWMxPinStatus, uint16_t MCM_PWMx1PinStatus)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
	assert_param(IS_MCM_FLT_DETECT_MODE(MCM_DetectMode));
	assert_param(IS_MCM_FLT_PWMx_PIN_STATUS(MCM_PWMxPinStatus));
	assert_param(IS_MCM_FLT_PWMx1_PIN_STATUS(MCM_PWMx1PinStatus));

    MCM_FLT_REG_UNLOCK(MCMx);

	MCMx->FLTCON.V32 &= ~(MCM_FLTCON_FLTM_Msk | MCM_FLTCON_FOUT0_Msk | MCM_FLTCON_FOUT1_Msk);
	MCMx->FLTCON.V32 |= (MCM_DetectMode | MCM_PWMxPinStatus | MCM_PWMx1PinStatus);	

    MCM_FLT_REG_LOCK(MCMx);
}

/**
  * @brief  Clear the MCMx's FLT pending flag.
  * @note   This function only can be called in Latch mode of FLT,because this bit will be
  *         set by hardware in Each Mode.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @retval None
  */
void MCM_ClearFLTStatusFlag(MCM_TypeDef* MCMx)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_FLT_REG_UNLOCK(MCMx);

	MCMx->FLTCON.V32 &= ~MCM_FLTCON_FLTSTAT_Msk;

    MCM_FLT_REG_LOCK(MCMx);
}

/**
  * @brief  Check whether the specified MCM FLT status flag is set or not  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @retval None
  */
FlagStatus MCM_GetFLTStatus(MCM_TypeDef* MCMx)
{
	FlagStatus bitStatus;
	
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

	if ((MCMx->FLTCON.V32 & MCM_FLTCON_FLTSTAT_Msk) != RESET)
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
  * @brief  Set the MCMx PWM0 module dead time register value.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  PWM0DT: specifie the PWM0 port dead time register value.
  * @param  PWM01DT: specifie the PWM01 port dead time register value.
  * @retval None
  */
void MCM_PWM0DeadTimeConfig(MCM_TypeDef* MCMx, uint16_t PWM0DT,uint16_t PWM01DT)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMDT00 = PWM0DT;
	MCMx->PWMDT01 = PWM01DT;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx PWM1 module dead time register value.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  PWM0DT: specifie the PWM1 port dead time register value.
  * @param  PWM01DT: specifie the PWM11 port dead time register value.
  * @retval None
  */
void MCM_PWM1DeadTimeConfig(MCM_TypeDef* MCMx, uint16_t PWM1DT,uint16_t PWM11DT)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMDT10 = PWM1DT;
	MCMx->PWMDT11 = PWM11DT;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx PWM2 module dead time register value.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  PWM0DT: specifie the PWM2 port dead time register value.
  * @param  PWM01DT: specifie the PWM21 port dead time register value.
  * @retval None
  */
void MCM_PWM2DeadTimeConfig(MCM_TypeDef* MCMx, uint16_t PWM2DT,uint16_t PWM21DT)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMDT20 = PWM2DT;
	MCMx->PWMDT21 = PWM21DT;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx Trigger ADC Register 1 value.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  Value1: specifie Trigger ADC Register 1 value.
  *          This parameter can be 0 to 0xFFFF.
  * @retval None
  */
void MCM_SetADCTriggerValue1(MCM_TypeDef* MCMx, uint16_t Value1)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMCMP1 = Value1;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx Trigger ADC Register 2 value.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  Value1: specifie Trigger ADC Register 2 value.
  *          This parameter can be 0 to 0xFFFF.  
  * @retval None
  */
void MCM_SetADCTriggerValue2(MCM_TypeDef* MCMx, uint16_t Value2)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMCMP2 = Value2;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx Trigger ADC Register 3 value.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  Value1: specifie Trigger ADC Register 3 value.
  *          This parameter can be 0 to 0xFFFF.  
  * @retval None
  */
void MCM_SetADCTriggerValue3(MCM_TypeDef* MCMx, uint16_t Value3)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMCMP3 = Value3;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Set the MCMx Trigger ADC Register 4 value.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  Value1: specifie Trigger ADC Register 4 value.
  *          This parameter can be 0 to 0xFFFF.  
  * @retval None
  */
void MCM_SetADCTriggerValue4(MCM_TypeDef* MCMx, uint16_t Value4)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMCMP4 = Value4;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Configure the MCMx ADC trigger timing.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_ADCTrigger: 
  * @retval None
  */
void MCM_ADCTriggerConfig(MCM_TypeDef* MCMx, uint16_t MCM_ADCTrigger)
{
	uint8_t i = 0;
	uint16_t temp = 0;
	
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	temp = MCMx->PWMCON2.V32;

	for (i=0;i<4;i++)
	{
		if (((MCM_ADCTrigger >> (i * 2)) & 0x100) == 0x100)
		{
			temp &= (uint16_t)0xFFFC << (i * 2);
			temp |= (MCM_ADCTrigger & ((uint16_t)0x0003 << (i * 2)));
		}
	}
	
	MCMx->PWMCON2.V32 = temp;

    MCM_REG_LOCK(MCMx);
}


/**
  * @brief  Configure the MCMx ADC trigger 1 sources.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_ADCTrigger1: specifie the ADC trigger 1 source.
  *          This parameter can be one of the following values:
  *            @arg MCM_TRIGGER_CMP1_None
  *            @arg MCM_TRIGGER_CMP1_RisingFalling
  *            @arg MCM_TRIGGER_CMP1_Rising
  *            @arg MCM_TRIGGER_CMP1_Falling  
  * @retval None
  */
void MCM_ADCTrigger1Config(MCM_TypeDef* MCMx, uint16_t MCM_ADCTrigger1)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_ADC_TRIGGER1_SOURCE(MCM_ADCTrigger1));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMCON2.V32 &= ~MCM_PWMCON2_CMP1_Msk;
	MCMx->PWMCON2.V32 |= MCM_ADCTrigger1;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Configure the MCMx ADC trigger 2 sources.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_ADCTrigger2: specifie the ADC trigger 2 source.
  *          This parameter can be one of the following values:
  *            @arg MCM_TRIGGER_CMP2_None
  *            @arg MCM_TRIGGER_CMP2_RisingFalling
  *            @arg MCM_TRIGGER_CMP2_Rising
  *            @arg MCM_TRIGGER_CMP2_Falling  
  * @retval None
  */
void MCM_ADCTrigger2Config(MCM_TypeDef* MCMx, uint16_t MCM_ADCTrigger2)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_ADC_TRIGGER2_SOURCE(MCM_ADCTrigger2));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMCON2.V32 &= ~MCM_PWMCON2_CMP2_Msk;
	MCMx->PWMCON2.V32 |= MCM_ADCTrigger2;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Configure the MCMx ADC trigger 3 sources.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_ADCTrigger3: specifie the ADC trigger 3 source.
  *          This parameter can be one of the following values:
  *            @arg MCM_TRIGGER_CMP3_None
  *            @arg MCM_TRIGGER_CMP3_RisingFalling
  *            @arg MCM_TRIGGER_CMP3_Rising
  *            @arg MCM_TRIGGER_CMP3_Falling  
  * @retval None
  */
void MCM_ADCTrigger3Config(MCM_TypeDef* MCMx, uint16_t MCM_ADCTrigger3)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_ADC_TRIGGER3_SOURCE(MCM_ADCTrigger3));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMCON2.V32 &= ~MCM_PWMCON2_CMP3_Msk;
	MCMx->PWMCON2.V32 |= MCM_ADCTrigger3;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Configure the MCMx ADC trigger 4 sources.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_ADCTrigger4: specifie the ADC trigger 4 source.
  *          This parameter can be one of the following values:
  *            @arg MCM_TRIGGER_CMP4_None
  *            @arg MCM_TRIGGER_CMP4_RisingFalling
  *            @arg MCM_TRIGGER_CMP4_Rising
  *            @arg MCM_TRIGGER_CMP4_Falling  
  * @retval None
  */
void MCM_ADCTrigger4Config(MCM_TypeDef* MCMx, uint16_t MCM_ADCTrigger4)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_ADC_TRIGGER4_SOURCE(MCM_ADCTrigger4));

    MCM_REG_UNLOCK(MCMx);

	MCMx->PWMCON2.V32 &= ~MCM_PWMCON2_CMP4_Msk;
	MCMx->PWMCON2.V32 |= MCM_ADCTrigger4;

    MCM_REG_LOCK(MCMx);
}


/**
  * @brief  Set the MCMx Duty saturation minimum and maximum Register value. 
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMMin: specifie the minimum value of the duty saturation.
  *          This parameter can be 0 to 0xFFFF.
  * @param  MCM_PWMMax: specifie the maximum value of the duty saturation.
  *          This parameter can be 0 to 0xFFFF.  
  * @retval None
  */
void MCM_DutySaturationMinMaxValue(MCM_TypeDef* MCMx, uint16_t MCM_PWMMin, uint16_t MCM_PWMMax)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

    MCMx->PWMDMIN = MCM_PWMMin;
    MCMx->PWMDMAX = MCM_PWMMax;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Configure the MCMx Duty Register Change or not with the duty saturation function.   
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_MinSelect: specifie the minimum register to set.
  *          This parameter can be one of the following values:
  *            @arg MCM_MinSelect_None
  *            @arg MCM_MinSelect_ToMin
  *            @arg MCM_MinSelect_ToZero
  * @param  MCM_MaxSelect: specifie the maximum register to set.
  *          This parameter can be one of the following values:
  *            @arg MCM_MaxSelect_None
  *            @arg MCM_MaxSelect_ToMax
  *            @arg MCM_MaxSelect_ToPeriod   
  * @retval None
  */
void MCM_DutySaturationConfig(MCM_TypeDef* MCMx, uint8_t MCM_MinSelect, uint8_t MCM_MaxSelect)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_MIN_SELECT(MCM_MinSelect));
    assert_param(IS_MCM_MAX_SELECT(MCM_MaxSelect));

    MCM_REG_UNLOCK(MCMx);

    MCMx->PSCON.V32 &= ~(MCM_PSCON_MINSELECT_Msk | MCM_PSCON_MAXSELECT_Msk);
    MCMx->PSCON.V32 |= MCM_MinSelect | MCM_MaxSelect;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Open or close the MCMx's Duty Saturation function.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  OnOffState: state of the Duty Saturation function.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF
  * @retval None
  */
void MCM_DutySaturationOnOff(MCM_TypeDef* MCMx, CmdState OnOffState)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_CMD_STATE(OnOffState));

    MCM_REG_UNLOCK(MCMx);

    if (OnOffState != OFF)
    {
        /* Enable the duty saturation function */
        MCMx->PSCON.V32 |= MCM_PSCON_STATRUN_Msk;
    }
    else
    {
        /* Disable the duty saturation function */
        MCMx->PSCON.V32 &= ~MCM_PSCON_STATRUN_Msk;
    }

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  configure the MCMx's phase shift function.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_ShiftSelect: specifie the selection of the phase shift function.
  *          This parameter can be one of the following values:
  *            @arg MCM_PS_MaxToMin_PWMD210             
  *            @arg MCM_PS_MaxToMin_PWMD201             
  *            @arg MCM_PS_MaxToMin_PWMD021             
  *            @arg MCM_PS_MaxToMin_PWMD012             
  *            @arg MCM_PS_MaxToMin_PWMD102             
  *            @arg MCM_PS_MaxToMin_PWMD120
  * @param  MCM_Shift1: specifie the Phase Shift Register 1 value.
  *          This parameter can be 0 to 0xFFFF.
  * @param  MCM_Shift2: specifie the Phase Shift Register 2 value.
  *          This parameter can be 0 to 0xFFFF.  
  * @retval None
  */
void MCM_PhaseShiftConfig(MCM_TypeDef* MCMx, uint8_t MCM_ShiftSelect, uint16_t MCM_Shift1, uint16_t MCM_Shift2)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_PS_MAXTOMIN_SELECT(MCM_ShiftSelect));

    MCM_REG_UNLOCK(MCMx);

    MCMx->PSCON.V32 &= ~MCM_PSCON_SECTOR_Msk;
    MCMx->PSCON.V32 |= MCM_ShiftSelect;

    MCMx->PWMDCMP1 = MCM_Shift1;
    MCMx->PWMDCMP2 = MCM_Shift2;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Open or close the MCMx's Phase Shift function.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  OnOffState: state of the Phase Shift function.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF  
  * @retval None
  */
void MCM_PhaseShiftOnOff(MCM_TypeDef* MCMx, CmdState OnOffState)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_CMD_STATE(OnOffState));

    MCM_REG_UNLOCK(MCMx);

    if (OnOffState != OFF)
    {
        /* Enable the Phase Shift function */
        MCMx->PSCON.V32 |= MCM_PSCON_SHIFTRUN_Msk;
    }
    else
    {
        /* Disable the Phase Shift function */
        MCMx->PSCON.V32 &= ~MCM_PSCON_SHIFTRUN_Msk;
    }

    MCM_REG_LOCK(MCMx);
}


/**
  * @brief  Open or close the MCMx short circuit protection function.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  OnOffState: state of the short circuit protection.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF  
  * @retval None
  */
void MCM_ShortCircuitProtectOnOff(MCM_TypeDef* MCMx, CmdState OnOffState)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_CMD_STATE(OnOffState));

    MCM_FLT_REG_UNLOCK(MCMx);

    if (OnOffState != OFF)
    {
        MCMx->POSCR.V32 |= MCM_POSCR_OLSEN_Msk;
    }
    else
    {
        MCMx->POSCR.V32 &= ~MCM_POSCR_OLSEN_Msk;
    }

    MCM_FLT_REG_LOCK(MCMx);
}

/**
  * @brief  Configure the MCMx short circuit output level.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_PWMxPin: specifie the PWM Pin of the short circuit function.
  *          This parameter can be any combination of the following values:
  *            @arg MCM_ShortCircuit_PWM0Pin     
  *            @arg MCM_ShortCircuit_PWM1Pin         
  *            @arg MCM_ShortCircuit_PWM2Pin        
  *            @arg MCM_ShortCircuit_PWM01Pin    
  *            @arg MCM_ShortCircuit_PWM11Pin       
  *            @arg MCM_ShortCircuit_PWM21Pin       
  * @param  MCM_OutputLevel: specifie the output level of the short circuit function.
  *          This parameter can be one of the following values:
  *            @arg MCM_ShortCircuit_Level_Low
  *            @arg MCM_ShortCircuit_Level_High
  * @retval None
  */
void MCM_ShortCircuitOutPutLevel(MCM_TypeDef* MCMx, uint8_t MCM_PWMxPin, uint8_t MCM_OutputLevel)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
    assert_param(IS_MCM_SHORT_CIRCUIT_PIN(MCM_PWMxPin));
    assert_param(IS_MCM_SHORT_CIRCUIT_LEVEL(MCM_OutputLevel));

    MCM_FLT_REG_UNLOCK(MCMx);

    if (MCM_OutputLevel != MCM_ShortCircuit_Level_Low)
    {
        MCMx->POSCR.V32 |= MCM_PWMxPin;
    }
    else
    {
        MCMx->POSCR.V32 &= ~MCM_PWMxPin;
    }

    MCM_FLT_REG_LOCK(MCMx);
}

/**
  * @brief  Fills each MCM_SCInitStruct member with its default value.
  * @param  MCM_SCInitStruct : pointer to a @ref MCM_SCInitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
void MCM_SCStructInit(MCM_SCInitTypeDef* MCM_SCInitStruct)
{
    MCM_SCInitStruct->SC_ProtectPWM = 0x00;
    MCM_SCInitStruct->SC_InputLevel = MCM_SC_InputLevel_High;
    MCM_SCInitStruct->SC_ProtectTime = MCM_SC_ProtectTime_600CLK;
    MCM_SCInitStruct->SC_Filter = MCM_SC_Filter_None;
}

/**
  * @brief  Initialize the MCMx Stoppage Check(SC) module 1 according to 
  *         the specified parameters in the MCM_SCInitStruct.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_SCInitStruct: pointer to a @ref MCM_SCInitTypeDef structure
  *         that contains the configuration information for the Stoppage Check(SC) module 1.
  * @retval None
  */
void MCM_SC1Init(MCM_TypeDef* MCMx, MCM_SCInitTypeDef* MCM_SCInitStruct)
{
	uint16_t temp = 0;
	
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	/* Reset SCTIME/SCDEB/SCS/ bits */
	MCMx->SC1CON.V32 &= (uint16_t)0xF700;

	temp = MCM_SCInitStruct->SC_InputLevel;
	temp |= MCM_SCInitStruct->SC_ProtectPWM;
	temp |= MCM_SCInitStruct->SC_Filter;
	temp |= MCM_SCInitStruct->SC_ProtectTime;
	MCMx->SC1CON.V32 |= temp;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Initialize the MCMx Stoppage Check(SC) module 2 according to 
  *         the specified parameters in the MCM_SCInitStruct.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_SCInitStruct: pointer to a @ref MCM_SCInitTypeDef structure
  *         that contains the configuration information for the Stoppage Check(SC) module 2.
  * @retval None
  */
void MCM_SC2Init(MCM_TypeDef* MCMx, MCM_SCInitTypeDef* MCM_SCInitStruct)
{
	uint16_t temp = 0;
	
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	/* Reset SCTIME/SCDEB/SCS/ bits */
	MCMx->SC2CON.V32 &= (uint16_t)0xF700;

	temp = MCM_SCInitStruct->SC_InputLevel;
	temp |= MCM_SCInitStruct->SC_ProtectPWM;
	temp |= MCM_SCInitStruct->SC_Filter;
	temp |= MCM_SCInitStruct->SC_ProtectTime;

	MCMx->SC2CON.V32 |= temp;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Initialize the MCMx Stoppage Check(SC) module 3 according to 
  *         the specified parameters in the MCM_SCInitStruct.  
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  MCM_SCInitStruct: pointer to a @ref MCM_SCInitTypeDef structure
  *         that contains the configuration information for the Stoppage Check(SC) module 3.
  * @retval None
  */
void MCM_SC3Init(MCM_TypeDef* MCMx, MCM_SCInitTypeDef* MCM_SCInitStruct)
{
	uint16_t temp = 0;
	
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	/* Reset SCTIME/SCDEB/SCS/ bits */
	MCMx->SC3CON.V32 &= (uint16_t)0xF700;

	temp = MCM_SCInitStruct->SC_InputLevel;
	temp |= MCM_SCInitStruct->SC_ProtectPWM;
	temp |= MCM_SCInitStruct->SC_Filter;
	temp |= MCM_SCInitStruct->SC_ProtectTime;

	MCMx->SC3CON.V32 |= temp;

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Enable or disable the MCMx Stoppage Check(SC) module protect PWMx. 
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  SCx: specifie which SC module to be selected.
  *          This parameter can be any combination of the following values:
  *            @arg MCM_SC1
  *            @arg MCM_SC2
  *            @arg MCM_SC3  
  * @param  MCM_PWMx: specifie the PWMx to be protected or unprotected.
  *          This parameter can be any combination of the following values:
  *            @arg MCM_SC_Protect_PWM0
  *            @arg MCM_SC_Protect_PWM1
  *            @arg MCM_SC_Protect_PWM2  
  * @param  NewState: new state of the SC PWM protection function.
  *          This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE   
  * @retval None
  */
void MCM_SCPWMProtectConfig(MCM_TypeDef* MCMx, uint16_t SCx, uint16_t MCM_PWMx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));

    MCM_REG_UNLOCK(MCMx);

	if (NewState != DISABLE)
	{
		if ((SCx & MCM_SC1) == MCM_SC1)
		{
			MCMx->SC1CON.V32 |= MCM_PWMx;
		}
		if ((SCx & MCM_SC2) == MCM_SC2)
		{
			MCMx->SC2CON.V32 |= MCM_PWMx;
		}
		if ((SCx & MCM_SC3) == MCM_SC3)
		{
			MCMx->SC3CON.V32 |= MCM_PWMx;
		}
	}
	else
	{
		if ((SCx & MCM_SC1) == MCM_SC1)
		{
			MCMx->SC1CON.V32 &= ~MCM_PWMx;
		}
		if ((SCx & MCM_SC2) == MCM_SC2)
		{
			MCMx->SC2CON.V32 &= ~MCM_PWMx;
		}
		if ((SCx & MCM_SC3) == MCM_SC3)
		{
			MCMx->SC3CON.V32 &= ~MCM_PWMx;
		}
	}  

    MCM_REG_LOCK(MCMx);
}

/**
  * @brief  Enable or disable the MCMx's Stoppage Check(SC) module.
  * @param  MCMx: where x can be  1 or 2 to select the MCM peripheral.
  * @param  SCx: specifie which SC module to be selected.
  *          This parameter can be any combination of the following values:
  *            @arg MCM_SC1
  *            @arg MCM_SC2
  *            @arg MCM_SC3    
  * @param  OnOffState: state of the SC module.
  *          This parameter can be one of the following values:
  *            @arg ON
  *            @arg OFF  
  * @retval None
  */
void MCM_SCOnOff(MCM_TypeDef* MCMx, uint16_t SCx, CmdState OnOffState)
{    
	/* Check the parameters */
	assert_param(IS_MCM_ALL_PERIPH(MCMx));
	assert_param(IS_CMD_STATE(OnOffState));  

    MCM_REG_UNLOCK(MCMx);

	if (OnOffState != OFF)
	{
		if ((SCx & MCM_SC1) == MCM_SC1)
		{
			MCMx->SC1CON.V32 |= (MCM_SC1 & 0x1000);
		}
		if ((SCx & MCM_SC2) == MCM_SC2)
		{
			MCMx->SC2CON.V32 |= (MCM_SC2 & 0x1000);
		}
		if ((SCx & MCM_SC3) == MCM_SC3)
		{
			MCMx->SC3CON.V32 |= (MCM_SC3 & 0x1000);
		}
	}
	else
	{
		if ((SCx & MCM_SC1) == MCM_SC1)
		{
			MCMx->SC1CON.V32 &= ~(MCM_SC1 & 0x1000);
		}
		if ((SCx & MCM_SC2) == MCM_SC2)
		{
			MCMx->SC2CON.V32 &= ~(MCM_SC2 & 0x1000);
		}
		if ((SCx & MCM_SC3) == MCM_SC3)
		{
			MCMx->SC3CON.V32 &= ~(MCM_SC3 & 0x1000);
		}
	}

    MCM_REG_LOCK(MCMx);
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


