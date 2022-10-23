/**
  ******************************************************************************
  * @file    sh32f2xx_gpio.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provides firmware functions to manage the following
  *          functionalities of the GPIO peripheral:           
  *           - Initialization and Configuration
  *           - GPIO Read and Write
  *           - GPIO Alternate functions configuration

  *  @verbatim
  *
  *          ===================================================================
  *                                  How to use this driver
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
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_gpio.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup GPIO_MODULE GPIO 
  * @brief GPIO driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup GPIO_Private_Functions
  * @{
  */ 

/** @defgroup GPIO_Group1 Initialization and Configuration
 *  @brief    Initialization and Configuration
 *
 *  @verbatim    
 ===============================================================================
                     Initialization and Configuration
 ===============================================================================  

 *  @endverbatim
  * @{
  */



/**
  * @brief  Reset the GPIOx peripheral registers to their default reset values.
  * @note   By default,the GPIO Pins are configured in input floating mode(except JTAG pins).
  * @param  GPIOx: Where x can be A..E to select the GPIO peripheral.
  * @retval None
  */
void GPIO_Reset(GPIO_TypeDef* GPIOx)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    
    GPIOx(CFG)->LCKR.V32 = 0x5AA50000;
    GPIOx->MODER = (uint32_t)0x00;
    GPIOx(CFG)->AFRH.V32 = (uint32_t)0x00;
    GPIOx(CFG)->AFRL.V32 = (uint32_t)0x00;
    GPIOx(CFG)->PUPDR.V32 = (uint32_t)0x00;
    GPIOx(CFG)->OTYPER = (uint32_t)0x00;
    GPIOx(CFG)->ODRVR.V32 = (uint32_t)0x00;
    GPIOx(CFG)->TTLEN.V32 = (uint32_t)0x00;
    GPIOx->ODR = (uint32_t)0x00;
}
    
/**
  * @brief  Initialize the GPIOx peripheral according to the specified parameters in
  *         the GPIO_InitStruct.
  * @param  GPIOx: Where x can be A..E to select the GPIO peripheral.
  * @param  GPIO_InitStruct: Pointer to a @ref GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct) 
{
    uint32_t pinPos = 0x00,    pos = 0x00 , currentPin = 0x00;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));
    assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));
    assert_param(IS_GPIO_PUPD(GPIO_InitStruct->GPIO_PuPd));

    /* Configure the port pins */
    while((GPIO_InitStruct->GPIO_Pin >> pinPos) != 0x00)
    {
        pos = ((uint32_t)0x01) << pinPos;
        /* Get the port pins position */
        currentPin = (GPIO_InitStruct->GPIO_Pin) & pos;
        
        if (currentPin == pos)
        {                                           
            GPIOx->MODER &= ~((uint16_t)GPIO_MODER_MODER0_Msk << (pinPos));            
            GPIOx->MODER |= (((uint32_t)GPIO_InitStruct->GPIO_Mode) << pinPos);

            if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_OUT)
            {
                /* Check the output driver parameters */
                assert_param(IS_GPIO_ODRV(GPIO_InitStruct->GPIO_ODrv));
                               
                /*Output driver configuration */
                GPIOx(CFG)->ODRVR.V32 &= ~((uint16_t)GPIO_CFG_ODRVR_ODRVR0_Msk << (pinPos * 2));
                GPIOx(CFG)->ODRVR.V32 |= ((uint32_t)(GPIO_InitStruct->GPIO_ODrv) << (pinPos * 2));

                /* Output type parameter check */
                assert_param(IS_GPIO_OTYPE(GPIO_InitStruct->GPIO_OType));

                /* Output type configuration */
                GPIOx(CFG)->OTYPER &= ~((uint16_t)GPIO_CFG_OTYPER_OT0_Msk << pinPos);
                GPIOx(CFG)->OTYPER |= (uint16_t)(((uint16_t)GPIO_InitStruct->GPIO_OType) << pinPos);
            }
            
            /* Reset pin AF function to GPIO */
            if (GPIO_InitStruct->GPIO_Mode != GPIO_Mode_AF)
            {
                if (pinPos < 8)
                {
                    GPIOx(CFG)->AFRL.V32 &= ~(GPIO_CFG_AFRL_AFR0_Msk << (pinPos * 4));
                }
                else
                {
                    GPIOx(CFG)->AFRH.V32 &= ~(GPIO_CFG_AFRH_AFR8_Msk << ((pinPos - 8) * 4));
                }
            }

            /* Pull-up Pull-down configuration */
            GPIOx(CFG)->PUPDR.V32 &= ~((uint16_t)GPIO_CFG_PUPDR_PUPDR0_Msk << (pinPos * 2));
            GPIOx(CFG)->PUPDR.V32 |= (((uint32_t)GPIO_InitStruct->GPIO_PuPd) << (pinPos * 2));
        }
        
        pinPos++;
    }
}    

/**
  * @brief  Fills each GPIO_InitStruct member with its default value.
  * @param  GPIO_InitStruct : pointer to a @ref GPIO_InitTypeDef structure which will be initialized.
  * @retval None
  */
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct)
{
    /* Reset GPIO init structure parameters values */
    GPIO_InitStruct->GPIO_Pin  = GPIO_Pin_All;
    GPIO_InitStruct->GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct->GPIO_ODrv = GPIO_ODrv_NORMAL;
    GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_NOPULL;
}

/**
  * @brief  Select whether CMOS or TTL level for GPIO Pins.
  * @param  GPIOx: Where x can be A..E to select the GPIO peripheral.
  *         But only GPIOA and GPIOB are available.
  * @param  GPIO_Pin: Specifie the port bit to be set TTL or CMOS.
  *         This parameter can be any combination of GPIO_Pin_x where x can be 0 to 15. But
  *         only following values are available:
  *           GPIOA:
  *            @arg GPIO_Pin_2
  *            @arg GPIO_Pin_3
  *            @arg GPIO_Pin_8
  *            @arg GPIO_Pin_11
  *          GPIOB:
  *            @arg GPIO_Pin_0
  *            @arg GPIO_Pin_1
  *            @arg GPIO_Pin_4
  *            @arg GPIO_Pin_5            
  * @param  BitLevel: Specifie the level for selected GPIO Pins.
  *          This parameter can be one of the following values:
  *           @arg GPIO_BitLevel_CMOS
  *           @arg GPIO_BitLevel_TTL
  * @retval None
  */
void GPIO_PinTTLConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t GPIO_BitLevel)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_Pin));
    assert_param(IS_GPIO_BIT_LEVEL(GPIO_BitLevel));
    
    if (GPIO_BitLevel != GPIO_BitLevel_CMOS)
    {
        GPIOx(CFG)->TTLEN.V32 |= GPIO_Pin;
    }
    else
    {
        GPIOx(CFG)->TTLEN.V32 &= ~GPIO_Pin;
    }    
}

/**
  * @brief  Lock GPIO pins configuration registers.
  * @param  GPIOx: Where x can be A..E to select the GPIO peripheral.
  * @param  GPIO_Pin: Specifie the port bit to be locked.
  *          This parameter can be any combination of GPIO_Pin_x where x can be 0 to 15.
  * @param  BitLock: Lock or unlock specified port pins.
  *          This parameter can be one of the following values:
  *            @arg GPIO_BitLock_Disable
  *            @arg GPIO_BitLock_Enable
  * @retval None
  */
void GPIO_PinLock(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t GPIO_BitLock)
{
    uint32_t temp = 0x00;
    
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_Pin));
    assert_param(IS_GPIO_BIT_LOCK(GPIO_BitLock));
    
    temp = GPIOx(CFG)->LCKR.V32;
    
    if (GPIO_BitLock != GPIO_BitLock_Disable)
    {
        temp |= GPIO_Pin;
    }
    else
    {
        temp &= ~GPIO_Pin;
    }
    
    GPIOx(CFG)->LCKR.V32 = ((uint32_t)0x5AA5 << 16) | temp;
}

/**
  * @}
  */
  
  
/** @defgroup GPIO_Group2 Read and Write
 *  @brief    Read and Write 
 *
 *  @verbatim   
 ===============================================================================
                               Read and write
 ===============================================================================  

 *  @endverbatim
  * @{
  */
  
/**
  * @brief  Read the specified input port pin.
  * @param  GPIOx: Where x can be A..E to select the GPIO peripheral.
  * @param  GPIO_Pin: specifie the port pin to read.
  *          This parameter can be one of GPIO_Pin_x where x can be 0 to 15.
  * @retval The input port pin value.
  */
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint8_t bitstatus = 0x00;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GET_GPIO_PIN(GPIO_Pin));

    if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)Bit_RESET)
    {
        bitstatus = (uint8_t)Bit_SET;
    }
    else
    {
        bitstatus = (uint8_t)Bit_RESET;
    }
    return bitstatus;
}

/**
  * @brief  Reads the specified GPIO input data port.
  * @param  GPIOx: where x can be A..E to select the GPIO peripheral.
  * @retval GPIO input data port value.
  */
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    return ((uint16_t)GPIOx->IDR); 
}

/**
  * @brief  Read the specified output data port bit.
  * @param  GPIOx: where x can be A..E to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to read.
  *          This parameter can be one of GPIO_Pin_x where x can be 0 to 15.
  * @retval The output port pin value.
  */
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint8_t bitstatus = 0x00;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GET_GPIO_PIN(GPIO_Pin));

    if ((GPIOx->ODR & GPIO_Pin) != (uint32_t)Bit_RESET)
    {
        bitstatus = (uint8_t)Bit_SET;
    }
    else
    {
        bitstatus = (uint8_t)Bit_RESET;
    }
    return bitstatus;
}

/**
  * @brief  Read the specified GPIO output data port.
  * @param  GPIOx: where x can be A..E to select the GPIO peripheral.
  * @retval GPIO output data port value.
  */
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    return ((uint16_t)GPIOx->ODR);
}

/**
  * @brief  Set the selected data port bits.
  * @note   This functions uses GPIOx_BSRR register to allow atomic read/modify 
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  * @param  GPIOx: where x can be A..E to select the GPIO peripheral.
  * @param  GPIO_Pin: specifie the port bits to be written.
  *          This parameter can be any combination of GPIO_Pin_x where x can be 0 to 15.
  * @retval None
  */
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_Pin));

    GPIOx->BSRR.V32 = GPIO_Pin;
}

/**
  * @brief  Reset the selected data port bits.
  * @param  GPIOx: where x can be A..E to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bits to be written.
  *          This parameter can be any combination of GPIO_Pin_x where x can be 0 to 15.
  * @retval None
  */
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_Pin));

    GPIOx->BSRR.V32 = (uint32_t)GPIO_Pin << 16;
}

/**
  * @brief  Set or clear the selected data port bit.
  * @param  GPIOx: where x can be A..E to select the GPIO peripheral.
  * @param  GPIO_Pin: specifie the port bit to be written.
  *          This parameter can be one of GPIO_Pin_x where x can be 0 to 15.
  * @param  BitVal: specifie the value to be written to the selected bit.
  *          This parameter can be one of the BitAction enum values:
  *            @arg Bit_RESET: to clear the port pin
  *            @arg Bit_SET: to set the port pin
  * @retval None
  */
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
    assert_param(IS_GPIO_BIT_ACTION(BitVal));

    if (BitVal != Bit_RESET)
    {
        GPIOx->BSRR.V32 = GPIO_Pin;
    }
    else
    {
        GPIOx->BSRR.V32 = (uint32_t)GPIO_Pin << 16;
    }
}

/**
  * @brief  Write data to selected GPIO data port.
  * @param  GPIOx: where x can be A..E to select the GPIO peripheral.
  * @param  PortVal: specifie the value to be written to the port output data register.
  * @retval None
  */  
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx)); 

    GPIOx->ODR = PortVal;
}

/**
  * @brief  Toggle the specified GPIO pins.
  * @param  GPIOx: where x can be A..E to select the GPIO peripheral.
  * @param  GPIO_Pin: specifie the pins to be toggled.
  *          This parameter can be any combination of GPIO_Pin_x where x can be 0 to 15.
  * @retval None
  */
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    GPIOx->ODR ^= GPIO_Pin;
}
    
  
/**
  * @}
  */



/** @defgroup GPIO_Group3 Alternate function configuration
 *  @brief    Alternate function configuration 
 *
 *  @verbatim   
 ===============================================================================
                               Alternate function configuration
 ===============================================================================  

 *  @endverbatim
  * @{
  */


/**
  * @brief  Changes the mapping of the specified pin.
  * @param  GPIOx: where x can be A..E to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the pin for the alternate function.
  *         This parameter can be one of GPIO_Pin_x where x can be (0..15).
  * @param  GPIO_AF: selects the pin to used as Alternate function.
  *          This parameter can be one of the following values:
  *            @arg GPIO_AF_GPIO: Connect Pins to GPIO function
  *            @arg GPIO_AF_MCM1: Connect MCM1 pins to AF1 
  *            @arg GPIO_AF_MCM2: Connect MCM2 pins to AF2
  *            @arg GPIO_AF_GPT0: Connect GPT0 pins to AF3
  *            @arg GPIO_AF_GPT1: Connect GPT1 pins to AF3
  *            @arg GPIO_AF_GPT2: Connect GPT2 pins to AF3
  *            @arg GPIO_AF_GPTOE: Connect GPTOE pins to AF3
  *            @arg GPIO_AF_GPT3: Connect GPT3 pins to AF4
  *            @arg GPIO_AF_GPTETRG: Connect GPTETRG pins to AF4
  *            @arg GPIO_AF_TIM5: Connect TIM5 pins to AF5
  *            @arg GPIO_AF_TIM6: Connect TIM6 pins to AF5
  *            @arg GPIO_AF_TIM7: Connect TIM7 pins to AF5
  *            @arg GPIO_AF_TIM8: Connect TIM8 pins to AF5
  *            @arg GPIO_AF_QEI: Connect QEI pins to AF6
  *            @arg GPIO_AF_UART1: Connect UART1 pins to AF7
  *            @arg GPIO_AF_UART2: Connect UART2 pins to AF7
  *            @arg GPIO_AF_UART3: Connect UART3 pins to AF7
  *            @arg GPIO_AF_SPI1: Connect SPI1 pins to AF8
  *            @arg GPIO_AF_SPI2: Connect SPI2 pins to AF8
  *            @arg GPIO_AF_TWI1: Connect TWI1 pins to AF9
  *            @arg GPIO_AF_ADTRG: Connect ADTRG pins to AF10
  *            @arg GPIO_AF_ADC1: Connect ADC1 pins to AF15
  *            @arg GPIO_AF_ADC2: Connect ADC1 pins to AF15
  *            @arg GPIO_AF_ADC3: Connect ADC1 pins to AF15
  *            @arg GPIO_AF_OP1: Connect OP1 pins to AF15
  *            @arg GPIO_AF_OP2: Connect OP1 pins to AF15
  *            @arg GPIO_AF_OP3: Connect OP1 pins to AF15
  *            @arg GPIO_AF_CMP1: Connect CMP1 pins to AF15
  *            @arg GPIO_AF_CMP2: Connect CMP1 pins to AF15
  *            @arg GPIO_AF_CMP3: Connect CMP1 pins to AF15
  * @retval None
  */
void GPIO_PinAFConfig(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint8_t GPIO_AF)
{
    uint8_t pinPos = 0;
    uint16_t currentPin = 0;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
    assert_param(IS_GPIO_AF(GPIO_AF));

    /* Get the current GPIO Pin position */
    for (pinPos=0;pinPos<16;pinPos++)
    {
        currentPin = (uint16_t)0x01 << pinPos;
        if ((GPIO_Pin & currentPin) == currentPin)
        {
            break;
        }
    }
    
    if (pinPos > 0x07)
    {
        GPIOx(CFG)->AFRH.V32 &= (~((uint32_t)0x07 << ((pinPos & 0x07) * 4)));
        GPIOx(CFG)->AFRH.V32 |= ((uint32_t)GPIO_AF << ((pinPos & 0x07) * 4));
    }
    else
    {
        GPIOx(CFG)->AFRL.V32 &= (~((uint32_t)0x07 << (pinPos * 4)));
        GPIOx(CFG)->AFRL.V32 |= ((uint32_t)GPIO_AF << (pinPos * 4));
    }
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
