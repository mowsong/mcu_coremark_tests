/**
  ******************************************************************************
  * @file    sh32f2xx_rcc.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provides reset and clock module's APIs
  *         
  *  @verbatim
  *
  *          ===================================================================
  *                                   How to use this driver
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
#include "sh32f2xx_rcc.h"
#ifndef HSI_CLK
#define HSI_CLK 8000000
#endif
#ifndef HSE_CLK
#define HSE_CLK 8000000
#endif
      
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/*  reset and clock control module-----------------------------------------------------------*/
/** @defgroup RCC_MODULE  RCC 
   *  reset and clock control module(RCC)
  * @{
  */ 
        
/** @defgroup  RCC_Group_Pub_Funcs  Public Functions
 *  @brief   RCC public functions
 *
  * @{
  */
/**
  *@code   Example    
  *     RCC_ClocksTypeDef rccClocks;
  *     RCC_GetClocksFreq(& rccClocks);
   *@endcode
   */
/**
  * @brief  get current system's clock frequency
  * @param  Clocks: output clock frequency
  * @retval None
  */
void RCC_GetClocksFreq(RCC_Clocks_TypeDef* Clocks)
{
        uint32_t sysClk=0;
        assert_param(IS_IN_SRAM(Clocks) || IS_IN_CRAM(Clocks));
        if(RCC->CR.BIT.SW == RCC_SYS_SRC_HSI)
        {/*  clock source is HSI */
                sysClk = HSI_CLK;
        }
        else if(RCC->CR.BIT.SW == RCC_SYS_SRC_HSE)
        {/*  clock source is HSE */
                sysClk = HSE_CLK;
        }
        else if(RCC->CR.BIT.SW == RCC_SYS_SRC_PLL)
        {/*  clock source is PLL */
              if(RCC->CFGR.BIT.PLLSRC == PLL_SRC_HSI)  
              {
                    sysClk = HSI_CLK;
              }
              else if(RCC->CFGR.BIT.PLLSRC == PLL_SRC_HSE)
              {
                    sysClk = HSE_CLK;
              }
              else
              {
                  assert_param(0);
              }
              sysClk /= (RCC->CFGR.BIT.PLLXTPRE+1);
              sysClk *= (RCC->CFGR.BIT.PLLF+15);
              sysClk /= (RCC->CFGR.BIT.PLLK+1);
              /* fixed divider 2 */
              sysClk >>=1; 
        }
        else
        {
            assert_param(0);
        }
        
         Clocks->sysFreq = sysClk;
         sysClk >>= (RCC->CFGR.BIT.HPRE > 5 ? 5 : RCC->CFGR.BIT.HPRE);
         Clocks->hclkFreq = sysClk;
         Clocks->pclk1Freq= sysClk >> (RCC->CFGR.BIT.PPRE1<4 ? 0 : (RCC->CFGR.BIT.PPRE1-3));
         Clocks->pclk2Freq= sysClk >> (RCC->CFGR.BIT.PPRE2<4 ? 0 : (RCC->CFGR.BIT.PPRE2-3));
}
  
/**
  * @brief    restore default clocks
  * @retval None
  */
void RCC_Reset(void)
{
    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();
    
    /* switch to  HSI*/
    RCC->CR.BIT.SW = RCC_SYS_SRC_HSI;
    
    /* wait switch ready */
    while(RCC->CR.BIT.SW != RCC_SYS_SRC_HSI);
    
    /*  close HSE and PLL */
    RCC->CR.V32 = 0;
    
    /*  clear  CFGR  */
    RCC->CFGR.V32 =  0; 

    /*  clear interrupt enable bits */
    RCC->CIENR.V32 = 0;
    
    /* clear interrupt flags */
    RCC->CICLR.V32 = 0xFFFFFFFF;

    /* clear clock gates */
    RCC->AHBENR.V32  = 0;
    RCC->APB1ENR.V32 = 0;
    RCC->APB2ENR.V32 = 0;

    /* clear reset flags */
    RCC->RSTCLR.V32  = 0xFFFFFFFF;

    /* clear RC Trim */
    RCC->HSICAL.V32  = 0;
    
   /* Lock RCC registers */
    RCC_REGS_LOCK();
}

/**
  * @brief    configure HSE (external clock)
  * @param OnOffState   open or close the HSE
  *     @arg @b ON    open HSE
  *     @arg @b OFF   close HSE
  * @retval None
  */
void RCC_HSEOnOff(CmdState OnOffState)
{
    assert_param(IS_CMD_STATE(OnOffState));
       
    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();
    
    RCC->CR.BIT.HSEON = OnOffState;

   /* Lock RCC registers */
    RCC_REGS_LOCK();
}

/**
  * @brief    wait for HSE ready
  * @param TimeOut   timeout ticks
  * @retval ErrorStatus 
  *      @arg @b SUCCESS  HSE is ready 
  *      @arg @b ERROR     timeout
  */
ErrorStatus RCC_WaitForHSEReady(uint32_t TimeOut)
{
        assert_param(TimeOut > 0);
        do{
            if(RCC->CR.BIT.HSERDY)
                break;
            TimeOut--;
        }while(TimeOut > 0);
        return (TimeOut==0?ERROR:SUCCESS);
}

/**
  * @code   Example : HSI increase 0.25%
       RCC_AdjustHSITrimValue(RCC_HSITRIM_I25);    
  * @endcode 
  *@brief   adjust HSI frequency
  *@param HSITrimValue  trim value
  *     @arg @b RCC_HSITRIM_I00  ( + 0.0% )
  *     @arg @b RCC_HSITRIM_I25  ( + 0.25% )
  *     @arg @b RCC_HSITRIM_I50  ( + 0.50% )
  *     @arg @b RCC_HSITRIM_I75  ( + 0.75% )
  *     @arg @b RCC_HSITRIM_D100 (- 1% )
  *     @arg @b RCC_HSITRIM_D75  (- 0.75%  )
  *     @arg @b RCC_HSITRIM_D50  (- 0.50% )
  *     @arg @b RCC_HSITRIM_D25  (- 0.25% )
  *@retval  Trim reference value
  */
uint32_t RCC_AdjustHSITrimValue(RCC_HSITRIM_Type HSITrimValue)
{
    assert_param(IS_HSITRIM(HSITrimValue));
    
    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();

    /* set trim value*/
    RCC->HSICAL.BIT.HSITRIM = HSITrimValue;
    
    /* start  */
    RCC_HSICAL_TRIMRUN_BIT = SET;
    
    while(RCC_HSICAL_TRIMRUN_BIT == SET);
    
   /* Lock RCC registers */
    RCC_REGS_LOCK();    
	
		return RCC->HSICAL.BIT.TRIMREF;
}


/**
  * @code   Example
       // config PLL£¬HSI is 8MHz and PLL's output is 100MHz
       // 1. PLL source is HSI     ;can be PLL_SRC_HSI or PLL_SRC_HSE
       // 2. PLL pre-divider is 1  ;can be PLL_XTPRE_DIV1 or PLL_XTPRE_DIV2
       // 3. PLL F = 25            ;can be 15~78
       // 4. PLL K = 1             ;can be 1~16
       // 5. PLL output = HSI(8MHz)/pre-divider(1) * F(25)/K(1)/Fixed(2) = 100MHz
       RCC_PllInitTypeDef pllInit;
       pllInit.pllSRC    = PLL_SRC_HSI;  
       pllInit.xpreDiv   = PLL_XTPRE_DIV1; 
       pllInit.pllMul    = 25; 
       pllInit.pllDiv    = 1;  
  * @endcode 
  */
/**
  * @brief             configure PLL
  * @param PLLInit  PLL parameters @ref RCC_PLL_InitTypeDef
  * @retval None
  */
void RCC_PLLConfig(RCC_PLL_InitTypeDef* PLLInit)
{
    assert_param(IS_PLL_XTPRE(PLLInit->xpreDiv));
    assert_param(IS_PLL_SRC(PLLInit->pllSRC));
    assert_param((PLLInit->pllMul >= 15 && PLLInit->pllMul <= 78));
    assert_param((PLLInit->pllDiv >= 1 && PLLInit->pllDiv <= 16));
    
    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();

    RCC->CFGR.BIT.PLLXTPRE = PLLInit->xpreDiv;
    RCC->CFGR.BIT.PLLSRC   = PLLInit->pllSRC;
    RCC->CFGR.BIT.PLLF     = PLL_PLLF(PLLInit->pllMul);
    RCC->CFGR.BIT.PLLK     = PLL_PLLK(PLLInit->pllDiv);
    
   /* Lock RCC registers */
    RCC_REGS_LOCK();           
}

/**
  *@brief   open or close PLL
  *@param OnOffState  open or close PLL
  *     @arg @b ON   open PLL
  *     @arg @b OFF  close PLL
  *@retval None
  */
void RCC_PLLOnOff(CmdState OnOffState)
{
    assert_param(IS_CMD_STATE(OnOffState));

    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();

    RCC->CR.BIT.PLLON = OnOffState;
    
   /* Lock RCC registers */
    RCC_REGS_LOCK();        
}

/**
  * @brief    wait for PLL ready
  * @param    TimeOut   timeout ticks
  * @retval   ErrorStatus 
  *      @arg @b SUCCESS  PLL is ready 
  *      @arg @b ERROR     timeout
  */
ErrorStatus RCC_WaitForPLLReady(uint32_t TimeOut)
{
    assert_param(TimeOut > 0);
    do{
        if(RCC->CR.BIT.PLLRDY)
            break;
        TimeOut--;
    }while(TimeOut > 0);
    return (TimeOut==0?ERROR:SUCCESS);
}

/**
  *@brief    configure system clock
  *@param SysClkSrc  assign system clock source
  *     @arg @b RCC_SYS_SRC_HSI   
  *     @arg @b RCC_SYS_SRC_HSE  
  *     @arg @b RCC_SYS_SRC_PLL   
  * @retval None
  */
void RCC_SysClkConfig(RCC_SysSource_Type SysClkSrc)
{    
    uint8_t tmpSysClk;
    assert_param(IS_SYS_SOURCE(SysClkSrc));
    
    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();
    
    RCC->CR.BIT.SW = SysClkSrc;
    __DSB();
    
    /* wait switching ready */
    do{
        tmpSysClk = RCC->CR.BIT.SWS;
    }
    while(RCC->CR.BIT.SW != tmpSysClk );
 
    /* Lock RCC registers */
    RCC_REGS_LOCK();            
}

/**
  * @brief    get current system clock source 
  * @retval RCC_SysSource_Type current system clock source 
  *     @arg @b RCC_SYS_SRC_HSI  
  *     @arg @b RCC_SYS_SRC_HSE 
  *     @arg @b RCC_SYS_SRC_PLL  
  */
RCC_SysSource_Type RCC_GetSysClkSource(void)
{
    return ((RCC_SysSource_Type)RCC->CR.BIT.SWS);
}

/**
  *@brief    configure HCLK
  *@param  HCLK_DIV  system clock's divider
  *     @arg @b  RCC_HCLK_DIV1   system clock / 1
  *     @arg @b  RCC_HCLK_DIV2   system clock / 2
  *     @arg @b  RCC_HCLK_DIV4   system clock / 4
  *     @arg @b  RCC_HCLK_DIV8   system clock / 8
  *     @arg @b  RCC_HCLK_DIV16  system clock / 16
  *     @arg @b  RCC_HCLK_DIV32  system clock / 32
  *@retval  None
  */
void RCC_HCLKConfig(RCC_HCLK_DIV_Type HCLK_DIV)
{
    assert_param(IS_HCLK_DIV(HCLK_DIV));

    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();
    
    RCC->CFGR.BIT.HPRE = HCLK_DIV;
    
    /* Lock RCC registers */
    RCC_REGS_LOCK();               
}

/**
  *@brief    configure PCLK1
  *@param  PCLK1_DIV  HCLK's divider
  *     @arg @b  RCC_PCLK1_DIV1    HCLK / 1
  *     @arg @b  RCC_PCLK1_DIV2   HCLK / 2
  *     @arg @b  RCC_PCLK1_DIV4   HCLK / 4
  *     @arg @b  RCC_PCLK1_DIV8   HCLK / 8
  *     @arg @b  RCC_PCLK1_DIV16  HCLK / 16
  *@retval  None
  */
void RCC_PCLK1Config(RCC_PCLK1_DIV_Type PCLK1_DIV)
{
    assert_param(IS_PCLK1_DIV(PCLK1_DIV));
   
    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();
    
    RCC->CFGR.BIT.PPRE1 = PCLK1_DIV;
    
    /* Lock RCC registers */
    RCC_REGS_LOCK();               
}

/**
 * @brief    configure PCLK2
  *@param  PCLK2_DIV  HCLK's divider
  *   This parameter can be one of following values:
  *     @arg @b  RCC_PCLK2_DIV1    HCLK / 1
  *     @arg @b  RCC_PCLK2_DIV2   HCLK / 2
  *     @arg @b  RCC_PCLK2_DIV4   HCLK / 4
  *     @arg @b  RCC_PCLK2_DIV8   HCLK / 8
  *     @arg @b  RCC_PCLK2_DIV16  HCLK / 16
  *@retval  None
   */
void RCC_PCLK2Config(RCC_PCLK2_DIV_Type PCLK2_DIV)
{
    assert_param(IS_PCLK2_DIV(PCLK2_DIV));

    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();
    
    RCC->CFGR.BIT.PPRE2 = PCLK2_DIV;
    
    /* Lock RCC registers */
    RCC_REGS_LOCK();               
}

/**
 * @brief    configure interrupt enable or disable
  *@param  INT_SRC  interrupt source: can be one or more.
  *   This parameter can be any combination of following values:
  *     @arg @b  RCC_INT_HSERDY: HSE Ready Interrupt Enable
  *     @arg @b  RCC_INT_PLLRDY: PLL Ready Interrupt Enable   
  *@param  NewState enable or disable interrupt
  *     @arg @b  ENABLE  
  *     @arg @b  DISABLE
  *@retval  None
   */
void RCC_INTConfig(uint32_t INT_SRC, FunctionalState NewState)
{
    assert_param(IS_RCC_INTSRCS(INT_SRC));
    assert_param(IS_FUNCTION_STATE(NewState));
    
    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();
    
     if((INT_SRC & RCC_INT_HSERDY) == RCC_INT_HSERDY)
            RCC->CIENR.BIT.HSERDYIE = NewState;
     
     if((INT_SRC & RCC_INT_PLLRDY) == RCC_INT_PLLRDY)
            RCC->CIENR.BIT.PLLRDYIE = NewState;
   
    /* Lock RCC registers */
    RCC_REGS_LOCK();         

}


/**
 * @brief    Get RCC interrupt flag
  *@param  INTFlag  interrupt flag
  *   This parameter can be one of following values:
  *     @arg @b  RCC_INT_HSERDY:  HSE ready interrupt flag
  *     @arg @b  RCC_INT_PLLRDY   PLL ready interrupt flag
  *     @arg @b  RCC_INT_CSMHSE:  HSE clock abnormal flag
  *     @arg @b  RCC_INT_CSMPLL:  PLL clock abnormal flag
  *@retval  FlagStatus RCC Interrupt status
  *     @arg @b  SET    the flag is setted
  *     @arg @b  RESET  the flag is cleared
  */
FlagStatus  RCC_GetINTStatus(RCC_INT_Type INTFlag)
{
        assert_param(IS_RCC_INTFLAG(INTFlag));
        
        return ((FlagStatus)((RCC->CISTR.V32 & INTFlag) ? SET : RESET));
}


/**
 * @brief  clear RCC interrupt flag
  *@param  INTFlag  interrupt source, can select more than one
  *   This parameter can be any combination of following values:
  *     @arg @b  RCC_INT_HSERDY   HSE ready interrupt flag
  *     @arg @b  RCC_INT_PLLRDY   PLL ready interrupt flag
  *     @arg @b  RCC_INT_CSMHSE   CSM  abnormal flag
  *     @arg @b  RCC_INT_CSMPLL   CSM  abnormal flag
  *@retval  None
   */
void  RCC_ClearINTStatus(uint32_t INTFlag)
{
    assert_param(IS_RCC_INTFLAGS(INTFlag));

    /* Unlock RCC registers */
    RCC_REGS_UNLOCK();

    if((INTFlag & RCC_INT_CSMHSE) == RCC_INT_CSMHSE)
        INTFlag |= RCC_INT_CSMPLL;        
    
    RCC->CICLR.V32 = INTFlag;    
    
    /* Lock RCC registers */
    RCC_REGS_LOCK();            
}

    
/**
  * @code   Example : open ADC1 and ADC2 module's clock
        RCC_AHBPeriphClockOnOff(RCC_AHB_ADC1 | RCC_AHB_ADC2, ON);
  * @endcode 
  */
/**
  * @brief     open or close AHB modules' clock gate
  * @param  AHBModules:  AHB module bits @ref RCC_AHB_Type
  *   This parameter can be any combination of following values:
  *     @arg @b  RCC_AHB_GPIO:     for GPIO Module: Includes Config and IO
  *     @arg @b  RCC_AHB_GPIO_CFG: for GPIO Config
  *     @arg @b  RCC_AHB_GPIO_IOD: for GPIO Input and Output
  *     @arg @b  RCC_AHB_ADC1: for ADC1 Module
  *     @arg @b  RCC_AHB_ADC2: for ADC2 Module
  *     @arg @b  RCC_AHB_ADC3: for ADC3 Module
  *     @arg @b  RCC_AHB_SYSCFG: for System Config Module
  *     @arg @b  RCC_AHB_DMA:  for DMA Module
  *     @arg @b  RCC_AHB_MACP: for MACP Module
  *     @arg @b  RCC_AHB_CRC:  for CRC Module
  *     @arg @b  RCC_AHB_GPT0: for GPT0 Module
  *     @arg @b  RCC_AHB_GPT1: for GPT1 Module
  *     @arg @b  RCC_AHB_GPT2: for GPT2 Module
  *     @arg @b  RCC_AHB_GPT3: for GPT3 Module
  *     @arg @b  RCC_AHB_GPT:  for GPT Common Module
  *@param OnOffState  open or close related clock gate
  *     @arg @b  ON  open related clock gate
  *     @arg @b  OFF  close related clock gate
  * @retval   None
   */ 
void  RCC_AHBPeriphClockOnOff(uint32_t AHBModules,CmdState OnOffState)
{
    assert_param(IS_AHB_MODULES(AHBModules));
    assert_param(IS_CMD_STATE(OnOffState));
    
    /* Unlock RCC registers */
	RCC_REGS_UNLOCK();

    if(OnOffState == ON)
    {
        RCC->AHBENR.V32 |= AHBModules;	
    }
    else
    {   
        uint32_t tmpV = RCC->AHBENR.V32;
        tmpV &= ~AHBModules;
     //   printf("tmpV:%X\n",tmpV);
        if(AHBModules & (RCC_AHB_GPT0 | RCC_AHB_GPT1 | RCC_AHB_GPT2 | RCC_AHB_GPT3 ))
        {
            if(tmpV & (RCC_AHB_GPT0 | RCC_AHB_GPT1 | RCC_AHB_GPT2 | RCC_AHB_GPT3 ))
            {
                tmpV |= RCC_AHB_GPT;            
           //     printf("tmpV:%X\n",tmpV);
            }
        }
        RCC->AHBENR.V32 = tmpV;	
    }

   /* Lock RCC registers */
	RCC_REGS_LOCK();   
}


/**
  * @brief  open or close APB1 modules' clock gate
  * @param  APB1Modules:  APB1 module bits @ref RCC_APB1_Type
  *   This parameter can be any combination of following values:
  *     @arg @b  RCC_APB1_TIM5
  *     @arg @b  RCC_APB1_TIM6
  *     @arg @b  RCC_APB1_TIM7
  *     @arg @b  RCC_APB1_TIM8
  *     @arg @b  RCC_APB1_QEI
  *     @arg @b  RCC_APB1_UART1
  *     @arg @b  RCC_APB1_UART2
  *     @arg @b  RCC_APB1_UART3
  *     @arg @b  RCC_APB1_SPI1
  *     @arg @b  RCC_APB1_SPI2
  *     @arg @b  RCC_APB1_TWI1
  *     @arg @b  RCC_APB1_WWDT
  *     @arg @b  RCC_APB1_AMOC
  *@param OnOffState  open or close related clock gate
  *     @arg @b  ON  open related clock gate
  *     @arg @b  OFF  close related clock gate
  * @retval  None
  */
void  RCC_APB1PeriphClockOnOff(uint32_t APB1Modules,CmdState OnOffState)
{
    assert_param(IS_APB1_MODULES(APB1Modules));
    assert_param(IS_CMD_STATE(OnOffState));
    
    /* Unlock RCC registers */
 	RCC_REGS_UNLOCK();

    if(OnOffState == ON)
    {
        RCC->APB1ENR.V32 |= APB1Modules;	
    }
    else
    {
        RCC->APB1ENR.V32 &= ~APB1Modules;	
    }

   /* Lock RCC registers */
	RCC_REGS_LOCK();   
}


/**
  * @brief      open or close APB2 modules' clock gate
  * @param  APB2Modules:  APB2 module bits @ref RCC_APB2_Type
  *   This parameter can be any combination of following values:
  *     @arg @b  RCC_APB2_MCM1
  *     @arg @b  RCC_APB2_MCM2
  *@param OnOffState  open or close related clock gate
  *     @arg @b  ON  open related clock gate
  *     @arg @b  OFF  close related clock gate
  * @retval None
  */
void  RCC_APB2PeriphClockOnOff(uint32_t APB2Modules,CmdState OnOffState)
{
    assert_param(IS_APB2_MODULES(APB2Modules));
    assert_param(IS_CMD_STATE(OnOffState));
    
    /* Unlock RCC registers */
 	RCC_REGS_UNLOCK();
    
    if(OnOffState == ON)
    {
        RCC->APB2ENR.V32 |= APB2Modules;	
    }
    else
    {
        RCC->APB2ENR.V32 &= ~APB2Modules;	
    }
    
   /* Lock RCC registers */
	RCC_REGS_LOCK();   
}    



/**
  * @code   Example : reset ADC1 and ADC2 module
        RCC_AHBPeriphReset(RCC_AHB_ADC1|RCC_AHB_ADC2);
  * @endcode 
  */
/**
  * @brief     reset AHB modules
  * @param  AHBModules: AHB module bits @ref RCC_AHB_Type
  *   This parameter can be any combination of following values:
  *     @arg @b  RCC_AHB_GPIO    : for GPIO Module. Includes Config and IO
  *     @arg @b  RCC_AHB_GPIO_CFG: for GPIO Config
  *     @arg @b  RCC_AHB_GPIO_IOD: for GPIO Input and Output
  *     @arg @b  RCC_AHB_ADC1: for ADC1 Module
  *     @arg @b  RCC_AHB_ADC2: for ADC2 Module
  *     @arg @b  RCC_AHB_ADC3: for ADC3 Module
  *     @arg @b  RCC_AHB_SYSCFG: for System Config Module
  *     @arg @b  RCC_AHB_DMA:  for DMA Module
  *     @arg @b  RCC_AHB_MACP: for MACP Module
  *     @arg @b  RCC_AHB_CRC:  for CRC Module
  *     @arg @b  RCC_AHB_GPT0: for GPT0 Module
  *     @arg @b  RCC_AHB_GPT1: for GPT1 Module
  *     @arg @b  RCC_AHB_GPT2: for GPT2 Module
  *     @arg @b  RCC_AHB_GPT3: for GPT3 Module
  *     @arg @b  RCC_AHB_GPT:  for GPT Common Module
  * @retval   None
   */ 
void RCC_AHBPeriphReset(uint32_t AHBModules)
{
    assert_param(IS_AHB_MODULES(AHBModules));

    /* Unlock RCC registers */
 	RCC_REGS_UNLOCK();
    
    if(AHBModules & (RCC_AHB_GPT0 | RCC_AHB_GPT1 | RCC_AHB_GPT2 | RCC_AHB_GPT3 ))
    {
        uint32_t tmpV = RCC->AHBENR.V32;
        tmpV &= ~AHBModules;
        if(tmpV & (RCC_AHB_GPT0 | RCC_AHB_GPT1 | RCC_AHB_GPT2 | RCC_AHB_GPT3 ))
            AHBModules &= ~RCC_AHB_GPT;
    }  
    RCC->AHBRSTR.V32 = AHBModules;
    
   /* Lock RCC registers */
	RCC_REGS_LOCK();   
}


/**
  * @brief    reset APB1 modules
  * @param  APB1Modules:  APB1 module bits @ref RCC_APB1_Type
  *   This parameter can be any combination of following values:
  *     @arg @b  RCC_APB1_TIM5
  *     @arg @b  RCC_APB1_TIM6
  *     @arg @b  RCC_APB1_TIM7
  *     @arg @b  RCC_APB1_TIM8
  *     @arg @b  RCC_APB1_QEI
  *     @arg @b  RCC_APB1_UART1
  *     @arg @b  RCC_APB1_UART2
  *     @arg @b  RCC_APB1_UART3
  *     @arg @b  RCC_APB1_SPI1
  *     @arg @b  RCC_APB1_SPI2
  *     @arg @b  RCC_APB1_TWI1
  *     @arg @b  RCC_APB1_WWDT
  *     @arg @b  RCC_APB1_AMOC
  * @retval  None
  */
void RCC_APB1PeriphReset(uint32_t APB1Modules)
{
    assert_param(IS_APB1_MODULES(APB1Modules));

    /* Unlock RCC registers */
 	RCC_REGS_UNLOCK();

    RCC->APB1RSTR.V32 = APB1Modules;
        
   /* Lock RCC registers */
	RCC_REGS_LOCK();       
}

/**
  * @brief     reset APB2 modules
  * @param  APB2Modules:  APB2 module bits @ref RCC_APB2_Type
  *   This parameter can be any combination of following values:
  *     @arg @b  RCC_APB2_MCM1
  *     @arg @b  RCC_APB2_MCM2
  * @retval None
  */
void RCC_APB2PeriphReset(uint32_t APB2Modules)
{
    assert_param(IS_APB2_MODULES(APB2Modules));

    /* Unlock RCC registers */
 	RCC_REGS_UNLOCK();

    RCC->APB2RSTR.V32 = APB2Modules;
        
   /* Lock RCC registers */
	RCC_REGS_LOCK();           
}



/**
  * @brief     Get RCC reset flag
  * @param  ResetFlag:   RCC reset flag mask
  *     @arg @b  RCC_RST_PIN
  *     @arg @b  RCC_RST_LVR
  *     @arg @b  RCC_RST_POWERON
  *     @arg @b  RCC_RST_SOFTWARE
  *     @arg @b  RCC_RST_IWDT
  *     @arg @b  RCC_RST_WWDT
  *     @arg @b  RCC_RST_VCCLVR
  * @retval  FlagStatus RCC reset flag
  *     @arg @b  SET    the flag is setted
  *     @arg @b  RESET  the flag is cleared
  */
FlagStatus RCC_GetResetFlag(RCC_RESET_Type ResetFlag)
{
    assert_param(IS_RCC_RESET_Type(ResetFlag));

    return ((RCC->RSTSTR.V32 & ResetFlag) ? SET : RESET);
}


/**
  * @brief      clear RCC reset flags
  * @param  ResetFlags:   RCC reset flag clear bits
  *     @arg @b  RCC_RST_PIN
  *     @arg @b  RCC_RST_LVR
  *     @arg @b  RCC_RST_POWERON
  *     @arg @b  RCC_RST_SOFTWARE
  *     @arg @b  RCC_RST_IWDT
  *     @arg @b  RCC_RST_WWDT
  *     @arg @b  RCC_RST_VCCLVR
   * @retval None
  */
void RCC_ClearResetFlag(uint8_t ResetFlags)
{
    assert_param(IS_RCC_RESET_Types(ResetFlags));

    /* Unlock RCC registers */
 	RCC_REGS_UNLOCK();

    RCC->RSTCLR.V32 = ResetFlags;
        
   /* Lock RCC registers */
	RCC_REGS_LOCK();           
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


