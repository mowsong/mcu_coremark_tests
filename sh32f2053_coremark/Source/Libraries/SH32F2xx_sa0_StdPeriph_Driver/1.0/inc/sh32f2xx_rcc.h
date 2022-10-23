/**
  ******************************************************************************
  * @file    sh32f2xx_rcc.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provides reset and clock module's APIs
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SH32F2xx_RCC_H
#define __SH32F2xx_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"
     
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup RCC_MODULE
  * @{
  */ 

/** @defgroup RCC_Group_Constant  Public Constants
  * @{
  */ 
/*!< CLOCK SOURCE CONSTANT */
typedef enum {
    RCC_SYS_SRC_HSI  = 0,
    RCC_SYS_SRC_HSE = 1,
    RCC_SYS_SRC_PLL  = 2,
}RCC_SysSource_Type;
/*!  check clock source */
#define IS_SYS_SOURCE(src) (((src) == RCC_SYS_SRC_HSI) || ((src) == RCC_SYS_SRC_HSE) || ((src) == RCC_SYS_SRC_PLL))

/*!  PLL XTPRE CLOCK CONSTANT */
typedef enum {
    PLL_XTPRE_DIV1  = 0,
    PLL_XTPRE_DIV2  = 1,
}RCC_PLL_XTPRE_Type;
/*!  check pll 's pre-divider */
#define IS_PLL_XTPRE(div) (((div) == PLL_XTPRE_DIV1) || ((div) == PLL_XTPRE_DIV2))

/*!  PLL SOURCE CONSTANT */
typedef enum {
   PLL_SRC_HSI   = 0,
   PLL_SRC_HSE  = 1,
}RCC_PLL_SRC_Type;
/*!  check pll's input source */
#define IS_PLL_SRC(src) (((src) == PLL_SRC_HSI) || ((src) == PLL_SRC_HSE))


/*! RCC AHB CLOCK CONSTANT */
typedef enum {
    RCC_HCLK_DIV1  =  0,
    RCC_HCLK_DIV2  =  1,
    RCC_HCLK_DIV4  =  2,
    RCC_HCLK_DIV8  =  3,
    RCC_HCLK_DIV16 =  4,
    RCC_HCLK_DIV32 =  5,
}RCC_HCLK_DIV_Type;
/*!  check AHB frequence's divider value */
#define IS_HCLK_DIV(div)   (((div) == RCC_HCLK_DIV1)  \
                                      || ((div) == RCC_HCLK_DIV2)  \
                                      || ((div) == RCC_HCLK_DIV4)  \
                                      || ((div) == RCC_HCLK_DIV8)  \
                                      || ((div) == RCC_HCLK_DIV16) \
                                      || ((div) == RCC_HCLK_DIV32))


/*! RCC APB1 CLOCK CONSTANT */
typedef enum {
    RCC_PCLK1_DIV1   = 0,
    RCC_PCLK1_DIV2   = 4,
    RCC_PCLK1_DIV4   = 5,
    RCC_PCLK1_DIV8   = 6,
    RCC_PCLK1_DIV16  = 7,
}RCC_PCLK1_DIV_Type;
/*!  check APB1 frequence's divider value */
#define IS_PCLK1_DIV(div)   (((div) == RCC_PCLK1_DIV1)  \
                                       || ((div) == RCC_PCLK1_DIV2)  \
                                       || ((div) == RCC_PCLK1_DIV4)  \
                                       || ((div) == RCC_PCLK1_DIV8)  \
                                       || ((div) == RCC_PCLK1_DIV16)) 

/*!  RCC APB2 CLOCK CONSTANT */
typedef enum {
    RCC_PCLK2_DIV1  =  0,
    RCC_PCLK2_DIV2  =  4,
    RCC_PCLK2_DIV4  =  5,
    RCC_PCLK2_DIV8  =  6,
    RCC_PCLK2_DIV16 =  7,
}RCC_PCLK2_DIV_Type;
/*!  check APB2 frequence's divider value */
#define IS_PCLK2_DIV(div)   (((div) == RCC_PCLK2_DIV1)  \
                                       || ((div) == RCC_PCLK2_DIV2)  \
                                       || ((div) == RCC_PCLK2_DIV4)  \
                                       || ((div) == RCC_PCLK2_DIV8)  \
                                       || ((div) == RCC_PCLK2_DIV16)) 

/*!  RCC HSI TRIM CONSTANT */
typedef enum {
    RCC_HSITRIM_I00    =   0,  /*!< + 0.0% */
    RCC_HSITRIM_I25    =   1, /*!< + 0.25% */
    RCC_HSITRIM_I50    =   2, /*!< + 0.50% */
    RCC_HSITRIM_I75    =   3, /*!< + 0.75% */
    RCC_HSITRIM_D100   =   4,   /*!< - 1% */
    RCC_HSITRIM_D75    =   5,  /*!< - 0.75% */
    RCC_HSITRIM_D50    =   6,  /*!< - 0.50% */
    RCC_HSITRIM_D25    =   7,  /*!< - 0.25% */
}RCC_HSITRIM_Type;
/*!  check HSITRIM's value */
#define IS_HSITRIM(trim)    (((trim) == RCC_HSITRIM_I00)   \
                                       || ((trim) == RCC_HSITRIM_I25)   \
                                       || ((trim) == RCC_HSITRIM_I50)   \
                                       || ((trim) == RCC_HSITRIM_I75)   \
                                       || ((trim) == RCC_HSITRIM_D100) \
                                       || ((trim) == RCC_HSITRIM_D75)   \
                                       || ((trim) == RCC_HSITRIM_D50)   \
                                       || ((trim) == RCC_HSITRIM_D25) )

/*!  RCC Interrupt constant */
typedef enum
{
    RCC_INT_HSERDY = RCC_CISTR_HSERDYIF_Msk, /*!< Interrupt for HSE ready */
    RCC_INT_PLLRDY = RCC_CISTR_PLLRDYIF_Msk, /*!< Interrupt for PLL ready */
    RCC_INT_CSMHSE = RCC_CISTR_CSMHSEF_Msk,  /*!< Interrupt for CMS HSE error */
    RCC_INT_CSMPLL = RCC_CISTR_CSMPLLF_Msk,  /*!< Interrupt for CMS PLL error */ 
}RCC_INT_Type;
/*! check interrupt enable items */
#define IS_RCC_INTSRC(src) (((src)==RCC_INT_HSERDY) || ((src)==RCC_INT_PLLRDY))

/*! check interrupt enable items */
#define IS_RCC_INTSRCS(srcs) (((srcs)==RCC_INT_HSERDY)  \
                             || ((srcs)==RCC_INT_PLLRDY) \
                             || ((srcs) == (RCC_INT_HSERDY|RCC_INT_PLLRDY)))

/*! check interrupt flag items */
#define IS_RCC_INTFLAG(flag) (((flag)==RCC_INT_HSERDY) \
                             || ((flag)==RCC_INT_PLLRDY) \
                             || ((flag)==RCC_INT_CSMHSE) \
                             || ((flag)==RCC_INT_CSMPLL))
/*! All INT Flags */
#define RCC_INT_ALL    (RCC_INT_HSERDY|RCC_INT_PLLRDY|RCC_INT_CSMHSE|RCC_INT_CSMPLL)
/*! check interrupt flag items */
#define IS_RCC_INTFLAGS(flags)  ((((flags ) & RCC_INT_ALL) != 0) && (((flags) & (~RCC_INT_ALL)) == 0))


/*!  RCC AHB Modules */
typedef enum{
    RCC_AHB_GPIO_CFG   = RCC_AHBENR_IOCFGEN_Msk,
    RCC_AHB_GPIO_IOD   = RCC_AHBENR_IODATEN_Msk,
    RCC_AHB_GPIO       = RCC_AHB_GPIO_CFG | RCC_AHB_GPIO_IOD,
    RCC_AHB_ADC1   = RCC_AHBENR_ADC1EN_Msk,
    RCC_AHB_ADC2   = RCC_AHBENR_ADC2EN_Msk,
    RCC_AHB_ADC3   = RCC_AHBENR_ADC3EN_Msk,
    RCC_AHB_SYSCFG = RCC_AHBENR_SYSCFGEN_Msk,
    RCC_AHB_DMA    = RCC_AHBENR_DMAEN_Msk,
    RCC_AHB_MACP   = RCC_AHBENR_MACPEN_Msk,
    RCC_AHB_CRC    = RCC_AHBENR_CRCEN_Msk,
    RCC_AHB_GPT0   = RCC_AHBENR_GPT0EN_Msk|RCC_AHBENR_GPTEN_Msk,
    RCC_AHB_GPT1   = RCC_AHBENR_GPT1EN_Msk|RCC_AHBENR_GPTEN_Msk,
    RCC_AHB_GPT2   = RCC_AHBENR_GPT2EN_Msk|RCC_AHBENR_GPTEN_Msk,
    RCC_AHB_GPT3   = RCC_AHBENR_GPT3EN_Msk|RCC_AHBENR_GPTEN_Msk,
    RCC_AHB_GPT    = RCC_AHBENR_GPTEN_Msk, 
}RCC_AHB_Type;

/*! All APB2  Modules */
#define RCC_AHB_ALL    (0x7FFB)

/*! check AHB Module source */
#define IS_AHB_MODULES(m)   ((((m ) & RCC_AHB_ALL) != 0) && (((m) & (~RCC_AHB_ALL)) == 0))

/*!  RCC APB1 Modules */
typedef enum{
    RCC_APB1_TIM5   = RCC_APB1ENR_TIM5EN_Msk,
    RCC_APB1_TIM6   = RCC_APB1ENR_TIM6EN_Msk,
    RCC_APB1_TIM7   = RCC_APB1ENR_TIM7EN_Msk,
    RCC_APB1_TIM8   = RCC_APB1ENR_TIM8EN_Msk,
    RCC_APB1_QEI    = RCC_APB1ENR_QEIEN_Msk,
    RCC_APB1_UART1  = RCC_APB1ENR_UART1EN_Msk,
    RCC_APB1_UART2  = RCC_APB1ENR_UART2EN_Msk,
    RCC_APB1_UART3  = RCC_APB1ENR_UART3EN_Msk,
    RCC_APB1_SPI1   = RCC_APB1ENR_SPI1EN_Msk,
    RCC_APB1_SPI2   = RCC_APB1ENR_SPI2EN_Msk,
    RCC_APB1_TWI1   = RCC_APB1ENR_TWI1EN_Msk,
    RCC_APB1_WWDT   = RCC_APB1ENR_WWDTEN_Msk,
    RCC_APB1_AMOC   = RCC_APB1ENR_AMOCEN_Msk,
}RCC_APB1_Type;

/*! All APB2  Modules */
#define RCC_APB1_ALL    (0x3BFF)

/*! check APB1 clock source */
#define IS_APB1_MODULES(m) ((((m ) & RCC_APB1_ALL) != 0) && (((m) & (~RCC_APB1_ALL)) == 0))

/*!  RCC APB2 Modules */
typedef enum{
    RCC_APB2_MCM1 = RCC_APB2ENR_MCM1EN_Msk,
    RCC_APB2_MCM2 = RCC_APB2ENR_MCM2EN_Msk,
}RCC_APB2_Type;

/*! All APB2  Modules */
#define RCC_APB2_ALL    (RCC_APB2_MCM1|RCC_APB2_MCM2)

/*! check APB2 clock source */
#define IS_APB2_MODULES(m)  ((((m ) & RCC_APB2_ALL) != 0) && (((m) & (~RCC_APB2_ALL)) == 0))


/*!  RCC Reset Flags */
typedef enum{
    RCC_RST_PIN      =  RCC_RSTSTR_PINRSTF_Msk,
    RCC_RST_LVR      =  RCC_RSTSTR_LVRSTF_Msk,
    RCC_RST_POWERON  =  RCC_RSTSTR_PORSTF_Msk,
    RCC_RST_SOFTWARE =  RCC_RSTSTR_SWRSTF_Msk,
    RCC_RST_IWDT     =  RCC_RSTSTR_IWDTRSTF_Msk,
    RCC_RST_WWDT     =  RCC_RSTSTR_WWDTRSTF_Msk,
    RCC_RST_VCCLVR   =  RCC_RSTSTR_LVRSTF2_Msk,
}RCC_RESET_Type;

/*! Check RESET Type */
#define IS_RCC_RESET_Type(rst)    (   ((rst ) == RCC_RST_PIN ) \
                                  || ((rst ) == RCC_RST_LVR ) \
                                  || ((rst ) == RCC_RST_POWERON ) \
                                  || ((rst ) == RCC_RST_SOFTWARE ) \
                                  || ((rst ) == RCC_RST_IWDT ) \
                                  || ((rst ) == RCC_RST_WWDT ) \
                                  || ((rst ) == RCC_RST_VCCLVR ) \
                                  )
/*! All Reset Sources */                                  
#define RCC_RESET_ALL     (RCC_RST_PIN | RCC_RST_LVR | RCC_RST_POWERON | RCC_RST_SOFTWARE | RCC_RST_IWDT | RCC_RST_WWDT | RCC_RST_VCCLVR)                                  
/*! Check RESET Types */                                  
#define IS_RCC_RESET_Types(rst)  ((((rst ) & RCC_RESET_ALL) != 0) && (((rst) & (~RCC_RESET_ALL)) == 0))



/**
  * @}
  */ 

/** @defgroup RCC_Group_Types  Public Types
  * @{
  */     
    
/* Exported types ------------------------------------------------------------*/
    
/*! @struct  RCC_Clocks_TypeDef
  *  RCC structure of clock frequency datas, used to get current system's clock frequency
  */    
typedef struct
{
  uint32_t sysFreq;  /*!<  System clock frequency, unit is Hz */
  uint32_t hclkFreq;    /*!< AHB bus clock frequency, unit is Hz */
  uint32_t pclk1Freq;   /*!< APB1 bus clock frequency, unit is Hz */
  uint32_t pclk2Freq;   /*!< APB2 bus clock frequency, unit is Hz */
}RCC_Clocks_TypeDef;
    

/*! @struct  RCC_PLL_InitTypeDef
  *   PLL Initial structure
  */    
typedef struct
{
  RCC_PLL_XTPRE_Type   xpreDiv;   /*!<  Pre-divider for input clock */
  RCC_PLL_SRC_Type     pllSRC;    /*!< PLL clock source */
  uint8_t              pllMul;    /*!< PLL's F Value: 15~78*/
  uint8_t              pllDiv;    /*!< PLL's K Value: 1~16*/
}RCC_PLL_InitTypeDef;


/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RCC_Group_Macro  Public Macros
  * @{
  */ 

/** 
  *@brief  Unlock the RCC module's regsiter. RCC registers can be modified.
  */
#define RCC_REGS_UNLOCK() RCC->RCCLOCK=0x33CC

/** 
  *@brief  Lock the RCC module's regsiter. RCC registers cannot be modified.
  */
#define RCC_REGS_LOCK()   RCC->RCCLOCK=0


/*! PLL F VALUE CONSTANT  15~78*/
#define PLL_PLLF_15    0
/** 
  *@brief Calculate PLL's F value. Input value is 15~78.
  */
#define PLL_PLLF(f)      (PLL_PLLF_15+((f)-15))

/*! PLL K VALUE CONSTANT */
#define PLL_PLLK_1    0
/*! Calculate PLL's K value. Input value is 1~16*/
#define PLL_PLLK(n)     (PLL_PLLK_1+((n)-1))

      
/**
  * @}
  */ 

/* Exported functions --------------------------------------------------------*/ 
/** @addtogroup RCC_Group_Pub_Funcs
  * @{
  */     

/* get current clock requency*/
void RCC_GetClocksFreq(RCC_Clocks_TypeDef* Clocks);

/*  restore default clocks*/
void RCC_Reset(void);

/*  configure HSE*/
void RCC_HSEOnOff(CmdState OnOffState);

/*  wati HSE ready */
ErrorStatus RCC_WaitForHSEReady(uint32_t TimeOut);

/*  adjust HSI frequency */
uint32_t RCC_AdjustHSITrimValue(RCC_HSITRIM_Type HSITrimValue);

/* configure PLL's parameters */
void RCC_PLLConfig(RCC_PLL_InitTypeDef* PLLInit);

/* enable or disable PLL */
void RCC_PLLOnOff(CmdState OnOffState);

/* Wait PLL ready*/
ErrorStatus RCC_WaitForPLLReady(uint32_t TimeOut);

/* configure system's input clock source*/
void RCC_SysClkConfig(RCC_SysSource_Type SysClkSrc);

/* get current system clock's source */
RCC_SysSource_Type RCC_GetSysClkSource(void);

/*  configure HCLK*/
void RCC_HCLKConfig(RCC_HCLK_DIV_Type HCLK_DIV);

/* configure PCLK1 */
void RCC_PCLK1Config(RCC_PCLK1_DIV_Type PCLK1_DIV);

/* configure PCLK2 */
void RCC_PCLK2Config(RCC_PCLK2_DIV_Type PCKL2_DIV);

/* configure  RCC interrupt */ 
void RCC_INTConfig(uint32_t RCC_IT, FunctionalState NewState);

/* get RCC interrupt flag */
FlagStatus  RCC_GetINTStatus(RCC_INT_Type INTFlag);

/* clear RCC interrupt flag */
void  RCC_ClearINTStatus(uint32_t INTCFlag);


/* configure AHB modules' clock gate */ 
void  RCC_AHBPeriphClockOnOff(uint32_t AHBModules,CmdState OnOffState);

/* configure APB1 modules' clock gate */ 
void  RCC_APB1PeriphClockOnOff(uint32_t APB1Modules,CmdState OnOffState);

/* configure APB2 modules' clock gate */ 
void  RCC_APB2PeriphClockOnOff(uint32_t APB2Modules,CmdState OnOffState);

/* reset AHB modules */ 
void RCC_AHBPeriphReset(uint32_t AHBModules);

/* reset APB1 modules*/ 
void RCC_APB1PeriphReset(uint32_t APB1Modules);

/* reset APB2 modules */ 
void RCC_APB2PeriphReset(uint32_t APB2Modules);

/*  get RCC reset flags */
FlagStatus RCC_GetResetFlag(RCC_RESET_Type ResetFlags);

/*  clear RCC reset flags */
void RCC_ClearResetFlag(uint8_t ResetFlags);


/**
  * @}
  */ 


#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_RCC_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
