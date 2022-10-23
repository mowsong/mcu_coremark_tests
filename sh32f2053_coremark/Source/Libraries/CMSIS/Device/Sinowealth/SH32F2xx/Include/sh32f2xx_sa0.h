/**
  ******************************************************************************
  * @file    sh32f2xx_sa0.h
  * @version V1.0
  * @date    2018-05-30
  * @brief   Sinowealth Cortex-M3 based chip's device peripheral access layer 
  *          header File. This file contains all the peripheral register's 
  *          definitions, bits definitions and memory mapping.
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

#ifndef __SH32F2XX_SA0_H__
#define __SH32F2XX_SA0_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __CM3_REV                 0x0201  /*!< Core revision r2p1                            */
#define __MPU_PRESENT             1       /*!< SH32F2xxx provides an MPU                     */
#define __NVIC_PRIO_BITS          4       /*!< SH32F2xxx uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */



typedef enum
{
/******  Cortex-M3 Processor Exceptions Numbers ****************************************************************/
  NMI_IRQn                    = -14,    /*!< Non Maskable Interrupt                                            */
  HardFault_IRQn              = -13,    /*!< Hard Fault Interrupt                                              */
  MemManage_IRQn              = -12,    /*!< Cortex-M3 Memory Management Interrupt                             */
  BusFault_IRQn               = -11,    /*!< Cortex-M3 Bus Fault Interrupt                                     */
  UsageFault_IRQn             = -10,    /*!< Cortex-M3 Usage Fault Interrupt                                   */
  SVC_IRQn                    = -5,     /*!< Cortex-M3 SV Call Interrupt                                       */
  DebugMon_IRQn               = -4,     /*!< Cortex-M3 Debug Monitor Interrupt                                 */
  PendSV_IRQn                 = -2,     /*!< Cortex-M3 Pend SV Interrupt                                       */
  SysTick_IRQn                = -1,     /*!< Cortex-M3 System Tick Interrupt                                   */
/******  SH32Fxxx specific Interrupt Numbers *******************************************************************/
  WWDT_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  BOD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  Reserve1_IRQn               = 2,      /*!< Reserved                                                          */
  Reserve2_IRQn               = 3,      /*!< Reserved                                                          */
  RCC_IRQn                    = 4,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 5,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 6,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 7,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 8,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 9,      /*!< EXTI Line4 Interrupt                                              */
  DMA_CH0_IRQn                = 10,     /*!< DMA Channel0 Interrupt                                            */
  DMA_CH1_IRQn                = 11,     /*!< DMA Channel1 Interrupt                                            */
  DMA_CH2_7_IRQn              = 12,     /*!< DMA Channel2 ~ Channel7 Interrupt                                 */
  MCM1_FLT_IRQn               = 13,     /*!< MCM1 FAULT Interrupt                                              */
  MCM1_PWM_IRQn               = 14,     /*!< MCM1 PWM Interrupt                                                */
#ifdef SH32F284  
  Reserve284_0_IRQn           = 15,     /*!< Reserved                                                          */
  Reserve284_1_IRQn           = 16,     /*!< Reserved                                                          */
#else
  MCM2_FLT_IRQn               = 15,     /*!< MCM2 FAULT Interrupt                                              */
  MCM2_PWM_IRQn               = 16,     /*!< MCM2 PWM Interrupt                                                */
#endif  
  ADC1_IRQn                   = 17,     /*!< ADC1 Interrupt                                                    */
#ifdef SH32F284  
  Reserve284_3_IRQn           = 18,     /*!< Reserved                                                          */
#else
  ADC2_IRQn                   = 18,     /*!< ADC2 Interrupt                                                    */
#endif  
  ADC3_IRQn                   = 19,     /*!< ADC3 Interrupt                                                    */
  Reserve3_IRQn               = 20,     /*!< Reserved                                                          */
  Reserve4_IRQn               = 21,     /*!< Reserved                                                          */
  GPT0_GTCIV_IRQn             = 22,     /*!< GPT0 GTCIV Interrupt                                              */
  GPT0_GTCIN_IRQn             = 23,     /*!< GPT0 GTCIN Interrupt                                              */
  GPT1_GTCIV_IRQn             = 24,     /*!< GPT1 GTCIV Interrupt                                              */
  GPT1_GTCIN_IRQn             = 25,     /*!< GPT1 GTCIN Interrupt                                              */
  GPT2_GTCIV_IRQn             = 26,     /*!< GPT2 GTCIV Interrupt                                              */
  GPT2_GTCIN_IRQn             = 27,     /*!< GPT2 GTCIN Interrupt                                              */
#ifdef SH32F284  
  Reserve284_4_IRQn           = 28,     /*!< Reserved                                                          */
  Reserve284_5_IRQn           = 29,     /*!< Reserved                                                          */  
#else
  GPT3_GTCIV_IRQn             = 28,     /*!< GPT3 GTCIV Interrupt                                              */
  GPT3_GTCIN_IRQn             = 29,     /*!< GPT3 GTCIN Interrupt                                              */  
#endif  
  GPT_POE_IRQn                = 30,     /*!< GPT POE Global Interrupt                                          */
  EXTI9_5_IRQn                = 31,     /*!< EXTI Line5 ~ Line9 global Interrupt                               */
  CMP1_IRQn                   = 32,     /*!< CMP1 Interrupt                                                    */
  CMP2_IRQn                   = 33,     /*!< CMP2 Interrupt                                                    */
  CMP3_IRQn                   = 34,     /*!< CMP3 Interrupt                                                    */
  QEI_IRQn                    = 35,     /*!< QEI Interrupt                                                     */
  TWI_IRQn                    = 36,     /*!< TWI Interrupt                                                     */
  Reserve5_IRQn               = 37,     /*!< Reserved Interrupt                                                */
  SPI1_IRQn                   = 38,     /*!< SPI1 Interrupt                                                    */
#ifdef SH32F284  
  Reserve284_6_IRQn           = 39,     /*!< Reserved                                                          */
#else
  SPI2_IRQn                   = 39,     /*!< SPI2 Interrupt                                                    */
#endif  
  UART1_IRQn                  = 40,     /*!< UART1 Interrupt                                                   */  
  UART2_IRQn                  = 41,     /*!< UART2 Interrupt                                                   */
  UART3_IRQn                  = 42,     /*!< UART3 Interrupt                                                   */
  Reserved6_IRQn              = 43,     /*!< Reserved Interrupt                                                */
  EXTI15_10_IRQn              = 44,     /*!< EXTI Line10 ~ Line15 Interrupt                                    */
  Reserved7_IRQn              = 45,     /*!< Reserved Interrupt                                                */
  Reserved8_IRQn              = 46,     /*!< Reserved Interrupt                                                */
  Reserved9_IRQn              = 47,     /*!< Reserved Interrupt                                                */
  Reserved10_IRQn             = 48,     /*!< Reserved Interrupt                                                */
  TIM5_IRQn                   = 49,     /*!< TIM5 Interrupt                                                    */
  TIM6_IRQn                   = 50,     /*!< TIM6 Interrupt                                                    */
  TIM7_IRQn                   = 51,     /*!< TIM7 Interrupt                                                    */
  TIM8_IRQn                   = 52,     /*!< TIM8 Interrupt                                                    */
} IRQn_Type;

/* Includes ****************************************************************************************************/
#include <core_cm3.h>
#if defined ( __clang__ )
#include <arm_acle.h>
#endif
#include <stdint.h>

/**
*@brief FLASH_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  LATENCY   : 3;  /*!< [b2~b0]*/
                  uint32_t  rev0      : 5;  /*!< [b7~b3]*/
            __IO  uint32_t  PRFTEN    : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  ICEN      : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  DCEN      : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  CRST      : 1;  /*!< [b11~b11]*/
                  uint32_t  rev1      : 4;  /*!< [b15~b12]*/
            __O   uint32_t  LOCK      :16;  /*!< [b31~b16]*/
        }BIT;
    }ACR;                               /*!< 0000H */
    __O  uint32_t  MKYR;                                    /*!< 0004H */
    __O  uint32_t  E2KYR;                                   /*!< 0008H */
    union {
        __IO  uint32_t  V32;                                /*!< 000CH */
        struct {
            __I   uint32_t  EOP       : 1;  /*!< [b0~b0]*/
            __I   uint32_t  OPERR     : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      : 1;  /*!< [b2~b2]*/
            __I   uint32_t  FLSERR    : 1;  /*!< [b3~b3]*/
            __I   uint32_t  WRPRTERR  : 1;  /*!< [b4~b4]*/
            __I   uint32_t  PGPERR    : 1;  /*!< [b5~b5]*/
            __I   uint32_t  PGWERR    : 1;  /*!< [b6~b6]*/
            __I   uint32_t  STAERR    : 1;  /*!< [b7~b7]*/
                  uint32_t  rev1      : 7;  /*!< [b14~b8]*/
            __I   uint32_t  BSY       : 1;  /*!< [b15~b15]*/
            __O   uint32_t  EOPC      : 1;  /*!< [b16~b16]*/
            __O   uint32_t  OPERRC    : 1;  /*!< [b17~b17]*/
                  uint32_t  rev2      : 1;  /*!< [b18~b18]*/
            __O   uint32_t  FLSERRC   : 1;  /*!< [b19~b19]*/
            __O   uint32_t  WRPRTERRC : 1;  /*!< [b20~b20]*/
            __O   uint32_t  PGPERRC   : 1;  /*!< [b21~b21]*/
            __O   uint32_t  PGWERRC   : 1;  /*!< [b22~b22]*/
            __O   uint32_t  STAERRC   : 1;  /*!< [b23~b23]*/
                  uint32_t  rev3      : 8;  /*!< [b31~b24]*/
        }BIT;
    }SR;                               /*!< 000CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0010H */
        struct {
            __IO  uint32_t  SNB       : 7;  /*!< [b6~b0]*/
                  uint32_t  rev0      : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  STRT      : 1;  /*!< [b8~b8]*/
                  uint32_t  rev1      : 2;  /*!< [b10~b9]*/
            __IO  uint32_t  PSIZE     : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  INFLCK    : 1;  /*!< [b12~b12]*/
                  uint32_t  rev2      : 1;  /*!< [b13~b13]*/
            __IO  uint32_t  E2LCK     : 1;  /*!< [b14~b14]*/
            __IO  uint32_t  MNLCK     : 1;  /*!< [b15~b15]*/
            __IO  uint32_t  CMD       :16;  /*!< [b31~b16]*/
        }BIT;
    }CR;                               /*!< 0010H */
         uint32_t  Reserved0[2];                            /*!< 0014H */
    union {
        __IO  uint32_t  V32;                                /*!< 001CH */
        struct {
    __IO uint8_t   OPT0;                                    /*!< 001CH */
    __IO uint8_t   OPT1;                                    /*!< 001DH */
    __IO uint8_t   OPT2;                                    /*!< 001EH */
    __IO uint8_t   OPT3;                                    /*!< 001FH */
        }BIT;
    }OPR;                              /*!< 001CH */
    __I  uint16_t  RPR;                                     /*!< 0020H */
         uint16_t  Reserved1;                               /*!< 0022H */
    __I  uint32_t  WRPR;                                    /*!< 0024H */
         uint32_t  Reserved2[2];                            /*!< 0028H */
    __IO uint32_t  CNTR;                                    /*!< 0030H */
    __IO uint32_t  UPCNTR;                                  /*!< 0034H */
    __IO uint8_t   CNTCR;                                   /*!< 0038H */
         uint8_t   Reserved3[3];                            /*!< 0039H */
         uint32_t  Reserved4[1];                            /*!< 003CH */
    __O  uint32_t  IKYR;                                    /*!< 0040H */
         uint32_t  Reserved5[47];                           /*!< 0044H */
    __IO uint16_t  MEMRMP;                                  /*!< 0100H */
         uint16_t  Reserved6;                               /*!< 0102H */
         uint32_t  Reserved7[63];                           /*!< 0104H */
    union {
        __IO  uint32_t  V32;                                /*!< 0200H */
        struct {
    __IO uint8_t   OPT0;                                    /*!< 0200H */
    __IO uint8_t   OPT1;                                    /*!< 0201H */
    __IO uint8_t   OPT2;                                    /*!< 0202H */
    __IO uint8_t   OPT3;                                    /*!< 0203H */
        }BIT;
    }OPR_CUST1;                              /*!< 0200H */
    union {
        __IO  uint32_t  V32;                                /*!< 0204H */
        struct {
    __IO uint8_t   OPT0;                                    /*!< 0204H */
    __IO uint8_t   OPT1;                                    /*!< 0205H */
    __IO uint8_t   OPT2;                                    /*!< 0206H */
    __IO uint8_t   OPT3;                                    /*!< 0207H */
        }BIT;
    }OPR_DESI0;                              /*!< 0204H */
    union {
        __IO  uint32_t  V32;                                /*!< 0208H */
        struct {
    __IO uint8_t   OPT0;                                    /*!< 0208H */
    __IO uint8_t   OPT1;                                    /*!< 0209H */
    __IO uint8_t   OPT2;                                    /*!< 020AH */
    __IO uint8_t   OPT3;                                    /*!< 020BH */
        }BIT;
    }OPR_DESI1;                              /*!< 0208H */
    union {
        __IO  uint32_t  V32;                                /*!< 020CH */
        struct {
    __IO uint8_t   OPT0;                                    /*!< 020CH */
    __IO uint8_t   OPT1;                                    /*!< 020DH */
    __IO uint8_t   OPT2;                                    /*!< 020EH */
    __IO uint8_t   OPT3;                                    /*!< 020FH */
        }BIT;
    }OPR_DESI2;                              /*!< 020CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0210H */
        struct {
    __IO uint8_t   OPT0;                                    /*!< 0210H */
    __IO uint8_t   OPT1;                                    /*!< 0211H */
    __IO uint8_t   OPT2;                                    /*!< 0212H */
    __IO uint8_t   OPT3;                                    /*!< 0213H */
        }BIT;
    }OPR_DESI3;                              /*!< 0210H */
}FLASH_TypeDef;


/**
*@brief SYSCFG_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  VBOD      : 4;  /*!< [b3~b0]*/
            __IO  uint32_t  BODMD     : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  BODIE     : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  BODEN     : 1;  /*!< [b7~b7]*/
                  uint32_t  rev0      :24;  /*!< [b31~b8]*/
        }BIT;
    }PWRCR;                               /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __IO  uint32_t  BODIF     : 1;  /*!< [b0~b0]*/
            __I   uint32_t  BODF      : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      :30;  /*!< [b31~b2]*/
        }BIT;
    }PWRSR;                               /*!< 0004H */
    union {
        __IO  uint32_t  V32;                                /*!< 0008H */
        struct {
            __IO  uint32_t  OSCCFG    : 2;  /*!< [b1~b0]*/
            __IO  uint32_t  SWJCFG    : 3;  /*!< [b4~b2]*/
            __IO  uint32_t  IEN_EXTI0 : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  IEN_BOD   : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  IEN_CSM   : 1;  /*!< [b7~b7]*/
                  uint32_t  rev0      : 8;  /*!< [b15~b8]*/
            __O   uint32_t  LOCK      :16;  /*!< [b31~b16]*/
        }BIT;
    }SAFR;                               /*!< 0008H */
    union {
        __IO  uint32_t  V32;                                /*!< 000CH */
        struct {
    __IO uint8_t   CRAMLCK;                                 /*!< 000CH */
         uint8_t   rev0;                                    /*!< 000DH */
    __O  uint16_t  LOCK;                                    /*!< 000EH */
        }BIT;
    }CRAMLOCK;                              /*!< 000CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0010H */
        struct {
            __IO  uint32_t  DBG_SLEEP : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  DBG_STOP  : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      : 5;  /*!< [b6~b2]*/
            __IO  uint32_t  DBG_DMA   : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  DBG_IWDT  : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  DBG_WWDT  : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  DBG_GPT   : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  DBG_TIM   : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  DBG_MCM   : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  DBG_UART  : 1;  /*!< [b13~b13]*/
            __IO  uint32_t  DBG_SPI   : 1;  /*!< [b14~b14]*/
            __IO  uint32_t  DBG_TWI   : 1;  /*!< [b15~b15]*/
            __O   uint32_t  LOCK      :16;  /*!< [b31~b16]*/
        }BIT;
    }DBGCR;                               /*!< 0010H */
}SYSCFG_TypeDef;


/**
*@brief RCC_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  SW        : 2;  /*!< [b1~b0]*/
            __I   uint32_t  SWS       : 2;  /*!< [b3~b2]*/
            __IO  uint32_t  HSEON     : 1;  /*!< [b4~b4]*/
            __I   uint32_t  HSERDY    : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  PLLON     : 1;  /*!< [b6~b6]*/
            __I   uint32_t  PLLRDY    : 1;  /*!< [b7~b7]*/
                  uint32_t  rev0      :24;  /*!< [b31~b8]*/
        }BIT;
    }CR;                               /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __IO  uint32_t  HPRE      : 3;  /*!< [b2~b0]*/
            __IO  uint32_t  PPRE1     : 3;  /*!< [b5~b3]*/
            __IO  uint32_t  PPRE2     : 3;  /*!< [b8~b6]*/
            __IO  uint32_t  PLLK      : 3;  /*!< [b11~b9]*/
            __IO  uint32_t  PLLF      : 6;  /*!< [b17~b12]*/
            __IO  uint32_t  PLLSRC    : 1;  /*!< [b18~b18]*/
            __IO  uint32_t  PLLXTPRE  : 1;  /*!< [b19~b19]*/
                  uint32_t  rev0      :12;  /*!< [b31~b20]*/
        }BIT;
    }CFGR;                               /*!< 0004H */
    union {
        __IO  uint32_t  V32;                                /*!< 0008H */
        struct {
                  uint32_t  rev0      : 3;  /*!< [b2~b0]*/
            __IO  uint32_t  HSERDYIE  : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  PLLRDYIE  : 1;  /*!< [b4~b4]*/
                  uint32_t  rev1      :27;  /*!< [b31~b5]*/
        }BIT;
    }CIENR;                               /*!< 0008H */
    union {
        __I   uint32_t  V32;                                /*!< 000CH */
        struct {
                  uint32_t  rev0      : 3;  /*!< [b2~b0]*/
            __I   uint32_t  HSERDYIF  : 1;  /*!< [b3~b3]*/
            __I   uint32_t  PLLRDYIF  : 1;  /*!< [b4~b4]*/
                  uint32_t  rev1      : 1;  /*!< [b5~b5]*/
            __I   uint32_t  CSMHSEF   : 1;  /*!< [b6~b6]*/
            __I   uint32_t  CSMPLLF   : 1;  /*!< [b7~b7]*/
                  uint32_t  rev2      :24;  /*!< [b31~b8]*/
        }BIT;
    }CISTR;                               /*!< 000CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0010H */
        struct {
                  uint32_t  rev0      : 3;  /*!< [b2~b0]*/
            __O   uint32_t  HSERDYC   : 1;  /*!< [b3~b3]*/
            __O   uint32_t  PLLRDYC   : 1;  /*!< [b4~b4]*/
                  uint32_t  rev1      : 2;  /*!< [b6~b5]*/
            __O   uint32_t  CSMC      : 1;  /*!< [b7~b7]*/
                  uint32_t  rev2      :24;  /*!< [b31~b8]*/
        }BIT;
    }CICLR;                               /*!< 0010H */
    union {
        __IO  uint32_t  V32;                                /*!< 0014H */
        struct {
            __IO  uint32_t  IOCFGRST  : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  IODATRST  : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  ADC1RST   : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  ADC2RST   : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  ADC3RST   : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  SYSCFGRST : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  DMARST    : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  MACPRST   : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  CRCRST    : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  GPT0RST   : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  GPT1RST   : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  GPT2RST   : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  GPT3RST   : 1;  /*!< [b13~b13]*/
            __IO  uint32_t  GPTRST    : 1;  /*!< [b14~b14]*/
                  uint32_t  rev1      :17;  /*!< [b31~b15]*/
        }BIT;
    }AHBRSTR;                               /*!< 0014H */
    union {
        __IO  uint32_t  V32;                                /*!< 0018H */
        struct {
            __IO  uint32_t  MCM1RST   : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  MCM2RST   : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      :30;  /*!< [b31~b2]*/
        }BIT;
    }APB2RSTR;                               /*!< 0018H */
    union {
        __IO  uint32_t  V32;                                /*!< 001CH */
        struct {
            __IO  uint32_t  TIM5RST   : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TIM6RST   : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  TIM7RST   : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  TIM8RST   : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  QEIRST    : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  UART1RST  : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  UART2RST  : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  UART3RST  : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  SPI1RST   : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  SPI2RST   : 1;  /*!< [b9~b9]*/
                  uint32_t  rev0      : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  TWI1RST   : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  WWDTRST   : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  AMOCRST   : 1;  /*!< [b13~b13]*/
                  uint32_t  rev1      :18;  /*!< [b31~b14]*/
        }BIT;
    }APB1RSTR;                               /*!< 001CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0020H */
        struct {
            __IO  uint32_t  IOCFGEN   : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  IODATEN   : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  ADC1EN    : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  ADC2EN    : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  ADC3EN    : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  SYSCFGEN  : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  DMAEN     : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  MACPEN    : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  CRCEN     : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  GPT0EN    : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  GPT1EN    : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  GPT2EN    : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  GPT3EN    : 1;  /*!< [b13~b13]*/
            __IO  uint32_t  GPTEN     : 1;  /*!< [b14~b14]*/
                  uint32_t  rev1      :17;  /*!< [b31~b15]*/
        }BIT;
    }AHBENR;                               /*!< 0020H */
    union {
        __IO  uint32_t  V32;                                /*!< 0024H */
        struct {
            __IO  uint32_t  MCM1EN    : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  MCM2EN    : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      :30;  /*!< [b31~b2]*/
        }BIT;
    }APB2ENR;                               /*!< 0024H */
    union {
        __IO  uint32_t  V32;                                /*!< 0028H */
        struct {
            __IO  uint32_t  TIM5EN    : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TIM6EN    : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  TIM7EN    : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  TIM8EN    : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  QEIEN     : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  UART1EN   : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  UART2EN   : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  UART3EN   : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  SPI1EN    : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  SPI2EN    : 1;  /*!< [b9~b9]*/
                  uint32_t  rev0      : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  TWI1EN    : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  WWDTEN    : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  AMOCEN    : 1;  /*!< [b13~b13]*/
                  uint32_t  rev1      :18;  /*!< [b31~b14]*/
        }BIT;
    }APB1ENR;                               /*!< 0028H */
    union {
        __I   uint32_t  V32;                                /*!< 002CH */
        struct {
            __I   uint32_t  PINRSTF   : 1;  /*!< [b0~b0]*/
            __I   uint32_t  LVRSTF    : 1;  /*!< [b1~b1]*/
            __I   uint32_t  PORSTF    : 1;  /*!< [b2~b2]*/
            __I   uint32_t  SWRSTF    : 1;  /*!< [b3~b3]*/
            __I   uint32_t  IWDTRSTF  : 1;  /*!< [b4~b4]*/
            __I   uint32_t  WWDTRSTF  : 1;  /*!< [b5~b5]*/
            __I   uint32_t  LVRSTF2   : 1;  /*!< [b6~b6]*/
                  uint32_t  rev0      :25;  /*!< [b31~b7]*/
        }BIT;
    }RSTSTR;                               /*!< 002CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0030H */
        struct {
            __O   uint32_t  PINRSTFC  : 1;  /*!< [b0~b0]*/
            __O   uint32_t  LVRSTFC   : 1;  /*!< [b1~b1]*/
            __O   uint32_t  PORSTFC   : 1;  /*!< [b2~b2]*/
            __O   uint32_t  SWRSTFC   : 1;  /*!< [b3~b3]*/
            __O   uint32_t  IWDTRSTFC : 1;  /*!< [b4~b4]*/
            __O   uint32_t  WWDTRSTFC : 1;  /*!< [b5~b5]*/
            __O   uint32_t  LVRSTF2C  : 1;  /*!< [b6~b6]*/
                  uint32_t  rev0      :25;  /*!< [b31~b7]*/
        }BIT;
    }RSTCLR;                               /*!< 0030H */
    union {
        __IO  uint32_t  V32;                                /*!< 0034H */
        struct {
            __IO  uint32_t  HSITRIM   : 3;  /*!< [b2~b0]*/
                  uint32_t  rev0      : 5;  /*!< [b7~b3]*/
            __I   uint32_t  HSICAL    : 8;  /*!< [b15~b8]*/
            __I   uint32_t  TRIMREF   :13;  /*!< [b28~b16]*/
                  uint32_t  rev1      : 2;  /*!< [b30~b29]*/
            __IO  uint32_t  TRIMRUN   : 1;  /*!< [b31~b31]*/
        }BIT;
    }HSICAL;                               /*!< 0034H */
    __IO uint16_t  RCCLOCK;                                 /*!< 0038H */
         uint16_t  Reserved0;                               /*!< 003AH */
}RCC_TypeDef;


/**
*@brief IWDT_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  IWDTRLR   :12;  /*!< [b11~b0]*/
            __IO  uint32_t  IWDTPR    : 3;  /*!< [b14~b12]*/
            __IO  uint32_t  IWDTON    : 1;  /*!< [b15~b15]*/
            __O   uint32_t  LOCK      :16;  /*!< [b31~b16]*/
        }BIT;
    }CR;                               /*!< 0000H */
    __O  uint16_t  CLR;                                     /*!< 0004H */
         uint16_t  Reserved0;                               /*!< 0006H */
}IWDT_TypeDef;


/**
*@brief WWDT_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  WWDTRLR   : 8;  /*!< [b7~b0]*/
            __IO  uint32_t  WWDTPR    : 3;  /*!< [b10~b8]*/
                  uint32_t  rev0      : 3;  /*!< [b13~b11]*/
            __IO  uint32_t  WWDTIE    : 1;  /*!< [b14~b14]*/
            __IO  uint32_t  WWDTON    : 1;  /*!< [b15~b15]*/
            __O   uint32_t  LOCK      :16;  /*!< [b31~b16]*/
        }BIT;
    }CR;                               /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __I   uint32_t  TCNT      : 8;  /*!< [b7~b0]*/
                  uint32_t  rev0      : 7;  /*!< [b14~b8]*/
            __IO  uint32_t  WWDTIF    : 1;  /*!< [b15~b15]*/
                  uint32_t  rev1      :16;  /*!< [b31~b16]*/
        }BIT;
    }SR;                               /*!< 0004H */
    __O  uint16_t  CLR;                                     /*!< 0008H */
         uint16_t  Reserved0;                               /*!< 000AH */
    union {
        __IO  uint32_t  V32;                                /*!< 000CH */
        struct {
    __IO uint8_t   WWDTWTR;                                 /*!< 000CH */
         uint8_t   rev0;                                    /*!< 000DH */
    __O  uint16_t  LOCK;                                    /*!< 000EH */
        }BIT;
    }WTR;                              /*!< 000CH */
}WWDT_TypeDef;


/**
*@brief CRC_registers
*/
typedef struct
{
    __IO uint32_t  DR;                                      /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __O   uint32_t  RELOAD    : 1;  /*!< [b0~b0]*/
                  uint32_t  rev0      : 7;  /*!< [b7~b1]*/
            __IO  uint32_t  MODE      : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  COMPLW    : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  RBITW     : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  RBYTEW    : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  COMPLR    : 1;  /*!< [b13~b13]*/
            __IO  uint32_t  RBITR     : 1;  /*!< [b14~b14]*/
            __IO  uint32_t  RBYTER    : 1;  /*!< [b15~b15]*/
                  uint32_t  rev1      :16;  /*!< [b31~b16]*/
        }BIT;
    }CR;                               /*!< 0004H */
    __IO uint32_t  INIT;                                    /*!< 0008H */
}CRC_TypeDef;


/**
*@brief RAMBIST_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  RAMADR    :14;  /*!< [b13~b0]*/
                  uint32_t  rev0      :14;  /*!< [b27~b14]*/
            __IO  uint32_t  RAMTYP    : 2;  /*!< [b29~b28]*/
                  uint32_t  rev1      : 2;  /*!< [b31~b30]*/
        }BIT;
    }ADDR;                               /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __IO  uint32_t  BLKSZ     : 3;  /*!< [b2~b0]*/
                  uint32_t  rev0      :12;  /*!< [b14~b3]*/
            __IO  uint32_t  SEL       : 1;  /*!< [b15~b15]*/
                  uint32_t  rev1      :16;  /*!< [b31~b16]*/
        }BIT;
    }CFG;                               /*!< 0004H */
    union {
        __IO  uint32_t  V32;                                /*!< 0008H */
        struct {
            __O   uint32_t  RUN       :16;  /*!< [b15~b0]*/
            __IO  uint32_t  MOD       : 1;  /*!< [b16~b16]*/
            __I   uint32_t  BSY       : 1;  /*!< [b17~b17]*/
            __IO  uint32_t  ERR       : 1;  /*!< [b18~b18]*/
                  uint32_t  rev0      :13;  /*!< [b31~b19]*/
        }BIT;
    }CSR;                               /*!< 0008H */
}RAMBIST_TypeDef;


/**
*@brief GPIO_CFG_registers
*/
typedef struct
{
    __IO uint16_t  OTYPER;                                  /*!< 0000H */
         uint16_t  Reserved0;                               /*!< 0002H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __IO  uint32_t  ODRVR0    : 2;  /*!< [b1~b0]*/
            __IO  uint32_t  ODRVR1    : 2;  /*!< [b3~b2]*/
            __IO  uint32_t  ODRVR2    : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  ODRVR3    : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  ODRVR4    : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  ODRVR5    : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  ODRVR6    : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  ODRVR7    : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  ODRVR8    : 2;  /*!< [b17~b16]*/
            __IO  uint32_t  ODRVR9    : 2;  /*!< [b19~b18]*/
            __IO  uint32_t  ODRVR10   : 2;  /*!< [b21~b20]*/
            __IO  uint32_t  ODRVR11   : 2;  /*!< [b23~b22]*/
            __IO  uint32_t  ODRVR12   : 2;  /*!< [b25~b24]*/
            __IO  uint32_t  ODRVR13   : 2;  /*!< [b27~b26]*/
            __IO  uint32_t  ODRVR14   : 2;  /*!< [b29~b28]*/
            __IO  uint32_t  ODRVR15   : 2;  /*!< [b31~b30]*/
        }BIT;
    }ODRVR;                               /*!< 0004H */
    union {
        __IO  uint32_t  V32;                                /*!< 0008H */
        struct {
            __IO  uint32_t  PUPDR0    : 2;  /*!< [b1~b0]*/
            __IO  uint32_t  PUPDR1    : 2;  /*!< [b3~b2]*/
            __IO  uint32_t  PUPDR2    : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  PUPDR3    : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  PUPDR4    : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  PUPDR5    : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  PUPDR6    : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PUPDR7    : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  PUPDR8    : 2;  /*!< [b17~b16]*/
            __IO  uint32_t  PUPDR9    : 2;  /*!< [b19~b18]*/
            __IO  uint32_t  PUPDR10   : 2;  /*!< [b21~b20]*/
            __IO  uint32_t  PUPDR11   : 2;  /*!< [b23~b22]*/
            __IO  uint32_t  PUPDR12   : 2;  /*!< [b25~b24]*/
            __IO  uint32_t  PUPDR13   : 2;  /*!< [b27~b26]*/
            __IO  uint32_t  PUPDR14   : 2;  /*!< [b29~b28]*/
            __IO  uint32_t  PUPDR15   : 2;  /*!< [b31~b30]*/
        }BIT;
    }PUPDR;                               /*!< 0008H */
    union {
        __IO  uint32_t  V32;                                /*!< 000CH */
        struct {
            __IO  uint32_t  AFR0      : 4;  /*!< [b3~b0]*/
            __IO  uint32_t  AFR1      : 4;  /*!< [b7~b4]*/
            __IO  uint32_t  AFR2      : 4;  /*!< [b11~b8]*/
            __IO  uint32_t  AFR3      : 4;  /*!< [b15~b12]*/
            __IO  uint32_t  AFR4      : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  AFR5      : 4;  /*!< [b23~b20]*/
            __IO  uint32_t  AFR6      : 4;  /*!< [b27~b24]*/
            __IO  uint32_t  AFR7      : 4;  /*!< [b31~b28]*/
        }BIT;
    }AFRL;                               /*!< 000CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0010H */
        struct {
            __IO  uint32_t  AFR8      : 4;  /*!< [b3~b0]*/
            __IO  uint32_t  AFR9      : 4;  /*!< [b7~b4]*/
            __IO  uint32_t  AFR10     : 4;  /*!< [b11~b8]*/
            __IO  uint32_t  AFR11     : 4;  /*!< [b15~b12]*/
            __IO  uint32_t  AFR12     : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  AFR13     : 4;  /*!< [b23~b20]*/
            __IO  uint32_t  AFR14     : 4;  /*!< [b27~b24]*/
            __IO  uint32_t  AFR15     : 4;  /*!< [b31~b28]*/
        }BIT;
    }AFRH;                               /*!< 0010H */
    union {
        __IO  uint32_t  V32;                                /*!< 0014H */
        struct {
                  uint32_t  rev0      : 2;  /*!< [b1~b0]*/
            __IO  uint32_t  PA2       : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  PA3       : 1;  /*!< [b3~b3]*/
                  uint32_t  rev1      : 4;  /*!< [b7~b4]*/
            __IO  uint32_t  PA8       : 1;  /*!< [b8~b8]*/
                  uint32_t  rev2      : 2;  /*!< [b10~b9]*/
            __IO  uint32_t  PA11      : 1;  /*!< [b11~b11]*/
                  uint32_t  rev3      :20;  /*!< [b31~b12]*/
        }BIT;
    }TTLEN;                               /*!< 0014H */
    union {
        __IO  uint32_t  V32;                                /*!< 0018H */
        struct {
    __IO uint16_t  LCK;                                     /*!< 0018H */
    __O  uint16_t  LOCK;                                    /*!< 001AH */
        }BIT;
    }LCKR;                              /*!< 0018H */
}GPIO_CFG_TypeDef;


/**
*@brief GPIO_IOD_registers
*/
typedef struct
{
    __IO uint16_t  MODER;                                   /*!< 0000H */
         uint16_t  Reserved0;                               /*!< 0002H */
    __I  uint16_t  IDR;                                     /*!< 0004H */
         uint16_t  Reserved1;                               /*!< 0006H */
    __IO uint16_t  ODR;                                     /*!< 0008H */
         uint16_t  Reserved2;                               /*!< 000AH */
    union {
        __IO  uint32_t  V32;                                /*!< 000CH */
        struct {
    __O  uint16_t  BS;                                      /*!< 000CH */
    __O  uint16_t  BR;                                      /*!< 000EH */
        }BIT;
    }BSRR;                              /*!< 000CH */
}GPIO_TypeDef;


/**
*@brief EXTI_registers
*/
typedef struct
{
    __IO uint16_t  IMR;                                     /*!< 0000H */
         uint16_t  Reserved0;                               /*!< 0002H */
    __IO uint16_t  EMR;                                     /*!< 0004H */
         uint16_t  Reserved1;                               /*!< 0006H */
    __IO uint16_t  TMSR;                                    /*!< 0008H */
         uint16_t  Reserved2;                               /*!< 000AH */
    __IO uint16_t  RTSR;                                    /*!< 000CH */
         uint16_t  Reserved3;                               /*!< 000EH */
    __IO uint16_t  FTSR;                                    /*!< 0010H */
         uint16_t  Reserved4;                               /*!< 0012H */
    __O  uint16_t  SWIER;                                   /*!< 0014H */
         uint16_t  Reserved5;                               /*!< 0016H */
    union {
        __IO  uint32_t  V32;                                /*!< 0018H */
        struct {
    __I  uint16_t  PR;                                      /*!< 0018H */
    __O  uint16_t  PRC;                                     /*!< 001AH */
        }BIT;
    }PR;                              /*!< 0018H */
    union {
        __IO  uint32_t  V32;                                /*!< 001CH */
        struct {
            __IO  uint32_t  EXTI0     : 3;  /*!< [b2~b0]*/
                  uint32_t  rev0      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  EXTI1     : 3;  /*!< [b6~b4]*/
                  uint32_t  rev1      : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  EXTI2     : 3;  /*!< [b10~b8]*/
                  uint32_t  rev2      : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  EXTI3     : 3;  /*!< [b14~b12]*/
                  uint32_t  rev3      : 1;  /*!< [b15~b15]*/
            __IO  uint32_t  EXTI4     : 3;  /*!< [b18~b16]*/
                  uint32_t  rev4      : 1;  /*!< [b19~b19]*/
            __IO  uint32_t  EXTI5     : 3;  /*!< [b22~b20]*/
                  uint32_t  rev5      : 1;  /*!< [b23~b23]*/
            __IO  uint32_t  EXTI6     : 3;  /*!< [b26~b24]*/
                  uint32_t  rev6      : 1;  /*!< [b27~b27]*/
            __IO  uint32_t  EXTI7     : 3;  /*!< [b30~b28]*/
                  uint32_t  rev7      : 1;  /*!< [b31~b31]*/
        }BIT;
    }CFGL;                               /*!< 001CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0020H */
        struct {
            __IO  uint32_t  EXTI8     : 3;  /*!< [b2~b0]*/
                  uint32_t  rev0      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  EXTI9     : 3;  /*!< [b6~b4]*/
                  uint32_t  rev1      : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  EXTI10    : 3;  /*!< [b10~b8]*/
                  uint32_t  rev2      : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  EXTI11    : 3;  /*!< [b14~b12]*/
                  uint32_t  rev3      : 1;  /*!< [b15~b15]*/
            __IO  uint32_t  EXTI12    : 3;  /*!< [b18~b16]*/
                  uint32_t  rev4      : 1;  /*!< [b19~b19]*/
            __IO  uint32_t  EXTI13    : 3;  /*!< [b22~b20]*/
                  uint32_t  rev5      : 1;  /*!< [b23~b23]*/
            __IO  uint32_t  EXTI14    : 3;  /*!< [b26~b24]*/
                  uint32_t  rev6      : 1;  /*!< [b27~b27]*/
            __IO  uint32_t  EXTI15    : 3;  /*!< [b30~b28]*/
                  uint32_t  rev7      : 1;  /*!< [b31~b31]*/
        }BIT;
    }CFGH;                               /*!< 0020H */
    union {
        __IO  uint32_t  V32;                                /*!< 0024H */
        struct {
            __IO  uint32_t  SN0       : 2;  /*!< [b1~b0]*/
            __IO  uint32_t  PS0       : 2;  /*!< [b3~b2]*/
            __IO  uint32_t  SN1       : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  PS1       : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  SN2       : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  PS2       : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  SN3       : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PS3       : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  SN4       : 2;  /*!< [b17~b16]*/
            __IO  uint32_t  PS4       : 2;  /*!< [b19~b18]*/
            __IO  uint32_t  SN5       : 2;  /*!< [b21~b20]*/
            __IO  uint32_t  PS5       : 2;  /*!< [b23~b22]*/
            __IO  uint32_t  SN6       : 2;  /*!< [b25~b24]*/
            __IO  uint32_t  PS6       : 2;  /*!< [b27~b26]*/
            __IO  uint32_t  SN7       : 2;  /*!< [b29~b28]*/
            __IO  uint32_t  PS7       : 2;  /*!< [b31~b30]*/
        }BIT;
    }SAMPL;                               /*!< 0024H */
    union {
        __IO  uint32_t  V32;                                /*!< 0028H */
        struct {
            __IO  uint32_t  SN8       : 2;  /*!< [b1~b0]*/
            __IO  uint32_t  PS8       : 2;  /*!< [b3~b2]*/
            __IO  uint32_t  SN9       : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  PS9       : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  SN10      : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  PS10      : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  SN11      : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PS11      : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  SN12      : 2;  /*!< [b17~b16]*/
            __IO  uint32_t  PS12      : 2;  /*!< [b19~b18]*/
            __IO  uint32_t  SN13      : 2;  /*!< [b21~b20]*/
            __IO  uint32_t  PS13      : 2;  /*!< [b23~b22]*/
            __IO  uint32_t  SN14      : 2;  /*!< [b25~b24]*/
            __IO  uint32_t  PS14      : 2;  /*!< [b27~b26]*/
            __IO  uint32_t  SN15      : 2;  /*!< [b29~b28]*/
            __IO  uint32_t  PS15      : 2;  /*!< [b31~b30]*/
        }BIT;
    }SAMPH;                               /*!< 0028H */
    __IO uint16_t  DMR;                                     /*!< 002CH */
         uint16_t  Reserved6;                               /*!< 002EH */
}EXTI_TypeDef;


/**
*@brief DMA_registers
*/
typedef struct
{
    union {
        __I   uint32_t  V32;                                /*!< 0000H */
        struct {
    __I  uint8_t   BEIF;                                    /*!< 0000H */
    __I  uint8_t   TCIF;                                    /*!< 0001H */
    __I  uint8_t   HTIF;                                    /*!< 0002H */
    __I  uint8_t   TEIF;                                    /*!< 0003H */
        }BIT;
    }IFSR;                              /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
    __O  uint8_t   CBEIF;                                   /*!< 0004H */
    __O  uint8_t   CTCIF;                                   /*!< 0005H */
    __O  uint8_t   CHTIF;                                   /*!< 0006H */
    __O  uint8_t   CTEIF;                                   /*!< 0007H */
        }BIT;
    }IFCR;                              /*!< 0004H */
    union {
        __IO  uint32_t  V32;                                /*!< 0008H */
        struct {
            __IO  uint32_t  SWTRG     : 8;  /*!< [b7~b0]*/
            __I   uint32_t  DBUSY     : 8;  /*!< [b15~b8]*/
            __IO  uint32_t  BURSTIDLE : 4;  /*!< [b19~b16]*/
                  uint32_t  rev0      : 4;  /*!< [b23~b20]*/
            __IO  uint32_t  RELOAD    : 8;  /*!< [b31~b24]*/
        }BIT;
    }CSR;                               /*!< 0008H */
         uint32_t  Reserved0[1];                            /*!< 000CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0010H */
        struct {
            __IO  uint32_t  EN        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TCIE      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  HTIE      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  BEIE      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  TEIE      : 1;  /*!< [b4~b4]*/
                  uint32_t  rev0      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  DPTYP     : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  SPTYP     : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  DSIZE     : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  SSIZE     : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PL        : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  BURSTLEN  : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  STRMSEL   : 3;  /*!< [b22~b20]*/
            __IO  uint32_t  TRGMODE   : 1;  /*!< [b23~b23]*/
                  uint32_t  rev1      : 8;  /*!< [b31~b24]*/
        }BIT;
    }CCR0;                               /*!< 0010H */
    __IO uint16_t  NPKT0;                                   /*!< 0014H */
         uint8_t   Reserved1[2];                            /*!< 0016H */
    __I  uint16_t  CPKT0;                                   /*!< 0018H */
         uint8_t   Reserved2[2];                            /*!< 001AH */
    __IO uint32_t  SAR0;                                    /*!< 001CH */
    __IO uint32_t  DAR0;                                    /*!< 0020H */
         uint32_t  Reserved3[3];                            /*!< 0024H */
    union {
        __IO  uint32_t  V32;                                /*!< 0030H */
        struct {
            __IO  uint32_t  EN        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TCIE      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  HTIE      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  BEIE      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  TEIE      : 1;  /*!< [b4~b4]*/
                  uint32_t  rev0      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  DPTYP     : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  SPTYP     : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  DSIZE     : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  SSIZE     : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PL        : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  BURSTLEN  : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  STRMSEL   : 3;  /*!< [b22~b20]*/
            __IO  uint32_t  TRGMODE   : 1;  /*!< [b23~b23]*/
                  uint32_t  rev1      : 8;  /*!< [b31~b24]*/
        }BIT;
    }CCR1;                               /*!< 0030H */
    __IO uint16_t  NPKT1;                                   /*!< 0034H */
         uint8_t   Reserved4[2];                            /*!< 0036H */
    __I  uint16_t  CPKT1;                                   /*!< 0038H */
         uint8_t   Reserved5[2];                            /*!< 003AH */
    __IO uint32_t  SAR1;                                    /*!< 003CH */
    __IO uint32_t  DAR1;                                    /*!< 0040H */
         uint32_t  Reserved6[3];                            /*!< 0044H */
    union {
        __IO  uint32_t  V32;                                /*!< 0050H */
        struct {
            __IO  uint32_t  EN        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TCIE      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  HTIE      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  BEIE      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  TEIE      : 1;  /*!< [b4~b4]*/
                  uint32_t  rev0      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  DPTYP     : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  SPTYP     : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  DSIZE     : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  SSIZE     : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PL        : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  BURSTLEN  : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  STRMSEL   : 3;  /*!< [b22~b20]*/
            __IO  uint32_t  TRGMODE   : 1;  /*!< [b23~b23]*/
                  uint32_t  rev1      : 8;  /*!< [b31~b24]*/
        }BIT;
    }CCR2;                               /*!< 0050H */
    __IO uint16_t  NPKT2;                                   /*!< 0054H */
         uint8_t   Reserved7[2];                            /*!< 0056H */
    __I  uint16_t  CPKT2;                                   /*!< 0058H */
         uint8_t   Reserved8[2];                            /*!< 005AH */
    __IO uint32_t  SAR2;                                    /*!< 005CH */
    __IO uint32_t  DAR2;                                    /*!< 0060H */
         uint32_t  Reserved9[3];                            /*!< 0064H */
    union {
        __IO  uint32_t  V32;                                /*!< 0070H */
        struct {
            __IO  uint32_t  EN        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TCIE      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  HTIE      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  BEIE      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  TEIE      : 1;  /*!< [b4~b4]*/
                  uint32_t  rev0      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  DPTYP     : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  SPTYP     : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  DSIZE     : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  SSIZE     : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PL        : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  BURSTLEN  : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  STRMSEL   : 3;  /*!< [b22~b20]*/
            __IO  uint32_t  TRGMODE   : 1;  /*!< [b23~b23]*/
                  uint32_t  rev1      : 8;  /*!< [b31~b24]*/
        }BIT;
    }CCR3;                               /*!< 0070H */
    __IO uint16_t  NPKT3;                                   /*!< 0074H */
         uint8_t   Reserved10[2];                           /*!< 0076H */
    __I  uint16_t  CPKT3;                                   /*!< 0078H */
         uint8_t   Reserved11[2];                           /*!< 007AH */
    __IO uint32_t  SAR3;                                    /*!< 007CH */
    __IO uint32_t  DAR3;                                    /*!< 0080H */
         uint32_t  Reserved12[3];                           /*!< 0084H */
    union {
        __IO  uint32_t  V32;                                /*!< 0090H */
        struct {
            __IO  uint32_t  EN        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TCIE      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  HTIE      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  BEIE      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  TEIE      : 1;  /*!< [b4~b4]*/
                  uint32_t  rev0      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  DPTYP     : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  SPTYP     : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  DSIZE     : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  SSIZE     : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PL        : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  BURSTLEN  : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  STRMSEL   : 3;  /*!< [b22~b20]*/
            __IO  uint32_t  TRGMODE   : 1;  /*!< [b23~b23]*/
                  uint32_t  rev1      : 8;  /*!< [b31~b24]*/
        }BIT;
    }CCR4;                               /*!< 0090H */
    __IO uint16_t  NPKT4;                                   /*!< 0094H */
         uint8_t   Reserved13[2];                           /*!< 0096H */
    __I  uint16_t  CPKT4;                                   /*!< 0098H */
         uint8_t   Reserved14[2];                           /*!< 009AH */
    __IO uint32_t  SAR4;                                    /*!< 009CH */
    __IO uint32_t  DAR4;                                    /*!< 00A0H */
         uint32_t  Reserved15[3];                           /*!< 00A4H */
    union {
        __IO  uint32_t  V32;                                /*!< 00B0H */
        struct {
            __IO  uint32_t  EN        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TCIE      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  HTIE      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  BEIE      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  TEIE      : 1;  /*!< [b4~b4]*/
                  uint32_t  rev0      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  DPTYP     : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  SPTYP     : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  DSIZE     : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  SSIZE     : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PL        : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  BURSTLEN  : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  STRMSEL   : 3;  /*!< [b22~b20]*/
            __IO  uint32_t  TRGMODE   : 1;  /*!< [b23~b23]*/
                  uint32_t  rev1      : 8;  /*!< [b31~b24]*/
        }BIT;
    }CCR5;                               /*!< 00B0H */
    __IO uint16_t  NPKT5;                                   /*!< 00B4H */
         uint8_t   Reserved16[2];                           /*!< 00B6H */
    __I  uint16_t  CPKT5;                                   /*!< 00B8H */
         uint8_t   Reserved17[2];                           /*!< 00BAH */
    __IO uint32_t  SAR5;                                    /*!< 00BCH */
    __IO uint32_t  DAR5;                                    /*!< 00C0H */
         uint32_t  Reserved18[3];                           /*!< 00C4H */
    union {
        __IO  uint32_t  V32;                                /*!< 00D0H */
        struct {
            __IO  uint32_t  EN        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TCIE      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  HTIE      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  BEIE      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  TEIE      : 1;  /*!< [b4~b4]*/
                  uint32_t  rev0      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  DPTYP     : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  SPTYP     : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  DSIZE     : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  SSIZE     : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PL        : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  BURSTLEN  : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  STRMSEL   : 3;  /*!< [b22~b20]*/
            __IO  uint32_t  TRGMODE   : 1;  /*!< [b23~b23]*/
                  uint32_t  rev1      : 8;  /*!< [b31~b24]*/
        }BIT;
    }CCR6;                               /*!< 00D0H */
    __IO uint16_t  NPKT6;                                   /*!< 00D4H */
         uint8_t   Reserved19[2];                           /*!< 00D6H */
    __I  uint16_t  CPKT6;                                   /*!< 00D8H */
         uint8_t   Reserved20[2];                           /*!< 00DAH */
    __IO uint32_t  SAR6;                                    /*!< 00DCH */
    __IO uint32_t  DAR6;                                    /*!< 00E0H */
         uint32_t  Reserved21[3];                           /*!< 00E4H */
    union {
        __IO  uint32_t  V32;                                /*!< 00F0H */
        struct {
            __IO  uint32_t  EN        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TCIE      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  HTIE      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  BEIE      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  TEIE      : 1;  /*!< [b4~b4]*/
                  uint32_t  rev0      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  DPTYP     : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  SPTYP     : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  DSIZE     : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  SSIZE     : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  PL        : 2;  /*!< [b15~b14]*/
            __IO  uint32_t  BURSTLEN  : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  STRMSEL   : 3;  /*!< [b22~b20]*/
            __IO  uint32_t  TRGMODE   : 1;  /*!< [b23~b23]*/
                  uint32_t  rev1      : 8;  /*!< [b31~b24]*/
        }BIT;
    }CCR7;                               /*!< 00F0H */
    __IO uint16_t  NPKT7;                                   /*!< 00F4H */
         uint8_t   Reserved22[2];                           /*!< 00F6H */
    __I  uint16_t  CPKT7;                                   /*!< 00F8H */
         uint8_t   Reserved23[2];                           /*!< 00FAH */
    __IO uint32_t  SAR7;                                    /*!< 00FCH */
    __IO uint32_t  DAR7;                                    /*!< 0100H */
}DMA_TypeDef;


/**
*@brief MCM_registers
*/
typedef struct
{
    __IO uint8_t   PWMOE;                                   /*!< 0000H */
         uint8_t   Reserved0[3];                            /*!< 0001H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __IO  uint32_t  PWM0S     : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  PWM1S     : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  PWM2S     : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  PWM01S    : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  PWM11S    : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  PWM21S    : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  PDCON0    : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  PDCON1    : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  PDCON2    : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  PWMSYM    : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  PTMOD     : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  POSTPS    : 3;  /*!< [b14~b12]*/
            __IO  uint32_t  POUTMOD   : 1;  /*!< [b15~b15]*/
                  uint32_t  rev0      :16;  /*!< [b31~b16]*/
        }BIT;
    }PWMCON1;                               /*!< 0004H */
    union {
        __IO  uint32_t  V32;                                /*!< 0008H */
        struct {
            __IO  uint32_t  CMP1      : 2;  /*!< [b1~b0]*/
            __IO  uint32_t  CMP2      : 2;  /*!< [b3~b2]*/
            __IO  uint32_t  CMP3      : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  CMP4      : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  OSYNC     : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  DILDEN    : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  CILDEN    : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  ZDLDEN    : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  PDLDEN    : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  ZCMLDEN   : 1;  /*!< [b13~b13]*/
            __IO  uint32_t  PCMLDEN   : 1;  /*!< [b14~b14]*/
            __IO  uint32_t  MCMSYNEN  : 1;  /*!< [b15~b15]*/
                  uint32_t  rev0      :16;  /*!< [b31~b16]*/
        }BIT;
    }PWMCON2;                               /*!< 0008H */
    __IO uint16_t  PWMP;                                    /*!< 000CH */
         uint16_t  Reserved1;                               /*!< 000EH */
    __IO uint16_t  PWMC;                                    /*!< 0010H */
         uint16_t  Reserved2;                               /*!< 0012H */
    __IO uint16_t  PWMPSQ;                                  /*!< 0014H */
         uint16_t  Reserved3;                               /*!< 0016H */
    __IO uint16_t  PWM0D;                                   /*!< 0018H */
         uint16_t  Reserved4;                               /*!< 001AH */
    __IO uint16_t  PWM1D;                                   /*!< 001CH */
         uint16_t  Reserved5;                               /*!< 001EH */
    __IO uint16_t  PWM2D;                                   /*!< 0020H */
         uint16_t  Reserved6;                               /*!< 0022H */
    __IO uint16_t  PWM01D;                                  /*!< 0024H */
         uint16_t  Reserved7;                               /*!< 0026H */
    __IO uint16_t  PWM11D;                                  /*!< 0028H */
         uint16_t  Reserved8;                               /*!< 002AH */
    __IO uint16_t  PWM21D;                                  /*!< 002CH */
         uint16_t  Reserved9;                               /*!< 002EH */
    __IO uint16_t  PWMCMP1;                                 /*!< 0030H */
         uint16_t  Reserved10;                              /*!< 0032H */
    __IO uint16_t  PWMCMP2;                                 /*!< 0034H */
         uint16_t  Reserved11;                              /*!< 0036H */
    __IO uint16_t  PWMCMP3;                                 /*!< 0038H */
         uint16_t  Reserved12;                              /*!< 003AH */
    __IO uint16_t  PWMCMP4;                                 /*!< 003CH */
         uint16_t  Reserved13;                              /*!< 003EH */
    __IO uint16_t  PWMDT00;                                 /*!< 0040H */
         uint16_t  Reserved14;                              /*!< 0042H */
    __IO uint16_t  PWMDT01;                                 /*!< 0044H */
         uint16_t  Reserved15;                              /*!< 0046H */
    __IO uint16_t  PWMDT10;                                 /*!< 0048H */
         uint16_t  Reserved16;                              /*!< 004AH */
    __IO uint16_t  PWMDT11;                                 /*!< 004CH */
         uint16_t  Reserved17;                              /*!< 004EH */
    __IO uint16_t  PWMDT20;                                 /*!< 0050H */
         uint16_t  Reserved18;                              /*!< 0052H */
    __IO uint16_t  PWMDT21;                                 /*!< 0054H */
         uint16_t  Reserved19;                              /*!< 0056H */
    union {
        __IO  uint32_t  V32;                                /*!< 0058H */
        struct {
            __IO  uint32_t  PMANUAL0  : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  PMANUAL1  : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  PMANUAL2  : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  PMANUAL01 : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  PMANUAL11 : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  PMANUAL21 : 1;  /*!< [b5~b5]*/
                  uint32_t  rev0      :26;  /*!< [b31~b6]*/
        }BIT;
    }PMANUALCON1;                               /*!< 0058H */
    union {
        __IO  uint32_t  V32;                                /*!< 005CH */
        struct {
            __IO  uint32_t  POUT0     : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  POUT1     : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  POUT2     : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  POUT01    : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  POUT11    : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  POUT21    : 1;  /*!< [b5~b5]*/
                  uint32_t  rev0      :26;  /*!< [b31~b6]*/
        }BIT;
    }PMANUALCON2;                               /*!< 005CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0060H */
        struct {
            __IO  uint32_t  FLTSTAT   : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  FLTM      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  FLT2S     : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  FLT2EN    : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  FLT2DEB   : 4;  /*!< [b7~b4]*/
            __IO  uint32_t  FLT1SEL   : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  FLT1EN    : 1;  /*!< [b10~b10]*/
                  uint32_t  rev0      : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  FOUT0     : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  FOUT1     : 2;  /*!< [b15~b14]*/
                  uint32_t  rev1      :16;  /*!< [b31~b16]*/
        }BIT;
    }FLTCON;                               /*!< 0060H */
    union {
        __IO  uint32_t  V32;                                /*!< 0064H */
        struct {
            __IO  uint32_t  OLSG0     : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  OLSG1     : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  OLSG2     : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  OLSG01    : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  OLSG11    : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  OLSG21    : 1;  /*!< [b5~b5]*/
                  uint32_t  rev0      : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  OLSEN     : 1;  /*!< [b7~b7]*/
                  uint32_t  rev1      :24;  /*!< [b31~b8]*/
        }BIT;
    }POSCR;                               /*!< 0064H */
    union {
        __IO  uint32_t  V32;                                /*!< 0068H */
        struct {
                  uint32_t  rev0      : 7;  /*!< [b6~b0]*/
            __IO  uint32_t  OSTDEN    : 1;  /*!< [b7~b7]*/
                  uint32_t  rev1      :24;  /*!< [b31~b8]*/
        }BIT;
    }POSTDCR;                               /*!< 0068H */
    union {
        __IO  uint32_t  V32;                                /*!< 006CH */
        struct {
            __IO  uint32_t  PTUD0DE   : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  PTDD0DE   : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  PTUD1DE   : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  PTDD1DE   : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  PTUD2DE   : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  PTDD2DE   : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  PWMZDE    : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  PWMPDE    : 1;  /*!< [b7~b7]*/
                  uint32_t  rev0      :24;  /*!< [b31~b8]*/
        }BIT;
    }PWMDMAEN;                               /*!< 006CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0070H */
        struct {
            __IO  uint32_t  PTUD0IE   : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  PTDD0IE   : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  PTUD1IE   : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  PTDD1IE   : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  PTUD2IE   : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  PTDD2IE   : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  PWMZIE    : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  PWMPIE    : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  FLTIE     : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  OIE       : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  OSTDIE    : 1;  /*!< [b10~b10]*/
                  uint32_t  rev0      :21;  /*!< [b31~b11]*/
        }BIT;
    }PWMINTEN;                               /*!< 0070H */
    union {
        __IO  uint32_t  V32;                                /*!< 0074H */
        struct {
            __I   uint32_t  PTUD0IF   : 1;  /*!< [b0~b0]*/
            __I   uint32_t  PTDD0IF   : 1;  /*!< [b1~b1]*/
            __I   uint32_t  PTUD1IF   : 1;  /*!< [b2~b2]*/
            __I   uint32_t  PTDD1IF   : 1;  /*!< [b3~b3]*/
            __I   uint32_t  PTUD2IF   : 1;  /*!< [b4~b4]*/
            __I   uint32_t  PTDD2IF   : 1;  /*!< [b5~b5]*/
            __I   uint32_t  PWMZIF    : 1;  /*!< [b6~b6]*/
            __I   uint32_t  PWMPIF    : 1;  /*!< [b7~b7]*/
            __I   uint32_t  FLTIF     : 1;  /*!< [b8~b8]*/
            __I   uint32_t  OSF       : 1;  /*!< [b9~b9]*/
            __I   uint32_t  OSTDF     : 1;  /*!< [b10~b10]*/
            __I   uint32_t  SC1STAT   : 1;  /*!< [b11~b11]*/
            __I   uint32_t  SC2STAT   : 1;  /*!< [b12~b12]*/
            __I   uint32_t  SC3STAT   : 1;  /*!< [b13~b13]*/
                  uint32_t  rev0      : 2;  /*!< [b15~b14]*/
            __O   uint32_t  PTUD0IFC  : 1;  /*!< [b16~b16]*/
            __O   uint32_t  PTDD0IFC  : 1;  /*!< [b17~b17]*/
            __O   uint32_t  PTUD1IFC  : 1;  /*!< [b18~b18]*/
            __O   uint32_t  PTDD1IFC  : 1;  /*!< [b19~b19]*/
            __O   uint32_t  PTUD2IFC  : 1;  /*!< [b20~b20]*/
            __O   uint32_t  PTDD2IFC  : 1;  /*!< [b21~b21]*/
            __O   uint32_t  PWMZIFC   : 1;  /*!< [b22~b22]*/
            __O   uint32_t  PWMPIFC   : 1;  /*!< [b23~b23]*/
            __O   uint32_t  FLTIFC    : 1;  /*!< [b24~b24]*/
            __O   uint32_t  OSFC      : 1;  /*!< [b25~b25]*/
            __O   uint32_t  OSTDFC    : 1;  /*!< [b26~b26]*/
            __O   uint32_t  SC1STATC  : 1;  /*!< [b27~b27]*/
            __O   uint32_t  SC2STATC  : 1;  /*!< [b28~b28]*/
            __O   uint32_t  SC3STATC  : 1;  /*!< [b29~b29]*/
                  uint32_t  rev1      : 2;  /*!< [b31~b30]*/
        }BIT;
    }PWMINTF;                               /*!< 0074H */
    __IO uint8_t   PWMRLDEN;                                /*!< 0078H */
         uint8_t   Reserved20[3];                           /*!< 0079H */
    union {
        __IO  uint32_t  V32;                                /*!< 007CH */
        struct {
            __IO  uint32_t  SHIFTRUN  : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  SECTOR    : 3;  /*!< [b3~b1]*/
            __IO  uint32_t  MAXSELECT : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  MINSELECT : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  STATRUN   : 1;  /*!< [b8~b8]*/
                  uint32_t  rev0      :23;  /*!< [b31~b9]*/
        }BIT;
    }PSCON;                               /*!< 007CH */
    __IO uint16_t  PWMDMAX;                                 /*!< 0080H */
         uint16_t  Reserved21;                              /*!< 0082H */
    __IO uint16_t  PWMDMIN;                                 /*!< 0084H */
         uint16_t  Reserved22;                              /*!< 0086H */
    __IO uint16_t  PWMDCMP1;                                /*!< 0088H */
         uint16_t  Reserved23;                              /*!< 008AH */
    __IO uint16_t  PWMDCMP2;                                /*!< 008CH */
         uint16_t  Reserved24;                              /*!< 008EH */
    union {
        __IO  uint32_t  V32;                                /*!< 0090H */
        struct {
            __IO  uint32_t  SCTIME    : 4;  /*!< [b3~b0]*/
            __IO  uint32_t  SCDEB     : 4;  /*!< [b7~b4]*/
            __IO  uint32_t  SCPWM0EN  : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  SCPWM1EN  : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  SCPWM2EN  : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  SCS       : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  SCEN      : 1;  /*!< [b12~b12]*/
                  uint32_t  rev0      :19;  /*!< [b31~b13]*/
        }BIT;
    }SC1CON;                               /*!< 0090H */
    union {
        __IO  uint32_t  V32;                                /*!< 0094H */
        struct {
            __IO  uint32_t  SCTIME    : 4;  /*!< [b3~b0]*/
            __IO  uint32_t  SCDEB     : 4;  /*!< [b7~b4]*/
            __IO  uint32_t  SCPWM0EN  : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  SCPWM1EN  : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  SCPWM2EN  : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  SCS       : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  SCEN      : 1;  /*!< [b12~b12]*/
                  uint32_t  rev0      :19;  /*!< [b31~b13]*/
        }BIT;
    }SC2CON;                               /*!< 0094H */
    union {
        __IO  uint32_t  V32;                                /*!< 0098H */
        struct {
            __IO  uint32_t  SCTIME    : 4;  /*!< [b3~b0]*/
            __IO  uint32_t  SCDEB     : 4;  /*!< [b7~b4]*/
            __IO  uint32_t  SCPWM0EN  : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  SCPWM1EN  : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  SCPWM2EN  : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  SCS       : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  SCEN      : 1;  /*!< [b12~b12]*/
                  uint32_t  rev0      :19;  /*!< [b31~b13]*/
        }BIT;
    }SC3CON;                               /*!< 0098H */
    __IO uint16_t  FLTWEN;                                  /*!< 009CH */
         uint16_t  Reserved25;                              /*!< 009EH */
}MCM_TypeDef;


/**
*@brief GPT_GLOBAL_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  CST0      : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  CST1      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  CST2      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  CST3      : 1;  /*!< [b3~b3]*/
                  uint32_t  rev0      :28;  /*!< [b31~b4]*/
        }BIT;
    }GTSTR;                               /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __IO  uint32_t  ETIPEN    : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  ETINEN    : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  ETPDMA    : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  ETNDMA    : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  SN        : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  PS        : 2;  /*!< [b7~b6]*/
                  uint32_t  rev0      :24;  /*!< [b31~b8]*/
        }BIT;
    }GTETINT;                               /*!< 0004H */
    union {
        __IO  uint32_t  V32;                                /*!< 0008H */
        struct {
            __IO  uint32_t  GPT0ABZE0 : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  GPT1ABZE0 : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  GPT2ABZE0 : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  GPT3ABZE0 : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  POEM      : 2;  /*!< [b5~b4]*/
                  uint32_t  rev0      : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  POEIE     : 1;  /*!< [b7~b7]*/
                  uint32_t  rev1      :24;  /*!< [b31~b8]*/
        }BIT;
    }GTPOECR;                               /*!< 0008H */
         uint32_t  Reserved0[4];                            /*!< 000CH */
    __IO uint16_t  GTPRWEN;                                 /*!< 001CH */
         uint16_t  Reserved1;                               /*!< 001EH */
    union {
        __IO  uint32_t  V32;                                /*!< 0020H */
        struct {
            __I   uint32_t  ETIPF     : 1;  /*!< [b0~b0]*/
            __I   uint32_t  ETINF     : 1;  /*!< [b1~b1]*/
            __I   uint32_t  POEIF     : 1;  /*!< [b2~b2]*/
                  uint32_t  rev0      :13;  /*!< [b15~b3]*/
            __O   uint32_t  ETIPFC    : 1;  /*!< [b16~b16]*/
            __O   uint32_t  ETINFC    : 1;  /*!< [b17~b17]*/
            __O   uint32_t  POEIFC    : 1;  /*!< [b18~b18]*/
                  uint32_t  rev1      :13;  /*!< [b31~b19]*/
        }BIT;
    }GTINTF;                               /*!< 0020H */
}GPT_GLOBAL_TypeDef;


/**
*@brief GPT_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  CSHW      : 2;  /*!< [b1~b0]*/
            __IO  uint32_t  CPHW      : 2;  /*!< [b3~b2]*/
            __IO  uint32_t  CCHW      : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  SYNC      : 2;  /*!< [b7~b6]*/
            __O   uint32_t  CCSW      : 1;  /*!< [b8~b8]*/
                  uint32_t  rev0      :23;  /*!< [b31~b9]*/
        }BIT;
    }GTHCR;                               /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __IO  uint32_t  DEB       : 4;  /*!< [b3~b0]*/
                  uint32_t  rev0      : 4;  /*!< [b7~b4]*/
            __IO  uint32_t  INAE      : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  INBE      : 1;  /*!< [b9~b9]*/
                  uint32_t  rev1      :22;  /*!< [b31~b10]*/
        }BIT;
    }GTDEB;                               /*!< 0004H */
    __IO uint8_t   GTWP;                                    /*!< 0008H */
         uint8_t   Reserved0[3];                            /*!< 0009H */
    union {
        __IO  uint32_t  V32;                                /*!< 000CH */
        struct {
            __IO  uint32_t  BDCCR     : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  BDPR      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  BDADTR    : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  BDDV      : 1;  /*!< [b3~b3]*/
                  uint32_t  rev0      :28;  /*!< [b31~b4]*/
        }BIT;
    }GTBDR;                               /*!< 000CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0010H */
        struct {
            __IO  uint32_t  GTIOA     : 6;  /*!< [b5~b0]*/
            __IO  uint32_t  OADFLT    : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  OAHLD     : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  GTIOB     : 6;  /*!< [b13~b8]*/
            __IO  uint32_t  OBDFLT    : 1;  /*!< [b14~b14]*/
            __IO  uint32_t  OBHLD     : 1;  /*!< [b15~b15]*/
                  uint32_t  rev0      :16;  /*!< [b31~b16]*/
        }BIT;
    }GTIOR;                               /*!< 0010H */
    union {
        __IO  uint32_t  V32;                                /*!< 0014H */
        struct {
            __IO  uint32_t  GTINTA    : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  GTINTB    : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  GTINTPR   : 2;  /*!< [b3~b2]*/
            __IO  uint32_t  EINT      : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  OINT      : 1;  /*!< [b5~b5]*/
                  uint32_t  rev0      : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  ADTRAUEN  : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  ADTRADEN  : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  ADTRBUEN  : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  ADTRBDEN  : 1;  /*!< [b11~b11]*/
                  uint32_t  rev1      :20;  /*!< [b31~b12]*/
        }BIT;
    }GTINTAD;                               /*!< 0014H */
    union {
        __IO  uint32_t  V32;                                /*!< 0018H */
        struct {
            __IO  uint32_t  TA        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TB        : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  PR        : 2;  /*!< [b3~b2]*/
                  uint32_t  rev0      :28;  /*!< [b31~b4]*/
        }BIT;
    }GTDMA;                               /*!< 0018H */
    union {
        __IO  uint32_t  V32;                                /*!< 001CH */
        struct {
            __IO  uint32_t  MD        : 3;  /*!< [b2~b0]*/
                  uint32_t  rev0      : 5;  /*!< [b7~b3]*/
            __IO  uint32_t  TPSC      : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  HIZ       : 1;  /*!< [b9~b9]*/
                  uint32_t  rev1      : 2;  /*!< [b11~b10]*/
            __IO  uint32_t  CCLR      : 2;  /*!< [b13~b12]*/
                  uint32_t  rev2      :18;  /*!< [b31~b14]*/
        }BIT;
    }GTCR;                               /*!< 001CH */
    __IO uint16_t  GTPSQ;                                   /*!< 0020H */
         uint16_t  Reserved1;                               /*!< 0022H */
    union {
        __IO  uint32_t  V32;                                /*!< 0024H */
        struct {
            __IO  uint32_t  CCRA      : 2;  /*!< [b1~b0]*/
            __IO  uint32_t  CCRB      : 2;  /*!< [b3~b2]*/
            __IO  uint32_t  PR        : 2;  /*!< [b5~b4]*/
            __O   uint32_t  CCRSWT    : 1;  /*!< [b6~b6]*/
                  uint32_t  rev0      : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  ADTTA     : 2;  /*!< [b9~b8]*/
            __IO  uint32_t  ADTDA     : 1;  /*!< [b10~b10]*/
                  uint32_t  rev1      : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  ADTTB     : 2;  /*!< [b13~b12]*/
            __IO  uint32_t  ADTDB     : 1;  /*!< [b14~b14]*/
                  uint32_t  rev2      :17;  /*!< [b31~b15]*/
        }BIT;
    }GTBER;                               /*!< 0024H */
    union {
        __IO  uint32_t  V32;                                /*!< 0028H */
        struct {
            __IO  uint32_t  UD        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  UDF       : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      :30;  /*!< [b31~b2]*/
        }BIT;
    }GTUDC;                               /*!< 0028H */
    union {
        __IO  uint32_t  V32;                                /*!< 002CH */
        struct {
            __IO  uint32_t  ITLA      : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  ITLB      : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      : 2;  /*!< [b3~b2]*/
            __IO  uint32_t  IVTC      : 2;  /*!< [b5~b4]*/
                  uint32_t  rev1      : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  IVTT      : 3;  /*!< [b10~b8]*/
                  uint32_t  rev2      : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  ADTAL     : 1;  /*!< [b12~b12]*/
                  uint32_t  rev3      : 1;  /*!< [b13~b13]*/
            __IO  uint32_t  ADTBL     : 1;  /*!< [b14~b14]*/
                  uint32_t  rev4      :17;  /*!< [b31~b15]*/
        }BIT;
    }GTITC;                               /*!< 002CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0030H */
        struct {
            __I   uint32_t  TCFA      : 1;  /*!< [b0~b0]*/
            __I   uint32_t  TCFB      : 1;  /*!< [b1~b1]*/
            __I   uint32_t  TCFPO     : 1;  /*!< [b2~b2]*/
            __I   uint32_t  TCFPU     : 1;  /*!< [b3~b3]*/
            __I   uint32_t  DTEF      : 1;  /*!< [b4~b4]*/
            __I   uint32_t  OSF       : 1;  /*!< [b5~b5]*/
                  uint32_t  rev0      : 2;  /*!< [b7~b6]*/
            __I   uint32_t  ITCNT     : 3;  /*!< [b10~b8]*/
                  uint32_t  rev1      : 4;  /*!< [b14~b11]*/
            __I   uint32_t  TUCF      : 1;  /*!< [b15~b15]*/
            __O   uint32_t  TCFAC     : 1;  /*!< [b16~b16]*/
            __O   uint32_t  TCFBC     : 1;  /*!< [b17~b17]*/
            __O   uint32_t  TCFPOC    : 1;  /*!< [b18~b18]*/
            __O   uint32_t  TCFPUC    : 1;  /*!< [b19~b19]*/
            __O   uint32_t  DTEFC     : 1;  /*!< [b20~b20]*/
            __O   uint32_t  OSFC      : 1;  /*!< [b21~b21]*/
                  uint32_t  rev2      :10;  /*!< [b31~b22]*/
        }BIT;
    }GTST;                               /*!< 0030H */
    union {
        __IO  uint32_t  V32;                                /*!< 0034H */
        struct {
    __IO uint16_t  TCNTL;                                   /*!< 0034H */
    __IO uint16_t  TCNTH;                                   /*!< 0036H */
        }BIT;
    }GTCNT;                              /*!< 0034H */
    __IO uint32_t  GTCCRA;                                  /*!< 0038H */
    __IO uint32_t  GTCCRB;                                  /*!< 003CH */
    __IO uint32_t  GTCCRC;                                  /*!< 0040H */
    __IO uint32_t  GTCCRD;                                  /*!< 0044H */
    __IO uint32_t  GTCCRE;                                  /*!< 0048H */
    __IO uint32_t  GTCCRF;                                  /*!< 004CH */
    __IO uint16_t  GTPR;                                    /*!< 0050H */
         uint16_t  Reserved2;                               /*!< 0052H */
    __IO uint16_t  GTPBR;                                   /*!< 0054H */
         uint16_t  Reserved3;                               /*!< 0056H */
    __IO uint16_t  GTPDBR;                                  /*!< 0058H */
         uint16_t  Reserved4;                               /*!< 005AH */
    __IO uint16_t  GTADTRA;                                 /*!< 005CH */
         uint16_t  Reserved5;                               /*!< 005EH */
    __IO uint16_t  GTADTRB;                                 /*!< 0060H */
         uint16_t  Reserved6;                               /*!< 0062H */
    __IO uint16_t  GTADTBRA;                                /*!< 0064H */
         uint16_t  Reserved7;                               /*!< 0066H */
    __IO uint16_t  GTADTBRB;                                /*!< 0068H */
         uint16_t  Reserved8;                               /*!< 006AH */
    __IO uint16_t  GTADTDBRA;                               /*!< 006CH */
         uint16_t  Reserved9;                               /*!< 006EH */
    __IO uint16_t  GTADTDBRB;                               /*!< 0070H */
         uint16_t  Reserved10;                              /*!< 0072H */
    union {
        __IO  uint32_t  V32;                                /*!< 0074H */
        struct {
            __IO  uint32_t  TDE       : 1;  /*!< [b0~b0]*/
                  uint32_t  rev0      : 3;  /*!< [b3~b1]*/
            __IO  uint32_t  TDBUE     : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  TDBDE     : 1;  /*!< [b5~b5]*/
                  uint32_t  rev1      : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  TDFER     : 1;  /*!< [b8~b8]*/
                  uint32_t  rev2      :23;  /*!< [b31~b9]*/
        }BIT;
    }GTDTCR;                               /*!< 0074H */
    __IO uint16_t  GTDVU;                                   /*!< 0078H */
         uint16_t  Reserved11;                              /*!< 007AH */
    __IO uint16_t  GTDVD;                                   /*!< 007CH */
         uint16_t  Reserved12;                              /*!< 007EH */
    __IO uint16_t  GTDBU;                                   /*!< 0080H */
         uint16_t  Reserved13;                              /*!< 0082H */
    __IO uint16_t  GTDBD;                                   /*!< 0084H */
         uint16_t  Reserved14;                              /*!< 0086H */
    union {
        __IO  uint32_t  V32;                                /*!< 0088H */
        struct {
            __IO  uint32_t  OLSGA     : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  OLSGB     : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      : 5;  /*!< [b6~b2]*/
            __IO  uint32_t  OLSEN     : 1;  /*!< [b7~b7]*/
                  uint32_t  rev1      :24;  /*!< [b31~b8]*/
        }BIT;
    }GTOSCR;                               /*!< 0088H */
}GPT_TypeDef;


/**
*@brief TIM_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  STR       : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  rev0      : 2;  /*!< [b2~b1]*/
            __IO  uint32_t  OPM       : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  CLKS      : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  IE        : 1;  /*!< [b6~b6]*/
                  uint32_t  rev1      : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  TRIGEN    : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  ETEN      : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  TC        : 1;  /*!< [b10~b10]*/
                  uint32_t  rev2      : 4;  /*!< [b14~b11]*/
            __IO  uint32_t  CASCEN    : 1;  /*!< [b15~b15]*/
                  uint32_t  rev3      :16;  /*!< [b31~b16]*/
        }BIT;
    }CR;                               /*!< 0000H */
    __IO uint32_t  TCNT;                                    /*!< 0004H */
    __IO uint32_t  TPR;                                     /*!< 0008H */
    __IO uint32_t  PSQ;                                     /*!< 000CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0010H */
        struct {
            __I   uint32_t  TF        : 1;  /*!< [b0~b0]*/
                  uint32_t  rev0      :15;  /*!< [b15~b1]*/
            __O   uint32_t  TFC       : 1;  /*!< [b16~b16]*/
                  uint32_t  rev1      :15;  /*!< [b31~b17]*/
        }BIT;
    }TIMINTF;                               /*!< 0010H */
}TIM_TypeDef;


/**
*@brief MACP_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  RUN       : 1;  /*!< [b0~b0]*/
            __I   uint32_t  OVF       : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  KADJ      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  XYMRS     : 1;  /*!< [b3~b3]*/
                  uint32_t  rev0      : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  FORMAT    : 2;  /*!< [b6~b5]*/
            __IO  uint32_t  MODE      : 1;  /*!< [b7~b7]*/
                  uint32_t  rev1      :24;  /*!< [b31~b8]*/
        }BIT;
    }CORDCSR0;                               /*!< 0000H */
    __IO uint32_t  OPRDX0;                                  /*!< 0004H */
    __IO uint32_t  OPRDY0;                                  /*!< 0008H */
    __IO uint32_t  OPRDZ0;                                  /*!< 000CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0010H */
        struct {
            __IO  uint32_t  RUN       : 1;  /*!< [b0~b0]*/
            __I   uint32_t  OVF       : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  KADJ      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  XYMRS     : 1;  /*!< [b3~b3]*/
                  uint32_t  rev0      : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  FORMAT    : 2;  /*!< [b6~b5]*/
            __IO  uint32_t  MODE      : 1;  /*!< [b7~b7]*/
                  uint32_t  rev1      :24;  /*!< [b31~b8]*/
        }BIT;
    }CORDCSR1;                               /*!< 0010H */
    __IO uint32_t  OPRDX1;                                  /*!< 0014H */
    __IO uint32_t  OPRDY1;                                  /*!< 0018H */
    __IO uint32_t  OPRDZ1;                                  /*!< 001CH */
         uint32_t  Reserved0[56];                           /*!< 0020H */
    union {
        __IO  uint32_t  V32;                                /*!< 0100H */
        struct {
            __IO  uint32_t  RUN       : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  SIGN      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  SAT       : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  Q         : 5;  /*!< [b7~b3]*/
            __IO  uint32_t  CMOD      : 1;  /*!< [b8~b8]*/
                  uint32_t  rev0      :23;  /*!< [b31~b9]*/
        }BIT;
    }IQDIVCSR0;                               /*!< 0100H */
    __IO uint32_t  IQDIVDND0;                               /*!< 0104H */
    __IO uint32_t  IQDIVSOR0;                               /*!< 0108H */
    __IO uint32_t  IQDIVRLT0;                               /*!< 010CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0110H */
        struct {
            __IO  uint32_t  RUN       : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  SIGN      : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  SAT       : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  Q         : 5;  /*!< [b7~b3]*/
            __IO  uint32_t  CMOD      : 1;  /*!< [b8~b8]*/
                  uint32_t  rev0      :23;  /*!< [b31~b9]*/
        }BIT;
    }IQDIVCSR1;                               /*!< 0110H */
    __IO uint32_t  IQDIVDND1;                               /*!< 0114H */
    __IO uint32_t  IQDIVSOR1;                               /*!< 0118H */
    __IO uint32_t  IQDIVRLT1;                               /*!< 011CH */
         uint32_t  Reserved1[56];                           /*!< 0120H */
    __IO uint8_t   SVCON;                                   /*!< 0200H */
         uint8_t   Reserved2[3];                            /*!< 0201H */
    __IO int32_t   SVUALPHA;                                /*!< 0204H */
    __IO int32_t   SVUBETA;                                 /*!< 0208H */
    union {
        __IO  uint32_t  V32;                                /*!< 020CH */
        struct {
            __IO  uint32_t  SVIQN     : 8;  /*!< [b7~b0]*/
            __IO  uint32_t  SVTYP     : 1;  /*!< [b8~b8]*/
                  uint32_t  rev0      :23;  /*!< [b31~b9]*/
        }BIT;
    }SVIQN;                               /*!< 020CH */
    __I  int32_t   SVTA;                                    /*!< 0210H */
    __I  int32_t   SVTB;                                    /*!< 0214H */
    __I  int32_t   SVTC;                                    /*!< 0218H */
    __I  uint8_t   SVSECTOR;                                /*!< 021CH */
         uint8_t   Reserved3[3];                            /*!< 021DH */
}MACP_TypeDef;


/**
*@brief ADC_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  ADSOC     : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  ADCTU     : 2;  /*!< [b2~b1]*/
            __IO  uint32_t  MOD       : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  REFC      : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  ADDE      : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  ADIE      : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  ADSTRS    : 7;  /*!< [b14~b8]*/
            __IO  uint32_t  ADON      : 1;  /*!< [b15~b15]*/
                  uint32_t  rev0      :16;  /*!< [b31~b16]*/
        }BIT;
    }ADCON1;                               /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __IO  uint32_t  TADC      : 4;  /*!< [b3~b0]*/
            __IO  uint32_t  ADMAXCH   : 3;  /*!< [b6~b4]*/
                  uint32_t  rev0      : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  TS        : 4;  /*!< [b11~b8]*/
                  uint32_t  rev1      : 4;  /*!< [b15~b12]*/
            __IO  uint32_t  TGAP      : 3;  /*!< [b18~b16]*/
                  uint32_t  rev2      :13;  /*!< [b31~b19]*/
        }BIT;
    }ADCON2;                               /*!< 0004H */
    __IO uint8_t   ADPCH;                                   /*!< 0008H */
         uint8_t   Reserved0[3];                            /*!< 0009H */
    __I  uint16_t  ADDR0;                                   /*!< 000CH */
         uint16_t  Reserved1;                               /*!< 000EH */
    __I  uint16_t  ADDR1;                                   /*!< 0010H */
         uint16_t  Reserved2;                               /*!< 0012H */
    __I  uint16_t  ADDR2;                                   /*!< 0014H */
         uint16_t  Reserved3;                               /*!< 0016H */
    __I  uint16_t  ADDR3;                                   /*!< 0018H */
         uint16_t  Reserved4;                               /*!< 001AH */
    __I  uint16_t  ADDR4;                                   /*!< 001CH */
         uint16_t  Reserved5;                               /*!< 001EH */
    __I  uint16_t  ADDR5;                                   /*!< 0020H */
         uint16_t  Reserved6;                               /*!< 0022H */
    __I  uint16_t  ADDR6;                                   /*!< 0024H */
         uint16_t  Reserved7;                               /*!< 0026H */
    __I  uint16_t  ADDR7;                                   /*!< 0028H */
         uint16_t  Reserved8;                               /*!< 002AH */
    union {
        __IO  uint32_t  V32;                                /*!< 002CH */
        struct {
            __IO  uint32_t  CSEL      : 3;  /*!< [b2~b0]*/
                  uint32_t  rev0      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  ADLDE     : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  ADLIE     : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  ADGDE     : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  ADGIE     : 1;  /*!< [b7~b7]*/
                  uint32_t  rev1      :24;  /*!< [b31~b8]*/
        }BIT;
    }ADCMPCON;                               /*!< 002CH */
    __IO uint16_t  ADDGT;                                   /*!< 0030H */
         uint16_t  Reserved9;                               /*!< 0032H */
    __IO uint16_t  ADDLT;                                   /*!< 0034H */
         uint16_t  Reserved10;                              /*!< 0036H */
    union {
        __IO  uint32_t  V32;                                /*!< 0038H */
        struct {
            __IO  uint32_t  SEQCH0    : 4;  /*!< [b3~b0]*/
            __IO  uint32_t  SEQCH1    : 4;  /*!< [b7~b4]*/
            __IO  uint32_t  SEQCH2    : 4;  /*!< [b11~b8]*/
            __IO  uint32_t  SEQCH3    : 4;  /*!< [b15~b12]*/
            __IO  uint32_t  SEQCH4    : 4;  /*!< [b19~b16]*/
            __IO  uint32_t  SEQCH5    : 4;  /*!< [b23~b20]*/
            __IO  uint32_t  SEQCH6    : 4;  /*!< [b27~b24]*/
            __IO  uint32_t  SEQCH7    : 4;  /*!< [b31~b28]*/
        }BIT;
    }SEQCHSEL;                               /*!< 0038H */
    __IO uint8_t   ADGAPON;                                 /*!< 003CH */
         uint8_t   Reserved11[3];                           /*!< 003DH */
    union {
        __IO  uint32_t  V32;                                /*!< 0040H */
        struct {
            __I   uint32_t  ADLIF     : 1;  /*!< [b0~b0]*/
            __I   uint32_t  ADGIF     : 1;  /*!< [b1~b1]*/
            __I   uint32_t  ADIF      : 1;  /*!< [b2~b2]*/
                  uint32_t  rev0      :13;  /*!< [b15~b3]*/
            __O   uint32_t  ADLIFC    : 1;  /*!< [b16~b16]*/
            __O   uint32_t  ADGIFC    : 1;  /*!< [b17~b17]*/
            __O   uint32_t  ADIFC     : 1;  /*!< [b18~b18]*/
                  uint32_t  rev1      :13;  /*!< [b31~b19]*/
        }BIT;
    }ADINTF;                               /*!< 0040H */
}ADC_TypeDef;


/**
*@brief QEI_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  QPMOD     : 2;  /*!< [b1~b0]*/
            __IO  uint32_t  QTSR      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  QEIEN     : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  QIDXEN    : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  QSWAP     : 1;  /*!< [b5~b5]*/
            __I   uint32_t  QIDXS     : 1;  /*!< [b6~b6]*/
            __I   uint32_t  QPDIR     : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  QTEPS     : 4;  /*!< [b11~b8]*/
                  uint32_t  rev0      :20;  /*!< [b31~b12]*/
        }BIT;
    }QEICON;                               /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __IO  uint32_t  QABCPS    : 3;  /*!< [b2~b0]*/
            __IO  uint32_t  QABFEN    : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  QIDXCPS   : 3;  /*!< [b6~b4]*/
            __IO  uint32_t  QIDXFEN   : 1;  /*!< [b7~b7]*/
                  uint32_t  rev0      :24;  /*!< [b31~b8]*/
        }BIT;
    }QFLTCON;                               /*!< 0004H */
    __IO uint16_t  QPOSCNT;                                 /*!< 0008H */
         uint16_t  Reserved0;                               /*!< 000AH */
    __IO uint16_t  QPOSMAX;                                 /*!< 000CH */
         uint16_t  Reserved1;                               /*!< 000EH */
    __I  uint32_t  QTMLAT;                                  /*!< 0010H */
    __IO uint32_t  QCNT;                                    /*!< 0014H */
    __IO uint16_t  QCNTMIN;                                 /*!< 0018H */
         uint16_t  Reserved2;                               /*!< 001AH */
    __IO uint32_t  QTPR;                                    /*!< 001CH */
    __IO uint16_t  QTPSQ;                                   /*!< 0020H */
         uint16_t  Reserved3;                               /*!< 0022H */
    union {
        __IO  uint32_t  V32;                                /*!< 0024H */
        struct {
            __IO  uint32_t  QPEIE     : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  QCEIE     : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  QEIIE     : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  QTCAPIE   : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  QTIE      : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  QPEDE     : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  QCEDE     : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  QEIDE     : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  QTCAPDE   : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  QTDE      : 1;  /*!< [b9~b9]*/
                  uint32_t  rev0      :22;  /*!< [b31~b10]*/
        }BIT;
    }QEIINT;                               /*!< 0024H */
    union {
        __IO  uint32_t  V32;                                /*!< 0028H */
        struct {
            __I   uint32_t  QPEIF     : 1;  /*!< [b0~b0]*/
            __I   uint32_t  QCEIF     : 1;  /*!< [b1~b1]*/
            __I   uint32_t  QEIIF     : 1;  /*!< [b2~b2]*/
            __I   uint32_t  QTCAPIF   : 1;  /*!< [b3~b3]*/
            __I   uint32_t  QTIF      : 1;  /*!< [b4~b4]*/
                  uint32_t  rev0      :11;  /*!< [b15~b5]*/
            __O   uint32_t  QPEIFC    : 1;  /*!< [b16~b16]*/
            __O   uint32_t  QCEIFC    : 1;  /*!< [b17~b17]*/
            __O   uint32_t  QEIIFC    : 1;  /*!< [b18~b18]*/
            __O   uint32_t  QTCAPIFC  : 1;  /*!< [b19~b19]*/
            __O   uint32_t  QTIFC     : 1;  /*!< [b20~b20]*/
                  uint32_t  rev1      :11;  /*!< [b31~b21]*/
        }BIT;
    }QTINTF;                               /*!< 0028H */
}QEI_TypeDef;


/**
*@brief AMOC_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  C1DEB     : 3;  /*!< [b2~b0]*/
            __IO  uint32_t  C1PCHS    : 2;  /*!< [b4~b3]*/
                  uint32_t  rev0      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  C1NCHS    : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  CMP1EN    : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  C1SMT     : 2;  /*!< [b9~b8]*/
            __I   uint32_t  C1OUT     : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  C1OUTEN   : 1;  /*!< [b11~b11]*/
                  uint32_t  rev1      : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  C1IES     : 2;  /*!< [b14~b13]*/
            __IO  uint32_t  C1DE      : 1;  /*!< [b15~b15]*/
            __IO  uint32_t  CMP1VRS   : 3;  /*!< [b18~b16]*/
            __IO  uint32_t  CMP1VCMP  : 1;  /*!< [b19~b19]*/
                  uint32_t  rev2      :12;  /*!< [b31~b20]*/
        }BIT;
    }CMP1CON;                               /*!< 0000H */
    union {
        __IO  uint32_t  V32;                                /*!< 0004H */
        struct {
            __IO  uint32_t  C2DEB     : 3;  /*!< [b2~b0]*/
            __IO  uint32_t  C2PCHS    : 3;  /*!< [b5~b3]*/
            __IO  uint32_t  C2NCHS    : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  CMP2EN    : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  C2SMT     : 2;  /*!< [b9~b8]*/
            __I   uint32_t  C2OUT     : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  C2OUTEN   : 1;  /*!< [b11~b11]*/
                  uint32_t  rev0      : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  C2IES     : 2;  /*!< [b14~b13]*/
            __IO  uint32_t  C2DE      : 1;  /*!< [b15~b15]*/
            __IO  uint32_t  CMP2VRS   : 3;  /*!< [b18~b16]*/
            __IO  uint32_t  CMP2VCMP  : 1;  /*!< [b19~b19]*/
                  uint32_t  rev1      :12;  /*!< [b31~b20]*/
        }BIT;
    }CMP2CON;                               /*!< 0004H */
    union {
        __IO  uint32_t  V32;                                /*!< 0008H */
        struct {
            __IO  uint32_t  C3DEB     : 3;  /*!< [b2~b0]*/
            __IO  uint32_t  C3PCHS    : 3;  /*!< [b5~b3]*/
            __IO  uint32_t  C3NCHS    : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  CMP3EN    : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  C3SMT     : 2;  /*!< [b9~b8]*/
            __I   uint32_t  C3OUT     : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  C3OUTEN   : 1;  /*!< [b11~b11]*/
                  uint32_t  rev0      : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  C3IES     : 2;  /*!< [b14~b13]*/
            __IO  uint32_t  C3DE      : 1;  /*!< [b15~b15]*/
            __IO  uint32_t  CMP3VRS   : 3;  /*!< [b18~b16]*/
            __IO  uint32_t  CMP3VCMP  : 1;  /*!< [b19~b19]*/
                  uint32_t  rev1      :12;  /*!< [b31~b20]*/
        }BIT;
    }CMP3CON;                               /*!< 0008H */
    union {
        __IO  uint32_t  V32;                                /*!< 000CH */
        struct {
            __I   uint32_t  C1IF      : 1;  /*!< [b0~b0]*/
            __I   uint32_t  C2IF      : 1;  /*!< [b1~b1]*/
            __I   uint32_t  C3IF      : 1;  /*!< [b2~b2]*/
                  uint32_t  rev0      :13;  /*!< [b15~b3]*/
            __O   uint32_t  C1IFC     : 1;  /*!< [b16~b16]*/
            __O   uint32_t  C2IFC     : 1;  /*!< [b17~b17]*/
            __O   uint32_t  C3IFC     : 1;  /*!< [b18~b18]*/
                  uint32_t  rev1      :13;  /*!< [b31~b19]*/
        }BIT;
    }CMPINTF;                               /*!< 000CH */
    union {
        __IO  uint32_t  V32;                                /*!< 0010H */
        struct {
            __IO  uint32_t  OP1EN     : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  OP2EN     : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  OP3EN     : 1;  /*!< [b2~b2]*/
                  uint32_t  rev0      :29;  /*!< [b31~b3]*/
        }BIT;
    }OPCON;                               /*!< 0010H */
    union {
        __IO  uint32_t  V32;                                /*!< 0014H */
        struct {
            __IO  uint32_t  VREFEN    : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  VREFSEL   : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      :30;  /*!< [b31~b2]*/
        }BIT;
    }AVREFCON;                               /*!< 0014H */
    union {
        __IO  uint32_t  V32;                                /*!< 0018H */
        struct {
            __IO  uint32_t  TPSEN     : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  TPSCHOP   : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      :30;  /*!< [b31~b2]*/
        }BIT;
    }TPSCON;                               /*!< 0018H */
}AMOC_TypeDef;


/**
*@brief UART_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __I   uint32_t  RI        : 1;  /*!< [b0~b0]*/
            __I   uint32_t  TI        : 1;  /*!< [b1~b1]*/
            __I   uint32_t  TC        : 1;  /*!< [b2~b2]*/
            __I   uint32_t  TXCOL     : 1;  /*!< [b3~b3]*/
            __I   uint32_t  RXOV      : 1;  /*!< [b4~b4]*/
            __I   uint32_t  FE        : 1;  /*!< [b5~b5]*/
            __I   uint32_t  PE        : 1;  /*!< [b6~b6]*/
            __I   uint32_t  LBD       : 1;  /*!< [b7~b7]*/
                  uint32_t  rev0      : 8;  /*!< [b15~b8]*/
            __O   uint32_t  RIC       : 1;  /*!< [b16~b16]*/
                  uint32_t  rev1      : 1;  /*!< [b17~b17]*/
            __O   uint32_t  TCC       : 1;  /*!< [b18~b18]*/
            __O   uint32_t  TXCOLC    : 1;  /*!< [b19~b19]*/
            __O   uint32_t  RXOVC     : 1;  /*!< [b20~b20]*/
            __O   uint32_t  FEC       : 1;  /*!< [b21~b21]*/
            __O   uint32_t  PEC       : 1;  /*!< [b22~b22]*/
            __O   uint32_t  LBDC      : 1;  /*!< [b23~b23]*/
                  uint32_t  rev2      : 8;  /*!< [b31~b24]*/
        }BIT;
    }FR;                               /*!< 0000H */
         uint32_t  Reserved0[1];                            /*!< 0004H */
    __IO uint8_t   TDR;                                     /*!< 0008H */
         uint8_t   Reserved1[3];                            /*!< 0009H */
    __I  uint8_t   RDR;                                     /*!< 000CH */
         uint8_t   Reserved2[3];                            /*!< 000DH */
    union {
        __IO  uint32_t  V32;                                /*!< 0010H */
        struct {
    __IO uint8_t   SADDR;                                   /*!< 0010H */
    __IO uint8_t   SMAR;                                    /*!< 0011H */
         uint16_t  rev0;                                    /*!< 0012H */
        }BIT;
    }ADDR;                              /*!< 0010H */
    union {
        __IO  uint32_t  V32;                                /*!< 0014H */
        struct {
            __IO  uint32_t  SBRT      :15;  /*!< [b14~b0]*/
                  uint32_t  rev0      : 1;  /*!< [b15~b15]*/
            __IO  uint32_t  BFINE     : 4;  /*!< [b19~b16]*/
                  uint32_t  rev1      :12;  /*!< [b31~b20]*/
        }BIT;
    }BRT;                               /*!< 0014H */
    union {
        __IO  uint32_t  V32;                                /*!< 0018H */
        struct {
            __IO  uint32_t  STOP      : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  SBRTEN    : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  SMOD      : 1;  /*!< [b2~b2]*/
            __IO  uint32_t  RIE       : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  TIE       : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  TCIE      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  LBDIE     : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  LBDL      : 1;  /*!< [b7~b7]*/
            __I   uint32_t  RB8       : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  TB8       : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  PS        : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  PCE       : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  SM2       : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  SM        : 2;  /*!< [b14~b13]*/
            __IO  uint32_t  SBK       : 1;  /*!< [b15~b15]*/
            __IO  uint32_t  LINEN     : 1;  /*!< [b16~b16]*/
            __IO  uint32_t  REN       : 1;  /*!< [b17~b17]*/
            __IO  uint32_t  TEN       : 1;  /*!< [b18~b18]*/
            __IO  uint32_t  DMAR      : 1;  /*!< [b19~b19]*/
            __IO  uint32_t  DMAT      : 1;  /*!< [b20~b20]*/
                  uint32_t  rev0      :11;  /*!< [b31~b21]*/
        }BIT;
    }CR;                               /*!< 0018H */
}UART_TypeDef;


/**
*@brief SPI_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __I   uint32_t  SPRI      : 1;  /*!< [b0~b0]*/
            __I   uint32_t  SPTI      : 1;  /*!< [b1~b1]*/
                  uint32_t  rev0      : 1;  /*!< [b2~b2]*/
            __I   uint32_t  MODF      : 1;  /*!< [b3~b3]*/
            __I   uint32_t  RXOV      : 1;  /*!< [b4~b4]*/
            __I   uint32_t  WCOL      : 1;  /*!< [b5~b5]*/
                  uint32_t  rev1      :10;  /*!< [b15~b6]*/
            __O   uint32_t  SPRIC     : 1;  /*!< [b16~b16]*/
            __O   uint32_t  SPTIC     : 1;  /*!< [b17~b17]*/
            __O   uint32_t  rev2      : 1;  /*!< [b18~b18]*/
            __O   uint32_t  MODFC     : 1;  /*!< [b19~b19]*/
            __O   uint32_t  RXOVC     : 1;  /*!< [b20~b20]*/
            __O   uint32_t  WCOLC     : 1;  /*!< [b21~b21]*/
                  uint32_t  rev3      :10;  /*!< [b31~b22]*/
        }BIT;
    }FR;                               /*!< 0000H */
    __IO uint16_t  TDR;                                     /*!< 0004H */
         uint16_t  Reserved0;                               /*!< 0006H */
    __I  uint16_t  RDR;                                     /*!< 0008H */
         uint16_t  Reserved1;                               /*!< 000AH */
    union {
        __IO  uint32_t  V32;                                /*!< 000CH */
        struct {
            __IO  uint32_t  SPR       : 4;  /*!< [b3~b0]*/
            __IO  uint32_t  SSDIS     : 1;  /*!< [b4~b4]*/
            __IO  uint32_t  CPOL      : 1;  /*!< [b5~b5]*/
            __IO  uint32_t  CPHA      : 1;  /*!< [b6~b6]*/
            __IO  uint32_t  MSTR      : 1;  /*!< [b7~b7]*/
            __IO  uint32_t  DIR       : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  SPDATL    : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  SPRIE     : 1;  /*!< [b10~b10]*/
            __IO  uint32_t  SPTIE     : 1;  /*!< [b11~b11]*/
            __IO  uint32_t  SPDMAR    : 1;  /*!< [b12~b12]*/
            __IO  uint32_t  SPDMAT    : 1;  /*!< [b13~b13]*/
            __IO  uint32_t  SPIEN     : 1;  /*!< [b14~b14]*/
            __IO  uint32_t  SPSFF     : 1;  /*!< [b15~b15]*/
                  uint32_t  rev0      :16;  /*!< [b31~b16]*/
        }BIT;
    }CR;                               /*!< 000CH */
}SPI_TypeDef;


/**
*@brief TWI_registers
*/
typedef struct
{
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __I   uint32_t  TWINT     : 1;  /*!< [b0~b0]*/
                  uint32_t  rev0      : 1;  /*!< [b1~b1]*/
            __I   uint32_t  TFREE     : 1;  /*!< [b2~b2]*/
            __I   uint32_t  TOUT      : 1;  /*!< [b3~b3]*/
                  uint32_t  rev1      :12;  /*!< [b15~b4]*/
            __O   uint32_t  TWINTC    : 1;  /*!< [b16~b16]*/
                  uint32_t  rev2      : 1;  /*!< [b17~b17]*/
            __O   uint32_t  TFREEC    : 1;  /*!< [b18~b18]*/
            __O   uint32_t  TOUTC     : 1;  /*!< [b19~b19]*/
                  uint32_t  rev3      :12;  /*!< [b31~b20]*/
        }BIT;
    }FR;                               /*!< 0000H */
    union {
        __I   uint32_t  V32;                                /*!< 0004H */
        struct {
                  uint32_t  rev0      : 3;  /*!< [b2~b0]*/
            __I   uint32_t  SR        : 5;  /*!< [b7~b3]*/
                  uint32_t  rev1      :24;  /*!< [b31~b8]*/
        }BIT;
    }STAT;                               /*!< 0004H */
    __IO uint8_t   HOC;                                     /*!< 0008H */
         uint8_t   Reserved0[3];                            /*!< 0009H */
    __IO uint8_t   BRT;                                     /*!< 000CH */
         uint8_t   Reserved1[3];                            /*!< 000DH */
    __IO uint8_t   DR;                                      /*!< 0010H */
         uint8_t   Reserved2[3];                            /*!< 0011H */
    union {
        __IO  uint32_t  V32;                                /*!< 0014H */
        struct {
            __IO  uint32_t  GC        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  ADDR      : 7;  /*!< [b7~b1]*/
                  uint32_t  rev0      : 9;  /*!< [b16~b8]*/
            __IO  uint32_t  TWIAMR    : 7;  /*!< [b23~b17]*/
                  uint32_t  rev1      : 8;  /*!< [b31~b24]*/
        }BIT;
    }ADDR;                               /*!< 0014H */
    union {
        __IO  uint32_t  V32;                                /*!< 0018H */
        struct {
            __IO  uint32_t  AA        : 1;  /*!< [b0~b0]*/
            __IO  uint32_t  STO       : 1;  /*!< [b1~b1]*/
            __IO  uint32_t  STA       : 1;  /*!< [b2~b2]*/
                  uint32_t  rev0      : 1;  /*!< [b3~b3]*/
            __IO  uint32_t  CR        : 2;  /*!< [b5~b4]*/
            __IO  uint32_t  CNT       : 2;  /*!< [b7~b6]*/
            __IO  uint32_t  EFREE     : 1;  /*!< [b8~b8]*/
            __IO  uint32_t  ETOT      : 1;  /*!< [b9~b9]*/
            __IO  uint32_t  TWINTE    : 1;  /*!< [b10~b10]*/
                  uint32_t  rev1      : 4;  /*!< [b14~b11]*/
            __IO  uint32_t  TWIEN     : 1;  /*!< [b15~b15]*/
            __IO  uint32_t  rev2      :16;  /*!< [b31~b16]*/
        }BIT;
    }CR;                               /*!< 0018H */
}TWI_TypeDef;


/* @addtogroup  Peripheral base address*/

#define TIM5_BASE                      ((uint32_t)0x40000400)    /* Bus:APB1        Module:TIM        */
#define TIM6_BASE                      ((uint32_t)0x40000800)    /* Bus:APB1        Module:TIM        */
#define TIM7_BASE                      ((uint32_t)0x40000C00)    /* Bus:APB1        Module:TIM        */
#define TIM8_BASE                      ((uint32_t)0x40001000)    /* Bus:APB1        Module:TIM        */
#define QEI_BASE                       ((uint32_t)0x40001C00)    /* Bus:APB1        Module:QEI        */
#define UART1_BASE                     ((uint32_t)0x40002000)    /* Bus:APB1        Module:UART       */
#define UART2_BASE                     ((uint32_t)0x40002400)    /* Bus:APB1        Module:UART       */
#define UART3_BASE                     ((uint32_t)0x40002800)    /* Bus:APB1        Module:UART       */
#define SPI1_BASE                      ((uint32_t)0x40002C00)    /* Bus:APB1        Module:SPI        */
#define SPI2_BASE                      ((uint32_t)0x40003000)    /* Bus:APB1        Module:SPI        */
#define TWI1_BASE                      ((uint32_t)0x40003400)    /* Bus:APB1        Module:TWI        */
#define IWDT_BASE                      ((uint32_t)0x40004C00)    /* Bus:APB1        Module:IWDT       */
#define WWDT_BASE                      ((uint32_t)0x40005000)    /* Bus:APB1        Module:WWDT       */
#define AMOC_BASE                      ((uint32_t)0x40005800)    /* Bus:APB1        Module:AMOC       */
#define MCM1_BASE                      ((uint32_t)0x40020000)    /* Bus:APB2        Module:MCM        */
#define MCM2_BASE                      ((uint32_t)0x40020400)    /* Bus:APB2        Module:MCM        */
#define EXTI_BASE                      ((uint32_t)0x40021000)    /* Bus:APB2        Module:EXTI       */
#define GPIOA_CFG_BASE                 ((uint32_t)0x40040000)    /* Bus:AHB         Module:GPIO_CFG   */
#define GPIOB_CFG_BASE                 ((uint32_t)0x40040020)    /* Bus:AHB         Module:GPIO_CFG   */
#define GPIOC_CFG_BASE                 ((uint32_t)0x40040040)    /* Bus:AHB         Module:GPIO_CFG   */
#define GPIOD_CFG_BASE                 ((uint32_t)0x40040060)    /* Bus:AHB         Module:GPIO_CFG   */
#define GPIOE_CFG_BASE                 ((uint32_t)0x40040080)    /* Bus:AHB         Module:GPIO_CFG   */
#define GPIOA_BASE                     ((uint32_t)0x40040400)    /* Bus:AHB         Module:GPIO       */
#define GPIOB_BASE                     ((uint32_t)0x40040420)    /* Bus:AHB         Module:GPIO       */
#define GPIOC_BASE                     ((uint32_t)0x40040440)    /* Bus:AHB         Module:GPIO       */
#define GPIOD_BASE                     ((uint32_t)0x40040460)    /* Bus:AHB         Module:GPIO       */
#define GPIOE_BASE                     ((uint32_t)0x40040480)    /* Bus:AHB         Module:GPIO       */
#define ADC1_BASE                      ((uint32_t)0x40041000)    /* Bus:AHB         Module:ADC        */
#define ADC2_BASE                      ((uint32_t)0x40041400)    /* Bus:AHB         Module:ADC        */
#define ADC3_BASE                      ((uint32_t)0x40041800)    /* Bus:AHB         Module:ADC        */
#define GPT_BASE                       ((uint32_t)0x40042000)    /* Bus:AHB         Module:GPT_GLOBAL */
#define GPT0_BASE                      ((uint32_t)0x40042400)    /* Bus:AHB         Module:GPT        */
#define GPT1_BASE                      ((uint32_t)0x40042800)    /* Bus:AHB         Module:GPT        */
#define GPT2_BASE                      ((uint32_t)0x40042C00)    /* Bus:AHB         Module:GPT        */
#define GPT3_BASE                      ((uint32_t)0x40043000)    /* Bus:AHB         Module:GPT        */
#define SYSCFG_BASE                    ((uint32_t)0x40044000)    /* Bus:AHB         Module:SYSCFG     */
#define RCC_BASE                       ((uint32_t)0x40044400)    /* Bus:AHB         Module:RCC        */
#define DMA_BASE                       ((uint32_t)0x40044800)    /* Bus:AHB         Module:DMA        */
#define FLASH_BASE                     ((uint32_t)0x40045000)    /* Bus:AHB         Module:FLASH      */
#define MACP_BASE                      ((uint32_t)0x40045400)    /* Bus:AHB         Module:MACP       */
#define CRC_BASE                       ((uint32_t)0x40045800)    /* Bus:AHB         Module:CRC        */
#define RAMBIST_BASE                   ((uint32_t)0x40045C00)    /* Bus:AHB         Module:RAMBIST    */

/* @addtogroup Peripheral_declaration */

#define FLASH                ((FLASH_TypeDef *)       FLASH_BASE)
#define SYSCFG               ((SYSCFG_TypeDef *)      SYSCFG_BASE)
#define RCC                  ((RCC_TypeDef *)         RCC_BASE)
#define IWDT                 ((IWDT_TypeDef *)        IWDT_BASE)
#define WWDT                 ((WWDT_TypeDef *)        WWDT_BASE)
#define CRC                  ((CRC_TypeDef *)         CRC_BASE)
#define RAMBIST              ((RAMBIST_TypeDef *)     RAMBIST_BASE)
#define GPIOA_CFG            ((GPIO_CFG_TypeDef *)    GPIOA_CFG_BASE)
#define GPIOB_CFG            ((GPIO_CFG_TypeDef *)    GPIOB_CFG_BASE)
#define GPIOC_CFG            ((GPIO_CFG_TypeDef *)    GPIOC_CFG_BASE)
#define GPIOD_CFG            ((GPIO_CFG_TypeDef *)    GPIOD_CFG_BASE)
#define GPIOE_CFG            ((GPIO_CFG_TypeDef *)    GPIOE_CFG_BASE)
#define GPIOA                ((GPIO_TypeDef *)        GPIOA_BASE)
#define GPIOB                ((GPIO_TypeDef *)        GPIOB_BASE)
#define GPIOC                ((GPIO_TypeDef *)        GPIOC_BASE)
#define GPIOD                ((GPIO_TypeDef *)        GPIOD_BASE)
#define GPIOE                ((GPIO_TypeDef *)        GPIOE_BASE)
#define EXTI                 ((EXTI_TypeDef *)        EXTI_BASE)
#define DMA                  ((DMA_TypeDef *)         DMA_BASE)
#define MCM1                 ((MCM_TypeDef *)         MCM1_BASE)
#define MCM2                 ((MCM_TypeDef *)         MCM2_BASE)
#define GPT                  ((GPT_GLOBAL_TypeDef *)  GPT_BASE)
#define GPT0                 ((GPT_TypeDef *)         GPT0_BASE)
#define GPT1                 ((GPT_TypeDef *)         GPT1_BASE)
#define GPT2                 ((GPT_TypeDef *)         GPT2_BASE)
#define GPT3                 ((GPT_TypeDef *)         GPT3_BASE)
#define TIM5                 ((TIM_TypeDef *)         TIM5_BASE)
#define TIM6                 ((TIM_TypeDef *)         TIM6_BASE)
#define TIM7                 ((TIM_TypeDef *)         TIM7_BASE)
#define TIM8                 ((TIM_TypeDef *)         TIM8_BASE)
#define MACP                 ((MACP_TypeDef *)        MACP_BASE)
#define ADC1                 ((ADC_TypeDef *)         ADC1_BASE)
#define ADC2                 ((ADC_TypeDef *)         ADC2_BASE)
#define ADC3                 ((ADC_TypeDef *)         ADC3_BASE)
#define QEI                  ((QEI_TypeDef *)         QEI_BASE)
#define AMOC                 ((AMOC_TypeDef *)        AMOC_BASE)
#define UART1                ((UART_TypeDef *)        UART1_BASE)
#define UART2                ((UART_TypeDef *)        UART2_BASE)
#define UART3                ((UART_TypeDef *)        UART3_BASE)
#define SPI1                 ((SPI_TypeDef *)         SPI1_BASE)
#define SPI2                 ((SPI_TypeDef *)         SPI2_BASE)
#define TWI1                 ((TWI_TypeDef *)         TWI1_BASE)

/* @addtogroup Peripheral Field Postion and mask */

#define FLASH_ACR_LATENCY_Pos        0                                            /*!<FLASH ACR: LATENCY Position */
#define FLASH_ACR_LATENCY_Msk        (0x7UL /*<< FLASH_ACR_LATENCY_Pos*/)         /*!<FLASH ACR: LATENCY Mask */

#define FLASH_ACR_PRFTEN_Pos         8                                            /*!<FLASH ACR: PRFTEN Position */
#define FLASH_ACR_PRFTEN_Msk         (0x1UL << FLASH_ACR_PRFTEN_Pos)              /*!<FLASH ACR: PRFTEN Mask */

#define FLASH_ACR_ICEN_Pos           9                                            /*!<FLASH ACR: ICEN Position */
#define FLASH_ACR_ICEN_Msk           (0x1UL << FLASH_ACR_ICEN_Pos)                /*!<FLASH ACR: ICEN Mask */

#define FLASH_ACR_DCEN_Pos           10                                           /*!<FLASH ACR: DCEN Position */
#define FLASH_ACR_DCEN_Msk           (0x1UL << FLASH_ACR_DCEN_Pos)                /*!<FLASH ACR: DCEN Mask */

#define FLASH_ACR_CRST_Pos           11                                           /*!<FLASH ACR: CRST Position */
#define FLASH_ACR_CRST_Msk           (0x1UL << FLASH_ACR_CRST_Pos)                /*!<FLASH ACR: CRST Mask */

#define FLASH_ACR_LOCK_Pos           16                                           /*!<FLASH ACR: LOCK Position */
#define FLASH_ACR_LOCK_Msk           (0xFFFFUL << FLASH_ACR_LOCK_Pos)             /*!<FLASH ACR: LOCK Mask */

#define FLASH_ACR_Msk_ALL            (FLASH_ACR_LATENCY_Msk        |\
                                      FLASH_ACR_PRFTEN_Msk         |\
                                      FLASH_ACR_ICEN_Msk           |\
                                      FLASH_ACR_DCEN_Msk           |\
                                      FLASH_ACR_CRST_Msk           |\
                                      FLASH_ACR_LOCK_Msk           )

#define FLASH_SR_EOP_Pos             0                                            /*!<FLASH SR: EOP Position */
#define FLASH_SR_EOP_Msk             (0x1UL /*<< FLASH_SR_EOP_Pos*/)              /*!<FLASH SR: EOP Mask */

#define FLASH_SR_OPERR_Pos           1                                            /*!<FLASH SR: OPERR Position */
#define FLASH_SR_OPERR_Msk           (0x1UL << FLASH_SR_OPERR_Pos)                /*!<FLASH SR: OPERR Mask */

#define FLASH_SR_FLSERR_Pos          3                                            /*!<FLASH SR: FLSERR Position */
#define FLASH_SR_FLSERR_Msk          (0x1UL << FLASH_SR_FLSERR_Pos)               /*!<FLASH SR: FLSERR Mask */

#define FLASH_SR_WRPRTERR_Pos        4                                            /*!<FLASH SR: WRPRTERR Position */
#define FLASH_SR_WRPRTERR_Msk        (0x1UL << FLASH_SR_WRPRTERR_Pos)             /*!<FLASH SR: WRPRTERR Mask */

#define FLASH_SR_PGPERR_Pos          5                                            /*!<FLASH SR: PGPERR Position */
#define FLASH_SR_PGPERR_Msk          (0x1UL << FLASH_SR_PGPERR_Pos)               /*!<FLASH SR: PGPERR Mask */

#define FLASH_SR_PGWERR_Pos          6                                            /*!<FLASH SR: PGWERR Position */
#define FLASH_SR_PGWERR_Msk          (0x1UL << FLASH_SR_PGWERR_Pos)               /*!<FLASH SR: PGWERR Mask */

#define FLASH_SR_STAERR_Pos          7                                            /*!<FLASH SR: STAERR Position */
#define FLASH_SR_STAERR_Msk          (0x1UL << FLASH_SR_STAERR_Pos)               /*!<FLASH SR: STAERR Mask */

#define FLASH_SR_BSY_Pos             15                                           /*!<FLASH SR: BSY Position */
#define FLASH_SR_BSY_Msk             (0x1UL << FLASH_SR_BSY_Pos)                  /*!<FLASH SR: BSY Mask */

#define FLASH_SR_EOPC_Pos            16                                           /*!<FLASH SR: EOPC Position */
#define FLASH_SR_EOPC_Msk            (0x1UL << FLASH_SR_EOPC_Pos)                 /*!<FLASH SR: EOPC Mask */

#define FLASH_SR_OPERRC_Pos          17                                           /*!<FLASH SR: OPERRC Position */
#define FLASH_SR_OPERRC_Msk          (0x1UL << FLASH_SR_OPERRC_Pos)               /*!<FLASH SR: OPERRC Mask */

#define FLASH_SR_FLSERRC_Pos         19                                           /*!<FLASH SR: FLSERRC Position */
#define FLASH_SR_FLSERRC_Msk         (0x1UL << FLASH_SR_FLSERRC_Pos)              /*!<FLASH SR: FLSERRC Mask */

#define FLASH_SR_WRPRTERRC_Pos       20                                           /*!<FLASH SR: WRPRTERRC Position */
#define FLASH_SR_WRPRTERRC_Msk       (0x1UL << FLASH_SR_WRPRTERRC_Pos)            /*!<FLASH SR: WRPRTERRC Mask */

#define FLASH_SR_PGPERRC_Pos         21                                           /*!<FLASH SR: PGPERRC Position */
#define FLASH_SR_PGPERRC_Msk         (0x1UL << FLASH_SR_PGPERRC_Pos)              /*!<FLASH SR: PGPERRC Mask */

#define FLASH_SR_PGWERRC_Pos         22                                           /*!<FLASH SR: PGWERRC Position */
#define FLASH_SR_PGWERRC_Msk         (0x1UL << FLASH_SR_PGWERRC_Pos)              /*!<FLASH SR: PGWERRC Mask */

#define FLASH_SR_STAERRC_Pos         23                                           /*!<FLASH SR: STAERRC Position */
#define FLASH_SR_STAERRC_Msk         (0x1UL << FLASH_SR_STAERRC_Pos)              /*!<FLASH SR: STAERRC Mask */

#define FLASH_SR_Msk_ALL             (FLASH_SR_EOP_Msk             |\
                                      FLASH_SR_OPERR_Msk           |\
                                      FLASH_SR_FLSERR_Msk          |\
                                      FLASH_SR_WRPRTERR_Msk        |\
                                      FLASH_SR_PGPERR_Msk          |\
                                      FLASH_SR_PGWERR_Msk          |\
                                      FLASH_SR_STAERR_Msk          |\
                                      FLASH_SR_BSY_Msk             |\
                                      FLASH_SR_EOPC_Msk            |\
                                      FLASH_SR_OPERRC_Msk          |\
                                      FLASH_SR_FLSERRC_Msk         |\
                                      FLASH_SR_WRPRTERRC_Msk       |\
                                      FLASH_SR_PGPERRC_Msk         |\
                                      FLASH_SR_PGWERRC_Msk         |\
                                      FLASH_SR_STAERRC_Msk         )

#define FLASH_CR_SNB_Pos             0                                            /*!<FLASH CR: SNB Position */
#define FLASH_CR_SNB_Msk             (0x7FUL /*<< FLASH_CR_SNB_Pos*/)             /*!<FLASH CR: SNB Mask */

#define FLASH_CR_STRT_Pos            8                                            /*!<FLASH CR: STRT Position */
#define FLASH_CR_STRT_Msk            (0x1UL << FLASH_CR_STRT_Pos)                 /*!<FLASH CR: STRT Mask */

#define FLASH_CR_PSIZE_Pos           11                                           /*!<FLASH CR: PSIZE Position */
#define FLASH_CR_PSIZE_Msk           (0x1UL << FLASH_CR_PSIZE_Pos)                /*!<FLASH CR: PSIZE Mask */

#define FLASH_CR_INFLCK_Pos          12                                           /*!<FLASH CR: INFLCK Position */
#define FLASH_CR_INFLCK_Msk          (0x1UL << FLASH_CR_INFLCK_Pos)               /*!<FLASH CR: INFLCK Mask */

#define FLASH_CR_E2LCK_Pos           14                                           /*!<FLASH CR: E2LCK Position */
#define FLASH_CR_E2LCK_Msk           (0x1UL << FLASH_CR_E2LCK_Pos)                /*!<FLASH CR: E2LCK Mask */

#define FLASH_CR_MNLCK_Pos           15                                           /*!<FLASH CR: MNLCK Position */
#define FLASH_CR_MNLCK_Msk           (0x1UL << FLASH_CR_MNLCK_Pos)                /*!<FLASH CR: MNLCK Mask */

#define FLASH_CR_CMD_Pos             16                                           /*!<FLASH CR: CMD Position */
#define FLASH_CR_CMD_Msk             (0xFFFFUL << FLASH_CR_CMD_Pos)               /*!<FLASH CR: CMD Mask */

#define FLASH_CR_Msk_ALL             (FLASH_CR_SNB_Msk             |\
                                      FLASH_CR_STRT_Msk            |\
                                      FLASH_CR_PSIZE_Msk           |\
                                      FLASH_CR_INFLCK_Msk          |\
                                      FLASH_CR_E2LCK_Msk           |\
                                      FLASH_CR_MNLCK_Msk           |\
                                      FLASH_CR_CMD_Msk             )

#define FLASH_OPR_OPT0_Pos           0                                            /*!<FLASH OPR: OPT0 Position */
#define FLASH_OPR_OPT0_Msk           (0xFFUL /*<< FLASH_OPR_OPT0_Pos*/)           /*!<FLASH OPR: OPT0 Mask */

#define FLASH_OPR_OPT1_Pos           8                                            /*!<FLASH OPR: OPT1 Position */
#define FLASH_OPR_OPT1_Msk           (0xFFUL << FLASH_OPR_OPT1_Pos)               /*!<FLASH OPR: OPT1 Mask */

#define FLASH_OPR_OPT2_Pos           16                                           /*!<FLASH OPR: OPT2 Position */
#define FLASH_OPR_OPT2_Msk           (0xFFUL << FLASH_OPR_OPT2_Pos)               /*!<FLASH OPR: OPT2 Mask */

#define FLASH_OPR_OPT3_Pos           24                                           /*!<FLASH OPR: OPT3 Position */
#define FLASH_OPR_OPT3_Msk           (0xFFUL << FLASH_OPR_OPT3_Pos)               /*!<FLASH OPR: OPT3 Mask */

#define FLASH_OPR_Msk_ALL            (FLASH_OPR_OPT0_Msk           |\
                                      FLASH_OPR_OPT1_Msk           |\
                                      FLASH_OPR_OPT2_Msk           |\
                                      FLASH_OPR_OPT3_Msk           )

#define FLASH_RPR_RDP_Pos            0                                            /*!<FLASH RPR: RDP Position */
#define FLASH_RPR_RDP_Msk            (0xFFFFUL /*<< FLASH_RPR_RDP_Pos*/)          /*!<FLASH RPR: RDP Mask */

#define FLASH_RPR_Msk_ALL            (FLASH_RPR_RDP_Msk           )

#define FLASH_CNTCR_CNTEN_Pos        0                                            /*!<FLASH CNTCR: CNTEN Position */
#define FLASH_CNTCR_CNTEN_Msk        (0x1UL /*<< FLASH_CNTCR_CNTEN_Pos*/)         /*!<FLASH CNTCR: CNTEN Mask */

#define FLASH_CNTCR_Msk_ALL          (FLASH_CNTCR_CNTEN_Msk       )

#define FLASH_MEMRMP_MEMMODE_Pos     0                                            /*!<FLASH MEMRMP: MEMMODE Position */
#define FLASH_MEMRMP_MEMMODE_Msk     (0xFFFFUL /*<< FLASH_MEMRMP_MEMMODE_Pos*/)   /*!<FLASH MEMRMP: MEMMODE Mask */

#define FLASH_MEMRMP_Msk_ALL         (FLASH_MEMRMP_MEMMODE_Msk    )

#define FLASH_OPR_CUST1_OPT0_Pos     0                                            /*!<FLASH OPR_CUST1: OPT0 Position */
#define FLASH_OPR_CUST1_OPT0_Msk     (0xFFUL /*<< FLASH_OPR_CUST1_OPT0_Pos*/)     /*!<FLASH OPR_CUST1: OPT0 Mask */

#define FLASH_OPR_CUST1_OPT1_Pos     8                                            /*!<FLASH OPR_CUST1: OPT1 Position */
#define FLASH_OPR_CUST1_OPT1_Msk     (0xFFUL << FLASH_OPR_CUST1_OPT1_Pos)         /*!<FLASH OPR_CUST1: OPT1 Mask */

#define FLASH_OPR_CUST1_OPT2_Pos     16                                           /*!<FLASH OPR_CUST1: OPT2 Position */
#define FLASH_OPR_CUST1_OPT2_Msk     (0xFFUL << FLASH_OPR_CUST1_OPT2_Pos)         /*!<FLASH OPR_CUST1: OPT2 Mask */

#define FLASH_OPR_CUST1_OPT3_Pos     24                                           /*!<FLASH OPR_CUST1: OPT3 Position */
#define FLASH_OPR_CUST1_OPT3_Msk     (0xFFUL << FLASH_OPR_CUST1_OPT3_Pos)         /*!<FLASH OPR_CUST1: OPT3 Mask */

#define FLASH_OPR_CUST1_Msk_ALL      (FLASH_OPR_CUST1_OPT0_Msk     |\
                                      FLASH_OPR_CUST1_OPT1_Msk     |\
                                      FLASH_OPR_CUST1_OPT2_Msk     |\
                                      FLASH_OPR_CUST1_OPT3_Msk     )

#define FLASH_OPR_DESI0_OPT0_Pos     0                                            /*!<FLASH OPR_DESI0: OPT0 Position */
#define FLASH_OPR_DESI0_OPT0_Msk     (0xFFUL /*<< FLASH_OPR_DESI0_OPT0_Pos*/)     /*!<FLASH OPR_DESI0: OPT0 Mask */

#define FLASH_OPR_DESI0_OPT1_Pos     8                                            /*!<FLASH OPR_DESI0: OPT1 Position */
#define FLASH_OPR_DESI0_OPT1_Msk     (0xFFUL << FLASH_OPR_DESI0_OPT1_Pos)         /*!<FLASH OPR_DESI0: OPT1 Mask */

#define FLASH_OPR_DESI0_OPT2_Pos     16                                           /*!<FLASH OPR_DESI0: OPT2 Position */
#define FLASH_OPR_DESI0_OPT2_Msk     (0xFFUL << FLASH_OPR_DESI0_OPT2_Pos)         /*!<FLASH OPR_DESI0: OPT2 Mask */

#define FLASH_OPR_DESI0_OPT3_Pos     24                                           /*!<FLASH OPR_DESI0: OPT3 Position */
#define FLASH_OPR_DESI0_OPT3_Msk     (0xFFUL << FLASH_OPR_DESI0_OPT3_Pos)         /*!<FLASH OPR_DESI0: OPT3 Mask */

#define FLASH_OPR_DESI0_Msk_ALL      (FLASH_OPR_DESI0_OPT0_Msk     |\
                                      FLASH_OPR_DESI0_OPT1_Msk     |\
                                      FLASH_OPR_DESI0_OPT2_Msk     |\
                                      FLASH_OPR_DESI0_OPT3_Msk     )

#define FLASH_OPR_DESI1_OPT0_Pos     0                                            /*!<FLASH OPR_DESI1: OPT0 Position */
#define FLASH_OPR_DESI1_OPT0_Msk     (0xFFUL /*<< FLASH_OPR_DESI1_OPT0_Pos*/)     /*!<FLASH OPR_DESI1: OPT0 Mask */

#define FLASH_OPR_DESI1_OPT1_Pos     8                                            /*!<FLASH OPR_DESI1: OPT1 Position */
#define FLASH_OPR_DESI1_OPT1_Msk     (0xFFUL << FLASH_OPR_DESI1_OPT1_Pos)         /*!<FLASH OPR_DESI1: OPT1 Mask */

#define FLASH_OPR_DESI1_OPT2_Pos     16                                           /*!<FLASH OPR_DESI1: OPT2 Position */
#define FLASH_OPR_DESI1_OPT2_Msk     (0xFFUL << FLASH_OPR_DESI1_OPT2_Pos)         /*!<FLASH OPR_DESI1: OPT2 Mask */

#define FLASH_OPR_DESI1_OPT3_Pos     24                                           /*!<FLASH OPR_DESI1: OPT3 Position */
#define FLASH_OPR_DESI1_OPT3_Msk     (0xFFUL << FLASH_OPR_DESI1_OPT3_Pos)         /*!<FLASH OPR_DESI1: OPT3 Mask */

#define FLASH_OPR_DESI1_Msk_ALL      (FLASH_OPR_DESI1_OPT0_Msk     |\
                                      FLASH_OPR_DESI1_OPT1_Msk     |\
                                      FLASH_OPR_DESI1_OPT2_Msk     |\
                                      FLASH_OPR_DESI1_OPT3_Msk     )

#define FLASH_OPR_DESI2_OPT0_Pos     0                                            /*!<FLASH OPR_DESI2: OPT0 Position */
#define FLASH_OPR_DESI2_OPT0_Msk     (0xFFUL /*<< FLASH_OPR_DESI2_OPT0_Pos*/)     /*!<FLASH OPR_DESI2: OPT0 Mask */

#define FLASH_OPR_DESI2_OPT1_Pos     8                                            /*!<FLASH OPR_DESI2: OPT1 Position */
#define FLASH_OPR_DESI2_OPT1_Msk     (0xFFUL << FLASH_OPR_DESI2_OPT1_Pos)         /*!<FLASH OPR_DESI2: OPT1 Mask */

#define FLASH_OPR_DESI2_OPT2_Pos     16                                           /*!<FLASH OPR_DESI2: OPT2 Position */
#define FLASH_OPR_DESI2_OPT2_Msk     (0xFFUL << FLASH_OPR_DESI2_OPT2_Pos)         /*!<FLASH OPR_DESI2: OPT2 Mask */

#define FLASH_OPR_DESI2_OPT3_Pos     24                                           /*!<FLASH OPR_DESI2: OPT3 Position */
#define FLASH_OPR_DESI2_OPT3_Msk     (0xFFUL << FLASH_OPR_DESI2_OPT3_Pos)         /*!<FLASH OPR_DESI2: OPT3 Mask */

#define FLASH_OPR_DESI2_Msk_ALL      (FLASH_OPR_DESI2_OPT0_Msk     |\
                                      FLASH_OPR_DESI2_OPT1_Msk     |\
                                      FLASH_OPR_DESI2_OPT2_Msk     |\
                                      FLASH_OPR_DESI2_OPT3_Msk     )

#define FLASH_OPR_DESI3_OPT0_Pos     0                                            /*!<FLASH OPR_DESI3: OPT0 Position */
#define FLASH_OPR_DESI3_OPT0_Msk     (0xFFUL /*<< FLASH_OPR_DESI3_OPT0_Pos*/)     /*!<FLASH OPR_DESI3: OPT0 Mask */

#define FLASH_OPR_DESI3_OPT1_Pos     8                                            /*!<FLASH OPR_DESI3: OPT1 Position */
#define FLASH_OPR_DESI3_OPT1_Msk     (0xFFUL << FLASH_OPR_DESI3_OPT1_Pos)         /*!<FLASH OPR_DESI3: OPT1 Mask */

#define FLASH_OPR_DESI3_OPT2_Pos     16                                           /*!<FLASH OPR_DESI3: OPT2 Position */
#define FLASH_OPR_DESI3_OPT2_Msk     (0xFFUL << FLASH_OPR_DESI3_OPT2_Pos)         /*!<FLASH OPR_DESI3: OPT2 Mask */

#define FLASH_OPR_DESI3_OPT3_Pos     24                                           /*!<FLASH OPR_DESI3: OPT3 Position */
#define FLASH_OPR_DESI3_OPT3_Msk     (0xFFUL << FLASH_OPR_DESI3_OPT3_Pos)         /*!<FLASH OPR_DESI3: OPT3 Mask */

#define FLASH_OPR_DESI3_Msk_ALL      (FLASH_OPR_DESI3_OPT0_Msk     |\
                                      FLASH_OPR_DESI3_OPT1_Msk     |\
                                      FLASH_OPR_DESI3_OPT2_Msk     |\
                                      FLASH_OPR_DESI3_OPT3_Msk     )

#define SYSCFG_PWRCR_VBOD_Pos        0                                            /*!<SYSCFG PWRCR: VBOD Position */
#define SYSCFG_PWRCR_VBOD_Msk        (0xFUL /*<< SYSCFG_PWRCR_VBOD_Pos*/)         /*!<SYSCFG PWRCR: VBOD Mask */

#define SYSCFG_PWRCR_BODMD_Pos       4                                            /*!<SYSCFG PWRCR: BODMD Position */
#define SYSCFG_PWRCR_BODMD_Msk       (0x3UL << SYSCFG_PWRCR_BODMD_Pos)            /*!<SYSCFG PWRCR: BODMD Mask */

#define SYSCFG_PWRCR_BODIE_Pos       6                                            /*!<SYSCFG PWRCR: BODIE Position */
#define SYSCFG_PWRCR_BODIE_Msk       (0x1UL << SYSCFG_PWRCR_BODIE_Pos)            /*!<SYSCFG PWRCR: BODIE Mask */

#define SYSCFG_PWRCR_BODEN_Pos       7                                            /*!<SYSCFG PWRCR: BODEN Position */
#define SYSCFG_PWRCR_BODEN_Msk       (0x1UL << SYSCFG_PWRCR_BODEN_Pos)            /*!<SYSCFG PWRCR: BODEN Mask */

#define SYSCFG_PWRCR_Msk_ALL         (SYSCFG_PWRCR_VBOD_Msk        |\
                                      SYSCFG_PWRCR_BODMD_Msk       |\
                                      SYSCFG_PWRCR_BODIE_Msk       |\
                                      SYSCFG_PWRCR_BODEN_Msk       )

#define SYSCFG_PWRSR_BODIF_Pos       0                                            /*!<SYSCFG PWRSR: BODIF Position */
#define SYSCFG_PWRSR_BODIF_Msk       (0x1UL /*<< SYSCFG_PWRSR_BODIF_Pos*/)        /*!<SYSCFG PWRSR: BODIF Mask */

#define SYSCFG_PWRSR_BODF_Pos        1                                            /*!<SYSCFG PWRSR: BODF Position */
#define SYSCFG_PWRSR_BODF_Msk        (0x1UL << SYSCFG_PWRSR_BODF_Pos)             /*!<SYSCFG PWRSR: BODF Mask */

#define SYSCFG_PWRSR_Msk_ALL         (SYSCFG_PWRSR_BODIF_Msk       |\
                                      SYSCFG_PWRSR_BODF_Msk        )

#define SYSCFG_SAFR_OSCCFG_Pos       0                                            /*!<SYSCFG SAFR: OSCCFG Position */
#define SYSCFG_SAFR_OSCCFG_Msk       (0x3UL /*<< SYSCFG_SAFR_OSCCFG_Pos*/)        /*!<SYSCFG SAFR: OSCCFG Mask */

#define SYSCFG_SAFR_SWJCFG_Pos       2                                            /*!<SYSCFG SAFR: SWJCFG Position */
#define SYSCFG_SAFR_SWJCFG_Msk       (0x7UL << SYSCFG_SAFR_SWJCFG_Pos)            /*!<SYSCFG SAFR: SWJCFG Mask */

#define SYSCFG_SAFR_IEN_EXTI0_Pos    5                                            /*!<SYSCFG SAFR: IEN_EXTI0 Position */
#define SYSCFG_SAFR_IEN_EXTI0_Msk    (0x1UL << SYSCFG_SAFR_IEN_EXTI0_Pos)         /*!<SYSCFG SAFR: IEN_EXTI0 Mask */

#define SYSCFG_SAFR_IEN_BOD_Pos      6                                            /*!<SYSCFG SAFR: IEN_BOD Position */
#define SYSCFG_SAFR_IEN_BOD_Msk      (0x1UL << SYSCFG_SAFR_IEN_BOD_Pos)           /*!<SYSCFG SAFR: IEN_BOD Mask */

#define SYSCFG_SAFR_IEN_CSM_Pos      7                                            /*!<SYSCFG SAFR: IEN_CSM Position */
#define SYSCFG_SAFR_IEN_CSM_Msk      (0x1UL << SYSCFG_SAFR_IEN_CSM_Pos)           /*!<SYSCFG SAFR: IEN_CSM Mask */

#define SYSCFG_SAFR_LOCK_Pos         16                                           /*!<SYSCFG SAFR: LOCK Position */
#define SYSCFG_SAFR_LOCK_Msk         (0xFFFFUL << SYSCFG_SAFR_LOCK_Pos)           /*!<SYSCFG SAFR: LOCK Mask */

#define SYSCFG_SAFR_Msk_ALL          (SYSCFG_SAFR_OSCCFG_Msk       |\
                                      SYSCFG_SAFR_SWJCFG_Msk       |\
                                      SYSCFG_SAFR_IEN_EXTI0_Msk    |\
                                      SYSCFG_SAFR_IEN_BOD_Msk      |\
                                      SYSCFG_SAFR_IEN_CSM_Msk      |\
                                      SYSCFG_SAFR_LOCK_Msk         )

#define SYSCFG_CRAMLOCK_CRAMLCK_Pos  0                                            /*!<SYSCFG CRAMLOCK: CRAMLCK Position */
#define SYSCFG_CRAMLOCK_CRAMLCK_Msk  (0xFFUL /*<< SYSCFG_CRAMLOCK_CRAMLCK_Pos*/)  /*!<SYSCFG CRAMLOCK: CRAMLCK Mask */

#define SYSCFG_CRAMLOCK_LOCK_Pos     16                                           /*!<SYSCFG CRAMLOCK: LOCK Position */
#define SYSCFG_CRAMLOCK_LOCK_Msk     (0xFFFFUL << SYSCFG_CRAMLOCK_LOCK_Pos)       /*!<SYSCFG CRAMLOCK: LOCK Mask */

#define SYSCFG_CRAMLOCK_Msk_ALL      (SYSCFG_CRAMLOCK_CRAMLCK_Msk  |\
                                      SYSCFG_CRAMLOCK_LOCK_Msk     )

#define SYSCFG_DBGCR_DBG_SLEEP_Pos   0                                            /*!<SYSCFG DBGCR: DBG_SLEEP Position */
#define SYSCFG_DBGCR_DBG_SLEEP_Msk   (0x1UL /*<< SYSCFG_DBGCR_DBG_SLEEP_Pos*/)    /*!<SYSCFG DBGCR: DBG_SLEEP Mask */

#define SYSCFG_DBGCR_DBG_STOP_Pos    1                                            /*!<SYSCFG DBGCR: DBG_STOP Position */
#define SYSCFG_DBGCR_DBG_STOP_Msk    (0x1UL << SYSCFG_DBGCR_DBG_STOP_Pos)         /*!<SYSCFG DBGCR: DBG_STOP Mask */

#define SYSCFG_DBGCR_DBG_DMA_Pos     7                                            /*!<SYSCFG DBGCR: DBG_DMA Position */
#define SYSCFG_DBGCR_DBG_DMA_Msk     (0x1UL << SYSCFG_DBGCR_DBG_DMA_Pos)          /*!<SYSCFG DBGCR: DBG_DMA Mask */

#define SYSCFG_DBGCR_DBG_IWDT_Pos    8                                            /*!<SYSCFG DBGCR: DBG_IWDT Position */
#define SYSCFG_DBGCR_DBG_IWDT_Msk    (0x1UL << SYSCFG_DBGCR_DBG_IWDT_Pos)         /*!<SYSCFG DBGCR: DBG_IWDT Mask */

#define SYSCFG_DBGCR_DBG_WWDT_Pos    9                                            /*!<SYSCFG DBGCR: DBG_WWDT Position */
#define SYSCFG_DBGCR_DBG_WWDT_Msk    (0x1UL << SYSCFG_DBGCR_DBG_WWDT_Pos)         /*!<SYSCFG DBGCR: DBG_WWDT Mask */

#define SYSCFG_DBGCR_DBG_GPT_Pos     10                                           /*!<SYSCFG DBGCR: DBG_GPT Position */
#define SYSCFG_DBGCR_DBG_GPT_Msk     (0x1UL << SYSCFG_DBGCR_DBG_GPT_Pos)          /*!<SYSCFG DBGCR: DBG_GPT Mask */

#define SYSCFG_DBGCR_DBG_TIM_Pos     11                                           /*!<SYSCFG DBGCR: DBG_TIM Position */
#define SYSCFG_DBGCR_DBG_TIM_Msk     (0x1UL << SYSCFG_DBGCR_DBG_TIM_Pos)          /*!<SYSCFG DBGCR: DBG_TIM Mask */

#define SYSCFG_DBGCR_DBG_MCM_Pos     12                                           /*!<SYSCFG DBGCR: DBG_MCM Position */
#define SYSCFG_DBGCR_DBG_MCM_Msk     (0x1UL << SYSCFG_DBGCR_DBG_MCM_Pos)          /*!<SYSCFG DBGCR: DBG_MCM Mask */

#define SYSCFG_DBGCR_DBG_UART_Pos    13                                           /*!<SYSCFG DBGCR: DBG_UART Position */
#define SYSCFG_DBGCR_DBG_UART_Msk    (0x1UL << SYSCFG_DBGCR_DBG_UART_Pos)         /*!<SYSCFG DBGCR: DBG_UART Mask */

#define SYSCFG_DBGCR_DBG_SPI_Pos     14                                           /*!<SYSCFG DBGCR: DBG_SPI Position */
#define SYSCFG_DBGCR_DBG_SPI_Msk     (0x1UL << SYSCFG_DBGCR_DBG_SPI_Pos)          /*!<SYSCFG DBGCR: DBG_SPI Mask */

#define SYSCFG_DBGCR_DBG_TWI_Pos     15                                           /*!<SYSCFG DBGCR: DBG_TWI Position */
#define SYSCFG_DBGCR_DBG_TWI_Msk     (0x1UL << SYSCFG_DBGCR_DBG_TWI_Pos)          /*!<SYSCFG DBGCR: DBG_TWI Mask */

#define SYSCFG_DBGCR_LOCK_Pos        16                                           /*!<SYSCFG DBGCR: LOCK Position */
#define SYSCFG_DBGCR_LOCK_Msk        (0xFFFFUL << SYSCFG_DBGCR_LOCK_Pos)          /*!<SYSCFG DBGCR: LOCK Mask */

#define SYSCFG_DBGCR_Msk_ALL         (SYSCFG_DBGCR_DBG_SLEEP_Msk   |\
                                      SYSCFG_DBGCR_DBG_STOP_Msk    |\
                                      SYSCFG_DBGCR_DBG_DMA_Msk     |\
                                      SYSCFG_DBGCR_DBG_IWDT_Msk    |\
                                      SYSCFG_DBGCR_DBG_WWDT_Msk    |\
                                      SYSCFG_DBGCR_DBG_GPT_Msk     |\
                                      SYSCFG_DBGCR_DBG_TIM_Msk     |\
                                      SYSCFG_DBGCR_DBG_MCM_Msk     |\
                                      SYSCFG_DBGCR_DBG_UART_Msk    |\
                                      SYSCFG_DBGCR_DBG_SPI_Msk     |\
                                      SYSCFG_DBGCR_DBG_TWI_Msk     |\
                                      SYSCFG_DBGCR_LOCK_Msk        )

#define RCC_CR_SW_Pos                0                                            /*!<RCC CR: SW Position */
#define RCC_CR_SW_Msk                (0x3UL /*<< RCC_CR_SW_Pos*/)                 /*!<RCC CR: SW Mask */

#define RCC_CR_SWS_Pos               2                                            /*!<RCC CR: SWS Position */
#define RCC_CR_SWS_Msk               (0x3UL << RCC_CR_SWS_Pos)                    /*!<RCC CR: SWS Mask */

#define RCC_CR_HSEON_Pos             4                                            /*!<RCC CR: HSEON Position */
#define RCC_CR_HSEON_Msk             (0x1UL << RCC_CR_HSEON_Pos)                  /*!<RCC CR: HSEON Mask */

#define RCC_CR_HSERDY_Pos            5                                            /*!<RCC CR: HSERDY Position */
#define RCC_CR_HSERDY_Msk            (0x1UL << RCC_CR_HSERDY_Pos)                 /*!<RCC CR: HSERDY Mask */

#define RCC_CR_PLLON_Pos             6                                            /*!<RCC CR: PLLON Position */
#define RCC_CR_PLLON_Msk             (0x1UL << RCC_CR_PLLON_Pos)                  /*!<RCC CR: PLLON Mask */

#define RCC_CR_PLLRDY_Pos            7                                            /*!<RCC CR: PLLRDY Position */
#define RCC_CR_PLLRDY_Msk            (0x1UL << RCC_CR_PLLRDY_Pos)                 /*!<RCC CR: PLLRDY Mask */

#define RCC_CR_Msk_ALL               (RCC_CR_SW_Msk                |\
                                      RCC_CR_SWS_Msk               |\
                                      RCC_CR_HSEON_Msk             |\
                                      RCC_CR_HSERDY_Msk            |\
                                      RCC_CR_PLLON_Msk             |\
                                      RCC_CR_PLLRDY_Msk            )

#define RCC_CFGR_HPRE_Pos            0                                            /*!<RCC CFGR: HPRE Position */
#define RCC_CFGR_HPRE_Msk            (0x7UL /*<< RCC_CFGR_HPRE_Pos*/)             /*!<RCC CFGR: HPRE Mask */

#define RCC_CFGR_PPRE1_Pos           3                                            /*!<RCC CFGR: PPRE1 Position */
#define RCC_CFGR_PPRE1_Msk           (0x7UL << RCC_CFGR_PPRE1_Pos)                /*!<RCC CFGR: PPRE1 Mask */

#define RCC_CFGR_PPRE2_Pos           6                                            /*!<RCC CFGR: PPRE2 Position */
#define RCC_CFGR_PPRE2_Msk           (0x7UL << RCC_CFGR_PPRE2_Pos)                /*!<RCC CFGR: PPRE2 Mask */

#define RCC_CFGR_PLLK_Pos            9                                            /*!<RCC CFGR: PLLK Position */
#define RCC_CFGR_PLLK_Msk            (0x7UL << RCC_CFGR_PLLK_Pos)                 /*!<RCC CFGR: PLLK Mask */

#define RCC_CFGR_PLLF_Pos            12                                           /*!<RCC CFGR: PLLF Position */
#define RCC_CFGR_PLLF_Msk            (0x3FUL << RCC_CFGR_PLLF_Pos)                /*!<RCC CFGR: PLLF Mask */

#define RCC_CFGR_PLLSRC_Pos          18                                           /*!<RCC CFGR: PLLSRC Position */
#define RCC_CFGR_PLLSRC_Msk          (0x1UL << RCC_CFGR_PLLSRC_Pos)               /*!<RCC CFGR: PLLSRC Mask */

#define RCC_CFGR_PLLXTPRE_Pos        19                                           /*!<RCC CFGR: PLLXTPRE Position */
#define RCC_CFGR_PLLXTPRE_Msk        (0x1UL << RCC_CFGR_PLLXTPRE_Pos)             /*!<RCC CFGR: PLLXTPRE Mask */

#define RCC_CFGR_Msk_ALL             (RCC_CFGR_HPRE_Msk            |\
                                      RCC_CFGR_PPRE1_Msk           |\
                                      RCC_CFGR_PPRE2_Msk           |\
                                      RCC_CFGR_PLLK_Msk            |\
                                      RCC_CFGR_PLLF_Msk            |\
                                      RCC_CFGR_PLLSRC_Msk          |\
                                      RCC_CFGR_PLLXTPRE_Msk        )

#define RCC_CIENR_HSERDYIE_Pos       3                                            /*!<RCC CIENR: HSERDYIE Position */
#define RCC_CIENR_HSERDYIE_Msk       (0x1UL << RCC_CIENR_HSERDYIE_Pos)            /*!<RCC CIENR: HSERDYIE Mask */

#define RCC_CIENR_PLLRDYIE_Pos       4                                            /*!<RCC CIENR: PLLRDYIE Position */
#define RCC_CIENR_PLLRDYIE_Msk       (0x1UL << RCC_CIENR_PLLRDYIE_Pos)            /*!<RCC CIENR: PLLRDYIE Mask */

#define RCC_CIENR_Msk_ALL            (RCC_CIENR_HSERDYIE_Msk       |\
                                      RCC_CIENR_PLLRDYIE_Msk       )

#define RCC_CISTR_HSERDYIF_Pos       3                                            /*!<RCC CISTR: HSERDYIF Position */
#define RCC_CISTR_HSERDYIF_Msk       (0x1UL << RCC_CISTR_HSERDYIF_Pos)            /*!<RCC CISTR: HSERDYIF Mask */

#define RCC_CISTR_PLLRDYIF_Pos       4                                            /*!<RCC CISTR: PLLRDYIF Position */
#define RCC_CISTR_PLLRDYIF_Msk       (0x1UL << RCC_CISTR_PLLRDYIF_Pos)            /*!<RCC CISTR: PLLRDYIF Mask */

#define RCC_CISTR_CSMHSEF_Pos        6                                            /*!<RCC CISTR: CSMHSEF Position */
#define RCC_CISTR_CSMHSEF_Msk        (0x1UL << RCC_CISTR_CSMHSEF_Pos)             /*!<RCC CISTR: CSMHSEF Mask */

#define RCC_CISTR_CSMPLLF_Pos        7                                            /*!<RCC CISTR: CSMPLLF Position */
#define RCC_CISTR_CSMPLLF_Msk        (0x1UL << RCC_CISTR_CSMPLLF_Pos)             /*!<RCC CISTR: CSMPLLF Mask */

#define RCC_CISTR_Msk_ALL            (RCC_CISTR_HSERDYIF_Msk       |\
                                      RCC_CISTR_PLLRDYIF_Msk       |\
                                      RCC_CISTR_CSMHSEF_Msk        |\
                                      RCC_CISTR_CSMPLLF_Msk        )

#define RCC_CICLR_HSERDYC_Pos        3                                            /*!<RCC CICLR: HSERDYC Position */
#define RCC_CICLR_HSERDYC_Msk        (0x1UL << RCC_CICLR_HSERDYC_Pos)             /*!<RCC CICLR: HSERDYC Mask */

#define RCC_CICLR_PLLRDYC_Pos        4                                            /*!<RCC CICLR: PLLRDYC Position */
#define RCC_CICLR_PLLRDYC_Msk        (0x1UL << RCC_CICLR_PLLRDYC_Pos)             /*!<RCC CICLR: PLLRDYC Mask */

#define RCC_CICLR_CSMC_Pos           7                                            /*!<RCC CICLR: CSMC Position */
#define RCC_CICLR_CSMC_Msk           (0x1UL << RCC_CICLR_CSMC_Pos)                /*!<RCC CICLR: CSMC Mask */

#define RCC_CICLR_Msk_ALL            (RCC_CICLR_HSERDYC_Msk        |\
                                      RCC_CICLR_PLLRDYC_Msk        |\
                                      RCC_CICLR_CSMC_Msk           )

#define RCC_AHBRSTR_IOCFGRST_Pos     0                                            /*!<RCC AHBRSTR: IOCFGRST Position */
#define RCC_AHBRSTR_IOCFGRST_Msk     (0x1UL /*<< RCC_AHBRSTR_IOCFGRST_Pos*/)      /*!<RCC AHBRSTR: IOCFGRST Mask */

#define RCC_AHBRSTR_IODATRST_Pos     1                                            /*!<RCC AHBRSTR: IODATRST Position */
#define RCC_AHBRSTR_IODATRST_Msk     (0x1UL << RCC_AHBRSTR_IODATRST_Pos)          /*!<RCC AHBRSTR: IODATRST Mask */

#define RCC_AHBRSTR_ADC1RST_Pos      3                                            /*!<RCC AHBRSTR: ADC1RST Position */
#define RCC_AHBRSTR_ADC1RST_Msk      (0x1UL << RCC_AHBRSTR_ADC1RST_Pos)           /*!<RCC AHBRSTR: ADC1RST Mask */

#define RCC_AHBRSTR_ADC2RST_Pos      4                                            /*!<RCC AHBRSTR: ADC2RST Position */
#define RCC_AHBRSTR_ADC2RST_Msk      (0x1UL << RCC_AHBRSTR_ADC2RST_Pos)           /*!<RCC AHBRSTR: ADC2RST Mask */

#define RCC_AHBRSTR_ADC3RST_Pos      5                                            /*!<RCC AHBRSTR: ADC3RST Position */
#define RCC_AHBRSTR_ADC3RST_Msk      (0x1UL << RCC_AHBRSTR_ADC3RST_Pos)           /*!<RCC AHBRSTR: ADC3RST Mask */

#define RCC_AHBRSTR_SYSCFGRST_Pos    6                                            /*!<RCC AHBRSTR: SYSCFGRST Position */
#define RCC_AHBRSTR_SYSCFGRST_Msk    (0x1UL << RCC_AHBRSTR_SYSCFGRST_Pos)         /*!<RCC AHBRSTR: SYSCFGRST Mask */

#define RCC_AHBRSTR_DMARST_Pos       7                                            /*!<RCC AHBRSTR: DMARST Position */
#define RCC_AHBRSTR_DMARST_Msk       (0x1UL << RCC_AHBRSTR_DMARST_Pos)            /*!<RCC AHBRSTR: DMARST Mask */

#define RCC_AHBRSTR_MACPRST_Pos      8                                            /*!<RCC AHBRSTR: MACPRST Position */
#define RCC_AHBRSTR_MACPRST_Msk      (0x1UL << RCC_AHBRSTR_MACPRST_Pos)           /*!<RCC AHBRSTR: MACPRST Mask */

#define RCC_AHBRSTR_CRCRST_Pos       9                                            /*!<RCC AHBRSTR: CRCRST Position */
#define RCC_AHBRSTR_CRCRST_Msk       (0x1UL << RCC_AHBRSTR_CRCRST_Pos)            /*!<RCC AHBRSTR: CRCRST Mask */

#define RCC_AHBRSTR_GPT0RST_Pos      10                                           /*!<RCC AHBRSTR: GPT0RST Position */
#define RCC_AHBRSTR_GPT0RST_Msk      (0x1UL << RCC_AHBRSTR_GPT0RST_Pos)           /*!<RCC AHBRSTR: GPT0RST Mask */

#define RCC_AHBRSTR_GPT1RST_Pos      11                                           /*!<RCC AHBRSTR: GPT1RST Position */
#define RCC_AHBRSTR_GPT1RST_Msk      (0x1UL << RCC_AHBRSTR_GPT1RST_Pos)           /*!<RCC AHBRSTR: GPT1RST Mask */

#define RCC_AHBRSTR_GPT2RST_Pos      12                                           /*!<RCC AHBRSTR: GPT2RST Position */
#define RCC_AHBRSTR_GPT2RST_Msk      (0x1UL << RCC_AHBRSTR_GPT2RST_Pos)           /*!<RCC AHBRSTR: GPT2RST Mask */

#define RCC_AHBRSTR_GPT3RST_Pos      13                                           /*!<RCC AHBRSTR: GPT3RST Position */
#define RCC_AHBRSTR_GPT3RST_Msk      (0x1UL << RCC_AHBRSTR_GPT3RST_Pos)           /*!<RCC AHBRSTR: GPT3RST Mask */

#define RCC_AHBRSTR_GPTRST_Pos       14                                           /*!<RCC AHBRSTR: GPTRST Position */
#define RCC_AHBRSTR_GPTRST_Msk       (0x1UL << RCC_AHBRSTR_GPTRST_Pos)            /*!<RCC AHBRSTR: GPTRST Mask */

#define RCC_AHBRSTR_Msk_ALL          (RCC_AHBRSTR_IOCFGRST_Msk     |\
                                      RCC_AHBRSTR_IODATRST_Msk     |\
                                      RCC_AHBRSTR_ADC1RST_Msk      |\
                                      RCC_AHBRSTR_ADC2RST_Msk      |\
                                      RCC_AHBRSTR_ADC3RST_Msk      |\
                                      RCC_AHBRSTR_SYSCFGRST_Msk    |\
                                      RCC_AHBRSTR_DMARST_Msk       |\
                                      RCC_AHBRSTR_MACPRST_Msk      |\
                                      RCC_AHBRSTR_CRCRST_Msk       |\
                                      RCC_AHBRSTR_GPT0RST_Msk      |\
                                      RCC_AHBRSTR_GPT1RST_Msk      |\
                                      RCC_AHBRSTR_GPT2RST_Msk      |\
                                      RCC_AHBRSTR_GPT3RST_Msk      |\
                                      RCC_AHBRSTR_GPTRST_Msk       )

#define RCC_APB2RSTR_MCM1RST_Pos     0                                            /*!<RCC APB2RSTR: MCM1RST Position */
#define RCC_APB2RSTR_MCM1RST_Msk     (0x1UL /*<< RCC_APB2RSTR_MCM1RST_Pos*/)      /*!<RCC APB2RSTR: MCM1RST Mask */

#define RCC_APB2RSTR_MCM2RST_Pos     1                                            /*!<RCC APB2RSTR: MCM2RST Position */
#define RCC_APB2RSTR_MCM2RST_Msk     (0x1UL << RCC_APB2RSTR_MCM2RST_Pos)          /*!<RCC APB2RSTR: MCM2RST Mask */

#define RCC_APB2RSTR_Msk_ALL         (RCC_APB2RSTR_MCM1RST_Msk     |\
                                      RCC_APB2RSTR_MCM2RST_Msk     )

#define RCC_APB1RSTR_TIM5RST_Pos     0                                            /*!<RCC APB1RSTR: TIM5RST Position */
#define RCC_APB1RSTR_TIM5RST_Msk     (0x1UL /*<< RCC_APB1RSTR_TIM5RST_Pos*/)      /*!<RCC APB1RSTR: TIM5RST Mask */

#define RCC_APB1RSTR_TIM6RST_Pos     1                                            /*!<RCC APB1RSTR: TIM6RST Position */
#define RCC_APB1RSTR_TIM6RST_Msk     (0x1UL << RCC_APB1RSTR_TIM6RST_Pos)          /*!<RCC APB1RSTR: TIM6RST Mask */

#define RCC_APB1RSTR_TIM7RST_Pos     2                                            /*!<RCC APB1RSTR: TIM7RST Position */
#define RCC_APB1RSTR_TIM7RST_Msk     (0x1UL << RCC_APB1RSTR_TIM7RST_Pos)          /*!<RCC APB1RSTR: TIM7RST Mask */

#define RCC_APB1RSTR_TIM8RST_Pos     3                                            /*!<RCC APB1RSTR: TIM8RST Position */
#define RCC_APB1RSTR_TIM8RST_Msk     (0x1UL << RCC_APB1RSTR_TIM8RST_Pos)          /*!<RCC APB1RSTR: TIM8RST Mask */

#define RCC_APB1RSTR_QEIRST_Pos      4                                            /*!<RCC APB1RSTR: QEIRST Position */
#define RCC_APB1RSTR_QEIRST_Msk      (0x1UL << RCC_APB1RSTR_QEIRST_Pos)           /*!<RCC APB1RSTR: QEIRST Mask */

#define RCC_APB1RSTR_UART1RST_Pos    5                                            /*!<RCC APB1RSTR: UART1RST Position */
#define RCC_APB1RSTR_UART1RST_Msk    (0x1UL << RCC_APB1RSTR_UART1RST_Pos)         /*!<RCC APB1RSTR: UART1RST Mask */

#define RCC_APB1RSTR_UART2RST_Pos    6                                            /*!<RCC APB1RSTR: UART2RST Position */
#define RCC_APB1RSTR_UART2RST_Msk    (0x1UL << RCC_APB1RSTR_UART2RST_Pos)         /*!<RCC APB1RSTR: UART2RST Mask */

#define RCC_APB1RSTR_UART3RST_Pos    7                                            /*!<RCC APB1RSTR: UART3RST Position */
#define RCC_APB1RSTR_UART3RST_Msk    (0x1UL << RCC_APB1RSTR_UART3RST_Pos)         /*!<RCC APB1RSTR: UART3RST Mask */

#define RCC_APB1RSTR_SPI1RST_Pos     8                                            /*!<RCC APB1RSTR: SPI1RST Position */
#define RCC_APB1RSTR_SPI1RST_Msk     (0x1UL << RCC_APB1RSTR_SPI1RST_Pos)          /*!<RCC APB1RSTR: SPI1RST Mask */

#define RCC_APB1RSTR_SPI2RST_Pos     9                                            /*!<RCC APB1RSTR: SPI2RST Position */
#define RCC_APB1RSTR_SPI2RST_Msk     (0x1UL << RCC_APB1RSTR_SPI2RST_Pos)          /*!<RCC APB1RSTR: SPI2RST Mask */

#define RCC_APB1RSTR_TWI1RST_Pos     11                                           /*!<RCC APB1RSTR: TWI1RST Position */
#define RCC_APB1RSTR_TWI1RST_Msk     (0x1UL << RCC_APB1RSTR_TWI1RST_Pos)          /*!<RCC APB1RSTR: TWI1RST Mask */

#define RCC_APB1RSTR_WWDTRST_Pos     12                                           /*!<RCC APB1RSTR: WWDTRST Position */
#define RCC_APB1RSTR_WWDTRST_Msk     (0x1UL << RCC_APB1RSTR_WWDTRST_Pos)          /*!<RCC APB1RSTR: WWDTRST Mask */

#define RCC_APB1RSTR_AMOCRST_Pos     13                                           /*!<RCC APB1RSTR: AMOCRST Position */
#define RCC_APB1RSTR_AMOCRST_Msk     (0x1UL << RCC_APB1RSTR_AMOCRST_Pos)          /*!<RCC APB1RSTR: AMOCRST Mask */

#define RCC_APB1RSTR_Msk_ALL         (RCC_APB1RSTR_TIM5RST_Msk     |\
                                      RCC_APB1RSTR_TIM6RST_Msk     |\
                                      RCC_APB1RSTR_TIM7RST_Msk     |\
                                      RCC_APB1RSTR_TIM8RST_Msk     |\
                                      RCC_APB1RSTR_QEIRST_Msk      |\
                                      RCC_APB1RSTR_UART1RST_Msk    |\
                                      RCC_APB1RSTR_UART2RST_Msk    |\
                                      RCC_APB1RSTR_UART3RST_Msk    |\
                                      RCC_APB1RSTR_SPI1RST_Msk     |\
                                      RCC_APB1RSTR_SPI2RST_Msk     |\
                                      RCC_APB1RSTR_TWI1RST_Msk     |\
                                      RCC_APB1RSTR_WWDTRST_Msk     |\
                                      RCC_APB1RSTR_AMOCRST_Msk     )

#define RCC_AHBENR_IOCFGEN_Pos       0                                            /*!<RCC AHBENR: IOCFGEN Position */
#define RCC_AHBENR_IOCFGEN_Msk       (0x1UL /*<< RCC_AHBENR_IOCFGEN_Pos*/)        /*!<RCC AHBENR: IOCFGEN Mask */

#define RCC_AHBENR_IODATEN_Pos       1                                            /*!<RCC AHBENR: IODATEN Position */
#define RCC_AHBENR_IODATEN_Msk       (0x1UL << RCC_AHBENR_IODATEN_Pos)            /*!<RCC AHBENR: IODATEN Mask */

#define RCC_AHBENR_ADC1EN_Pos        3                                            /*!<RCC AHBENR: ADC1EN Position */
#define RCC_AHBENR_ADC1EN_Msk        (0x1UL << RCC_AHBENR_ADC1EN_Pos)             /*!<RCC AHBENR: ADC1EN Mask */

#define RCC_AHBENR_ADC2EN_Pos        4                                            /*!<RCC AHBENR: ADC2EN Position */
#define RCC_AHBENR_ADC2EN_Msk        (0x1UL << RCC_AHBENR_ADC2EN_Pos)             /*!<RCC AHBENR: ADC2EN Mask */

#define RCC_AHBENR_ADC3EN_Pos        5                                            /*!<RCC AHBENR: ADC3EN Position */
#define RCC_AHBENR_ADC3EN_Msk        (0x1UL << RCC_AHBENR_ADC3EN_Pos)             /*!<RCC AHBENR: ADC3EN Mask */

#define RCC_AHBENR_SYSCFGEN_Pos      6                                            /*!<RCC AHBENR: SYSCFGEN Position */
#define RCC_AHBENR_SYSCFGEN_Msk      (0x1UL << RCC_AHBENR_SYSCFGEN_Pos)           /*!<RCC AHBENR: SYSCFGEN Mask */

#define RCC_AHBENR_DMAEN_Pos         7                                            /*!<RCC AHBENR: DMAEN Position */
#define RCC_AHBENR_DMAEN_Msk         (0x1UL << RCC_AHBENR_DMAEN_Pos)              /*!<RCC AHBENR: DMAEN Mask */

#define RCC_AHBENR_MACPEN_Pos        8                                            /*!<RCC AHBENR: MACPEN Position */
#define RCC_AHBENR_MACPEN_Msk        (0x1UL << RCC_AHBENR_MACPEN_Pos)             /*!<RCC AHBENR: MACPEN Mask */

#define RCC_AHBENR_CRCEN_Pos         9                                            /*!<RCC AHBENR: CRCEN Position */
#define RCC_AHBENR_CRCEN_Msk         (0x1UL << RCC_AHBENR_CRCEN_Pos)              /*!<RCC AHBENR: CRCEN Mask */

#define RCC_AHBENR_GPT0EN_Pos        10                                           /*!<RCC AHBENR: GPT0EN Position */
#define RCC_AHBENR_GPT0EN_Msk        (0x1UL << RCC_AHBENR_GPT0EN_Pos)             /*!<RCC AHBENR: GPT0EN Mask */

#define RCC_AHBENR_GPT1EN_Pos        11                                           /*!<RCC AHBENR: GPT1EN Position */
#define RCC_AHBENR_GPT1EN_Msk        (0x1UL << RCC_AHBENR_GPT1EN_Pos)             /*!<RCC AHBENR: GPT1EN Mask */

#define RCC_AHBENR_GPT2EN_Pos        12                                           /*!<RCC AHBENR: GPT2EN Position */
#define RCC_AHBENR_GPT2EN_Msk        (0x1UL << RCC_AHBENR_GPT2EN_Pos)             /*!<RCC AHBENR: GPT2EN Mask */

#define RCC_AHBENR_GPT3EN_Pos        13                                           /*!<RCC AHBENR: GPT3EN Position */
#define RCC_AHBENR_GPT3EN_Msk        (0x1UL << RCC_AHBENR_GPT3EN_Pos)             /*!<RCC AHBENR: GPT3EN Mask */

#define RCC_AHBENR_GPTEN_Pos         14                                           /*!<RCC AHBENR: GPTEN Position */
#define RCC_AHBENR_GPTEN_Msk         (0x1UL << RCC_AHBENR_GPTEN_Pos)              /*!<RCC AHBENR: GPTEN Mask */

#define RCC_AHBENR_Msk_ALL           (RCC_AHBENR_IOCFGEN_Msk       |\
                                      RCC_AHBENR_IODATEN_Msk       |\
                                      RCC_AHBENR_ADC1EN_Msk        |\
                                      RCC_AHBENR_ADC2EN_Msk        |\
                                      RCC_AHBENR_ADC3EN_Msk        |\
                                      RCC_AHBENR_SYSCFGEN_Msk      |\
                                      RCC_AHBENR_DMAEN_Msk         |\
                                      RCC_AHBENR_MACPEN_Msk        |\
                                      RCC_AHBENR_CRCEN_Msk         |\
                                      RCC_AHBENR_GPT0EN_Msk        |\
                                      RCC_AHBENR_GPT1EN_Msk        |\
                                      RCC_AHBENR_GPT2EN_Msk        |\
                                      RCC_AHBENR_GPT3EN_Msk        |\
                                      RCC_AHBENR_GPTEN_Msk         )

#define RCC_APB2ENR_MCM1EN_Pos       0                                            /*!<RCC APB2ENR: MCM1EN Position */
#define RCC_APB2ENR_MCM1EN_Msk       (0x1UL /*<< RCC_APB2ENR_MCM1EN_Pos*/)        /*!<RCC APB2ENR: MCM1EN Mask */

#define RCC_APB2ENR_MCM2EN_Pos       1                                            /*!<RCC APB2ENR: MCM2EN Position */
#define RCC_APB2ENR_MCM2EN_Msk       (0x1UL << RCC_APB2ENR_MCM2EN_Pos)            /*!<RCC APB2ENR: MCM2EN Mask */

#define RCC_APB2ENR_Msk_ALL          (RCC_APB2ENR_MCM1EN_Msk       |\
                                      RCC_APB2ENR_MCM2EN_Msk       )

#define RCC_APB1ENR_TIM5EN_Pos       0                                            /*!<RCC APB1ENR: TIM5EN Position */
#define RCC_APB1ENR_TIM5EN_Msk       (0x1UL /*<< RCC_APB1ENR_TIM5EN_Pos*/)        /*!<RCC APB1ENR: TIM5EN Mask */

#define RCC_APB1ENR_TIM6EN_Pos       1                                            /*!<RCC APB1ENR: TIM6EN Position */
#define RCC_APB1ENR_TIM6EN_Msk       (0x1UL << RCC_APB1ENR_TIM6EN_Pos)            /*!<RCC APB1ENR: TIM6EN Mask */

#define RCC_APB1ENR_TIM7EN_Pos       2                                            /*!<RCC APB1ENR: TIM7EN Position */
#define RCC_APB1ENR_TIM7EN_Msk       (0x1UL << RCC_APB1ENR_TIM7EN_Pos)            /*!<RCC APB1ENR: TIM7EN Mask */

#define RCC_APB1ENR_TIM8EN_Pos       3                                            /*!<RCC APB1ENR: TIM8EN Position */
#define RCC_APB1ENR_TIM8EN_Msk       (0x1UL << RCC_APB1ENR_TIM8EN_Pos)            /*!<RCC APB1ENR: TIM8EN Mask */

#define RCC_APB1ENR_QEIEN_Pos        4                                            /*!<RCC APB1ENR: QEIEN Position */
#define RCC_APB1ENR_QEIEN_Msk        (0x1UL << RCC_APB1ENR_QEIEN_Pos)             /*!<RCC APB1ENR: QEIEN Mask */

#define RCC_APB1ENR_UART1EN_Pos      5                                            /*!<RCC APB1ENR: UART1EN Position */
#define RCC_APB1ENR_UART1EN_Msk      (0x1UL << RCC_APB1ENR_UART1EN_Pos)           /*!<RCC APB1ENR: UART1EN Mask */

#define RCC_APB1ENR_UART2EN_Pos      6                                            /*!<RCC APB1ENR: UART2EN Position */
#define RCC_APB1ENR_UART2EN_Msk      (0x1UL << RCC_APB1ENR_UART2EN_Pos)           /*!<RCC APB1ENR: UART2EN Mask */

#define RCC_APB1ENR_UART3EN_Pos      7                                            /*!<RCC APB1ENR: UART3EN Position */
#define RCC_APB1ENR_UART3EN_Msk      (0x1UL << RCC_APB1ENR_UART3EN_Pos)           /*!<RCC APB1ENR: UART3EN Mask */

#define RCC_APB1ENR_SPI1EN_Pos       8                                            /*!<RCC APB1ENR: SPI1EN Position */
#define RCC_APB1ENR_SPI1EN_Msk       (0x1UL << RCC_APB1ENR_SPI1EN_Pos)            /*!<RCC APB1ENR: SPI1EN Mask */

#define RCC_APB1ENR_SPI2EN_Pos       9                                            /*!<RCC APB1ENR: SPI2EN Position */
#define RCC_APB1ENR_SPI2EN_Msk       (0x1UL << RCC_APB1ENR_SPI2EN_Pos)            /*!<RCC APB1ENR: SPI2EN Mask */

#define RCC_APB1ENR_TWI1EN_Pos       11                                           /*!<RCC APB1ENR: TWI1EN Position */
#define RCC_APB1ENR_TWI1EN_Msk       (0x1UL << RCC_APB1ENR_TWI1EN_Pos)            /*!<RCC APB1ENR: TWI1EN Mask */

#define RCC_APB1ENR_WWDTEN_Pos       12                                           /*!<RCC APB1ENR: WWDTEN Position */
#define RCC_APB1ENR_WWDTEN_Msk       (0x1UL << RCC_APB1ENR_WWDTEN_Pos)            /*!<RCC APB1ENR: WWDTEN Mask */

#define RCC_APB1ENR_AMOCEN_Pos       13                                           /*!<RCC APB1ENR: AMOCEN Position */
#define RCC_APB1ENR_AMOCEN_Msk       (0x1UL << RCC_APB1ENR_AMOCEN_Pos)            /*!<RCC APB1ENR: AMOCEN Mask */

#define RCC_APB1ENR_Msk_ALL          (RCC_APB1ENR_TIM5EN_Msk       |\
                                      RCC_APB1ENR_TIM6EN_Msk       |\
                                      RCC_APB1ENR_TIM7EN_Msk       |\
                                      RCC_APB1ENR_TIM8EN_Msk       |\
                                      RCC_APB1ENR_QEIEN_Msk        |\
                                      RCC_APB1ENR_UART1EN_Msk      |\
                                      RCC_APB1ENR_UART2EN_Msk      |\
                                      RCC_APB1ENR_UART3EN_Msk      |\
                                      RCC_APB1ENR_SPI1EN_Msk       |\
                                      RCC_APB1ENR_SPI2EN_Msk       |\
                                      RCC_APB1ENR_TWI1EN_Msk       |\
                                      RCC_APB1ENR_WWDTEN_Msk       |\
                                      RCC_APB1ENR_AMOCEN_Msk       )

#define RCC_RSTSTR_PINRSTF_Pos       0                                            /*!<RCC RSTSTR: PINRSTF Position */
#define RCC_RSTSTR_PINRSTF_Msk       (0x1UL /*<< RCC_RSTSTR_PINRSTF_Pos*/)        /*!<RCC RSTSTR: PINRSTF Mask */

#define RCC_RSTSTR_LVRSTF_Pos        1                                            /*!<RCC RSTSTR: LVRSTF Position */
#define RCC_RSTSTR_LVRSTF_Msk        (0x1UL << RCC_RSTSTR_LVRSTF_Pos)             /*!<RCC RSTSTR: LVRSTF Mask */

#define RCC_RSTSTR_PORSTF_Pos        2                                            /*!<RCC RSTSTR: PORSTF Position */
#define RCC_RSTSTR_PORSTF_Msk        (0x1UL << RCC_RSTSTR_PORSTF_Pos)             /*!<RCC RSTSTR: PORSTF Mask */

#define RCC_RSTSTR_SWRSTF_Pos        3                                            /*!<RCC RSTSTR: SWRSTF Position */
#define RCC_RSTSTR_SWRSTF_Msk        (0x1UL << RCC_RSTSTR_SWRSTF_Pos)             /*!<RCC RSTSTR: SWRSTF Mask */

#define RCC_RSTSTR_IWDTRSTF_Pos      4                                            /*!<RCC RSTSTR: IWDTRSTF Position */
#define RCC_RSTSTR_IWDTRSTF_Msk      (0x1UL << RCC_RSTSTR_IWDTRSTF_Pos)           /*!<RCC RSTSTR: IWDTRSTF Mask */

#define RCC_RSTSTR_WWDTRSTF_Pos      5                                            /*!<RCC RSTSTR: WWDTRSTF Position */
#define RCC_RSTSTR_WWDTRSTF_Msk      (0x1UL << RCC_RSTSTR_WWDTRSTF_Pos)           /*!<RCC RSTSTR: WWDTRSTF Mask */

#define RCC_RSTSTR_LVRSTF2_Pos       6                                            /*!<RCC RSTSTR: LVRSTF2 Position */
#define RCC_RSTSTR_LVRSTF2_Msk       (0x1UL << RCC_RSTSTR_LVRSTF2_Pos)            /*!<RCC RSTSTR: LVRSTF2 Mask */

#define RCC_RSTSTR_Msk_ALL           (RCC_RSTSTR_PINRSTF_Msk       |\
                                      RCC_RSTSTR_LVRSTF_Msk        |\
                                      RCC_RSTSTR_PORSTF_Msk        |\
                                      RCC_RSTSTR_SWRSTF_Msk        |\
                                      RCC_RSTSTR_IWDTRSTF_Msk      |\
                                      RCC_RSTSTR_WWDTRSTF_Msk      |\
                                      RCC_RSTSTR_LVRSTF2_Msk       )

#define RCC_RSTCLR_PINRSTFC_Pos      0                                            /*!<RCC RSTCLR: PINRSTFC Position */
#define RCC_RSTCLR_PINRSTFC_Msk      (0x1UL /*<< RCC_RSTCLR_PINRSTFC_Pos*/)       /*!<RCC RSTCLR: PINRSTFC Mask */

#define RCC_RSTCLR_LVRSTFC_Pos       1                                            /*!<RCC RSTCLR: LVRSTFC Position */
#define RCC_RSTCLR_LVRSTFC_Msk       (0x1UL << RCC_RSTCLR_LVRSTFC_Pos)            /*!<RCC RSTCLR: LVRSTFC Mask */

#define RCC_RSTCLR_PORSTFC_Pos       2                                            /*!<RCC RSTCLR: PORSTFC Position */
#define RCC_RSTCLR_PORSTFC_Msk       (0x1UL << RCC_RSTCLR_PORSTFC_Pos)            /*!<RCC RSTCLR: PORSTFC Mask */

#define RCC_RSTCLR_SWRSTFC_Pos       3                                            /*!<RCC RSTCLR: SWRSTFC Position */
#define RCC_RSTCLR_SWRSTFC_Msk       (0x1UL << RCC_RSTCLR_SWRSTFC_Pos)            /*!<RCC RSTCLR: SWRSTFC Mask */

#define RCC_RSTCLR_IWDTRSTFC_Pos     4                                            /*!<RCC RSTCLR: IWDTRSTFC Position */
#define RCC_RSTCLR_IWDTRSTFC_Msk     (0x1UL << RCC_RSTCLR_IWDTRSTFC_Pos)          /*!<RCC RSTCLR: IWDTRSTFC Mask */

#define RCC_RSTCLR_WWDTRSTFC_Pos     5                                            /*!<RCC RSTCLR: WWDTRSTFC Position */
#define RCC_RSTCLR_WWDTRSTFC_Msk     (0x1UL << RCC_RSTCLR_WWDTRSTFC_Pos)          /*!<RCC RSTCLR: WWDTRSTFC Mask */

#define RCC_RSTCLR_LVRSTF2C_Pos      6                                            /*!<RCC RSTCLR: LVRSTF2C Position */
#define RCC_RSTCLR_LVRSTF2C_Msk      (0x1UL << RCC_RSTCLR_LVRSTF2C_Pos)           /*!<RCC RSTCLR: LVRSTF2C Mask */

#define RCC_RSTCLR_Msk_ALL           (RCC_RSTCLR_PINRSTFC_Msk      |\
                                      RCC_RSTCLR_LVRSTFC_Msk       |\
                                      RCC_RSTCLR_PORSTFC_Msk       |\
                                      RCC_RSTCLR_SWRSTFC_Msk       |\
                                      RCC_RSTCLR_IWDTRSTFC_Msk     |\
                                      RCC_RSTCLR_WWDTRSTFC_Msk     |\
                                      RCC_RSTCLR_LVRSTF2C_Msk      )

#define RCC_HSICAL_HSITRIM_Pos       0                                            /*!<RCC HSICAL: HSITRIM Position */
#define RCC_HSICAL_HSITRIM_Msk       (0x7UL /*<< RCC_HSICAL_HSITRIM_Pos*/)        /*!<RCC HSICAL: HSITRIM Mask */

#define RCC_HSICAL_HSICAL_Pos        8                                            /*!<RCC HSICAL: HSICAL Position */
#define RCC_HSICAL_HSICAL_Msk        (0xFFUL << RCC_HSICAL_HSICAL_Pos)            /*!<RCC HSICAL: HSICAL Mask */

#define RCC_HSICAL_TRIMREF_Pos       16                                           /*!<RCC HSICAL: TRIMREF Position */
#define RCC_HSICAL_TRIMREF_Msk       (0x1FFFUL << RCC_HSICAL_TRIMREF_Pos)         /*!<RCC HSICAL: TRIMREF Mask */

#define RCC_HSICAL_TRIMRUN_Pos       31                                           /*!<RCC HSICAL: TRIMRUN Position */
#define RCC_HSICAL_TRIMRUN_Msk       (0x1UL << RCC_HSICAL_TRIMRUN_Pos)            /*!<RCC HSICAL: TRIMRUN Mask */

#define RCC_HSICAL_Msk_ALL           (RCC_HSICAL_HSITRIM_Msk       |\
                                      RCC_HSICAL_HSICAL_Msk        |\
                                      RCC_HSICAL_TRIMREF_Msk       |\
                                      RCC_HSICAL_TRIMRUN_Msk       )

#define RCC_RCCLOCK_LOCK_Pos         0                                            /*!<RCC RCCLOCK: LOCK Position */
#define RCC_RCCLOCK_LOCK_Msk         (0xFFFFUL /*<< RCC_RCCLOCK_LOCK_Pos*/)       /*!<RCC RCCLOCK: LOCK Mask */

#define RCC_RCCLOCK_Msk_ALL          (RCC_RCCLOCK_LOCK_Msk        )

#define IWDT_CR_IWDTRLR_Pos          0                                            /*!<IWDT CR: IWDTRLR Position */
#define IWDT_CR_IWDTRLR_Msk          (0xFFFUL /*<< IWDT_CR_IWDTRLR_Pos*/)         /*!<IWDT CR: IWDTRLR Mask */

#define IWDT_CR_IWDTPR_Pos           12                                           /*!<IWDT CR: IWDTPR Position */
#define IWDT_CR_IWDTPR_Msk           (0x7UL << IWDT_CR_IWDTPR_Pos)                /*!<IWDT CR: IWDTPR Mask */

#define IWDT_CR_IWDTON_Pos           15                                           /*!<IWDT CR: IWDTON Position */
#define IWDT_CR_IWDTON_Msk           (0x1UL << IWDT_CR_IWDTON_Pos)                /*!<IWDT CR: IWDTON Mask */

#define IWDT_CR_LOCK_Pos             16                                           /*!<IWDT CR: LOCK Position */
#define IWDT_CR_LOCK_Msk             (0xFFFFUL << IWDT_CR_LOCK_Pos)               /*!<IWDT CR: LOCK Mask */

#define IWDT_CR_Msk_ALL              (IWDT_CR_IWDTRLR_Msk          |\
                                      IWDT_CR_IWDTPR_Msk           |\
                                      IWDT_CR_IWDTON_Msk           |\
                                      IWDT_CR_LOCK_Msk             )

#define IWDT_CLR_IWDTCLR_Pos         0                                            /*!<IWDT CLR: IWDTCLR Position */
#define IWDT_CLR_IWDTCLR_Msk         (0xFFFFUL /*<< IWDT_CLR_IWDTCLR_Pos*/)       /*!<IWDT CLR: IWDTCLR Mask */

#define IWDT_CLR_Msk_ALL             (IWDT_CLR_IWDTCLR_Msk        )

#define WWDT_CR_WWDTRLR_Pos          0                                            /*!<WWDT CR: WWDTRLR Position */
#define WWDT_CR_WWDTRLR_Msk          (0xFFUL /*<< WWDT_CR_WWDTRLR_Pos*/)          /*!<WWDT CR: WWDTRLR Mask */

#define WWDT_CR_WWDTPR_Pos           8                                            /*!<WWDT CR: WWDTPR Position */
#define WWDT_CR_WWDTPR_Msk           (0x7UL << WWDT_CR_WWDTPR_Pos)                /*!<WWDT CR: WWDTPR Mask */

#define WWDT_CR_WWDTIE_Pos           14                                           /*!<WWDT CR: WWDTIE Position */
#define WWDT_CR_WWDTIE_Msk           (0x1UL << WWDT_CR_WWDTIE_Pos)                /*!<WWDT CR: WWDTIE Mask */

#define WWDT_CR_WWDTON_Pos           15                                           /*!<WWDT CR: WWDTON Position */
#define WWDT_CR_WWDTON_Msk           (0x1UL << WWDT_CR_WWDTON_Pos)                /*!<WWDT CR: WWDTON Mask */

#define WWDT_CR_LOCK_Pos             16                                           /*!<WWDT CR: LOCK Position */
#define WWDT_CR_LOCK_Msk             (0xFFFFUL << WWDT_CR_LOCK_Pos)               /*!<WWDT CR: LOCK Mask */

#define WWDT_CR_Msk_ALL              (WWDT_CR_WWDTRLR_Msk          |\
                                      WWDT_CR_WWDTPR_Msk           |\
                                      WWDT_CR_WWDTIE_Msk           |\
                                      WWDT_CR_WWDTON_Msk           |\
                                      WWDT_CR_LOCK_Msk             )

#define WWDT_SR_TCNT_Pos             0                                            /*!<WWDT SR: TCNT Position */
#define WWDT_SR_TCNT_Msk             (0xFFUL /*<< WWDT_SR_TCNT_Pos*/)             /*!<WWDT SR: TCNT Mask */

#define WWDT_SR_WWDTIF_Pos           15                                           /*!<WWDT SR: WWDTIF Position */
#define WWDT_SR_WWDTIF_Msk           (0x1UL << WWDT_SR_WWDTIF_Pos)                /*!<WWDT SR: WWDTIF Mask */

#define WWDT_SR_Msk_ALL              (WWDT_SR_TCNT_Msk             |\
                                      WWDT_SR_WWDTIF_Msk           )

#define WWDT_CLR_WWDTCLR_Pos         0                                            /*!<WWDT CLR: WWDTCLR Position */
#define WWDT_CLR_WWDTCLR_Msk         (0xFFFFUL /*<< WWDT_CLR_WWDTCLR_Pos*/)       /*!<WWDT CLR: WWDTCLR Mask */

#define WWDT_CLR_Msk_ALL             (WWDT_CLR_WWDTCLR_Msk        )

#define WWDT_WTR_WWDTWTR_Pos         0                                            /*!<WWDT WTR: WWDTWTR Position */
#define WWDT_WTR_WWDTWTR_Msk         (0xFFUL /*<< WWDT_WTR_WWDTWTR_Pos*/)         /*!<WWDT WTR: WWDTWTR Mask */

#define WWDT_WTR_LOCK_Pos            16                                           /*!<WWDT WTR: LOCK Position */
#define WWDT_WTR_LOCK_Msk            (0xFFFFUL << WWDT_WTR_LOCK_Pos)              /*!<WWDT WTR: LOCK Mask */

#define WWDT_WTR_Msk_ALL             (WWDT_WTR_WWDTWTR_Msk         |\
                                      WWDT_WTR_LOCK_Msk            )

#define CRC_CR_RELOAD_Pos            0                                            /*!<CRC CR: RELOAD Position */
#define CRC_CR_RELOAD_Msk            (0x1UL /*<< CRC_CR_RELOAD_Pos*/)             /*!<CRC CR: RELOAD Mask */

#define CRC_CR_MODE_Pos              8                                            /*!<CRC CR: MODE Position */
#define CRC_CR_MODE_Msk              (0x3UL << CRC_CR_MODE_Pos)                   /*!<CRC CR: MODE Mask */

#define CRC_CR_COMPLW_Pos            10                                           /*!<CRC CR: COMPLW Position */
#define CRC_CR_COMPLW_Msk            (0x1UL << CRC_CR_COMPLW_Pos)                 /*!<CRC CR: COMPLW Mask */

#define CRC_CR_RBITW_Pos             11                                           /*!<CRC CR: RBITW Position */
#define CRC_CR_RBITW_Msk             (0x1UL << CRC_CR_RBITW_Pos)                  /*!<CRC CR: RBITW Mask */

#define CRC_CR_RBYTEW_Pos            12                                           /*!<CRC CR: RBYTEW Position */
#define CRC_CR_RBYTEW_Msk            (0x1UL << CRC_CR_RBYTEW_Pos)                 /*!<CRC CR: RBYTEW Mask */

#define CRC_CR_COMPLR_Pos            13                                           /*!<CRC CR: COMPLR Position */
#define CRC_CR_COMPLR_Msk            (0x1UL << CRC_CR_COMPLR_Pos)                 /*!<CRC CR: COMPLR Mask */

#define CRC_CR_RBITR_Pos             14                                           /*!<CRC CR: RBITR Position */
#define CRC_CR_RBITR_Msk             (0x1UL << CRC_CR_RBITR_Pos)                  /*!<CRC CR: RBITR Mask */

#define CRC_CR_RBYTER_Pos            15                                           /*!<CRC CR: RBYTER Position */
#define CRC_CR_RBYTER_Msk            (0x1UL << CRC_CR_RBYTER_Pos)                 /*!<CRC CR: RBYTER Mask */

#define CRC_CR_Msk_ALL               (CRC_CR_RELOAD_Msk            |\
                                      CRC_CR_MODE_Msk              |\
                                      CRC_CR_COMPLW_Msk            |\
                                      CRC_CR_RBITW_Msk             |\
                                      CRC_CR_RBYTEW_Msk            |\
                                      CRC_CR_COMPLR_Msk            |\
                                      CRC_CR_RBITR_Msk             |\
                                      CRC_CR_RBYTER_Msk            )

#define RAMBIST_ADDR_RAMADR_Pos      0                                            /*!<RAMBIST ADDR: RAMADR Position */
#define RAMBIST_ADDR_RAMADR_Msk      (0x3FFFUL /*<< RAMBIST_ADDR_RAMADR_Pos*/)    /*!<RAMBIST ADDR: RAMADR Mask */

#define RAMBIST_ADDR_RAMTYP_Pos      28                                           /*!<RAMBIST ADDR: RAMTYP Position */
#define RAMBIST_ADDR_RAMTYP_Msk      (0x3UL << RAMBIST_ADDR_RAMTYP_Pos)           /*!<RAMBIST ADDR: RAMTYP Mask */

#define RAMBIST_ADDR_Msk_ALL         (RAMBIST_ADDR_RAMADR_Msk      |\
                                      RAMBIST_ADDR_RAMTYP_Msk      )

#define RAMBIST_CFG_BLKSZ_Pos        0                                            /*!<RAMBIST CFG: BLKSZ Position */
#define RAMBIST_CFG_BLKSZ_Msk        (0x7UL /*<< RAMBIST_CFG_BLKSZ_Pos*/)         /*!<RAMBIST CFG: BLKSZ Mask */

#define RAMBIST_CFG_SEL_Pos          15                                           /*!<RAMBIST CFG: SEL Position */
#define RAMBIST_CFG_SEL_Msk          (0x1UL << RAMBIST_CFG_SEL_Pos)               /*!<RAMBIST CFG: SEL Mask */

#define RAMBIST_CFG_Msk_ALL          (RAMBIST_CFG_BLKSZ_Msk        |\
                                      RAMBIST_CFG_SEL_Msk          )

#define RAMBIST_CSR_RUN_Pos          0                                            /*!<RAMBIST CSR: RUN Position */
#define RAMBIST_CSR_RUN_Msk          (0xFFFFUL /*<< RAMBIST_CSR_RUN_Pos*/)        /*!<RAMBIST CSR: RUN Mask */

#define RAMBIST_CSR_MOD_Pos          16                                           /*!<RAMBIST CSR: MOD Position */
#define RAMBIST_CSR_MOD_Msk          (0x1UL << RAMBIST_CSR_MOD_Pos)               /*!<RAMBIST CSR: MOD Mask */

#define RAMBIST_CSR_BSY_Pos          17                                           /*!<RAMBIST CSR: BSY Position */
#define RAMBIST_CSR_BSY_Msk          (0x1UL << RAMBIST_CSR_BSY_Pos)               /*!<RAMBIST CSR: BSY Mask */

#define RAMBIST_CSR_ERR_Pos          18                                           /*!<RAMBIST CSR: ERR Position */
#define RAMBIST_CSR_ERR_Msk          (0x1UL << RAMBIST_CSR_ERR_Pos)               /*!<RAMBIST CSR: ERR Mask */

#define RAMBIST_CSR_Msk_ALL          (RAMBIST_CSR_RUN_Msk          |\
                                      RAMBIST_CSR_MOD_Msk          |\
                                      RAMBIST_CSR_BSY_Msk          |\
                                      RAMBIST_CSR_ERR_Msk          )

#define GPIO_CFG_OTYPER_OT_Pos       0                                            /*!<GPIO_CFG OTYPER: OT Position */
#define GPIO_CFG_OTYPER_OT_Msk       (0xFFFFUL /*<< GPIO_CFG_OTYPER_OT_Pos*/)     /*!<GPIO_CFG OTYPER: OT Mask */
#define GPIO_CFG_OTYPER_OT0_Msk      (0x1UL /*<< GPIO_CFG_OTYPER_OT_Pos*/)        /*!<GPIO_CFG OTYPER: OT0 Mask */

#define GPIO_CFG_OTYPER_Msk_ALL      (GPIO_CFG_OTYPER_OT_Msk      )

#define GPIO_CFG_ODRVR_ODRVR0_Pos    0                                            /*!<GPIO_CFG ODRVR: ODRVR0 Position */
#define GPIO_CFG_ODRVR_ODRVR0_Msk    (0x3UL /*<< GPIO_CFG_ODRVR_ODRVR0_Pos*/)     /*!<GPIO_CFG ODRVR: ODRVR0 Mask */

#define GPIO_CFG_ODRVR_ODRVR1_Pos    2                                            /*!<GPIO_CFG ODRVR: ODRVR1 Position */
#define GPIO_CFG_ODRVR_ODRVR1_Msk    (0x3UL << GPIO_CFG_ODRVR_ODRVR1_Pos)         /*!<GPIO_CFG ODRVR: ODRVR1 Mask */

#define GPIO_CFG_ODRVR_ODRVR2_Pos    4                                            /*!<GPIO_CFG ODRVR: ODRVR2 Position */
#define GPIO_CFG_ODRVR_ODRVR2_Msk    (0x3UL << GPIO_CFG_ODRVR_ODRVR2_Pos)         /*!<GPIO_CFG ODRVR: ODRVR2 Mask */

#define GPIO_CFG_ODRVR_ODRVR3_Pos    6                                            /*!<GPIO_CFG ODRVR: ODRVR3 Position */
#define GPIO_CFG_ODRVR_ODRVR3_Msk    (0x3UL << GPIO_CFG_ODRVR_ODRVR3_Pos)         /*!<GPIO_CFG ODRVR: ODRVR3 Mask */

#define GPIO_CFG_ODRVR_ODRVR4_Pos    8                                            /*!<GPIO_CFG ODRVR: ODRVR4 Position */
#define GPIO_CFG_ODRVR_ODRVR4_Msk    (0x3UL << GPIO_CFG_ODRVR_ODRVR4_Pos)         /*!<GPIO_CFG ODRVR: ODRVR4 Mask */

#define GPIO_CFG_ODRVR_ODRVR5_Pos    10                                           /*!<GPIO_CFG ODRVR: ODRVR5 Position */
#define GPIO_CFG_ODRVR_ODRVR5_Msk    (0x3UL << GPIO_CFG_ODRVR_ODRVR5_Pos)         /*!<GPIO_CFG ODRVR: ODRVR5 Mask */

#define GPIO_CFG_ODRVR_ODRVR6_Pos    12                                           /*!<GPIO_CFG ODRVR: ODRVR6 Position */
#define GPIO_CFG_ODRVR_ODRVR6_Msk    (0x3UL << GPIO_CFG_ODRVR_ODRVR6_Pos)         /*!<GPIO_CFG ODRVR: ODRVR6 Mask */

#define GPIO_CFG_ODRVR_ODRVR7_Pos    14                                           /*!<GPIO_CFG ODRVR: ODRVR7 Position */
#define GPIO_CFG_ODRVR_ODRVR7_Msk    (0x3UL << GPIO_CFG_ODRVR_ODRVR7_Pos)         /*!<GPIO_CFG ODRVR: ODRVR7 Mask */

#define GPIO_CFG_ODRVR_ODRVR8_Pos    16                                           /*!<GPIO_CFG ODRVR: ODRVR8 Position */
#define GPIO_CFG_ODRVR_ODRVR8_Msk    (0x3UL << GPIO_CFG_ODRVR_ODRVR8_Pos)         /*!<GPIO_CFG ODRVR: ODRVR8 Mask */

#define GPIO_CFG_ODRVR_ODRVR9_Pos    18                                           /*!<GPIO_CFG ODRVR: ODRVR9 Position */
#define GPIO_CFG_ODRVR_ODRVR9_Msk    (0x3UL << GPIO_CFG_ODRVR_ODRVR9_Pos)         /*!<GPIO_CFG ODRVR: ODRVR9 Mask */

#define GPIO_CFG_ODRVR_ODRVR10_Pos   20                                           /*!<GPIO_CFG ODRVR: ODRVR10 Position */
#define GPIO_CFG_ODRVR_ODRVR10_Msk   (0x3UL << GPIO_CFG_ODRVR_ODRVR10_Pos)        /*!<GPIO_CFG ODRVR: ODRVR10 Mask */

#define GPIO_CFG_ODRVR_ODRVR11_Pos   22                                           /*!<GPIO_CFG ODRVR: ODRVR11 Position */
#define GPIO_CFG_ODRVR_ODRVR11_Msk   (0x3UL << GPIO_CFG_ODRVR_ODRVR11_Pos)        /*!<GPIO_CFG ODRVR: ODRVR11 Mask */

#define GPIO_CFG_ODRVR_ODRVR12_Pos   24                                           /*!<GPIO_CFG ODRVR: ODRVR12 Position */
#define GPIO_CFG_ODRVR_ODRVR12_Msk   (0x3UL << GPIO_CFG_ODRVR_ODRVR12_Pos)        /*!<GPIO_CFG ODRVR: ODRVR12 Mask */

#define GPIO_CFG_ODRVR_ODRVR13_Pos   26                                           /*!<GPIO_CFG ODRVR: ODRVR13 Position */
#define GPIO_CFG_ODRVR_ODRVR13_Msk   (0x3UL << GPIO_CFG_ODRVR_ODRVR13_Pos)        /*!<GPIO_CFG ODRVR: ODRVR13 Mask */

#define GPIO_CFG_ODRVR_ODRVR14_Pos   28                                           /*!<GPIO_CFG ODRVR: ODRVR14 Position */
#define GPIO_CFG_ODRVR_ODRVR14_Msk   (0x3UL << GPIO_CFG_ODRVR_ODRVR14_Pos)        /*!<GPIO_CFG ODRVR: ODRVR14 Mask */

#define GPIO_CFG_ODRVR_ODRVR15_Pos   30                                           /*!<GPIO_CFG ODRVR: ODRVR15 Position */
#define GPIO_CFG_ODRVR_ODRVR15_Msk   (0x3UL << GPIO_CFG_ODRVR_ODRVR15_Pos)        /*!<GPIO_CFG ODRVR: ODRVR15 Mask */

#define GPIO_CFG_ODRVR_Msk_ALL       (GPIO_CFG_ODRVR_ODRVR0_Msk    |\
                                      GPIO_CFG_ODRVR_ODRVR1_Msk    |\
                                      GPIO_CFG_ODRVR_ODRVR2_Msk    |\
                                      GPIO_CFG_ODRVR_ODRVR3_Msk    |\
                                      GPIO_CFG_ODRVR_ODRVR4_Msk    |\
                                      GPIO_CFG_ODRVR_ODRVR5_Msk    |\
                                      GPIO_CFG_ODRVR_ODRVR6_Msk    |\
                                      GPIO_CFG_ODRVR_ODRVR7_Msk    |\
                                      GPIO_CFG_ODRVR_ODRVR8_Msk    |\
                                      GPIO_CFG_ODRVR_ODRVR9_Msk    |\
                                      GPIO_CFG_ODRVR_ODRVR10_Msk   |\
                                      GPIO_CFG_ODRVR_ODRVR11_Msk   |\
                                      GPIO_CFG_ODRVR_ODRVR12_Msk   |\
                                      GPIO_CFG_ODRVR_ODRVR13_Msk   |\
                                      GPIO_CFG_ODRVR_ODRVR14_Msk   |\
                                      GPIO_CFG_ODRVR_ODRVR15_Msk   )

#define GPIO_CFG_PUPDR_PUPDR0_Pos    0                                            /*!<GPIO_CFG PUPDR: PUPDR0 Position */
#define GPIO_CFG_PUPDR_PUPDR0_Msk    (0x3UL /*<< GPIO_CFG_PUPDR_PUPDR0_Pos*/)     /*!<GPIO_CFG PUPDR: PUPDR0 Mask */

#define GPIO_CFG_PUPDR_PUPDR1_Pos    2                                            /*!<GPIO_CFG PUPDR: PUPDR1 Position */
#define GPIO_CFG_PUPDR_PUPDR1_Msk    (0x3UL << GPIO_CFG_PUPDR_PUPDR1_Pos)         /*!<GPIO_CFG PUPDR: PUPDR1 Mask */

#define GPIO_CFG_PUPDR_PUPDR2_Pos    4                                            /*!<GPIO_CFG PUPDR: PUPDR2 Position */
#define GPIO_CFG_PUPDR_PUPDR2_Msk    (0x3UL << GPIO_CFG_PUPDR_PUPDR2_Pos)         /*!<GPIO_CFG PUPDR: PUPDR2 Mask */

#define GPIO_CFG_PUPDR_PUPDR3_Pos    6                                            /*!<GPIO_CFG PUPDR: PUPDR3 Position */
#define GPIO_CFG_PUPDR_PUPDR3_Msk    (0x3UL << GPIO_CFG_PUPDR_PUPDR3_Pos)         /*!<GPIO_CFG PUPDR: PUPDR3 Mask */

#define GPIO_CFG_PUPDR_PUPDR4_Pos    8                                            /*!<GPIO_CFG PUPDR: PUPDR4 Position */
#define GPIO_CFG_PUPDR_PUPDR4_Msk    (0x3UL << GPIO_CFG_PUPDR_PUPDR4_Pos)         /*!<GPIO_CFG PUPDR: PUPDR4 Mask */

#define GPIO_CFG_PUPDR_PUPDR5_Pos    10                                           /*!<GPIO_CFG PUPDR: PUPDR5 Position */
#define GPIO_CFG_PUPDR_PUPDR5_Msk    (0x3UL << GPIO_CFG_PUPDR_PUPDR5_Pos)         /*!<GPIO_CFG PUPDR: PUPDR5 Mask */

#define GPIO_CFG_PUPDR_PUPDR6_Pos    12                                           /*!<GPIO_CFG PUPDR: PUPDR6 Position */
#define GPIO_CFG_PUPDR_PUPDR6_Msk    (0x3UL << GPIO_CFG_PUPDR_PUPDR6_Pos)         /*!<GPIO_CFG PUPDR: PUPDR6 Mask */

#define GPIO_CFG_PUPDR_PUPDR7_Pos    14                                           /*!<GPIO_CFG PUPDR: PUPDR7 Position */
#define GPIO_CFG_PUPDR_PUPDR7_Msk    (0x3UL << GPIO_CFG_PUPDR_PUPDR7_Pos)         /*!<GPIO_CFG PUPDR: PUPDR7 Mask */

#define GPIO_CFG_PUPDR_PUPDR8_Pos    16                                           /*!<GPIO_CFG PUPDR: PUPDR8 Position */
#define GPIO_CFG_PUPDR_PUPDR8_Msk    (0x3UL << GPIO_CFG_PUPDR_PUPDR8_Pos)         /*!<GPIO_CFG PUPDR: PUPDR8 Mask */

#define GPIO_CFG_PUPDR_PUPDR9_Pos    18                                           /*!<GPIO_CFG PUPDR: PUPDR9 Position */
#define GPIO_CFG_PUPDR_PUPDR9_Msk    (0x3UL << GPIO_CFG_PUPDR_PUPDR9_Pos)         /*!<GPIO_CFG PUPDR: PUPDR9 Mask */

#define GPIO_CFG_PUPDR_PUPDR10_Pos   20                                           /*!<GPIO_CFG PUPDR: PUPDR10 Position */
#define GPIO_CFG_PUPDR_PUPDR10_Msk   (0x3UL << GPIO_CFG_PUPDR_PUPDR10_Pos)        /*!<GPIO_CFG PUPDR: PUPDR10 Mask */

#define GPIO_CFG_PUPDR_PUPDR11_Pos   22                                           /*!<GPIO_CFG PUPDR: PUPDR11 Position */
#define GPIO_CFG_PUPDR_PUPDR11_Msk   (0x3UL << GPIO_CFG_PUPDR_PUPDR11_Pos)        /*!<GPIO_CFG PUPDR: PUPDR11 Mask */

#define GPIO_CFG_PUPDR_PUPDR12_Pos   24                                           /*!<GPIO_CFG PUPDR: PUPDR12 Position */
#define GPIO_CFG_PUPDR_PUPDR12_Msk   (0x3UL << GPIO_CFG_PUPDR_PUPDR12_Pos)        /*!<GPIO_CFG PUPDR: PUPDR12 Mask */

#define GPIO_CFG_PUPDR_PUPDR13_Pos   26                                           /*!<GPIO_CFG PUPDR: PUPDR13 Position */
#define GPIO_CFG_PUPDR_PUPDR13_Msk   (0x3UL << GPIO_CFG_PUPDR_PUPDR13_Pos)        /*!<GPIO_CFG PUPDR: PUPDR13 Mask */

#define GPIO_CFG_PUPDR_PUPDR14_Pos   28                                           /*!<GPIO_CFG PUPDR: PUPDR14 Position */
#define GPIO_CFG_PUPDR_PUPDR14_Msk   (0x3UL << GPIO_CFG_PUPDR_PUPDR14_Pos)        /*!<GPIO_CFG PUPDR: PUPDR14 Mask */

#define GPIO_CFG_PUPDR_PUPDR15_Pos   30                                           /*!<GPIO_CFG PUPDR: PUPDR15 Position */
#define GPIO_CFG_PUPDR_PUPDR15_Msk   (0x3UL << GPIO_CFG_PUPDR_PUPDR15_Pos)        /*!<GPIO_CFG PUPDR: PUPDR15 Mask */

#define GPIO_CFG_PUPDR_Msk_ALL       (GPIO_CFG_PUPDR_PUPDR0_Msk    |\
                                      GPIO_CFG_PUPDR_PUPDR1_Msk    |\
                                      GPIO_CFG_PUPDR_PUPDR2_Msk    |\
                                      GPIO_CFG_PUPDR_PUPDR3_Msk    |\
                                      GPIO_CFG_PUPDR_PUPDR4_Msk    |\
                                      GPIO_CFG_PUPDR_PUPDR5_Msk    |\
                                      GPIO_CFG_PUPDR_PUPDR6_Msk    |\
                                      GPIO_CFG_PUPDR_PUPDR7_Msk    |\
                                      GPIO_CFG_PUPDR_PUPDR8_Msk    |\
                                      GPIO_CFG_PUPDR_PUPDR9_Msk    |\
                                      GPIO_CFG_PUPDR_PUPDR10_Msk   |\
                                      GPIO_CFG_PUPDR_PUPDR11_Msk   |\
                                      GPIO_CFG_PUPDR_PUPDR12_Msk   |\
                                      GPIO_CFG_PUPDR_PUPDR13_Msk   |\
                                      GPIO_CFG_PUPDR_PUPDR14_Msk   |\
                                      GPIO_CFG_PUPDR_PUPDR15_Msk   )

#define GPIO_CFG_AFRL_AFR0_Pos       0                                            /*!<GPIO_CFG AFRL: AFR0 Position */
#define GPIO_CFG_AFRL_AFR0_Msk       (0xFUL /*<< GPIO_CFG_AFRL_AFR0_Pos*/)        /*!<GPIO_CFG AFRL: AFR0 Mask */

#define GPIO_CFG_AFRL_AFR1_Pos       4                                            /*!<GPIO_CFG AFRL: AFR1 Position */
#define GPIO_CFG_AFRL_AFR1_Msk       (0xFUL << GPIO_CFG_AFRL_AFR1_Pos)            /*!<GPIO_CFG AFRL: AFR1 Mask */

#define GPIO_CFG_AFRL_AFR2_Pos       8                                            /*!<GPIO_CFG AFRL: AFR2 Position */
#define GPIO_CFG_AFRL_AFR2_Msk       (0xFUL << GPIO_CFG_AFRL_AFR2_Pos)            /*!<GPIO_CFG AFRL: AFR2 Mask */

#define GPIO_CFG_AFRL_AFR3_Pos       12                                           /*!<GPIO_CFG AFRL: AFR3 Position */
#define GPIO_CFG_AFRL_AFR3_Msk       (0xFUL << GPIO_CFG_AFRL_AFR3_Pos)            /*!<GPIO_CFG AFRL: AFR3 Mask */

#define GPIO_CFG_AFRL_AFR4_Pos       16                                           /*!<GPIO_CFG AFRL: AFR4 Position */
#define GPIO_CFG_AFRL_AFR4_Msk       (0xFUL << GPIO_CFG_AFRL_AFR4_Pos)            /*!<GPIO_CFG AFRL: AFR4 Mask */

#define GPIO_CFG_AFRL_AFR5_Pos       20                                           /*!<GPIO_CFG AFRL: AFR5 Position */
#define GPIO_CFG_AFRL_AFR5_Msk       (0xFUL << GPIO_CFG_AFRL_AFR5_Pos)            /*!<GPIO_CFG AFRL: AFR5 Mask */

#define GPIO_CFG_AFRL_AFR6_Pos       24                                           /*!<GPIO_CFG AFRL: AFR6 Position */
#define GPIO_CFG_AFRL_AFR6_Msk       (0xFUL << GPIO_CFG_AFRL_AFR6_Pos)            /*!<GPIO_CFG AFRL: AFR6 Mask */

#define GPIO_CFG_AFRL_AFR7_Pos       28                                           /*!<GPIO_CFG AFRL: AFR7 Position */
#define GPIO_CFG_AFRL_AFR7_Msk       (0xFUL << GPIO_CFG_AFRL_AFR7_Pos)            /*!<GPIO_CFG AFRL: AFR7 Mask */

#define GPIO_CFG_AFRL_Msk_ALL        (GPIO_CFG_AFRL_AFR0_Msk       |\
                                      GPIO_CFG_AFRL_AFR1_Msk       |\
                                      GPIO_CFG_AFRL_AFR2_Msk       |\
                                      GPIO_CFG_AFRL_AFR3_Msk       |\
                                      GPIO_CFG_AFRL_AFR4_Msk       |\
                                      GPIO_CFG_AFRL_AFR5_Msk       |\
                                      GPIO_CFG_AFRL_AFR6_Msk       |\
                                      GPIO_CFG_AFRL_AFR7_Msk       )

#define GPIO_CFG_AFRH_AFR8_Pos       0                                            /*!<GPIO_CFG AFRH: AFR8 Position */
#define GPIO_CFG_AFRH_AFR8_Msk       (0xFUL /*<< GPIO_CFG_AFRH_AFR8_Pos*/)        /*!<GPIO_CFG AFRH: AFR8 Mask */

#define GPIO_CFG_AFRH_AFR9_Pos       4                                            /*!<GPIO_CFG AFRH: AFR9 Position */
#define GPIO_CFG_AFRH_AFR9_Msk       (0xFUL << GPIO_CFG_AFRH_AFR9_Pos)            /*!<GPIO_CFG AFRH: AFR9 Mask */

#define GPIO_CFG_AFRH_AFR10_Pos      8                                            /*!<GPIO_CFG AFRH: AFR10 Position */
#define GPIO_CFG_AFRH_AFR10_Msk      (0xFUL << GPIO_CFG_AFRH_AFR10_Pos)           /*!<GPIO_CFG AFRH: AFR10 Mask */

#define GPIO_CFG_AFRH_AFR11_Pos      12                                           /*!<GPIO_CFG AFRH: AFR11 Position */
#define GPIO_CFG_AFRH_AFR11_Msk      (0xFUL << GPIO_CFG_AFRH_AFR11_Pos)           /*!<GPIO_CFG AFRH: AFR11 Mask */

#define GPIO_CFG_AFRH_AFR12_Pos      16                                           /*!<GPIO_CFG AFRH: AFR12 Position */
#define GPIO_CFG_AFRH_AFR12_Msk      (0xFUL << GPIO_CFG_AFRH_AFR12_Pos)           /*!<GPIO_CFG AFRH: AFR12 Mask */

#define GPIO_CFG_AFRH_AFR13_Pos      20                                           /*!<GPIO_CFG AFRH: AFR13 Position */
#define GPIO_CFG_AFRH_AFR13_Msk      (0xFUL << GPIO_CFG_AFRH_AFR13_Pos)           /*!<GPIO_CFG AFRH: AFR13 Mask */

#define GPIO_CFG_AFRH_AFR14_Pos      24                                           /*!<GPIO_CFG AFRH: AFR14 Position */
#define GPIO_CFG_AFRH_AFR14_Msk      (0xFUL << GPIO_CFG_AFRH_AFR14_Pos)           /*!<GPIO_CFG AFRH: AFR14 Mask */

#define GPIO_CFG_AFRH_AFR15_Pos      28                                           /*!<GPIO_CFG AFRH: AFR15 Position */
#define GPIO_CFG_AFRH_AFR15_Msk      (0xFUL << GPIO_CFG_AFRH_AFR15_Pos)           /*!<GPIO_CFG AFRH: AFR15 Mask */

#define GPIO_CFG_AFRH_Msk_ALL        (GPIO_CFG_AFRH_AFR8_Msk       |\
                                      GPIO_CFG_AFRH_AFR9_Msk       |\
                                      GPIO_CFG_AFRH_AFR10_Msk      |\
                                      GPIO_CFG_AFRH_AFR11_Msk      |\
                                      GPIO_CFG_AFRH_AFR12_Msk      |\
                                      GPIO_CFG_AFRH_AFR13_Msk      |\
                                      GPIO_CFG_AFRH_AFR14_Msk      |\
                                      GPIO_CFG_AFRH_AFR15_Msk      )

#define GPIO_CFG_TTLEN_PA2_Pos       2                                            /*!<GPIO_CFG TTLEN: PA2 Position */
#define GPIO_CFG_TTLEN_PA2_Msk       (0x1UL << GPIO_CFG_TTLEN_PA2_Pos)            /*!<GPIO_CFG TTLEN: PA2 Mask */

#define GPIO_CFG_TTLEN_PA3_Pos       3                                            /*!<GPIO_CFG TTLEN: PA3 Position */
#define GPIO_CFG_TTLEN_PA3_Msk       (0x1UL << GPIO_CFG_TTLEN_PA3_Pos)            /*!<GPIO_CFG TTLEN: PA3 Mask */

#define GPIO_CFG_TTLEN_PA8_Pos       8                                            /*!<GPIO_CFG TTLEN: PA8 Position */
#define GPIO_CFG_TTLEN_PA8_Msk       (0x1UL << GPIO_CFG_TTLEN_PA8_Pos)            /*!<GPIO_CFG TTLEN: PA8 Mask */

#define GPIO_CFG_TTLEN_PA11_Pos      11                                           /*!<GPIO_CFG TTLEN: PA11 Position */
#define GPIO_CFG_TTLEN_PA11_Msk      (0x1UL << GPIO_CFG_TTLEN_PA11_Pos)           /*!<GPIO_CFG TTLEN: PA11 Mask */

#define GPIO_CFG_TTLEN_Msk_ALL       (GPIO_CFG_TTLEN_PA2_Msk       |\
                                      GPIO_CFG_TTLEN_PA3_Msk       |\
                                      GPIO_CFG_TTLEN_PA8_Msk       |\
                                      GPIO_CFG_TTLEN_PA11_Msk      )

#define GPIO_CFG_LCKR_LCK_Pos        0                                            /*!<GPIO_CFG LCKR: LCK Position */
#define GPIO_CFG_LCKR_LCK_Msk        (0xFFFFUL /*<< GPIO_CFG_LCKR_LCK_Pos*/)      /*!<GPIO_CFG LCKR: LCK Mask */

#define GPIO_CFG_LCKR_LOCK_Pos       16                                           /*!<GPIO_CFG LCKR: LOCK Position */
#define GPIO_CFG_LCKR_LOCK_Msk       (0xFFFFUL << GPIO_CFG_LCKR_LOCK_Pos)         /*!<GPIO_CFG LCKR: LOCK Mask */

#define GPIO_CFG_LCKR_Msk_ALL        (GPIO_CFG_LCKR_LCK_Msk        |\
                                      GPIO_CFG_LCKR_LOCK_Msk       )

#define GPIO_MODER_MODER_Pos         0                                            /*!<GPIO MODER: MODER Position */
#define GPIO_MODER_MODER_Msk         (0xFFFFUL /*<< GPIO_MODER_MODER_Pos*/)       /*!<GPIO MODER: MODER Mask */
#define GPIO_MODER_MODER0_Msk        (0x1UL /*<< GPIO_MODER_MODER_Pos*/)          /*!<GPIO MODER: MODER0 Mask */

#define GPIO_MODER_Msk_ALL           (GPIO_MODER_MODER_Msk        )

#define GPIO_IDR_IDR_Pos             0                                            /*!<GPIO IDR: IDR Position */
#define GPIO_IDR_IDR_Msk             (0xFFFFUL /*<< GPIO_IDR_IDR_Pos*/)           /*!<GPIO IDR: IDR Mask */

#define GPIO_IDR_Msk_ALL             (GPIO_IDR_IDR_Msk            )

#define GPIO_ODR_ODR_Pos             0                                            /*!<GPIO ODR: ODR Position */
#define GPIO_ODR_ODR_Msk             (0xFFFFUL /*<< GPIO_ODR_ODR_Pos*/)           /*!<GPIO ODR: ODR Mask */

#define GPIO_ODR_Msk_ALL             (GPIO_ODR_ODR_Msk            )

#define GPIO_BSRR_BS_Pos             0                                            /*!<GPIO BSRR: BS Position */
#define GPIO_BSRR_BS_Msk             (0xFFFFUL /*<< GPIO_BSRR_BS_Pos*/)           /*!<GPIO BSRR: BS Mask */

#define GPIO_BSRR_BR_Pos             16                                           /*!<GPIO BSRR: BR Position */
#define GPIO_BSRR_BR_Msk             (0xFFFFUL << GPIO_BSRR_BR_Pos)               /*!<GPIO BSRR: BR Mask */

#define GPIO_BSRR_Msk_ALL            (GPIO_BSRR_BS_Msk             |\
                                      GPIO_BSRR_BR_Msk             )

#define EXTI_IMR_IMR_Pos             0                                            /*!<EXTI IMR: IMR Position */
#define EXTI_IMR_IMR_Msk             (0xFFFFUL /*<< EXTI_IMR_IMR_Pos*/)           /*!<EXTI IMR: IMR Mask */

#define EXTI_IMR_Msk_ALL             (EXTI_IMR_IMR_Msk            )

#define EXTI_EMR_EMR_Pos             0                                            /*!<EXTI EMR: EMR Position */
#define EXTI_EMR_EMR_Msk             (0xFFFFUL /*<< EXTI_EMR_EMR_Pos*/)           /*!<EXTI EMR: EMR Mask */

#define EXTI_EMR_Msk_ALL             (EXTI_EMR_EMR_Msk            )

#define EXTI_TMSR_TMR_Pos            0                                            /*!<EXTI TMSR: TMR Position */
#define EXTI_TMSR_TMR_Msk            (0xFFFFUL /*<< EXTI_TMSR_TMR_Pos*/)          /*!<EXTI TMSR: TMR Mask */

#define EXTI_TMSR_Msk_ALL            (EXTI_TMSR_TMR_Msk           )

#define EXTI_RTSR_RTR_Pos            0                                            /*!<EXTI RTSR: RTR Position */
#define EXTI_RTSR_RTR_Msk            (0xFFFFUL /*<< EXTI_RTSR_RTR_Pos*/)          /*!<EXTI RTSR: RTR Mask */

#define EXTI_RTSR_Msk_ALL            (EXTI_RTSR_RTR_Msk           )

#define EXTI_FTSR_FTR_Pos            0                                            /*!<EXTI FTSR: FTR Position */
#define EXTI_FTSR_FTR_Msk            (0xFFFFUL /*<< EXTI_FTSR_FTR_Pos*/)          /*!<EXTI FTSR: FTR Mask */

#define EXTI_FTSR_Msk_ALL            (EXTI_FTSR_FTR_Msk           )

#define EXTI_SWIER_SWIER_Pos         0                                            /*!<EXTI SWIER: SWIER Position */
#define EXTI_SWIER_SWIER_Msk         (0xFFFFUL /*<< EXTI_SWIER_SWIER_Pos*/)       /*!<EXTI SWIER: SWIER Mask */

#define EXTI_SWIER_Msk_ALL           (EXTI_SWIER_SWIER_Msk        )

#define EXTI_PR_PR_Pos               0                                            /*!<EXTI PR: PR Position */
#define EXTI_PR_PR_Msk               (0xFFFFUL /*<< EXTI_PR_PR_Pos*/)             /*!<EXTI PR: PR Mask */

#define EXTI_PR_PRC_Pos              16                                           /*!<EXTI PR: PRC Position */
#define EXTI_PR_PRC_Msk              (0xFFFFUL << EXTI_PR_PRC_Pos)                /*!<EXTI PR: PRC Mask */

#define EXTI_PR_Msk_ALL              (EXTI_PR_PR_Msk               |\
                                      EXTI_PR_PRC_Msk              )

#define EXTI_CFGL_EXTI0_Pos          0                                            /*!<EXTI CFGL: EXTI0 Position */
#define EXTI_CFGL_EXTI0_Msk          (0x7UL /*<< EXTI_CFGL_EXTI0_Pos*/)           /*!<EXTI CFGL: EXTI0 Mask */

#define EXTI_CFGL_EXTI1_Pos          4                                            /*!<EXTI CFGL: EXTI1 Position */
#define EXTI_CFGL_EXTI1_Msk          (0x7UL << EXTI_CFGL_EXTI1_Pos)               /*!<EXTI CFGL: EXTI1 Mask */

#define EXTI_CFGL_EXTI2_Pos          8                                            /*!<EXTI CFGL: EXTI2 Position */
#define EXTI_CFGL_EXTI2_Msk          (0x7UL << EXTI_CFGL_EXTI2_Pos)               /*!<EXTI CFGL: EXTI2 Mask */

#define EXTI_CFGL_EXTI3_Pos          12                                           /*!<EXTI CFGL: EXTI3 Position */
#define EXTI_CFGL_EXTI3_Msk          (0x7UL << EXTI_CFGL_EXTI3_Pos)               /*!<EXTI CFGL: EXTI3 Mask */

#define EXTI_CFGL_EXTI4_Pos          16                                           /*!<EXTI CFGL: EXTI4 Position */
#define EXTI_CFGL_EXTI4_Msk          (0x7UL << EXTI_CFGL_EXTI4_Pos)               /*!<EXTI CFGL: EXTI4 Mask */

#define EXTI_CFGL_EXTI5_Pos          20                                           /*!<EXTI CFGL: EXTI5 Position */
#define EXTI_CFGL_EXTI5_Msk          (0x7UL << EXTI_CFGL_EXTI5_Pos)               /*!<EXTI CFGL: EXTI5 Mask */

#define EXTI_CFGL_EXTI6_Pos          24                                           /*!<EXTI CFGL: EXTI6 Position */
#define EXTI_CFGL_EXTI6_Msk          (0x7UL << EXTI_CFGL_EXTI6_Pos)               /*!<EXTI CFGL: EXTI6 Mask */

#define EXTI_CFGL_EXTI7_Pos          28                                           /*!<EXTI CFGL: EXTI7 Position */
#define EXTI_CFGL_EXTI7_Msk          (0x7UL << EXTI_CFGL_EXTI7_Pos)               /*!<EXTI CFGL: EXTI7 Mask */

#define EXTI_CFGL_Msk_ALL            (EXTI_CFGL_EXTI0_Msk          |\
                                      EXTI_CFGL_EXTI1_Msk          |\
                                      EXTI_CFGL_EXTI2_Msk          |\
                                      EXTI_CFGL_EXTI3_Msk          |\
                                      EXTI_CFGL_EXTI4_Msk          |\
                                      EXTI_CFGL_EXTI5_Msk          |\
                                      EXTI_CFGL_EXTI6_Msk          |\
                                      EXTI_CFGL_EXTI7_Msk          )

#define EXTI_CFGH_EXTI8_Pos          0                                            /*!<EXTI CFGH: EXTI8 Position */
#define EXTI_CFGH_EXTI8_Msk          (0x7UL /*<< EXTI_CFGH_EXTI8_Pos*/)           /*!<EXTI CFGH: EXTI8 Mask */

#define EXTI_CFGH_EXTI9_Pos          4                                            /*!<EXTI CFGH: EXTI9 Position */
#define EXTI_CFGH_EXTI9_Msk          (0x7UL << EXTI_CFGH_EXTI9_Pos)               /*!<EXTI CFGH: EXTI9 Mask */

#define EXTI_CFGH_EXTI10_Pos         8                                            /*!<EXTI CFGH: EXTI10 Position */
#define EXTI_CFGH_EXTI10_Msk         (0x7UL << EXTI_CFGH_EXTI10_Pos)              /*!<EXTI CFGH: EXTI10 Mask */

#define EXTI_CFGH_EXTI11_Pos         12                                           /*!<EXTI CFGH: EXTI11 Position */
#define EXTI_CFGH_EXTI11_Msk         (0x7UL << EXTI_CFGH_EXTI11_Pos)              /*!<EXTI CFGH: EXTI11 Mask */

#define EXTI_CFGH_EXTI12_Pos         16                                           /*!<EXTI CFGH: EXTI12 Position */
#define EXTI_CFGH_EXTI12_Msk         (0x7UL << EXTI_CFGH_EXTI12_Pos)              /*!<EXTI CFGH: EXTI12 Mask */

#define EXTI_CFGH_EXTI13_Pos         20                                           /*!<EXTI CFGH: EXTI13 Position */
#define EXTI_CFGH_EXTI13_Msk         (0x7UL << EXTI_CFGH_EXTI13_Pos)              /*!<EXTI CFGH: EXTI13 Mask */

#define EXTI_CFGH_EXTI14_Pos         24                                           /*!<EXTI CFGH: EXTI14 Position */
#define EXTI_CFGH_EXTI14_Msk         (0x7UL << EXTI_CFGH_EXTI14_Pos)              /*!<EXTI CFGH: EXTI14 Mask */

#define EXTI_CFGH_EXTI15_Pos         28                                           /*!<EXTI CFGH: EXTI15 Position */
#define EXTI_CFGH_EXTI15_Msk         (0x7UL << EXTI_CFGH_EXTI15_Pos)              /*!<EXTI CFGH: EXTI15 Mask */

#define EXTI_CFGH_Msk_ALL            (EXTI_CFGH_EXTI8_Msk          |\
                                      EXTI_CFGH_EXTI9_Msk          |\
                                      EXTI_CFGH_EXTI10_Msk         |\
                                      EXTI_CFGH_EXTI11_Msk         |\
                                      EXTI_CFGH_EXTI12_Msk         |\
                                      EXTI_CFGH_EXTI13_Msk         |\
                                      EXTI_CFGH_EXTI14_Msk         |\
                                      EXTI_CFGH_EXTI15_Msk         )

#define EXTI_SAMPL_SN0_Pos           0                                            /*!<EXTI SAMPL: SN0 Position */
#define EXTI_SAMPL_SN0_Msk           (0x3UL /*<< EXTI_SAMPL_SN0_Pos*/)            /*!<EXTI SAMPL: SN0 Mask */

#define EXTI_SAMPL_PS0_Pos           2                                            /*!<EXTI SAMPL: PS0 Position */
#define EXTI_SAMPL_PS0_Msk           (0x3UL << EXTI_SAMPL_PS0_Pos)                /*!<EXTI SAMPL: PS0 Mask */

#define EXTI_SAMPL_SN1_Pos           4                                            /*!<EXTI SAMPL: SN1 Position */
#define EXTI_SAMPL_SN1_Msk           (0x3UL << EXTI_SAMPL_SN1_Pos)                /*!<EXTI SAMPL: SN1 Mask */

#define EXTI_SAMPL_PS1_Pos           6                                            /*!<EXTI SAMPL: PS1 Position */
#define EXTI_SAMPL_PS1_Msk           (0x3UL << EXTI_SAMPL_PS1_Pos)                /*!<EXTI SAMPL: PS1 Mask */

#define EXTI_SAMPL_SN2_Pos           8                                            /*!<EXTI SAMPL: SN2 Position */
#define EXTI_SAMPL_SN2_Msk           (0x3UL << EXTI_SAMPL_SN2_Pos)                /*!<EXTI SAMPL: SN2 Mask */

#define EXTI_SAMPL_PS2_Pos           10                                           /*!<EXTI SAMPL: PS2 Position */
#define EXTI_SAMPL_PS2_Msk           (0x3UL << EXTI_SAMPL_PS2_Pos)                /*!<EXTI SAMPL: PS2 Mask */

#define EXTI_SAMPL_SN3_Pos           12                                           /*!<EXTI SAMPL: SN3 Position */
#define EXTI_SAMPL_SN3_Msk           (0x3UL << EXTI_SAMPL_SN3_Pos)                /*!<EXTI SAMPL: SN3 Mask */

#define EXTI_SAMPL_PS3_Pos           14                                           /*!<EXTI SAMPL: PS3 Position */
#define EXTI_SAMPL_PS3_Msk           (0x3UL << EXTI_SAMPL_PS3_Pos)                /*!<EXTI SAMPL: PS3 Mask */

#define EXTI_SAMPL_SN4_Pos           16                                           /*!<EXTI SAMPL: SN4 Position */
#define EXTI_SAMPL_SN4_Msk           (0x3UL << EXTI_SAMPL_SN4_Pos)                /*!<EXTI SAMPL: SN4 Mask */

#define EXTI_SAMPL_PS4_Pos           18                                           /*!<EXTI SAMPL: PS4 Position */
#define EXTI_SAMPL_PS4_Msk           (0x3UL << EXTI_SAMPL_PS4_Pos)                /*!<EXTI SAMPL: PS4 Mask */

#define EXTI_SAMPL_SN5_Pos           20                                           /*!<EXTI SAMPL: SN5 Position */
#define EXTI_SAMPL_SN5_Msk           (0x3UL << EXTI_SAMPL_SN5_Pos)                /*!<EXTI SAMPL: SN5 Mask */

#define EXTI_SAMPL_PS5_Pos           22                                           /*!<EXTI SAMPL: PS5 Position */
#define EXTI_SAMPL_PS5_Msk           (0x3UL << EXTI_SAMPL_PS5_Pos)                /*!<EXTI SAMPL: PS5 Mask */

#define EXTI_SAMPL_SN6_Pos           24                                           /*!<EXTI SAMPL: SN6 Position */
#define EXTI_SAMPL_SN6_Msk           (0x3UL << EXTI_SAMPL_SN6_Pos)                /*!<EXTI SAMPL: SN6 Mask */

#define EXTI_SAMPL_PS6_Pos           26                                           /*!<EXTI SAMPL: PS6 Position */
#define EXTI_SAMPL_PS6_Msk           (0x3UL << EXTI_SAMPL_PS6_Pos)                /*!<EXTI SAMPL: PS6 Mask */

#define EXTI_SAMPL_SN7_Pos           28                                           /*!<EXTI SAMPL: SN7 Position */
#define EXTI_SAMPL_SN7_Msk           (0x3UL << EXTI_SAMPL_SN7_Pos)                /*!<EXTI SAMPL: SN7 Mask */

#define EXTI_SAMPL_PS7_Pos           30                                           /*!<EXTI SAMPL: PS7 Position */
#define EXTI_SAMPL_PS7_Msk           (0x3UL << EXTI_SAMPL_PS7_Pos)                /*!<EXTI SAMPL: PS7 Mask */

#define EXTI_SAMPL_Msk_ALL           (EXTI_SAMPL_SN0_Msk           |\
                                      EXTI_SAMPL_PS0_Msk           |\
                                      EXTI_SAMPL_SN1_Msk           |\
                                      EXTI_SAMPL_PS1_Msk           |\
                                      EXTI_SAMPL_SN2_Msk           |\
                                      EXTI_SAMPL_PS2_Msk           |\
                                      EXTI_SAMPL_SN3_Msk           |\
                                      EXTI_SAMPL_PS3_Msk           |\
                                      EXTI_SAMPL_SN4_Msk           |\
                                      EXTI_SAMPL_PS4_Msk           |\
                                      EXTI_SAMPL_SN5_Msk           |\
                                      EXTI_SAMPL_PS5_Msk           |\
                                      EXTI_SAMPL_SN6_Msk           |\
                                      EXTI_SAMPL_PS6_Msk           |\
                                      EXTI_SAMPL_SN7_Msk           |\
                                      EXTI_SAMPL_PS7_Msk           )

#define EXTI_SAMPH_SN8_Pos           0                                            /*!<EXTI SAMPH: SN8 Position */
#define EXTI_SAMPH_SN8_Msk           (0x3UL /*<< EXTI_SAMPH_SN8_Pos*/)            /*!<EXTI SAMPH: SN8 Mask */

#define EXTI_SAMPH_PS8_Pos           2                                            /*!<EXTI SAMPH: PS8 Position */
#define EXTI_SAMPH_PS8_Msk           (0x3UL << EXTI_SAMPH_PS8_Pos)                /*!<EXTI SAMPH: PS8 Mask */

#define EXTI_SAMPH_SN9_Pos           4                                            /*!<EXTI SAMPH: SN9 Position */
#define EXTI_SAMPH_SN9_Msk           (0x3UL << EXTI_SAMPH_SN9_Pos)                /*!<EXTI SAMPH: SN9 Mask */

#define EXTI_SAMPH_PS9_Pos           6                                            /*!<EXTI SAMPH: PS9 Position */
#define EXTI_SAMPH_PS9_Msk           (0x3UL << EXTI_SAMPH_PS9_Pos)                /*!<EXTI SAMPH: PS9 Mask */

#define EXTI_SAMPH_SN10_Pos          8                                            /*!<EXTI SAMPH: SN10 Position */
#define EXTI_SAMPH_SN10_Msk          (0x3UL << EXTI_SAMPH_SN10_Pos)               /*!<EXTI SAMPH: SN10 Mask */

#define EXTI_SAMPH_PS10_Pos          10                                           /*!<EXTI SAMPH: PS10 Position */
#define EXTI_SAMPH_PS10_Msk          (0x3UL << EXTI_SAMPH_PS10_Pos)               /*!<EXTI SAMPH: PS10 Mask */

#define EXTI_SAMPH_SN11_Pos          12                                           /*!<EXTI SAMPH: SN11 Position */
#define EXTI_SAMPH_SN11_Msk          (0x3UL << EXTI_SAMPH_SN11_Pos)               /*!<EXTI SAMPH: SN11 Mask */

#define EXTI_SAMPH_PS11_Pos          14                                           /*!<EXTI SAMPH: PS11 Position */
#define EXTI_SAMPH_PS11_Msk          (0x3UL << EXTI_SAMPH_PS11_Pos)               /*!<EXTI SAMPH: PS11 Mask */

#define EXTI_SAMPH_SN12_Pos          16                                           /*!<EXTI SAMPH: SN12 Position */
#define EXTI_SAMPH_SN12_Msk          (0x3UL << EXTI_SAMPH_SN12_Pos)               /*!<EXTI SAMPH: SN12 Mask */

#define EXTI_SAMPH_PS12_Pos          18                                           /*!<EXTI SAMPH: PS12 Position */
#define EXTI_SAMPH_PS12_Msk          (0x3UL << EXTI_SAMPH_PS12_Pos)               /*!<EXTI SAMPH: PS12 Mask */

#define EXTI_SAMPH_SN13_Pos          20                                           /*!<EXTI SAMPH: SN13 Position */
#define EXTI_SAMPH_SN13_Msk          (0x3UL << EXTI_SAMPH_SN13_Pos)               /*!<EXTI SAMPH: SN13 Mask */

#define EXTI_SAMPH_PS13_Pos          22                                           /*!<EXTI SAMPH: PS13 Position */
#define EXTI_SAMPH_PS13_Msk          (0x3UL << EXTI_SAMPH_PS13_Pos)               /*!<EXTI SAMPH: PS13 Mask */

#define EXTI_SAMPH_SN14_Pos          24                                           /*!<EXTI SAMPH: SN14 Position */
#define EXTI_SAMPH_SN14_Msk          (0x3UL << EXTI_SAMPH_SN14_Pos)               /*!<EXTI SAMPH: SN14 Mask */

#define EXTI_SAMPH_PS14_Pos          26                                           /*!<EXTI SAMPH: PS14 Position */
#define EXTI_SAMPH_PS14_Msk          (0x3UL << EXTI_SAMPH_PS14_Pos)               /*!<EXTI SAMPH: PS14 Mask */

#define EXTI_SAMPH_SN15_Pos          28                                           /*!<EXTI SAMPH: SN15 Position */
#define EXTI_SAMPH_SN15_Msk          (0x3UL << EXTI_SAMPH_SN15_Pos)               /*!<EXTI SAMPH: SN15 Mask */

#define EXTI_SAMPH_PS15_Pos          30                                           /*!<EXTI SAMPH: PS15 Position */
#define EXTI_SAMPH_PS15_Msk          (0x3UL << EXTI_SAMPH_PS15_Pos)               /*!<EXTI SAMPH: PS15 Mask */

#define EXTI_SAMPH_Msk_ALL           (EXTI_SAMPH_SN8_Msk           |\
                                      EXTI_SAMPH_PS8_Msk           |\
                                      EXTI_SAMPH_SN9_Msk           |\
                                      EXTI_SAMPH_PS9_Msk           |\
                                      EXTI_SAMPH_SN10_Msk          |\
                                      EXTI_SAMPH_PS10_Msk          |\
                                      EXTI_SAMPH_SN11_Msk          |\
                                      EXTI_SAMPH_PS11_Msk          |\
                                      EXTI_SAMPH_SN12_Msk          |\
                                      EXTI_SAMPH_PS12_Msk          |\
                                      EXTI_SAMPH_SN13_Msk          |\
                                      EXTI_SAMPH_PS13_Msk          |\
                                      EXTI_SAMPH_SN14_Msk          |\
                                      EXTI_SAMPH_PS14_Msk          |\
                                      EXTI_SAMPH_SN15_Msk          |\
                                      EXTI_SAMPH_PS15_Msk          )

#define EXTI_DMR_DMR_Pos             0                                            /*!<EXTI DMR: DMR Position */
#define EXTI_DMR_DMR_Msk             (0xFFFFUL /*<< EXTI_DMR_DMR_Pos*/)           /*!<EXTI DMR: DMR Mask */

#define EXTI_DMR_Msk_ALL             (EXTI_DMR_DMR_Msk            )

#define DMA_IFSR_BEIF_Pos            0                                            /*!<DMA IFSR: BEIF Position */
#define DMA_IFSR_BEIF_Msk            (0xFFUL /*<< DMA_IFSR_BEIF_Pos*/)            /*!<DMA IFSR: BEIF Mask */

#define DMA_IFSR_TCIF_Pos            8                                            /*!<DMA IFSR: TCIF Position */
#define DMA_IFSR_TCIF_Msk            (0xFFUL << DMA_IFSR_TCIF_Pos)                /*!<DMA IFSR: TCIF Mask */

#define DMA_IFSR_HTIF_Pos            16                                           /*!<DMA IFSR: HTIF Position */
#define DMA_IFSR_HTIF_Msk            (0xFFUL << DMA_IFSR_HTIF_Pos)                /*!<DMA IFSR: HTIF Mask */

#define DMA_IFSR_TEIF_Pos            24                                           /*!<DMA IFSR: TEIF Position */
#define DMA_IFSR_TEIF_Msk            (0xFFUL << DMA_IFSR_TEIF_Pos)                /*!<DMA IFSR: TEIF Mask */

#define DMA_IFSR_Msk_ALL             (DMA_IFSR_BEIF_Msk            |\
                                      DMA_IFSR_TCIF_Msk            |\
                                      DMA_IFSR_HTIF_Msk            |\
                                      DMA_IFSR_TEIF_Msk            )

#define DMA_IFCR_CBEIF_Pos           0                                            /*!<DMA IFCR: CBEIF Position */
#define DMA_IFCR_CBEIF_Msk           (0xFFUL /*<< DMA_IFCR_CBEIF_Pos*/)           /*!<DMA IFCR: CBEIF Mask */

#define DMA_IFCR_CTCIF_Pos           8                                            /*!<DMA IFCR: CTCIF Position */
#define DMA_IFCR_CTCIF_Msk           (0xFFUL << DMA_IFCR_CTCIF_Pos)               /*!<DMA IFCR: CTCIF Mask */

#define DMA_IFCR_CHTIF_Pos           16                                           /*!<DMA IFCR: CHTIF Position */
#define DMA_IFCR_CHTIF_Msk           (0xFFUL << DMA_IFCR_CHTIF_Pos)               /*!<DMA IFCR: CHTIF Mask */

#define DMA_IFCR_CTEIF_Pos           24                                           /*!<DMA IFCR: CTEIF Position */
#define DMA_IFCR_CTEIF_Msk           (0xFFUL << DMA_IFCR_CTEIF_Pos)               /*!<DMA IFCR: CTEIF Mask */

#define DMA_IFCR_Msk_ALL             (DMA_IFCR_CBEIF_Msk           |\
                                      DMA_IFCR_CTCIF_Msk           |\
                                      DMA_IFCR_CHTIF_Msk           |\
                                      DMA_IFCR_CTEIF_Msk           )

#define DMA_CSR_SWTRG_Pos            0                                            /*!<DMA CSR: SWTRG Position */
#define DMA_CSR_SWTRG_Msk            (0xFFUL /*<< DMA_CSR_SWTRG_Pos*/)            /*!<DMA CSR: SWTRG Mask */

#define DMA_CSR_DBUSY_Pos            8                                            /*!<DMA CSR: DBUSY Position */
#define DMA_CSR_DBUSY_Msk            (0xFFUL << DMA_CSR_DBUSY_Pos)                /*!<DMA CSR: DBUSY Mask */

#define DMA_CSR_BURSTIDLE_Pos        16                                           /*!<DMA CSR: BURSTIDLE Position */
#define DMA_CSR_BURSTIDLE_Msk        (0xFUL << DMA_CSR_BURSTIDLE_Pos)             /*!<DMA CSR: BURSTIDLE Mask */

#define DMA_CSR_RELOAD_Pos           24                                           /*!<DMA CSR: RELOAD Position */
#define DMA_CSR_RELOAD_Msk           (0xFFUL << DMA_CSR_RELOAD_Pos)               /*!<DMA CSR: RELOAD Mask */

#define DMA_CSR_Msk_ALL              (DMA_CSR_SWTRG_Msk            |\
                                      DMA_CSR_DBUSY_Msk            |\
                                      DMA_CSR_BURSTIDLE_Msk        |\
                                      DMA_CSR_RELOAD_Msk           )

#define DMA_CCR0_EN_Pos              0                                            /*!<DMA CCR0: EN Position */
#define DMA_CCR0_EN_Msk              (0x1UL /*<< DMA_CCR0_EN_Pos*/)               /*!<DMA CCR0: EN Mask */

#define DMA_CCR0_TCIE_Pos            1                                            /*!<DMA CCR0: TCIE Position */
#define DMA_CCR0_TCIE_Msk            (0x1UL << DMA_CCR0_TCIE_Pos)                 /*!<DMA CCR0: TCIE Mask */

#define DMA_CCR0_HTIE_Pos            2                                            /*!<DMA CCR0: HTIE Position */
#define DMA_CCR0_HTIE_Msk            (0x1UL << DMA_CCR0_HTIE_Pos)                 /*!<DMA CCR0: HTIE Mask */

#define DMA_CCR0_BEIE_Pos            3                                            /*!<DMA CCR0: BEIE Position */
#define DMA_CCR0_BEIE_Msk            (0x1UL << DMA_CCR0_BEIE_Pos)                 /*!<DMA CCR0: BEIE Mask */

#define DMA_CCR0_TEIE_Pos            4                                            /*!<DMA CCR0: TEIE Position */
#define DMA_CCR0_TEIE_Msk            (0x1UL << DMA_CCR0_TEIE_Pos)                 /*!<DMA CCR0: TEIE Mask */

#define DMA_CCR0_DPTYP_Pos           6                                            /*!<DMA CCR0: DPTYP Position */
#define DMA_CCR0_DPTYP_Msk           (0x3UL << DMA_CCR0_DPTYP_Pos)                /*!<DMA CCR0: DPTYP Mask */

#define DMA_CCR0_SPTYP_Pos           8                                            /*!<DMA CCR0: SPTYP Position */
#define DMA_CCR0_SPTYP_Msk           (0x3UL << DMA_CCR0_SPTYP_Pos)                /*!<DMA CCR0: SPTYP Mask */

#define DMA_CCR0_DSIZE_Pos           10                                           /*!<DMA CCR0: DSIZE Position */
#define DMA_CCR0_DSIZE_Msk           (0x3UL << DMA_CCR0_DSIZE_Pos)                /*!<DMA CCR0: DSIZE Mask */

#define DMA_CCR0_SSIZE_Pos           12                                           /*!<DMA CCR0: SSIZE Position */
#define DMA_CCR0_SSIZE_Msk           (0x3UL << DMA_CCR0_SSIZE_Pos)                /*!<DMA CCR0: SSIZE Mask */

#define DMA_CCR0_PL_Pos              14                                           /*!<DMA CCR0: PL Position */
#define DMA_CCR0_PL_Msk              (0x3UL << DMA_CCR0_PL_Pos)                   /*!<DMA CCR0: PL Mask */

#define DMA_CCR0_BURSTLEN_Pos        16                                           /*!<DMA CCR0: BURSTLEN Position */
#define DMA_CCR0_BURSTLEN_Msk        (0xFUL << DMA_CCR0_BURSTLEN_Pos)             /*!<DMA CCR0: BURSTLEN Mask */

#define DMA_CCR0_STRMSEL_Pos         20                                           /*!<DMA CCR0: STRMSEL Position */
#define DMA_CCR0_STRMSEL_Msk         (0x7UL << DMA_CCR0_STRMSEL_Pos)              /*!<DMA CCR0: STRMSEL Mask */

#define DMA_CCR0_TRGMODE_Pos         23                                           /*!<DMA CCR0: TRGMODE Position */
#define DMA_CCR0_TRGMODE_Msk         (0x1UL << DMA_CCR0_TRGMODE_Pos)              /*!<DMA CCR0: TRGMODE Mask */

#define DMA_CCR0_Msk_ALL             (DMA_CCR0_EN_Msk              |\
                                      DMA_CCR0_TCIE_Msk            |\
                                      DMA_CCR0_HTIE_Msk            |\
                                      DMA_CCR0_BEIE_Msk            |\
                                      DMA_CCR0_TEIE_Msk            |\
                                      DMA_CCR0_DPTYP_Msk           |\
                                      DMA_CCR0_SPTYP_Msk           |\
                                      DMA_CCR0_DSIZE_Msk           |\
                                      DMA_CCR0_SSIZE_Msk           |\
                                      DMA_CCR0_PL_Msk              |\
                                      DMA_CCR0_BURSTLEN_Msk        |\
                                      DMA_CCR0_STRMSEL_Msk         |\
                                      DMA_CCR0_TRGMODE_Msk         )

#define DMA_NPKT0_NPKT_Pos           0                                            /*!<DMA NPKT0: NPKT Position */
#define DMA_NPKT0_NPKT_Msk           (0x7FFUL /*<< DMA_NPKT0_NPKT_Pos*/)          /*!<DMA NPKT0: NPKT Mask */

#define DMA_NPKT0_Msk_ALL            (DMA_NPKT0_NPKT_Msk          )

#define DMA_CPKT0_CPKT_Pos           0                                            /*!<DMA CPKT0: CPKT Position */
#define DMA_CPKT0_CPKT_Msk           (0x7FFUL /*<< DMA_CPKT0_CPKT_Pos*/)          /*!<DMA CPKT0: CPKT Mask */

#define DMA_CPKT0_Msk_ALL            (DMA_CPKT0_CPKT_Msk          )

#define DMA_CCR1_EN_Pos              0                                            /*!<DMA CCR1: EN Position */
#define DMA_CCR1_EN_Msk              (0x1UL /*<< DMA_CCR1_EN_Pos*/)               /*!<DMA CCR1: EN Mask */

#define DMA_CCR1_TCIE_Pos            1                                            /*!<DMA CCR1: TCIE Position */
#define DMA_CCR1_TCIE_Msk            (0x1UL << DMA_CCR1_TCIE_Pos)                 /*!<DMA CCR1: TCIE Mask */

#define DMA_CCR1_HTIE_Pos            2                                            /*!<DMA CCR1: HTIE Position */
#define DMA_CCR1_HTIE_Msk            (0x1UL << DMA_CCR1_HTIE_Pos)                 /*!<DMA CCR1: HTIE Mask */

#define DMA_CCR1_BEIE_Pos            3                                            /*!<DMA CCR1: BEIE Position */
#define DMA_CCR1_BEIE_Msk            (0x1UL << DMA_CCR1_BEIE_Pos)                 /*!<DMA CCR1: BEIE Mask */

#define DMA_CCR1_TEIE_Pos            4                                            /*!<DMA CCR1: TEIE Position */
#define DMA_CCR1_TEIE_Msk            (0x1UL << DMA_CCR1_TEIE_Pos)                 /*!<DMA CCR1: TEIE Mask */

#define DMA_CCR1_DPTYP_Pos           6                                            /*!<DMA CCR1: DPTYP Position */
#define DMA_CCR1_DPTYP_Msk           (0x3UL << DMA_CCR1_DPTYP_Pos)                /*!<DMA CCR1: DPTYP Mask */

#define DMA_CCR1_SPTYP_Pos           8                                            /*!<DMA CCR1: SPTYP Position */
#define DMA_CCR1_SPTYP_Msk           (0x3UL << DMA_CCR1_SPTYP_Pos)                /*!<DMA CCR1: SPTYP Mask */

#define DMA_CCR1_DSIZE_Pos           10                                           /*!<DMA CCR1: DSIZE Position */
#define DMA_CCR1_DSIZE_Msk           (0x3UL << DMA_CCR1_DSIZE_Pos)                /*!<DMA CCR1: DSIZE Mask */

#define DMA_CCR1_SSIZE_Pos           12                                           /*!<DMA CCR1: SSIZE Position */
#define DMA_CCR1_SSIZE_Msk           (0x3UL << DMA_CCR1_SSIZE_Pos)                /*!<DMA CCR1: SSIZE Mask */

#define DMA_CCR1_PL_Pos              14                                           /*!<DMA CCR1: PL Position */
#define DMA_CCR1_PL_Msk              (0x3UL << DMA_CCR1_PL_Pos)                   /*!<DMA CCR1: PL Mask */

#define DMA_CCR1_BURSTLEN_Pos        16                                           /*!<DMA CCR1: BURSTLEN Position */
#define DMA_CCR1_BURSTLEN_Msk        (0xFUL << DMA_CCR1_BURSTLEN_Pos)             /*!<DMA CCR1: BURSTLEN Mask */

#define DMA_CCR1_STRMSEL_Pos         20                                           /*!<DMA CCR1: STRMSEL Position */
#define DMA_CCR1_STRMSEL_Msk         (0x7UL << DMA_CCR1_STRMSEL_Pos)              /*!<DMA CCR1: STRMSEL Mask */

#define DMA_CCR1_TRGMODE_Pos         23                                           /*!<DMA CCR1: TRGMODE Position */
#define DMA_CCR1_TRGMODE_Msk         (0x1UL << DMA_CCR1_TRGMODE_Pos)              /*!<DMA CCR1: TRGMODE Mask */

#define DMA_CCR1_Msk_ALL             (DMA_CCR1_EN_Msk              |\
                                      DMA_CCR1_TCIE_Msk            |\
                                      DMA_CCR1_HTIE_Msk            |\
                                      DMA_CCR1_BEIE_Msk            |\
                                      DMA_CCR1_TEIE_Msk            |\
                                      DMA_CCR1_DPTYP_Msk           |\
                                      DMA_CCR1_SPTYP_Msk           |\
                                      DMA_CCR1_DSIZE_Msk           |\
                                      DMA_CCR1_SSIZE_Msk           |\
                                      DMA_CCR1_PL_Msk              |\
                                      DMA_CCR1_BURSTLEN_Msk        |\
                                      DMA_CCR1_STRMSEL_Msk         |\
                                      DMA_CCR1_TRGMODE_Msk         )

#define DMA_NPKT1_NPKT_Pos           0                                            /*!<DMA NPKT1: NPKT Position */
#define DMA_NPKT1_NPKT_Msk           (0x7FFUL /*<< DMA_NPKT1_NPKT_Pos*/)          /*!<DMA NPKT1: NPKT Mask */

#define DMA_NPKT1_Msk_ALL            (DMA_NPKT1_NPKT_Msk          )

#define DMA_CPKT1_CPKT_Pos           0                                            /*!<DMA CPKT1: CPKT Position */
#define DMA_CPKT1_CPKT_Msk           (0x7FFUL /*<< DMA_CPKT1_CPKT_Pos*/)          /*!<DMA CPKT1: CPKT Mask */

#define DMA_CPKT1_Msk_ALL            (DMA_CPKT1_CPKT_Msk          )

#define DMA_CCR2_EN_Pos              0                                            /*!<DMA CCR2: EN Position */
#define DMA_CCR2_EN_Msk              (0x1UL /*<< DMA_CCR2_EN_Pos*/)               /*!<DMA CCR2: EN Mask */

#define DMA_CCR2_TCIE_Pos            1                                            /*!<DMA CCR2: TCIE Position */
#define DMA_CCR2_TCIE_Msk            (0x1UL << DMA_CCR2_TCIE_Pos)                 /*!<DMA CCR2: TCIE Mask */

#define DMA_CCR2_HTIE_Pos            2                                            /*!<DMA CCR2: HTIE Position */
#define DMA_CCR2_HTIE_Msk            (0x1UL << DMA_CCR2_HTIE_Pos)                 /*!<DMA CCR2: HTIE Mask */

#define DMA_CCR2_BEIE_Pos            3                                            /*!<DMA CCR2: BEIE Position */
#define DMA_CCR2_BEIE_Msk            (0x1UL << DMA_CCR2_BEIE_Pos)                 /*!<DMA CCR2: BEIE Mask */

#define DMA_CCR2_TEIE_Pos            4                                            /*!<DMA CCR2: TEIE Position */
#define DMA_CCR2_TEIE_Msk            (0x1UL << DMA_CCR2_TEIE_Pos)                 /*!<DMA CCR2: TEIE Mask */

#define DMA_CCR2_DPTYP_Pos           6                                            /*!<DMA CCR2: DPTYP Position */
#define DMA_CCR2_DPTYP_Msk           (0x3UL << DMA_CCR2_DPTYP_Pos)                /*!<DMA CCR2: DPTYP Mask */

#define DMA_CCR2_SPTYP_Pos           8                                            /*!<DMA CCR2: SPTYP Position */
#define DMA_CCR2_SPTYP_Msk           (0x3UL << DMA_CCR2_SPTYP_Pos)                /*!<DMA CCR2: SPTYP Mask */

#define DMA_CCR2_DSIZE_Pos           10                                           /*!<DMA CCR2: DSIZE Position */
#define DMA_CCR2_DSIZE_Msk           (0x3UL << DMA_CCR2_DSIZE_Pos)                /*!<DMA CCR2: DSIZE Mask */

#define DMA_CCR2_SSIZE_Pos           12                                           /*!<DMA CCR2: SSIZE Position */
#define DMA_CCR2_SSIZE_Msk           (0x3UL << DMA_CCR2_SSIZE_Pos)                /*!<DMA CCR2: SSIZE Mask */

#define DMA_CCR2_PL_Pos              14                                           /*!<DMA CCR2: PL Position */
#define DMA_CCR2_PL_Msk              (0x3UL << DMA_CCR2_PL_Pos)                   /*!<DMA CCR2: PL Mask */

#define DMA_CCR2_BURSTLEN_Pos        16                                           /*!<DMA CCR2: BURSTLEN Position */
#define DMA_CCR2_BURSTLEN_Msk        (0xFUL << DMA_CCR2_BURSTLEN_Pos)             /*!<DMA CCR2: BURSTLEN Mask */

#define DMA_CCR2_STRMSEL_Pos         20                                           /*!<DMA CCR2: STRMSEL Position */
#define DMA_CCR2_STRMSEL_Msk         (0x7UL << DMA_CCR2_STRMSEL_Pos)              /*!<DMA CCR2: STRMSEL Mask */

#define DMA_CCR2_TRGMODE_Pos         23                                           /*!<DMA CCR2: TRGMODE Position */
#define DMA_CCR2_TRGMODE_Msk         (0x1UL << DMA_CCR2_TRGMODE_Pos)              /*!<DMA CCR2: TRGMODE Mask */

#define DMA_CCR2_Msk_ALL             (DMA_CCR2_EN_Msk              |\
                                      DMA_CCR2_TCIE_Msk            |\
                                      DMA_CCR2_HTIE_Msk            |\
                                      DMA_CCR2_BEIE_Msk            |\
                                      DMA_CCR2_TEIE_Msk            |\
                                      DMA_CCR2_DPTYP_Msk           |\
                                      DMA_CCR2_SPTYP_Msk           |\
                                      DMA_CCR2_DSIZE_Msk           |\
                                      DMA_CCR2_SSIZE_Msk           |\
                                      DMA_CCR2_PL_Msk              |\
                                      DMA_CCR2_BURSTLEN_Msk        |\
                                      DMA_CCR2_STRMSEL_Msk         |\
                                      DMA_CCR2_TRGMODE_Msk         )

#define DMA_NPKT2_NPKT_Pos           0                                            /*!<DMA NPKT2: NPKT Position */
#define DMA_NPKT2_NPKT_Msk           (0x7FFUL /*<< DMA_NPKT2_NPKT_Pos*/)          /*!<DMA NPKT2: NPKT Mask */

#define DMA_NPKT2_Msk_ALL            (DMA_NPKT2_NPKT_Msk          )

#define DMA_CPKT2_CPKT_Pos           0                                            /*!<DMA CPKT2: CPKT Position */
#define DMA_CPKT2_CPKT_Msk           (0x7FFUL /*<< DMA_CPKT2_CPKT_Pos*/)          /*!<DMA CPKT2: CPKT Mask */

#define DMA_CPKT2_Msk_ALL            (DMA_CPKT2_CPKT_Msk          )

#define DMA_CCR3_EN_Pos              0                                            /*!<DMA CCR3: EN Position */
#define DMA_CCR3_EN_Msk              (0x1UL /*<< DMA_CCR3_EN_Pos*/)               /*!<DMA CCR3: EN Mask */

#define DMA_CCR3_TCIE_Pos            1                                            /*!<DMA CCR3: TCIE Position */
#define DMA_CCR3_TCIE_Msk            (0x1UL << DMA_CCR3_TCIE_Pos)                 /*!<DMA CCR3: TCIE Mask */

#define DMA_CCR3_HTIE_Pos            2                                            /*!<DMA CCR3: HTIE Position */
#define DMA_CCR3_HTIE_Msk            (0x1UL << DMA_CCR3_HTIE_Pos)                 /*!<DMA CCR3: HTIE Mask */

#define DMA_CCR3_BEIE_Pos            3                                            /*!<DMA CCR3: BEIE Position */
#define DMA_CCR3_BEIE_Msk            (0x1UL << DMA_CCR3_BEIE_Pos)                 /*!<DMA CCR3: BEIE Mask */

#define DMA_CCR3_TEIE_Pos            4                                            /*!<DMA CCR3: TEIE Position */
#define DMA_CCR3_TEIE_Msk            (0x1UL << DMA_CCR3_TEIE_Pos)                 /*!<DMA CCR3: TEIE Mask */

#define DMA_CCR3_DPTYP_Pos           6                                            /*!<DMA CCR3: DPTYP Position */
#define DMA_CCR3_DPTYP_Msk           (0x3UL << DMA_CCR3_DPTYP_Pos)                /*!<DMA CCR3: DPTYP Mask */

#define DMA_CCR3_SPTYP_Pos           8                                            /*!<DMA CCR3: SPTYP Position */
#define DMA_CCR3_SPTYP_Msk           (0x3UL << DMA_CCR3_SPTYP_Pos)                /*!<DMA CCR3: SPTYP Mask */

#define DMA_CCR3_DSIZE_Pos           10                                           /*!<DMA CCR3: DSIZE Position */
#define DMA_CCR3_DSIZE_Msk           (0x3UL << DMA_CCR3_DSIZE_Pos)                /*!<DMA CCR3: DSIZE Mask */

#define DMA_CCR3_SSIZE_Pos           12                                           /*!<DMA CCR3: SSIZE Position */
#define DMA_CCR3_SSIZE_Msk           (0x3UL << DMA_CCR3_SSIZE_Pos)                /*!<DMA CCR3: SSIZE Mask */

#define DMA_CCR3_PL_Pos              14                                           /*!<DMA CCR3: PL Position */
#define DMA_CCR3_PL_Msk              (0x3UL << DMA_CCR3_PL_Pos)                   /*!<DMA CCR3: PL Mask */

#define DMA_CCR3_BURSTLEN_Pos        16                                           /*!<DMA CCR3: BURSTLEN Position */
#define DMA_CCR3_BURSTLEN_Msk        (0xFUL << DMA_CCR3_BURSTLEN_Pos)             /*!<DMA CCR3: BURSTLEN Mask */

#define DMA_CCR3_STRMSEL_Pos         20                                           /*!<DMA CCR3: STRMSEL Position */
#define DMA_CCR3_STRMSEL_Msk         (0x7UL << DMA_CCR3_STRMSEL_Pos)              /*!<DMA CCR3: STRMSEL Mask */

#define DMA_CCR3_TRGMODE_Pos         23                                           /*!<DMA CCR3: TRGMODE Position */
#define DMA_CCR3_TRGMODE_Msk         (0x1UL << DMA_CCR3_TRGMODE_Pos)              /*!<DMA CCR3: TRGMODE Mask */

#define DMA_CCR3_Msk_ALL             (DMA_CCR3_EN_Msk              |\
                                      DMA_CCR3_TCIE_Msk            |\
                                      DMA_CCR3_HTIE_Msk            |\
                                      DMA_CCR3_BEIE_Msk            |\
                                      DMA_CCR3_TEIE_Msk            |\
                                      DMA_CCR3_DPTYP_Msk           |\
                                      DMA_CCR3_SPTYP_Msk           |\
                                      DMA_CCR3_DSIZE_Msk           |\
                                      DMA_CCR3_SSIZE_Msk           |\
                                      DMA_CCR3_PL_Msk              |\
                                      DMA_CCR3_BURSTLEN_Msk        |\
                                      DMA_CCR3_STRMSEL_Msk         |\
                                      DMA_CCR3_TRGMODE_Msk         )

#define DMA_NPKT3_NPKT_Pos           0                                            /*!<DMA NPKT3: NPKT Position */
#define DMA_NPKT3_NPKT_Msk           (0x7FFUL /*<< DMA_NPKT3_NPKT_Pos*/)          /*!<DMA NPKT3: NPKT Mask */

#define DMA_NPKT3_Msk_ALL            (DMA_NPKT3_NPKT_Msk          )

#define DMA_CPKT3_CPKT_Pos           0                                            /*!<DMA CPKT3: CPKT Position */
#define DMA_CPKT3_CPKT_Msk           (0x7FFUL /*<< DMA_CPKT3_CPKT_Pos*/)          /*!<DMA CPKT3: CPKT Mask */

#define DMA_CPKT3_Msk_ALL            (DMA_CPKT3_CPKT_Msk          )

#define DMA_CCR4_EN_Pos              0                                            /*!<DMA CCR4: EN Position */
#define DMA_CCR4_EN_Msk              (0x1UL /*<< DMA_CCR4_EN_Pos*/)               /*!<DMA CCR4: EN Mask */

#define DMA_CCR4_TCIE_Pos            1                                            /*!<DMA CCR4: TCIE Position */
#define DMA_CCR4_TCIE_Msk            (0x1UL << DMA_CCR4_TCIE_Pos)                 /*!<DMA CCR4: TCIE Mask */

#define DMA_CCR4_HTIE_Pos            2                                            /*!<DMA CCR4: HTIE Position */
#define DMA_CCR4_HTIE_Msk            (0x1UL << DMA_CCR4_HTIE_Pos)                 /*!<DMA CCR4: HTIE Mask */

#define DMA_CCR4_BEIE_Pos            3                                            /*!<DMA CCR4: BEIE Position */
#define DMA_CCR4_BEIE_Msk            (0x1UL << DMA_CCR4_BEIE_Pos)                 /*!<DMA CCR4: BEIE Mask */

#define DMA_CCR4_TEIE_Pos            4                                            /*!<DMA CCR4: TEIE Position */
#define DMA_CCR4_TEIE_Msk            (0x1UL << DMA_CCR4_TEIE_Pos)                 /*!<DMA CCR4: TEIE Mask */

#define DMA_CCR4_DPTYP_Pos           6                                            /*!<DMA CCR4: DPTYP Position */
#define DMA_CCR4_DPTYP_Msk           (0x3UL << DMA_CCR4_DPTYP_Pos)                /*!<DMA CCR4: DPTYP Mask */

#define DMA_CCR4_SPTYP_Pos           8                                            /*!<DMA CCR4: SPTYP Position */
#define DMA_CCR4_SPTYP_Msk           (0x3UL << DMA_CCR4_SPTYP_Pos)                /*!<DMA CCR4: SPTYP Mask */

#define DMA_CCR4_DSIZE_Pos           10                                           /*!<DMA CCR4: DSIZE Position */
#define DMA_CCR4_DSIZE_Msk           (0x3UL << DMA_CCR4_DSIZE_Pos)                /*!<DMA CCR4: DSIZE Mask */

#define DMA_CCR4_SSIZE_Pos           12                                           /*!<DMA CCR4: SSIZE Position */
#define DMA_CCR4_SSIZE_Msk           (0x3UL << DMA_CCR4_SSIZE_Pos)                /*!<DMA CCR4: SSIZE Mask */

#define DMA_CCR4_PL_Pos              14                                           /*!<DMA CCR4: PL Position */
#define DMA_CCR4_PL_Msk              (0x3UL << DMA_CCR4_PL_Pos)                   /*!<DMA CCR4: PL Mask */

#define DMA_CCR4_BURSTLEN_Pos        16                                           /*!<DMA CCR4: BURSTLEN Position */
#define DMA_CCR4_BURSTLEN_Msk        (0xFUL << DMA_CCR4_BURSTLEN_Pos)             /*!<DMA CCR4: BURSTLEN Mask */

#define DMA_CCR4_STRMSEL_Pos         20                                           /*!<DMA CCR4: STRMSEL Position */
#define DMA_CCR4_STRMSEL_Msk         (0x7UL << DMA_CCR4_STRMSEL_Pos)              /*!<DMA CCR4: STRMSEL Mask */

#define DMA_CCR4_TRGMODE_Pos         23                                           /*!<DMA CCR4: TRGMODE Position */
#define DMA_CCR4_TRGMODE_Msk         (0x1UL << DMA_CCR4_TRGMODE_Pos)              /*!<DMA CCR4: TRGMODE Mask */

#define DMA_CCR4_Msk_ALL             (DMA_CCR4_EN_Msk              |\
                                      DMA_CCR4_TCIE_Msk            |\
                                      DMA_CCR4_HTIE_Msk            |\
                                      DMA_CCR4_BEIE_Msk            |\
                                      DMA_CCR4_TEIE_Msk            |\
                                      DMA_CCR4_DPTYP_Msk           |\
                                      DMA_CCR4_SPTYP_Msk           |\
                                      DMA_CCR4_DSIZE_Msk           |\
                                      DMA_CCR4_SSIZE_Msk           |\
                                      DMA_CCR4_PL_Msk              |\
                                      DMA_CCR4_BURSTLEN_Msk        |\
                                      DMA_CCR4_STRMSEL_Msk         |\
                                      DMA_CCR4_TRGMODE_Msk         )

#define DMA_NPKT4_NPKT_Pos           0                                            /*!<DMA NPKT4: NPKT Position */
#define DMA_NPKT4_NPKT_Msk           (0x7FFUL /*<< DMA_NPKT4_NPKT_Pos*/)          /*!<DMA NPKT4: NPKT Mask */

#define DMA_NPKT4_Msk_ALL            (DMA_NPKT4_NPKT_Msk          )

#define DMA_CPKT4_CPKT_Pos           0                                            /*!<DMA CPKT4: CPKT Position */
#define DMA_CPKT4_CPKT_Msk           (0x7FFUL /*<< DMA_CPKT4_CPKT_Pos*/)          /*!<DMA CPKT4: CPKT Mask */

#define DMA_CPKT4_Msk_ALL            (DMA_CPKT4_CPKT_Msk          )

#define DMA_CCR5_EN_Pos              0                                            /*!<DMA CCR5: EN Position */
#define DMA_CCR5_EN_Msk              (0x1UL /*<< DMA_CCR5_EN_Pos*/)               /*!<DMA CCR5: EN Mask */

#define DMA_CCR5_TCIE_Pos            1                                            /*!<DMA CCR5: TCIE Position */
#define DMA_CCR5_TCIE_Msk            (0x1UL << DMA_CCR5_TCIE_Pos)                 /*!<DMA CCR5: TCIE Mask */

#define DMA_CCR5_HTIE_Pos            2                                            /*!<DMA CCR5: HTIE Position */
#define DMA_CCR5_HTIE_Msk            (0x1UL << DMA_CCR5_HTIE_Pos)                 /*!<DMA CCR5: HTIE Mask */

#define DMA_CCR5_BEIE_Pos            3                                            /*!<DMA CCR5: BEIE Position */
#define DMA_CCR5_BEIE_Msk            (0x1UL << DMA_CCR5_BEIE_Pos)                 /*!<DMA CCR5: BEIE Mask */

#define DMA_CCR5_TEIE_Pos            4                                            /*!<DMA CCR5: TEIE Position */
#define DMA_CCR5_TEIE_Msk            (0x1UL << DMA_CCR5_TEIE_Pos)                 /*!<DMA CCR5: TEIE Mask */

#define DMA_CCR5_DPTYP_Pos           6                                            /*!<DMA CCR5: DPTYP Position */
#define DMA_CCR5_DPTYP_Msk           (0x3UL << DMA_CCR5_DPTYP_Pos)                /*!<DMA CCR5: DPTYP Mask */

#define DMA_CCR5_SPTYP_Pos           8                                            /*!<DMA CCR5: SPTYP Position */
#define DMA_CCR5_SPTYP_Msk           (0x3UL << DMA_CCR5_SPTYP_Pos)                /*!<DMA CCR5: SPTYP Mask */

#define DMA_CCR5_DSIZE_Pos           10                                           /*!<DMA CCR5: DSIZE Position */
#define DMA_CCR5_DSIZE_Msk           (0x3UL << DMA_CCR5_DSIZE_Pos)                /*!<DMA CCR5: DSIZE Mask */

#define DMA_CCR5_SSIZE_Pos           12                                           /*!<DMA CCR5: SSIZE Position */
#define DMA_CCR5_SSIZE_Msk           (0x3UL << DMA_CCR5_SSIZE_Pos)                /*!<DMA CCR5: SSIZE Mask */

#define DMA_CCR5_PL_Pos              14                                           /*!<DMA CCR5: PL Position */
#define DMA_CCR5_PL_Msk              (0x3UL << DMA_CCR5_PL_Pos)                   /*!<DMA CCR5: PL Mask */

#define DMA_CCR5_BURSTLEN_Pos        16                                           /*!<DMA CCR5: BURSTLEN Position */
#define DMA_CCR5_BURSTLEN_Msk        (0xFUL << DMA_CCR5_BURSTLEN_Pos)             /*!<DMA CCR5: BURSTLEN Mask */

#define DMA_CCR5_STRMSEL_Pos         20                                           /*!<DMA CCR5: STRMSEL Position */
#define DMA_CCR5_STRMSEL_Msk         (0x7UL << DMA_CCR5_STRMSEL_Pos)              /*!<DMA CCR5: STRMSEL Mask */

#define DMA_CCR5_TRGMODE_Pos         23                                           /*!<DMA CCR5: TRGMODE Position */
#define DMA_CCR5_TRGMODE_Msk         (0x1UL << DMA_CCR5_TRGMODE_Pos)              /*!<DMA CCR5: TRGMODE Mask */

#define DMA_CCR5_Msk_ALL             (DMA_CCR5_EN_Msk              |\
                                      DMA_CCR5_TCIE_Msk            |\
                                      DMA_CCR5_HTIE_Msk            |\
                                      DMA_CCR5_BEIE_Msk            |\
                                      DMA_CCR5_TEIE_Msk            |\
                                      DMA_CCR5_DPTYP_Msk           |\
                                      DMA_CCR5_SPTYP_Msk           |\
                                      DMA_CCR5_DSIZE_Msk           |\
                                      DMA_CCR5_SSIZE_Msk           |\
                                      DMA_CCR5_PL_Msk              |\
                                      DMA_CCR5_BURSTLEN_Msk        |\
                                      DMA_CCR5_STRMSEL_Msk         |\
                                      DMA_CCR5_TRGMODE_Msk         )

#define DMA_NPKT5_NPKT_Pos           0                                            /*!<DMA NPKT5: NPKT Position */
#define DMA_NPKT5_NPKT_Msk           (0x7FFUL /*<< DMA_NPKT5_NPKT_Pos*/)          /*!<DMA NPKT5: NPKT Mask */

#define DMA_NPKT5_Msk_ALL            (DMA_NPKT5_NPKT_Msk          )

#define DMA_CPKT5_CPKT_Pos           0                                            /*!<DMA CPKT5: CPKT Position */
#define DMA_CPKT5_CPKT_Msk           (0x7FFUL /*<< DMA_CPKT5_CPKT_Pos*/)          /*!<DMA CPKT5: CPKT Mask */

#define DMA_CPKT5_Msk_ALL            (DMA_CPKT5_CPKT_Msk          )

#define DMA_CCR6_EN_Pos              0                                            /*!<DMA CCR6: EN Position */
#define DMA_CCR6_EN_Msk              (0x1UL /*<< DMA_CCR6_EN_Pos*/)               /*!<DMA CCR6: EN Mask */

#define DMA_CCR6_TCIE_Pos            1                                            /*!<DMA CCR6: TCIE Position */
#define DMA_CCR6_TCIE_Msk            (0x1UL << DMA_CCR6_TCIE_Pos)                 /*!<DMA CCR6: TCIE Mask */

#define DMA_CCR6_HTIE_Pos            2                                            /*!<DMA CCR6: HTIE Position */
#define DMA_CCR6_HTIE_Msk            (0x1UL << DMA_CCR6_HTIE_Pos)                 /*!<DMA CCR6: HTIE Mask */

#define DMA_CCR6_BEIE_Pos            3                                            /*!<DMA CCR6: BEIE Position */
#define DMA_CCR6_BEIE_Msk            (0x1UL << DMA_CCR6_BEIE_Pos)                 /*!<DMA CCR6: BEIE Mask */

#define DMA_CCR6_TEIE_Pos            4                                            /*!<DMA CCR6: TEIE Position */
#define DMA_CCR6_TEIE_Msk            (0x1UL << DMA_CCR6_TEIE_Pos)                 /*!<DMA CCR6: TEIE Mask */

#define DMA_CCR6_DPTYP_Pos           6                                            /*!<DMA CCR6: DPTYP Position */
#define DMA_CCR6_DPTYP_Msk           (0x3UL << DMA_CCR6_DPTYP_Pos)                /*!<DMA CCR6: DPTYP Mask */

#define DMA_CCR6_SPTYP_Pos           8                                            /*!<DMA CCR6: SPTYP Position */
#define DMA_CCR6_SPTYP_Msk           (0x3UL << DMA_CCR6_SPTYP_Pos)                /*!<DMA CCR6: SPTYP Mask */

#define DMA_CCR6_DSIZE_Pos           10                                           /*!<DMA CCR6: DSIZE Position */
#define DMA_CCR6_DSIZE_Msk           (0x3UL << DMA_CCR6_DSIZE_Pos)                /*!<DMA CCR6: DSIZE Mask */

#define DMA_CCR6_SSIZE_Pos           12                                           /*!<DMA CCR6: SSIZE Position */
#define DMA_CCR6_SSIZE_Msk           (0x3UL << DMA_CCR6_SSIZE_Pos)                /*!<DMA CCR6: SSIZE Mask */

#define DMA_CCR6_PL_Pos              14                                           /*!<DMA CCR6: PL Position */
#define DMA_CCR6_PL_Msk              (0x3UL << DMA_CCR6_PL_Pos)                   /*!<DMA CCR6: PL Mask */

#define DMA_CCR6_BURSTLEN_Pos        16                                           /*!<DMA CCR6: BURSTLEN Position */
#define DMA_CCR6_BURSTLEN_Msk        (0xFUL << DMA_CCR6_BURSTLEN_Pos)             /*!<DMA CCR6: BURSTLEN Mask */

#define DMA_CCR6_STRMSEL_Pos         20                                           /*!<DMA CCR6: STRMSEL Position */
#define DMA_CCR6_STRMSEL_Msk         (0x7UL << DMA_CCR6_STRMSEL_Pos)              /*!<DMA CCR6: STRMSEL Mask */

#define DMA_CCR6_TRGMODE_Pos         23                                           /*!<DMA CCR6: TRGMODE Position */
#define DMA_CCR6_TRGMODE_Msk         (0x1UL << DMA_CCR6_TRGMODE_Pos)              /*!<DMA CCR6: TRGMODE Mask */

#define DMA_CCR6_Msk_ALL             (DMA_CCR6_EN_Msk              |\
                                      DMA_CCR6_TCIE_Msk            |\
                                      DMA_CCR6_HTIE_Msk            |\
                                      DMA_CCR6_BEIE_Msk            |\
                                      DMA_CCR6_TEIE_Msk            |\
                                      DMA_CCR6_DPTYP_Msk           |\
                                      DMA_CCR6_SPTYP_Msk           |\
                                      DMA_CCR6_DSIZE_Msk           |\
                                      DMA_CCR6_SSIZE_Msk           |\
                                      DMA_CCR6_PL_Msk              |\
                                      DMA_CCR6_BURSTLEN_Msk        |\
                                      DMA_CCR6_STRMSEL_Msk         |\
                                      DMA_CCR6_TRGMODE_Msk         )

#define DMA_NPKT6_NPKT_Pos           0                                            /*!<DMA NPKT6: NPKT Position */
#define DMA_NPKT6_NPKT_Msk           (0x7FFUL /*<< DMA_NPKT6_NPKT_Pos*/)          /*!<DMA NPKT6: NPKT Mask */

#define DMA_NPKT6_Msk_ALL            (DMA_NPKT6_NPKT_Msk          )

#define DMA_CPKT6_CPKT_Pos           0                                            /*!<DMA CPKT6: CPKT Position */
#define DMA_CPKT6_CPKT_Msk           (0x7FFUL /*<< DMA_CPKT6_CPKT_Pos*/)          /*!<DMA CPKT6: CPKT Mask */

#define DMA_CPKT6_Msk_ALL            (DMA_CPKT6_CPKT_Msk          )

#define DMA_CCR7_EN_Pos              0                                            /*!<DMA CCR7: EN Position */
#define DMA_CCR7_EN_Msk              (0x1UL /*<< DMA_CCR7_EN_Pos*/)               /*!<DMA CCR7: EN Mask */

#define DMA_CCR7_TCIE_Pos            1                                            /*!<DMA CCR7: TCIE Position */
#define DMA_CCR7_TCIE_Msk            (0x1UL << DMA_CCR7_TCIE_Pos)                 /*!<DMA CCR7: TCIE Mask */

#define DMA_CCR7_HTIE_Pos            2                                            /*!<DMA CCR7: HTIE Position */
#define DMA_CCR7_HTIE_Msk            (0x1UL << DMA_CCR7_HTIE_Pos)                 /*!<DMA CCR7: HTIE Mask */

#define DMA_CCR7_BEIE_Pos            3                                            /*!<DMA CCR7: BEIE Position */
#define DMA_CCR7_BEIE_Msk            (0x1UL << DMA_CCR7_BEIE_Pos)                 /*!<DMA CCR7: BEIE Mask */

#define DMA_CCR7_TEIE_Pos            4                                            /*!<DMA CCR7: TEIE Position */
#define DMA_CCR7_TEIE_Msk            (0x1UL << DMA_CCR7_TEIE_Pos)                 /*!<DMA CCR7: TEIE Mask */

#define DMA_CCR7_DPTYP_Pos           6                                            /*!<DMA CCR7: DPTYP Position */
#define DMA_CCR7_DPTYP_Msk           (0x3UL << DMA_CCR7_DPTYP_Pos)                /*!<DMA CCR7: DPTYP Mask */

#define DMA_CCR7_SPTYP_Pos           8                                            /*!<DMA CCR7: SPTYP Position */
#define DMA_CCR7_SPTYP_Msk           (0x3UL << DMA_CCR7_SPTYP_Pos)                /*!<DMA CCR7: SPTYP Mask */

#define DMA_CCR7_DSIZE_Pos           10                                           /*!<DMA CCR7: DSIZE Position */
#define DMA_CCR7_DSIZE_Msk           (0x3UL << DMA_CCR7_DSIZE_Pos)                /*!<DMA CCR7: DSIZE Mask */

#define DMA_CCR7_SSIZE_Pos           12                                           /*!<DMA CCR7: SSIZE Position */
#define DMA_CCR7_SSIZE_Msk           (0x3UL << DMA_CCR7_SSIZE_Pos)                /*!<DMA CCR7: SSIZE Mask */

#define DMA_CCR7_PL_Pos              14                                           /*!<DMA CCR7: PL Position */
#define DMA_CCR7_PL_Msk              (0x3UL << DMA_CCR7_PL_Pos)                   /*!<DMA CCR7: PL Mask */

#define DMA_CCR7_BURSTLEN_Pos        16                                           /*!<DMA CCR7: BURSTLEN Position */
#define DMA_CCR7_BURSTLEN_Msk        (0xFUL << DMA_CCR7_BURSTLEN_Pos)             /*!<DMA CCR7: BURSTLEN Mask */

#define DMA_CCR7_STRMSEL_Pos         20                                           /*!<DMA CCR7: STRMSEL Position */
#define DMA_CCR7_STRMSEL_Msk         (0x7UL << DMA_CCR7_STRMSEL_Pos)              /*!<DMA CCR7: STRMSEL Mask */

#define DMA_CCR7_TRGMODE_Pos         23                                           /*!<DMA CCR7: TRGMODE Position */
#define DMA_CCR7_TRGMODE_Msk         (0x1UL << DMA_CCR7_TRGMODE_Pos)              /*!<DMA CCR7: TRGMODE Mask */

#define DMA_CCR7_Msk_ALL             (DMA_CCR7_EN_Msk              |\
                                      DMA_CCR7_TCIE_Msk            |\
                                      DMA_CCR7_HTIE_Msk            |\
                                      DMA_CCR7_BEIE_Msk            |\
                                      DMA_CCR7_TEIE_Msk            |\
                                      DMA_CCR7_DPTYP_Msk           |\
                                      DMA_CCR7_SPTYP_Msk           |\
                                      DMA_CCR7_DSIZE_Msk           |\
                                      DMA_CCR7_SSIZE_Msk           |\
                                      DMA_CCR7_PL_Msk              |\
                                      DMA_CCR7_BURSTLEN_Msk        |\
                                      DMA_CCR7_STRMSEL_Msk         |\
                                      DMA_CCR7_TRGMODE_Msk         )

#define DMA_NPKT7_NPKT_Pos           0                                            /*!<DMA NPKT7: NPKT Position */
#define DMA_NPKT7_NPKT_Msk           (0x7FFUL /*<< DMA_NPKT7_NPKT_Pos*/)          /*!<DMA NPKT7: NPKT Mask */

#define DMA_NPKT7_Msk_ALL            (DMA_NPKT7_NPKT_Msk          )

#define DMA_CPKT7_CPKT_Pos           0                                            /*!<DMA CPKT7: CPKT Position */
#define DMA_CPKT7_CPKT_Msk           (0x7FFUL /*<< DMA_CPKT7_CPKT_Pos*/)          /*!<DMA CPKT7: CPKT Mask */

#define DMA_CPKT7_Msk_ALL            (DMA_CPKT7_CPKT_Msk          )

#define MCM_PWMOE_PWMOE_Pos          0                                            /*!<MCM PWMOE: PWMOE Position */
#define MCM_PWMOE_PWMOE_Msk          (0x1UL /*<< MCM_PWMOE_PWMOE_Pos*/)           /*!<MCM PWMOE: PWMOE Mask */

#define MCM_PWMOE_Msk_ALL            (MCM_PWMOE_PWMOE_Msk         )

#define MCM_PWMCON1_PWM0S_Pos        0                                            /*!<MCM PWMCON1: PWM0S Position */
#define MCM_PWMCON1_PWM0S_Msk        (0x1UL /*<< MCM_PWMCON1_PWM0S_Pos*/)         /*!<MCM PWMCON1: PWM0S Mask */

#define MCM_PWMCON1_PWM1S_Pos        1                                            /*!<MCM PWMCON1: PWM1S Position */
#define MCM_PWMCON1_PWM1S_Msk        (0x1UL << MCM_PWMCON1_PWM1S_Pos)             /*!<MCM PWMCON1: PWM1S Mask */

#define MCM_PWMCON1_PWM2S_Pos        2                                            /*!<MCM PWMCON1: PWM2S Position */
#define MCM_PWMCON1_PWM2S_Msk        (0x1UL << MCM_PWMCON1_PWM2S_Pos)             /*!<MCM PWMCON1: PWM2S Mask */

#define MCM_PWMCON1_PWM01S_Pos       3                                            /*!<MCM PWMCON1: PWM01S Position */
#define MCM_PWMCON1_PWM01S_Msk       (0x1UL << MCM_PWMCON1_PWM01S_Pos)            /*!<MCM PWMCON1: PWM01S Mask */

#define MCM_PWMCON1_PWM11S_Pos       4                                            /*!<MCM PWMCON1: PWM11S Position */
#define MCM_PWMCON1_PWM11S_Msk       (0x1UL << MCM_PWMCON1_PWM11S_Pos)            /*!<MCM PWMCON1: PWM11S Mask */

#define MCM_PWMCON1_PWM21S_Pos       5                                            /*!<MCM PWMCON1: PWM21S Position */
#define MCM_PWMCON1_PWM21S_Msk       (0x1UL << MCM_PWMCON1_PWM21S_Pos)            /*!<MCM PWMCON1: PWM21S Mask */

#define MCM_PWMCON1_PDCON0_Pos       6                                            /*!<MCM PWMCON1: PDCON0 Position */
#define MCM_PWMCON1_PDCON0_Msk       (0x1UL << MCM_PWMCON1_PDCON0_Pos)            /*!<MCM PWMCON1: PDCON0 Mask */

#define MCM_PWMCON1_PDCON1_Pos       7                                            /*!<MCM PWMCON1: PDCON1 Position */
#define MCM_PWMCON1_PDCON1_Msk       (0x1UL << MCM_PWMCON1_PDCON1_Pos)            /*!<MCM PWMCON1: PDCON1 Mask */

#define MCM_PWMCON1_PDCON2_Pos       8                                            /*!<MCM PWMCON1: PDCON2 Position */
#define MCM_PWMCON1_PDCON2_Msk       (0x1UL << MCM_PWMCON1_PDCON2_Pos)            /*!<MCM PWMCON1: PDCON2 Mask */

#define MCM_PWMCON1_PWMSYM_Pos       9                                            /*!<MCM PWMCON1: PWMSYM Position */
#define MCM_PWMCON1_PWMSYM_Msk       (0x1UL << MCM_PWMCON1_PWMSYM_Pos)            /*!<MCM PWMCON1: PWMSYM Mask */

#define MCM_PWMCON1_PTMOD_Pos        10                                           /*!<MCM PWMCON1: PTMOD Position */
#define MCM_PWMCON1_PTMOD_Msk        (0x3UL << MCM_PWMCON1_PTMOD_Pos)             /*!<MCM PWMCON1: PTMOD Mask */

#define MCM_PWMCON1_POSTPS_Pos       12                                           /*!<MCM PWMCON1: POSTPS Position */
#define MCM_PWMCON1_POSTPS_Msk       (0x7UL << MCM_PWMCON1_POSTPS_Pos)            /*!<MCM PWMCON1: POSTPS Mask */

#define MCM_PWMCON1_POUTMOD_Pos      15                                           /*!<MCM PWMCON1: POUTMOD Position */
#define MCM_PWMCON1_POUTMOD_Msk      (0x1UL << MCM_PWMCON1_POUTMOD_Pos)           /*!<MCM PWMCON1: POUTMOD Mask */

#define MCM_PWMCON1_Msk_ALL          (MCM_PWMCON1_PWM0S_Msk        |\
                                      MCM_PWMCON1_PWM1S_Msk        |\
                                      MCM_PWMCON1_PWM2S_Msk        |\
                                      MCM_PWMCON1_PWM01S_Msk       |\
                                      MCM_PWMCON1_PWM11S_Msk       |\
                                      MCM_PWMCON1_PWM21S_Msk       |\
                                      MCM_PWMCON1_PDCON0_Msk       |\
                                      MCM_PWMCON1_PDCON1_Msk       |\
                                      MCM_PWMCON1_PDCON2_Msk       |\
                                      MCM_PWMCON1_PWMSYM_Msk       |\
                                      MCM_PWMCON1_PTMOD_Msk        |\
                                      MCM_PWMCON1_POSTPS_Msk       |\
                                      MCM_PWMCON1_POUTMOD_Msk      )

#define MCM_PWMCON2_CMP1_Pos         0                                            /*!<MCM PWMCON2: CMP1 Position */
#define MCM_PWMCON2_CMP1_Msk         (0x3UL /*<< MCM_PWMCON2_CMP1_Pos*/)          /*!<MCM PWMCON2: CMP1 Mask */

#define MCM_PWMCON2_CMP2_Pos         2                                            /*!<MCM PWMCON2: CMP2 Position */
#define MCM_PWMCON2_CMP2_Msk         (0x3UL << MCM_PWMCON2_CMP2_Pos)              /*!<MCM PWMCON2: CMP2 Mask */

#define MCM_PWMCON2_CMP3_Pos         4                                            /*!<MCM PWMCON2: CMP3 Position */
#define MCM_PWMCON2_CMP3_Msk         (0x3UL << MCM_PWMCON2_CMP3_Pos)              /*!<MCM PWMCON2: CMP3 Mask */

#define MCM_PWMCON2_CMP4_Pos         6                                            /*!<MCM PWMCON2: CMP4 Position */
#define MCM_PWMCON2_CMP4_Msk         (0x3UL << MCM_PWMCON2_CMP4_Pos)              /*!<MCM PWMCON2: CMP4 Mask */

#define MCM_PWMCON2_OSYNC_Pos        8                                            /*!<MCM PWMCON2: OSYNC Position */
#define MCM_PWMCON2_OSYNC_Msk        (0x1UL << MCM_PWMCON2_OSYNC_Pos)             /*!<MCM PWMCON2: OSYNC Mask */

#define MCM_PWMCON2_DILDEN_Pos       9                                            /*!<MCM PWMCON2: DILDEN Position */
#define MCM_PWMCON2_DILDEN_Msk       (0x1UL << MCM_PWMCON2_DILDEN_Pos)            /*!<MCM PWMCON2: DILDEN Mask */

#define MCM_PWMCON2_CILDEN_Pos       10                                           /*!<MCM PWMCON2: CILDEN Position */
#define MCM_PWMCON2_CILDEN_Msk       (0x1UL << MCM_PWMCON2_CILDEN_Pos)            /*!<MCM PWMCON2: CILDEN Mask */

#define MCM_PWMCON2_ZDLDEN_Pos       11                                           /*!<MCM PWMCON2: ZDLDEN Position */
#define MCM_PWMCON2_ZDLDEN_Msk       (0x1UL << MCM_PWMCON2_ZDLDEN_Pos)            /*!<MCM PWMCON2: ZDLDEN Mask */

#define MCM_PWMCON2_PDLDEN_Pos       12                                           /*!<MCM PWMCON2: PDLDEN Position */
#define MCM_PWMCON2_PDLDEN_Msk       (0x1UL << MCM_PWMCON2_PDLDEN_Pos)            /*!<MCM PWMCON2: PDLDEN Mask */

#define MCM_PWMCON2_ZCMLDEN_Pos      13                                           /*!<MCM PWMCON2: ZCMLDEN Position */
#define MCM_PWMCON2_ZCMLDEN_Msk      (0x1UL << MCM_PWMCON2_ZCMLDEN_Pos)           /*!<MCM PWMCON2: ZCMLDEN Mask */

#define MCM_PWMCON2_PCMLDEN_Pos      14                                           /*!<MCM PWMCON2: PCMLDEN Position */
#define MCM_PWMCON2_PCMLDEN_Msk      (0x1UL << MCM_PWMCON2_PCMLDEN_Pos)           /*!<MCM PWMCON2: PCMLDEN Mask */

#define MCM_PWMCON2_MCMSYNEN_Pos     15                                           /*!<MCM PWMCON2: MCMSYNEN Position */
#define MCM_PWMCON2_MCMSYNEN_Msk     (0x1UL << MCM_PWMCON2_MCMSYNEN_Pos)          /*!<MCM PWMCON2: MCMSYNEN Mask */

#define MCM_PWMCON2_Msk_ALL          (MCM_PWMCON2_CMP1_Msk         |\
                                      MCM_PWMCON2_CMP2_Msk         |\
                                      MCM_PWMCON2_CMP3_Msk         |\
                                      MCM_PWMCON2_CMP4_Msk         |\
                                      MCM_PWMCON2_OSYNC_Msk        |\
                                      MCM_PWMCON2_DILDEN_Msk       |\
                                      MCM_PWMCON2_CILDEN_Msk       |\
                                      MCM_PWMCON2_ZDLDEN_Msk       |\
                                      MCM_PWMCON2_PDLDEN_Msk       |\
                                      MCM_PWMCON2_ZCMLDEN_Msk      |\
                                      MCM_PWMCON2_PCMLDEN_Msk      |\
                                      MCM_PWMCON2_MCMSYNEN_Msk     )

#define MCM_PWMP_PWMP_Pos            0                                            /*!<MCM PWMP: PWMP Position */
#define MCM_PWMP_PWMP_Msk            (0xFFFFUL /*<< MCM_PWMP_PWMP_Pos*/)          /*!<MCM PWMP: PWMP Mask */

#define MCM_PWMP_Msk_ALL             (MCM_PWMP_PWMP_Msk           )

#define MCM_PWMC_PWMC_Pos            0                                            /*!<MCM PWMC: PWMC Position */
#define MCM_PWMC_PWMC_Msk            (0xFFFFUL /*<< MCM_PWMC_PWMC_Pos*/)          /*!<MCM PWMC: PWMC Mask */

#define MCM_PWMC_Msk_ALL             (MCM_PWMC_PWMC_Msk           )

#define MCM_PWMPSQ_PWMPSQ_Pos        0                                            /*!<MCM PWMPSQ: PWMPSQ Position */
#define MCM_PWMPSQ_PWMPSQ_Msk        (0xFFFFUL /*<< MCM_PWMPSQ_PWMPSQ_Pos*/)      /*!<MCM PWMPSQ: PWMPSQ Mask */

#define MCM_PWMPSQ_Msk_ALL           (MCM_PWMPSQ_PWMPSQ_Msk       )

#define MCM_PWM0D_PWM0D_Pos          0                                            /*!<MCM PWM0D: PWM0D Position */
#define MCM_PWM0D_PWM0D_Msk          (0xFFFFUL /*<< MCM_PWM0D_PWM0D_Pos*/)        /*!<MCM PWM0D: PWM0D Mask */

#define MCM_PWM0D_Msk_ALL            (MCM_PWM0D_PWM0D_Msk         )

#define MCM_PWM1D_PWM1D_Pos          0                                            /*!<MCM PWM1D: PWM1D Position */
#define MCM_PWM1D_PWM1D_Msk          (0xFFFFUL /*<< MCM_PWM1D_PWM1D_Pos*/)        /*!<MCM PWM1D: PWM1D Mask */

#define MCM_PWM1D_Msk_ALL            (MCM_PWM1D_PWM1D_Msk         )

#define MCM_PWM2D_PWM2D_Pos          0                                            /*!<MCM PWM2D: PWM2D Position */
#define MCM_PWM2D_PWM2D_Msk          (0xFFFFUL /*<< MCM_PWM2D_PWM2D_Pos*/)        /*!<MCM PWM2D: PWM2D Mask */

#define MCM_PWM2D_Msk_ALL            (MCM_PWM2D_PWM2D_Msk         )

#define MCM_PWM01D_PWM01D_Pos        0                                            /*!<MCM PWM01D: PWM01D Position */
#define MCM_PWM01D_PWM01D_Msk        (0xFFFFUL /*<< MCM_PWM01D_PWM01D_Pos*/)      /*!<MCM PWM01D: PWM01D Mask */

#define MCM_PWM01D_Msk_ALL           (MCM_PWM01D_PWM01D_Msk       )

#define MCM_PWM11D_PWM11D_Pos        0                                            /*!<MCM PWM11D: PWM11D Position */
#define MCM_PWM11D_PWM11D_Msk        (0xFFFFUL /*<< MCM_PWM11D_PWM11D_Pos*/)      /*!<MCM PWM11D: PWM11D Mask */

#define MCM_PWM11D_Msk_ALL           (MCM_PWM11D_PWM11D_Msk       )

#define MCM_PWM21D_PWM21D_Pos        0                                            /*!<MCM PWM21D: PWM21D Position */
#define MCM_PWM21D_PWM21D_Msk        (0xFFFFUL /*<< MCM_PWM21D_PWM21D_Pos*/)      /*!<MCM PWM21D: PWM21D Mask */

#define MCM_PWM21D_Msk_ALL           (MCM_PWM21D_PWM21D_Msk       )

#define MCM_PWMCMP1_PWMCMP1_Pos      0                                            /*!<MCM PWMCMP1: PWMCMP1 Position */
#define MCM_PWMCMP1_PWMCMP1_Msk      (0xFFFFUL /*<< MCM_PWMCMP1_PWMCMP1_Pos*/)    /*!<MCM PWMCMP1: PWMCMP1 Mask */

#define MCM_PWMCMP1_Msk_ALL          (MCM_PWMCMP1_PWMCMP1_Msk     )

#define MCM_PWMCMP2_PWMCMP2_Pos      0                                            /*!<MCM PWMCMP2: PWMCMP2 Position */
#define MCM_PWMCMP2_PWMCMP2_Msk      (0xFFFFUL /*<< MCM_PWMCMP2_PWMCMP2_Pos*/)    /*!<MCM PWMCMP2: PWMCMP2 Mask */

#define MCM_PWMCMP2_Msk_ALL          (MCM_PWMCMP2_PWMCMP2_Msk     )

#define MCM_PWMCMP3_PWMCMP3_Pos      0                                            /*!<MCM PWMCMP3: PWMCMP3 Position */
#define MCM_PWMCMP3_PWMCMP3_Msk      (0xFFFFUL /*<< MCM_PWMCMP3_PWMCMP3_Pos*/)    /*!<MCM PWMCMP3: PWMCMP3 Mask */

#define MCM_PWMCMP3_Msk_ALL          (MCM_PWMCMP3_PWMCMP3_Msk     )

#define MCM_PWMCMP4_PWMCMP4_Pos      0                                            /*!<MCM PWMCMP4: PWMCMP4 Position */
#define MCM_PWMCMP4_PWMCMP4_Msk      (0xFFFFUL /*<< MCM_PWMCMP4_PWMCMP4_Pos*/)    /*!<MCM PWMCMP4: PWMCMP4 Mask */

#define MCM_PWMCMP4_Msk_ALL          (MCM_PWMCMP4_PWMCMP4_Msk     )

#define MCM_PWMDT00_PWMDT00_Pos      0                                            /*!<MCM PWMDT00: PWMDT00 Position */
#define MCM_PWMDT00_PWMDT00_Msk      (0xFFFFUL /*<< MCM_PWMDT00_PWMDT00_Pos*/)    /*!<MCM PWMDT00: PWMDT00 Mask */

#define MCM_PWMDT00_Msk_ALL          (MCM_PWMDT00_PWMDT00_Msk     )

#define MCM_PWMDT01_PWMDT01_Pos      0                                            /*!<MCM PWMDT01: PWMDT01 Position */
#define MCM_PWMDT01_PWMDT01_Msk      (0xFFFFUL /*<< MCM_PWMDT01_PWMDT01_Pos*/)    /*!<MCM PWMDT01: PWMDT01 Mask */

#define MCM_PWMDT01_Msk_ALL          (MCM_PWMDT01_PWMDT01_Msk     )

#define MCM_PWMDT10_PWMDT10_Pos      0                                            /*!<MCM PWMDT10: PWMDT10 Position */
#define MCM_PWMDT10_PWMDT10_Msk      (0xFFFFUL /*<< MCM_PWMDT10_PWMDT10_Pos*/)    /*!<MCM PWMDT10: PWMDT10 Mask */

#define MCM_PWMDT10_Msk_ALL          (MCM_PWMDT10_PWMDT10_Msk     )

#define MCM_PWMDT11_PWMDT11_Pos      0                                            /*!<MCM PWMDT11: PWMDT11 Position */
#define MCM_PWMDT11_PWMDT11_Msk      (0xFFFFUL /*<< MCM_PWMDT11_PWMDT11_Pos*/)    /*!<MCM PWMDT11: PWMDT11 Mask */

#define MCM_PWMDT11_Msk_ALL          (MCM_PWMDT11_PWMDT11_Msk     )

#define MCM_PWMDT20_PWMDT20_Pos      0                                            /*!<MCM PWMDT20: PWMDT20 Position */
#define MCM_PWMDT20_PWMDT20_Msk      (0xFFFFUL /*<< MCM_PWMDT20_PWMDT20_Pos*/)    /*!<MCM PWMDT20: PWMDT20 Mask */

#define MCM_PWMDT20_Msk_ALL          (MCM_PWMDT20_PWMDT20_Msk     )

#define MCM_PWMDT21_PWMDT21_Pos      0                                            /*!<MCM PWMDT21: PWMDT21 Position */
#define MCM_PWMDT21_PWMDT21_Msk      (0xFFFFUL /*<< MCM_PWMDT21_PWMDT21_Pos*/)    /*!<MCM PWMDT21: PWMDT21 Mask */

#define MCM_PWMDT21_Msk_ALL          (MCM_PWMDT21_PWMDT21_Msk     )

#define MCM_PMANUALCON1_PMANUAL0_Pos 0                                            /*!<MCM PMANUALCON1: PMANUAL0 Position */
#define MCM_PMANUALCON1_PMANUAL0_Msk (0x1UL /*<< MCM_PMANUALCON1_PMANUAL0_Pos*/)  /*!<MCM PMANUALCON1: PMANUAL0 Mask */

#define MCM_PMANUALCON1_PMANUAL1_Pos 1                                            /*!<MCM PMANUALCON1: PMANUAL1 Position */
#define MCM_PMANUALCON1_PMANUAL1_Msk (0x1UL << MCM_PMANUALCON1_PMANUAL1_Pos)      /*!<MCM PMANUALCON1: PMANUAL1 Mask */

#define MCM_PMANUALCON1_PMANUAL2_Pos 2                                            /*!<MCM PMANUALCON1: PMANUAL2 Position */
#define MCM_PMANUALCON1_PMANUAL2_Msk (0x1UL << MCM_PMANUALCON1_PMANUAL2_Pos)      /*!<MCM PMANUALCON1: PMANUAL2 Mask */

#define MCM_PMANUALCON1_PMANUAL01_Pos 3                                            /*!<MCM PMANUALCON1: PMANUAL01 Position */
#define MCM_PMANUALCON1_PMANUAL01_Msk (0x1UL << MCM_PMANUALCON1_PMANUAL01_Pos)     /*!<MCM PMANUALCON1: PMANUAL01 Mask */

#define MCM_PMANUALCON1_PMANUAL11_Pos 4                                            /*!<MCM PMANUALCON1: PMANUAL11 Position */
#define MCM_PMANUALCON1_PMANUAL11_Msk (0x1UL << MCM_PMANUALCON1_PMANUAL11_Pos)     /*!<MCM PMANUALCON1: PMANUAL11 Mask */

#define MCM_PMANUALCON1_PMANUAL21_Pos 5                                            /*!<MCM PMANUALCON1: PMANUAL21 Position */
#define MCM_PMANUALCON1_PMANUAL21_Msk (0x1UL << MCM_PMANUALCON1_PMANUAL21_Pos)     /*!<MCM PMANUALCON1: PMANUAL21 Mask */

#define MCM_PMANUALCON1_Msk_ALL      (MCM_PMANUALCON1_PMANUAL0_Msk |\
                                      MCM_PMANUALCON1_PMANUAL1_Msk |\
                                      MCM_PMANUALCON1_PMANUAL2_Msk |\
                                      MCM_PMANUALCON1_PMANUAL01_Msk |\
                                      MCM_PMANUALCON1_PMANUAL11_Msk |\
                                      MCM_PMANUALCON1_PMANUAL21_Msk )

#define MCM_PMANUALCON2_POUT0_Pos    0                                            /*!<MCM PMANUALCON2: POUT0 Position */
#define MCM_PMANUALCON2_POUT0_Msk    (0x1UL /*<< MCM_PMANUALCON2_POUT0_Pos*/)     /*!<MCM PMANUALCON2: POUT0 Mask */

#define MCM_PMANUALCON2_POUT1_Pos    1                                            /*!<MCM PMANUALCON2: POUT1 Position */
#define MCM_PMANUALCON2_POUT1_Msk    (0x1UL << MCM_PMANUALCON2_POUT1_Pos)         /*!<MCM PMANUALCON2: POUT1 Mask */

#define MCM_PMANUALCON2_POUT2_Pos    2                                            /*!<MCM PMANUALCON2: POUT2 Position */
#define MCM_PMANUALCON2_POUT2_Msk    (0x1UL << MCM_PMANUALCON2_POUT2_Pos)         /*!<MCM PMANUALCON2: POUT2 Mask */

#define MCM_PMANUALCON2_POUT01_Pos   3                                            /*!<MCM PMANUALCON2: POUT01 Position */
#define MCM_PMANUALCON2_POUT01_Msk   (0x1UL << MCM_PMANUALCON2_POUT01_Pos)        /*!<MCM PMANUALCON2: POUT01 Mask */

#define MCM_PMANUALCON2_POUT11_Pos   4                                            /*!<MCM PMANUALCON2: POUT11 Position */
#define MCM_PMANUALCON2_POUT11_Msk   (0x1UL << MCM_PMANUALCON2_POUT11_Pos)        /*!<MCM PMANUALCON2: POUT11 Mask */

#define MCM_PMANUALCON2_POUT21_Pos   5                                            /*!<MCM PMANUALCON2: POUT21 Position */
#define MCM_PMANUALCON2_POUT21_Msk   (0x1UL << MCM_PMANUALCON2_POUT21_Pos)        /*!<MCM PMANUALCON2: POUT21 Mask */

#define MCM_PMANUALCON2_Msk_ALL      (MCM_PMANUALCON2_POUT0_Msk    |\
                                      MCM_PMANUALCON2_POUT1_Msk    |\
                                      MCM_PMANUALCON2_POUT2_Msk    |\
                                      MCM_PMANUALCON2_POUT01_Msk   |\
                                      MCM_PMANUALCON2_POUT11_Msk   |\
                                      MCM_PMANUALCON2_POUT21_Msk   )

#define MCM_FLTCON_FLTSTAT_Pos       0                                            /*!<MCM FLTCON: FLTSTAT Position */
#define MCM_FLTCON_FLTSTAT_Msk       (0x1UL /*<< MCM_FLTCON_FLTSTAT_Pos*/)        /*!<MCM FLTCON: FLTSTAT Mask */

#define MCM_FLTCON_FLTM_Pos          1                                            /*!<MCM FLTCON: FLTM Position */
#define MCM_FLTCON_FLTM_Msk          (0x1UL << MCM_FLTCON_FLTM_Pos)               /*!<MCM FLTCON: FLTM Mask */

#define MCM_FLTCON_FLT2S_Pos         2                                            /*!<MCM FLTCON: FLT2S Position */
#define MCM_FLTCON_FLT2S_Msk         (0x1UL << MCM_FLTCON_FLT2S_Pos)              /*!<MCM FLTCON: FLT2S Mask */

#define MCM_FLTCON_FLT2EN_Pos        3                                            /*!<MCM FLTCON: FLT2EN Position */
#define MCM_FLTCON_FLT2EN_Msk        (0x1UL << MCM_FLTCON_FLT2EN_Pos)             /*!<MCM FLTCON: FLT2EN Mask */

#define MCM_FLTCON_FLT2DEB_Pos       4                                            /*!<MCM FLTCON: FLT2DEB Position */
#define MCM_FLTCON_FLT2DEB_Msk       (0xFUL << MCM_FLTCON_FLT2DEB_Pos)            /*!<MCM FLTCON: FLT2DEB Mask */

#define MCM_FLTCON_FLT1SEL_Pos       8                                            /*!<MCM FLTCON: FLT1SEL Position */
#define MCM_FLTCON_FLT1SEL_Msk       (0x3UL << MCM_FLTCON_FLT1SEL_Pos)            /*!<MCM FLTCON: FLT1SEL Mask */

#define MCM_FLTCON_FLT1EN_Pos        10                                           /*!<MCM FLTCON: FLT1EN Position */
#define MCM_FLTCON_FLT1EN_Msk        (0x1UL << MCM_FLTCON_FLT1EN_Pos)             /*!<MCM FLTCON: FLT1EN Mask */

#define MCM_FLTCON_FOUT0_Pos         12                                           /*!<MCM FLTCON: FOUT0 Position */
#define MCM_FLTCON_FOUT0_Msk         (0x3UL << MCM_FLTCON_FOUT0_Pos)              /*!<MCM FLTCON: FOUT0 Mask */

#define MCM_FLTCON_FOUT1_Pos         14                                           /*!<MCM FLTCON: FOUT1 Position */
#define MCM_FLTCON_FOUT1_Msk         (0x3UL << MCM_FLTCON_FOUT1_Pos)              /*!<MCM FLTCON: FOUT1 Mask */

#define MCM_FLTCON_Msk_ALL           (MCM_FLTCON_FLTSTAT_Msk       |\
                                      MCM_FLTCON_FLTM_Msk          |\
                                      MCM_FLTCON_FLT2S_Msk         |\
                                      MCM_FLTCON_FLT2EN_Msk        |\
                                      MCM_FLTCON_FLT2DEB_Msk       |\
                                      MCM_FLTCON_FLT1SEL_Msk       |\
                                      MCM_FLTCON_FLT1EN_Msk        |\
                                      MCM_FLTCON_FOUT0_Msk         |\
                                      MCM_FLTCON_FOUT1_Msk         )

#define MCM_POSCR_OLSG0_Pos          0                                            /*!<MCM POSCR: OLSG0 Position */
#define MCM_POSCR_OLSG0_Msk          (0x1UL /*<< MCM_POSCR_OLSG0_Pos*/)           /*!<MCM POSCR: OLSG0 Mask */

#define MCM_POSCR_OLSG1_Pos          1                                            /*!<MCM POSCR: OLSG1 Position */
#define MCM_POSCR_OLSG1_Msk          (0x1UL << MCM_POSCR_OLSG1_Pos)               /*!<MCM POSCR: OLSG1 Mask */

#define MCM_POSCR_OLSG2_Pos          2                                            /*!<MCM POSCR: OLSG2 Position */
#define MCM_POSCR_OLSG2_Msk          (0x1UL << MCM_POSCR_OLSG2_Pos)               /*!<MCM POSCR: OLSG2 Mask */

#define MCM_POSCR_OLSG01_Pos         3                                            /*!<MCM POSCR: OLSG01 Position */
#define MCM_POSCR_OLSG01_Msk         (0x1UL << MCM_POSCR_OLSG01_Pos)              /*!<MCM POSCR: OLSG01 Mask */

#define MCM_POSCR_OLSG11_Pos         4                                            /*!<MCM POSCR: OLSG11 Position */
#define MCM_POSCR_OLSG11_Msk         (0x1UL << MCM_POSCR_OLSG11_Pos)              /*!<MCM POSCR: OLSG11 Mask */

#define MCM_POSCR_OLSG21_Pos         5                                            /*!<MCM POSCR: OLSG21 Position */
#define MCM_POSCR_OLSG21_Msk         (0x1UL << MCM_POSCR_OLSG21_Pos)              /*!<MCM POSCR: OLSG21 Mask */

#define MCM_POSCR_OLSEN_Pos          7                                            /*!<MCM POSCR: OLSEN Position */
#define MCM_POSCR_OLSEN_Msk          (0x1UL << MCM_POSCR_OLSEN_Pos)               /*!<MCM POSCR: OLSEN Mask */

#define MCM_POSCR_Msk_ALL            (MCM_POSCR_OLSG0_Msk          |\
                                      MCM_POSCR_OLSG1_Msk          |\
                                      MCM_POSCR_OLSG2_Msk          |\
                                      MCM_POSCR_OLSG01_Msk         |\
                                      MCM_POSCR_OLSG11_Msk         |\
                                      MCM_POSCR_OLSG21_Msk         |\
                                      MCM_POSCR_OLSEN_Msk          )

#define MCM_POSTDCR_OSTDEN_Pos       7                                            /*!<MCM POSTDCR: OSTDEN Position */
#define MCM_POSTDCR_OSTDEN_Msk       (0x1UL << MCM_POSTDCR_OSTDEN_Pos)            /*!<MCM POSTDCR: OSTDEN Mask */

#define MCM_POSTDCR_Msk_ALL          (MCM_POSTDCR_OSTDEN_Msk      )

#define MCM_PWMDMAEN_PTUD0DE_Pos     0                                            /*!<MCM PWMDMAEN: PTUD0DE Position */
#define MCM_PWMDMAEN_PTUD0DE_Msk     (0x1UL /*<< MCM_PWMDMAEN_PTUD0DE_Pos*/)      /*!<MCM PWMDMAEN: PTUD0DE Mask */

#define MCM_PWMDMAEN_PTDD0DE_Pos     1                                            /*!<MCM PWMDMAEN: PTDD0DE Position */
#define MCM_PWMDMAEN_PTDD0DE_Msk     (0x1UL << MCM_PWMDMAEN_PTDD0DE_Pos)          /*!<MCM PWMDMAEN: PTDD0DE Mask */

#define MCM_PWMDMAEN_PTUD1DE_Pos     2                                            /*!<MCM PWMDMAEN: PTUD1DE Position */
#define MCM_PWMDMAEN_PTUD1DE_Msk     (0x1UL << MCM_PWMDMAEN_PTUD1DE_Pos)          /*!<MCM PWMDMAEN: PTUD1DE Mask */

#define MCM_PWMDMAEN_PTDD1DE_Pos     3                                            /*!<MCM PWMDMAEN: PTDD1DE Position */
#define MCM_PWMDMAEN_PTDD1DE_Msk     (0x1UL << MCM_PWMDMAEN_PTDD1DE_Pos)          /*!<MCM PWMDMAEN: PTDD1DE Mask */

#define MCM_PWMDMAEN_PTUD2DE_Pos     4                                            /*!<MCM PWMDMAEN: PTUD2DE Position */
#define MCM_PWMDMAEN_PTUD2DE_Msk     (0x1UL << MCM_PWMDMAEN_PTUD2DE_Pos)          /*!<MCM PWMDMAEN: PTUD2DE Mask */

#define MCM_PWMDMAEN_PTDD2DE_Pos     5                                            /*!<MCM PWMDMAEN: PTDD2DE Position */
#define MCM_PWMDMAEN_PTDD2DE_Msk     (0x1UL << MCM_PWMDMAEN_PTDD2DE_Pos)          /*!<MCM PWMDMAEN: PTDD2DE Mask */

#define MCM_PWMDMAEN_PWMZDE_Pos      6                                            /*!<MCM PWMDMAEN: PWMZDE Position */
#define MCM_PWMDMAEN_PWMZDE_Msk      (0x1UL << MCM_PWMDMAEN_PWMZDE_Pos)           /*!<MCM PWMDMAEN: PWMZDE Mask */

#define MCM_PWMDMAEN_PWMPDE_Pos      7                                            /*!<MCM PWMDMAEN: PWMPDE Position */
#define MCM_PWMDMAEN_PWMPDE_Msk      (0x1UL << MCM_PWMDMAEN_PWMPDE_Pos)           /*!<MCM PWMDMAEN: PWMPDE Mask */

#define MCM_PWMDMAEN_Msk_ALL         (MCM_PWMDMAEN_PTUD0DE_Msk     |\
                                      MCM_PWMDMAEN_PTDD0DE_Msk     |\
                                      MCM_PWMDMAEN_PTUD1DE_Msk     |\
                                      MCM_PWMDMAEN_PTDD1DE_Msk     |\
                                      MCM_PWMDMAEN_PTUD2DE_Msk     |\
                                      MCM_PWMDMAEN_PTDD2DE_Msk     |\
                                      MCM_PWMDMAEN_PWMZDE_Msk      |\
                                      MCM_PWMDMAEN_PWMPDE_Msk      )

#define MCM_PWMINTEN_PTUD0IE_Pos     0                                            /*!<MCM PWMINTEN: PTUD0IE Position */
#define MCM_PWMINTEN_PTUD0IE_Msk     (0x1UL /*<< MCM_PWMINTEN_PTUD0IE_Pos*/)      /*!<MCM PWMINTEN: PTUD0IE Mask */

#define MCM_PWMINTEN_PTDD0IE_Pos     1                                            /*!<MCM PWMINTEN: PTDD0IE Position */
#define MCM_PWMINTEN_PTDD0IE_Msk     (0x1UL << MCM_PWMINTEN_PTDD0IE_Pos)          /*!<MCM PWMINTEN: PTDD0IE Mask */

#define MCM_PWMINTEN_PTUD1IE_Pos     2                                            /*!<MCM PWMINTEN: PTUD1IE Position */
#define MCM_PWMINTEN_PTUD1IE_Msk     (0x1UL << MCM_PWMINTEN_PTUD1IE_Pos)          /*!<MCM PWMINTEN: PTUD1IE Mask */

#define MCM_PWMINTEN_PTDD1IE_Pos     3                                            /*!<MCM PWMINTEN: PTDD1IE Position */
#define MCM_PWMINTEN_PTDD1IE_Msk     (0x1UL << MCM_PWMINTEN_PTDD1IE_Pos)          /*!<MCM PWMINTEN: PTDD1IE Mask */

#define MCM_PWMINTEN_PTUD2IE_Pos     4                                            /*!<MCM PWMINTEN: PTUD2IE Position */
#define MCM_PWMINTEN_PTUD2IE_Msk     (0x1UL << MCM_PWMINTEN_PTUD2IE_Pos)          /*!<MCM PWMINTEN: PTUD2IE Mask */

#define MCM_PWMINTEN_PTDD2IE_Pos     5                                            /*!<MCM PWMINTEN: PTDD2IE Position */
#define MCM_PWMINTEN_PTDD2IE_Msk     (0x1UL << MCM_PWMINTEN_PTDD2IE_Pos)          /*!<MCM PWMINTEN: PTDD2IE Mask */

#define MCM_PWMINTEN_PWMZIE_Pos      6                                            /*!<MCM PWMINTEN: PWMZIE Position */
#define MCM_PWMINTEN_PWMZIE_Msk      (0x1UL << MCM_PWMINTEN_PWMZIE_Pos)           /*!<MCM PWMINTEN: PWMZIE Mask */

#define MCM_PWMINTEN_PWMPIE_Pos      7                                            /*!<MCM PWMINTEN: PWMPIE Position */
#define MCM_PWMINTEN_PWMPIE_Msk      (0x1UL << MCM_PWMINTEN_PWMPIE_Pos)           /*!<MCM PWMINTEN: PWMPIE Mask */

#define MCM_PWMINTEN_FLTIE_Pos       8                                            /*!<MCM PWMINTEN: FLTIE Position */
#define MCM_PWMINTEN_FLTIE_Msk       (0x1UL << MCM_PWMINTEN_FLTIE_Pos)            /*!<MCM PWMINTEN: FLTIE Mask */

#define MCM_PWMINTEN_OIE_Pos         9                                            /*!<MCM PWMINTEN: OIE Position */
#define MCM_PWMINTEN_OIE_Msk         (0x1UL << MCM_PWMINTEN_OIE_Pos)              /*!<MCM PWMINTEN: OIE Mask */

#define MCM_PWMINTEN_OSTDIE_Pos      10                                           /*!<MCM PWMINTEN: OSTDIE Position */
#define MCM_PWMINTEN_OSTDIE_Msk      (0x1UL << MCM_PWMINTEN_OSTDIE_Pos)           /*!<MCM PWMINTEN: OSTDIE Mask */

#define MCM_PWMINTEN_Msk_ALL         (MCM_PWMINTEN_PTUD0IE_Msk     |\
                                      MCM_PWMINTEN_PTDD0IE_Msk     |\
                                      MCM_PWMINTEN_PTUD1IE_Msk     |\
                                      MCM_PWMINTEN_PTDD1IE_Msk     |\
                                      MCM_PWMINTEN_PTUD2IE_Msk     |\
                                      MCM_PWMINTEN_PTDD2IE_Msk     |\
                                      MCM_PWMINTEN_PWMZIE_Msk      |\
                                      MCM_PWMINTEN_PWMPIE_Msk      |\
                                      MCM_PWMINTEN_FLTIE_Msk       |\
                                      MCM_PWMINTEN_OIE_Msk         |\
                                      MCM_PWMINTEN_OSTDIE_Msk      )

#define MCM_PWMINTF_PTUD0IF_Pos      0                                            /*!<MCM PWMINTF: PTUD0IF Position */
#define MCM_PWMINTF_PTUD0IF_Msk      (0x1UL /*<< MCM_PWMINTF_PTUD0IF_Pos*/)       /*!<MCM PWMINTF: PTUD0IF Mask */

#define MCM_PWMINTF_PTDD0IF_Pos      1                                            /*!<MCM PWMINTF: PTDD0IF Position */
#define MCM_PWMINTF_PTDD0IF_Msk      (0x1UL << MCM_PWMINTF_PTDD0IF_Pos)           /*!<MCM PWMINTF: PTDD0IF Mask */

#define MCM_PWMINTF_PTUD1IF_Pos      2                                            /*!<MCM PWMINTF: PTUD1IF Position */
#define MCM_PWMINTF_PTUD1IF_Msk      (0x1UL << MCM_PWMINTF_PTUD1IF_Pos)           /*!<MCM PWMINTF: PTUD1IF Mask */

#define MCM_PWMINTF_PTDD1IF_Pos      3                                            /*!<MCM PWMINTF: PTDD1IF Position */
#define MCM_PWMINTF_PTDD1IF_Msk      (0x1UL << MCM_PWMINTF_PTDD1IF_Pos)           /*!<MCM PWMINTF: PTDD1IF Mask */

#define MCM_PWMINTF_PTUD2IF_Pos      4                                            /*!<MCM PWMINTF: PTUD2IF Position */
#define MCM_PWMINTF_PTUD2IF_Msk      (0x1UL << MCM_PWMINTF_PTUD2IF_Pos)           /*!<MCM PWMINTF: PTUD2IF Mask */

#define MCM_PWMINTF_PTDD2IF_Pos      5                                            /*!<MCM PWMINTF: PTDD2IF Position */
#define MCM_PWMINTF_PTDD2IF_Msk      (0x1UL << MCM_PWMINTF_PTDD2IF_Pos)           /*!<MCM PWMINTF: PTDD2IF Mask */

#define MCM_PWMINTF_PWMZIF_Pos       6                                            /*!<MCM PWMINTF: PWMZIF Position */
#define MCM_PWMINTF_PWMZIF_Msk       (0x1UL << MCM_PWMINTF_PWMZIF_Pos)            /*!<MCM PWMINTF: PWMZIF Mask */

#define MCM_PWMINTF_PWMPIF_Pos       7                                            /*!<MCM PWMINTF: PWMPIF Position */
#define MCM_PWMINTF_PWMPIF_Msk       (0x1UL << MCM_PWMINTF_PWMPIF_Pos)            /*!<MCM PWMINTF: PWMPIF Mask */

#define MCM_PWMINTF_FLTIF_Pos        8                                            /*!<MCM PWMINTF: FLTIF Position */
#define MCM_PWMINTF_FLTIF_Msk        (0x1UL << MCM_PWMINTF_FLTIF_Pos)             /*!<MCM PWMINTF: FLTIF Mask */

#define MCM_PWMINTF_OSF_Pos          9                                            /*!<MCM PWMINTF: OSF Position */
#define MCM_PWMINTF_OSF_Msk          (0x1UL << MCM_PWMINTF_OSF_Pos)               /*!<MCM PWMINTF: OSF Mask */

#define MCM_PWMINTF_OSTDF_Pos        10                                           /*!<MCM PWMINTF: OSTDF Position */
#define MCM_PWMINTF_OSTDF_Msk        (0x1UL << MCM_PWMINTF_OSTDF_Pos)             /*!<MCM PWMINTF: OSTDF Mask */

#define MCM_PWMINTF_SC1STAT_Pos      11                                           /*!<MCM PWMINTF: SC1STAT Position */
#define MCM_PWMINTF_SC1STAT_Msk      (0x1UL << MCM_PWMINTF_SC1STAT_Pos)           /*!<MCM PWMINTF: SC1STAT Mask */

#define MCM_PWMINTF_SC2STAT_Pos      12                                           /*!<MCM PWMINTF: SC2STAT Position */
#define MCM_PWMINTF_SC2STAT_Msk      (0x1UL << MCM_PWMINTF_SC2STAT_Pos)           /*!<MCM PWMINTF: SC2STAT Mask */

#define MCM_PWMINTF_SC3STAT_Pos      13                                           /*!<MCM PWMINTF: SC3STAT Position */
#define MCM_PWMINTF_SC3STAT_Msk      (0x1UL << MCM_PWMINTF_SC3STAT_Pos)           /*!<MCM PWMINTF: SC3STAT Mask */

#define MCM_PWMINTF_PTUD0IFC_Pos     16                                           /*!<MCM PWMINTF: PTUD0IFC Position */
#define MCM_PWMINTF_PTUD0IFC_Msk     (0x1UL << MCM_PWMINTF_PTUD0IFC_Pos)          /*!<MCM PWMINTF: PTUD0IFC Mask */

#define MCM_PWMINTF_PTDD0IFC_Pos     17                                           /*!<MCM PWMINTF: PTDD0IFC Position */
#define MCM_PWMINTF_PTDD0IFC_Msk     (0x1UL << MCM_PWMINTF_PTDD0IFC_Pos)          /*!<MCM PWMINTF: PTDD0IFC Mask */

#define MCM_PWMINTF_PTUD1IFC_Pos     18                                           /*!<MCM PWMINTF: PTUD1IFC Position */
#define MCM_PWMINTF_PTUD1IFC_Msk     (0x1UL << MCM_PWMINTF_PTUD1IFC_Pos)          /*!<MCM PWMINTF: PTUD1IFC Mask */

#define MCM_PWMINTF_PTDD1IFC_Pos     19                                           /*!<MCM PWMINTF: PTDD1IFC Position */
#define MCM_PWMINTF_PTDD1IFC_Msk     (0x1UL << MCM_PWMINTF_PTDD1IFC_Pos)          /*!<MCM PWMINTF: PTDD1IFC Mask */

#define MCM_PWMINTF_PTUD2IFC_Pos     20                                           /*!<MCM PWMINTF: PTUD2IFC Position */
#define MCM_PWMINTF_PTUD2IFC_Msk     (0x1UL << MCM_PWMINTF_PTUD2IFC_Pos)          /*!<MCM PWMINTF: PTUD2IFC Mask */

#define MCM_PWMINTF_PTDD2IFC_Pos     21                                           /*!<MCM PWMINTF: PTDD2IFC Position */
#define MCM_PWMINTF_PTDD2IFC_Msk     (0x1UL << MCM_PWMINTF_PTDD2IFC_Pos)          /*!<MCM PWMINTF: PTDD2IFC Mask */

#define MCM_PWMINTF_PWMZIFC_Pos      22                                           /*!<MCM PWMINTF: PWMZIFC Position */
#define MCM_PWMINTF_PWMZIFC_Msk      (0x1UL << MCM_PWMINTF_PWMZIFC_Pos)           /*!<MCM PWMINTF: PWMZIFC Mask */

#define MCM_PWMINTF_PWMPIFC_Pos      23                                           /*!<MCM PWMINTF: PWMPIFC Position */
#define MCM_PWMINTF_PWMPIFC_Msk      (0x1UL << MCM_PWMINTF_PWMPIFC_Pos)           /*!<MCM PWMINTF: PWMPIFC Mask */

#define MCM_PWMINTF_FLTIFC_Pos       24                                           /*!<MCM PWMINTF: FLTIFC Position */
#define MCM_PWMINTF_FLTIFC_Msk       (0x1UL << MCM_PWMINTF_FLTIFC_Pos)            /*!<MCM PWMINTF: FLTIFC Mask */

#define MCM_PWMINTF_OSFC_Pos         25                                           /*!<MCM PWMINTF: OSFC Position */
#define MCM_PWMINTF_OSFC_Msk         (0x1UL << MCM_PWMINTF_OSFC_Pos)              /*!<MCM PWMINTF: OSFC Mask */

#define MCM_PWMINTF_OSTDFC_Pos       26                                           /*!<MCM PWMINTF: OSTDFC Position */
#define MCM_PWMINTF_OSTDFC_Msk       (0x1UL << MCM_PWMINTF_OSTDFC_Pos)            /*!<MCM PWMINTF: OSTDFC Mask */

#define MCM_PWMINTF_SC1STATC_Pos     27                                           /*!<MCM PWMINTF: SC1STATC Position */
#define MCM_PWMINTF_SC1STATC_Msk     (0x1UL << MCM_PWMINTF_SC1STATC_Pos)          /*!<MCM PWMINTF: SC1STATC Mask */

#define MCM_PWMINTF_SC2STATC_Pos     28                                           /*!<MCM PWMINTF: SC2STATC Position */
#define MCM_PWMINTF_SC2STATC_Msk     (0x1UL << MCM_PWMINTF_SC2STATC_Pos)          /*!<MCM PWMINTF: SC2STATC Mask */

#define MCM_PWMINTF_SC3STATC_Pos     29                                           /*!<MCM PWMINTF: SC3STATC Position */
#define MCM_PWMINTF_SC3STATC_Msk     (0x1UL << MCM_PWMINTF_SC3STATC_Pos)          /*!<MCM PWMINTF: SC3STATC Mask */

#define MCM_PWMINTF_Msk_ALL          (MCM_PWMINTF_PTUD0IF_Msk      |\
                                      MCM_PWMINTF_PTDD0IF_Msk      |\
                                      MCM_PWMINTF_PTUD1IF_Msk      |\
                                      MCM_PWMINTF_PTDD1IF_Msk      |\
                                      MCM_PWMINTF_PTUD2IF_Msk      |\
                                      MCM_PWMINTF_PTDD2IF_Msk      |\
                                      MCM_PWMINTF_PWMZIF_Msk       |\
                                      MCM_PWMINTF_PWMPIF_Msk       |\
                                      MCM_PWMINTF_FLTIF_Msk        |\
                                      MCM_PWMINTF_OSF_Msk          |\
                                      MCM_PWMINTF_OSTDF_Msk        |\
                                      MCM_PWMINTF_SC1STAT_Msk      |\
                                      MCM_PWMINTF_SC2STAT_Msk      |\
                                      MCM_PWMINTF_SC3STAT_Msk      |\
                                      MCM_PWMINTF_PTUD0IFC_Msk     |\
                                      MCM_PWMINTF_PTDD0IFC_Msk     |\
                                      MCM_PWMINTF_PTUD1IFC_Msk     |\
                                      MCM_PWMINTF_PTDD1IFC_Msk     |\
                                      MCM_PWMINTF_PTUD2IFC_Msk     |\
                                      MCM_PWMINTF_PTDD2IFC_Msk     |\
                                      MCM_PWMINTF_PWMZIFC_Msk      |\
                                      MCM_PWMINTF_PWMPIFC_Msk      |\
                                      MCM_PWMINTF_FLTIFC_Msk       |\
                                      MCM_PWMINTF_OSFC_Msk         |\
                                      MCM_PWMINTF_OSTDFC_Msk       |\
                                      MCM_PWMINTF_SC1STATC_Msk     |\
                                      MCM_PWMINTF_SC2STATC_Msk     |\
                                      MCM_PWMINTF_SC3STATC_Msk     )

#define MCM_PWMRLDEN_PWMRLDEN_Pos    0                                            /*!<MCM PWMRLDEN: PWMRLDEN Position */
#define MCM_PWMRLDEN_PWMRLDEN_Msk    (0xFFUL /*<< MCM_PWMRLDEN_PWMRLDEN_Pos*/)    /*!<MCM PWMRLDEN: PWMRLDEN Mask */

#define MCM_PWMRLDEN_Msk_ALL         (MCM_PWMRLDEN_PWMRLDEN_Msk   )

#define MCM_PSCON_SHIFTRUN_Pos       0                                            /*!<MCM PSCON: SHIFTRUN Position */
#define MCM_PSCON_SHIFTRUN_Msk       (0x1UL /*<< MCM_PSCON_SHIFTRUN_Pos*/)        /*!<MCM PSCON: SHIFTRUN Mask */

#define MCM_PSCON_SECTOR_Pos         1                                            /*!<MCM PSCON: SECTOR Position */
#define MCM_PSCON_SECTOR_Msk         (0x7UL << MCM_PSCON_SECTOR_Pos)              /*!<MCM PSCON: SECTOR Mask */

#define MCM_PSCON_MAXSELECT_Pos      4                                            /*!<MCM PSCON: MAXSELECT Position */
#define MCM_PSCON_MAXSELECT_Msk      (0x3UL << MCM_PSCON_MAXSELECT_Pos)           /*!<MCM PSCON: MAXSELECT Mask */

#define MCM_PSCON_MINSELECT_Pos      6                                            /*!<MCM PSCON: MINSELECT Position */
#define MCM_PSCON_MINSELECT_Msk      (0x3UL << MCM_PSCON_MINSELECT_Pos)           /*!<MCM PSCON: MINSELECT Mask */

#define MCM_PSCON_STATRUN_Pos        8                                            /*!<MCM PSCON: STATRUN Position */
#define MCM_PSCON_STATRUN_Msk        (0x1UL << MCM_PSCON_STATRUN_Pos)             /*!<MCM PSCON: STATRUN Mask */

#define MCM_PSCON_Msk_ALL            (MCM_PSCON_SHIFTRUN_Msk       |\
                                      MCM_PSCON_SECTOR_Msk         |\
                                      MCM_PSCON_MAXSELECT_Msk      |\
                                      MCM_PSCON_MINSELECT_Msk      |\
                                      MCM_PSCON_STATRUN_Msk        )

#define MCM_PWMDMAX_PWMDMAX_Pos      0                                            /*!<MCM PWMDMAX: PWMDMAX Position */
#define MCM_PWMDMAX_PWMDMAX_Msk      (0xFFFFUL /*<< MCM_PWMDMAX_PWMDMAX_Pos*/)    /*!<MCM PWMDMAX: PWMDMAX Mask */

#define MCM_PWMDMAX_Msk_ALL          (MCM_PWMDMAX_PWMDMAX_Msk     )

#define MCM_PWMDMIN_PWMDMIN_Pos      0                                            /*!<MCM PWMDMIN: PWMDMIN Position */
#define MCM_PWMDMIN_PWMDMIN_Msk      (0xFFFFUL /*<< MCM_PWMDMIN_PWMDMIN_Pos*/)    /*!<MCM PWMDMIN: PWMDMIN Mask */

#define MCM_PWMDMIN_Msk_ALL          (MCM_PWMDMIN_PWMDMIN_Msk     )

#define MCM_PWMDCMP1_PWMDCMP1_Pos    0                                            /*!<MCM PWMDCMP1: PWMDCMP1 Position */
#define MCM_PWMDCMP1_PWMDCMP1_Msk    (0xFFFFUL /*<< MCM_PWMDCMP1_PWMDCMP1_Pos*/)  /*!<MCM PWMDCMP1: PWMDCMP1 Mask */

#define MCM_PWMDCMP1_Msk_ALL         (MCM_PWMDCMP1_PWMDCMP1_Msk   )

#define MCM_PWMDCMP2_PWMDCMP2_Pos    0                                            /*!<MCM PWMDCMP2: PWMDCMP2 Position */
#define MCM_PWMDCMP2_PWMDCMP2_Msk    (0xFFFFUL /*<< MCM_PWMDCMP2_PWMDCMP2_Pos*/)  /*!<MCM PWMDCMP2: PWMDCMP2 Mask */

#define MCM_PWMDCMP2_Msk_ALL         (MCM_PWMDCMP2_PWMDCMP2_Msk   )

#define MCM_SC1CON_SCTIME_Pos        0                                            /*!<MCM SC1CON: SCTIME Position */
#define MCM_SC1CON_SCTIME_Msk        (0xFUL /*<< MCM_SC1CON_SCTIME_Pos*/)         /*!<MCM SC1CON: SCTIME Mask */

#define MCM_SC1CON_SCDEB_Pos         4                                            /*!<MCM SC1CON: SCDEB Position */
#define MCM_SC1CON_SCDEB_Msk         (0xFUL << MCM_SC1CON_SCDEB_Pos)              /*!<MCM SC1CON: SCDEB Mask */

#define MCM_SC1CON_SCPWM0EN_Pos      8                                            /*!<MCM SC1CON: SCPWM0EN Position */
#define MCM_SC1CON_SCPWM0EN_Msk      (0x1UL << MCM_SC1CON_SCPWM0EN_Pos)           /*!<MCM SC1CON: SCPWM0EN Mask */

#define MCM_SC1CON_SCPWM1EN_Pos      9                                            /*!<MCM SC1CON: SCPWM1EN Position */
#define MCM_SC1CON_SCPWM1EN_Msk      (0x1UL << MCM_SC1CON_SCPWM1EN_Pos)           /*!<MCM SC1CON: SCPWM1EN Mask */

#define MCM_SC1CON_SCPWM2EN_Pos      10                                           /*!<MCM SC1CON: SCPWM2EN Position */
#define MCM_SC1CON_SCPWM2EN_Msk      (0x1UL << MCM_SC1CON_SCPWM2EN_Pos)           /*!<MCM SC1CON: SCPWM2EN Mask */

#define MCM_SC1CON_SCS_Pos           11                                           /*!<MCM SC1CON: SCS Position */
#define MCM_SC1CON_SCS_Msk           (0x1UL << MCM_SC1CON_SCS_Pos)                /*!<MCM SC1CON: SCS Mask */

#define MCM_SC1CON_SCEN_Pos          12                                           /*!<MCM SC1CON: SCEN Position */
#define MCM_SC1CON_SCEN_Msk          (0x1UL << MCM_SC1CON_SCEN_Pos)               /*!<MCM SC1CON: SCEN Mask */

#define MCM_SC1CON_Msk_ALL           (MCM_SC1CON_SCTIME_Msk        |\
                                      MCM_SC1CON_SCDEB_Msk         |\
                                      MCM_SC1CON_SCPWM0EN_Msk      |\
                                      MCM_SC1CON_SCPWM1EN_Msk      |\
                                      MCM_SC1CON_SCPWM2EN_Msk      |\
                                      MCM_SC1CON_SCS_Msk           |\
                                      MCM_SC1CON_SCEN_Msk          )

#define MCM_SC2CON_SCTIME_Pos        0                                            /*!<MCM SC2CON: SCTIME Position */
#define MCM_SC2CON_SCTIME_Msk        (0xFUL /*<< MCM_SC2CON_SCTIME_Pos*/)         /*!<MCM SC2CON: SCTIME Mask */

#define MCM_SC2CON_SCDEB_Pos         4                                            /*!<MCM SC2CON: SCDEB Position */
#define MCM_SC2CON_SCDEB_Msk         (0xFUL << MCM_SC2CON_SCDEB_Pos)              /*!<MCM SC2CON: SCDEB Mask */

#define MCM_SC2CON_SCPWM0EN_Pos      8                                            /*!<MCM SC2CON: SCPWM0EN Position */
#define MCM_SC2CON_SCPWM0EN_Msk      (0x1UL << MCM_SC2CON_SCPWM0EN_Pos)           /*!<MCM SC2CON: SCPWM0EN Mask */

#define MCM_SC2CON_SCPWM1EN_Pos      9                                            /*!<MCM SC2CON: SCPWM1EN Position */
#define MCM_SC2CON_SCPWM1EN_Msk      (0x1UL << MCM_SC2CON_SCPWM1EN_Pos)           /*!<MCM SC2CON: SCPWM1EN Mask */

#define MCM_SC2CON_SCPWM2EN_Pos      10                                           /*!<MCM SC2CON: SCPWM2EN Position */
#define MCM_SC2CON_SCPWM2EN_Msk      (0x1UL << MCM_SC2CON_SCPWM2EN_Pos)           /*!<MCM SC2CON: SCPWM2EN Mask */

#define MCM_SC2CON_SCS_Pos           11                                           /*!<MCM SC2CON: SCS Position */
#define MCM_SC2CON_SCS_Msk           (0x1UL << MCM_SC2CON_SCS_Pos)                /*!<MCM SC2CON: SCS Mask */

#define MCM_SC2CON_SCEN_Pos          12                                           /*!<MCM SC2CON: SCEN Position */
#define MCM_SC2CON_SCEN_Msk          (0x1UL << MCM_SC2CON_SCEN_Pos)               /*!<MCM SC2CON: SCEN Mask */

#define MCM_SC2CON_Msk_ALL           (MCM_SC2CON_SCTIME_Msk        |\
                                      MCM_SC2CON_SCDEB_Msk         |\
                                      MCM_SC2CON_SCPWM0EN_Msk      |\
                                      MCM_SC2CON_SCPWM1EN_Msk      |\
                                      MCM_SC2CON_SCPWM2EN_Msk      |\
                                      MCM_SC2CON_SCS_Msk           |\
                                      MCM_SC2CON_SCEN_Msk          )

#define MCM_SC3CON_SCTIME_Pos        0                                            /*!<MCM SC3CON: SCTIME Position */
#define MCM_SC3CON_SCTIME_Msk        (0xFUL /*<< MCM_SC3CON_SCTIME_Pos*/)         /*!<MCM SC3CON: SCTIME Mask */

#define MCM_SC3CON_SCDEB_Pos         4                                            /*!<MCM SC3CON: SCDEB Position */
#define MCM_SC3CON_SCDEB_Msk         (0xFUL << MCM_SC3CON_SCDEB_Pos)              /*!<MCM SC3CON: SCDEB Mask */

#define MCM_SC3CON_SCPWM0EN_Pos      8                                            /*!<MCM SC3CON: SCPWM0EN Position */
#define MCM_SC3CON_SCPWM0EN_Msk      (0x1UL << MCM_SC3CON_SCPWM0EN_Pos)           /*!<MCM SC3CON: SCPWM0EN Mask */

#define MCM_SC3CON_SCPWM1EN_Pos      9                                            /*!<MCM SC3CON: SCPWM1EN Position */
#define MCM_SC3CON_SCPWM1EN_Msk      (0x1UL << MCM_SC3CON_SCPWM1EN_Pos)           /*!<MCM SC3CON: SCPWM1EN Mask */

#define MCM_SC3CON_SCPWM2EN_Pos      10                                           /*!<MCM SC3CON: SCPWM2EN Position */
#define MCM_SC3CON_SCPWM2EN_Msk      (0x1UL << MCM_SC3CON_SCPWM2EN_Pos)           /*!<MCM SC3CON: SCPWM2EN Mask */

#define MCM_SC3CON_SCS_Pos           11                                           /*!<MCM SC3CON: SCS Position */
#define MCM_SC3CON_SCS_Msk           (0x1UL << MCM_SC3CON_SCS_Pos)                /*!<MCM SC3CON: SCS Mask */

#define MCM_SC3CON_SCEN_Pos          12                                           /*!<MCM SC3CON: SCEN Position */
#define MCM_SC3CON_SCEN_Msk          (0x1UL << MCM_SC3CON_SCEN_Pos)               /*!<MCM SC3CON: SCEN Mask */

#define MCM_SC3CON_Msk_ALL           (MCM_SC3CON_SCTIME_Msk        |\
                                      MCM_SC3CON_SCDEB_Msk         |\
                                      MCM_SC3CON_SCPWM0EN_Msk      |\
                                      MCM_SC3CON_SCPWM1EN_Msk      |\
                                      MCM_SC3CON_SCPWM2EN_Msk      |\
                                      MCM_SC3CON_SCS_Msk           |\
                                      MCM_SC3CON_SCEN_Msk          )

#define MCM_FLTWEN_FLTWEN_Pos        0                                            /*!<MCM FLTWEN: FLTWEN Position */
#define MCM_FLTWEN_FLTWEN_Msk        (0xFFFFUL /*<< MCM_FLTWEN_FLTWEN_Pos*/)      /*!<MCM FLTWEN: FLTWEN Mask */

#define MCM_FLTWEN_Msk_ALL           (MCM_FLTWEN_FLTWEN_Msk       )

#define GPT_GTSTR_CST0_Pos           0                                            /*!<GPT GTSTR: CST0 Position */
#define GPT_GTSTR_CST0_Msk           (0x1UL /*<< GPT_GTSTR_CST0_Pos*/)            /*!<GPT GTSTR: CST0 Mask */

#define GPT_GTSTR_CST1_Pos           1                                            /*!<GPT GTSTR: CST1 Position */
#define GPT_GTSTR_CST1_Msk           (0x1UL << GPT_GTSTR_CST1_Pos)                /*!<GPT GTSTR: CST1 Mask */

#define GPT_GTSTR_CST2_Pos           2                                            /*!<GPT GTSTR: CST2 Position */
#define GPT_GTSTR_CST2_Msk           (0x1UL << GPT_GTSTR_CST2_Pos)                /*!<GPT GTSTR: CST2 Mask */

#define GPT_GTSTR_CST3_Pos           3                                            /*!<GPT GTSTR: CST3 Position */
#define GPT_GTSTR_CST3_Msk           (0x1UL << GPT_GTSTR_CST3_Pos)                /*!<GPT GTSTR: CST3 Mask */

#define GPT_GTSTR_Msk_ALL            (GPT_GTSTR_CST0_Msk           |\
                                      GPT_GTSTR_CST1_Msk           |\
                                      GPT_GTSTR_CST2_Msk           |\
                                      GPT_GTSTR_CST3_Msk           )

#define GPT_GTETINT_ETIPEN_Pos       0                                            /*!<GPT GTETINT: ETIPEN Position */
#define GPT_GTETINT_ETIPEN_Msk       (0x1UL /*<< GPT_GTETINT_ETIPEN_Pos*/)        /*!<GPT GTETINT: ETIPEN Mask */

#define GPT_GTETINT_ETINEN_Pos       1                                            /*!<GPT GTETINT: ETINEN Position */
#define GPT_GTETINT_ETINEN_Msk       (0x1UL << GPT_GTETINT_ETINEN_Pos)            /*!<GPT GTETINT: ETINEN Mask */

#define GPT_GTETINT_ETPDMA_Pos       2                                            /*!<GPT GTETINT: ETPDMA Position */
#define GPT_GTETINT_ETPDMA_Msk       (0x1UL << GPT_GTETINT_ETPDMA_Pos)            /*!<GPT GTETINT: ETPDMA Mask */

#define GPT_GTETINT_ETNDMA_Pos       3                                            /*!<GPT GTETINT: ETNDMA Position */
#define GPT_GTETINT_ETNDMA_Msk       (0x1UL << GPT_GTETINT_ETNDMA_Pos)            /*!<GPT GTETINT: ETNDMA Mask */

#define GPT_GTETINT_SN_Pos           4                                            /*!<GPT GTETINT: SN Position */
#define GPT_GTETINT_SN_Msk           (0x3UL << GPT_GTETINT_SN_Pos)                /*!<GPT GTETINT: SN Mask */

#define GPT_GTETINT_PS_Pos           6                                            /*!<GPT GTETINT: PS Position */
#define GPT_GTETINT_PS_Msk           (0x3UL << GPT_GTETINT_PS_Pos)                /*!<GPT GTETINT: PS Mask */

#define GPT_GTETINT_Msk_ALL          (GPT_GTETINT_ETIPEN_Msk       |\
                                      GPT_GTETINT_ETINEN_Msk       |\
                                      GPT_GTETINT_ETPDMA_Msk       |\
                                      GPT_GTETINT_ETNDMA_Msk       |\
                                      GPT_GTETINT_SN_Msk           |\
                                      GPT_GTETINT_PS_Msk           )

#define GPT_GTPOECR_GPT0ABZE0_Pos    0                                            /*!<GPT GTPOECR: GPT0ABZE0 Position */
#define GPT_GTPOECR_GPT0ABZE0_Msk    (0x1UL /*<< GPT_GTPOECR_GPT0ABZE0_Pos*/)     /*!<GPT GTPOECR: GPT0ABZE0 Mask */

#define GPT_GTPOECR_GPT1ABZE0_Pos    1                                            /*!<GPT GTPOECR: GPT1ABZE0 Position */
#define GPT_GTPOECR_GPT1ABZE0_Msk    (0x1UL << GPT_GTPOECR_GPT1ABZE0_Pos)         /*!<GPT GTPOECR: GPT1ABZE0 Mask */

#define GPT_GTPOECR_GPT2ABZE0_Pos    2                                            /*!<GPT GTPOECR: GPT2ABZE0 Position */
#define GPT_GTPOECR_GPT2ABZE0_Msk    (0x1UL << GPT_GTPOECR_GPT2ABZE0_Pos)         /*!<GPT GTPOECR: GPT2ABZE0 Mask */

#define GPT_GTPOECR_GPT3ABZE0_Pos    3                                            /*!<GPT GTPOECR: GPT3ABZE0 Position */
#define GPT_GTPOECR_GPT3ABZE0_Msk    (0x1UL << GPT_GTPOECR_GPT3ABZE0_Pos)         /*!<GPT GTPOECR: GPT3ABZE0 Mask */

#define GPT_GTPOECR_POEM_Pos         4                                            /*!<GPT GTPOECR: POEM Position */
#define GPT_GTPOECR_POEM_Msk         (0x3UL << GPT_GTPOECR_POEM_Pos)              /*!<GPT GTPOECR: POEM Mask */

#define GPT_GTPOECR_POEIE_Pos        7                                            /*!<GPT GTPOECR: POEIE Position */
#define GPT_GTPOECR_POEIE_Msk        (0x1UL << GPT_GTPOECR_POEIE_Pos)             /*!<GPT GTPOECR: POEIE Mask */

#define GPT_GTPOECR_Msk_ALL          (GPT_GTPOECR_GPT0ABZE0_Msk    |\
                                      GPT_GTPOECR_GPT1ABZE0_Msk    |\
                                      GPT_GTPOECR_GPT2ABZE0_Msk    |\
                                      GPT_GTPOECR_GPT3ABZE0_Msk    |\
                                      GPT_GTPOECR_POEM_Msk         |\
                                      GPT_GTPOECR_POEIE_Msk        )

#define GPT_GTPRWEN_GTPRWEN_Pos      0                                            /*!<GPT GTPRWEN: GTPRWEN Position */
#define GPT_GTPRWEN_GTPRWEN_Msk      (0xFFFFUL /*<< GPT_GTPRWEN_GTPRWEN_Pos*/)    /*!<GPT GTPRWEN: GTPRWEN Mask */

#define GPT_GTPRWEN_Msk_ALL          (GPT_GTPRWEN_GTPRWEN_Msk     )

#define GPT_GTINTF_ETIPF_Pos         0                                            /*!<GPT GTINTF: ETIPF Position */
#define GPT_GTINTF_ETIPF_Msk         (0x1UL /*<< GPT_GTINTF_ETIPF_Pos*/)          /*!<GPT GTINTF: ETIPF Mask */

#define GPT_GTINTF_ETINF_Pos         1                                            /*!<GPT GTINTF: ETINF Position */
#define GPT_GTINTF_ETINF_Msk         (0x1UL << GPT_GTINTF_ETINF_Pos)              /*!<GPT GTINTF: ETINF Mask */

#define GPT_GTINTF_POEIF_Pos         2                                            /*!<GPT GTINTF: POEIF Position */
#define GPT_GTINTF_POEIF_Msk         (0x1UL << GPT_GTINTF_POEIF_Pos)              /*!<GPT GTINTF: POEIF Mask */

#define GPT_GTINTF_ETIPFC_Pos        16                                           /*!<GPT GTINTF: ETIPFC Position */
#define GPT_GTINTF_ETIPFC_Msk        (0x1UL << GPT_GTINTF_ETIPFC_Pos)             /*!<GPT GTINTF: ETIPFC Mask */

#define GPT_GTINTF_ETINFC_Pos        17                                           /*!<GPT GTINTF: ETINFC Position */
#define GPT_GTINTF_ETINFC_Msk        (0x1UL << GPT_GTINTF_ETINFC_Pos)             /*!<GPT GTINTF: ETINFC Mask */

#define GPT_GTINTF_POEIFC_Pos        18                                           /*!<GPT GTINTF: POEIFC Position */
#define GPT_GTINTF_POEIFC_Msk        (0x1UL << GPT_GTINTF_POEIFC_Pos)             /*!<GPT GTINTF: POEIFC Mask */

#define GPT_GTINTF_Msk_ALL           (GPT_GTINTF_ETIPF_Msk         |\
                                      GPT_GTINTF_ETINF_Msk         |\
                                      GPT_GTINTF_POEIF_Msk         |\
                                      GPT_GTINTF_ETIPFC_Msk        |\
                                      GPT_GTINTF_ETINFC_Msk        |\
                                      GPT_GTINTF_POEIFC_Msk        )

#define GPT_GTHCR_CSHW_Pos           0                                            /*!<GPT GTHCR: CSHW Position */
#define GPT_GTHCR_CSHW_Msk           (0x3UL /*<< GPT_GTHCR_CSHW_Pos*/)            /*!<GPT GTHCR: CSHW Mask */

#define GPT_GTHCR_CPHW_Pos           2                                            /*!<GPT GTHCR: CPHW Position */
#define GPT_GTHCR_CPHW_Msk           (0x3UL << GPT_GTHCR_CPHW_Pos)                /*!<GPT GTHCR: CPHW Mask */

#define GPT_GTHCR_CCHW_Pos           4                                            /*!<GPT GTHCR: CCHW Position */
#define GPT_GTHCR_CCHW_Msk           (0x3UL << GPT_GTHCR_CCHW_Pos)                /*!<GPT GTHCR: CCHW Mask */

#define GPT_GTHCR_SYNC_Pos           6                                            /*!<GPT GTHCR: SYNC Position */
#define GPT_GTHCR_SYNC_Msk           (0x3UL << GPT_GTHCR_SYNC_Pos)                /*!<GPT GTHCR: SYNC Mask */

#define GPT_GTHCR_CCSW_Pos           8                                            /*!<GPT GTHCR: CCSW Position */
#define GPT_GTHCR_CCSW_Msk           (0x1UL << GPT_GTHCR_CCSW_Pos)                /*!<GPT GTHCR: CCSW Mask */

#define GPT_GTHCR_Msk_ALL            (GPT_GTHCR_CSHW_Msk           |\
                                      GPT_GTHCR_CPHW_Msk           |\
                                      GPT_GTHCR_CCHW_Msk           |\
                                      GPT_GTHCR_SYNC_Msk           |\
                                      GPT_GTHCR_CCSW_Msk           )

#define GPT_GTDEB_DEB_Pos            0                                            /*!<GPT GTDEB: DEB Position */
#define GPT_GTDEB_DEB_Msk            (0xFUL /*<< GPT_GTDEB_DEB_Pos*/)             /*!<GPT GTDEB: DEB Mask */

#define GPT_GTDEB_INAE_Pos           8                                            /*!<GPT GTDEB: INAE Position */
#define GPT_GTDEB_INAE_Msk           (0x1UL << GPT_GTDEB_INAE_Pos)                /*!<GPT GTDEB: INAE Mask */

#define GPT_GTDEB_INBE_Pos           9                                            /*!<GPT GTDEB: INBE Position */
#define GPT_GTDEB_INBE_Msk           (0x1UL << GPT_GTDEB_INBE_Pos)                /*!<GPT GTDEB: INBE Mask */

#define GPT_GTDEB_Msk_ALL            (GPT_GTDEB_DEB_Msk            |\
                                      GPT_GTDEB_INAE_Msk           |\
                                      GPT_GTDEB_INBE_Msk           )

#define GPT_GTWP_GTWP_Pos            0                                            /*!<GPT GTWP: GTWP Position */
#define GPT_GTWP_GTWP_Msk            (0xFFUL /*<< GPT_GTWP_GTWP_Pos*/)            /*!<GPT GTWP: GTWP Mask */

#define GPT_GTWP_Msk_ALL             (GPT_GTWP_GTWP_Msk           )

#define GPT_GTBDR_BDCCR_Pos          0                                            /*!<GPT GTBDR: BDCCR Position */
#define GPT_GTBDR_BDCCR_Msk          (0x1UL /*<< GPT_GTBDR_BDCCR_Pos*/)           /*!<GPT GTBDR: BDCCR Mask */

#define GPT_GTBDR_BDPR_Pos           1                                            /*!<GPT GTBDR: BDPR Position */
#define GPT_GTBDR_BDPR_Msk           (0x1UL << GPT_GTBDR_BDPR_Pos)                /*!<GPT GTBDR: BDPR Mask */

#define GPT_GTBDR_BDADTR_Pos         2                                            /*!<GPT GTBDR: BDADTR Position */
#define GPT_GTBDR_BDADTR_Msk         (0x1UL << GPT_GTBDR_BDADTR_Pos)              /*!<GPT GTBDR: BDADTR Mask */

#define GPT_GTBDR_BDDV_Pos           3                                            /*!<GPT GTBDR: BDDV Position */
#define GPT_GTBDR_BDDV_Msk           (0x1UL << GPT_GTBDR_BDDV_Pos)                /*!<GPT GTBDR: BDDV Mask */

#define GPT_GTBDR_Msk_ALL            (GPT_GTBDR_BDCCR_Msk          |\
                                      GPT_GTBDR_BDPR_Msk           |\
                                      GPT_GTBDR_BDADTR_Msk         |\
                                      GPT_GTBDR_BDDV_Msk           )

#define GPT_GTIOR_GTIOA_Pos          0                                            /*!<GPT GTIOR: GTIOA Position */
#define GPT_GTIOR_GTIOA_Msk          (0x3FUL /*<< GPT_GTIOR_GTIOA_Pos*/)          /*!<GPT GTIOR: GTIOA Mask */

#define GPT_GTIOR_OADFLT_Pos         6                                            /*!<GPT GTIOR: OADFLT Position */
#define GPT_GTIOR_OADFLT_Msk         (0x1UL << GPT_GTIOR_OADFLT_Pos)              /*!<GPT GTIOR: OADFLT Mask */

#define GPT_GTIOR_OAHLD_Pos          7                                            /*!<GPT GTIOR: OAHLD Position */
#define GPT_GTIOR_OAHLD_Msk          (0x1UL << GPT_GTIOR_OAHLD_Pos)               /*!<GPT GTIOR: OAHLD Mask */

#define GPT_GTIOR_GTIOB_Pos          8                                            /*!<GPT GTIOR: GTIOB Position */
#define GPT_GTIOR_GTIOB_Msk          (0x3FUL << GPT_GTIOR_GTIOB_Pos)              /*!<GPT GTIOR: GTIOB Mask */

#define GPT_GTIOR_OBDFLT_Pos         14                                           /*!<GPT GTIOR: OBDFLT Position */
#define GPT_GTIOR_OBDFLT_Msk         (0x1UL << GPT_GTIOR_OBDFLT_Pos)              /*!<GPT GTIOR: OBDFLT Mask */

#define GPT_GTIOR_OBHLD_Pos          15                                           /*!<GPT GTIOR: OBHLD Position */
#define GPT_GTIOR_OBHLD_Msk          (0x1UL << GPT_GTIOR_OBHLD_Pos)               /*!<GPT GTIOR: OBHLD Mask */

#define GPT_GTIOR_Msk_ALL            (GPT_GTIOR_GTIOA_Msk          |\
                                      GPT_GTIOR_OADFLT_Msk         |\
                                      GPT_GTIOR_OAHLD_Msk          |\
                                      GPT_GTIOR_GTIOB_Msk          |\
                                      GPT_GTIOR_OBDFLT_Msk         |\
                                      GPT_GTIOR_OBHLD_Msk          )

#define GPT_GTINTAD_GTINTA_Pos       0                                            /*!<GPT GTINTAD: GTINTA Position */
#define GPT_GTINTAD_GTINTA_Msk       (0x1UL /*<< GPT_GTINTAD_GTINTA_Pos*/)        /*!<GPT GTINTAD: GTINTA Mask */

#define GPT_GTINTAD_GTINTB_Pos       1                                            /*!<GPT GTINTAD: GTINTB Position */
#define GPT_GTINTAD_GTINTB_Msk       (0x1UL << GPT_GTINTAD_GTINTB_Pos)            /*!<GPT GTINTAD: GTINTB Mask */

#define GPT_GTINTAD_GTINTPR_Pos      2                                            /*!<GPT GTINTAD: GTINTPR Position */
#define GPT_GTINTAD_GTINTPR_Msk      (0x3UL << GPT_GTINTAD_GTINTPR_Pos)           /*!<GPT GTINTAD: GTINTPR Mask */

#define GPT_GTINTAD_EINT_Pos         4                                            /*!<GPT GTINTAD: EINT Position */
#define GPT_GTINTAD_EINT_Msk         (0x1UL << GPT_GTINTAD_EINT_Pos)              /*!<GPT GTINTAD: EINT Mask */

#define GPT_GTINTAD_OINT_Pos         5                                            /*!<GPT GTINTAD: OINT Position */
#define GPT_GTINTAD_OINT_Msk         (0x1UL << GPT_GTINTAD_OINT_Pos)              /*!<GPT GTINTAD: OINT Mask */

#define GPT_GTINTAD_ADTRAUEN_Pos     8                                            /*!<GPT GTINTAD: ADTRAUEN Position */
#define GPT_GTINTAD_ADTRAUEN_Msk     (0x1UL << GPT_GTINTAD_ADTRAUEN_Pos)          /*!<GPT GTINTAD: ADTRAUEN Mask */

#define GPT_GTINTAD_ADTRADEN_Pos     9                                            /*!<GPT GTINTAD: ADTRADEN Position */
#define GPT_GTINTAD_ADTRADEN_Msk     (0x1UL << GPT_GTINTAD_ADTRADEN_Pos)          /*!<GPT GTINTAD: ADTRADEN Mask */

#define GPT_GTINTAD_ADTRBUEN_Pos     10                                           /*!<GPT GTINTAD: ADTRBUEN Position */
#define GPT_GTINTAD_ADTRBUEN_Msk     (0x1UL << GPT_GTINTAD_ADTRBUEN_Pos)          /*!<GPT GTINTAD: ADTRBUEN Mask */

#define GPT_GTINTAD_ADTRBDEN_Pos     11                                           /*!<GPT GTINTAD: ADTRBDEN Position */
#define GPT_GTINTAD_ADTRBDEN_Msk     (0x1UL << GPT_GTINTAD_ADTRBDEN_Pos)          /*!<GPT GTINTAD: ADTRBDEN Mask */

#define GPT_GTINTAD_Msk_ALL          (GPT_GTINTAD_GTINTA_Msk       |\
                                      GPT_GTINTAD_GTINTB_Msk       |\
                                      GPT_GTINTAD_GTINTPR_Msk      |\
                                      GPT_GTINTAD_EINT_Msk         |\
                                      GPT_GTINTAD_OINT_Msk         |\
                                      GPT_GTINTAD_ADTRAUEN_Msk     |\
                                      GPT_GTINTAD_ADTRADEN_Msk     |\
                                      GPT_GTINTAD_ADTRBUEN_Msk     |\
                                      GPT_GTINTAD_ADTRBDEN_Msk     )

#define GPT_GTDMA_TA_Pos             0                                            /*!<GPT GTDMA: TA Position */
#define GPT_GTDMA_TA_Msk             (0x1UL /*<< GPT_GTDMA_TA_Pos*/)              /*!<GPT GTDMA: TA Mask */

#define GPT_GTDMA_TB_Pos             1                                            /*!<GPT GTDMA: TB Position */
#define GPT_GTDMA_TB_Msk             (0x1UL << GPT_GTDMA_TB_Pos)                  /*!<GPT GTDMA: TB Mask */

#define GPT_GTDMA_PR_Pos             2                                            /*!<GPT GTDMA: PR Position */
#define GPT_GTDMA_PR_Msk             (0x3UL << GPT_GTDMA_PR_Pos)                  /*!<GPT GTDMA: PR Mask */

#define GPT_GTDMA_Msk_ALL            (GPT_GTDMA_TA_Msk             |\
                                      GPT_GTDMA_TB_Msk             |\
                                      GPT_GTDMA_PR_Msk             )

#define GPT_GTCR_MD_Pos              0                                            /*!<GPT GTCR: MD Position */
#define GPT_GTCR_MD_Msk              (0x7UL /*<< GPT_GTCR_MD_Pos*/)               /*!<GPT GTCR: MD Mask */

#define GPT_GTCR_TPSC_Pos            8                                            /*!<GPT GTCR: TPSC Position */
#define GPT_GTCR_TPSC_Msk            (0x1UL << GPT_GTCR_TPSC_Pos)                 /*!<GPT GTCR: TPSC Mask */

#define GPT_GTCR_HIZ_Pos             9                                            /*!<GPT GTCR: HIZ Position */
#define GPT_GTCR_HIZ_Msk             (0x1UL << GPT_GTCR_HIZ_Pos)                  /*!<GPT GTCR: HIZ Mask */

#define GPT_GTCR_CCLR_Pos            12                                           /*!<GPT GTCR: CCLR Position */
#define GPT_GTCR_CCLR_Msk            (0x3UL << GPT_GTCR_CCLR_Pos)                 /*!<GPT GTCR: CCLR Mask */

#define GPT_GTCR_Msk_ALL             (GPT_GTCR_MD_Msk              |\
                                      GPT_GTCR_TPSC_Msk            |\
                                      GPT_GTCR_HIZ_Msk             |\
                                      GPT_GTCR_CCLR_Msk            )

#define GPT_GTPSQ_GTPSQ_Pos          0                                            /*!<GPT GTPSQ: GTPSQ Position */
#define GPT_GTPSQ_GTPSQ_Msk          (0xFFFFUL /*<< GPT_GTPSQ_GTPSQ_Pos*/)        /*!<GPT GTPSQ: GTPSQ Mask */

#define GPT_GTPSQ_Msk_ALL            (GPT_GTPSQ_GTPSQ_Msk         )

#define GPT_GTBER_CCRA_Pos           0                                            /*!<GPT GTBER: CCRA Position */
#define GPT_GTBER_CCRA_Msk           (0x3UL /*<< GPT_GTBER_CCRA_Pos*/)            /*!<GPT GTBER: CCRA Mask */

#define GPT_GTBER_CCRB_Pos           2                                            /*!<GPT GTBER: CCRB Position */
#define GPT_GTBER_CCRB_Msk           (0x3UL << GPT_GTBER_CCRB_Pos)                /*!<GPT GTBER: CCRB Mask */

#define GPT_GTBER_PR_Pos             4                                            /*!<GPT GTBER: PR Position */
#define GPT_GTBER_PR_Msk             (0x3UL << GPT_GTBER_PR_Pos)                  /*!<GPT GTBER: PR Mask */

#define GPT_GTBER_CCRSWT_Pos         6                                            /*!<GPT GTBER: CCRSWT Position */
#define GPT_GTBER_CCRSWT_Msk         (0x1UL << GPT_GTBER_CCRSWT_Pos)              /*!<GPT GTBER: CCRSWT Mask */

#define GPT_GTBER_ADTTA_Pos          8                                            /*!<GPT GTBER: ADTTA Position */
#define GPT_GTBER_ADTTA_Msk          (0x3UL << GPT_GTBER_ADTTA_Pos)               /*!<GPT GTBER: ADTTA Mask */

#define GPT_GTBER_ADTDA_Pos          10                                           /*!<GPT GTBER: ADTDA Position */
#define GPT_GTBER_ADTDA_Msk          (0x1UL << GPT_GTBER_ADTDA_Pos)               /*!<GPT GTBER: ADTDA Mask */

#define GPT_GTBER_ADTTB_Pos          12                                           /*!<GPT GTBER: ADTTB Position */
#define GPT_GTBER_ADTTB_Msk          (0x3UL << GPT_GTBER_ADTTB_Pos)               /*!<GPT GTBER: ADTTB Mask */

#define GPT_GTBER_ADTDB_Pos          14                                           /*!<GPT GTBER: ADTDB Position */
#define GPT_GTBER_ADTDB_Msk          (0x1UL << GPT_GTBER_ADTDB_Pos)               /*!<GPT GTBER: ADTDB Mask */

#define GPT_GTBER_Msk_ALL            (GPT_GTBER_CCRA_Msk           |\
                                      GPT_GTBER_CCRB_Msk           |\
                                      GPT_GTBER_PR_Msk             |\
                                      GPT_GTBER_CCRSWT_Msk         |\
                                      GPT_GTBER_ADTTA_Msk          |\
                                      GPT_GTBER_ADTDA_Msk          |\
                                      GPT_GTBER_ADTTB_Msk          |\
                                      GPT_GTBER_ADTDB_Msk          )

#define GPT_GTUDC_UD_Pos             0                                            /*!<GPT GTUDC: UD Position */
#define GPT_GTUDC_UD_Msk             (0x1UL /*<< GPT_GTUDC_UD_Pos*/)              /*!<GPT GTUDC: UD Mask */

#define GPT_GTUDC_UDF_Pos            1                                            /*!<GPT GTUDC: UDF Position */
#define GPT_GTUDC_UDF_Msk            (0x1UL << GPT_GTUDC_UDF_Pos)                 /*!<GPT GTUDC: UDF Mask */

#define GPT_GTUDC_Msk_ALL            (GPT_GTUDC_UD_Msk             |\
                                      GPT_GTUDC_UDF_Msk            )

#define GPT_GTITC_ITLA_Pos           0                                            /*!<GPT GTITC: ITLA Position */
#define GPT_GTITC_ITLA_Msk           (0x1UL /*<< GPT_GTITC_ITLA_Pos*/)            /*!<GPT GTITC: ITLA Mask */

#define GPT_GTITC_ITLB_Pos           1                                            /*!<GPT GTITC: ITLB Position */
#define GPT_GTITC_ITLB_Msk           (0x1UL << GPT_GTITC_ITLB_Pos)                /*!<GPT GTITC: ITLB Mask */

#define GPT_GTITC_IVTC_Pos           4                                            /*!<GPT GTITC: IVTC Position */
#define GPT_GTITC_IVTC_Msk           (0x3UL << GPT_GTITC_IVTC_Pos)                /*!<GPT GTITC: IVTC Mask */

#define GPT_GTITC_IVTT_Pos           8                                            /*!<GPT GTITC: IVTT Position */
#define GPT_GTITC_IVTT_Msk           (0x7UL << GPT_GTITC_IVTT_Pos)                /*!<GPT GTITC: IVTT Mask */

#define GPT_GTITC_ADTAL_Pos          12                                           /*!<GPT GTITC: ADTAL Position */
#define GPT_GTITC_ADTAL_Msk          (0x1UL << GPT_GTITC_ADTAL_Pos)               /*!<GPT GTITC: ADTAL Mask */

#define GPT_GTITC_ADTBL_Pos          14                                           /*!<GPT GTITC: ADTBL Position */
#define GPT_GTITC_ADTBL_Msk          (0x1UL << GPT_GTITC_ADTBL_Pos)               /*!<GPT GTITC: ADTBL Mask */

#define GPT_GTITC_Msk_ALL            (GPT_GTITC_ITLA_Msk           |\
                                      GPT_GTITC_ITLB_Msk           |\
                                      GPT_GTITC_IVTC_Msk           |\
                                      GPT_GTITC_IVTT_Msk           |\
                                      GPT_GTITC_ADTAL_Msk          |\
                                      GPT_GTITC_ADTBL_Msk          )

#define GPT_GTST_TCFA_Pos            0                                            /*!<GPT GTST: TCFA Position */
#define GPT_GTST_TCFA_Msk            (0x1UL /*<< GPT_GTST_TCFA_Pos*/)             /*!<GPT GTST: TCFA Mask */

#define GPT_GTST_TCFB_Pos            1                                            /*!<GPT GTST: TCFB Position */
#define GPT_GTST_TCFB_Msk            (0x1UL << GPT_GTST_TCFB_Pos)                 /*!<GPT GTST: TCFB Mask */

#define GPT_GTST_TCFPO_Pos           2                                            /*!<GPT GTST: TCFPO Position */
#define GPT_GTST_TCFPO_Msk           (0x1UL << GPT_GTST_TCFPO_Pos)                /*!<GPT GTST: TCFPO Mask */

#define GPT_GTST_TCFPU_Pos           3                                            /*!<GPT GTST: TCFPU Position */
#define GPT_GTST_TCFPU_Msk           (0x1UL << GPT_GTST_TCFPU_Pos)                /*!<GPT GTST: TCFPU Mask */

#define GPT_GTST_DTEF_Pos            4                                            /*!<GPT GTST: DTEF Position */
#define GPT_GTST_DTEF_Msk            (0x1UL << GPT_GTST_DTEF_Pos)                 /*!<GPT GTST: DTEF Mask */

#define GPT_GTST_OSF_Pos             5                                            /*!<GPT GTST: OSF Position */
#define GPT_GTST_OSF_Msk             (0x1UL << GPT_GTST_OSF_Pos)                  /*!<GPT GTST: OSF Mask */

#define GPT_GTST_ITCNT_Pos           8                                            /*!<GPT GTST: ITCNT Position */
#define GPT_GTST_ITCNT_Msk           (0x7UL << GPT_GTST_ITCNT_Pos)                /*!<GPT GTST: ITCNT Mask */

#define GPT_GTST_TUCF_Pos            15                                           /*!<GPT GTST: TUCF Position */
#define GPT_GTST_TUCF_Msk            (0x1UL << GPT_GTST_TUCF_Pos)                 /*!<GPT GTST: TUCF Mask */

#define GPT_GTST_TCFAC_Pos           16                                           /*!<GPT GTST: TCFAC Position */
#define GPT_GTST_TCFAC_Msk           (0x1UL << GPT_GTST_TCFAC_Pos)                /*!<GPT GTST: TCFAC Mask */

#define GPT_GTST_TCFBC_Pos           17                                           /*!<GPT GTST: TCFBC Position */
#define GPT_GTST_TCFBC_Msk           (0x1UL << GPT_GTST_TCFBC_Pos)                /*!<GPT GTST: TCFBC Mask */

#define GPT_GTST_TCFPOC_Pos          18                                           /*!<GPT GTST: TCFPOC Position */
#define GPT_GTST_TCFPOC_Msk          (0x1UL << GPT_GTST_TCFPOC_Pos)               /*!<GPT GTST: TCFPOC Mask */

#define GPT_GTST_TCFPUC_Pos          19                                           /*!<GPT GTST: TCFPUC Position */
#define GPT_GTST_TCFPUC_Msk          (0x1UL << GPT_GTST_TCFPUC_Pos)               /*!<GPT GTST: TCFPUC Mask */

#define GPT_GTST_DTEFC_Pos           20                                           /*!<GPT GTST: DTEFC Position */
#define GPT_GTST_DTEFC_Msk           (0x1UL << GPT_GTST_DTEFC_Pos)                /*!<GPT GTST: DTEFC Mask */

#define GPT_GTST_OSFC_Pos            21                                           /*!<GPT GTST: OSFC Position */
#define GPT_GTST_OSFC_Msk            (0x1UL << GPT_GTST_OSFC_Pos)                 /*!<GPT GTST: OSFC Mask */

#define GPT_GTST_Msk_ALL             (GPT_GTST_TCFA_Msk            |\
                                      GPT_GTST_TCFB_Msk            |\
                                      GPT_GTST_TCFPO_Msk           |\
                                      GPT_GTST_TCFPU_Msk           |\
                                      GPT_GTST_DTEF_Msk            |\
                                      GPT_GTST_OSF_Msk             |\
                                      GPT_GTST_ITCNT_Msk           |\
                                      GPT_GTST_TUCF_Msk            |\
                                      GPT_GTST_TCFAC_Msk           |\
                                      GPT_GTST_TCFBC_Msk           |\
                                      GPT_GTST_TCFPOC_Msk          |\
                                      GPT_GTST_TCFPUC_Msk          |\
                                      GPT_GTST_DTEFC_Msk           |\
                                      GPT_GTST_OSFC_Msk            )

#define GPT_GTCNT_TCNTL_Pos          0                                            /*!<GPT GTCNT: TCNTL Position */
#define GPT_GTCNT_TCNTL_Msk          (0xFFFFUL /*<< GPT_GTCNT_TCNTL_Pos*/)        /*!<GPT GTCNT: TCNTL Mask */

#define GPT_GTCNT_TCNTH_Pos          16                                           /*!<GPT GTCNT: TCNTH Position */
#define GPT_GTCNT_TCNTH_Msk          (0xFFFFUL << GPT_GTCNT_TCNTH_Pos)            /*!<GPT GTCNT: TCNTH Mask */

#define GPT_GTCNT_Msk_ALL            (GPT_GTCNT_TCNTL_Msk          |\
                                      GPT_GTCNT_TCNTH_Msk          )

#define GPT_GTCCRA_GTCCRAL_Pos       0                                            /*!<GPT GTCCRA: GTCCRAL Position */
#define GPT_GTCCRA_GTCCRAL_Msk       (0xFFFFUL /*<< GPT_GTCCRA_GTCCRAL_Pos*/)     /*!<GPT GTCCRA: GTCCRAL Mask */

#define GPT_GTCCRA_GTCCRAH_Pos       16                                           /*!<GPT GTCCRA: GTCCRAH Position */
#define GPT_GTCCRA_GTCCRAH_Msk       (0xFFFFUL << GPT_GTCCRA_GTCCRAH_Pos)         /*!<GPT GTCCRA: GTCCRAH Mask */

#define GPT_GTCCRA_Msk_ALL           (GPT_GTCCRA_GTCCRAL_Msk       |\
                                      GPT_GTCCRA_GTCCRAH_Msk       )

#define GPT_GTCCRB_GTCCRBL_Pos       0                                            /*!<GPT GTCCRB: GTCCRBL Position */
#define GPT_GTCCRB_GTCCRBL_Msk       (0xFFFFUL /*<< GPT_GTCCRB_GTCCRBL_Pos*/)     /*!<GPT GTCCRB: GTCCRBL Mask */

#define GPT_GTCCRB_GTCCRBH_Pos       16                                           /*!<GPT GTCCRB: GTCCRBH Position */
#define GPT_GTCCRB_GTCCRBH_Msk       (0xFFFFUL << GPT_GTCCRB_GTCCRBH_Pos)         /*!<GPT GTCCRB: GTCCRBH Mask */

#define GPT_GTCCRB_Msk_ALL           (GPT_GTCCRB_GTCCRBL_Msk       |\
                                      GPT_GTCCRB_GTCCRBH_Msk       )

#define GPT_GTCCRC_GTCCRCL_Pos       0                                            /*!<GPT GTCCRC: GTCCRCL Position */
#define GPT_GTCCRC_GTCCRCL_Msk       (0xFFFFUL /*<< GPT_GTCCRC_GTCCRCL_Pos*/)     /*!<GPT GTCCRC: GTCCRCL Mask */

#define GPT_GTCCRC_GTCCRCH_Pos       16                                           /*!<GPT GTCCRC: GTCCRCH Position */
#define GPT_GTCCRC_GTCCRCH_Msk       (0xFFFFUL << GPT_GTCCRC_GTCCRCH_Pos)         /*!<GPT GTCCRC: GTCCRCH Mask */

#define GPT_GTCCRC_Msk_ALL           (GPT_GTCCRC_GTCCRCL_Msk       |\
                                      GPT_GTCCRC_GTCCRCH_Msk       )

#define GPT_GTCCRD_GTCCRDL_Pos       0                                            /*!<GPT GTCCRD: GTCCRDL Position */
#define GPT_GTCCRD_GTCCRDL_Msk       (0xFFFFUL /*<< GPT_GTCCRD_GTCCRDL_Pos*/)     /*!<GPT GTCCRD: GTCCRDL Mask */

#define GPT_GTCCRD_GTCCRDH_Pos       16                                           /*!<GPT GTCCRD: GTCCRDH Position */
#define GPT_GTCCRD_GTCCRDH_Msk       (0xFFFFUL << GPT_GTCCRD_GTCCRDH_Pos)         /*!<GPT GTCCRD: GTCCRDH Mask */

#define GPT_GTCCRD_Msk_ALL           (GPT_GTCCRD_GTCCRDL_Msk       |\
                                      GPT_GTCCRD_GTCCRDH_Msk       )

#define GPT_GTCCRE_GTCCREL_Pos       0                                            /*!<GPT GTCCRE: GTCCREL Position */
#define GPT_GTCCRE_GTCCREL_Msk       (0xFFFFUL /*<< GPT_GTCCRE_GTCCREL_Pos*/)     /*!<GPT GTCCRE: GTCCREL Mask */

#define GPT_GTCCRE_GTCCREH_Pos       16                                           /*!<GPT GTCCRE: GTCCREH Position */
#define GPT_GTCCRE_GTCCREH_Msk       (0xFFFFUL << GPT_GTCCRE_GTCCREH_Pos)         /*!<GPT GTCCRE: GTCCREH Mask */

#define GPT_GTCCRE_Msk_ALL           (GPT_GTCCRE_GTCCREL_Msk       |\
                                      GPT_GTCCRE_GTCCREH_Msk       )

#define GPT_GTCCRF_GTCCRFL_Pos       0                                            /*!<GPT GTCCRF: GTCCRFL Position */
#define GPT_GTCCRF_GTCCRFL_Msk       (0xFFFFUL /*<< GPT_GTCCRF_GTCCRFL_Pos*/)     /*!<GPT GTCCRF: GTCCRFL Mask */

#define GPT_GTCCRF_GTCCRFH_Pos       16                                           /*!<GPT GTCCRF: GTCCRFH Position */
#define GPT_GTCCRF_GTCCRFH_Msk       (0xFFFFUL << GPT_GTCCRF_GTCCRFH_Pos)         /*!<GPT GTCCRF: GTCCRFH Mask */

#define GPT_GTCCRF_Msk_ALL           (GPT_GTCCRF_GTCCRFL_Msk       |\
                                      GPT_GTCCRF_GTCCRFH_Msk       )

#define GPT_GTPR_GTPR_Pos            0                                            /*!<GPT GTPR: GTPR Position */
#define GPT_GTPR_GTPR_Msk            (0xFFFFUL /*<< GPT_GTPR_GTPR_Pos*/)          /*!<GPT GTPR: GTPR Mask */

#define GPT_GTPR_Msk_ALL             (GPT_GTPR_GTPR_Msk           )

#define GPT_GTPBR_GTPBR_Pos          0                                            /*!<GPT GTPBR: GTPBR Position */
#define GPT_GTPBR_GTPBR_Msk          (0xFFFFUL /*<< GPT_GTPBR_GTPBR_Pos*/)        /*!<GPT GTPBR: GTPBR Mask */

#define GPT_GTPBR_Msk_ALL            (GPT_GTPBR_GTPBR_Msk         )

#define GPT_GTPDBR_GTPDBR_Pos        0                                            /*!<GPT GTPDBR: GTPDBR Position */
#define GPT_GTPDBR_GTPDBR_Msk        (0xFFFFUL /*<< GPT_GTPDBR_GTPDBR_Pos*/)      /*!<GPT GTPDBR: GTPDBR Mask */

#define GPT_GTPDBR_Msk_ALL           (GPT_GTPDBR_GTPDBR_Msk       )

#define GPT_GTADTRA_GTADTRA_Pos      0                                            /*!<GPT GTADTRA: GTADTRA Position */
#define GPT_GTADTRA_GTADTRA_Msk      (0xFFFFUL /*<< GPT_GTADTRA_GTADTRA_Pos*/)    /*!<GPT GTADTRA: GTADTRA Mask */

#define GPT_GTADTRA_Msk_ALL          (GPT_GTADTRA_GTADTRA_Msk     )

#define GPT_GTADTRB_GTADTRB_Pos      0                                            /*!<GPT GTADTRB: GTADTRB Position */
#define GPT_GTADTRB_GTADTRB_Msk      (0xFFFFUL /*<< GPT_GTADTRB_GTADTRB_Pos*/)    /*!<GPT GTADTRB: GTADTRB Mask */

#define GPT_GTADTRB_Msk_ALL          (GPT_GTADTRB_GTADTRB_Msk     )

#define GPT_GTADTBRA_GTADTBRA_Pos    0                                            /*!<GPT GTADTBRA: GTADTBRA Position */
#define GPT_GTADTBRA_GTADTBRA_Msk    (0xFFFFUL /*<< GPT_GTADTBRA_GTADTBRA_Pos*/)  /*!<GPT GTADTBRA: GTADTBRA Mask */

#define GPT_GTADTBRA_Msk_ALL         (GPT_GTADTBRA_GTADTBRA_Msk   )

#define GPT_GTADTBRB_GTADTBRB_Pos    0                                            /*!<GPT GTADTBRB: GTADTBRB Position */
#define GPT_GTADTBRB_GTADTBRB_Msk    (0xFFFFUL /*<< GPT_GTADTBRB_GTADTBRB_Pos*/)  /*!<GPT GTADTBRB: GTADTBRB Mask */

#define GPT_GTADTBRB_Msk_ALL         (GPT_GTADTBRB_GTADTBRB_Msk   )

#define GPT_GTADTDBRA_GTADTDBRA_Pos  0                                            /*!<GPT GTADTDBRA: GTADTDBRA Position */
#define GPT_GTADTDBRA_GTADTDBRA_Msk  (0xFFFFUL /*<< GPT_GTADTDBRA_GTADTDBRA_Pos*/) /*!<GPT GTADTDBRA: GTADTDBRA Mask */

#define GPT_GTADTDBRA_Msk_ALL        (GPT_GTADTDBRA_GTADTDBRA_Msk )

#define GPT_GTADTDBRB_GTADTDBRB_Pos  0                                            /*!<GPT GTADTDBRB: GTADTDBRB Position */
#define GPT_GTADTDBRB_GTADTDBRB_Msk  (0xFFFFUL /*<< GPT_GTADTDBRB_GTADTDBRB_Pos*/) /*!<GPT GTADTDBRB: GTADTDBRB Mask */

#define GPT_GTADTDBRB_Msk_ALL        (GPT_GTADTDBRB_GTADTDBRB_Msk )

#define GPT_GTDTCR_TDE_Pos           0                                            /*!<GPT GTDTCR: TDE Position */
#define GPT_GTDTCR_TDE_Msk           (0x1UL /*<< GPT_GTDTCR_TDE_Pos*/)            /*!<GPT GTDTCR: TDE Mask */

#define GPT_GTDTCR_TDBUE_Pos         4                                            /*!<GPT GTDTCR: TDBUE Position */
#define GPT_GTDTCR_TDBUE_Msk         (0x1UL << GPT_GTDTCR_TDBUE_Pos)              /*!<GPT GTDTCR: TDBUE Mask */

#define GPT_GTDTCR_TDBDE_Pos         5                                            /*!<GPT GTDTCR: TDBDE Position */
#define GPT_GTDTCR_TDBDE_Msk         (0x1UL << GPT_GTDTCR_TDBDE_Pos)              /*!<GPT GTDTCR: TDBDE Mask */

#define GPT_GTDTCR_TDFER_Pos         8                                            /*!<GPT GTDTCR: TDFER Position */
#define GPT_GTDTCR_TDFER_Msk         (0x1UL << GPT_GTDTCR_TDFER_Pos)              /*!<GPT GTDTCR: TDFER Mask */

#define GPT_GTDTCR_Msk_ALL           (GPT_GTDTCR_TDE_Msk           |\
                                      GPT_GTDTCR_TDBUE_Msk         |\
                                      GPT_GTDTCR_TDBDE_Msk         |\
                                      GPT_GTDTCR_TDFER_Msk         )

#define GPT_GTDVU_GTDVU_Pos          0                                            /*!<GPT GTDVU: GTDVU Position */
#define GPT_GTDVU_GTDVU_Msk          (0xFFFFUL /*<< GPT_GTDVU_GTDVU_Pos*/)        /*!<GPT GTDVU: GTDVU Mask */

#define GPT_GTDVU_Msk_ALL            (GPT_GTDVU_GTDVU_Msk         )

#define GPT_GTDVD_GTDVD_Pos          0                                            /*!<GPT GTDVD: GTDVD Position */
#define GPT_GTDVD_GTDVD_Msk          (0xFFFFUL /*<< GPT_GTDVD_GTDVD_Pos*/)        /*!<GPT GTDVD: GTDVD Mask */

#define GPT_GTDVD_Msk_ALL            (GPT_GTDVD_GTDVD_Msk         )

#define GPT_GTDBU_GTDBU_Pos          0                                            /*!<GPT GTDBU: GTDBU Position */
#define GPT_GTDBU_GTDBU_Msk          (0xFFFFUL /*<< GPT_GTDBU_GTDBU_Pos*/)        /*!<GPT GTDBU: GTDBU Mask */

#define GPT_GTDBU_Msk_ALL            (GPT_GTDBU_GTDBU_Msk         )

#define GPT_GTDBD_GTDBD_Pos          0                                            /*!<GPT GTDBD: GTDBD Position */
#define GPT_GTDBD_GTDBD_Msk          (0xFFFFUL /*<< GPT_GTDBD_GTDBD_Pos*/)        /*!<GPT GTDBD: GTDBD Mask */

#define GPT_GTDBD_Msk_ALL            (GPT_GTDBD_GTDBD_Msk         )

#define GPT_GTOSCR_OLSGA_Pos         0                                            /*!<GPT GTOSCR: OLSGA Position */
#define GPT_GTOSCR_OLSGA_Msk         (0x1UL /*<< GPT_GTOSCR_OLSGA_Pos*/)          /*!<GPT GTOSCR: OLSGA Mask */

#define GPT_GTOSCR_OLSGB_Pos         1                                            /*!<GPT GTOSCR: OLSGB Position */
#define GPT_GTOSCR_OLSGB_Msk         (0x1UL << GPT_GTOSCR_OLSGB_Pos)              /*!<GPT GTOSCR: OLSGB Mask */

#define GPT_GTOSCR_OLSEN_Pos         7                                            /*!<GPT GTOSCR: OLSEN Position */
#define GPT_GTOSCR_OLSEN_Msk         (0x1UL << GPT_GTOSCR_OLSEN_Pos)              /*!<GPT GTOSCR: OLSEN Mask */

#define GPT_GTOSCR_Msk_ALL           (GPT_GTOSCR_OLSGA_Msk         |\
                                      GPT_GTOSCR_OLSGB_Msk         |\
                                      GPT_GTOSCR_OLSEN_Msk         )

#define TIM_CR_STR_Pos               0                                            /*!<TIM CR: STR Position */
#define TIM_CR_STR_Msk               (0x1UL /*<< TIM_CR_STR_Pos*/)                /*!<TIM CR: STR Mask */

#define TIM_CR_OPM_Pos               3                                            /*!<TIM CR: OPM Position */
#define TIM_CR_OPM_Msk               (0x1UL << TIM_CR_OPM_Pos)                    /*!<TIM CR: OPM Mask */

#define TIM_CR_CLKS_Pos              4                                            /*!<TIM CR: CLKS Position */
#define TIM_CR_CLKS_Msk              (0x3UL << TIM_CR_CLKS_Pos)                   /*!<TIM CR: CLKS Mask */

#define TIM_CR_IE_Pos                6                                            /*!<TIM CR: IE Position */
#define TIM_CR_IE_Msk                (0x1UL << TIM_CR_IE_Pos)                     /*!<TIM CR: IE Mask */

#define TIM_CR_TRIGEN_Pos            8                                            /*!<TIM CR: TRIGEN Position */
#define TIM_CR_TRIGEN_Msk            (0x1UL << TIM_CR_TRIGEN_Pos)                 /*!<TIM CR: TRIGEN Mask */

#define TIM_CR_ETEN_Pos              9                                            /*!<TIM CR: ETEN Position */
#define TIM_CR_ETEN_Msk              (0x1UL << TIM_CR_ETEN_Pos)                   /*!<TIM CR: ETEN Mask */

#define TIM_CR_TC_Pos                10                                           /*!<TIM CR: TC Position */
#define TIM_CR_TC_Msk                (0x1UL << TIM_CR_TC_Pos)                     /*!<TIM CR: TC Mask */

#define TIM_CR_CASCEN_Pos            15                                           /*!<TIM CR: CASCEN Position */
#define TIM_CR_CASCEN_Msk            (0x1UL << TIM_CR_CASCEN_Pos)                 /*!<TIM CR: CASCEN Mask */

#define TIM_CR_Msk_ALL               (TIM_CR_STR_Msk               |\
                                      TIM_CR_OPM_Msk               |\
                                      TIM_CR_CLKS_Msk              |\
                                      TIM_CR_IE_Msk                |\
                                      TIM_CR_TRIGEN_Msk            |\
                                      TIM_CR_ETEN_Msk              |\
                                      TIM_CR_TC_Msk                |\
                                      TIM_CR_CASCEN_Msk            )

#define TIM_TCNT_TCNTL_Pos           0                                            /*!<TIM TCNT: TCNTL Position */
#define TIM_TCNT_TCNTL_Msk           (0xFFFFUL /*<< TIM_TCNT_TCNTL_Pos*/)         /*!<TIM TCNT: TCNTL Mask */

#define TIM_TCNT_TCNTH_Pos           16                                           /*!<TIM TCNT: TCNTH Position */
#define TIM_TCNT_TCNTH_Msk           (0xFFFFUL << TIM_TCNT_TCNTH_Pos)             /*!<TIM TCNT: TCNTH Mask */

#define TIM_TCNT_Msk_ALL             (TIM_TCNT_TCNTL_Msk           |\
                                      TIM_TCNT_TCNTH_Msk           )

#define TIM_TPR_TPRL_Pos             0                                            /*!<TIM TPR: TPRL Position */
#define TIM_TPR_TPRL_Msk             (0xFFFFUL /*<< TIM_TPR_TPRL_Pos*/)           /*!<TIM TPR: TPRL Mask */

#define TIM_TPR_TPRH_Pos             16                                           /*!<TIM TPR: TPRH Position */
#define TIM_TPR_TPRH_Msk             (0xFFFFUL << TIM_TPR_TPRH_Pos)               /*!<TIM TPR: TPRH Mask */

#define TIM_TPR_Msk_ALL              (TIM_TPR_TPRL_Msk             |\
                                      TIM_TPR_TPRH_Msk             )

#define TIM_PSQ_PSQL_Pos             0                                            /*!<TIM PSQ: PSQL Position */
#define TIM_PSQ_PSQL_Msk             (0xFFFFUL /*<< TIM_PSQ_PSQL_Pos*/)           /*!<TIM PSQ: PSQL Mask */

#define TIM_PSQ_PSQH_Pos             16                                           /*!<TIM PSQ: PSQH Position */
#define TIM_PSQ_PSQH_Msk             (0xFFFFUL << TIM_PSQ_PSQH_Pos)               /*!<TIM PSQ: PSQH Mask */

#define TIM_PSQ_Msk_ALL              (TIM_PSQ_PSQL_Msk             |\
                                      TIM_PSQ_PSQH_Msk             )

#define TIM_TIMINTF_TF_Pos           0                                            /*!<TIM TIMINTF: TF Position */
#define TIM_TIMINTF_TF_Msk           (0x1UL /*<< TIM_TIMINTF_TF_Pos*/)            /*!<TIM TIMINTF: TF Mask */

#define TIM_TIMINTF_TFC_Pos          16                                           /*!<TIM TIMINTF: TFC Position */
#define TIM_TIMINTF_TFC_Msk          (0x1UL << TIM_TIMINTF_TFC_Pos)               /*!<TIM TIMINTF: TFC Mask */

#define TIM_TIMINTF_Msk_ALL          (TIM_TIMINTF_TF_Msk           |\
                                      TIM_TIMINTF_TFC_Msk          )

#define MACP_CORDCSR0_RUN_Pos        0                                            /*!<MACP CORDCSR0: RUN Position */
#define MACP_CORDCSR0_RUN_Msk        (0x1UL /*<< MACP_CORDCSR0_RUN_Pos*/)         /*!<MACP CORDCSR0: RUN Mask */

#define MACP_CORDCSR0_OVF_Pos        1                                            /*!<MACP CORDCSR0: OVF Position */
#define MACP_CORDCSR0_OVF_Msk        (0x1UL << MACP_CORDCSR0_OVF_Pos)             /*!<MACP CORDCSR0: OVF Mask */

#define MACP_CORDCSR0_KADJ_Pos       2                                            /*!<MACP CORDCSR0: KADJ Position */
#define MACP_CORDCSR0_KADJ_Msk       (0x1UL << MACP_CORDCSR0_KADJ_Pos)            /*!<MACP CORDCSR0: KADJ Mask */

#define MACP_CORDCSR0_XYMRS_Pos      3                                            /*!<MACP CORDCSR0: XYMRS Position */
#define MACP_CORDCSR0_XYMRS_Msk      (0x1UL << MACP_CORDCSR0_XYMRS_Pos)           /*!<MACP CORDCSR0: XYMRS Mask */

#define MACP_CORDCSR0_FORMAT_Pos     5                                            /*!<MACP CORDCSR0: FORMAT Position */
#define MACP_CORDCSR0_FORMAT_Msk     (0x3UL << MACP_CORDCSR0_FORMAT_Pos)          /*!<MACP CORDCSR0: FORMAT Mask */

#define MACP_CORDCSR0_MODE_Pos       7                                            /*!<MACP CORDCSR0: MODE Position */
#define MACP_CORDCSR0_MODE_Msk       (0x1UL << MACP_CORDCSR0_MODE_Pos)            /*!<MACP CORDCSR0: MODE Mask */

#define MACP_CORDCSR0_Msk_ALL        (MACP_CORDCSR0_RUN_Msk        |\
                                      MACP_CORDCSR0_OVF_Msk        |\
                                      MACP_CORDCSR0_KADJ_Msk       |\
                                      MACP_CORDCSR0_XYMRS_Msk      |\
                                      MACP_CORDCSR0_FORMAT_Msk     |\
                                      MACP_CORDCSR0_MODE_Msk       )

#define MACP_CORDCSR1_RUN_Pos        0                                            /*!<MACP CORDCSR1: RUN Position */
#define MACP_CORDCSR1_RUN_Msk        (0x1UL /*<< MACP_CORDCSR1_RUN_Pos*/)         /*!<MACP CORDCSR1: RUN Mask */

#define MACP_CORDCSR1_OVF_Pos        1                                            /*!<MACP CORDCSR1: OVF Position */
#define MACP_CORDCSR1_OVF_Msk        (0x1UL << MACP_CORDCSR1_OVF_Pos)             /*!<MACP CORDCSR1: OVF Mask */

#define MACP_CORDCSR1_KADJ_Pos       2                                            /*!<MACP CORDCSR1: KADJ Position */
#define MACP_CORDCSR1_KADJ_Msk       (0x1UL << MACP_CORDCSR1_KADJ_Pos)            /*!<MACP CORDCSR1: KADJ Mask */

#define MACP_CORDCSR1_XYMRS_Pos      3                                            /*!<MACP CORDCSR1: XYMRS Position */
#define MACP_CORDCSR1_XYMRS_Msk      (0x1UL << MACP_CORDCSR1_XYMRS_Pos)           /*!<MACP CORDCSR1: XYMRS Mask */

#define MACP_CORDCSR1_FORMAT_Pos     5                                            /*!<MACP CORDCSR1: FORMAT Position */
#define MACP_CORDCSR1_FORMAT_Msk     (0x3UL << MACP_CORDCSR1_FORMAT_Pos)          /*!<MACP CORDCSR1: FORMAT Mask */

#define MACP_CORDCSR1_MODE_Pos       7                                            /*!<MACP CORDCSR1: MODE Position */
#define MACP_CORDCSR1_MODE_Msk       (0x1UL << MACP_CORDCSR1_MODE_Pos)            /*!<MACP CORDCSR1: MODE Mask */

#define MACP_CORDCSR1_Msk_ALL        (MACP_CORDCSR1_RUN_Msk        |\
                                      MACP_CORDCSR1_OVF_Msk        |\
                                      MACP_CORDCSR1_KADJ_Msk       |\
                                      MACP_CORDCSR1_XYMRS_Msk      |\
                                      MACP_CORDCSR1_FORMAT_Msk     |\
                                      MACP_CORDCSR1_MODE_Msk       )

#define MACP_IQDIVCSR0_RUN_Pos       0                                            /*!<MACP IQDIVCSR0: RUN Position */
#define MACP_IQDIVCSR0_RUN_Msk       (0x1UL /*<< MACP_IQDIVCSR0_RUN_Pos*/)        /*!<MACP IQDIVCSR0: RUN Mask */

#define MACP_IQDIVCSR0_SIGN_Pos      1                                            /*!<MACP IQDIVCSR0: SIGN Position */
#define MACP_IQDIVCSR0_SIGN_Msk      (0x1UL << MACP_IQDIVCSR0_SIGN_Pos)           /*!<MACP IQDIVCSR0: SIGN Mask */

#define MACP_IQDIVCSR0_SAT_Pos       2                                            /*!<MACP IQDIVCSR0: SAT Position */
#define MACP_IQDIVCSR0_SAT_Msk       (0x1UL << MACP_IQDIVCSR0_SAT_Pos)            /*!<MACP IQDIVCSR0: SAT Mask */

#define MACP_IQDIVCSR0_Q_Pos         3                                            /*!<MACP IQDIVCSR0: Q Position */
#define MACP_IQDIVCSR0_Q_Msk         (0x1FUL << MACP_IQDIVCSR0_Q_Pos)             /*!<MACP IQDIVCSR0: Q Mask */

#define MACP_IQDIVCSR0_CMOD_Pos      8                                            /*!<MACP IQDIVCSR0: CMOD Position */
#define MACP_IQDIVCSR0_CMOD_Msk      (0x1UL << MACP_IQDIVCSR0_CMOD_Pos)           /*!<MACP IQDIVCSR0: CMOD Mask */

#define MACP_IQDIVCSR0_Msk_ALL       (MACP_IQDIVCSR0_RUN_Msk       |\
                                      MACP_IQDIVCSR0_SIGN_Msk      |\
                                      MACP_IQDIVCSR0_SAT_Msk       |\
                                      MACP_IQDIVCSR0_Q_Msk         |\
                                      MACP_IQDIVCSR0_CMOD_Msk      )

#define MACP_IQDIVCSR1_RUN_Pos       0                                            /*!<MACP IQDIVCSR1: RUN Position */
#define MACP_IQDIVCSR1_RUN_Msk       (0x1UL /*<< MACP_IQDIVCSR1_RUN_Pos*/)        /*!<MACP IQDIVCSR1: RUN Mask */

#define MACP_IQDIVCSR1_SIGN_Pos      1                                            /*!<MACP IQDIVCSR1: SIGN Position */
#define MACP_IQDIVCSR1_SIGN_Msk      (0x1UL << MACP_IQDIVCSR1_SIGN_Pos)           /*!<MACP IQDIVCSR1: SIGN Mask */

#define MACP_IQDIVCSR1_SAT_Pos       2                                            /*!<MACP IQDIVCSR1: SAT Position */
#define MACP_IQDIVCSR1_SAT_Msk       (0x1UL << MACP_IQDIVCSR1_SAT_Pos)            /*!<MACP IQDIVCSR1: SAT Mask */

#define MACP_IQDIVCSR1_Q_Pos         3                                            /*!<MACP IQDIVCSR1: Q Position */
#define MACP_IQDIVCSR1_Q_Msk         (0x1FUL << MACP_IQDIVCSR1_Q_Pos)             /*!<MACP IQDIVCSR1: Q Mask */

#define MACP_IQDIVCSR1_CMOD_Pos      8                                            /*!<MACP IQDIVCSR1: CMOD Position */
#define MACP_IQDIVCSR1_CMOD_Msk      (0x1UL << MACP_IQDIVCSR1_CMOD_Pos)           /*!<MACP IQDIVCSR1: CMOD Mask */

#define MACP_IQDIVCSR1_Msk_ALL       (MACP_IQDIVCSR1_RUN_Msk       |\
                                      MACP_IQDIVCSR1_SIGN_Msk      |\
                                      MACP_IQDIVCSR1_SAT_Msk       |\
                                      MACP_IQDIVCSR1_Q_Msk         |\
                                      MACP_IQDIVCSR1_CMOD_Msk      )

#define MACP_SVCON_RUN_Pos           0                                            /*!<MACP SVCON: RUN Position */
#define MACP_SVCON_RUN_Msk           (0x1UL /*<< MACP_SVCON_RUN_Pos*/)            /*!<MACP SVCON: RUN Mask */

#define MACP_SVCON_Msk_ALL           (MACP_SVCON_RUN_Msk          )

#define MACP_SVIQN_SVIQN_Pos         0                                            /*!<MACP SVIQN: SVIQN Position */
#define MACP_SVIQN_SVIQN_Msk         (0xFFUL /*<< MACP_SVIQN_SVIQN_Pos*/)         /*!<MACP SVIQN: SVIQN Mask */

#define MACP_SVIQN_SVTYP_Pos         8                                            /*!<MACP SVIQN: SVTYP Position */
#define MACP_SVIQN_SVTYP_Msk         (0x1UL << MACP_SVIQN_SVTYP_Pos)              /*!<MACP SVIQN: SVTYP Mask */

#define MACP_SVIQN_Msk_ALL           (MACP_SVIQN_SVIQN_Msk         |\
                                      MACP_SVIQN_SVTYP_Msk         )

#define MACP_SVSECTOR_SVSECTOR_Pos   0                                            /*!<MACP SVSECTOR: SVSECTOR Position */
#define MACP_SVSECTOR_SVSECTOR_Msk   (0xFFUL /*<< MACP_SVSECTOR_SVSECTOR_Pos*/)   /*!<MACP SVSECTOR: SVSECTOR Mask */

#define MACP_SVSECTOR_Msk_ALL        (MACP_SVSECTOR_SVSECTOR_Msk  )

#define ADC_ADCON1_ADSOC_Pos         0                                            /*!<ADC ADCON1: ADSOC Position */
#define ADC_ADCON1_ADSOC_Msk         (0x1UL /*<< ADC_ADCON1_ADSOC_Pos*/)          /*!<ADC ADCON1: ADSOC Mask */

#define ADC_ADCON1_ADCTU_Pos         1                                            /*!<ADC ADCON1: ADCTU Position */
#define ADC_ADCON1_ADCTU_Msk         (0x3UL << ADC_ADCON1_ADCTU_Pos)              /*!<ADC ADCON1: ADCTU Mask */

#define ADC_ADCON1_MOD_Pos           3                                            /*!<ADC ADCON1: MOD Position */
#define ADC_ADCON1_MOD_Msk           (0x1UL << ADC_ADCON1_MOD_Pos)                /*!<ADC ADCON1: MOD Mask */

#define ADC_ADCON1_REFC_Pos          4                                            /*!<ADC ADCON1: REFC Position */
#define ADC_ADCON1_REFC_Msk          (0x3UL << ADC_ADCON1_REFC_Pos)               /*!<ADC ADCON1: REFC Mask */

#define ADC_ADCON1_ADDE_Pos          6                                            /*!<ADC ADCON1: ADDE Position */
#define ADC_ADCON1_ADDE_Msk          (0x1UL << ADC_ADCON1_ADDE_Pos)               /*!<ADC ADCON1: ADDE Mask */

#define ADC_ADCON1_ADIE_Pos          7                                            /*!<ADC ADCON1: ADIE Position */
#define ADC_ADCON1_ADIE_Msk          (0x1UL << ADC_ADCON1_ADIE_Pos)               /*!<ADC ADCON1: ADIE Mask */

#define ADC_ADCON1_ADSTRS_Pos        8                                            /*!<ADC ADCON1: ADSTRS Position */
#define ADC_ADCON1_ADSTRS_Msk        (0x7FUL << ADC_ADCON1_ADSTRS_Pos)            /*!<ADC ADCON1: ADSTRS Mask */

#define ADC_ADCON1_ADON_Pos          15                                           /*!<ADC ADCON1: ADON Position */
#define ADC_ADCON1_ADON_Msk          (0x1UL << ADC_ADCON1_ADON_Pos)               /*!<ADC ADCON1: ADON Mask */

#define ADC_ADCON1_Msk_ALL           (ADC_ADCON1_ADSOC_Msk         |\
                                      ADC_ADCON1_ADCTU_Msk         |\
                                      ADC_ADCON1_MOD_Msk           |\
                                      ADC_ADCON1_REFC_Msk          |\
                                      ADC_ADCON1_ADDE_Msk          |\
                                      ADC_ADCON1_ADIE_Msk          |\
                                      ADC_ADCON1_ADSTRS_Msk        |\
                                      ADC_ADCON1_ADON_Msk          )

#define ADC_ADCON2_TADC_Pos          0                                            /*!<ADC ADCON2: TADC Position */
#define ADC_ADCON2_TADC_Msk          (0xFUL /*<< ADC_ADCON2_TADC_Pos*/)           /*!<ADC ADCON2: TADC Mask */

#define ADC_ADCON2_ADMAXCH_Pos       4                                            /*!<ADC ADCON2: ADMAXCH Position */
#define ADC_ADCON2_ADMAXCH_Msk       (0x7UL << ADC_ADCON2_ADMAXCH_Pos)            /*!<ADC ADCON2: ADMAXCH Mask */

#define ADC_ADCON2_TS_Pos            8                                            /*!<ADC ADCON2: TS Position */
#define ADC_ADCON2_TS_Msk            (0xFUL << ADC_ADCON2_TS_Pos)                 /*!<ADC ADCON2: TS Mask */

#define ADC_ADCON2_TGAP_Pos          16                                           /*!<ADC ADCON2: TGAP Position */
#define ADC_ADCON2_TGAP_Msk          (0x7UL << ADC_ADCON2_TGAP_Pos)               /*!<ADC ADCON2: TGAP Mask */

#define ADC_ADCON2_Msk_ALL           (ADC_ADCON2_TADC_Msk          |\
                                      ADC_ADCON2_ADMAXCH_Msk       |\
                                      ADC_ADCON2_TS_Msk            |\
                                      ADC_ADCON2_TGAP_Msk          )

#define ADC_ADPCH_ADPCH_Pos          0                                            /*!<ADC ADPCH: ADPCH Position */
#define ADC_ADPCH_ADPCH_Msk          (0x7UL /*<< ADC_ADPCH_ADPCH_Pos*/)           /*!<ADC ADPCH: ADPCH Mask */

#define ADC_ADPCH_Msk_ALL            (ADC_ADPCH_ADPCH_Msk         )

#define ADC_ADDR0_ADDR0_Pos          0                                            /*!<ADC ADDR0: ADDR0 Position */
#define ADC_ADDR0_ADDR0_Msk          (0xFFFFUL /*<< ADC_ADDR0_ADDR0_Pos*/)        /*!<ADC ADDR0: ADDR0 Mask */

#define ADC_ADDR0_Msk_ALL            (ADC_ADDR0_ADDR0_Msk         )

#define ADC_ADDR1_ADDR1_Pos          0                                            /*!<ADC ADDR1: ADDR1 Position */
#define ADC_ADDR1_ADDR1_Msk          (0xFFFFUL /*<< ADC_ADDR1_ADDR1_Pos*/)        /*!<ADC ADDR1: ADDR1 Mask */

#define ADC_ADDR1_Msk_ALL            (ADC_ADDR1_ADDR1_Msk         )

#define ADC_ADDR2_ADDR2_Pos          0                                            /*!<ADC ADDR2: ADDR2 Position */
#define ADC_ADDR2_ADDR2_Msk          (0xFFFFUL /*<< ADC_ADDR2_ADDR2_Pos*/)        /*!<ADC ADDR2: ADDR2 Mask */

#define ADC_ADDR2_Msk_ALL            (ADC_ADDR2_ADDR2_Msk         )

#define ADC_ADDR3_ADDR3_Pos          0                                            /*!<ADC ADDR3: ADDR3 Position */
#define ADC_ADDR3_ADDR3_Msk          (0xFFFFUL /*<< ADC_ADDR3_ADDR3_Pos*/)        /*!<ADC ADDR3: ADDR3 Mask */

#define ADC_ADDR3_Msk_ALL            (ADC_ADDR3_ADDR3_Msk         )

#define ADC_ADDR4_ADDR4_Pos          0                                            /*!<ADC ADDR4: ADDR4 Position */
#define ADC_ADDR4_ADDR4_Msk          (0xFFFFUL /*<< ADC_ADDR4_ADDR4_Pos*/)        /*!<ADC ADDR4: ADDR4 Mask */

#define ADC_ADDR4_Msk_ALL            (ADC_ADDR4_ADDR4_Msk         )

#define ADC_ADDR5_ADDR5_Pos          0                                            /*!<ADC ADDR5: ADDR5 Position */
#define ADC_ADDR5_ADDR5_Msk          (0xFFFFUL /*<< ADC_ADDR5_ADDR5_Pos*/)        /*!<ADC ADDR5: ADDR5 Mask */

#define ADC_ADDR5_Msk_ALL            (ADC_ADDR5_ADDR5_Msk         )

#define ADC_ADDR6_ADDR6_Pos          0                                            /*!<ADC ADDR6: ADDR6 Position */
#define ADC_ADDR6_ADDR6_Msk          (0xFFFFUL /*<< ADC_ADDR6_ADDR6_Pos*/)        /*!<ADC ADDR6: ADDR6 Mask */

#define ADC_ADDR6_Msk_ALL            (ADC_ADDR6_ADDR6_Msk         )

#define ADC_ADDR7_ADDR7_Pos          0                                            /*!<ADC ADDR7: ADDR7 Position */
#define ADC_ADDR7_ADDR7_Msk          (0xFFFFUL /*<< ADC_ADDR7_ADDR7_Pos*/)        /*!<ADC ADDR7: ADDR7 Mask */

#define ADC_ADDR7_Msk_ALL            (ADC_ADDR7_ADDR7_Msk         )

#define ADC_ADCMPCON_CSEL_Pos        0                                            /*!<ADC ADCMPCON: CSEL Position */
#define ADC_ADCMPCON_CSEL_Msk        (0x7UL /*<< ADC_ADCMPCON_CSEL_Pos*/)         /*!<ADC ADCMPCON: CSEL Mask */

#define ADC_ADCMPCON_ADLDE_Pos       4                                            /*!<ADC ADCMPCON: ADLDE Position */
#define ADC_ADCMPCON_ADLDE_Msk       (0x1UL << ADC_ADCMPCON_ADLDE_Pos)            /*!<ADC ADCMPCON: ADLDE Mask */

#define ADC_ADCMPCON_ADLIE_Pos       5                                            /*!<ADC ADCMPCON: ADLIE Position */
#define ADC_ADCMPCON_ADLIE_Msk       (0x1UL << ADC_ADCMPCON_ADLIE_Pos)            /*!<ADC ADCMPCON: ADLIE Mask */

#define ADC_ADCMPCON_ADGDE_Pos       6                                            /*!<ADC ADCMPCON: ADGDE Position */
#define ADC_ADCMPCON_ADGDE_Msk       (0x1UL << ADC_ADCMPCON_ADGDE_Pos)            /*!<ADC ADCMPCON: ADGDE Mask */

#define ADC_ADCMPCON_ADGIE_Pos       7                                            /*!<ADC ADCMPCON: ADGIE Position */
#define ADC_ADCMPCON_ADGIE_Msk       (0x1UL << ADC_ADCMPCON_ADGIE_Pos)            /*!<ADC ADCMPCON: ADGIE Mask */

#define ADC_ADCMPCON_Msk_ALL         (ADC_ADCMPCON_CSEL_Msk        |\
                                      ADC_ADCMPCON_ADLDE_Msk       |\
                                      ADC_ADCMPCON_ADLIE_Msk       |\
                                      ADC_ADCMPCON_ADGDE_Msk       |\
                                      ADC_ADCMPCON_ADGIE_Msk       )

#define ADC_ADDGT_GT_Pos             0                                            /*!<ADC ADDGT: GT Position */
#define ADC_ADDGT_GT_Msk             (0xFFFFUL /*<< ADC_ADDGT_GT_Pos*/)           /*!<ADC ADDGT: GT Mask */

#define ADC_ADDGT_Msk_ALL            (ADC_ADDGT_GT_Msk            )

#define ADC_ADDLT_LT_Pos             0                                            /*!<ADC ADDLT: LT Position */
#define ADC_ADDLT_LT_Msk             (0xFFFFUL /*<< ADC_ADDLT_LT_Pos*/)           /*!<ADC ADDLT: LT Mask */

#define ADC_ADDLT_Msk_ALL            (ADC_ADDLT_LT_Msk            )

#define ADC_SEQCHSEL_SEQCH0_Pos      0                                            /*!<ADC SEQCHSEL: SEQCH0 Position */
#define ADC_SEQCHSEL_SEQCH0_Msk      (0xFUL /*<< ADC_SEQCHSEL_SEQCH0_Pos*/)       /*!<ADC SEQCHSEL: SEQCH0 Mask */

#define ADC_SEQCHSEL_SEQCH1_Pos      4                                            /*!<ADC SEQCHSEL: SEQCH1 Position */
#define ADC_SEQCHSEL_SEQCH1_Msk      (0xFUL << ADC_SEQCHSEL_SEQCH1_Pos)           /*!<ADC SEQCHSEL: SEQCH1 Mask */

#define ADC_SEQCHSEL_SEQCH2_Pos      8                                            /*!<ADC SEQCHSEL: SEQCH2 Position */
#define ADC_SEQCHSEL_SEQCH2_Msk      (0xFUL << ADC_SEQCHSEL_SEQCH2_Pos)           /*!<ADC SEQCHSEL: SEQCH2 Mask */

#define ADC_SEQCHSEL_SEQCH3_Pos      12                                           /*!<ADC SEQCHSEL: SEQCH3 Position */
#define ADC_SEQCHSEL_SEQCH3_Msk      (0xFUL << ADC_SEQCHSEL_SEQCH3_Pos)           /*!<ADC SEQCHSEL: SEQCH3 Mask */

#define ADC_SEQCHSEL_SEQCH4_Pos      16                                           /*!<ADC SEQCHSEL: SEQCH4 Position */
#define ADC_SEQCHSEL_SEQCH4_Msk      (0xFUL << ADC_SEQCHSEL_SEQCH4_Pos)           /*!<ADC SEQCHSEL: SEQCH4 Mask */

#define ADC_SEQCHSEL_SEQCH5_Pos      20                                           /*!<ADC SEQCHSEL: SEQCH5 Position */
#define ADC_SEQCHSEL_SEQCH5_Msk      (0xFUL << ADC_SEQCHSEL_SEQCH5_Pos)           /*!<ADC SEQCHSEL: SEQCH5 Mask */

#define ADC_SEQCHSEL_SEQCH6_Pos      24                                           /*!<ADC SEQCHSEL: SEQCH6 Position */
#define ADC_SEQCHSEL_SEQCH6_Msk      (0xFUL << ADC_SEQCHSEL_SEQCH6_Pos)           /*!<ADC SEQCHSEL: SEQCH6 Mask */

#define ADC_SEQCHSEL_SEQCH7_Pos      28                                           /*!<ADC SEQCHSEL: SEQCH7 Position */
#define ADC_SEQCHSEL_SEQCH7_Msk      (0xFUL << ADC_SEQCHSEL_SEQCH7_Pos)           /*!<ADC SEQCHSEL: SEQCH7 Mask */

#define ADC_SEQCHSEL_Msk_ALL         (ADC_SEQCHSEL_SEQCH0_Msk      |\
                                      ADC_SEQCHSEL_SEQCH1_Msk      |\
                                      ADC_SEQCHSEL_SEQCH2_Msk      |\
                                      ADC_SEQCHSEL_SEQCH3_Msk      |\
                                      ADC_SEQCHSEL_SEQCH4_Msk      |\
                                      ADC_SEQCHSEL_SEQCH5_Msk      |\
                                      ADC_SEQCHSEL_SEQCH6_Msk      |\
                                      ADC_SEQCHSEL_SEQCH7_Msk      )

#define ADC_ADGAPON_GAPEN_Pos        0                                            /*!<ADC ADGAPON: GAPEN Position */
#define ADC_ADGAPON_GAPEN_Msk        (0xFFUL /*<< ADC_ADGAPON_GAPEN_Pos*/)        /*!<ADC ADGAPON: GAPEN Mask */

#define ADC_ADGAPON_Msk_ALL          (ADC_ADGAPON_GAPEN_Msk       )

#define ADC_ADINTF_ADLIF_Pos         0                                            /*!<ADC ADINTF: ADLIF Position */
#define ADC_ADINTF_ADLIF_Msk         (0x1UL /*<< ADC_ADINTF_ADLIF_Pos*/)          /*!<ADC ADINTF: ADLIF Mask */

#define ADC_ADINTF_ADGIF_Pos         1                                            /*!<ADC ADINTF: ADGIF Position */
#define ADC_ADINTF_ADGIF_Msk         (0x1UL << ADC_ADINTF_ADGIF_Pos)              /*!<ADC ADINTF: ADGIF Mask */

#define ADC_ADINTF_ADIF_Pos          2                                            /*!<ADC ADINTF: ADIF Position */
#define ADC_ADINTF_ADIF_Msk          (0x1UL << ADC_ADINTF_ADIF_Pos)               /*!<ADC ADINTF: ADIF Mask */

#define ADC_ADINTF_ADLIFC_Pos        16                                           /*!<ADC ADINTF: ADLIFC Position */
#define ADC_ADINTF_ADLIFC_Msk        (0x1UL << ADC_ADINTF_ADLIFC_Pos)             /*!<ADC ADINTF: ADLIFC Mask */

#define ADC_ADINTF_ADGIFC_Pos        17                                           /*!<ADC ADINTF: ADGIFC Position */
#define ADC_ADINTF_ADGIFC_Msk        (0x1UL << ADC_ADINTF_ADGIFC_Pos)             /*!<ADC ADINTF: ADGIFC Mask */

#define ADC_ADINTF_ADIFC_Pos         18                                           /*!<ADC ADINTF: ADIFC Position */
#define ADC_ADINTF_ADIFC_Msk         (0x1UL << ADC_ADINTF_ADIFC_Pos)              /*!<ADC ADINTF: ADIFC Mask */

#define ADC_ADINTF_Msk_ALL           (ADC_ADINTF_ADLIF_Msk         |\
                                      ADC_ADINTF_ADGIF_Msk         |\
                                      ADC_ADINTF_ADIF_Msk          |\
                                      ADC_ADINTF_ADLIFC_Msk        |\
                                      ADC_ADINTF_ADGIFC_Msk        |\
                                      ADC_ADINTF_ADIFC_Msk         )

#define QEI_QEICON_QPMOD_Pos         0                                            /*!<QEI QEICON: QPMOD Position */
#define QEI_QEICON_QPMOD_Msk         (0x3UL /*<< QEI_QEICON_QPMOD_Pos*/)          /*!<QEI QEICON: QPMOD Mask */

#define QEI_QEICON_QTSR_Pos          2                                            /*!<QEI QEICON: QTSR Position */
#define QEI_QEICON_QTSR_Msk          (0x1UL << QEI_QEICON_QTSR_Pos)               /*!<QEI QEICON: QTSR Mask */

#define QEI_QEICON_QEIEN_Pos         3                                            /*!<QEI QEICON: QEIEN Position */
#define QEI_QEICON_QEIEN_Msk         (0x1UL << QEI_QEICON_QEIEN_Pos)              /*!<QEI QEICON: QEIEN Mask */

#define QEI_QEICON_QIDXEN_Pos        4                                            /*!<QEI QEICON: QIDXEN Position */
#define QEI_QEICON_QIDXEN_Msk        (0x1UL << QEI_QEICON_QIDXEN_Pos)             /*!<QEI QEICON: QIDXEN Mask */

#define QEI_QEICON_QSWAP_Pos         5                                            /*!<QEI QEICON: QSWAP Position */
#define QEI_QEICON_QSWAP_Msk         (0x1UL << QEI_QEICON_QSWAP_Pos)              /*!<QEI QEICON: QSWAP Mask */

#define QEI_QEICON_QIDXS_Pos         6                                            /*!<QEI QEICON: QIDXS Position */
#define QEI_QEICON_QIDXS_Msk         (0x1UL << QEI_QEICON_QIDXS_Pos)              /*!<QEI QEICON: QIDXS Mask */

#define QEI_QEICON_QPDIR_Pos         7                                            /*!<QEI QEICON: QPDIR Position */
#define QEI_QEICON_QPDIR_Msk         (0x1UL << QEI_QEICON_QPDIR_Pos)              /*!<QEI QEICON: QPDIR Mask */

#define QEI_QEICON_QTEPS_Pos         8                                            /*!<QEI QEICON: QTEPS Position */
#define QEI_QEICON_QTEPS_Msk         (0xFUL << QEI_QEICON_QTEPS_Pos)              /*!<QEI QEICON: QTEPS Mask */

#define QEI_QEICON_Msk_ALL           (QEI_QEICON_QPMOD_Msk         |\
                                      QEI_QEICON_QTSR_Msk          |\
                                      QEI_QEICON_QEIEN_Msk         |\
                                      QEI_QEICON_QIDXEN_Msk        |\
                                      QEI_QEICON_QSWAP_Msk         |\
                                      QEI_QEICON_QIDXS_Msk         |\
                                      QEI_QEICON_QPDIR_Msk         |\
                                      QEI_QEICON_QTEPS_Msk         )

#define QEI_QFLTCON_QABCPS_Pos       0                                            /*!<QEI QFLTCON: QABCPS Position */
#define QEI_QFLTCON_QABCPS_Msk       (0x7UL /*<< QEI_QFLTCON_QABCPS_Pos*/)        /*!<QEI QFLTCON: QABCPS Mask */

#define QEI_QFLTCON_QABFEN_Pos       3                                            /*!<QEI QFLTCON: QABFEN Position */
#define QEI_QFLTCON_QABFEN_Msk       (0x1UL << QEI_QFLTCON_QABFEN_Pos)            /*!<QEI QFLTCON: QABFEN Mask */

#define QEI_QFLTCON_QIDXCPS_Pos      4                                            /*!<QEI QFLTCON: QIDXCPS Position */
#define QEI_QFLTCON_QIDXCPS_Msk      (0x7UL << QEI_QFLTCON_QIDXCPS_Pos)           /*!<QEI QFLTCON: QIDXCPS Mask */

#define QEI_QFLTCON_QIDXFEN_Pos      7                                            /*!<QEI QFLTCON: QIDXFEN Position */
#define QEI_QFLTCON_QIDXFEN_Msk      (0x1UL << QEI_QFLTCON_QIDXFEN_Pos)           /*!<QEI QFLTCON: QIDXFEN Mask */

#define QEI_QFLTCON_Msk_ALL          (QEI_QFLTCON_QABCPS_Msk       |\
                                      QEI_QFLTCON_QABFEN_Msk       |\
                                      QEI_QFLTCON_QIDXCPS_Msk      |\
                                      QEI_QFLTCON_QIDXFEN_Msk      )

#define QEI_QPOSCNT_QPOSCNT_Pos      0                                            /*!<QEI QPOSCNT: QPOSCNT Position */
#define QEI_QPOSCNT_QPOSCNT_Msk      (0xFFFFUL /*<< QEI_QPOSCNT_QPOSCNT_Pos*/)    /*!<QEI QPOSCNT: QPOSCNT Mask */

#define QEI_QPOSCNT_Msk_ALL          (QEI_QPOSCNT_QPOSCNT_Msk     )

#define QEI_QPOSMAX_QPOSMAX_Pos      0                                            /*!<QEI QPOSMAX: QPOSMAX Position */
#define QEI_QPOSMAX_QPOSMAX_Msk      (0xFFFFUL /*<< QEI_QPOSMAX_QPOSMAX_Pos*/)    /*!<QEI QPOSMAX: QPOSMAX Mask */

#define QEI_QPOSMAX_Msk_ALL          (QEI_QPOSMAX_QPOSMAX_Msk     )

#define QEI_QCNTMIN_QCNTMIN_Pos      0                                            /*!<QEI QCNTMIN: QCNTMIN Position */
#define QEI_QCNTMIN_QCNTMIN_Msk      (0xFFFFUL /*<< QEI_QCNTMIN_QCNTMIN_Pos*/)    /*!<QEI QCNTMIN: QCNTMIN Mask */

#define QEI_QCNTMIN_Msk_ALL          (QEI_QCNTMIN_QCNTMIN_Msk     )

#define QEI_QTPSQ_QTPSQ_Pos          0                                            /*!<QEI QTPSQ: QTPSQ Position */
#define QEI_QTPSQ_QTPSQ_Msk          (0xFFFFUL /*<< QEI_QTPSQ_QTPSQ_Pos*/)        /*!<QEI QTPSQ: QTPSQ Mask */

#define QEI_QTPSQ_Msk_ALL            (QEI_QTPSQ_QTPSQ_Msk         )

#define QEI_QEIINT_QPEIE_Pos         0                                            /*!<QEI QEIINT: QPEIE Position */
#define QEI_QEIINT_QPEIE_Msk         (0x1UL /*<< QEI_QEIINT_QPEIE_Pos*/)          /*!<QEI QEIINT: QPEIE Mask */

#define QEI_QEIINT_QCEIE_Pos         1                                            /*!<QEI QEIINT: QCEIE Position */
#define QEI_QEIINT_QCEIE_Msk         (0x1UL << QEI_QEIINT_QCEIE_Pos)              /*!<QEI QEIINT: QCEIE Mask */

#define QEI_QEIINT_QEIIE_Pos         2                                            /*!<QEI QEIINT: QEIIE Position */
#define QEI_QEIINT_QEIIE_Msk         (0x1UL << QEI_QEIINT_QEIIE_Pos)              /*!<QEI QEIINT: QEIIE Mask */

#define QEI_QEIINT_QTCAPIE_Pos       3                                            /*!<QEI QEIINT: QTCAPIE Position */
#define QEI_QEIINT_QTCAPIE_Msk       (0x1UL << QEI_QEIINT_QTCAPIE_Pos)            /*!<QEI QEIINT: QTCAPIE Mask */

#define QEI_QEIINT_QTIE_Pos          4                                            /*!<QEI QEIINT: QTIE Position */
#define QEI_QEIINT_QTIE_Msk          (0x1UL << QEI_QEIINT_QTIE_Pos)               /*!<QEI QEIINT: QTIE Mask */

#define QEI_QEIINT_QPEDE_Pos         5                                            /*!<QEI QEIINT: QPEDE Position */
#define QEI_QEIINT_QPEDE_Msk         (0x1UL << QEI_QEIINT_QPEDE_Pos)              /*!<QEI QEIINT: QPEDE Mask */

#define QEI_QEIINT_QCEDE_Pos         6                                            /*!<QEI QEIINT: QCEDE Position */
#define QEI_QEIINT_QCEDE_Msk         (0x1UL << QEI_QEIINT_QCEDE_Pos)              /*!<QEI QEIINT: QCEDE Mask */

#define QEI_QEIINT_QEIDE_Pos         7                                            /*!<QEI QEIINT: QEIDE Position */
#define QEI_QEIINT_QEIDE_Msk         (0x1UL << QEI_QEIINT_QEIDE_Pos)              /*!<QEI QEIINT: QEIDE Mask */

#define QEI_QEIINT_QTCAPDE_Pos       8                                            /*!<QEI QEIINT: QTCAPDE Position */
#define QEI_QEIINT_QTCAPDE_Msk       (0x1UL << QEI_QEIINT_QTCAPDE_Pos)            /*!<QEI QEIINT: QTCAPDE Mask */

#define QEI_QEIINT_QTDE_Pos          9                                            /*!<QEI QEIINT: QTDE Position */
#define QEI_QEIINT_QTDE_Msk          (0x1UL << QEI_QEIINT_QTDE_Pos)               /*!<QEI QEIINT: QTDE Mask */

#define QEI_QEIINT_Msk_ALL           (QEI_QEIINT_QPEIE_Msk         |\
                                      QEI_QEIINT_QCEIE_Msk         |\
                                      QEI_QEIINT_QEIIE_Msk         |\
                                      QEI_QEIINT_QTCAPIE_Msk       |\
                                      QEI_QEIINT_QTIE_Msk          |\
                                      QEI_QEIINT_QPEDE_Msk         |\
                                      QEI_QEIINT_QCEDE_Msk         |\
                                      QEI_QEIINT_QEIDE_Msk         |\
                                      QEI_QEIINT_QTCAPDE_Msk       |\
                                      QEI_QEIINT_QTDE_Msk          )

#define QEI_QTINTF_QPEIF_Pos         0                                            /*!<QEI QTINTF: QPEIF Position */
#define QEI_QTINTF_QPEIF_Msk         (0x1UL /*<< QEI_QTINTF_QPEIF_Pos*/)          /*!<QEI QTINTF: QPEIF Mask */

#define QEI_QTINTF_QCEIF_Pos         1                                            /*!<QEI QTINTF: QCEIF Position */
#define QEI_QTINTF_QCEIF_Msk         (0x1UL << QEI_QTINTF_QCEIF_Pos)              /*!<QEI QTINTF: QCEIF Mask */

#define QEI_QTINTF_QEIIF_Pos         2                                            /*!<QEI QTINTF: QEIIF Position */
#define QEI_QTINTF_QEIIF_Msk         (0x1UL << QEI_QTINTF_QEIIF_Pos)              /*!<QEI QTINTF: QEIIF Mask */

#define QEI_QTINTF_QTCAPIF_Pos       3                                            /*!<QEI QTINTF: QTCAPIF Position */
#define QEI_QTINTF_QTCAPIF_Msk       (0x1UL << QEI_QTINTF_QTCAPIF_Pos)            /*!<QEI QTINTF: QTCAPIF Mask */

#define QEI_QTINTF_QTIF_Pos          4                                            /*!<QEI QTINTF: QTIF Position */
#define QEI_QTINTF_QTIF_Msk          (0x1UL << QEI_QTINTF_QTIF_Pos)               /*!<QEI QTINTF: QTIF Mask */

#define QEI_QTINTF_QPEIFC_Pos        16                                           /*!<QEI QTINTF: QPEIFC Position */
#define QEI_QTINTF_QPEIFC_Msk        (0x1UL << QEI_QTINTF_QPEIFC_Pos)             /*!<QEI QTINTF: QPEIFC Mask */

#define QEI_QTINTF_QCEIFC_Pos        17                                           /*!<QEI QTINTF: QCEIFC Position */
#define QEI_QTINTF_QCEIFC_Msk        (0x1UL << QEI_QTINTF_QCEIFC_Pos)             /*!<QEI QTINTF: QCEIFC Mask */

#define QEI_QTINTF_QEIIFC_Pos        18                                           /*!<QEI QTINTF: QEIIFC Position */
#define QEI_QTINTF_QEIIFC_Msk        (0x1UL << QEI_QTINTF_QEIIFC_Pos)             /*!<QEI QTINTF: QEIIFC Mask */

#define QEI_QTINTF_QTCAPIFC_Pos      19                                           /*!<QEI QTINTF: QTCAPIFC Position */
#define QEI_QTINTF_QTCAPIFC_Msk      (0x1UL << QEI_QTINTF_QTCAPIFC_Pos)           /*!<QEI QTINTF: QTCAPIFC Mask */

#define QEI_QTINTF_QTIFC_Pos         20                                           /*!<QEI QTINTF: QTIFC Position */
#define QEI_QTINTF_QTIFC_Msk         (0x1UL << QEI_QTINTF_QTIFC_Pos)              /*!<QEI QTINTF: QTIFC Mask */

#define QEI_QTINTF_Msk_ALL           (QEI_QTINTF_QPEIF_Msk         |\
                                      QEI_QTINTF_QCEIF_Msk         |\
                                      QEI_QTINTF_QEIIF_Msk         |\
                                      QEI_QTINTF_QTCAPIF_Msk       |\
                                      QEI_QTINTF_QTIF_Msk          |\
                                      QEI_QTINTF_QPEIFC_Msk        |\
                                      QEI_QTINTF_QCEIFC_Msk        |\
                                      QEI_QTINTF_QEIIFC_Msk        |\
                                      QEI_QTINTF_QTCAPIFC_Msk      |\
                                      QEI_QTINTF_QTIFC_Msk         )

#define AMOC_CMP1CON_C1DEB_Pos       0                                            /*!<AMOC CMP1CON: C1DEB Position */
#define AMOC_CMP1CON_C1DEB_Msk       (0x7UL /*<< AMOC_CMP1CON_C1DEB_Pos*/)        /*!<AMOC CMP1CON: C1DEB Mask */

#define AMOC_CMP1CON_C1PCHS_Pos      3                                            /*!<AMOC CMP1CON: C1PCHS Position */
#define AMOC_CMP1CON_C1PCHS_Msk      (0x3UL << AMOC_CMP1CON_C1PCHS_Pos)           /*!<AMOC CMP1CON: C1PCHS Mask */

#define AMOC_CMP1CON_C1NCHS_Pos      6                                            /*!<AMOC CMP1CON: C1NCHS Position */
#define AMOC_CMP1CON_C1NCHS_Msk      (0x1UL << AMOC_CMP1CON_C1NCHS_Pos)           /*!<AMOC CMP1CON: C1NCHS Mask */

#define AMOC_CMP1CON_CMP1EN_Pos      7                                            /*!<AMOC CMP1CON: CMP1EN Position */
#define AMOC_CMP1CON_CMP1EN_Msk      (0x1UL << AMOC_CMP1CON_CMP1EN_Pos)           /*!<AMOC CMP1CON: CMP1EN Mask */

#define AMOC_CMP1CON_C1SMT_Pos       8                                            /*!<AMOC CMP1CON: C1SMT Position */
#define AMOC_CMP1CON_C1SMT_Msk       (0x3UL << AMOC_CMP1CON_C1SMT_Pos)            /*!<AMOC CMP1CON: C1SMT Mask */

#define AMOC_CMP1CON_C1OUT_Pos       10                                           /*!<AMOC CMP1CON: C1OUT Position */
#define AMOC_CMP1CON_C1OUT_Msk       (0x1UL << AMOC_CMP1CON_C1OUT_Pos)            /*!<AMOC CMP1CON: C1OUT Mask */

#define AMOC_CMP1CON_C1OUTEN_Pos     11                                           /*!<AMOC CMP1CON: C1OUTEN Position */
#define AMOC_CMP1CON_C1OUTEN_Msk     (0x1UL << AMOC_CMP1CON_C1OUTEN_Pos)          /*!<AMOC CMP1CON: C1OUTEN Mask */

#define AMOC_CMP1CON_C1IES_Pos       13                                           /*!<AMOC CMP1CON: C1IES Position */
#define AMOC_CMP1CON_C1IES_Msk       (0x3UL << AMOC_CMP1CON_C1IES_Pos)            /*!<AMOC CMP1CON: C1IES Mask */

#define AMOC_CMP1CON_C1DE_Pos        15                                           /*!<AMOC CMP1CON: C1DE Position */
#define AMOC_CMP1CON_C1DE_Msk        (0x1UL << AMOC_CMP1CON_C1DE_Pos)             /*!<AMOC CMP1CON: C1DE Mask */

#define AMOC_CMP1CON_CMP1VRS_Pos     16                                           /*!<AMOC CMP1CON: CMP1VRS Position */
#define AMOC_CMP1CON_CMP1VRS_Msk     (0x7UL << AMOC_CMP1CON_CMP1VRS_Pos)          /*!<AMOC CMP1CON: CMP1VRS Mask */

#define AMOC_CMP1CON_CMP1VCMP_Pos    19                                           /*!<AMOC CMP1CON: CMP1VCMP Position */
#define AMOC_CMP1CON_CMP1VCMP_Msk    (0x1UL << AMOC_CMP1CON_CMP1VCMP_Pos)         /*!<AMOC CMP1CON: CMP1VCMP Mask */

#define AMOC_CMP1CON_Msk_ALL         (AMOC_CMP1CON_C1DEB_Msk       |\
                                      AMOC_CMP1CON_C1PCHS_Msk      |\
                                      AMOC_CMP1CON_C1NCHS_Msk      |\
                                      AMOC_CMP1CON_CMP1EN_Msk      |\
                                      AMOC_CMP1CON_C1SMT_Msk       |\
                                      AMOC_CMP1CON_C1OUT_Msk       |\
                                      AMOC_CMP1CON_C1OUTEN_Msk     |\
                                      AMOC_CMP1CON_C1IES_Msk       |\
                                      AMOC_CMP1CON_C1DE_Msk        |\
                                      AMOC_CMP1CON_CMP1VRS_Msk     |\
                                      AMOC_CMP1CON_CMP1VCMP_Msk    )

#define AMOC_CMP2CON_C2DEB_Pos       0                                            /*!<AMOC CMP2CON: C2DEB Position */
#define AMOC_CMP2CON_C2DEB_Msk       (0x7UL /*<< AMOC_CMP2CON_C2DEB_Pos*/)        /*!<AMOC CMP2CON: C2DEB Mask */

#define AMOC_CMP2CON_C2PCHS_Pos      3                                            /*!<AMOC CMP2CON: C2PCHS Position */
#define AMOC_CMP2CON_C2PCHS_Msk      (0x7UL << AMOC_CMP2CON_C2PCHS_Pos)           /*!<AMOC CMP2CON: C2PCHS Mask */

#define AMOC_CMP2CON_C2NCHS_Pos      6                                            /*!<AMOC CMP2CON: C2NCHS Position */
#define AMOC_CMP2CON_C2NCHS_Msk      (0x1UL << AMOC_CMP2CON_C2NCHS_Pos)           /*!<AMOC CMP2CON: C2NCHS Mask */

#define AMOC_CMP2CON_CMP2EN_Pos      7                                            /*!<AMOC CMP2CON: CMP2EN Position */
#define AMOC_CMP2CON_CMP2EN_Msk      (0x1UL << AMOC_CMP2CON_CMP2EN_Pos)           /*!<AMOC CMP2CON: CMP2EN Mask */

#define AMOC_CMP2CON_C2SMT_Pos       8                                            /*!<AMOC CMP2CON: C2SMT Position */
#define AMOC_CMP2CON_C2SMT_Msk       (0x3UL << AMOC_CMP2CON_C2SMT_Pos)            /*!<AMOC CMP2CON: C2SMT Mask */

#define AMOC_CMP2CON_C2OUT_Pos       10                                           /*!<AMOC CMP2CON: C2OUT Position */
#define AMOC_CMP2CON_C2OUT_Msk       (0x1UL << AMOC_CMP2CON_C2OUT_Pos)            /*!<AMOC CMP2CON: C2OUT Mask */

#define AMOC_CMP2CON_C2OUTEN_Pos     11                                           /*!<AMOC CMP2CON: C2OUTEN Position */
#define AMOC_CMP2CON_C2OUTEN_Msk     (0x1UL << AMOC_CMP2CON_C2OUTEN_Pos)          /*!<AMOC CMP2CON: C2OUTEN Mask */

#define AMOC_CMP2CON_C2IES_Pos       13                                           /*!<AMOC CMP2CON: C2IES Position */
#define AMOC_CMP2CON_C2IES_Msk       (0x3UL << AMOC_CMP2CON_C2IES_Pos)            /*!<AMOC CMP2CON: C2IES Mask */

#define AMOC_CMP2CON_C2DE_Pos        15                                           /*!<AMOC CMP2CON: C2DE Position */
#define AMOC_CMP2CON_C2DE_Msk        (0x1UL << AMOC_CMP2CON_C2DE_Pos)             /*!<AMOC CMP2CON: C2DE Mask */

#define AMOC_CMP2CON_CMP2VRS_Pos     16                                           /*!<AMOC CMP2CON: CMP2VRS Position */
#define AMOC_CMP2CON_CMP2VRS_Msk     (0x7UL << AMOC_CMP2CON_CMP2VRS_Pos)          /*!<AMOC CMP2CON: CMP2VRS Mask */

#define AMOC_CMP2CON_CMP2VCMP_Pos    19                                           /*!<AMOC CMP2CON: CMP2VCMP Position */
#define AMOC_CMP2CON_CMP2VCMP_Msk    (0x1UL << AMOC_CMP2CON_CMP2VCMP_Pos)         /*!<AMOC CMP2CON: CMP2VCMP Mask */

#define AMOC_CMP2CON_Msk_ALL         (AMOC_CMP2CON_C2DEB_Msk       |\
                                      AMOC_CMP2CON_C2PCHS_Msk      |\
                                      AMOC_CMP2CON_C2NCHS_Msk      |\
                                      AMOC_CMP2CON_CMP2EN_Msk      |\
                                      AMOC_CMP2CON_C2SMT_Msk       |\
                                      AMOC_CMP2CON_C2OUT_Msk       |\
                                      AMOC_CMP2CON_C2OUTEN_Msk     |\
                                      AMOC_CMP2CON_C2IES_Msk       |\
                                      AMOC_CMP2CON_C2DE_Msk        |\
                                      AMOC_CMP2CON_CMP2VRS_Msk     |\
                                      AMOC_CMP2CON_CMP2VCMP_Msk    )

#define AMOC_CMP3CON_C3DEB_Pos       0                                            /*!<AMOC CMP3CON: C3DEB Position */
#define AMOC_CMP3CON_C3DEB_Msk       (0x7UL /*<< AMOC_CMP3CON_C3DEB_Pos*/)        /*!<AMOC CMP3CON: C3DEB Mask */

#define AMOC_CMP3CON_C3PCHS_Pos      3                                            /*!<AMOC CMP3CON: C3PCHS Position */
#define AMOC_CMP3CON_C3PCHS_Msk      (0x7UL << AMOC_CMP3CON_C3PCHS_Pos)           /*!<AMOC CMP3CON: C3PCHS Mask */

#define AMOC_CMP3CON_C3NCHS_Pos      6                                            /*!<AMOC CMP3CON: C3NCHS Position */
#define AMOC_CMP3CON_C3NCHS_Msk      (0x1UL << AMOC_CMP3CON_C3NCHS_Pos)           /*!<AMOC CMP3CON: C3NCHS Mask */

#define AMOC_CMP3CON_CMP3EN_Pos      7                                            /*!<AMOC CMP3CON: CMP3EN Position */
#define AMOC_CMP3CON_CMP3EN_Msk      (0x1UL << AMOC_CMP3CON_CMP3EN_Pos)           /*!<AMOC CMP3CON: CMP3EN Mask */

#define AMOC_CMP3CON_C3SMT_Pos       8                                            /*!<AMOC CMP3CON: C3SMT Position */
#define AMOC_CMP3CON_C3SMT_Msk       (0x3UL << AMOC_CMP3CON_C3SMT_Pos)            /*!<AMOC CMP3CON: C3SMT Mask */

#define AMOC_CMP3CON_C3OUT_Pos       10                                           /*!<AMOC CMP3CON: C3OUT Position */
#define AMOC_CMP3CON_C3OUT_Msk       (0x1UL << AMOC_CMP3CON_C3OUT_Pos)            /*!<AMOC CMP3CON: C3OUT Mask */

#define AMOC_CMP3CON_C3OUTEN_Pos     11                                           /*!<AMOC CMP3CON: C3OUTEN Position */
#define AMOC_CMP3CON_C3OUTEN_Msk     (0x1UL << AMOC_CMP3CON_C3OUTEN_Pos)          /*!<AMOC CMP3CON: C3OUTEN Mask */

#define AMOC_CMP3CON_C3IES_Pos       13                                           /*!<AMOC CMP3CON: C3IES Position */
#define AMOC_CMP3CON_C3IES_Msk       (0x3UL << AMOC_CMP3CON_C3IES_Pos)            /*!<AMOC CMP3CON: C3IES Mask */

#define AMOC_CMP3CON_C3DE_Pos        15                                           /*!<AMOC CMP3CON: C3DE Position */
#define AMOC_CMP3CON_C3DE_Msk        (0x1UL << AMOC_CMP3CON_C3DE_Pos)             /*!<AMOC CMP3CON: C3DE Mask */

#define AMOC_CMP3CON_CMP3VRS_Pos     16                                           /*!<AMOC CMP3CON: CMP3VRS Position */
#define AMOC_CMP3CON_CMP3VRS_Msk     (0x7UL << AMOC_CMP3CON_CMP3VRS_Pos)          /*!<AMOC CMP3CON: CMP3VRS Mask */

#define AMOC_CMP3CON_CMP3VCMP_Pos    19                                           /*!<AMOC CMP3CON: CMP3VCMP Position */
#define AMOC_CMP3CON_CMP3VCMP_Msk    (0x1UL << AMOC_CMP3CON_CMP3VCMP_Pos)         /*!<AMOC CMP3CON: CMP3VCMP Mask */

#define AMOC_CMP3CON_Msk_ALL         (AMOC_CMP3CON_C3DEB_Msk       |\
                                      AMOC_CMP3CON_C3PCHS_Msk      |\
                                      AMOC_CMP3CON_C3NCHS_Msk      |\
                                      AMOC_CMP3CON_CMP3EN_Msk      |\
                                      AMOC_CMP3CON_C3SMT_Msk       |\
                                      AMOC_CMP3CON_C3OUT_Msk       |\
                                      AMOC_CMP3CON_C3OUTEN_Msk     |\
                                      AMOC_CMP3CON_C3IES_Msk       |\
                                      AMOC_CMP3CON_C3DE_Msk        |\
                                      AMOC_CMP3CON_CMP3VRS_Msk     |\
                                      AMOC_CMP3CON_CMP3VCMP_Msk    )

#define AMOC_CMPINTF_C1IF_Pos        0                                            /*!<AMOC CMPINTF: C1IF Position */
#define AMOC_CMPINTF_C1IF_Msk        (0x1UL /*<< AMOC_CMPINTF_C1IF_Pos*/)         /*!<AMOC CMPINTF: C1IF Mask */

#define AMOC_CMPINTF_C2IF_Pos        1                                            /*!<AMOC CMPINTF: C2IF Position */
#define AMOC_CMPINTF_C2IF_Msk        (0x1UL << AMOC_CMPINTF_C2IF_Pos)             /*!<AMOC CMPINTF: C2IF Mask */

#define AMOC_CMPINTF_C3IF_Pos        2                                            /*!<AMOC CMPINTF: C3IF Position */
#define AMOC_CMPINTF_C3IF_Msk        (0x1UL << AMOC_CMPINTF_C3IF_Pos)             /*!<AMOC CMPINTF: C3IF Mask */

#define AMOC_CMPINTF_C1IFC_Pos       16                                           /*!<AMOC CMPINTF: C1IFC Position */
#define AMOC_CMPINTF_C1IFC_Msk       (0x1UL << AMOC_CMPINTF_C1IFC_Pos)            /*!<AMOC CMPINTF: C1IFC Mask */

#define AMOC_CMPINTF_C2IFC_Pos       17                                           /*!<AMOC CMPINTF: C2IFC Position */
#define AMOC_CMPINTF_C2IFC_Msk       (0x1UL << AMOC_CMPINTF_C2IFC_Pos)            /*!<AMOC CMPINTF: C2IFC Mask */

#define AMOC_CMPINTF_C3IFC_Pos       18                                           /*!<AMOC CMPINTF: C3IFC Position */
#define AMOC_CMPINTF_C3IFC_Msk       (0x1UL << AMOC_CMPINTF_C3IFC_Pos)            /*!<AMOC CMPINTF: C3IFC Mask */

#define AMOC_CMPINTF_Msk_ALL         (AMOC_CMPINTF_C1IF_Msk        |\
                                      AMOC_CMPINTF_C2IF_Msk        |\
                                      AMOC_CMPINTF_C3IF_Msk        |\
                                      AMOC_CMPINTF_C1IFC_Msk       |\
                                      AMOC_CMPINTF_C2IFC_Msk       |\
                                      AMOC_CMPINTF_C3IFC_Msk       )

#define AMOC_OPCON_OP1EN_Pos         0                                            /*!<AMOC OPCON: OP1EN Position */
#define AMOC_OPCON_OP1EN_Msk         (0x1UL /*<< AMOC_OPCON_OP1EN_Pos*/)          /*!<AMOC OPCON: OP1EN Mask */

#define AMOC_OPCON_OP2EN_Pos         1                                            /*!<AMOC OPCON: OP2EN Position */
#define AMOC_OPCON_OP2EN_Msk         (0x1UL << AMOC_OPCON_OP2EN_Pos)              /*!<AMOC OPCON: OP2EN Mask */

#define AMOC_OPCON_OP3EN_Pos         2                                            /*!<AMOC OPCON: OP3EN Position */
#define AMOC_OPCON_OP3EN_Msk         (0x1UL << AMOC_OPCON_OP3EN_Pos)              /*!<AMOC OPCON: OP3EN Mask */

#define AMOC_OPCON_Msk_ALL           (AMOC_OPCON_OP1EN_Msk         |\
                                      AMOC_OPCON_OP2EN_Msk         |\
                                      AMOC_OPCON_OP3EN_Msk         )

#define AMOC_AVREFCON_VREFEN_Pos     0                                            /*!<AMOC AVREFCON: VREFEN Position */
#define AMOC_AVREFCON_VREFEN_Msk     (0x1UL /*<< AMOC_AVREFCON_VREFEN_Pos*/)      /*!<AMOC AVREFCON: VREFEN Mask */

#define AMOC_AVREFCON_VREFSEL_Pos    1                                            /*!<AMOC AVREFCON: VREFSEL Position */
#define AMOC_AVREFCON_VREFSEL_Msk    (0x1UL << AMOC_AVREFCON_VREFSEL_Pos)         /*!<AMOC AVREFCON: VREFSEL Mask */

#define AMOC_AVREFCON_Msk_ALL        (AMOC_AVREFCON_VREFEN_Msk     |\
                                      AMOC_AVREFCON_VREFSEL_Msk    )

#define AMOC_TPSCON_TPSEN_Pos        0                                            /*!<AMOC TPSCON: TPSEN Position */
#define AMOC_TPSCON_TPSEN_Msk        (0x1UL /*<< AMOC_TPSCON_TPSEN_Pos*/)         /*!<AMOC TPSCON: TPSEN Mask */

#define AMOC_TPSCON_TPSCHOP_Pos      1                                            /*!<AMOC TPSCON: TPSCHOP Position */
#define AMOC_TPSCON_TPSCHOP_Msk      (0x1UL << AMOC_TPSCON_TPSCHOP_Pos)           /*!<AMOC TPSCON: TPSCHOP Mask */

#define AMOC_TPSCON_Msk_ALL          (AMOC_TPSCON_TPSEN_Msk        |\
                                      AMOC_TPSCON_TPSCHOP_Msk      )

#define UART_FR_RI_Pos               0                                            /*!<UART FR: RI Position */
#define UART_FR_RI_Msk               (0x1UL /*<< UART_FR_RI_Pos*/)                /*!<UART FR: RI Mask */

#define UART_FR_TI_Pos               1                                            /*!<UART FR: TI Position */
#define UART_FR_TI_Msk               (0x1UL << UART_FR_TI_Pos)                    /*!<UART FR: TI Mask */

#define UART_FR_TC_Pos               2                                            /*!<UART FR: TC Position */
#define UART_FR_TC_Msk               (0x1UL << UART_FR_TC_Pos)                    /*!<UART FR: TC Mask */

#define UART_FR_TXCOL_Pos            3                                            /*!<UART FR: TXCOL Position */
#define UART_FR_TXCOL_Msk            (0x1UL << UART_FR_TXCOL_Pos)                 /*!<UART FR: TXCOL Mask */

#define UART_FR_RXOV_Pos             4                                            /*!<UART FR: RXOV Position */
#define UART_FR_RXOV_Msk             (0x1UL << UART_FR_RXOV_Pos)                  /*!<UART FR: RXOV Mask */

#define UART_FR_FE_Pos               5                                            /*!<UART FR: FE Position */
#define UART_FR_FE_Msk               (0x1UL << UART_FR_FE_Pos)                    /*!<UART FR: FE Mask */

#define UART_FR_PE_Pos               6                                            /*!<UART FR: PE Position */
#define UART_FR_PE_Msk               (0x1UL << UART_FR_PE_Pos)                    /*!<UART FR: PE Mask */

#define UART_FR_LBD_Pos              7                                            /*!<UART FR: LBD Position */
#define UART_FR_LBD_Msk              (0x1UL << UART_FR_LBD_Pos)                   /*!<UART FR: LBD Mask */

#define UART_FR_RIC_Pos              16                                           /*!<UART FR: RIC Position */
#define UART_FR_RIC_Msk              (0x1UL << UART_FR_RIC_Pos)                   /*!<UART FR: RIC Mask */

#define UART_FR_TCC_Pos              18                                           /*!<UART FR: TCC Position */
#define UART_FR_TCC_Msk              (0x1UL << UART_FR_TCC_Pos)                   /*!<UART FR: TCC Mask */

#define UART_FR_TXCOLC_Pos           19                                           /*!<UART FR: TXCOLC Position */
#define UART_FR_TXCOLC_Msk           (0x1UL << UART_FR_TXCOLC_Pos)                /*!<UART FR: TXCOLC Mask */

#define UART_FR_RXOVC_Pos            20                                           /*!<UART FR: RXOVC Position */
#define UART_FR_RXOVC_Msk            (0x1UL << UART_FR_RXOVC_Pos)                 /*!<UART FR: RXOVC Mask */

#define UART_FR_FEC_Pos              21                                           /*!<UART FR: FEC Position */
#define UART_FR_FEC_Msk              (0x1UL << UART_FR_FEC_Pos)                   /*!<UART FR: FEC Mask */

#define UART_FR_PEC_Pos              22                                           /*!<UART FR: PEC Position */
#define UART_FR_PEC_Msk              (0x1UL << UART_FR_PEC_Pos)                   /*!<UART FR: PEC Mask */

#define UART_FR_LBDC_Pos             23                                           /*!<UART FR: LBDC Position */
#define UART_FR_LBDC_Msk             (0x1UL << UART_FR_LBDC_Pos)                  /*!<UART FR: LBDC Mask */

#define UART_FR_Msk_ALL              (UART_FR_RI_Msk               |\
                                      UART_FR_TI_Msk               |\
                                      UART_FR_TC_Msk               |\
                                      UART_FR_TXCOL_Msk            |\
                                      UART_FR_RXOV_Msk             |\
                                      UART_FR_FE_Msk               |\
                                      UART_FR_PE_Msk               |\
                                      UART_FR_LBD_Msk              |\
                                      UART_FR_RIC_Msk              |\
                                      UART_FR_TCC_Msk              |\
                                      UART_FR_TXCOLC_Msk           |\
                                      UART_FR_RXOVC_Msk            |\
                                      UART_FR_FEC_Msk              |\
                                      UART_FR_PEC_Msk              |\
                                      UART_FR_LBDC_Msk             )

#define UART_TDR_TDR_Pos             0                                            /*!<UART TDR: TDR Position */
#define UART_TDR_TDR_Msk             (0xFFUL /*<< UART_TDR_TDR_Pos*/)             /*!<UART TDR: TDR Mask */

#define UART_TDR_Msk_ALL             (UART_TDR_TDR_Msk            )

#define UART_RDR_RDR_Pos             0                                            /*!<UART RDR: RDR Position */
#define UART_RDR_RDR_Msk             (0xFFUL /*<< UART_RDR_RDR_Pos*/)             /*!<UART RDR: RDR Mask */

#define UART_RDR_Msk_ALL             (UART_RDR_RDR_Msk            )

#define UART_ADDR_SADDR_Pos          0                                            /*!<UART ADDR: SADDR Position */
#define UART_ADDR_SADDR_Msk          (0xFFUL /*<< UART_ADDR_SADDR_Pos*/)          /*!<UART ADDR: SADDR Mask */

#define UART_ADDR_SMAR_Pos           8                                            /*!<UART ADDR: SMAR Position */
#define UART_ADDR_SMAR_Msk           (0xFFUL << UART_ADDR_SMAR_Pos)               /*!<UART ADDR: SMAR Mask */

#define UART_ADDR_Msk_ALL            (UART_ADDR_SADDR_Msk          |\
                                      UART_ADDR_SMAR_Msk           )

#define UART_BRT_SBRT_Pos            0                                            /*!<UART BRT: SBRT Position */
#define UART_BRT_SBRT_Msk            (0x7FFFUL /*<< UART_BRT_SBRT_Pos*/)          /*!<UART BRT: SBRT Mask */

#define UART_BRT_BFINE_Pos           16                                           /*!<UART BRT: BFINE Position */
#define UART_BRT_BFINE_Msk           (0xFUL << UART_BRT_BFINE_Pos)                /*!<UART BRT: BFINE Mask */

#define UART_BRT_Msk_ALL             (UART_BRT_SBRT_Msk            |\
                                      UART_BRT_BFINE_Msk           )

#define UART_CR_STOP_Pos             0                                            /*!<UART CR: STOP Position */
#define UART_CR_STOP_Msk             (0x1UL /*<< UART_CR_STOP_Pos*/)              /*!<UART CR: STOP Mask */

#define UART_CR_SBRTEN_Pos           1                                            /*!<UART CR: SBRTEN Position */
#define UART_CR_SBRTEN_Msk           (0x1UL << UART_CR_SBRTEN_Pos)                /*!<UART CR: SBRTEN Mask */

#define UART_CR_SMOD_Pos             2                                            /*!<UART CR: SMOD Position */
#define UART_CR_SMOD_Msk             (0x1UL << UART_CR_SMOD_Pos)                  /*!<UART CR: SMOD Mask */

#define UART_CR_RIE_Pos              3                                            /*!<UART CR: RIE Position */
#define UART_CR_RIE_Msk              (0x1UL << UART_CR_RIE_Pos)                   /*!<UART CR: RIE Mask */

#define UART_CR_TIE_Pos              4                                            /*!<UART CR: TIE Position */
#define UART_CR_TIE_Msk              (0x1UL << UART_CR_TIE_Pos)                   /*!<UART CR: TIE Mask */

#define UART_CR_TCIE_Pos             5                                            /*!<UART CR: TCIE Position */
#define UART_CR_TCIE_Msk             (0x1UL << UART_CR_TCIE_Pos)                  /*!<UART CR: TCIE Mask */

#define UART_CR_LBDIE_Pos            6                                            /*!<UART CR: LBDIE Position */
#define UART_CR_LBDIE_Msk            (0x1UL << UART_CR_LBDIE_Pos)                 /*!<UART CR: LBDIE Mask */

#define UART_CR_LBDL_Pos             7                                            /*!<UART CR: LBDL Position */
#define UART_CR_LBDL_Msk             (0x1UL << UART_CR_LBDL_Pos)                  /*!<UART CR: LBDL Mask */

#define UART_CR_RB8_Pos              8                                            /*!<UART CR: RB8 Position */
#define UART_CR_RB8_Msk              (0x1UL << UART_CR_RB8_Pos)                   /*!<UART CR: RB8 Mask */

#define UART_CR_TB8_Pos              9                                            /*!<UART CR: TB8 Position */
#define UART_CR_TB8_Msk              (0x1UL << UART_CR_TB8_Pos)                   /*!<UART CR: TB8 Mask */

#define UART_CR_PS_Pos               10                                           /*!<UART CR: PS Position */
#define UART_CR_PS_Msk               (0x1UL << UART_CR_PS_Pos)                    /*!<UART CR: PS Mask */

#define UART_CR_PCE_Pos              11                                           /*!<UART CR: PCE Position */
#define UART_CR_PCE_Msk              (0x1UL << UART_CR_PCE_Pos)                   /*!<UART CR: PCE Mask */

#define UART_CR_SM2_Pos              12                                           /*!<UART CR: SM2 Position */
#define UART_CR_SM2_Msk              (0x1UL << UART_CR_SM2_Pos)                   /*!<UART CR: SM2 Mask */

#define UART_CR_SM_Pos               13                                           /*!<UART CR: SM Position */
#define UART_CR_SM_Msk               (0x3UL << UART_CR_SM_Pos)                    /*!<UART CR: SM Mask */

#define UART_CR_SBK_Pos              15                                           /*!<UART CR: SBK Position */
#define UART_CR_SBK_Msk              (0x1UL << UART_CR_SBK_Pos)                   /*!<UART CR: SBK Mask */

#define UART_CR_LINEN_Pos            16                                           /*!<UART CR: LINEN Position */
#define UART_CR_LINEN_Msk            (0x1UL << UART_CR_LINEN_Pos)                 /*!<UART CR: LINEN Mask */

#define UART_CR_REN_Pos              17                                           /*!<UART CR: REN Position */
#define UART_CR_REN_Msk              (0x1UL << UART_CR_REN_Pos)                   /*!<UART CR: REN Mask */

#define UART_CR_TEN_Pos              18                                           /*!<UART CR: TEN Position */
#define UART_CR_TEN_Msk              (0x1UL << UART_CR_TEN_Pos)                   /*!<UART CR: TEN Mask */

#define UART_CR_DMAR_Pos             19                                           /*!<UART CR: DMAR Position */
#define UART_CR_DMAR_Msk             (0x1UL << UART_CR_DMAR_Pos)                  /*!<UART CR: DMAR Mask */

#define UART_CR_DMAT_Pos             20                                           /*!<UART CR: DMAT Position */
#define UART_CR_DMAT_Msk             (0x1UL << UART_CR_DMAT_Pos)                  /*!<UART CR: DMAT Mask */

#define UART_CR_Msk_ALL              (UART_CR_STOP_Msk             |\
                                      UART_CR_SBRTEN_Msk           |\
                                      UART_CR_SMOD_Msk             |\
                                      UART_CR_RIE_Msk              |\
                                      UART_CR_TIE_Msk              |\
                                      UART_CR_TCIE_Msk             |\
                                      UART_CR_LBDIE_Msk            |\
                                      UART_CR_LBDL_Msk             |\
                                      UART_CR_RB8_Msk              |\
                                      UART_CR_TB8_Msk              |\
                                      UART_CR_PS_Msk               |\
                                      UART_CR_PCE_Msk              |\
                                      UART_CR_SM2_Msk              |\
                                      UART_CR_SM_Msk               |\
                                      UART_CR_SBK_Msk              |\
                                      UART_CR_LINEN_Msk            |\
                                      UART_CR_REN_Msk              |\
                                      UART_CR_TEN_Msk              |\
                                      UART_CR_DMAR_Msk             |\
                                      UART_CR_DMAT_Msk             )

#define SPI_FR_SPRI_Pos              0                                            /*!<SPI FR: SPRI Position */
#define SPI_FR_SPRI_Msk              (0x1UL /*<< SPI_FR_SPRI_Pos*/)               /*!<SPI FR: SPRI Mask */

#define SPI_FR_SPTI_Pos              1                                            /*!<SPI FR: SPTI Position */
#define SPI_FR_SPTI_Msk              (0x1UL << SPI_FR_SPTI_Pos)                   /*!<SPI FR: SPTI Mask */

#define SPI_FR_MODF_Pos              3                                            /*!<SPI FR: MODF Position */
#define SPI_FR_MODF_Msk              (0x1UL << SPI_FR_MODF_Pos)                   /*!<SPI FR: MODF Mask */

#define SPI_FR_RXOV_Pos              4                                            /*!<SPI FR: RXOV Position */
#define SPI_FR_RXOV_Msk              (0x1UL << SPI_FR_RXOV_Pos)                   /*!<SPI FR: RXOV Mask */

#define SPI_FR_WCOL_Pos              5                                            /*!<SPI FR: WCOL Position */
#define SPI_FR_WCOL_Msk              (0x1UL << SPI_FR_WCOL_Pos)                   /*!<SPI FR: WCOL Mask */

#define SPI_FR_SPRIC_Pos             16                                           /*!<SPI FR: SPRIC Position */
#define SPI_FR_SPRIC_Msk             (0x1UL << SPI_FR_SPRIC_Pos)                  /*!<SPI FR: SPRIC Mask */

#define SPI_FR_SPTIC_Pos             17                                           /*!<SPI FR: SPTIC Position */
#define SPI_FR_SPTIC_Msk             (0x1UL << SPI_FR_SPTIC_Pos)                  /*!<SPI FR: SPTIC Mask */

#define SPI_FR_MODFC_Pos             19                                           /*!<SPI FR: MODFC Position */
#define SPI_FR_MODFC_Msk             (0x1UL << SPI_FR_MODFC_Pos)                  /*!<SPI FR: MODFC Mask */

#define SPI_FR_RXOVC_Pos             20                                           /*!<SPI FR: RXOVC Position */
#define SPI_FR_RXOVC_Msk             (0x1UL << SPI_FR_RXOVC_Pos)                  /*!<SPI FR: RXOVC Mask */

#define SPI_FR_WCOLC_Pos             21                                           /*!<SPI FR: WCOLC Position */
#define SPI_FR_WCOLC_Msk             (0x1UL << SPI_FR_WCOLC_Pos)                  /*!<SPI FR: WCOLC Mask */

#define SPI_FR_Msk_ALL               (SPI_FR_SPRI_Msk              |\
                                      SPI_FR_SPTI_Msk              |\
                                      SPI_FR_MODF_Msk              |\
                                      SPI_FR_RXOV_Msk              |\
                                      SPI_FR_WCOL_Msk              |\
                                      SPI_FR_SPRIC_Msk             |\
                                      SPI_FR_SPTIC_Msk             |\
                                      SPI_FR_MODFC_Msk             |\
                                      SPI_FR_RXOVC_Msk             |\
                                      SPI_FR_WCOLC_Msk             )

#define SPI_TDR_TDR_Pos              0                                            /*!<SPI TDR: TDR Position */
#define SPI_TDR_TDR_Msk              (0xFFFFUL /*<< SPI_TDR_TDR_Pos*/)            /*!<SPI TDR: TDR Mask */

#define SPI_TDR_Msk_ALL              (SPI_TDR_TDR_Msk             )

#define SPI_RDR_RDR_Pos              0                                            /*!<SPI RDR: RDR Position */
#define SPI_RDR_RDR_Msk              (0xFFFFUL /*<< SPI_RDR_RDR_Pos*/)            /*!<SPI RDR: RDR Mask */

#define SPI_RDR_Msk_ALL              (SPI_RDR_RDR_Msk             )

#define SPI_CR_SPR_Pos               0                                            /*!<SPI CR: SPR Position */
#define SPI_CR_SPR_Msk               (0xFUL /*<< SPI_CR_SPR_Pos*/)                /*!<SPI CR: SPR Mask */

#define SPI_CR_SSDIS_Pos             4                                            /*!<SPI CR: SSDIS Position */
#define SPI_CR_SSDIS_Msk             (0x1UL << SPI_CR_SSDIS_Pos)                  /*!<SPI CR: SSDIS Mask */

#define SPI_CR_CPOL_Pos              5                                            /*!<SPI CR: CPOL Position */
#define SPI_CR_CPOL_Msk              (0x1UL << SPI_CR_CPOL_Pos)                   /*!<SPI CR: CPOL Mask */

#define SPI_CR_CPHA_Pos              6                                            /*!<SPI CR: CPHA Position */
#define SPI_CR_CPHA_Msk              (0x1UL << SPI_CR_CPHA_Pos)                   /*!<SPI CR: CPHA Mask */

#define SPI_CR_MSTR_Pos              7                                            /*!<SPI CR: MSTR Position */
#define SPI_CR_MSTR_Msk              (0x1UL << SPI_CR_MSTR_Pos)                   /*!<SPI CR: MSTR Mask */

#define SPI_CR_DIR_Pos               8                                            /*!<SPI CR: DIR Position */
#define SPI_CR_DIR_Msk               (0x1UL << SPI_CR_DIR_Pos)                    /*!<SPI CR: DIR Mask */

#define SPI_CR_SPDATL_Pos            9                                            /*!<SPI CR: SPDATL Position */
#define SPI_CR_SPDATL_Msk            (0x1UL << SPI_CR_SPDATL_Pos)                 /*!<SPI CR: SPDATL Mask */

#define SPI_CR_SPRIE_Pos             10                                           /*!<SPI CR: SPRIE Position */
#define SPI_CR_SPRIE_Msk             (0x1UL << SPI_CR_SPRIE_Pos)                  /*!<SPI CR: SPRIE Mask */

#define SPI_CR_SPTIE_Pos             11                                           /*!<SPI CR: SPTIE Position */
#define SPI_CR_SPTIE_Msk             (0x1UL << SPI_CR_SPTIE_Pos)                  /*!<SPI CR: SPTIE Mask */

#define SPI_CR_SPDMAR_Pos            12                                           /*!<SPI CR: SPDMAR Position */
#define SPI_CR_SPDMAR_Msk            (0x1UL << SPI_CR_SPDMAR_Pos)                 /*!<SPI CR: SPDMAR Mask */

#define SPI_CR_SPDMAT_Pos            13                                           /*!<SPI CR: SPDMAT Position */
#define SPI_CR_SPDMAT_Msk            (0x1UL << SPI_CR_SPDMAT_Pos)                 /*!<SPI CR: SPDMAT Mask */

#define SPI_CR_SPIEN_Pos             14                                           /*!<SPI CR: SPIEN Position */
#define SPI_CR_SPIEN_Msk             (0x1UL << SPI_CR_SPIEN_Pos)                  /*!<SPI CR: SPIEN Mask */

#define SPI_CR_SPSFF_Pos             15                                           /*!<SPI CR: SPSFF Position */
#define SPI_CR_SPSFF_Msk             (0x1UL << SPI_CR_SPSFF_Pos)                  /*!<SPI CR: SPSFF Mask */

#define SPI_CR_Msk_ALL               (SPI_CR_SPR_Msk               |\
                                      SPI_CR_SSDIS_Msk             |\
                                      SPI_CR_CPOL_Msk              |\
                                      SPI_CR_CPHA_Msk              |\
                                      SPI_CR_MSTR_Msk              |\
                                      SPI_CR_DIR_Msk               |\
                                      SPI_CR_SPDATL_Msk            |\
                                      SPI_CR_SPRIE_Msk             |\
                                      SPI_CR_SPTIE_Msk             |\
                                      SPI_CR_SPDMAR_Msk            |\
                                      SPI_CR_SPDMAT_Msk            |\
                                      SPI_CR_SPIEN_Msk             |\
                                      SPI_CR_SPSFF_Msk             )

#define TWI_FR_TWINT_Pos             0                                            /*!<TWI FR: TWINT Position */
#define TWI_FR_TWINT_Msk             (0x1UL /*<< TWI_FR_TWINT_Pos*/)              /*!<TWI FR: TWINT Mask */

#define TWI_FR_TFREE_Pos             2                                            /*!<TWI FR: TFREE Position */
#define TWI_FR_TFREE_Msk             (0x1UL << TWI_FR_TFREE_Pos)                  /*!<TWI FR: TFREE Mask */

#define TWI_FR_TOUT_Pos              3                                            /*!<TWI FR: TOUT Position */
#define TWI_FR_TOUT_Msk              (0x1UL << TWI_FR_TOUT_Pos)                   /*!<TWI FR: TOUT Mask */

#define TWI_FR_TWINTC_Pos            16                                           /*!<TWI FR: TWINTC Position */
#define TWI_FR_TWINTC_Msk            (0x1UL << TWI_FR_TWINTC_Pos)                 /*!<TWI FR: TWINTC Mask */

#define TWI_FR_TFREEC_Pos            18                                           /*!<TWI FR: TFREEC Position */
#define TWI_FR_TFREEC_Msk            (0x1UL << TWI_FR_TFREEC_Pos)                 /*!<TWI FR: TFREEC Mask */

#define TWI_FR_TOUTC_Pos             19                                           /*!<TWI FR: TOUTC Position */
#define TWI_FR_TOUTC_Msk             (0x1UL << TWI_FR_TOUTC_Pos)                  /*!<TWI FR: TOUTC Mask */

#define TWI_FR_Msk_ALL               (TWI_FR_TWINT_Msk             |\
                                      TWI_FR_TFREE_Msk             |\
                                      TWI_FR_TOUT_Msk              |\
                                      TWI_FR_TWINTC_Msk            |\
                                      TWI_FR_TFREEC_Msk            |\
                                      TWI_FR_TOUTC_Msk             )

#define TWI_STAT_SR_Pos              3                                            /*!<TWI STAT: SR Position */
#define TWI_STAT_SR_Msk              (0x1FUL << TWI_STAT_SR_Pos)                  /*!<TWI STAT: SR Mask */

#define TWI_STAT_Msk_ALL             (TWI_STAT_SR_Msk             )

#define TWI_HOC_HOC_Pos              0                                            /*!<TWI HOC: HOC Position */
#define TWI_HOC_HOC_Msk              (0xFFUL /*<< TWI_HOC_HOC_Pos*/)              /*!<TWI HOC: HOC Mask */

#define TWI_HOC_Msk_ALL              (TWI_HOC_HOC_Msk             )

#define TWI_BRT_BRT_Pos              0                                            /*!<TWI BRT: BRT Position */
#define TWI_BRT_BRT_Msk              (0xFFUL /*<< TWI_BRT_BRT_Pos*/)              /*!<TWI BRT: BRT Mask */

#define TWI_BRT_Msk_ALL              (TWI_BRT_BRT_Msk             )

#define TWI_DR_DR_Pos                0                                            /*!<TWI DR: DR Position */
#define TWI_DR_DR_Msk                (0xFFUL /*<< TWI_DR_DR_Pos*/)                /*!<TWI DR: DR Mask */

#define TWI_DR_Msk_ALL               (TWI_DR_DR_Msk               )

#define TWI_ADDR_GC_Pos              0                                            /*!<TWI ADDR: GC Position */
#define TWI_ADDR_GC_Msk              (0x1UL /*<< TWI_ADDR_GC_Pos*/)               /*!<TWI ADDR: GC Mask */

#define TWI_ADDR_ADDR_Pos            1                                            /*!<TWI ADDR: ADDR Position */
#define TWI_ADDR_ADDR_Msk            (0x7FUL << TWI_ADDR_ADDR_Pos)                /*!<TWI ADDR: ADDR Mask */

#define TWI_ADDR_TWIAMR_Pos          17                                           /*!<TWI ADDR: TWIAMR Position */
#define TWI_ADDR_TWIAMR_Msk          (0x7FUL << TWI_ADDR_TWIAMR_Pos)              /*!<TWI ADDR: TWIAMR Mask */

#define TWI_ADDR_Msk_ALL             (TWI_ADDR_GC_Msk              |\
                                      TWI_ADDR_ADDR_Msk            |\
                                      TWI_ADDR_TWIAMR_Msk          )

#define TWI_CR_AA_Pos                0                                            /*!<TWI CR: AA Position */
#define TWI_CR_AA_Msk                (0x1UL /*<< TWI_CR_AA_Pos*/)                 /*!<TWI CR: AA Mask */

#define TWI_CR_STO_Pos               1                                            /*!<TWI CR: STO Position */
#define TWI_CR_STO_Msk               (0x1UL << TWI_CR_STO_Pos)                    /*!<TWI CR: STO Mask */

#define TWI_CR_STA_Pos               2                                            /*!<TWI CR: STA Position */
#define TWI_CR_STA_Msk               (0x1UL << TWI_CR_STA_Pos)                    /*!<TWI CR: STA Mask */

#define TWI_CR_CR_Pos                4                                            /*!<TWI CR: CR Position */
#define TWI_CR_CR_Msk                (0x3UL << TWI_CR_CR_Pos)                     /*!<TWI CR: CR Mask */

#define TWI_CR_CNT_Pos               6                                            /*!<TWI CR: CNT Position */
#define TWI_CR_CNT_Msk               (0x3UL << TWI_CR_CNT_Pos)                    /*!<TWI CR: CNT Mask */

#define TWI_CR_EFREE_Pos             8                                            /*!<TWI CR: EFREE Position */
#define TWI_CR_EFREE_Msk             (0x1UL << TWI_CR_EFREE_Pos)                  /*!<TWI CR: EFREE Mask */

#define TWI_CR_ETOT_Pos              9                                            /*!<TWI CR: ETOT Position */
#define TWI_CR_ETOT_Msk              (0x1UL << TWI_CR_ETOT_Pos)                   /*!<TWI CR: ETOT Mask */

#define TWI_CR_TWINTE_Pos            10                                           /*!<TWI CR: TWINTE Position */
#define TWI_CR_TWINTE_Msk            (0x1UL << TWI_CR_TWINTE_Pos)                 /*!<TWI CR: TWINTE Mask */

#define TWI_CR_TWIEN_Pos             15                                           /*!<TWI CR: TWIEN Position */
#define TWI_CR_TWIEN_Msk             (0x1UL << TWI_CR_TWIEN_Pos)                  /*!<TWI CR: TWIEN Mask */

#define TWI_CR_Msk_ALL               (TWI_CR_AA_Msk                |\
                                      TWI_CR_STO_Msk               |\
                                      TWI_CR_STA_Msk               |\
                                      TWI_CR_CR_Msk                |\
                                      TWI_CR_CNT_Msk               |\
                                      TWI_CR_EFREE_Msk             |\
                                      TWI_CR_ETOT_Msk              |\
                                      TWI_CR_TWINTE_Msk            |\
                                      TWI_CR_TWIEN_Msk             )


/* @addtogroup Peripheral Bitband Address */

#define FLASH_ACR_PRFTEN_BIT                      (*(__IO uint32_t*)(0x428A0020))
#define FLASH_ACR_ICEN_BIT                        (*(__IO uint32_t*)(0x428A0024))
#define FLASH_ACR_DCEN_BIT                        (*(__IO uint32_t*)(0x428A0028))
#define FLASH_ACR_CRST_BIT                        (*(__IO uint32_t*)(0x428A002C))
#define FLASH_SR_EOP_BIT                          (*(__I  uint32_t*)(0x428A0180))
#define FLASH_SR_OPERR_BIT                        (*(__I  uint32_t*)(0x428A0184))
#define FLASH_SR_FLSERR_BIT                       (*(__I  uint32_t*)(0x428A018C))
#define FLASH_SR_WRPRTERR_BIT                     (*(__I  uint32_t*)(0x428A0190))
#define FLASH_SR_PGPERR_BIT                       (*(__I  uint32_t*)(0x428A0194))
#define FLASH_SR_PGWERR_BIT                       (*(__I  uint32_t*)(0x428A0198))
#define FLASH_SR_STAERR_BIT                       (*(__I  uint32_t*)(0x428A019C))
#define FLASH_SR_BSY_BIT                          (*(__I  uint32_t*)(0x428A01BC))
#define FLASH_SR_EOPC_BIT                         (*(__O  uint32_t*)(0x428A01C0))
#define FLASH_SR_OPERRC_BIT                       (*(__O  uint32_t*)(0x428A01C4))
#define FLASH_SR_FLSERRC_BIT                      (*(__O  uint32_t*)(0x428A01CC))
#define FLASH_SR_WRPRTERRC_BIT                    (*(__O  uint32_t*)(0x428A01D0))
#define FLASH_SR_PGPERRC_BIT                      (*(__O  uint32_t*)(0x428A01D4))
#define FLASH_SR_PGWERRC_BIT                      (*(__O  uint32_t*)(0x428A01D8))
#define FLASH_SR_STAERRC_BIT                      (*(__O  uint32_t*)(0x428A01DC))
#define FLASH_CR_STRT_BIT                         (*(__IO uint32_t*)(0x428A0220))
#define FLASH_CR_PSIZE_BIT                        (*(__IO uint32_t*)(0x428A022C))
#define FLASH_CR_INFLCK_BIT                       (*(__IO uint32_t*)(0x428A0230))
#define FLASH_CR_E2LCK_BIT                        (*(__IO uint32_t*)(0x428A0238))
#define FLASH_CR_MNLCK_BIT                        (*(__IO uint32_t*)(0x428A023C))
#define FLASH_CNTCR_CNTEN_BIT                     (*(__IO uint32_t*)(0x428A0700))
#define SYSCFG_PWRCR_BODIE_BIT                    (*(__IO uint32_t*)(0x42880018))
#define SYSCFG_PWRCR_BODEN_BIT                    (*(__IO uint32_t*)(0x4288001C))
#define SYSCFG_PWRSR_BODIF_BIT                    (*(__IO uint32_t*)(0x42880080))
#define SYSCFG_PWRSR_BODF_BIT                     (*(__I  uint32_t*)(0x42880084))
#define SYSCFG_SAFR_IEN_EXTI0_BIT                 (*(__IO uint32_t*)(0x42880114))
#define SYSCFG_SAFR_IEN_BOD_BIT                   (*(__IO uint32_t*)(0x42880118))
#define SYSCFG_SAFR_IEN_CSM_BIT                   (*(__IO uint32_t*)(0x4288011C))
#define SYSCFG_DBGCR_DBG_SLEEP_BIT                (*(__IO uint32_t*)(0x42880200))
#define SYSCFG_DBGCR_DBG_STOP_BIT                 (*(__IO uint32_t*)(0x42880204))
#define SYSCFG_DBGCR_DBG_DMA_BIT                  (*(__IO uint32_t*)(0x4288021C))
#define SYSCFG_DBGCR_DBG_IWDT_BIT                 (*(__IO uint32_t*)(0x42880220))
#define SYSCFG_DBGCR_DBG_WWDT_BIT                 (*(__IO uint32_t*)(0x42880224))
#define SYSCFG_DBGCR_DBG_GPT_BIT                  (*(__IO uint32_t*)(0x42880228))
#define SYSCFG_DBGCR_DBG_TIM_BIT                  (*(__IO uint32_t*)(0x4288022C))
#define SYSCFG_DBGCR_DBG_MCM_BIT                  (*(__IO uint32_t*)(0x42880230))
#define SYSCFG_DBGCR_DBG_UART_BIT                 (*(__IO uint32_t*)(0x42880234))
#define SYSCFG_DBGCR_DBG_SPI_BIT                  (*(__IO uint32_t*)(0x42880238))
#define SYSCFG_DBGCR_DBG_TWI_BIT                  (*(__IO uint32_t*)(0x4288023C))
#define RCC_CR_HSEON_BIT                          (*(__IO uint32_t*)(0x42888010))
#define RCC_CR_HSERDY_BIT                         (*(__I  uint32_t*)(0x42888014))
#define RCC_CR_PLLON_BIT                          (*(__IO uint32_t*)(0x42888018))
#define RCC_CR_PLLRDY_BIT                         (*(__I  uint32_t*)(0x4288801C))
#define RCC_CFGR_PLLSRC_BIT                       (*(__IO uint32_t*)(0x428880C8))
#define RCC_CFGR_PLLXTPRE_BIT                     (*(__IO uint32_t*)(0x428880CC))
#define RCC_CIENR_HSERDYIE_BIT                    (*(__IO uint32_t*)(0x4288810C))
#define RCC_CIENR_PLLRDYIE_BIT                    (*(__IO uint32_t*)(0x42888110))
#define RCC_CISTR_HSERDYIF_BIT                    (*(__I  uint32_t*)(0x4288818C))
#define RCC_CISTR_PLLRDYIF_BIT                    (*(__I  uint32_t*)(0x42888190))
#define RCC_CISTR_CSMHSEF_BIT                     (*(__I  uint32_t*)(0x42888198))
#define RCC_CISTR_CSMPLLF_BIT                     (*(__I  uint32_t*)(0x4288819C))
#define RCC_CICLR_HSERDYC_BIT                     (*(__O  uint32_t*)(0x4288820C))
#define RCC_CICLR_PLLRDYC_BIT                     (*(__O  uint32_t*)(0x42888210))
#define RCC_CICLR_CSMC_BIT                        (*(__O  uint32_t*)(0x4288821C))
#define RCC_AHBRSTR_IOCFGRST_BIT                  (*(__IO uint32_t*)(0x42888280))
#define RCC_AHBRSTR_IODATRST_BIT                  (*(__IO uint32_t*)(0x42888284))
#define RCC_AHBRSTR_ADC1RST_BIT                   (*(__IO uint32_t*)(0x4288828C))
#define RCC_AHBRSTR_ADC2RST_BIT                   (*(__IO uint32_t*)(0x42888290))
#define RCC_AHBRSTR_ADC3RST_BIT                   (*(__IO uint32_t*)(0x42888294))
#define RCC_AHBRSTR_SYSCFGRST_BIT                 (*(__IO uint32_t*)(0x42888298))
#define RCC_AHBRSTR_DMARST_BIT                    (*(__IO uint32_t*)(0x4288829C))
#define RCC_AHBRSTR_MACPRST_BIT                   (*(__IO uint32_t*)(0x428882A0))
#define RCC_AHBRSTR_CRCRST_BIT                    (*(__IO uint32_t*)(0x428882A4))
#define RCC_AHBRSTR_GPT0RST_BIT                   (*(__IO uint32_t*)(0x428882A8))
#define RCC_AHBRSTR_GPT1RST_BIT                   (*(__IO uint32_t*)(0x428882AC))
#define RCC_AHBRSTR_GPT2RST_BIT                   (*(__IO uint32_t*)(0x428882B0))
#define RCC_AHBRSTR_GPT3RST_BIT                   (*(__IO uint32_t*)(0x428882B4))
#define RCC_AHBRSTR_GPTRST_BIT                    (*(__IO uint32_t*)(0x428882B8))
#define RCC_APB2RSTR_MCM1RST_BIT                  (*(__IO uint32_t*)(0x42888300))
#define RCC_APB2RSTR_MCM2RST_BIT                  (*(__IO uint32_t*)(0x42888304))
#define RCC_APB1RSTR_TIM5RST_BIT                  (*(__IO uint32_t*)(0x42888380))
#define RCC_APB1RSTR_TIM6RST_BIT                  (*(__IO uint32_t*)(0x42888384))
#define RCC_APB1RSTR_TIM7RST_BIT                  (*(__IO uint32_t*)(0x42888388))
#define RCC_APB1RSTR_TIM8RST_BIT                  (*(__IO uint32_t*)(0x4288838C))
#define RCC_APB1RSTR_QEIRST_BIT                   (*(__IO uint32_t*)(0x42888390))
#define RCC_APB1RSTR_UART1RST_BIT                 (*(__IO uint32_t*)(0x42888394))
#define RCC_APB1RSTR_UART2RST_BIT                 (*(__IO uint32_t*)(0x42888398))
#define RCC_APB1RSTR_UART3RST_BIT                 (*(__IO uint32_t*)(0x4288839C))
#define RCC_APB1RSTR_SPI1RST_BIT                  (*(__IO uint32_t*)(0x428883A0))
#define RCC_APB1RSTR_SPI2RST_BIT                  (*(__IO uint32_t*)(0x428883A4))
#define RCC_APB1RSTR_TWI1RST_BIT                  (*(__IO uint32_t*)(0x428883AC))
#define RCC_APB1RSTR_WWDTRST_BIT                  (*(__IO uint32_t*)(0x428883B0))
#define RCC_APB1RSTR_AMOCRST_BIT                  (*(__IO uint32_t*)(0x428883B4))
#define RCC_AHBENR_IOCFGEN_BIT                    (*(__IO uint32_t*)(0x42888400))
#define RCC_AHBENR_IODATEN_BIT                    (*(__IO uint32_t*)(0x42888404))
#define RCC_AHBENR_ADC1EN_BIT                     (*(__IO uint32_t*)(0x4288840C))
#define RCC_AHBENR_ADC2EN_BIT                     (*(__IO uint32_t*)(0x42888410))
#define RCC_AHBENR_ADC3EN_BIT                     (*(__IO uint32_t*)(0x42888414))
#define RCC_AHBENR_SYSCFGEN_BIT                   (*(__IO uint32_t*)(0x42888418))
#define RCC_AHBENR_DMAEN_BIT                      (*(__IO uint32_t*)(0x4288841C))
#define RCC_AHBENR_MACPEN_BIT                     (*(__IO uint32_t*)(0x42888420))
#define RCC_AHBENR_CRCEN_BIT                      (*(__IO uint32_t*)(0x42888424))
#define RCC_AHBENR_GPT0EN_BIT                     (*(__IO uint32_t*)(0x42888428))
#define RCC_AHBENR_GPT1EN_BIT                     (*(__IO uint32_t*)(0x4288842C))
#define RCC_AHBENR_GPT2EN_BIT                     (*(__IO uint32_t*)(0x42888430))
#define RCC_AHBENR_GPT3EN_BIT                     (*(__IO uint32_t*)(0x42888434))
#define RCC_AHBENR_GPTEN_BIT                      (*(__IO uint32_t*)(0x42888438))
#define RCC_APB2ENR_MCM1EN_BIT                    (*(__IO uint32_t*)(0x42888480))
#define RCC_APB2ENR_MCM2EN_BIT                    (*(__IO uint32_t*)(0x42888484))
#define RCC_APB1ENR_TIM5EN_BIT                    (*(__IO uint32_t*)(0x42888500))
#define RCC_APB1ENR_TIM6EN_BIT                    (*(__IO uint32_t*)(0x42888504))
#define RCC_APB1ENR_TIM7EN_BIT                    (*(__IO uint32_t*)(0x42888508))
#define RCC_APB1ENR_TIM8EN_BIT                    (*(__IO uint32_t*)(0x4288850C))
#define RCC_APB1ENR_QEIEN_BIT                     (*(__IO uint32_t*)(0x42888510))
#define RCC_APB1ENR_UART1EN_BIT                   (*(__IO uint32_t*)(0x42888514))
#define RCC_APB1ENR_UART2EN_BIT                   (*(__IO uint32_t*)(0x42888518))
#define RCC_APB1ENR_UART3EN_BIT                   (*(__IO uint32_t*)(0x4288851C))
#define RCC_APB1ENR_SPI1EN_BIT                    (*(__IO uint32_t*)(0x42888520))
#define RCC_APB1ENR_SPI2EN_BIT                    (*(__IO uint32_t*)(0x42888524))
#define RCC_APB1ENR_TWI1EN_BIT                    (*(__IO uint32_t*)(0x4288852C))
#define RCC_APB1ENR_WWDTEN_BIT                    (*(__IO uint32_t*)(0x42888530))
#define RCC_APB1ENR_AMOCEN_BIT                    (*(__IO uint32_t*)(0x42888534))
#define RCC_RSTSTR_PINRSTF_BIT                    (*(__I  uint32_t*)(0x42888580))
#define RCC_RSTSTR_LVRSTF_BIT                     (*(__I  uint32_t*)(0x42888584))
#define RCC_RSTSTR_PORSTF_BIT                     (*(__I  uint32_t*)(0x42888588))
#define RCC_RSTSTR_SWRSTF_BIT                     (*(__I  uint32_t*)(0x4288858C))
#define RCC_RSTSTR_IWDTRSTF_BIT                   (*(__I  uint32_t*)(0x42888590))
#define RCC_RSTSTR_WWDTRSTF_BIT                   (*(__I  uint32_t*)(0x42888594))
#define RCC_RSTSTR_LVRSTF2_BIT                    (*(__I  uint32_t*)(0x42888598))
#define RCC_RSTCLR_PINRSTFC_BIT                   (*(__O  uint32_t*)(0x42888600))
#define RCC_RSTCLR_LVRSTFC_BIT                    (*(__O  uint32_t*)(0x42888604))
#define RCC_RSTCLR_PORSTFC_BIT                    (*(__O  uint32_t*)(0x42888608))
#define RCC_RSTCLR_SWRSTFC_BIT                    (*(__O  uint32_t*)(0x4288860C))
#define RCC_RSTCLR_IWDTRSTFC_BIT                  (*(__O  uint32_t*)(0x42888610))
#define RCC_RSTCLR_WWDTRSTFC_BIT                  (*(__O  uint32_t*)(0x42888614))
#define RCC_RSTCLR_LVRSTF2C_BIT                   (*(__O  uint32_t*)(0x42888618))
#define RCC_HSICAL_TRIMRUN_BIT                    (*(__IO uint32_t*)(0x428886FC))
#define IWDT_CR_IWDTON_BIT                        (*(__IO uint32_t*)(0x4209803C))
#define WWDT_CR_WWDTIE_BIT                        (*(__IO uint32_t*)(0x420A0038))
#define WWDT_CR_WWDTON_BIT                        (*(__IO uint32_t*)(0x420A003C))
#define WWDT_SR_WWDTIF_BIT                        (*(__IO uint32_t*)(0x420A00BC))
#define CRC_CR_RELOAD_BIT                         (*(__O  uint32_t*)(0x428B0080))
#define CRC_CR_COMPLW_BIT                         (*(__IO uint32_t*)(0x428B00A8))
#define CRC_CR_RBITW_BIT                          (*(__IO uint32_t*)(0x428B00AC))
#define CRC_CR_RBYTEW_BIT                         (*(__IO uint32_t*)(0x428B00B0))
#define CRC_CR_COMPLR_BIT                         (*(__IO uint32_t*)(0x428B00B4))
#define CRC_CR_RBITR_BIT                          (*(__IO uint32_t*)(0x428B00B8))
#define CRC_CR_RBYTER_BIT                         (*(__IO uint32_t*)(0x428B00BC))
#define RAMBIST_CFG_SEL_BIT                       (*(__IO uint32_t*)(0x428B80BC))
#define RAMBIST_CSR_MOD_BIT                       (*(__IO uint32_t*)(0x428B8140))
#define RAMBIST_CSR_BSY_BIT                       (*(__I  uint32_t*)(0x428B8144))
#define RAMBIST_CSR_ERR_BIT                       (*(__IO uint32_t*)(0x428B8148))
#define GPIOA_CFG_TTLEN_PA2_BIT                   (*(__IO uint32_t*)(0x42800288))
#define GPIOA_CFG_TTLEN_PA3_BIT                   (*(__IO uint32_t*)(0x4280028C))
#define GPIOA_CFG_TTLEN_PA8_BIT                   (*(__IO uint32_t*)(0x428002A0))
#define GPIOA_CFG_TTLEN_PA11_BIT                  (*(__IO uint32_t*)(0x428002AC))
#define GPIOB_CFG_TTLEN_PA2_BIT                   (*(__IO uint32_t*)(0x42800688))
#define GPIOB_CFG_TTLEN_PA3_BIT                   (*(__IO uint32_t*)(0x4280068C))
#define GPIOB_CFG_TTLEN_PA8_BIT                   (*(__IO uint32_t*)(0x428006A0))
#define GPIOB_CFG_TTLEN_PA11_BIT                  (*(__IO uint32_t*)(0x428006AC))
#define GPIOC_CFG_TTLEN_PA2_BIT                   (*(__IO uint32_t*)(0x42800A88))
#define GPIOC_CFG_TTLEN_PA3_BIT                   (*(__IO uint32_t*)(0x42800A8C))
#define GPIOC_CFG_TTLEN_PA8_BIT                   (*(__IO uint32_t*)(0x42800AA0))
#define GPIOC_CFG_TTLEN_PA11_BIT                  (*(__IO uint32_t*)(0x42800AAC))
#define GPIOD_CFG_TTLEN_PA2_BIT                   (*(__IO uint32_t*)(0x42800E88))
#define GPIOD_CFG_TTLEN_PA3_BIT                   (*(__IO uint32_t*)(0x42800E8C))
#define GPIOD_CFG_TTLEN_PA8_BIT                   (*(__IO uint32_t*)(0x42800EA0))
#define GPIOD_CFG_TTLEN_PA11_BIT                  (*(__IO uint32_t*)(0x42800EAC))
#define GPIOE_CFG_TTLEN_PA2_BIT                   (*(__IO uint32_t*)(0x42801288))
#define GPIOE_CFG_TTLEN_PA3_BIT                   (*(__IO uint32_t*)(0x4280128C))
#define GPIOE_CFG_TTLEN_PA8_BIT                   (*(__IO uint32_t*)(0x428012A0))
#define GPIOE_CFG_TTLEN_PA11_BIT                  (*(__IO uint32_t*)(0x428012AC))
#define DMA_CCR0_EN_BIT                           (*(__IO uint32_t*)(0x42890200))
#define DMA_CCR0_TCIE_BIT                         (*(__IO uint32_t*)(0x42890204))
#define DMA_CCR0_HTIE_BIT                         (*(__IO uint32_t*)(0x42890208))
#define DMA_CCR0_BEIE_BIT                         (*(__IO uint32_t*)(0x4289020C))
#define DMA_CCR0_TEIE_BIT                         (*(__IO uint32_t*)(0x42890210))
#define DMA_CCR0_TRGMODE_BIT                      (*(__IO uint32_t*)(0x4289025C))
#define DMA_CCR1_EN_BIT                           (*(__IO uint32_t*)(0x42890600))
#define DMA_CCR1_TCIE_BIT                         (*(__IO uint32_t*)(0x42890604))
#define DMA_CCR1_HTIE_BIT                         (*(__IO uint32_t*)(0x42890608))
#define DMA_CCR1_BEIE_BIT                         (*(__IO uint32_t*)(0x4289060C))
#define DMA_CCR1_TEIE_BIT                         (*(__IO uint32_t*)(0x42890610))
#define DMA_CCR1_TRGMODE_BIT                      (*(__IO uint32_t*)(0x4289065C))
#define DMA_CCR2_EN_BIT                           (*(__IO uint32_t*)(0x42890A00))
#define DMA_CCR2_TCIE_BIT                         (*(__IO uint32_t*)(0x42890A04))
#define DMA_CCR2_HTIE_BIT                         (*(__IO uint32_t*)(0x42890A08))
#define DMA_CCR2_BEIE_BIT                         (*(__IO uint32_t*)(0x42890A0C))
#define DMA_CCR2_TEIE_BIT                         (*(__IO uint32_t*)(0x42890A10))
#define DMA_CCR2_TRGMODE_BIT                      (*(__IO uint32_t*)(0x42890A5C))
#define DMA_CCR3_EN_BIT                           (*(__IO uint32_t*)(0x42890E00))
#define DMA_CCR3_TCIE_BIT                         (*(__IO uint32_t*)(0x42890E04))
#define DMA_CCR3_HTIE_BIT                         (*(__IO uint32_t*)(0x42890E08))
#define DMA_CCR3_BEIE_BIT                         (*(__IO uint32_t*)(0x42890E0C))
#define DMA_CCR3_TEIE_BIT                         (*(__IO uint32_t*)(0x42890E10))
#define DMA_CCR3_TRGMODE_BIT                      (*(__IO uint32_t*)(0x42890E5C))
#define DMA_CCR4_EN_BIT                           (*(__IO uint32_t*)(0x42891200))
#define DMA_CCR4_TCIE_BIT                         (*(__IO uint32_t*)(0x42891204))
#define DMA_CCR4_HTIE_BIT                         (*(__IO uint32_t*)(0x42891208))
#define DMA_CCR4_BEIE_BIT                         (*(__IO uint32_t*)(0x4289120C))
#define DMA_CCR4_TEIE_BIT                         (*(__IO uint32_t*)(0x42891210))
#define DMA_CCR4_TRGMODE_BIT                      (*(__IO uint32_t*)(0x4289125C))
#define DMA_CCR5_EN_BIT                           (*(__IO uint32_t*)(0x42891600))
#define DMA_CCR5_TCIE_BIT                         (*(__IO uint32_t*)(0x42891604))
#define DMA_CCR5_HTIE_BIT                         (*(__IO uint32_t*)(0x42891608))
#define DMA_CCR5_BEIE_BIT                         (*(__IO uint32_t*)(0x4289160C))
#define DMA_CCR5_TEIE_BIT                         (*(__IO uint32_t*)(0x42891610))
#define DMA_CCR5_TRGMODE_BIT                      (*(__IO uint32_t*)(0x4289165C))
#define DMA_CCR6_EN_BIT                           (*(__IO uint32_t*)(0x42891A00))
#define DMA_CCR6_TCIE_BIT                         (*(__IO uint32_t*)(0x42891A04))
#define DMA_CCR6_HTIE_BIT                         (*(__IO uint32_t*)(0x42891A08))
#define DMA_CCR6_BEIE_BIT                         (*(__IO uint32_t*)(0x42891A0C))
#define DMA_CCR6_TEIE_BIT                         (*(__IO uint32_t*)(0x42891A10))
#define DMA_CCR6_TRGMODE_BIT                      (*(__IO uint32_t*)(0x42891A5C))
#define DMA_CCR7_EN_BIT                           (*(__IO uint32_t*)(0x42891E00))
#define DMA_CCR7_TCIE_BIT                         (*(__IO uint32_t*)(0x42891E04))
#define DMA_CCR7_HTIE_BIT                         (*(__IO uint32_t*)(0x42891E08))
#define DMA_CCR7_BEIE_BIT                         (*(__IO uint32_t*)(0x42891E0C))
#define DMA_CCR7_TEIE_BIT                         (*(__IO uint32_t*)(0x42891E10))
#define DMA_CCR7_TRGMODE_BIT                      (*(__IO uint32_t*)(0x42891E5C))
#define MCM1_PWMOE_PWMOE_BIT                      (*(__IO uint32_t*)(0x42400000))
#define MCM1_PWMCON1_PWM0S_BIT                    (*(__IO uint32_t*)(0x42400080))
#define MCM1_PWMCON1_PWM1S_BIT                    (*(__IO uint32_t*)(0x42400084))
#define MCM1_PWMCON1_PWM2S_BIT                    (*(__IO uint32_t*)(0x42400088))
#define MCM1_PWMCON1_PWM01S_BIT                   (*(__IO uint32_t*)(0x4240008C))
#define MCM1_PWMCON1_PWM11S_BIT                   (*(__IO uint32_t*)(0x42400090))
#define MCM1_PWMCON1_PWM21S_BIT                   (*(__IO uint32_t*)(0x42400094))
#define MCM1_PWMCON1_PDCON0_BIT                   (*(__IO uint32_t*)(0x42400098))
#define MCM1_PWMCON1_PDCON1_BIT                   (*(__IO uint32_t*)(0x4240009C))
#define MCM1_PWMCON1_PDCON2_BIT                   (*(__IO uint32_t*)(0x424000A0))
#define MCM1_PWMCON1_PWMSYM_BIT                   (*(__IO uint32_t*)(0x424000A4))
#define MCM1_PWMCON1_POUTMOD_BIT                  (*(__IO uint32_t*)(0x424000BC))
#define MCM1_PWMCON2_OSYNC_BIT                    (*(__IO uint32_t*)(0x42400120))
#define MCM1_PWMCON2_DILDEN_BIT                   (*(__IO uint32_t*)(0x42400124))
#define MCM1_PWMCON2_CILDEN_BIT                   (*(__IO uint32_t*)(0x42400128))
#define MCM1_PWMCON2_ZDLDEN_BIT                   (*(__IO uint32_t*)(0x4240012C))
#define MCM1_PWMCON2_PDLDEN_BIT                   (*(__IO uint32_t*)(0x42400130))
#define MCM1_PWMCON2_ZCMLDEN_BIT                  (*(__IO uint32_t*)(0x42400134))
#define MCM1_PWMCON2_PCMLDEN_BIT                  (*(__IO uint32_t*)(0x42400138))
#define MCM1_PWMCON2_MCMSYNEN_BIT                 (*(__IO uint32_t*)(0x4240013C))
#define MCM1_PMANUALCON1_PMANUAL0_BIT             (*(__IO uint32_t*)(0x42400B00))
#define MCM1_PMANUALCON1_PMANUAL1_BIT             (*(__IO uint32_t*)(0x42400B04))
#define MCM1_PMANUALCON1_PMANUAL2_BIT             (*(__IO uint32_t*)(0x42400B08))
#define MCM1_PMANUALCON1_PMANUAL01_BIT            (*(__IO uint32_t*)(0x42400B0C))
#define MCM1_PMANUALCON1_PMANUAL11_BIT            (*(__IO uint32_t*)(0x42400B10))
#define MCM1_PMANUALCON1_PMANUAL21_BIT            (*(__IO uint32_t*)(0x42400B14))
#define MCM1_PMANUALCON2_POUT0_BIT                (*(__IO uint32_t*)(0x42400B80))
#define MCM1_PMANUALCON2_POUT1_BIT                (*(__IO uint32_t*)(0x42400B84))
#define MCM1_PMANUALCON2_POUT2_BIT                (*(__IO uint32_t*)(0x42400B88))
#define MCM1_PMANUALCON2_POUT01_BIT               (*(__IO uint32_t*)(0x42400B8C))
#define MCM1_PMANUALCON2_POUT11_BIT               (*(__IO uint32_t*)(0x42400B90))
#define MCM1_PMANUALCON2_POUT21_BIT               (*(__IO uint32_t*)(0x42400B94))
#define MCM1_FLTCON_FLTSTAT_BIT                   (*(__IO uint32_t*)(0x42400C00))
#define MCM1_FLTCON_FLTM_BIT                      (*(__IO uint32_t*)(0x42400C04))
#define MCM1_FLTCON_FLT2S_BIT                     (*(__IO uint32_t*)(0x42400C08))
#define MCM1_FLTCON_FLT2EN_BIT                    (*(__IO uint32_t*)(0x42400C0C))
#define MCM1_FLTCON_FLT1EN_BIT                    (*(__IO uint32_t*)(0x42400C28))
#define MCM1_POSCR_OLSG0_BIT                      (*(__IO uint32_t*)(0x42400C80))
#define MCM1_POSCR_OLSG1_BIT                      (*(__IO uint32_t*)(0x42400C84))
#define MCM1_POSCR_OLSG2_BIT                      (*(__IO uint32_t*)(0x42400C88))
#define MCM1_POSCR_OLSG01_BIT                     (*(__IO uint32_t*)(0x42400C8C))
#define MCM1_POSCR_OLSG11_BIT                     (*(__IO uint32_t*)(0x42400C90))
#define MCM1_POSCR_OLSG21_BIT                     (*(__IO uint32_t*)(0x42400C94))
#define MCM1_POSCR_OLSEN_BIT                      (*(__IO uint32_t*)(0x42400C9C))
#define MCM1_POSTDCR_OSTDEN_BIT                   (*(__IO uint32_t*)(0x42400D1C))
#define MCM1_PWMDMAEN_PTUD0DE_BIT                 (*(__IO uint32_t*)(0x42400D80))
#define MCM1_PWMDMAEN_PTDD0DE_BIT                 (*(__IO uint32_t*)(0x42400D84))
#define MCM1_PWMDMAEN_PTUD1DE_BIT                 (*(__IO uint32_t*)(0x42400D88))
#define MCM1_PWMDMAEN_PTDD1DE_BIT                 (*(__IO uint32_t*)(0x42400D8C))
#define MCM1_PWMDMAEN_PTUD2DE_BIT                 (*(__IO uint32_t*)(0x42400D90))
#define MCM1_PWMDMAEN_PTDD2DE_BIT                 (*(__IO uint32_t*)(0x42400D94))
#define MCM1_PWMDMAEN_PWMZDE_BIT                  (*(__IO uint32_t*)(0x42400D98))
#define MCM1_PWMDMAEN_PWMPDE_BIT                  (*(__IO uint32_t*)(0x42400D9C))
#define MCM1_PWMINTEN_PTUD0IE_BIT                 (*(__IO uint32_t*)(0x42400E00))
#define MCM1_PWMINTEN_PTDD0IE_BIT                 (*(__IO uint32_t*)(0x42400E04))
#define MCM1_PWMINTEN_PTUD1IE_BIT                 (*(__IO uint32_t*)(0x42400E08))
#define MCM1_PWMINTEN_PTDD1IE_BIT                 (*(__IO uint32_t*)(0x42400E0C))
#define MCM1_PWMINTEN_PTUD2IE_BIT                 (*(__IO uint32_t*)(0x42400E10))
#define MCM1_PWMINTEN_PTDD2IE_BIT                 (*(__IO uint32_t*)(0x42400E14))
#define MCM1_PWMINTEN_PWMZIE_BIT                  (*(__IO uint32_t*)(0x42400E18))
#define MCM1_PWMINTEN_PWMPIE_BIT                  (*(__IO uint32_t*)(0x42400E1C))
#define MCM1_PWMINTEN_FLTIE_BIT                   (*(__IO uint32_t*)(0x42400E20))
#define MCM1_PWMINTEN_OIE_BIT                     (*(__IO uint32_t*)(0x42400E24))
#define MCM1_PWMINTEN_OSTDIE_BIT                  (*(__IO uint32_t*)(0x42400E28))
#define MCM1_PWMINTF_PTUD0IF_BIT                  (*(__I  uint32_t*)(0x42400E80))
#define MCM1_PWMINTF_PTDD0IF_BIT                  (*(__I  uint32_t*)(0x42400E84))
#define MCM1_PWMINTF_PTUD1IF_BIT                  (*(__I  uint32_t*)(0x42400E88))
#define MCM1_PWMINTF_PTDD1IF_BIT                  (*(__I  uint32_t*)(0x42400E8C))
#define MCM1_PWMINTF_PTUD2IF_BIT                  (*(__I  uint32_t*)(0x42400E90))
#define MCM1_PWMINTF_PTDD2IF_BIT                  (*(__I  uint32_t*)(0x42400E94))
#define MCM1_PWMINTF_PWMZIF_BIT                   (*(__I  uint32_t*)(0x42400E98))
#define MCM1_PWMINTF_PWMPIF_BIT                   (*(__I  uint32_t*)(0x42400E9C))
#define MCM1_PWMINTF_FLTIF_BIT                    (*(__I  uint32_t*)(0x42400EA0))
#define MCM1_PWMINTF_OSF_BIT                      (*(__I  uint32_t*)(0x42400EA4))
#define MCM1_PWMINTF_OSTDF_BIT                    (*(__I  uint32_t*)(0x42400EA8))
#define MCM1_PWMINTF_SC1STAT_BIT                  (*(__I  uint32_t*)(0x42400EAC))
#define MCM1_PWMINTF_SC2STAT_BIT                  (*(__I  uint32_t*)(0x42400EB0))
#define MCM1_PWMINTF_SC3STAT_BIT                  (*(__I  uint32_t*)(0x42400EB4))
#define MCM1_PWMINTF_PTUD0IFC_BIT                 (*(__O  uint32_t*)(0x42400EC0))
#define MCM1_PWMINTF_PTDD0IFC_BIT                 (*(__O  uint32_t*)(0x42400EC4))
#define MCM1_PWMINTF_PTUD1IFC_BIT                 (*(__O  uint32_t*)(0x42400EC8))
#define MCM1_PWMINTF_PTDD1IFC_BIT                 (*(__O  uint32_t*)(0x42400ECC))
#define MCM1_PWMINTF_PTUD2IFC_BIT                 (*(__O  uint32_t*)(0x42400ED0))
#define MCM1_PWMINTF_PTDD2IFC_BIT                 (*(__O  uint32_t*)(0x42400ED4))
#define MCM1_PWMINTF_PWMZIFC_BIT                  (*(__O  uint32_t*)(0x42400ED8))
#define MCM1_PWMINTF_PWMPIFC_BIT                  (*(__O  uint32_t*)(0x42400EDC))
#define MCM1_PWMINTF_FLTIFC_BIT                   (*(__O  uint32_t*)(0x42400EE0))
#define MCM1_PWMINTF_OSFC_BIT                     (*(__O  uint32_t*)(0x42400EE4))
#define MCM1_PWMINTF_OSTDFC_BIT                   (*(__O  uint32_t*)(0x42400EE8))
#define MCM1_PWMINTF_SC1STATC_BIT                 (*(__O  uint32_t*)(0x42400EEC))
#define MCM1_PWMINTF_SC2STATC_BIT                 (*(__O  uint32_t*)(0x42400EF0))
#define MCM1_PWMINTF_SC3STATC_BIT                 (*(__O  uint32_t*)(0x42400EF4))
#define MCM1_PSCON_SHIFTRUN_BIT                   (*(__IO uint32_t*)(0x42400F80))
#define MCM1_PSCON_STATRUN_BIT                    (*(__IO uint32_t*)(0x42400FA0))
#define MCM1_SC1CON_SCPWM0EN_BIT                  (*(__IO uint32_t*)(0x42401220))
#define MCM1_SC1CON_SCPWM1EN_BIT                  (*(__IO uint32_t*)(0x42401224))
#define MCM1_SC1CON_SCPWM2EN_BIT                  (*(__IO uint32_t*)(0x42401228))
#define MCM1_SC1CON_SCS_BIT                       (*(__IO uint32_t*)(0x4240122C))
#define MCM1_SC1CON_SCEN_BIT                      (*(__IO uint32_t*)(0x42401230))
#define MCM1_SC2CON_SCPWM0EN_BIT                  (*(__IO uint32_t*)(0x424012A0))
#define MCM1_SC2CON_SCPWM1EN_BIT                  (*(__IO uint32_t*)(0x424012A4))
#define MCM1_SC2CON_SCPWM2EN_BIT                  (*(__IO uint32_t*)(0x424012A8))
#define MCM1_SC2CON_SCS_BIT                       (*(__IO uint32_t*)(0x424012AC))
#define MCM1_SC2CON_SCEN_BIT                      (*(__IO uint32_t*)(0x424012B0))
#define MCM1_SC3CON_SCPWM0EN_BIT                  (*(__IO uint32_t*)(0x42401320))
#define MCM1_SC3CON_SCPWM1EN_BIT                  (*(__IO uint32_t*)(0x42401324))
#define MCM1_SC3CON_SCPWM2EN_BIT                  (*(__IO uint32_t*)(0x42401328))
#define MCM1_SC3CON_SCS_BIT                       (*(__IO uint32_t*)(0x4240132C))
#define MCM1_SC3CON_SCEN_BIT                      (*(__IO uint32_t*)(0x42401330))
#define MCM2_PWMOE_PWMOE_BIT                      (*(__IO uint32_t*)(0x42408000))
#define MCM2_PWMCON1_PWM0S_BIT                    (*(__IO uint32_t*)(0x42408080))
#define MCM2_PWMCON1_PWM1S_BIT                    (*(__IO uint32_t*)(0x42408084))
#define MCM2_PWMCON1_PWM2S_BIT                    (*(__IO uint32_t*)(0x42408088))
#define MCM2_PWMCON1_PWM01S_BIT                   (*(__IO uint32_t*)(0x4240808C))
#define MCM2_PWMCON1_PWM11S_BIT                   (*(__IO uint32_t*)(0x42408090))
#define MCM2_PWMCON1_PWM21S_BIT                   (*(__IO uint32_t*)(0x42408094))
#define MCM2_PWMCON1_PDCON0_BIT                   (*(__IO uint32_t*)(0x42408098))
#define MCM2_PWMCON1_PDCON1_BIT                   (*(__IO uint32_t*)(0x4240809C))
#define MCM2_PWMCON1_PDCON2_BIT                   (*(__IO uint32_t*)(0x424080A0))
#define MCM2_PWMCON1_PWMSYM_BIT                   (*(__IO uint32_t*)(0x424080A4))
#define MCM2_PWMCON1_POUTMOD_BIT                  (*(__IO uint32_t*)(0x424080BC))
#define MCM2_PWMCON2_OSYNC_BIT                    (*(__IO uint32_t*)(0x42408120))
#define MCM2_PWMCON2_DILDEN_BIT                   (*(__IO uint32_t*)(0x42408124))
#define MCM2_PWMCON2_CILDEN_BIT                   (*(__IO uint32_t*)(0x42408128))
#define MCM2_PWMCON2_ZDLDEN_BIT                   (*(__IO uint32_t*)(0x4240812C))
#define MCM2_PWMCON2_PDLDEN_BIT                   (*(__IO uint32_t*)(0x42408130))
#define MCM2_PWMCON2_ZCMLDEN_BIT                  (*(__IO uint32_t*)(0x42408134))
#define MCM2_PWMCON2_PCMLDEN_BIT                  (*(__IO uint32_t*)(0x42408138))
#define MCM2_PWMCON2_MCMSYNEN_BIT                 (*(__IO uint32_t*)(0x4240813C))
#define MCM2_PMANUALCON1_PMANUAL0_BIT             (*(__IO uint32_t*)(0x42408B00))
#define MCM2_PMANUALCON1_PMANUAL1_BIT             (*(__IO uint32_t*)(0x42408B04))
#define MCM2_PMANUALCON1_PMANUAL2_BIT             (*(__IO uint32_t*)(0x42408B08))
#define MCM2_PMANUALCON1_PMANUAL01_BIT            (*(__IO uint32_t*)(0x42408B0C))
#define MCM2_PMANUALCON1_PMANUAL11_BIT            (*(__IO uint32_t*)(0x42408B10))
#define MCM2_PMANUALCON1_PMANUAL21_BIT            (*(__IO uint32_t*)(0x42408B14))
#define MCM2_PMANUALCON2_POUT0_BIT                (*(__IO uint32_t*)(0x42408B80))
#define MCM2_PMANUALCON2_POUT1_BIT                (*(__IO uint32_t*)(0x42408B84))
#define MCM2_PMANUALCON2_POUT2_BIT                (*(__IO uint32_t*)(0x42408B88))
#define MCM2_PMANUALCON2_POUT01_BIT               (*(__IO uint32_t*)(0x42408B8C))
#define MCM2_PMANUALCON2_POUT11_BIT               (*(__IO uint32_t*)(0x42408B90))
#define MCM2_PMANUALCON2_POUT21_BIT               (*(__IO uint32_t*)(0x42408B94))
#define MCM2_FLTCON_FLTSTAT_BIT                   (*(__IO uint32_t*)(0x42408C00))
#define MCM2_FLTCON_FLTM_BIT                      (*(__IO uint32_t*)(0x42408C04))
#define MCM2_FLTCON_FLT2S_BIT                     (*(__IO uint32_t*)(0x42408C08))
#define MCM2_FLTCON_FLT2EN_BIT                    (*(__IO uint32_t*)(0x42408C0C))
#define MCM2_FLTCON_FLT1EN_BIT                    (*(__IO uint32_t*)(0x42408C28))
#define MCM2_POSCR_OLSG0_BIT                      (*(__IO uint32_t*)(0x42408C80))
#define MCM2_POSCR_OLSG1_BIT                      (*(__IO uint32_t*)(0x42408C84))
#define MCM2_POSCR_OLSG2_BIT                      (*(__IO uint32_t*)(0x42408C88))
#define MCM2_POSCR_OLSG01_BIT                     (*(__IO uint32_t*)(0x42408C8C))
#define MCM2_POSCR_OLSG11_BIT                     (*(__IO uint32_t*)(0x42408C90))
#define MCM2_POSCR_OLSG21_BIT                     (*(__IO uint32_t*)(0x42408C94))
#define MCM2_POSCR_OLSEN_BIT                      (*(__IO uint32_t*)(0x42408C9C))
#define MCM2_POSTDCR_OSTDEN_BIT                   (*(__IO uint32_t*)(0x42408D1C))
#define MCM2_PWMDMAEN_PTUD0DE_BIT                 (*(__IO uint32_t*)(0x42408D80))
#define MCM2_PWMDMAEN_PTDD0DE_BIT                 (*(__IO uint32_t*)(0x42408D84))
#define MCM2_PWMDMAEN_PTUD1DE_BIT                 (*(__IO uint32_t*)(0x42408D88))
#define MCM2_PWMDMAEN_PTDD1DE_BIT                 (*(__IO uint32_t*)(0x42408D8C))
#define MCM2_PWMDMAEN_PTUD2DE_BIT                 (*(__IO uint32_t*)(0x42408D90))
#define MCM2_PWMDMAEN_PTDD2DE_BIT                 (*(__IO uint32_t*)(0x42408D94))
#define MCM2_PWMDMAEN_PWMZDE_BIT                  (*(__IO uint32_t*)(0x42408D98))
#define MCM2_PWMDMAEN_PWMPDE_BIT                  (*(__IO uint32_t*)(0x42408D9C))
#define MCM2_PWMINTEN_PTUD0IE_BIT                 (*(__IO uint32_t*)(0x42408E00))
#define MCM2_PWMINTEN_PTDD0IE_BIT                 (*(__IO uint32_t*)(0x42408E04))
#define MCM2_PWMINTEN_PTUD1IE_BIT                 (*(__IO uint32_t*)(0x42408E08))
#define MCM2_PWMINTEN_PTDD1IE_BIT                 (*(__IO uint32_t*)(0x42408E0C))
#define MCM2_PWMINTEN_PTUD2IE_BIT                 (*(__IO uint32_t*)(0x42408E10))
#define MCM2_PWMINTEN_PTDD2IE_BIT                 (*(__IO uint32_t*)(0x42408E14))
#define MCM2_PWMINTEN_PWMZIE_BIT                  (*(__IO uint32_t*)(0x42408E18))
#define MCM2_PWMINTEN_PWMPIE_BIT                  (*(__IO uint32_t*)(0x42408E1C))
#define MCM2_PWMINTEN_FLTIE_BIT                   (*(__IO uint32_t*)(0x42408E20))
#define MCM2_PWMINTEN_OIE_BIT                     (*(__IO uint32_t*)(0x42408E24))
#define MCM2_PWMINTEN_OSTDIE_BIT                  (*(__IO uint32_t*)(0x42408E28))
#define MCM2_PWMINTF_PTUD0IF_BIT                  (*(__I  uint32_t*)(0x42408E80))
#define MCM2_PWMINTF_PTDD0IF_BIT                  (*(__I  uint32_t*)(0x42408E84))
#define MCM2_PWMINTF_PTUD1IF_BIT                  (*(__I  uint32_t*)(0x42408E88))
#define MCM2_PWMINTF_PTDD1IF_BIT                  (*(__I  uint32_t*)(0x42408E8C))
#define MCM2_PWMINTF_PTUD2IF_BIT                  (*(__I  uint32_t*)(0x42408E90))
#define MCM2_PWMINTF_PTDD2IF_BIT                  (*(__I  uint32_t*)(0x42408E94))
#define MCM2_PWMINTF_PWMZIF_BIT                   (*(__I  uint32_t*)(0x42408E98))
#define MCM2_PWMINTF_PWMPIF_BIT                   (*(__I  uint32_t*)(0x42408E9C))
#define MCM2_PWMINTF_FLTIF_BIT                    (*(__I  uint32_t*)(0x42408EA0))
#define MCM2_PWMINTF_OSF_BIT                      (*(__I  uint32_t*)(0x42408EA4))
#define MCM2_PWMINTF_OSTDF_BIT                    (*(__I  uint32_t*)(0x42408EA8))
#define MCM2_PWMINTF_SC1STAT_BIT                  (*(__I  uint32_t*)(0x42408EAC))
#define MCM2_PWMINTF_SC2STAT_BIT                  (*(__I  uint32_t*)(0x42408EB0))
#define MCM2_PWMINTF_SC3STAT_BIT                  (*(__I  uint32_t*)(0x42408EB4))
#define MCM2_PWMINTF_PTUD0IFC_BIT                 (*(__O  uint32_t*)(0x42408EC0))
#define MCM2_PWMINTF_PTDD0IFC_BIT                 (*(__O  uint32_t*)(0x42408EC4))
#define MCM2_PWMINTF_PTUD1IFC_BIT                 (*(__O  uint32_t*)(0x42408EC8))
#define MCM2_PWMINTF_PTDD1IFC_BIT                 (*(__O  uint32_t*)(0x42408ECC))
#define MCM2_PWMINTF_PTUD2IFC_BIT                 (*(__O  uint32_t*)(0x42408ED0))
#define MCM2_PWMINTF_PTDD2IFC_BIT                 (*(__O  uint32_t*)(0x42408ED4))
#define MCM2_PWMINTF_PWMZIFC_BIT                  (*(__O  uint32_t*)(0x42408ED8))
#define MCM2_PWMINTF_PWMPIFC_BIT                  (*(__O  uint32_t*)(0x42408EDC))
#define MCM2_PWMINTF_FLTIFC_BIT                   (*(__O  uint32_t*)(0x42408EE0))
#define MCM2_PWMINTF_OSFC_BIT                     (*(__O  uint32_t*)(0x42408EE4))
#define MCM2_PWMINTF_OSTDFC_BIT                   (*(__O  uint32_t*)(0x42408EE8))
#define MCM2_PWMINTF_SC1STATC_BIT                 (*(__O  uint32_t*)(0x42408EEC))
#define MCM2_PWMINTF_SC2STATC_BIT                 (*(__O  uint32_t*)(0x42408EF0))
#define MCM2_PWMINTF_SC3STATC_BIT                 (*(__O  uint32_t*)(0x42408EF4))
#define MCM2_PSCON_SHIFTRUN_BIT                   (*(__IO uint32_t*)(0x42408F80))
#define MCM2_PSCON_STATRUN_BIT                    (*(__IO uint32_t*)(0x42408FA0))
#define MCM2_SC1CON_SCPWM0EN_BIT                  (*(__IO uint32_t*)(0x42409220))
#define MCM2_SC1CON_SCPWM1EN_BIT                  (*(__IO uint32_t*)(0x42409224))
#define MCM2_SC1CON_SCPWM2EN_BIT                  (*(__IO uint32_t*)(0x42409228))
#define MCM2_SC1CON_SCS_BIT                       (*(__IO uint32_t*)(0x4240922C))
#define MCM2_SC1CON_SCEN_BIT                      (*(__IO uint32_t*)(0x42409230))
#define MCM2_SC2CON_SCPWM0EN_BIT                  (*(__IO uint32_t*)(0x424092A0))
#define MCM2_SC2CON_SCPWM1EN_BIT                  (*(__IO uint32_t*)(0x424092A4))
#define MCM2_SC2CON_SCPWM2EN_BIT                  (*(__IO uint32_t*)(0x424092A8))
#define MCM2_SC2CON_SCS_BIT                       (*(__IO uint32_t*)(0x424092AC))
#define MCM2_SC2CON_SCEN_BIT                      (*(__IO uint32_t*)(0x424092B0))
#define MCM2_SC3CON_SCPWM0EN_BIT                  (*(__IO uint32_t*)(0x42409320))
#define MCM2_SC3CON_SCPWM1EN_BIT                  (*(__IO uint32_t*)(0x42409324))
#define MCM2_SC3CON_SCPWM2EN_BIT                  (*(__IO uint32_t*)(0x42409328))
#define MCM2_SC3CON_SCS_BIT                       (*(__IO uint32_t*)(0x4240932C))
#define MCM2_SC3CON_SCEN_BIT                      (*(__IO uint32_t*)(0x42409330))
#define GPT_GTSTR_CST0_BIT                        (*(__IO uint32_t*)(0x42840000))
#define GPT_GTSTR_CST1_BIT                        (*(__IO uint32_t*)(0x42840004))
#define GPT_GTSTR_CST2_BIT                        (*(__IO uint32_t*)(0x42840008))
#define GPT_GTSTR_CST3_BIT                        (*(__IO uint32_t*)(0x4284000C))
#define GPT_GTETINT_ETIPEN_BIT                    (*(__IO uint32_t*)(0x42840080))
#define GPT_GTETINT_ETINEN_BIT                    (*(__IO uint32_t*)(0x42840084))
#define GPT_GTETINT_ETPDMA_BIT                    (*(__IO uint32_t*)(0x42840088))
#define GPT_GTETINT_ETNDMA_BIT                    (*(__IO uint32_t*)(0x4284008C))
#define GPT_GTPOECR_GPT0ABZE0_BIT                 (*(__IO uint32_t*)(0x42840100))
#define GPT_GTPOECR_GPT1ABZE0_BIT                 (*(__IO uint32_t*)(0x42840104))
#define GPT_GTPOECR_GPT2ABZE0_BIT                 (*(__IO uint32_t*)(0x42840108))
#define GPT_GTPOECR_GPT3ABZE0_BIT                 (*(__IO uint32_t*)(0x4284010C))
#define GPT_GTPOECR_POEIE_BIT                     (*(__IO uint32_t*)(0x4284011C))
#define GPT_GTINTF_ETIPF_BIT                      (*(__I  uint32_t*)(0x42840400))
#define GPT_GTINTF_ETINF_BIT                      (*(__I  uint32_t*)(0x42840404))
#define GPT_GTINTF_POEIF_BIT                      (*(__I  uint32_t*)(0x42840408))
#define GPT_GTINTF_ETIPFC_BIT                     (*(__O  uint32_t*)(0x42840440))
#define GPT_GTINTF_ETINFC_BIT                     (*(__O  uint32_t*)(0x42840444))
#define GPT_GTINTF_POEIFC_BIT                     (*(__O  uint32_t*)(0x42840448))
#define GPT0_GTHCR_CCSW_BIT                       (*(__O  uint32_t*)(0x42848020))
#define GPT0_GTDEB_INAE_BIT                       (*(__IO uint32_t*)(0x428480A0))
#define GPT0_GTDEB_INBE_BIT                       (*(__IO uint32_t*)(0x428480A4))
#define GPT0_GTBDR_BDCCR_BIT                      (*(__IO uint32_t*)(0x42848180))
#define GPT0_GTBDR_BDPR_BIT                       (*(__IO uint32_t*)(0x42848184))
#define GPT0_GTBDR_BDADTR_BIT                     (*(__IO uint32_t*)(0x42848188))
#define GPT0_GTBDR_BDDV_BIT                       (*(__IO uint32_t*)(0x4284818C))
#define GPT0_GTIOR_OADFLT_BIT                     (*(__IO uint32_t*)(0x42848218))
#define GPT0_GTIOR_OAHLD_BIT                      (*(__IO uint32_t*)(0x4284821C))
#define GPT0_GTIOR_OBDFLT_BIT                     (*(__IO uint32_t*)(0x42848238))
#define GPT0_GTIOR_OBHLD_BIT                      (*(__IO uint32_t*)(0x4284823C))
#define GPT0_GTINTAD_GTINTA_BIT                   (*(__IO uint32_t*)(0x42848280))
#define GPT0_GTINTAD_GTINTB_BIT                   (*(__IO uint32_t*)(0x42848284))
#define GPT0_GTINTAD_EINT_BIT                     (*(__IO uint32_t*)(0x42848290))
#define GPT0_GTINTAD_OINT_BIT                     (*(__IO uint32_t*)(0x42848294))
#define GPT0_GTINTAD_ADTRAUEN_BIT                 (*(__IO uint32_t*)(0x428482A0))
#define GPT0_GTINTAD_ADTRADEN_BIT                 (*(__IO uint32_t*)(0x428482A4))
#define GPT0_GTINTAD_ADTRBUEN_BIT                 (*(__IO uint32_t*)(0x428482A8))
#define GPT0_GTINTAD_ADTRBDEN_BIT                 (*(__IO uint32_t*)(0x428482AC))
#define GPT0_GTDMA_TA_BIT                         (*(__IO uint32_t*)(0x42848300))
#define GPT0_GTDMA_TB_BIT                         (*(__IO uint32_t*)(0x42848304))
#define GPT0_GTCR_TPSC_BIT                        (*(__IO uint32_t*)(0x428483A0))
#define GPT0_GTCR_HIZ_BIT                         (*(__IO uint32_t*)(0x428483A4))
#define GPT0_GTBER_CCRSWT_BIT                     (*(__O  uint32_t*)(0x42848498))
#define GPT0_GTBER_ADTDA_BIT                      (*(__IO uint32_t*)(0x428484A8))
#define GPT0_GTBER_ADTDB_BIT                      (*(__IO uint32_t*)(0x428484B8))
#define GPT0_GTUDC_UD_BIT                         (*(__IO uint32_t*)(0x42848500))
#define GPT0_GTUDC_UDF_BIT                        (*(__IO uint32_t*)(0x42848504))
#define GPT0_GTITC_ITLA_BIT                       (*(__IO uint32_t*)(0x42848580))
#define GPT0_GTITC_ITLB_BIT                       (*(__IO uint32_t*)(0x42848584))
#define GPT0_GTITC_ADTAL_BIT                      (*(__IO uint32_t*)(0x428485B0))
#define GPT0_GTITC_ADTBL_BIT                      (*(__IO uint32_t*)(0x428485B8))
#define GPT0_GTST_TCFA_BIT                        (*(__I  uint32_t*)(0x42848600))
#define GPT0_GTST_TCFB_BIT                        (*(__I  uint32_t*)(0x42848604))
#define GPT0_GTST_TCFPO_BIT                       (*(__I  uint32_t*)(0x42848608))
#define GPT0_GTST_TCFPU_BIT                       (*(__I  uint32_t*)(0x4284860C))
#define GPT0_GTST_DTEF_BIT                        (*(__I  uint32_t*)(0x42848610))
#define GPT0_GTST_OSF_BIT                         (*(__I  uint32_t*)(0x42848614))
#define GPT0_GTST_TUCF_BIT                        (*(__I  uint32_t*)(0x4284863C))
#define GPT0_GTST_TCFAC_BIT                       (*(__O  uint32_t*)(0x42848640))
#define GPT0_GTST_TCFBC_BIT                       (*(__O  uint32_t*)(0x42848644))
#define GPT0_GTST_TCFPOC_BIT                      (*(__O  uint32_t*)(0x42848648))
#define GPT0_GTST_TCFPUC_BIT                      (*(__O  uint32_t*)(0x4284864C))
#define GPT0_GTST_DTEFC_BIT                       (*(__O  uint32_t*)(0x42848650))
#define GPT0_GTST_OSFC_BIT                        (*(__O  uint32_t*)(0x42848654))
#define GPT0_GTDTCR_TDE_BIT                       (*(__IO uint32_t*)(0x42848E80))
#define GPT0_GTDTCR_TDBUE_BIT                     (*(__IO uint32_t*)(0x42848E90))
#define GPT0_GTDTCR_TDBDE_BIT                     (*(__IO uint32_t*)(0x42848E94))
#define GPT0_GTDTCR_TDFER_BIT                     (*(__IO uint32_t*)(0x42848EA0))
#define GPT0_GTOSCR_OLSGA_BIT                     (*(__IO uint32_t*)(0x42849100))
#define GPT0_GTOSCR_OLSGB_BIT                     (*(__IO uint32_t*)(0x42849104))
#define GPT0_GTOSCR_OLSEN_BIT                     (*(__IO uint32_t*)(0x4284911C))
#define GPT1_GTHCR_CCSW_BIT                       (*(__O  uint32_t*)(0x42850020))
#define GPT1_GTDEB_INAE_BIT                       (*(__IO uint32_t*)(0x428500A0))
#define GPT1_GTDEB_INBE_BIT                       (*(__IO uint32_t*)(0x428500A4))
#define GPT1_GTBDR_BDCCR_BIT                      (*(__IO uint32_t*)(0x42850180))
#define GPT1_GTBDR_BDPR_BIT                       (*(__IO uint32_t*)(0x42850184))
#define GPT1_GTBDR_BDADTR_BIT                     (*(__IO uint32_t*)(0x42850188))
#define GPT1_GTBDR_BDDV_BIT                       (*(__IO uint32_t*)(0x4285018C))
#define GPT1_GTIOR_OADFLT_BIT                     (*(__IO uint32_t*)(0x42850218))
#define GPT1_GTIOR_OAHLD_BIT                      (*(__IO uint32_t*)(0x4285021C))
#define GPT1_GTIOR_OBDFLT_BIT                     (*(__IO uint32_t*)(0x42850238))
#define GPT1_GTIOR_OBHLD_BIT                      (*(__IO uint32_t*)(0x4285023C))
#define GPT1_GTINTAD_GTINTA_BIT                   (*(__IO uint32_t*)(0x42850280))
#define GPT1_GTINTAD_GTINTB_BIT                   (*(__IO uint32_t*)(0x42850284))
#define GPT1_GTINTAD_EINT_BIT                     (*(__IO uint32_t*)(0x42850290))
#define GPT1_GTINTAD_OINT_BIT                     (*(__IO uint32_t*)(0x42850294))
#define GPT1_GTINTAD_ADTRAUEN_BIT                 (*(__IO uint32_t*)(0x428502A0))
#define GPT1_GTINTAD_ADTRADEN_BIT                 (*(__IO uint32_t*)(0x428502A4))
#define GPT1_GTINTAD_ADTRBUEN_BIT                 (*(__IO uint32_t*)(0x428502A8))
#define GPT1_GTINTAD_ADTRBDEN_BIT                 (*(__IO uint32_t*)(0x428502AC))
#define GPT1_GTDMA_TA_BIT                         (*(__IO uint32_t*)(0x42850300))
#define GPT1_GTDMA_TB_BIT                         (*(__IO uint32_t*)(0x42850304))
#define GPT1_GTCR_TPSC_BIT                        (*(__IO uint32_t*)(0x428503A0))
#define GPT1_GTCR_HIZ_BIT                         (*(__IO uint32_t*)(0x428503A4))
#define GPT1_GTBER_CCRSWT_BIT                     (*(__O  uint32_t*)(0x42850498))
#define GPT1_GTBER_ADTDA_BIT                      (*(__IO uint32_t*)(0x428504A8))
#define GPT1_GTBER_ADTDB_BIT                      (*(__IO uint32_t*)(0x428504B8))
#define GPT1_GTUDC_UD_BIT                         (*(__IO uint32_t*)(0x42850500))
#define GPT1_GTUDC_UDF_BIT                        (*(__IO uint32_t*)(0x42850504))
#define GPT1_GTITC_ITLA_BIT                       (*(__IO uint32_t*)(0x42850580))
#define GPT1_GTITC_ITLB_BIT                       (*(__IO uint32_t*)(0x42850584))
#define GPT1_GTITC_ADTAL_BIT                      (*(__IO uint32_t*)(0x428505B0))
#define GPT1_GTITC_ADTBL_BIT                      (*(__IO uint32_t*)(0x428505B8))
#define GPT1_GTST_TCFA_BIT                        (*(__I  uint32_t*)(0x42850600))
#define GPT1_GTST_TCFB_BIT                        (*(__I  uint32_t*)(0x42850604))
#define GPT1_GTST_TCFPO_BIT                       (*(__I  uint32_t*)(0x42850608))
#define GPT1_GTST_TCFPU_BIT                       (*(__I  uint32_t*)(0x4285060C))
#define GPT1_GTST_DTEF_BIT                        (*(__I  uint32_t*)(0x42850610))
#define GPT1_GTST_OSF_BIT                         (*(__I  uint32_t*)(0x42850614))
#define GPT1_GTST_TUCF_BIT                        (*(__I  uint32_t*)(0x4285063C))
#define GPT1_GTST_TCFAC_BIT                       (*(__O  uint32_t*)(0x42850640))
#define GPT1_GTST_TCFBC_BIT                       (*(__O  uint32_t*)(0x42850644))
#define GPT1_GTST_TCFPOC_BIT                      (*(__O  uint32_t*)(0x42850648))
#define GPT1_GTST_TCFPUC_BIT                      (*(__O  uint32_t*)(0x4285064C))
#define GPT1_GTST_DTEFC_BIT                       (*(__O  uint32_t*)(0x42850650))
#define GPT1_GTST_OSFC_BIT                        (*(__O  uint32_t*)(0x42850654))
#define GPT1_GTDTCR_TDE_BIT                       (*(__IO uint32_t*)(0x42850E80))
#define GPT1_GTDTCR_TDBUE_BIT                     (*(__IO uint32_t*)(0x42850E90))
#define GPT1_GTDTCR_TDBDE_BIT                     (*(__IO uint32_t*)(0x42850E94))
#define GPT1_GTDTCR_TDFER_BIT                     (*(__IO uint32_t*)(0x42850EA0))
#define GPT1_GTOSCR_OLSGA_BIT                     (*(__IO uint32_t*)(0x42851100))
#define GPT1_GTOSCR_OLSGB_BIT                     (*(__IO uint32_t*)(0x42851104))
#define GPT1_GTOSCR_OLSEN_BIT                     (*(__IO uint32_t*)(0x4285111C))
#define GPT2_GTHCR_CCSW_BIT                       (*(__O  uint32_t*)(0x42858020))
#define GPT2_GTDEB_INAE_BIT                       (*(__IO uint32_t*)(0x428580A0))
#define GPT2_GTDEB_INBE_BIT                       (*(__IO uint32_t*)(0x428580A4))
#define GPT2_GTBDR_BDCCR_BIT                      (*(__IO uint32_t*)(0x42858180))
#define GPT2_GTBDR_BDPR_BIT                       (*(__IO uint32_t*)(0x42858184))
#define GPT2_GTBDR_BDADTR_BIT                     (*(__IO uint32_t*)(0x42858188))
#define GPT2_GTBDR_BDDV_BIT                       (*(__IO uint32_t*)(0x4285818C))
#define GPT2_GTIOR_OADFLT_BIT                     (*(__IO uint32_t*)(0x42858218))
#define GPT2_GTIOR_OAHLD_BIT                      (*(__IO uint32_t*)(0x4285821C))
#define GPT2_GTIOR_OBDFLT_BIT                     (*(__IO uint32_t*)(0x42858238))
#define GPT2_GTIOR_OBHLD_BIT                      (*(__IO uint32_t*)(0x4285823C))
#define GPT2_GTINTAD_GTINTA_BIT                   (*(__IO uint32_t*)(0x42858280))
#define GPT2_GTINTAD_GTINTB_BIT                   (*(__IO uint32_t*)(0x42858284))
#define GPT2_GTINTAD_EINT_BIT                     (*(__IO uint32_t*)(0x42858290))
#define GPT2_GTINTAD_OINT_BIT                     (*(__IO uint32_t*)(0x42858294))
#define GPT2_GTINTAD_ADTRAUEN_BIT                 (*(__IO uint32_t*)(0x428582A0))
#define GPT2_GTINTAD_ADTRADEN_BIT                 (*(__IO uint32_t*)(0x428582A4))
#define GPT2_GTINTAD_ADTRBUEN_BIT                 (*(__IO uint32_t*)(0x428582A8))
#define GPT2_GTINTAD_ADTRBDEN_BIT                 (*(__IO uint32_t*)(0x428582AC))
#define GPT2_GTDMA_TA_BIT                         (*(__IO uint32_t*)(0x42858300))
#define GPT2_GTDMA_TB_BIT                         (*(__IO uint32_t*)(0x42858304))
#define GPT2_GTCR_TPSC_BIT                        (*(__IO uint32_t*)(0x428583A0))
#define GPT2_GTCR_HIZ_BIT                         (*(__IO uint32_t*)(0x428583A4))
#define GPT2_GTBER_CCRSWT_BIT                     (*(__O  uint32_t*)(0x42858498))
#define GPT2_GTBER_ADTDA_BIT                      (*(__IO uint32_t*)(0x428584A8))
#define GPT2_GTBER_ADTDB_BIT                      (*(__IO uint32_t*)(0x428584B8))
#define GPT2_GTUDC_UD_BIT                         (*(__IO uint32_t*)(0x42858500))
#define GPT2_GTUDC_UDF_BIT                        (*(__IO uint32_t*)(0x42858504))
#define GPT2_GTITC_ITLA_BIT                       (*(__IO uint32_t*)(0x42858580))
#define GPT2_GTITC_ITLB_BIT                       (*(__IO uint32_t*)(0x42858584))
#define GPT2_GTITC_ADTAL_BIT                      (*(__IO uint32_t*)(0x428585B0))
#define GPT2_GTITC_ADTBL_BIT                      (*(__IO uint32_t*)(0x428585B8))
#define GPT2_GTST_TCFA_BIT                        (*(__I  uint32_t*)(0x42858600))
#define GPT2_GTST_TCFB_BIT                        (*(__I  uint32_t*)(0x42858604))
#define GPT2_GTST_TCFPO_BIT                       (*(__I  uint32_t*)(0x42858608))
#define GPT2_GTST_TCFPU_BIT                       (*(__I  uint32_t*)(0x4285860C))
#define GPT2_GTST_DTEF_BIT                        (*(__I  uint32_t*)(0x42858610))
#define GPT2_GTST_OSF_BIT                         (*(__I  uint32_t*)(0x42858614))
#define GPT2_GTST_TUCF_BIT                        (*(__I  uint32_t*)(0x4285863C))
#define GPT2_GTST_TCFAC_BIT                       (*(__O  uint32_t*)(0x42858640))
#define GPT2_GTST_TCFBC_BIT                       (*(__O  uint32_t*)(0x42858644))
#define GPT2_GTST_TCFPOC_BIT                      (*(__O  uint32_t*)(0x42858648))
#define GPT2_GTST_TCFPUC_BIT                      (*(__O  uint32_t*)(0x4285864C))
#define GPT2_GTST_DTEFC_BIT                       (*(__O  uint32_t*)(0x42858650))
#define GPT2_GTST_OSFC_BIT                        (*(__O  uint32_t*)(0x42858654))
#define GPT2_GTDTCR_TDE_BIT                       (*(__IO uint32_t*)(0x42858E80))
#define GPT2_GTDTCR_TDBUE_BIT                     (*(__IO uint32_t*)(0x42858E90))
#define GPT2_GTDTCR_TDBDE_BIT                     (*(__IO uint32_t*)(0x42858E94))
#define GPT2_GTDTCR_TDFER_BIT                     (*(__IO uint32_t*)(0x42858EA0))
#define GPT2_GTOSCR_OLSGA_BIT                     (*(__IO uint32_t*)(0x42859100))
#define GPT2_GTOSCR_OLSGB_BIT                     (*(__IO uint32_t*)(0x42859104))
#define GPT2_GTOSCR_OLSEN_BIT                     (*(__IO uint32_t*)(0x4285911C))
#define GPT3_GTHCR_CCSW_BIT                       (*(__O  uint32_t*)(0x42860020))
#define GPT3_GTDEB_INAE_BIT                       (*(__IO uint32_t*)(0x428600A0))
#define GPT3_GTDEB_INBE_BIT                       (*(__IO uint32_t*)(0x428600A4))
#define GPT3_GTBDR_BDCCR_BIT                      (*(__IO uint32_t*)(0x42860180))
#define GPT3_GTBDR_BDPR_BIT                       (*(__IO uint32_t*)(0x42860184))
#define GPT3_GTBDR_BDADTR_BIT                     (*(__IO uint32_t*)(0x42860188))
#define GPT3_GTBDR_BDDV_BIT                       (*(__IO uint32_t*)(0x4286018C))
#define GPT3_GTIOR_OADFLT_BIT                     (*(__IO uint32_t*)(0x42860218))
#define GPT3_GTIOR_OAHLD_BIT                      (*(__IO uint32_t*)(0x4286021C))
#define GPT3_GTIOR_OBDFLT_BIT                     (*(__IO uint32_t*)(0x42860238))
#define GPT3_GTIOR_OBHLD_BIT                      (*(__IO uint32_t*)(0x4286023C))
#define GPT3_GTINTAD_GTINTA_BIT                   (*(__IO uint32_t*)(0x42860280))
#define GPT3_GTINTAD_GTINTB_BIT                   (*(__IO uint32_t*)(0x42860284))
#define GPT3_GTINTAD_EINT_BIT                     (*(__IO uint32_t*)(0x42860290))
#define GPT3_GTINTAD_OINT_BIT                     (*(__IO uint32_t*)(0x42860294))
#define GPT3_GTINTAD_ADTRAUEN_BIT                 (*(__IO uint32_t*)(0x428602A0))
#define GPT3_GTINTAD_ADTRADEN_BIT                 (*(__IO uint32_t*)(0x428602A4))
#define GPT3_GTINTAD_ADTRBUEN_BIT                 (*(__IO uint32_t*)(0x428602A8))
#define GPT3_GTINTAD_ADTRBDEN_BIT                 (*(__IO uint32_t*)(0x428602AC))
#define GPT3_GTDMA_TA_BIT                         (*(__IO uint32_t*)(0x42860300))
#define GPT3_GTDMA_TB_BIT                         (*(__IO uint32_t*)(0x42860304))
#define GPT3_GTCR_TPSC_BIT                        (*(__IO uint32_t*)(0x428603A0))
#define GPT3_GTCR_HIZ_BIT                         (*(__IO uint32_t*)(0x428603A4))
#define GPT3_GTBER_CCRSWT_BIT                     (*(__O  uint32_t*)(0x42860498))
#define GPT3_GTBER_ADTDA_BIT                      (*(__IO uint32_t*)(0x428604A8))
#define GPT3_GTBER_ADTDB_BIT                      (*(__IO uint32_t*)(0x428604B8))
#define GPT3_GTUDC_UD_BIT                         (*(__IO uint32_t*)(0x42860500))
#define GPT3_GTUDC_UDF_BIT                        (*(__IO uint32_t*)(0x42860504))
#define GPT3_GTITC_ITLA_BIT                       (*(__IO uint32_t*)(0x42860580))
#define GPT3_GTITC_ITLB_BIT                       (*(__IO uint32_t*)(0x42860584))
#define GPT3_GTITC_ADTAL_BIT                      (*(__IO uint32_t*)(0x428605B0))
#define GPT3_GTITC_ADTBL_BIT                      (*(__IO uint32_t*)(0x428605B8))
#define GPT3_GTST_TCFA_BIT                        (*(__I  uint32_t*)(0x42860600))
#define GPT3_GTST_TCFB_BIT                        (*(__I  uint32_t*)(0x42860604))
#define GPT3_GTST_TCFPO_BIT                       (*(__I  uint32_t*)(0x42860608))
#define GPT3_GTST_TCFPU_BIT                       (*(__I  uint32_t*)(0x4286060C))
#define GPT3_GTST_DTEF_BIT                        (*(__I  uint32_t*)(0x42860610))
#define GPT3_GTST_OSF_BIT                         (*(__I  uint32_t*)(0x42860614))
#define GPT3_GTST_TUCF_BIT                        (*(__I  uint32_t*)(0x4286063C))
#define GPT3_GTST_TCFAC_BIT                       (*(__O  uint32_t*)(0x42860640))
#define GPT3_GTST_TCFBC_BIT                       (*(__O  uint32_t*)(0x42860644))
#define GPT3_GTST_TCFPOC_BIT                      (*(__O  uint32_t*)(0x42860648))
#define GPT3_GTST_TCFPUC_BIT                      (*(__O  uint32_t*)(0x4286064C))
#define GPT3_GTST_DTEFC_BIT                       (*(__O  uint32_t*)(0x42860650))
#define GPT3_GTST_OSFC_BIT                        (*(__O  uint32_t*)(0x42860654))
#define GPT3_GTDTCR_TDE_BIT                       (*(__IO uint32_t*)(0x42860E80))
#define GPT3_GTDTCR_TDBUE_BIT                     (*(__IO uint32_t*)(0x42860E90))
#define GPT3_GTDTCR_TDBDE_BIT                     (*(__IO uint32_t*)(0x42860E94))
#define GPT3_GTDTCR_TDFER_BIT                     (*(__IO uint32_t*)(0x42860EA0))
#define GPT3_GTOSCR_OLSGA_BIT                     (*(__IO uint32_t*)(0x42861100))
#define GPT3_GTOSCR_OLSGB_BIT                     (*(__IO uint32_t*)(0x42861104))
#define GPT3_GTOSCR_OLSEN_BIT                     (*(__IO uint32_t*)(0x4286111C))
#define TIM5_CR_STR_BIT                           (*(__IO uint32_t*)(0x42008000))
#define TIM5_CR_OPM_BIT                           (*(__IO uint32_t*)(0x4200800C))
#define TIM5_CR_IE_BIT                            (*(__IO uint32_t*)(0x42008018))
#define TIM5_CR_TRIGEN_BIT                        (*(__IO uint32_t*)(0x42008020))
#define TIM5_CR_ETEN_BIT                          (*(__IO uint32_t*)(0x42008024))
#define TIM5_CR_TC_BIT                            (*(__IO uint32_t*)(0x42008028))
#define TIM5_CR_CASCEN_BIT                        (*(__IO uint32_t*)(0x4200803C))
#define TIM5_TIMINTF_TF_BIT                       (*(__I  uint32_t*)(0x42008200))
#define TIM5_TIMINTF_TFC_BIT                      (*(__O  uint32_t*)(0x42008240))
#define TIM6_CR_STR_BIT                           (*(__IO uint32_t*)(0x42010000))
#define TIM6_CR_OPM_BIT                           (*(__IO uint32_t*)(0x4201000C))
#define TIM6_CR_IE_BIT                            (*(__IO uint32_t*)(0x42010018))
#define TIM6_CR_TRIGEN_BIT                        (*(__IO uint32_t*)(0x42010020))
#define TIM6_CR_ETEN_BIT                          (*(__IO uint32_t*)(0x42010024))
#define TIM6_CR_TC_BIT                            (*(__IO uint32_t*)(0x42010028))
#define TIM6_CR_CASCEN_BIT                        (*(__IO uint32_t*)(0x4201003C))
#define TIM6_TIMINTF_TF_BIT                       (*(__I  uint32_t*)(0x42010200))
#define TIM6_TIMINTF_TFC_BIT                      (*(__O  uint32_t*)(0x42010240))
#define TIM7_CR_STR_BIT                           (*(__IO uint32_t*)(0x42018000))
#define TIM7_CR_OPM_BIT                           (*(__IO uint32_t*)(0x4201800C))
#define TIM7_CR_IE_BIT                            (*(__IO uint32_t*)(0x42018018))
#define TIM7_CR_TRIGEN_BIT                        (*(__IO uint32_t*)(0x42018020))
#define TIM7_CR_ETEN_BIT                          (*(__IO uint32_t*)(0x42018024))
#define TIM7_CR_TC_BIT                            (*(__IO uint32_t*)(0x42018028))
#define TIM7_CR_CASCEN_BIT                        (*(__IO uint32_t*)(0x4201803C))
#define TIM7_TIMINTF_TF_BIT                       (*(__I  uint32_t*)(0x42018200))
#define TIM7_TIMINTF_TFC_BIT                      (*(__O  uint32_t*)(0x42018240))
#define TIM8_CR_STR_BIT                           (*(__IO uint32_t*)(0x42020000))
#define TIM8_CR_OPM_BIT                           (*(__IO uint32_t*)(0x4202000C))
#define TIM8_CR_IE_BIT                            (*(__IO uint32_t*)(0x42020018))
#define TIM8_CR_TRIGEN_BIT                        (*(__IO uint32_t*)(0x42020020))
#define TIM8_CR_ETEN_BIT                          (*(__IO uint32_t*)(0x42020024))
#define TIM8_CR_TC_BIT                            (*(__IO uint32_t*)(0x42020028))
#define TIM8_CR_CASCEN_BIT                        (*(__IO uint32_t*)(0x4202003C))
#define TIM8_TIMINTF_TF_BIT                       (*(__I  uint32_t*)(0x42020200))
#define TIM8_TIMINTF_TFC_BIT                      (*(__O  uint32_t*)(0x42020240))
#define MACP_CORDCSR0_RUN_BIT                     (*(__IO uint32_t*)(0x428A8000))
#define MACP_CORDCSR0_OVF_BIT                     (*(__I  uint32_t*)(0x428A8004))
#define MACP_CORDCSR0_KADJ_BIT                    (*(__IO uint32_t*)(0x428A8008))
#define MACP_CORDCSR0_XYMRS_BIT                   (*(__IO uint32_t*)(0x428A800C))
#define MACP_CORDCSR0_MODE_BIT                    (*(__IO uint32_t*)(0x428A801C))
#define MACP_CORDCSR1_RUN_BIT                     (*(__IO uint32_t*)(0x428A8200))
#define MACP_CORDCSR1_OVF_BIT                     (*(__I  uint32_t*)(0x428A8204))
#define MACP_CORDCSR1_KADJ_BIT                    (*(__IO uint32_t*)(0x428A8208))
#define MACP_CORDCSR1_XYMRS_BIT                   (*(__IO uint32_t*)(0x428A820C))
#define MACP_CORDCSR1_MODE_BIT                    (*(__IO uint32_t*)(0x428A821C))
#define MACP_IQDIVCSR0_RUN_BIT                    (*(__IO uint32_t*)(0x428AA000))
#define MACP_IQDIVCSR0_SIGN_BIT                   (*(__IO uint32_t*)(0x428AA004))
#define MACP_IQDIVCSR0_SAT_BIT                    (*(__IO uint32_t*)(0x428AA008))
#define MACP_IQDIVCSR0_CMOD_BIT                   (*(__IO uint32_t*)(0x428AA020))
#define MACP_IQDIVCSR1_RUN_BIT                    (*(__IO uint32_t*)(0x428AA200))
#define MACP_IQDIVCSR1_SIGN_BIT                   (*(__IO uint32_t*)(0x428AA204))
#define MACP_IQDIVCSR1_SAT_BIT                    (*(__IO uint32_t*)(0x428AA208))
#define MACP_IQDIVCSR1_CMOD_BIT                   (*(__IO uint32_t*)(0x428AA220))
#define MACP_SVCON_RUN_BIT                        (*(__IO uint32_t*)(0x428AC000))
#define MACP_SVIQN_SVTYP_BIT                      (*(__IO uint32_t*)(0x428AC1A0))
#define ADC1_ADCON1_ADSOC_BIT                     (*(__IO uint32_t*)(0x42820000))
#define ADC1_ADCON1_MOD_BIT                       (*(__IO uint32_t*)(0x4282000C))
#define ADC1_ADCON1_ADDE_BIT                      (*(__IO uint32_t*)(0x42820018))
#define ADC1_ADCON1_ADIE_BIT                      (*(__IO uint32_t*)(0x4282001C))
#define ADC1_ADCON1_ADON_BIT                      (*(__IO uint32_t*)(0x4282003C))
#define ADC1_ADCMPCON_ADLDE_BIT                   (*(__IO uint32_t*)(0x42820590))
#define ADC1_ADCMPCON_ADLIE_BIT                   (*(__IO uint32_t*)(0x42820594))
#define ADC1_ADCMPCON_ADGDE_BIT                   (*(__IO uint32_t*)(0x42820598))
#define ADC1_ADCMPCON_ADGIE_BIT                   (*(__IO uint32_t*)(0x4282059C))
#define ADC1_ADINTF_ADLIF_BIT                     (*(__I  uint32_t*)(0x42820800))
#define ADC1_ADINTF_ADGIF_BIT                     (*(__I  uint32_t*)(0x42820804))
#define ADC1_ADINTF_ADIF_BIT                      (*(__I  uint32_t*)(0x42820808))
#define ADC1_ADINTF_ADLIFC_BIT                    (*(__O  uint32_t*)(0x42820840))
#define ADC1_ADINTF_ADGIFC_BIT                    (*(__O  uint32_t*)(0x42820844))
#define ADC1_ADINTF_ADIFC_BIT                     (*(__O  uint32_t*)(0x42820848))
#define ADC2_ADCON1_ADSOC_BIT                     (*(__IO uint32_t*)(0x42828000))
#define ADC2_ADCON1_MOD_BIT                       (*(__IO uint32_t*)(0x4282800C))
#define ADC2_ADCON1_ADDE_BIT                      (*(__IO uint32_t*)(0x42828018))
#define ADC2_ADCON1_ADIE_BIT                      (*(__IO uint32_t*)(0x4282801C))
#define ADC2_ADCON1_ADON_BIT                      (*(__IO uint32_t*)(0x4282803C))
#define ADC2_ADCMPCON_ADLDE_BIT                   (*(__IO uint32_t*)(0x42828590))
#define ADC2_ADCMPCON_ADLIE_BIT                   (*(__IO uint32_t*)(0x42828594))
#define ADC2_ADCMPCON_ADGDE_BIT                   (*(__IO uint32_t*)(0x42828598))
#define ADC2_ADCMPCON_ADGIE_BIT                   (*(__IO uint32_t*)(0x4282859C))
#define ADC2_ADINTF_ADLIF_BIT                     (*(__I  uint32_t*)(0x42828800))
#define ADC2_ADINTF_ADGIF_BIT                     (*(__I  uint32_t*)(0x42828804))
#define ADC2_ADINTF_ADIF_BIT                      (*(__I  uint32_t*)(0x42828808))
#define ADC2_ADINTF_ADLIFC_BIT                    (*(__O  uint32_t*)(0x42828840))
#define ADC2_ADINTF_ADGIFC_BIT                    (*(__O  uint32_t*)(0x42828844))
#define ADC2_ADINTF_ADIFC_BIT                     (*(__O  uint32_t*)(0x42828848))
#define ADC3_ADCON1_ADSOC_BIT                     (*(__IO uint32_t*)(0x42830000))
#define ADC3_ADCON1_MOD_BIT                       (*(__IO uint32_t*)(0x4283000C))
#define ADC3_ADCON1_ADDE_BIT                      (*(__IO uint32_t*)(0x42830018))
#define ADC3_ADCON1_ADIE_BIT                      (*(__IO uint32_t*)(0x4283001C))
#define ADC3_ADCON1_ADON_BIT                      (*(__IO uint32_t*)(0x4283003C))
#define ADC3_ADCMPCON_ADLDE_BIT                   (*(__IO uint32_t*)(0x42830590))
#define ADC3_ADCMPCON_ADLIE_BIT                   (*(__IO uint32_t*)(0x42830594))
#define ADC3_ADCMPCON_ADGDE_BIT                   (*(__IO uint32_t*)(0x42830598))
#define ADC3_ADCMPCON_ADGIE_BIT                   (*(__IO uint32_t*)(0x4283059C))
#define ADC3_ADINTF_ADLIF_BIT                     (*(__I  uint32_t*)(0x42830800))
#define ADC3_ADINTF_ADGIF_BIT                     (*(__I  uint32_t*)(0x42830804))
#define ADC3_ADINTF_ADIF_BIT                      (*(__I  uint32_t*)(0x42830808))
#define ADC3_ADINTF_ADLIFC_BIT                    (*(__O  uint32_t*)(0x42830840))
#define ADC3_ADINTF_ADGIFC_BIT                    (*(__O  uint32_t*)(0x42830844))
#define ADC3_ADINTF_ADIFC_BIT                     (*(__O  uint32_t*)(0x42830848))
#define QEI_QEICON_QTSR_BIT                       (*(__IO uint32_t*)(0x42038008))
#define QEI_QEICON_QEIEN_BIT                      (*(__IO uint32_t*)(0x4203800C))
#define QEI_QEICON_QIDXEN_BIT                     (*(__IO uint32_t*)(0x42038010))
#define QEI_QEICON_QSWAP_BIT                      (*(__IO uint32_t*)(0x42038014))
#define QEI_QEICON_QIDXS_BIT                      (*(__I  uint32_t*)(0x42038018))
#define QEI_QEICON_QPDIR_BIT                      (*(__I  uint32_t*)(0x4203801C))
#define QEI_QFLTCON_QABFEN_BIT                    (*(__IO uint32_t*)(0x4203808C))
#define QEI_QFLTCON_QIDXFEN_BIT                   (*(__IO uint32_t*)(0x4203809C))
#define QEI_QEIINT_QPEIE_BIT                      (*(__IO uint32_t*)(0x42038480))
#define QEI_QEIINT_QCEIE_BIT                      (*(__IO uint32_t*)(0x42038484))
#define QEI_QEIINT_QEIIE_BIT                      (*(__IO uint32_t*)(0x42038488))
#define QEI_QEIINT_QTCAPIE_BIT                    (*(__IO uint32_t*)(0x4203848C))
#define QEI_QEIINT_QTIE_BIT                       (*(__IO uint32_t*)(0x42038490))
#define QEI_QEIINT_QPEDE_BIT                      (*(__IO uint32_t*)(0x42038494))
#define QEI_QEIINT_QCEDE_BIT                      (*(__IO uint32_t*)(0x42038498))
#define QEI_QEIINT_QEIDE_BIT                      (*(__IO uint32_t*)(0x4203849C))
#define QEI_QEIINT_QTCAPDE_BIT                    (*(__IO uint32_t*)(0x420384A0))
#define QEI_QEIINT_QTDE_BIT                       (*(__IO uint32_t*)(0x420384A4))
#define QEI_QTINTF_QPEIF_BIT                      (*(__I  uint32_t*)(0x42038500))
#define QEI_QTINTF_QCEIF_BIT                      (*(__I  uint32_t*)(0x42038504))
#define QEI_QTINTF_QEIIF_BIT                      (*(__I  uint32_t*)(0x42038508))
#define QEI_QTINTF_QTCAPIF_BIT                    (*(__I  uint32_t*)(0x4203850C))
#define QEI_QTINTF_QTIF_BIT                       (*(__I  uint32_t*)(0x42038510))
#define QEI_QTINTF_QPEIFC_BIT                     (*(__O  uint32_t*)(0x42038540))
#define QEI_QTINTF_QCEIFC_BIT                     (*(__O  uint32_t*)(0x42038544))
#define QEI_QTINTF_QEIIFC_BIT                     (*(__O  uint32_t*)(0x42038548))
#define QEI_QTINTF_QTCAPIFC_BIT                   (*(__O  uint32_t*)(0x4203854C))
#define QEI_QTINTF_QTIFC_BIT                      (*(__O  uint32_t*)(0x42038550))
#define AMOC_CMP1CON_C1NCHS_BIT                   (*(__IO uint32_t*)(0x420B0018))
#define AMOC_CMP1CON_CMP1EN_BIT                   (*(__IO uint32_t*)(0x420B001C))
#define AMOC_CMP1CON_C1OUT_BIT                    (*(__I  uint32_t*)(0x420B0028))
#define AMOC_CMP1CON_C1OUTEN_BIT                  (*(__IO uint32_t*)(0x420B002C))
#define AMOC_CMP1CON_C1DE_BIT                     (*(__IO uint32_t*)(0x420B003C))
#define AMOC_CMP1CON_CMP1VCMP_BIT                 (*(__IO uint32_t*)(0x420B004C))
#define AMOC_CMP2CON_C2NCHS_BIT                   (*(__IO uint32_t*)(0x420B0098))
#define AMOC_CMP2CON_CMP2EN_BIT                   (*(__IO uint32_t*)(0x420B009C))
#define AMOC_CMP2CON_C2OUT_BIT                    (*(__I  uint32_t*)(0x420B00A8))
#define AMOC_CMP2CON_C2OUTEN_BIT                  (*(__IO uint32_t*)(0x420B00AC))
#define AMOC_CMP2CON_C2DE_BIT                     (*(__IO uint32_t*)(0x420B00BC))
#define AMOC_CMP2CON_CMP2VCMP_BIT                 (*(__IO uint32_t*)(0x420B00CC))
#define AMOC_CMP3CON_C3NCHS_BIT                   (*(__IO uint32_t*)(0x420B0118))
#define AMOC_CMP3CON_CMP3EN_BIT                   (*(__IO uint32_t*)(0x420B011C))
#define AMOC_CMP3CON_C3OUT_BIT                    (*(__I  uint32_t*)(0x420B0128))
#define AMOC_CMP3CON_C3OUTEN_BIT                  (*(__IO uint32_t*)(0x420B012C))
#define AMOC_CMP3CON_C3DE_BIT                     (*(__IO uint32_t*)(0x420B013C))
#define AMOC_CMP3CON_CMP3VCMP_BIT                 (*(__IO uint32_t*)(0x420B014C))
#define AMOC_CMPINTF_C1IF_BIT                     (*(__I  uint32_t*)(0x420B0180))
#define AMOC_CMPINTF_C2IF_BIT                     (*(__I  uint32_t*)(0x420B0184))
#define AMOC_CMPINTF_C3IF_BIT                     (*(__I  uint32_t*)(0x420B0188))
#define AMOC_CMPINTF_C1IFC_BIT                    (*(__O  uint32_t*)(0x420B01C0))
#define AMOC_CMPINTF_C2IFC_BIT                    (*(__O  uint32_t*)(0x420B01C4))
#define AMOC_CMPINTF_C3IFC_BIT                    (*(__O  uint32_t*)(0x420B01C8))
#define AMOC_OPCON_OP1EN_BIT                      (*(__IO uint32_t*)(0x420B0200))
#define AMOC_OPCON_OP2EN_BIT                      (*(__IO uint32_t*)(0x420B0204))
#define AMOC_OPCON_OP3EN_BIT                      (*(__IO uint32_t*)(0x420B0208))
#define AMOC_AVREFCON_VREFEN_BIT                  (*(__IO uint32_t*)(0x420B0280))
#define AMOC_AVREFCON_VREFSEL_BIT                 (*(__IO uint32_t*)(0x420B0284))
#define AMOC_TPSCON_TPSEN_BIT                     (*(__IO uint32_t*)(0x420B0300))
#define AMOC_TPSCON_TPSCHOP_BIT                   (*(__IO uint32_t*)(0x420B0304))
#define UART1_FR_RI_BIT                           (*(__I  uint32_t*)(0x42040000))
#define UART1_FR_TI_BIT                           (*(__I  uint32_t*)(0x42040004))
#define UART1_FR_TC_BIT                           (*(__I  uint32_t*)(0x42040008))
#define UART1_FR_TXCOL_BIT                        (*(__I  uint32_t*)(0x4204000C))
#define UART1_FR_RXOV_BIT                         (*(__I  uint32_t*)(0x42040010))
#define UART1_FR_FE_BIT                           (*(__I  uint32_t*)(0x42040014))
#define UART1_FR_PE_BIT                           (*(__I  uint32_t*)(0x42040018))
#define UART1_FR_LBD_BIT                          (*(__I  uint32_t*)(0x4204001C))
#define UART1_FR_RIC_BIT                          (*(__O  uint32_t*)(0x42040040))
#define UART1_FR_TCC_BIT                          (*(__O  uint32_t*)(0x42040048))
#define UART1_FR_TXCOLC_BIT                       (*(__O  uint32_t*)(0x4204004C))
#define UART1_FR_RXOVC_BIT                        (*(__O  uint32_t*)(0x42040050))
#define UART1_FR_FEC_BIT                          (*(__O  uint32_t*)(0x42040054))
#define UART1_FR_PEC_BIT                          (*(__O  uint32_t*)(0x42040058))
#define UART1_FR_LBDC_BIT                         (*(__O  uint32_t*)(0x4204005C))
#define UART1_CR_STOP_BIT                         (*(__IO uint32_t*)(0x42040300))
#define UART1_CR_SBRTEN_BIT                       (*(__IO uint32_t*)(0x42040304))
#define UART1_CR_SMOD_BIT                         (*(__IO uint32_t*)(0x42040308))
#define UART1_CR_RIE_BIT                          (*(__IO uint32_t*)(0x4204030C))
#define UART1_CR_TIE_BIT                          (*(__IO uint32_t*)(0x42040310))
#define UART1_CR_TCIE_BIT                         (*(__IO uint32_t*)(0x42040314))
#define UART1_CR_LBDIE_BIT                        (*(__IO uint32_t*)(0x42040318))
#define UART1_CR_LBDL_BIT                         (*(__IO uint32_t*)(0x4204031C))
#define UART1_CR_RB8_BIT                          (*(__I  uint32_t*)(0x42040320))
#define UART1_CR_TB8_BIT                          (*(__IO uint32_t*)(0x42040324))
#define UART1_CR_PS_BIT                           (*(__IO uint32_t*)(0x42040328))
#define UART1_CR_PCE_BIT                          (*(__IO uint32_t*)(0x4204032C))
#define UART1_CR_SM2_BIT                          (*(__IO uint32_t*)(0x42040330))
#define UART1_CR_SBK_BIT                          (*(__IO uint32_t*)(0x4204033C))
#define UART1_CR_LINEN_BIT                        (*(__IO uint32_t*)(0x42040340))
#define UART1_CR_REN_BIT                          (*(__IO uint32_t*)(0x42040344))
#define UART1_CR_TEN_BIT                          (*(__IO uint32_t*)(0x42040348))
#define UART1_CR_DMAR_BIT                         (*(__IO uint32_t*)(0x4204034C))
#define UART1_CR_DMAT_BIT                         (*(__IO uint32_t*)(0x42040350))
#define UART2_FR_RI_BIT                           (*(__I  uint32_t*)(0x42048000))
#define UART2_FR_TI_BIT                           (*(__I  uint32_t*)(0x42048004))
#define UART2_FR_TC_BIT                           (*(__I  uint32_t*)(0x42048008))
#define UART2_FR_TXCOL_BIT                        (*(__I  uint32_t*)(0x4204800C))
#define UART2_FR_RXOV_BIT                         (*(__I  uint32_t*)(0x42048010))
#define UART2_FR_FE_BIT                           (*(__I  uint32_t*)(0x42048014))
#define UART2_FR_PE_BIT                           (*(__I  uint32_t*)(0x42048018))
#define UART2_FR_LBD_BIT                          (*(__I  uint32_t*)(0x4204801C))
#define UART2_FR_RIC_BIT                          (*(__O  uint32_t*)(0x42048040))
#define UART2_FR_TCC_BIT                          (*(__O  uint32_t*)(0x42048048))
#define UART2_FR_TXCOLC_BIT                       (*(__O  uint32_t*)(0x4204804C))
#define UART2_FR_RXOVC_BIT                        (*(__O  uint32_t*)(0x42048050))
#define UART2_FR_FEC_BIT                          (*(__O  uint32_t*)(0x42048054))
#define UART2_FR_PEC_BIT                          (*(__O  uint32_t*)(0x42048058))
#define UART2_FR_LBDC_BIT                         (*(__O  uint32_t*)(0x4204805C))
#define UART2_CR_STOP_BIT                         (*(__IO uint32_t*)(0x42048300))
#define UART2_CR_SBRTEN_BIT                       (*(__IO uint32_t*)(0x42048304))
#define UART2_CR_SMOD_BIT                         (*(__IO uint32_t*)(0x42048308))
#define UART2_CR_RIE_BIT                          (*(__IO uint32_t*)(0x4204830C))
#define UART2_CR_TIE_BIT                          (*(__IO uint32_t*)(0x42048310))
#define UART2_CR_TCIE_BIT                         (*(__IO uint32_t*)(0x42048314))
#define UART2_CR_LBDIE_BIT                        (*(__IO uint32_t*)(0x42048318))
#define UART2_CR_LBDL_BIT                         (*(__IO uint32_t*)(0x4204831C))
#define UART2_CR_RB8_BIT                          (*(__I  uint32_t*)(0x42048320))
#define UART2_CR_TB8_BIT                          (*(__IO uint32_t*)(0x42048324))
#define UART2_CR_PS_BIT                           (*(__IO uint32_t*)(0x42048328))
#define UART2_CR_PCE_BIT                          (*(__IO uint32_t*)(0x4204832C))
#define UART2_CR_SM2_BIT                          (*(__IO uint32_t*)(0x42048330))
#define UART2_CR_SBK_BIT                          (*(__IO uint32_t*)(0x4204833C))
#define UART2_CR_LINEN_BIT                        (*(__IO uint32_t*)(0x42048340))
#define UART2_CR_REN_BIT                          (*(__IO uint32_t*)(0x42048344))
#define UART2_CR_TEN_BIT                          (*(__IO uint32_t*)(0x42048348))
#define UART2_CR_DMAR_BIT                         (*(__IO uint32_t*)(0x4204834C))
#define UART2_CR_DMAT_BIT                         (*(__IO uint32_t*)(0x42048350))
#define UART3_FR_RI_BIT                           (*(__I  uint32_t*)(0x42050000))
#define UART3_FR_TI_BIT                           (*(__I  uint32_t*)(0x42050004))
#define UART3_FR_TC_BIT                           (*(__I  uint32_t*)(0x42050008))
#define UART3_FR_TXCOL_BIT                        (*(__I  uint32_t*)(0x4205000C))
#define UART3_FR_RXOV_BIT                         (*(__I  uint32_t*)(0x42050010))
#define UART3_FR_FE_BIT                           (*(__I  uint32_t*)(0x42050014))
#define UART3_FR_PE_BIT                           (*(__I  uint32_t*)(0x42050018))
#define UART3_FR_LBD_BIT                          (*(__I  uint32_t*)(0x4205001C))
#define UART3_FR_RIC_BIT                          (*(__O  uint32_t*)(0x42050040))
#define UART3_FR_TCC_BIT                          (*(__O  uint32_t*)(0x42050048))
#define UART3_FR_TXCOLC_BIT                       (*(__O  uint32_t*)(0x4205004C))
#define UART3_FR_RXOVC_BIT                        (*(__O  uint32_t*)(0x42050050))
#define UART3_FR_FEC_BIT                          (*(__O  uint32_t*)(0x42050054))
#define UART3_FR_PEC_BIT                          (*(__O  uint32_t*)(0x42050058))
#define UART3_FR_LBDC_BIT                         (*(__O  uint32_t*)(0x4205005C))
#define UART3_CR_STOP_BIT                         (*(__IO uint32_t*)(0x42050300))
#define UART3_CR_SBRTEN_BIT                       (*(__IO uint32_t*)(0x42050304))
#define UART3_CR_SMOD_BIT                         (*(__IO uint32_t*)(0x42050308))
#define UART3_CR_RIE_BIT                          (*(__IO uint32_t*)(0x4205030C))
#define UART3_CR_TIE_BIT                          (*(__IO uint32_t*)(0x42050310))
#define UART3_CR_TCIE_BIT                         (*(__IO uint32_t*)(0x42050314))
#define UART3_CR_LBDIE_BIT                        (*(__IO uint32_t*)(0x42050318))
#define UART3_CR_LBDL_BIT                         (*(__IO uint32_t*)(0x4205031C))
#define UART3_CR_RB8_BIT                          (*(__I  uint32_t*)(0x42050320))
#define UART3_CR_TB8_BIT                          (*(__IO uint32_t*)(0x42050324))
#define UART3_CR_PS_BIT                           (*(__IO uint32_t*)(0x42050328))
#define UART3_CR_PCE_BIT                          (*(__IO uint32_t*)(0x4205032C))
#define UART3_CR_SM2_BIT                          (*(__IO uint32_t*)(0x42050330))
#define UART3_CR_SBK_BIT                          (*(__IO uint32_t*)(0x4205033C))
#define UART3_CR_LINEN_BIT                        (*(__IO uint32_t*)(0x42050340))
#define UART3_CR_REN_BIT                          (*(__IO uint32_t*)(0x42050344))
#define UART3_CR_TEN_BIT                          (*(__IO uint32_t*)(0x42050348))
#define UART3_CR_DMAR_BIT                         (*(__IO uint32_t*)(0x4205034C))
#define UART3_CR_DMAT_BIT                         (*(__IO uint32_t*)(0x42050350))
#define SPI1_FR_SPRI_BIT                          (*(__I  uint32_t*)(0x42058000))
#define SPI1_FR_SPTI_BIT                          (*(__I  uint32_t*)(0x42058004))
#define SPI1_FR_MODF_BIT                          (*(__I  uint32_t*)(0x4205800C))
#define SPI1_FR_RXOV_BIT                          (*(__I  uint32_t*)(0x42058010))
#define SPI1_FR_WCOL_BIT                          (*(__I  uint32_t*)(0x42058014))
#define SPI1_FR_SPRIC_BIT                         (*(__O  uint32_t*)(0x42058040))
#define SPI1_FR_SPTIC_BIT                         (*(__O  uint32_t*)(0x42058044))
#define SPI1_FR_MODFC_BIT                         (*(__O  uint32_t*)(0x4205804C))
#define SPI1_FR_RXOVC_BIT                         (*(__O  uint32_t*)(0x42058050))
#define SPI1_FR_WCOLC_BIT                         (*(__O  uint32_t*)(0x42058054))
#define SPI1_CR_SSDIS_BIT                         (*(__IO uint32_t*)(0x42058190))
#define SPI1_CR_CPOL_BIT                          (*(__IO uint32_t*)(0x42058194))
#define SPI1_CR_CPHA_BIT                          (*(__IO uint32_t*)(0x42058198))
#define SPI1_CR_MSTR_BIT                          (*(__IO uint32_t*)(0x4205819C))
#define SPI1_CR_DIR_BIT                           (*(__IO uint32_t*)(0x420581A0))
#define SPI1_CR_SPDATL_BIT                        (*(__IO uint32_t*)(0x420581A4))
#define SPI1_CR_SPRIE_BIT                         (*(__IO uint32_t*)(0x420581A8))
#define SPI1_CR_SPTIE_BIT                         (*(__IO uint32_t*)(0x420581AC))
#define SPI1_CR_SPDMAR_BIT                        (*(__IO uint32_t*)(0x420581B0))
#define SPI1_CR_SPDMAT_BIT                        (*(__IO uint32_t*)(0x420581B4))
#define SPI1_CR_SPIEN_BIT                         (*(__IO uint32_t*)(0x420581B8))
#define SPI1_CR_SPSFF_BIT                         (*(__IO uint32_t*)(0x420581BC))
#define SPI2_FR_SPRI_BIT                          (*(__I  uint32_t*)(0x42060000))
#define SPI2_FR_SPTI_BIT                          (*(__I  uint32_t*)(0x42060004))
#define SPI2_FR_MODF_BIT                          (*(__I  uint32_t*)(0x4206000C))
#define SPI2_FR_RXOV_BIT                          (*(__I  uint32_t*)(0x42060010))
#define SPI2_FR_WCOL_BIT                          (*(__I  uint32_t*)(0x42060014))
#define SPI2_FR_SPRIC_BIT                         (*(__O  uint32_t*)(0x42060040))
#define SPI2_FR_SPTIC_BIT                         (*(__O  uint32_t*)(0x42060044))
#define SPI2_FR_MODFC_BIT                         (*(__O  uint32_t*)(0x4206004C))
#define SPI2_FR_RXOVC_BIT                         (*(__O  uint32_t*)(0x42060050))
#define SPI2_FR_WCOLC_BIT                         (*(__O  uint32_t*)(0x42060054))
#define SPI2_CR_SSDIS_BIT                         (*(__IO uint32_t*)(0x42060190))
#define SPI2_CR_CPOL_BIT                          (*(__IO uint32_t*)(0x42060194))
#define SPI2_CR_CPHA_BIT                          (*(__IO uint32_t*)(0x42060198))
#define SPI2_CR_MSTR_BIT                          (*(__IO uint32_t*)(0x4206019C))
#define SPI2_CR_DIR_BIT                           (*(__IO uint32_t*)(0x420601A0))
#define SPI2_CR_SPDATL_BIT                        (*(__IO uint32_t*)(0x420601A4))
#define SPI2_CR_SPRIE_BIT                         (*(__IO uint32_t*)(0x420601A8))
#define SPI2_CR_SPTIE_BIT                         (*(__IO uint32_t*)(0x420601AC))
#define SPI2_CR_SPDMAR_BIT                        (*(__IO uint32_t*)(0x420601B0))
#define SPI2_CR_SPDMAT_BIT                        (*(__IO uint32_t*)(0x420601B4))
#define SPI2_CR_SPIEN_BIT                         (*(__IO uint32_t*)(0x420601B8))
#define SPI2_CR_SPSFF_BIT                         (*(__IO uint32_t*)(0x420601BC))
#define TWI1_FR_TWINT_BIT                         (*(__I  uint32_t*)(0x42068000))
#define TWI1_FR_TFREE_BIT                         (*(__I  uint32_t*)(0x42068008))
#define TWI1_FR_TOUT_BIT                          (*(__I  uint32_t*)(0x4206800C))
#define TWI1_FR_TWINTC_BIT                        (*(__O  uint32_t*)(0x42068040))
#define TWI1_FR_TFREEC_BIT                        (*(__O  uint32_t*)(0x42068048))
#define TWI1_FR_TOUTC_BIT                         (*(__O  uint32_t*)(0x4206804C))
#define TWI1_ADDR_GC_BIT                          (*(__IO uint32_t*)(0x42068280))
#define TWI1_CR_AA_BIT                            (*(__IO uint32_t*)(0x42068300))
#define TWI1_CR_STO_BIT                           (*(__IO uint32_t*)(0x42068304))
#define TWI1_CR_STA_BIT                           (*(__IO uint32_t*)(0x42068308))
#define TWI1_CR_EFREE_BIT                         (*(__IO uint32_t*)(0x42068320))
#define TWI1_CR_ETOT_BIT                          (*(__IO uint32_t*)(0x42068324))
#define TWI1_CR_TWINTE_BIT                        (*(__IO uint32_t*)(0x42068328))
#define TWI1_CR_TWIEN_BIT                         (*(__IO uint32_t*)(0x4206833C))

#define BIT(module,reg,bitnum) (module##_##reg##_##bitnum)
#define BITBAND(addr,bitnum) *((uint32_t*)(((addr)&0xF0000000)+0x02000000 + ((((addr)&0xFFFFF)<<5)+((bitnum)<<2))))

#define GPIO_CFG_OFFSET    (GPIOA_BASE - GPIOA_CFG_BASE)
#define GPIOx(CFG)         ((GPIO_CFG_TypeDef*)  ((uint32_t)GPIOx - GPIO_CFG_OFFSET))  


__STATIC_INLINE void NVIC_SetVectorBase(uint32_t vector_base){
	SCB->VTOR = vector_base;
}

__STATIC_INLINE void RCC_WriteEnable(){
	RCC->RCCLOCK = 0x33CC;
}
__STATIC_INLINE void RCC_WriteDisable(){
	RCC->RCCLOCK = 0;
}


#ifdef __cplusplus
}  /*extern "C"*/
#endif

#endif /*__SH32F2XX_SA0_H__*/
/******************* (C) COPYRIGHT 2016 Sinowealth *****END OF FILE****/

