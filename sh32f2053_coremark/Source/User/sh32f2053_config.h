/*
********************************************************************************
* @file    sh32f2053_config.h  
* @author  
* @version V1.0.0
* @date    15-April-2016
* @brief   Library configuration file.
********************************************************************************
*/
#ifndef __SH32F2053_CONFIG_H__
#define __SH32F2053_CONFIG_H__

/*****************************************************************************
//--------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*****************************************************************************/

/**********************
//<o> Default Assert Function
//  <0=>DISABLE    
//  <1=>ENABLE
**********************/     
#define ASSERT_ENABLE 0

#if ASSERT_ENABLE==1
#define DEFAULT_ASSERT_ENABLE
#endif    

/**********************
//Debug Printf Options
//====== Debug Printf ========    
//<o>Debug Printf Device
//   <0=>No Printf support
//   <1=>ITM
//   <2=>UART

**********************/     
#define USE_MODULE_DBG_PRINTF 2
#if USE_MODULE_DBG_PRINTF
#define _MODULE_DBG_PRINTF
#endif

/**********************
// <h> UART Options
**********************/     
#if USE_MODULE_DBG_PRINTF == 2
#define _DBG_DEVICE_UART
/**********************
//       <o>UART SELECT
//             <1=>UART 1 PA[11,12]
//             <2=>UART 2 PA[8,9]
//             <3=>UART 3 PA[0,1]
**********************/     
#define DBG_UART_NO 1
/**********************
//       <o>Baudrate
//             <115200=> 115200
//             <57600=>  57600
//             <38400=>  38400
//             <19200=>  19200
//             <9600=>   9600
**********************/     
#define DBG_UART_BAUDRATE 115200

#if DBG_UART_NO==1
#define DBG_UART       UART1
#define DBG_UART_CLKGATE (RCC_APB1ENR_UART1EN_Msk)
#define DBG_UART_PORT   GPIOA
#define DBG_UART_RX     GPIO_Pin_11
#define DBG_UART_TX     GPIO_Pin_12
#define DBG_UART_IOAF   GPIO_AF_UART1
#endif
#if DBG_UART_NO==2
#define DBG_UART       UART2
#define DBG_UART_CLKGATE (RCC_APB1ENR_UART2EN_Msk)
#define DBG_UART_PORT   GPIOA
#define DBG_UART_RX     GPIO_Pin_8
#define DBG_UART_TX     GPIO_Pin_9
#define DBG_UART_IOAF   GPIO_AF_UART2
#endif
#if DBG_UART_NO==3
#define DBG_UART       UART3
#define DBG_UART_CLKGATE (RCC_APB1ENR_UART3EN_Msk)
#define DBG_UART_PORT   GPIOA
#define DBG_UART_RX     GPIO_Pin_0
#define DBG_UART_TX     GPIO_Pin_1
#define DBG_UART_IOAF   GPIO_AF_UART3
#endif

#endif

/**********************
//   </h>     
//   <h>ITM Options
**********************/     
#if USE_MODULE_DBG_PRINTF == 1
#define _DBG_DEVICE_ITM
/**********************
//       <o> ITM Frequency
//       <500000=>500KHz
//       <1000000=>1MHz
**********************/     
#define ITM_FREQ_DEFAULT 500000
/**********************
//    </h>
**********************/     
#endif
#if USE_MODULE_DBG_PRINTF > 2
#error "Only one DBG printf device can selected!"
#endif
/**********************
// debug printf
**********************/     


/**********************
//<e> Use SH32F Standard Peripheral Driver
**********************/     
#define USE_STD_LIBRARY 1
/**********************
//    <q>FLASH
**********************/     
#define USE_MODULE_FLASH 0

/**********************
//    <q>System Config
**********************/     
#define USE_MODULE_SYSCFG 0

/**********************
//    <q>RCC
**********************/     
#define USE_MODULE_RCC 1

/**********************
//    <q>IWDT
**********************/     
#define USE_MODULE_IWDT 1

/**********************
//    <q>WWDT
**********************/     
#define USE_MODULE_WWDT 0

/**********************
//    <q>CRC
**********************/     
#define USE_MODULE_CRC 0

/**********************
//    <q>RAM Bist
**********************/     
#define USE_MODULE_RAMBIST 0

/**********************
//    <q>GPIO    
**********************/     
#define USE_MODULE_GPIO 1

/**********************
//    <q>EXTI    
**********************/     
#define USE_MODULE_EXTI 0

/**********************
//    <q>DMA    
**********************/     
#define USE_MODULE_DMA 0

/**********************
//    <q>MCM    
**********************/     
#define USE_MODULE_MCM 0

/**********************
//    <q>GPT
**********************/     
#define USE_MODULE_GPT 0

/**********************
//    <q>TIMER
**********************/     
#define USE_MODULE_TIMER 0

/**********************
//    <q>MACP
**********************/     
#define USE_MODULE_MACP 0

/**********************
//    <q>ADC
**********************/     
#define USE_MODULE_ADC 0

/**********************
//    <q>QEI
**********************/     
#define USE_MODULE_QEI 0

/**********************
//    <q>AMOC
**********************/     
#define USE_MODULE_AMOC 0

/**********************
//    <q>UART
**********************/     
#define USE_MODULE_UART 1

/**********************
//    <q>SPI
**********************/     
#define USE_MODULE_SPI 0

/**********************
//    <q>TWI
**********************/     
#define USE_MODULE_TWI 0

/**********************
//    <q>MPU
**********************/     
#define USE_MODULE_MPU 0

/**********************
//    <q>NVIC
**********************/     
#define USE_MODULE_NVIC 0

/**********************
//    <q>Cortex-M3
**********************/     
#define USE_MODULE_CORTEX 0



/**********************
//</e>     
**********************/  
#if USE_MODULE_FLASH
#define _MODULE_FLASH
#endif

#if USE_MODULE_SYSCFG
#define _MODULE_SYSCFG
#endif

#if USE_MODULE_RCC
#define _MODULE_RCC
#endif

#if USE_MODULE_IWDT
#define _MODULE_IWDT
#endif

#if USE_MODULE_WWDT
#define _MODULE_WWDT
#endif

#if USE_MODULE_CRC
#define _MODULE_CRC
#endif

#if USE_MODULE_RAMBIST
#define _MODULE_RAMBIST
#endif

#if USE_MODULE_GPIO
#define _MODULE_GPIO
#endif

#if USE_MODULE_EXTI
#define _MODULE_EXTI
#endif

#if USE_MODULE_DMA
#define _MODULE_DMA
#endif

#if USE_MODULE_MCM
#define _MODULE_MCM
#endif

#if USE_MODULE_GPT
#define _MODULE_GPT
#endif

#if USE_MODULE_TIMER
#define _MODULE_TIMER
#endif

#if USE_MODULE_MACP
#define _MODULE_MACP
#endif

#if USE_MODULE_ADC
#define _MODULE_ADC
#endif

#if USE_MODULE_QEI
#define _MODULE_QEI
#endif

#if USE_MODULE_AMOC
#define _MODULE_AMOC
#endif

#if USE_MODULE_UART
#define _MODULE_UART
#endif

#if USE_MODULE_SPI
#define _MODULE_SPI
#endif

#if USE_MODULE_TWI
#define _MODULE_TWI
#endif

#if USE_MODULE_MPU
#define _MODULE_MPU
#endif

#if USE_MODULE_NVIC
#define _MODULE_NVIC
#endif

#if USE_MODULE_CORTEX
#define _MODULE_CORTEX
#endif


/***********************
//<e>Use Default SystemInit
************************/
#define USE_DEFAULT_SYSTEMINIT 1
#if USE_DEFAULT_SYSTEMINIT
#define DEFAULT_SYSTEMINIT
#endif

/***********************
//<o> Clock Source  
//    <0=>HSI (8MHz)
//    <1=>HSE       
//    <2=>PLL       
**********************/     
#define CLK_SRC 2
/*********************
//<o> HSI CLK 
**********************/
#define HSI_CLK 8000000
/*********************
//<o> HSE CLK 
**********************/
#define HSE_CLK 8000000


/*********************
//<e> Use PLL
**********************/

#define PLL_EN  1
/***********************
//<o> PLL Clock Source  
//    <0=>HSI (8MHz)
//    <1=>HSE       
**********************/
#define PLL_CLK_SRC 0

/*********************
//<q> CLK/2 TO PLL
**********************/
#define PLL_XTPRE 0
#define PLL_IN PLL_CLK_SRC
/*********************
//<o> PLL MUL
**********************/
#define PLL_MUL 30
/*********************
//<o> PLL DIV
**********************/
#define PLL_DIV 1
/*********************
//</e>
**********************/
#if PLL_MUL < 15
#error "PLL MUL Need 15~46"
#endif
#if PLL_MUL > 46
#error "PLL MUL Need 15~46"
#endif


#if PLL_DIV < 1
#error "PLL DIV Need 1~8"
#endif

/*********************
//<o>AHB CLOCK (HCLK)
//    <0=>FCLK/1
//    <1=>FCLK/2
//    <2=>FCLK/4
//    <3=>FCLK/8
//    <4=>FCLK/16
//    <5=>FCLK/32
**********************/
#define CLK_AHB 0

/*********************
//<o>APB1 CLOCK
//    <0=>HCLK/1
//    <4=>HCLK/2
//    <5=>HCLK/4
//    <6=>HCLK/8
//    <7=>HCLK/16
**********************/
#define CLK_APB1 5

/*********************
//<o>APB2 CLOCK
//    <0=>HCLK/1
//    <4=>HCLK/2
//    <5=>HCLK/4
//    <6=>HCLK/8
//    <7=>HCLK/16
**********************/
#define CLK_APB2 4

/*********************
//<o>FLASH Latency
//    <0=>0
//    <1=>1
//    <2=>2
//    <3=>3
//    <4=>4
//    <5=>5
//    <6=>6
//    <7=>7
**********************/
#define FLASH_LATENCY 3
/*********************
//<q> FLASH Prefetch
**********************/
#define PREFETCH_EN 1
/*********************
//<q> FLASH Instruction Cache
**********************/
#define ICACHE_EN 1
/*********************
//<q> FLASH Data Cache
**********************/
#define DCACHE_EN 1

/***********************
//</e>
************************/


#if  (CLK_SRC == 0)
#define CLK_HZ   HSI_CLK
#elif (CLK_SRC == 1)
#define CLK_HZ   HSE_CLK
#else
#if (PLL_EN == 0)
#error ("System Clock Source is PLL, so PLL must enable")
#endif
#if (PLL_CLK_SRC == 0)
#define CLK_HZ   HSI_CLK
#else
#define CLK_HZ   HSE_CLK
#endif
#endif


#if PLL_EN
#define SYS_CLK  (CLK_HZ/(PLL_XTPRE+1) * PLL_MUL / PLL_DIV / 2)/(1<<CLK_AHB)
#else
#define SYS_CLK  CLK_HZ/(1<<CLK_AHB)
#endif



#if SYS_CLK > 35000000  && FLASH_LATENCY < 1
#error ("Flash Latency need >= 1")
#endif
#if SYS_CLK > 70000000  && FLASH_LATENCY < 2
#error ("Flash Latency need >= 2")
#endif
#if SYS_CLK > 110000000  && FLASH_LATENCY < 3
#error ("Flash Latency need >= 3")
#endif
#if SYS_CLK > 150000000  && FLASH_LATENCY < 4
#error ("Flash Latency need >= 4")
#endif
#if SYS_CLK > 180000000  && FLASH_LATENCY < 5
#error ("Flash Latency need >= 5")
#endif



#endif //__SH32F2053_CONFIG_H__
