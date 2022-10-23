;/******************** (C) COPYRIGHT 2017 Sinowealth ********************
;* File Name          : startup_sh32f9803_iar.s
;* Author             : MCD Application Team
;* Version            : V1.0.0
;* Date               : 07-March-2017
;* Description        : SH32F205xx devices vector table for EWARM toolchain.
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == _iar_program_start,
;*                      - Set the vector table entries with the exceptions ISR 
;*                        address.
;*                      - Branches to main in the C library (which eventually
;*                        calls main()).
;*                      After Reset the Cortex-M3 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;********************************************************************************
;* 
;* Redistribution and use in source and binary forms, with or without modification,
;* are permitted provided that the following conditions are met:
;*   1. Redistributions of source code must retain the above copyright notice,
;*      this list of conditions and the following disclaimer.
;*   2. Redistributions in binary form must reproduce the above copyright notice,
;*      this list of conditions and the following disclaimer in the documentation
;*      and/or other materials provided with the distribution.
;*   3. Neither the name of STMicroelectronics nor the names of its contributors
;*      may be used to endorse or promote products derived from this software
;*      without specific prior written permission.
;*
;* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
;* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
;* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;* 
;*******************************************************************************
; *
; * SINOWEALTH IS SUPPLYING THIS SOFTWARE FOR USE EXCLUSIVELY ON SINOWEALTH'S 
; * MICROCONTROLLER PRODUCTS. IT IS PROTECTED UNDER APPLICABLE COPYRIGHT LAWS. 
; * THIS SOFTWARE IS FOR GUIDANCE IN ORDER TO SAVE TIME. AS A RESULT, SINOWEALTH 
; * SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES 
; * WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH FIRMWARE AND/OR
; * THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN 
; * CONNECTION WITH THEIR PRODUCTS.
; *
; * <h2><center>&copy; COPYRIGHT 2017 Sinowealth</center></h2>
; *
; ******************************************************************************
; */
;
;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;



        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT (3)

        SECTION .intvec:CODE:NOROOT (2)

        EXTERN  __iar_program_start
        PUBLIC  __vector_table

        DATA
__vector_table

     DCD      sfe(CSTACK)          ;/* -16 (000H) Top of Stack                                  */
     DCD      Reset_Handler        ;/* -15 (004H) Reset Handler                                 */
     DCD      NMI_Handler          ;/* -14 (008H) Non Maskable Interrupt                        */
     DCD      HardFault_Handler    ;/* -13 (00CH) Hard Fault Interrupt                          */
     DCD      MemManage_Handler    ;/* -12 (010H) Cortex-M3 Memory Management Interrupt         */
     DCD      BusFault_Handler     ;/* -11 (014H) Cortex-M3 Bus Fault Interrupt                 */
     DCD      UsageFault_Handler   ;/* -10 (018H) Cortex-M3 Usage Fault Interrupt               */
     DCD      Default_Handler      ;/*  -9 (01CH) Reserved                                      */
     DCD      Default_Handler      ;/*  -8 (020H) Reserved                                      */
     DCD      Default_Handler      ;/*  -7 (024H) Reserved                                      */
     DCD      Default_Handler      ;/*  -6 (028H) Reserved                                      */
     DCD      SVC_Handler          ;/*  -5 (02CH) Cortex-M3 SV Call Interrupt                   */
     DCD      DebugMon_Handler     ;/*  -4 (030H) Cortex-M3 Debug Monitor Interrupt             */
     DCD      Default_Handler      ;/*  -3 (034H) Reserved                                      */
     DCD      PendSV_Handler       ;/*  -2 (038H) Cortex-M3 Pend SV Interrupt                   */
     DCD      SysTick_Handler      ;/*  -1 (03CH) Cortex-M3 System Tick Interrupt               */
     DCD      WWDT_Handler         ;/*   0 (040H) Window WatchDog Interrupt                     */
     DCD      BOD_Handler          ;/*   1 (044H) PVD through EXTI Line detection Interrupt     */
     DCD      Default_Handler      ;/*   2 (048H) Reserved                                      */
     DCD      Default_Handler      ;/*   3 (04CH) Reserved                                      */
     DCD      RCC_Handler          ;/*   4 (050H) RCC global Interrupt                          */
     DCD      EXTI0_Handler        ;/*   5 (054H) EXTI Line0 Interrupt                          */
     DCD      EXTI1_Handler        ;/*   6 (058H) EXTI Line1 Interrupt                          */
     DCD      EXTI2_Handler        ;/*   7 (05CH) EXTI Line2 Interrupt                          */
     DCD      EXTI3_Handler        ;/*   8 (060H) EXTI Line3 Interrupt                          */
     DCD      EXTI4_Handler        ;/*   9 (064H) EXTI Line4 Interrupt                          */
     DCD      DMA_CH0_Handler      ;/*  10 (068H) DMA Channel0 Interrupt                        */
     DCD      DMA_CH1_Handler      ;/*  11 (06CH) DMA Channel1 Interrupt                        */
     DCD      DMA_CH2_7_Handler    ;/*  12 (070H) DMA Channel2 ~ Channel7 Interrupt             */
     DCD      MCM1_FLT_Handler     ;/*  13 (074H) MCM1 FAULT Interrupt                          */
     DCD      MCM1_PWM_Handler     ;/*  14 (078H) MCM1 PWM Interrupt                            */
     DCD      Default_Handler      ;/*  15 (07CH) Reserved                                      */
     DCD      Default_Handler      ;/*  16 (080H) Reserved                                      */
     DCD      ADC1_Handler         ;/*  17 (084H) ADC1 Interrupt                                */
     DCD      Default_Handler      ;/*  18 (088H) Reserved                                      */
     DCD      ADC3_Handler         ;/*  19 (08CH) ADC3 Interrupt                                */
     DCD      Default_Handler      ;/*  20 (090H) Reserved                                      */
     DCD      Default_Handler      ;/*  21 (094H) Reserved                                      */
     DCD      GPT0_GTCIV_Handler   ;/*  22 (098H) GPT0 GTCIV Interrupt                          */
     DCD      GPT0_GTCIN_Handler   ;/*  23 (09CH) GPT0 GTCIN Interrupt                          */
     DCD      GPT1_GTCIV_Handler   ;/*  24 (0A0H) GPT1 GTCIV Interrupt                          */
     DCD      GPT1_GTCIN_Handler   ;/*  25 (0A4H) GPT1 GTCIN Interrupt                          */
     DCD      GPT2_GTCIV_Handler   ;/*  26 (0A8H) GPT2 GTCIV Interrupt                          */
     DCD      GPT2_GTCIN_Handler   ;/*  27 (0ACH) GPT2 GTCIN Interrupt                          */
     DCD      Default_Handler      ;/*  28 (0B0H) Reserved                                      */
     DCD      Default_Handler      ;/*  29 (0B4H) Reserved                                      */
     DCD      GPT_POE_Handler      ;/*  30 (0B8H) GPT POE Global Interrupt                      */
     DCD      EXTI9_5_Handler      ;/*  31 (0BCH) EXTI Line5 ~ Line9 global Interrupt           */
     DCD      CMP1_Handler         ;/*  32 (0C0H) CMP1 Interrupt                                */
     DCD      CMP2_Handler         ;/*  33 (0C4H) CMP2 Interrupt                                */
     DCD      CMP3_Handler         ;/*  34 (0C8H) CMP3 Interrupt                                */
     DCD      QEI_Handler          ;/*  35 (0CCH) QEI Interrupt                                 */
     DCD      TWI_Handler          ;/*  36 (0D0H) TWI Interrupt                                 */
     DCD      Default_Handler      ;/*  37 (0D4H) Reserved Interrupt                            */
     DCD      SPI1_Handler         ;/*  38 (0D8H) SPI1 Interrupt                                */
     DCD      Default_Handler      ;/*  39 (0DCH) Reserved                                      */
     DCD      UART1_Handler        ;/*  40 (0E0H) UART1 Interrupt                               */
     DCD      UART2_Handler        ;/*  41 (0E4H) UART2 Interrupt                               */
     DCD      UART3_Handler        ;/*  42 (0E8H) UART3 Interrupt                               */
     DCD      Default_Handler      ;/*  43 (0ECH) Reserved Interrupt                            */
     DCD      EXTI15_10_Handler    ;/*  44 (0F0H) EXTI Line10 ~ Line15 Interrupt                */
     DCD      Default_Handler      ;/*  45 (0F4H) Reserved Interrupt                            */
     DCD      Default_Handler      ;/*  46 (0F8H) Reserved Interrupt                            */
     DCD      Default_Handler      ;/*  47 (0FCH) Reserved Interrupt                            */
     DCD      Default_Handler      ;/*  48 (100H) Reserved Interrupt                            */
     DCD      TIM5_Handler         ;/*  49 (104H) TIM5 Interrupt                                */
     DCD      TIM6_Handler         ;/*  50 (108H) TIM6 Interrupt                                */
     DCD      TIM7_Handler         ;/*  51 (10CH) TIM7 Interrupt                                */
     DCD      TIM8_Handler         ;/*  52 (110H) TIM8 Interrupt                                */


	THUMB
	PUBWEAK Reset_Handler
	SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
	
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0
	
	PUBWEAK SystemInit
	SECTION .text:CODE:REORDER:NOROOT(1)
SystemInit
	BX LR
	
	PUBWEAK NMI_Handler
	SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
	B NMI_Handler	

	PUBWEAK HardFault_Handler
	SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
	B HardFault_Handler
	
	PUBWEAK MemManage_Handler
	SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
	B MemManage_Handler

	PUBWEAK BusFault_Handler
	SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
	B BusFault_Handler

	PUBWEAK UsageFault_Handler
	SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
	B UsageFault_Handler

	PUBWEAK SVC_Handler
	SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
	B SVC_Handler

	PUBWEAK DebugMon_Handler
	SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
	B DebugMon_Handler

	PUBWEAK PendSV_Handler
	SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
	B PendSV_Handler

	PUBWEAK SysTick_Handler
	SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
	B SysTick_Handler
	
					
    PUBWEAK    WWDT_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
WWDT_Handler        
    B WWDT_Handler        
    PUBWEAK    BOD_Handler         
    SECTION .text:CODE:REORDER:NOROOT(1)
BOD_Handler         
    B BOD_Handler         
    PUBWEAK    RCC_Handler         
    SECTION .text:CODE:REORDER:NOROOT(1)
RCC_Handler         
    B RCC_Handler         
    PUBWEAK    EXTI0_Handler       
    SECTION .text:CODE:REORDER:NOROOT(1)
EXTI0_Handler       
    B EXTI0_Handler       
    PUBWEAK    EXTI1_Handler       
    SECTION .text:CODE:REORDER:NOROOT(1)
EXTI1_Handler       
    B EXTI1_Handler       
    PUBWEAK    EXTI2_Handler       
    SECTION .text:CODE:REORDER:NOROOT(1)
EXTI2_Handler       
    B EXTI2_Handler       
    PUBWEAK    EXTI3_Handler       
    SECTION .text:CODE:REORDER:NOROOT(1)
EXTI3_Handler       
    B EXTI3_Handler       
    PUBWEAK    EXTI4_Handler       
    SECTION .text:CODE:REORDER:NOROOT(1)
EXTI4_Handler       
    B EXTI4_Handler       
    PUBWEAK    DMA_CH0_Handler     
    SECTION .text:CODE:REORDER:NOROOT(1)
DMA_CH0_Handler     
    B DMA_CH0_Handler     
    PUBWEAK    DMA_CH1_Handler     
    SECTION .text:CODE:REORDER:NOROOT(1)
DMA_CH1_Handler     
    B DMA_CH1_Handler     
    PUBWEAK    DMA_CH2_7_Handler   
    SECTION .text:CODE:REORDER:NOROOT(1)
DMA_CH2_7_Handler   
    B DMA_CH2_7_Handler   
    PUBWEAK    MCM1_FLT_Handler    
    SECTION .text:CODE:REORDER:NOROOT(1)
MCM1_FLT_Handler    
    B MCM1_FLT_Handler    
    PUBWEAK    MCM1_PWM_Handler    
    SECTION .text:CODE:REORDER:NOROOT(1)
MCM1_PWM_Handler    
    B MCM1_PWM_Handler    
    PUBWEAK    ADC1_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
ADC1_Handler        
    B ADC1_Handler        
    PUBWEAK    ADC3_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
ADC3_Handler        
    B ADC3_Handler        
    PUBWEAK    GPT0_GTCIV_Handler  
    SECTION .text:CODE:REORDER:NOROOT(1)
GPT0_GTCIV_Handler  
    B GPT0_GTCIV_Handler  
    PUBWEAK    GPT0_GTCIN_Handler  
    SECTION .text:CODE:REORDER:NOROOT(1)
GPT0_GTCIN_Handler  
    B GPT0_GTCIN_Handler  
    PUBWEAK    GPT1_GTCIV_Handler  
    SECTION .text:CODE:REORDER:NOROOT(1)
GPT1_GTCIV_Handler  
    B GPT1_GTCIV_Handler  
    PUBWEAK    GPT1_GTCIN_Handler  
    SECTION .text:CODE:REORDER:NOROOT(1)
GPT1_GTCIN_Handler  
    B GPT1_GTCIN_Handler  
    PUBWEAK    GPT2_GTCIV_Handler  
    SECTION .text:CODE:REORDER:NOROOT(1)
GPT2_GTCIV_Handler  
    B GPT2_GTCIV_Handler  
    PUBWEAK    GPT2_GTCIN_Handler  
    SECTION .text:CODE:REORDER:NOROOT(1)
GPT2_GTCIN_Handler  
    B GPT2_GTCIN_Handler  
    PUBWEAK    GPT_POE_Handler     
    SECTION .text:CODE:REORDER:NOROOT(1)
GPT_POE_Handler     
    B GPT_POE_Handler     
    PUBWEAK    EXTI9_5_Handler     
    SECTION .text:CODE:REORDER:NOROOT(1)
EXTI9_5_Handler     
    B EXTI9_5_Handler     
    PUBWEAK    CMP1_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
CMP1_Handler        
    B CMP1_Handler        
    PUBWEAK    CMP2_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
CMP2_Handler        
    B CMP2_Handler        
    PUBWEAK    CMP3_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
CMP3_Handler        
    B CMP3_Handler        
    PUBWEAK    QEI_Handler         
    SECTION .text:CODE:REORDER:NOROOT(1)
QEI_Handler         
    B QEI_Handler         
    PUBWEAK    TWI_Handler         
    SECTION .text:CODE:REORDER:NOROOT(1)
TWI_Handler         
    B TWI_Handler         
    PUBWEAK    SPI1_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
SPI1_Handler        
    B SPI1_Handler        
    PUBWEAK    UART1_Handler       
    SECTION .text:CODE:REORDER:NOROOT(1)
UART1_Handler       
    B UART1_Handler       
    PUBWEAK    UART2_Handler       
    SECTION .text:CODE:REORDER:NOROOT(1)
UART2_Handler       
    B UART2_Handler       
    PUBWEAK    UART3_Handler       
    SECTION .text:CODE:REORDER:NOROOT(1)
UART3_Handler       
    B UART3_Handler       
    PUBWEAK    EXTI15_10_Handler   
    SECTION .text:CODE:REORDER:NOROOT(1)
EXTI15_10_Handler   
    B EXTI15_10_Handler   
    PUBWEAK    TIM5_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
TIM5_Handler        
    B TIM5_Handler        
    PUBWEAK    TIM6_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
TIM6_Handler        
    B TIM6_Handler        
    PUBWEAK    TIM7_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
TIM7_Handler        
    B TIM7_Handler        
    PUBWEAK    TIM8_Handler        
    SECTION .text:CODE:REORDER:NOROOT(1)
TIM8_Handler        
    B TIM8_Handler        
	
	PUBWEAK Default_Handler
	SECTION .text:CODE:REORDER:NOROOT(1)
Default_Handler
	B Default_Handler
	
	END
/*********************** (C) COPYRIGHT STMicroelectronics *** END OF FILE*****/	
	
	
