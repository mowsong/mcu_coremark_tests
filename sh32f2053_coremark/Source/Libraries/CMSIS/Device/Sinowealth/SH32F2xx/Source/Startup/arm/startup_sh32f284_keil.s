;/*****************************************************************************/
;/* startup_sh32f284_keil.s: Startup file for Sinowealth SH32F2 device series */
;/*****************************************************************************/
;/* <<< Use Configuration Wizard in Context Menu >>>                          */ 
;/*****************************************************************************/
;/* This file is part of the uVision/ARM development tools.                   */
;/* Copyright (c) 2005-2007 Keil Software. All rights reserved.               */
;/* This software may only be used under the terms of a valid, current,       */
;/* end user licence from KEIL for a compatible version of KEIL software      */
;/* development tools. Nothing else gives you the right to use this software. */
;/*****************************************************************************/
; *
; * SINOWEALTH IS SUPPLYING THIS SOFTWARE FOR USE EXCLUSIVELY ON SINOWEALTH'S 
; * MICROCONTROLLER PRODUCTS. IT IS PROTECTED UNDER APPLICABLE COPYRIGHT LAWS. 
; * THIS SOFTWARE IS FOR GUIDANCE IN ORDER TO SAVE TIME. AS A RESULT, SINOWEALTH 
; * SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES 
; * WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH FIRMWARE AND/OR
; * THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN 
; * CONNECTION WITH THEIR PRODUCTS.
; *
; * COPYRIGHT 2017 Sinowealth
; *
; ******************************************************************************
; */

;<e>Run Application in User Mode
USER_MODE_EN      EQU  0
;</e>

;// <h> Stack Configuration
;   <o0>System Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;   <o1>User Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;// </h> 
SystemStack_Size  EQU     0x00000100
UserStack_Size    EQU     0x00000000

		  AREA    STACK, NOINIT, READWRITE, ALIGN=3
System_Stack_Mem  SPACE   SystemStack_Size
system_stack_top
User_Stack_Mem    SPACE   UserStack_Size
usr_stack_top
__initial_sp

;// <h> Heap Configuration
;//   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF>
;// </h>
Heap_Size        EQU     0x00000100

;/*****************************************************************************/
;/*-------- <<< end of configuration section >>> -----------------------------*/
;/*****************************************************************************/

                 AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem         SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB
; Vector Table Mapped to Address 0 at Reset
                         AREA     RESET, DATA, READONLY
                         EXPORT   __Vectors
                         EXPORT   __Vectors_End
                         EXPORT   __Vectors_Size
__Vectors
                         DCD      system_stack_top     ; -16 (000H) Top of Stack
                         DCD      Reset_Handler        ; -15 (004H) Reset Handler
                         DCD      NMI_Handler          ; -14 (008H) Non Maskable Interrupt
                         DCD      HardFault_Handler    ; -13 (00CH) Hard Fault Interrupt
                         DCD      MemManage_Handler    ; -12 (010H) Cortex-M3 Memory Management Interrupt
                         DCD      BusFault_Handler     ; -11 (014H) Cortex-M3 Bus Fault Interrupt
                         DCD      UsageFault_Handler   ; -10 (018H) Cortex-M3 Usage Fault Interrupt
                         DCD      Default_Handler      ;  -9 (01CH) Reserved
                         DCD      Default_Handler      ;  -8 (020H) Reserved
                         DCD      Default_Handler      ;  -7 (024H) Reserved
                         DCD      Default_Handler      ;  -6 (028H) Reserved
                         DCD      SVC_Handler          ;  -5 (02CH) Cortex-M3 SV Call Interrupt
                         DCD      DebugMon_Handler     ;  -4 (030H) Cortex-M3 Debug Monitor Interrupt
                         DCD      Default_Handler      ;  -3 (034H) Reserved
                         DCD      PendSV_Handler       ;  -2 (038H) Cortex-M3 Pend SV Interrupt
                         DCD      SysTick_Handler      ;  -1 (03CH) Cortex-M3 System Tick Interrupt
                         DCD      WWDT_Handler         ;   0 (040H) Window WatchDog Interrupt
                         DCD      BOD_Handler          ;   1 (044H) PVD through EXTI Line detection Interrupt
                         DCD      Default_Handler      ;   2 (048H) Reserved
                         DCD      Default_Handler      ;   3 (04CH) Reserved
                         DCD      RCC_Handler          ;   4 (050H) RCC global Interrupt
                         DCD      EXTI0_Handler        ;   5 (054H) EXTI Line0 Interrupt
                         DCD      EXTI1_Handler        ;   6 (058H) EXTI Line1 Interrupt
                         DCD      EXTI2_Handler        ;   7 (05CH) EXTI Line2 Interrupt
                         DCD      EXTI3_Handler        ;   8 (060H) EXTI Line3 Interrupt
                         DCD      EXTI4_Handler        ;   9 (064H) EXTI Line4 Interrupt
                         DCD      DMA_CH0_Handler      ;  10 (068H) DMA Channel0 Interrupt
                         DCD      DMA_CH1_Handler      ;  11 (06CH) DMA Channel1 Interrupt
                         DCD      DMA_CH2_7_Handler    ;  12 (070H) DMA Channel2 ~ Channel7 Interrupt
                         DCD      MCM1_FLT_Handler     ;  13 (074H) MCM1 FAULT Interrupt
                         DCD      MCM1_PWM_Handler     ;  14 (078H) MCM1 PWM Interrupt
                         DCD      Default_Handler      ;  15 (07CH) Reserved
                         DCD      Default_Handler      ;  16 (080H) Reserved
                         DCD      ADC1_Handler         ;  17 (084H) ADC1 Interrupt
                         DCD      Default_Handler      ;  18 (088H) Reserved
                         DCD      ADC3_Handler         ;  19 (08CH) ADC3 Interrupt
                         DCD      Default_Handler      ;  20 (090H) Reserved
                         DCD      Default_Handler      ;  21 (094H) Reserved
                         DCD      GPT0_GTCIV_Handler   ;  22 (098H) GPT0 GTCIV Interrupt
                         DCD      GPT0_GTCIN_Handler   ;  23 (09CH) GPT0 GTCIN Interrupt
                         DCD      GPT1_GTCIV_Handler   ;  24 (0A0H) GPT1 GTCIV Interrupt
                         DCD      GPT1_GTCIN_Handler   ;  25 (0A4H) GPT1 GTCIN Interrupt
                         DCD      GPT2_GTCIV_Handler   ;  26 (0A8H) GPT2 GTCIV Interrupt
                         DCD      GPT2_GTCIN_Handler   ;  27 (0ACH) GPT2 GTCIN Interrupt
                         DCD      Default_Handler      ;  28 (0B0H) Reserved
                         DCD      Default_Handler      ;  29 (0B4H) Reserved
                         DCD      GPT_POE_Handler      ;  30 (0B8H) GPT POE Global Interrupt
                         DCD      EXTI9_5_Handler      ;  31 (0BCH) EXTI Line5 ~ Line9 global Interrupt
                         DCD      CMP1_Handler         ;  32 (0C0H) CMP1 Interrupt
                         DCD      CMP2_Handler         ;  33 (0C4H) CMP2 Interrupt
                         DCD      CMP3_Handler         ;  34 (0C8H) CMP3 Interrupt
                         DCD      QEI_Handler          ;  35 (0CCH) QEI Interrupt
                         DCD      TWI_Handler          ;  36 (0D0H) TWI Interrupt
                         DCD      Default_Handler      ;  37 (0D4H) Reserved Interrupt
                         DCD      SPI1_Handler         ;  38 (0D8H) SPI1 Interrupt
                         DCD      Default_Handler      ;  39 (0DCH) Reserved
                         DCD      UART1_Handler        ;  40 (0E0H) UART1 Interrupt
                         DCD      UART2_Handler        ;  41 (0E4H) UART2 Interrupt
                         DCD      UART3_Handler        ;  42 (0E8H) UART3 Interrupt
                         DCD      Default_Handler      ;  43 (0ECH) Reserved Interrupt
                         DCD      EXTI15_10_Handler    ;  44 (0F0H) EXTI Line10 ~ Line15 Interrupt
                         DCD      Default_Handler      ;  45 (0F4H) Reserved Interrupt
                         DCD      Default_Handler      ;  46 (0F8H) Reserved Interrupt
                         DCD      Default_Handler      ;  47 (0FCH) Reserved Interrupt
                         DCD      Default_Handler      ;  48 (100H) Reserved Interrupt
                         DCD      TIM5_Handler         ;  49 (104H) TIM5 Interrupt
                         DCD      TIM6_Handler         ;  50 (108H) TIM6 Interrupt
                         DCD      TIM7_Handler         ;  51 (10CH) TIM7 Interrupt
                         DCD      TIM8_Handler         ;  52 (110H) TIM8 Interrupt


__Vectors_End
__Vectors_Size           EQU      __Vectors_End - __Vectors

                         AREA     |.text|, CODE, READONLY
Reset_Handler            PROC
                         EXPORT   Reset_Handler        [WEAK]
                         IMPORT   __main
                         LDR      R0, =SystemInit
                         BLX      R0
               IF  USER_MODE_EN = 1
                         ;switch to user mode
                         LDR      R0, =__initial_sp
                         MSR      PSP, R0                                      
                         MOV      R0,  #3
                         MSR      CONTROL, R0         
               ENDIF                         
                         LDR      R0, =__main
                         BX       R0
                         ENDP    

NMI_Handler              PROC
                         EXPORT   NMI_Handler          [WEAK]
                         B        .
                         ENDP    

HardFault_Handler        PROC
                         EXPORT   HardFault_Handler    [WEAK]
                         B        .
                         ENDP    

MemManage_Handler        PROC
                         EXPORT   MemManage_Handler    [WEAK]
                         B        .
                         ENDP    

BusFault_Handler         PROC
                         EXPORT   BusFault_Handler     [WEAK]
                         B        .
                         ENDP    

UsageFault_Handler       PROC
                         EXPORT   UsageFault_Handler   [WEAK]
                         B        .
                         ENDP    

SVC_Handler              PROC
                         EXPORT   SVC_Handler          [WEAK]
                 IF  USER_MODE_EN = 1
                         ;instruction svc push xPSR,PC,LR,R12,R3,R2,R1,R0
                         TST      LR,   #0x4         ;test return addr bit 2
                         ITE      EQ                 ;if 0
                         MRSEQ    R0,    MSP         ; use main stack
                         MRSNE    R0,    PSP         ; use user stack
                         B        SVC_Call_Proc
                ELSE
                         B        .
                ENDIF
                         ENDP    
                IF  USER_MODE_EN = 1
SVC_Call_Proc            PROC
                         EXPORT   SVC_Call_Proc       [WEAK]
                         BX       LR
                         ENDP
                ENDIF

DebugMon_Handler         PROC
                         EXPORT   DebugMon_Handler     [WEAK]
                         B        .
                         ENDP    

PendSV_Handler           PROC
                         EXPORT   PendSV_Handler       [WEAK]
                         B        .
                         ENDP    

SysTick_Handler          PROC
                         EXPORT   SysTick_Handler      [WEAK]
                         B        .
                         ENDP    

Default_Handler          PROC
                         EXPORT   WWDT_Handler         [WEAK]
                         EXPORT   BOD_Handler          [WEAK]
                         EXPORT   RCC_Handler          [WEAK]
                         EXPORT   EXTI0_Handler        [WEAK]
                         EXPORT   EXTI1_Handler        [WEAK]
                         EXPORT   EXTI2_Handler        [WEAK]
                         EXPORT   EXTI3_Handler        [WEAK]
                         EXPORT   EXTI4_Handler        [WEAK]
                         EXPORT   DMA_CH0_Handler      [WEAK]
                         EXPORT   DMA_CH1_Handler      [WEAK]
                         EXPORT   DMA_CH2_7_Handler    [WEAK]
                         EXPORT   MCM1_FLT_Handler     [WEAK]
                         EXPORT   MCM1_PWM_Handler     [WEAK]
                         EXPORT   ADC1_Handler         [WEAK]
                         EXPORT   ADC3_Handler         [WEAK]
                         EXPORT   GPT0_GTCIV_Handler   [WEAK]
                         EXPORT   GPT0_GTCIN_Handler   [WEAK]
                         EXPORT   GPT1_GTCIV_Handler   [WEAK]
                         EXPORT   GPT1_GTCIN_Handler   [WEAK]
                         EXPORT   GPT2_GTCIV_Handler   [WEAK]
                         EXPORT   GPT2_GTCIN_Handler   [WEAK]
                         EXPORT   GPT_POE_Handler      [WEAK]
                         EXPORT   EXTI9_5_Handler      [WEAK]
                         EXPORT   CMP1_Handler         [WEAK]
                         EXPORT   CMP2_Handler         [WEAK]
                         EXPORT   CMP3_Handler         [WEAK]
                         EXPORT   QEI_Handler          [WEAK]
                         EXPORT   TWI_Handler          [WEAK]
                         EXPORT   SPI1_Handler         [WEAK]
                         EXPORT   UART1_Handler        [WEAK]
                         EXPORT   UART2_Handler        [WEAK]
                         EXPORT   UART3_Handler        [WEAK]
                         EXPORT   EXTI15_10_Handler    [WEAK]
                         EXPORT   TIM5_Handler         [WEAK]
                         EXPORT   TIM6_Handler         [WEAK]
                         EXPORT   TIM7_Handler         [WEAK]
                         EXPORT   TIM8_Handler         [WEAK]
WWDT_Handler
BOD_Handler
RCC_Handler
EXTI0_Handler
EXTI1_Handler
EXTI2_Handler
EXTI3_Handler
EXTI4_Handler
DMA_CH0_Handler
DMA_CH1_Handler
DMA_CH2_7_Handler
MCM1_FLT_Handler
MCM1_PWM_Handler
ADC1_Handler
ADC3_Handler
GPT0_GTCIV_Handler
GPT0_GTCIN_Handler
GPT1_GTCIV_Handler
GPT1_GTCIN_Handler
GPT2_GTCIV_Handler
GPT2_GTCIN_Handler
GPT_POE_Handler
EXTI9_5_Handler
CMP1_Handler
CMP2_Handler
CMP3_Handler
QEI_Handler
TWI_Handler
SPI1_Handler
UART1_Handler
UART2_Handler
UART3_Handler
EXTI15_10_Handler
TIM5_Handler
TIM6_Handler
TIM7_Handler
TIM8_Handler
                         B        .
                         ENDP    

                         
SystemInit  PROC
                EXPORT  SystemInit  [WEAK]
                BX LR      
                ENDP
                    
                ALIGN


                ALIGN
; User Initial Stack & Heap

                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE
                
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =__initial_sp;
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = User_Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END
