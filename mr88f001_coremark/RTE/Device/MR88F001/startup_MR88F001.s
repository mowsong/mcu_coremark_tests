;/******************************************************************************
; * @file    startup_MR88F001.s
; * @brief   CMSIS Cortex-M0 Core Device Startup File for Device MR88F001
; *          
; * @version 2.0.3
; * @date    2021-11-23
; *
; * @note
; * Copyright (C) MR Semi Co. Ltd. All rights reserved.
; *
; * @par
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000C00

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000100

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler
                DCD     0
                DCD     VD_Handler
                DCD     LFDET_Handler
                DCD     FLSC_Handler
                DCD     UART0_Handler
                DCD     UART1_Handler
                DCD     UART2_Handler
                DCD     UART3_Handler
                DCD     ADC_Handler
                DCD     I2C0_Handler
                DCD     SPI0_Handler
                DCD     SPI1_Handler
                DCD     TIM0_Handler
                DCD     TIM1_Handler
                DCD     TIM2_Handler
                DCD     TIM3_Handler
                DCD     ATIM_Handler
                DCD     LPTIM_Handler
                DCD     LEDC_Handler
                DCD     TOUCH_Handler
                DCD     DMA_Handler
                DCD     0
                DCD     0
                DCD     WKUP_Handler
                DCD     EXTI_Handler
                DCD     I2C1_Handler
                DCD     BSTIM0_Handler
                DCD     BSTIM1_Handler
                DCD     0
                DCD     0
                DCD     0
                DCD     0
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
; ensure system clock is 8MHz HSRC              
                LDR     R0, =0x40010214 
                LDR     R1, =0x1        
                STR     R1, [R0]
                LDR     R0, =0x4001020C 
                LDR     R1, =0x0        
                STR     R1, [R0]
; small delay to allow SWD quick connect                
                LDR     R1, =0x0    
                LDR     R0, =0x4001022C
                LDR     R1, [R0]
                MOVS    R2, #1
                LSLS    R2, R2, #12
                ORRS    R1, R1, R2
                STR     R1, [R0]
                LDR     R0, =0x40006000
                LDR     R1, =0x5A5A5A5A
                STR     R1, [R0]
                LDR     R0, =0
                LDR     R1, =80000 ; (800cycle/ms * 100ms) = 100ms
                LDR     R2, =1
Startup_L       CMP     R0, R1
                ADD     R0, R0, R2
                BLT     Startup_L

                LDR     R0, =0x40006000
                LDR     R1, =0x5A5A5A5A
                STR     R1, [R0]
                
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .
                ENDP
Default_Handler PROC
                EXPORT VD_Handler           [WEAK]
                EXPORT LFDET_Handler        [WEAK]
                EXPORT FLSC_Handler         [WEAK]
                EXPORT UART0_Handler        [WEAK]
                EXPORT UART1_Handler        [WEAK]
                EXPORT UART2_Handler        [WEAK]
                EXPORT UART3_Handler        [WEAK]
                EXPORT ADC_Handler          [WEAK]
                EXPORT I2C0_Handler         [WEAK]
                EXPORT SPI0_Handler         [WEAK]
                EXPORT SPI1_Handler         [WEAK]
                EXPORT TIM0_Handler         [WEAK]
                EXPORT TIM1_Handler         [WEAK]
                EXPORT TIM2_Handler         [WEAK]
                EXPORT TIM3_Handler         [WEAK]
                EXPORT ATIM_Handler         [WEAK]
                EXPORT LPTIM_Handler        [WEAK]
                EXPORT LEDC_Handler         [WEAK]
                EXPORT TOUCH_Handler        [WEAK]
                EXPORT DMA_Handler          [WEAK]
                EXPORT WKUP_Handler         [WEAK]
                EXPORT EXTI_Handler         [WEAK] 
                EXPORT I2C1_Handler         [WEAK]
                EXPORT BSTIM0_Handler       [WEAK]
                EXPORT BSTIM1_Handler       [WEAK]
VD_Handler
LFDET_Handler
FLSC_Handler
UART0_Handler
UART1_Handler
UART2_Handler
UART3_Handler
ADC_Handler
I2C0_Handler
I2C1_Handler
SPI0_Handler
SPI1_Handler
TIM0_Handler
TIM1_Handler
TIM2_Handler
TIM3_Handler
ATIM_Handler
LPTIM_Handler
LEDC_Handler
TOUCH_Handler
DMA_Handler
WKUP_Handler
EXTI_Handler
BSTIM0_Handler
BSTIM1_Handler
                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
