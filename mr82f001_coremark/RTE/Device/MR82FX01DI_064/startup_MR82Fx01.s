;/******************************************************************************
; * @file     startup_MR82Fx01.s
; * @brief    CMSIS Star Core Device Startup File for
; *           Device MR86F001
; * @version 1.0.7
; * @date    2021-08-22
; *
; * @note
; * Copyright (C) 2019 MR Semi Co. Ltd. All rights reserved.
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
                DCD     ADC0_Handler
                DCD     ADC1_Handler
                DCD     ADC2_Handler
                DCD     TIM0BK_Handler
                DCD     TIM1BK_Handler
                DCD     TIM0_Handler
                DCD     TIM1_Handler
                DCD     ATIM0BK_Handler
                DCD     ATIM1BK_Handler
                DCD     ATIM0_Handler
                DCD     ATIM1_Handler
                DCD     0
                DCD     CMP1_Handler
                DCD     CMP2_Handler
                DCD     CMP3_Handler
                DCD     WKUP_Handler
                DCD     I2C0_Handler
                DCD     I2C1_Handler
                DCD     BSTIM0_Handler
                DCD     BSTIM1_Handler
                DCD     BSTIM2_Handler
                DCD     BSTIM3_Handler
                DCD     FPU_Handler
                DCD     SPI0_Handler
                DCD     SPI1_Handler
                DCD     LPTIM_Handler
                DCD     EXTIA_Handler
                DCD     EXTIB_Handler
                DCD     EXTIC_Handler
                DCD     EXTID_Handler
                DCD     DMA_Handler
                DCD     DMAERR_Handler
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                
; small delay to allow SWD quick connect
                LDR     R0, =0x4001022C
                LDR     R1, [R0]
                MOVS    R2, #1
                LSLS    R2, R2, #12
                ORRS    R1, R1, R2
                STR     R1, [R0]
                LDR     R0, =0x40006000
                LDR     R1, =0x5A5A5A5A
                STR     R1, [R0]
				LDR 	R0, =0
				LDR		R1, =1600
				LDR		R2, =1
Startup_L		CMP		R0, R1
				ADD		R0, R0, R2
				BLT 	Startup_L
                
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
                EXPORT ADC0_Handler         [WEAK]
                EXPORT ADC1_Handler         [WEAK]
                EXPORT ADC2_Handler         [WEAK]
                EXPORT TIM0BK_Handler       [WEAK]
                EXPORT TIM1BK_Handler       [WEAK]
                EXPORT TIM0_Handler         [WEAK]
                EXPORT TIM1_Handler         [WEAK]
                EXPORT ATIM0BK_Handler      [WEAK]
                EXPORT ATIM1BK_Handler      [WEAK]
                EXPORT ATIM0_Handler        [WEAK]
                EXPORT ATIM1_Handler        [WEAK]
                EXPORT CMP1_Handler         [WEAK]
                EXPORT CMP2_Handler         [WEAK]
                EXPORT CMP3_Handler         [WEAK]
                EXPORT WKUP_Handler         [WEAK]
                EXPORT I2C0_Handler         [WEAK]
                EXPORT I2C1_Handler         [WEAK]
                EXPORT BSTIM0_Handler       [WEAK]
                EXPORT BSTIM1_Handler       [WEAK]
                EXPORT BSTIM2_Handler       [WEAK]
                EXPORT BSTIM3_Handler       [WEAK]
                EXPORT FPU_Handler          [WEAK]
                EXPORT SPI0_Handler         [WEAK]
                EXPORT SPI1_Handler         [WEAK]
                EXPORT LPTIM_Handler        [WEAK]
                EXPORT EXTIA_Handler        [WEAK] 
                EXPORT EXTIB_Handler        [WEAK] 
                EXPORT EXTIC_Handler        [WEAK] 
                EXPORT EXTID_Handler        [WEAK] 
                EXPORT DMA_Handler          [WEAK]
                EXPORT DMAERR_Handler       [WEAK]

VD_Handler
LFDET_Handler
FLSC_Handler
UART0_Handler
UART1_Handler
UART2_Handler
UART3_Handler
ADC0_Handler
ADC1_Handler
ADC2_Handler
TIM0BK_Handler
TIM1BK_Handler
TIM0_Handler
TIM1_Handler
ATIM0BK_Handler
ATIM1BK_Handler
ATIM0_Handler
ATIM1_Handler
CMP1_Handler
CMP2_Handler
CMP3_Handler
WKUP_Handler
I2C0_Handler
I2C1_Handler
BSTIM0_Handler
BSTIM1_Handler
BSTIM2_Handler
BSTIM3_Handler
FPU_Handler
SPI0_Handler
SPI1_Handler
LPTIM_Handler
EXTIA_Handler
EXTIB_Handler
EXTIC_Handler
EXTID_Handler
DMA_Handler
DMAERR_Handler
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
