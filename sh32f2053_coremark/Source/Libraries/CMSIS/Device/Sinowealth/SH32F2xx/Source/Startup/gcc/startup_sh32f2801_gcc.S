;/*******************************************************************************/
;/* startup_sh32f2801_gcc.s: Startup file for Sinowealth SH32F2xx device series  */
;/********************************************************************************/
;/* This file is generated for gcc compiler                                      */
;/********************************************************************************/
;/*	                                                                             */
;/* SINOWEALTH IS SUPPLYING THIS SOFTWARE FOR USE EXCLUSIVELY ON SINOWEALTH'S    */
;/* MICROCONTROLLER PRODUCTS. IT IS PROTECTED UNDER APPLICABLE COPYRIGHT LAWS.   */
;/* THIS SOFTWARE IS FOR GUIDANCE IN ORDER TO SAVE TIME. AS A RESULT, SINOWEALTH */
;/* SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES   */
;/* WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH FIRMWARE AND/OR  */
;/* THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN      */
;/* CONNECTION WITH THEIR PRODUCTS.                                              */
;/*                                                                              */
;/* <h2><center>&copy; COPYRIGHT 2017 Sinowealth</center></h2>                   */
;/*                                                                              */
;/********************************************************************************/
;/********************************************************************************/

    .syntax unified
    .cpu cortex-m3
    .thumb
    
    .global    Reset_Handler

;// .section  .co_stack,"w"
;//	.space    0x400   /*stack size limit: check this while linking*/
;//	.section  .heap, "w"
;//	.space    0x400   /*heap size limit: check this while linking*/

   
    .section  .isr_vector ,"a",%progbits
     .long    __stack              ;/* -16 (000H) Top of Stack                                  */
     .long    Reset_Handler        ;/* -15 (004H) Reset Handler                                 */
     .long    NMI_Handler          ;/* -14 (008H) Non Maskable Interrupt                        */
     .long    HardFault_Handler    ;/* -13 (00CH) Hard Fault Interrupt                          */
     .long    MemManage_Handler    ;/* -12 (010H) Cortex-M3 Memory Management Interrupt         */
     .long    BusFault_Handler     ;/* -11 (014H) Cortex-M3 Bus Fault Interrupt                 */
     .long    UsageFault_Handler   ;/* -10 (018H) Cortex-M3 Usage Fault Interrupt               */
     .long    Default_Handler      ;/*  -9 (01CH) Reserved                                      */
     .long    Default_Handler      ;/*  -8 (020H) Reserved                                      */
     .long    Default_Handler      ;/*  -7 (024H) Reserved                                      */
     .long    Default_Handler      ;/*  -6 (028H) Reserved                                      */
     .long    SVC_Handler          ;/*  -5 (02CH) Cortex-M3 SV Call Interrupt                   */
     .long    DebugMon_Handler     ;/*  -4 (030H) Cortex-M3 Debug Monitor Interrupt             */
     .long    Default_Handler      ;/*  -3 (034H) Reserved                                      */
     .long    PendSV_Handler       ;/*  -2 (038H) Cortex-M3 Pend SV Interrupt                   */
     .long    SysTick_Handler      ;/*  -1 (03CH) Cortex-M3 System Tick Interrupt               */
     .long    WWDT_Handler         ;/*   0 (040H) Window WatchDog Interrupt                     */
     .long    BOD_Handler          ;/*   1 (044H) PVD through EXTI Line detection Interrupt     */
     .long    Default_Handler      ;/*   2 (048H) Reserved                                      */
     .long    Default_Handler      ;/*   3 (04CH) Reserved                                      */
     .long    RCC_Handler          ;/*   4 (050H) RCC global Interrupt                          */
     .long    EXTI0_Handler        ;/*   5 (054H) EXTI Line0 Interrupt                          */
     .long    EXTI1_Handler        ;/*   6 (058H) EXTI Line1 Interrupt                          */
     .long    EXTI2_Handler        ;/*   7 (05CH) EXTI Line2 Interrupt                          */
     .long    EXTI3_Handler        ;/*   8 (060H) EXTI Line3 Interrupt                          */
     .long    EXTI4_Handler        ;/*   9 (064H) EXTI Line4 Interrupt                          */
     .long    DMA_CH0_Handler      ;/*  10 (068H) DMA Channel0 Interrupt                        */
     .long    DMA_CH1_Handler      ;/*  11 (06CH) DMA Channel1 Interrupt                        */
     .long    DMA_CH2_7_Handler    ;/*  12 (070H) DMA Channel2 ~ Channel7 Interrupt             */
     .long    MCM1_FLT_Handler     ;/*  13 (074H) MCM1 FAULT Interrupt                          */
     .long    MCM1_PWM_Handler     ;/*  14 (078H) MCM1 PWM Interrupt                            */
     .long    Default_Handler      ;/*  15 (07CH) Reserved                                      */
     .long    Default_Handler      ;/*  16 (080H) Reserved                                      */
     .long    ADC1_Handler         ;/*  17 (084H) ADC1 Interrupt                                */
     .long    Default_Handler      ;/*  18 (088H) Reserved                                      */
     .long    ADC3_Handler         ;/*  19 (08CH) ADC3 Interrupt                                */
     .long    Default_Handler      ;/*  20 (090H) Reserved                                      */
     .long    Default_Handler      ;/*  21 (094H) Reserved                                      */
     .long    GPT0_GTCIV_Handler   ;/*  22 (098H) GPT0 GTCIV Interrupt                          */
     .long    GPT0_GTCIN_Handler   ;/*  23 (09CH) GPT0 GTCIN Interrupt                          */
     .long    GPT1_GTCIV_Handler   ;/*  24 (0A0H) GPT1 GTCIV Interrupt                          */
     .long    GPT1_GTCIN_Handler   ;/*  25 (0A4H) GPT1 GTCIN Interrupt                          */
     .long    GPT2_GTCIV_Handler   ;/*  26 (0A8H) GPT2 GTCIV Interrupt                          */
     .long    GPT2_GTCIN_Handler   ;/*  27 (0ACH) GPT2 GTCIN Interrupt                          */
     .long    Default_Handler      ;/*  28 (0B0H) Reserved                                      */
     .long    Default_Handler      ;/*  29 (0B4H) Reserved                                      */
     .long    GPT_POE_Handler      ;/*  30 (0B8H) GPT POE Global Interrupt                      */
     .long    EXTI9_5_Handler      ;/*  31 (0BCH) EXTI Line5 ~ Line9 global Interrupt           */
     .long    CMP1_Handler         ;/*  32 (0C0H) CMP1 Interrupt                                */
     .long    CMP2_Handler         ;/*  33 (0C4H) CMP2 Interrupt                                */
     .long    CMP3_Handler         ;/*  34 (0C8H) CMP3 Interrupt                                */
     .long    QEI_Handler          ;/*  35 (0CCH) QEI Interrupt                                 */
     .long    TWI_Handler          ;/*  36 (0D0H) TWI Interrupt                                 */
     .long    Default_Handler      ;/*  37 (0D4H) Reserved Interrupt                            */
     .long    SPI1_Handler         ;/*  38 (0D8H) SPI1 Interrupt                                */
     .long    Default_Handler      ;/*  39 (0DCH) Reserved                                      */
     .long    UART1_Handler        ;/*  40 (0E0H) UART1 Interrupt                               */
     .long    UART2_Handler        ;/*  41 (0E4H) UART2 Interrupt                               */
     .long    UART3_Handler        ;/*  42 (0E8H) UART3 Interrupt                               */
     .long    Default_Handler      ;/*  43 (0ECH) Reserved Interrupt                            */
     .long    EXTI15_10_Handler    ;/*  44 (0F0H) EXTI Line10 ~ Line15 Interrupt                */
     .long    Default_Handler      ;/*  45 (0F4H) Reserved Interrupt                            */
     .long    Default_Handler      ;/*  46 (0F8H) Reserved Interrupt                            */
     .long    Default_Handler      ;/*  47 (0FCH) Reserved Interrupt                            */
     .long    Default_Handler      ;/*  48 (100H) Reserved Interrupt                            */
     .long    TIM5_Handler         ;/*  49 (104H) TIM5 Interrupt                                */
     .long    TIM6_Handler         ;/*  50 (108H) TIM6 Interrupt                                */
     .long    TIM7_Handler         ;/*  51 (10CH) TIM7 Interrupt                                */
     .long    TIM8_Handler         ;/*  52 (110H) TIM8 Interrupt                                */
    
      .section  .text.Reset_Handler,"ax",%progbits
      .type Reset_Handler, %function
Reset_Handler:

    /* Call the low level init function.*/
    bl  SystemInit   
    
    movs r1, #0
    b    LoopCopyDataInit

CopyDataInit:
    ldr  r3, =_sidata
    ldr  r3, [r3, r1]
    str  r3, [r0, r1]
    adds r1, r1, #4
    
LoopCopyDataInit:
    ldr  r0, =_sdata
    ldr  r3, =_edata
    adds r2, r0, r1
    cmp  r2, r3
    bcc  CopyDataInit
    ldr  r2, =_sbss
    b    LoopFillZerobss
  
    ;/* Zero fill the bss segment. */  

FillZerobss:
    movs r3, #0
    str  r3, [r2], #4
    
LoopFillZerobss:
    ldr r3, =_ebss
    cmp r2, r3
    bcc FillZerobss
  
   /* copy cram_code */
    movs r1, #0
    b    LoopCopyCRamInit
CopyCRamInit:
    ldr  r3, =_sicramcode
    ldr  r3, [r3, r1]
    str  r3, [r0, r1]
    adds r1, r1, #4
LoopCopyCRamInit:
    ldr  r0, =_scramcode
    ldr  r3, =_ecramcode
    adds r2, r0, r1
    cmp  r2, r3
    bcc  CopyCRamInit
      
   /* Call static constructors */
   /* bl  __libc_init_array*/
	
  /* Call the application's entry point.*/
    bl  main
    bx  lr    
    b  .  
    
    .size  Reset_Handler, .-Reset_Handler
    
    .weak      NMI_Handler
    .thumb_set NMI_Handler,          System_Handler
  
    .weak      HardFault_Handler
    .thumb_set HardFault_Handler,    System_Handler
  
    .weak      MemManage_Handler
    .thumb_set MemManage_Handler,    System_Handler
  
    .weak      BusFault_Handler
    .thumb_set BusFault_Handler,     System_Handler

    .weak      UsageFault_Handler
    .thumb_set UsageFault_Handler,   System_Handler

    .weak      SVC_Handler
    .thumb_set SVC_Handler,          System_Handler

    .weak      DebugMon_Handler
    .thumb_set DebugMon_Handler,     System_Handler

    .weak      PendSV_Handler
    .thumb_set PendSV_Handler,       System_Handler

    .weak      SysTick_Handler
    .thumb_set SysTick_Handler,      System_Handler    

    .weak      WWDT_Handler        
    .thumb_set WWDT_Handler,        User_Handler
    .weak      BOD_Handler         
    .thumb_set BOD_Handler,         User_Handler
    .weak      RCC_Handler         
    .thumb_set RCC_Handler,         User_Handler
    .weak      EXTI0_Handler       
    .thumb_set EXTI0_Handler,       User_Handler
    .weak      EXTI1_Handler       
    .thumb_set EXTI1_Handler,       User_Handler
    .weak      EXTI2_Handler       
    .thumb_set EXTI2_Handler,       User_Handler
    .weak      EXTI3_Handler       
    .thumb_set EXTI3_Handler,       User_Handler
    .weak      EXTI4_Handler       
    .thumb_set EXTI4_Handler,       User_Handler
    .weak      DMA_CH0_Handler     
    .thumb_set DMA_CH0_Handler,     User_Handler
    .weak      DMA_CH1_Handler     
    .thumb_set DMA_CH1_Handler,     User_Handler
    .weak      DMA_CH2_7_Handler   
    .thumb_set DMA_CH2_7_Handler,   User_Handler
    .weak      MCM1_FLT_Handler    
    .thumb_set MCM1_FLT_Handler,    User_Handler
    .weak      MCM1_PWM_Handler    
    .thumb_set MCM1_PWM_Handler,    User_Handler
    .weak      ADC1_Handler        
    .thumb_set ADC1_Handler,        User_Handler
    .weak      ADC3_Handler        
    .thumb_set ADC3_Handler,        User_Handler
    .weak      GPT0_GTCIV_Handler  
    .thumb_set GPT0_GTCIV_Handler,  User_Handler
    .weak      GPT0_GTCIN_Handler  
    .thumb_set GPT0_GTCIN_Handler,  User_Handler
    .weak      GPT1_GTCIV_Handler  
    .thumb_set GPT1_GTCIV_Handler,  User_Handler
    .weak      GPT1_GTCIN_Handler  
    .thumb_set GPT1_GTCIN_Handler,  User_Handler
    .weak      GPT2_GTCIV_Handler  
    .thumb_set GPT2_GTCIV_Handler,  User_Handler
    .weak      GPT2_GTCIN_Handler  
    .thumb_set GPT2_GTCIN_Handler,  User_Handler
    .weak      GPT_POE_Handler     
    .thumb_set GPT_POE_Handler,     User_Handler
    .weak      EXTI9_5_Handler     
    .thumb_set EXTI9_5_Handler,     User_Handler
    .weak      CMP1_Handler        
    .thumb_set CMP1_Handler,        User_Handler
    .weak      CMP2_Handler        
    .thumb_set CMP2_Handler,        User_Handler
    .weak      CMP3_Handler        
    .thumb_set CMP3_Handler,        User_Handler
    .weak      QEI_Handler         
    .thumb_set QEI_Handler,         User_Handler
    .weak      TWI_Handler         
    .thumb_set TWI_Handler,         User_Handler
    .weak      SPI1_Handler        
    .thumb_set SPI1_Handler,        User_Handler
    .weak      UART1_Handler       
    .thumb_set UART1_Handler,       User_Handler
    .weak      UART2_Handler       
    .thumb_set UART2_Handler,       User_Handler
    .weak      UART3_Handler       
    .thumb_set UART3_Handler,       User_Handler
    .weak      EXTI15_10_Handler   
    .thumb_set EXTI15_10_Handler,   User_Handler
    .weak      TIM5_Handler        
    .thumb_set TIM5_Handler,        User_Handler
    .weak      TIM6_Handler        
    .thumb_set TIM6_Handler,        User_Handler
    .weak      TIM7_Handler        
    .thumb_set TIM7_Handler,        User_Handler
    .weak      TIM8_Handler        
    .thumb_set TIM8_Handler,        User_Handler
   
    .section  .text.System_Handler,"ax",%progbits
    .type System_Handler, %function
System_Handler:
   b  .
   .size  System_Handler, .-System_Handler
   
    .section  .text.Default_Handler,"ax",%progbits
    .type Default_Handler, %function
Default_Handler:
   b  .   
   .size  Default_Handler, .-Default_Handler

 
    .section  .text.User_Handler,"ax",%progbits
    .type User_Handler, %function
User_Handler:
   b  .
   .size  User_Handler, .-User_Handler
               
   .weak SystemInit
   .thumb_set SystemInit, SystemInit_Default
   .section  .text.SystemInit,"ax",%progbits   
   .type SystemInit_Default, %function
SystemInit_Default:
   bx lr
   .size SystemInit_Default, .-SystemInit_Default
