/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : Main program body.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*
 *@Note
 ���ڴ�ӡ�������̣�
 USART1_Tx(PA9)��
 ��������ʾʹ�� USART1(PA9) ����ӡ���Կ������

*/

#include "debug.h"

void SysTick_Config(uint32_t tick);
void SysTick_Init(void);

/* Global typedef */

/* Global define */

/* Global Variable */
volatile uint32_t TICK = 0;

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
void platform_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART_Printf_Init(115200);
    SysTick_Init();

}

uint32_t SysTick_Get(void)
{
    uint32_t cntl = (*(__IO uint32_t *)0xE000F004);

    return cntl;
}

void SysTick_Init(void)
{
    SysTick->CTLR  = 0;
    SysTick->CNTL0 = 0xff;
    SysTick->CNTL1 = 0xff;
    SysTick->CNTL2 = 0xff;
    SysTick->CNTL3 = 0xff;
    SysTick->CNTH0 = 0xff;
    SysTick->CNTH1 = 0xff;
    SysTick->CNTH2 = 0xff;
    SysTick->CNTH3 = 0xff;
    SysTick->CTLR  = 1;
}

