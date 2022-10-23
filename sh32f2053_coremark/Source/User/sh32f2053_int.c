/**
  ******************************************************************************
  * @file    sh32f2053_int.c 
  * @author  
  * @version V1.1.0
  * @date    15-April-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "sh32f2053_int.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */  
/* NMI ISR Handler template, uncomment it for using*/
/*void NMI_Handler(void)
{
}*/

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
/* HardFault ISR Handler template, uncomment it for using*/
//void HardFault_Handler(void)
//{
//  /* Go to infinite loop when Hard Fault exception occurs */
//  while (1)
//  {
//  }
//}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
/* Memory Manage ISR Handler template, uncomment it for using*/
//void MemManage_Handler(void)
//{
//  /* Go to infinite loop when Memory Manage exception occurs */
//  while (1)
//  {
//  }
//}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
/* Bus Fault ISR Handler template, uncomment it for using*/
//void BusFault_Handler(void)
//{
//  /* Go to infinite loop when Bus Fault exception occurs */
//  while (1)
//  {
//  }
//}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
/* Usage Fault ISR Handler template, uncomment it for using*/
//void UsageFault_Handler(void)
//{
//  /* Go to infinite loop when Usage Fault exception occurs */
//  while (1)
//  {
//  }
//}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
/* HardFault ISR Handler template, uncomment it for using*/
/*void SVC_Handler(void)
{
}*/

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
/* HardFault ISR Handler template, uncomment it for using*/
/*void DebugMon_Handler(void)
{
}*/

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/* HardFault ISR Handler template, uncomment it for using*/
/*void PendSV_Handler(void)
{
}*/

/**
  * @brief  This function handles SysTick exception.
  * @param  None
  * @retval None
  */
/* HardFault ISR Handler template, uncomment it for using*/
/*void SysTick_Handler(void)
{
}*/

/******************************************************************************/
/*                 SH32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_sh32fxxx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
