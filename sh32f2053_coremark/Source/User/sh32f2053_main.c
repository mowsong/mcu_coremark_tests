/**
  ******************************************************************************
  * @file    main.c
  * @author  
  * @version V1.1.0
  * @date    15-April-2016
  * @brief   
  ******************************************************************************
  */  
#include "sh32f2053.h"
#include "sh32f2053_periph_init.h"

/**
  * @brief  Main function
  * @param  None.
  * @retval None
  */
int platform_init(void)
{	
    RetargetInit();
    Peripherals_Init();
    SysTick_Config(120000UL); 
    return 0;
}

