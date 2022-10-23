/**
  ******************************************************************************
  * @file    sh32f205_retarget.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   this file provides printf retargetting functions
  *         
  *  @verbatim
  *
  *          ======================================================
  *                                   How to use this driver APIs
  *          ======================================================
  * 1. modify sh32f2xx_config.h which in the project
  *   - enable the printf if needed, then select output device is ITM or UART
  * 2. modify some parameters according by output device
  * 3. add this file into the project
  * 4. use printf to output the debugging information
  * 5. if disable this feature in config file, no codes added into the project
  *  @endverbatim
  *
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
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "sh32f205.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @defgroup Retarget_Group Retarget
  * @{
  */ 

/** @defgroup Retarget_Group_Pub_Funcs  Public Functions
  * @{
  */ 

#ifdef _MODULE_DBG_PRINTF


#if defined ( __CC_ARM )
struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;
#elif defined(__ICCARM__)
struct __FILE { int handle; /* Add whatever you need here */ };
typedef struct __FILE FILE;
FILE __stdout;
FILE __stdin;
#endif
  
#include <stdio.h>


#ifdef _DBG_DEVICE_ITM
//Cortex-M3 Debug Kernel Registers
#define ITM_DATA      0xE0000000
#define ITM_LOCK_KEY  0xC5ACCE55
#define ITM_KEY       (*((volatile unsigned long*)(0xE0000FB0)))
#define ITM_CTRL      (*((volatile unsigned long*)(0xE0000E80)))
#define ITM_PRIV      (*((volatile unsigned long*)(0xE0000E40)))
#define ITM_TRIG      (*((volatile unsigned long*)(0xE0000E20)))
#define ITM_ENABLE    (*((volatile unsigned long*)(0xE0000E00)))
#define ITM_PORT0     (*((volatile unsigned long*)(ITM_DATA)))
#define ITM_PORT1     (*((volatile unsigned long*)(ITM_DATA+4)))
#define DBGMCU_CR     (*((volatile unsigned long*)(0xE0042004)))
#define ITM_FREQ      (*((volatile unsigned long*)(0xE0040010)))   
#define ITM_CHARS     (*((volatile unsigned long*)(0xE0040004)))   
                    
#define DEM_CR        (*((volatile unsigned long*)(0xE000EDFC)))
#define DEM_CR_TRCENA (1 << 24)
#endif /*_DBG_DEVICE_ITM*/

/**
  * @brief    retarget initial function, parameters defined in sh32f2xx_config.h 
  * \n  1. initial the retarget device
  * \n  2. can be selected as ITM or UART
  * @retval  None
  */
void RetargetInit(void)
{
#ifdef _DBG_DEVICE_ITM  
    DEM_CR = 0;
    ITM_ENABLE = 0X0;
    DEM_CR |= DEM_CR_TRCENA;
    ITM_FREQ = SYS_CLK/ITM_FREQ_DEFAULT - 1;
    ITM_KEY = ITM_LOCK_KEY;
    ITM_CTRL = (1<<23)|(0x1<<0);
    ITM_PRIV = 0X1;
    ITM_ENABLE = 0X1;    
#endif /* _DBG_DEVICE_ITM */

#ifdef _DBG_DEVICE_UART
    /* Enable Clock Gates */
    RCC_APB1PeriphClockOnOff (DBG_UART_CLKGATE,ON);
    /* GPIO Config */
    RCC_AHBPeriphClockOnOff (RCC_AHB_GPIO,ON);
    GPIO_PinAFConfig(DBG_UART_PORT,DBG_UART_RX,DBG_UART_IOAF);
    GPIO_PinAFConfig(DBG_UART_PORT,DBG_UART_TX,DBG_UART_IOAF);
    {
        UART_InitTypeDef uartInit;
        uartInit.UART_Mode = UART_Mode_1;
        uartInit.UART_BaudRate = DBG_UART_BAUDRATE;
        uartInit.UART_DataLength = UART_DataLength_8Bit;
        uartInit.UART_StopBits = UART_StopBits_1;
        uartInit.UART_Parity = UART_Parity_None;
        uartInit.UART_Enable = UART_CR_TEN_Msk | UART_CR_REN_Msk;
        UART_Init(DBG_UART,&uartInit);
    }
#endif    
}


#ifdef _DBG_DEVICE_UART
/**
  * @brief send data to UART
  * @param ch: data to transmit
  * @retval  uint8_t data to transmit
  */
uint8_t dbg_uart_send(uint8_t ch)
{
   while(UART_GetFlagStatus(DBG_UART,UART_FLAG_TI) == RESET)
   {
        CLR_WDT();
   }
   UART_SendData(DBG_UART,ch);
   return (ch);
}   

/**
  * @brief read data from to UART
  * @retval  uint8_t data readed
  */
uint8_t dbg_uart_read()
{
   while(UART_GetFlagStatus(DBG_UART,UART_FLAG_RI) == RESET)
   {
       CLR_WDT();
   }        
   return (uint8_t)UART_ReceiveData8(DBG_UART);
}
  
#endif 
    

#ifdef _DBG_DEVICE_ITM

/* define scanf input buffer */
volatile int32_t ITM_RxBuffer = ITM_RXBUFFER_EMPTY;

#endif /* _DBG_DEVICE_ITM*/

/**
  * @brief    output character function
  * @param  ch  output character
  * @param  f   file handle
  * @retval  int   outputted character
  */
int fputc(int ch, FILE *f) 
{  
#ifdef _DBG_DEVICE_ITM
  if (ch == '\n')  {
    ITM_SendChar('\r');
  }
  return (ITM_SendChar(ch));
#endif 
  
#ifdef _DBG_DEVICE_UART
   if(ch == '\n'){
       dbg_uart_send('\r');
   }
   return (int)dbg_uart_send(ch);
#endif  
}

/**
  * @brief    input retarget
  * @param  f  file handle
  * @retval  int character which read
  */
int fgetc(FILE *f) 
{
  char ch;
#ifdef _DBG_DEVICE_ITM  
  while (ITM_CheckChar() != 1) 
  {    
      CLR_WDT();
  }
  ch = ITM_ReceiveChar();
  ITM_SendChar(ch);
  if(ch == '\r')
      ITM_SendChar('\n');
  flush_printfbuffer();
#endif

#ifdef _DBG_DEVICE_UART
  ch = dbg_uart_read();
  dbg_uart_send(ch);
  if(ch == '\r')
     dbg_uart_send('\n');
#endif 
   return ch;
}

#if defined ( __CC_ARM )
/*! flush printf buffer to output device. No body code*/
void flush_printfbuffer(void)
{
}
#endif

#if defined(__ICCARM__)

#ifdef _DBG_DEVICE_UART
/*! for replace the function in getchar.o */
int _UngetChar(void)
{
    return 0;
}
/*! for replace the function in getchar.o */
int __ungetchar(int a)
{
    return 0;
}
/**
  * @brief  replace the function getchar for IAR scanf
  * @retval  int data readed from input device
  */
int getchar(void)
{
   char ch = dbg_uart_read();
   dbg_uart_send(ch);
   if(ch == '\r')
      dbg_uart_send('\n');
   return (int)ch;
}

/**
  * @brief  replace the function putchar for IAR printf
  * @param ch data to output device
  * @retval  int data to output device
  */
int putchar(int ch) 
{  
   if(ch == '\n')
   {
       dbg_uart_send('\r');
   }
   return (int)dbg_uart_send(ch);
}
#endif /*_DBG_DEVICE_UART*/

/*! flush printf buffer to output device. No body code*/
void flush_printfbuffer(void)
{

}

#endif /*__ICCARM__*/



#if defined(__GNUC__)
/**
  * @brief  provide the function for GCC printf
  * @param  fd: file pointer no used
  * @param  pBuffer data buffer
  * @param  size data buffer size
  * @retval int outputted data size
  */
int _write (int fd, char *pBuffer, int size)  
{  
   int i;
#ifdef _DBG_DEVICE_ITM    
   for (i = 0; i < size; i++)  
   {  
      ITM_SendChar(pBuffer[i]);
      CLR_WDT();
   }  
#endif
   
#ifdef _DBG_DEVICE_UART   
   for (i = 0; i < size; i++)  
   {  
      if(pBuffer[i] == '\n')
      {
        dbg_uart_send('\r');
      }
      dbg_uart_send(pBuffer[i]);
   }     
#endif   
   return size;  
}  

/**
  * @brief  provide the function for GCC scanf
  * @param  fd: file pointer no used
  * @param  pBuffer data buffer
  * @param  size data buffer size
  * @retval int readed data size
  */
int _read (int fd, char *pBuffer, int size)  
{  
    int i;
#ifdef _DBG_DEVICE_ITM  
    for(i = 0; i < size; i++)
    {
        while (ITM_CheckChar() != 1) 
        {
            CLR_WDT();
        }
        pBuffer[i]=ITM_ReceiveChar();
        ITM_SendChar(pBuffer[i]);
        if(pBuffer[i] == '\r')
        {
            ITM_SendChar('\n');
            i += 1;
            pBuffer[i]='\n';
            i++;
            break;
        }
    }
#endif    
    
#ifdef _DBG_DEVICE_UART
    for(i = 0; i < size; i++)
    {
        pBuffer[i]=dbg_uart_read();
        dbg_uart_send(pBuffer[i]);
        if(pBuffer[i] == '\r')
        {
            dbg_uart_send('\n');
            i += 1;
            pBuffer[i]='\n';
            i++;
            break;
        }
    }    
#endif    
   return i;  
}      
#if !defined(__CC_ARM)
/*! flush printf buffer to output device */
void flush_printfbuffer(void)
{
    fflush(0);
}
#endif


#endif  /* __GNUC__ */
  
#endif /* _MODULE_DBG_PRINTF */

#ifdef DEFAULT_ASSERT_ENABLE
void assert_failed(uint8_t* file, uint32_t line)
{
#ifdef _MODULE_DBG_PRINTF     
    printf("assert failed:%s (%d)\n",file,(int)line);
#else
    __BKPT(0);
#endif    
}
#endif /* DEFAULT_ASSERT_ENABLE*/


/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/


