/*----------------------------------------------------------------------------
 * Name:    Retarget.c
 * Purpose: 'Retarget' layer for target-dependent low level functions
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2012 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include <rt_misc.h>

#pragma import(__use_no_semihosting_swi)


extern int  _PutChar(int c);                               
extern int  _GetChar(void);                                


struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;
FILE __stderr;


int fputc(int c, FILE *f) {
  if (c == '\n')  {
    _PutChar('\r');
  }
  return (_PutChar(c));
}


int fgetc(FILE *f) {
  return (_PutChar(_GetChar()));
}


int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}


void _ttywrch(int c) {
  _PutChar(c);
}


void _sys_exit(int return_code) {
label:  goto label;  /* endless loop */
}
