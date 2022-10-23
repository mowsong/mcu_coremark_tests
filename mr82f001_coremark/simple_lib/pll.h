
#ifndef __PLL_H__
#define __PLL_H__

#include "MR82Fx01.h"

typedef struct {
  uint8_t   PLLINSEL;
  uint8_t   PLLINSEL_A;
  uint8_t   CLKR;
  uint16_t  CLKF;
  uint8_t   CLKOD;
  uint16_t  BWADJ;
  float     freq;
} PLL_MODEL_t;


extern PLL_MODEL_t pll_hsrc8_12M_out    ;
extern PLL_MODEL_t pll_hsrc8_16M_out    ;
extern PLL_MODEL_t pll_hsrc8_20M_out    ;
extern PLL_MODEL_t pll_hsrc8_24M_out    ;
extern PLL_MODEL_t pll_hsrc8_28M_out    ;
extern PLL_MODEL_t pll_hsrc8_32M_out    ;
extern PLL_MODEL_t pll_hsrc8_36M_out    ;
extern PLL_MODEL_t pll_hsrc8_40M_out    ;
extern PLL_MODEL_t pll_hsrc8_44M_out    ;
extern PLL_MODEL_t pll_hsrc8_48M_out    ;
extern PLL_MODEL_t pll_hsrc8_52M_out    ;
extern PLL_MODEL_t pll_hsrc8_56M_out    ;
extern PLL_MODEL_t pll_hsrc8_60M_out    ;
extern PLL_MODEL_t pll_hsrc8_64M_out    ;
extern PLL_MODEL_t pll_hsrc8_68M_out    ;
extern PLL_MODEL_t pll_hsrc8_72M_out    ;
extern PLL_MODEL_t pll_hsrc8_76M_out    ;
extern PLL_MODEL_t pll_hsrc8_80M_out    ;
extern PLL_MODEL_t pll_hsrc8_84M_out    ;
extern PLL_MODEL_t pll_hsrc8_88M_out    ;
extern PLL_MODEL_t pll_hsrc8_92M_out    ;
extern PLL_MODEL_t pll_hsrc8_96M_out    ;
extern PLL_MODEL_t pll_hsrc8_100M_out   ;
extern PLL_MODEL_t pll_hsrc8_104M_out   ;
extern PLL_MODEL_t pll_hsrc8_108M_out   ;
extern PLL_MODEL_t pll_hsrc8_112M_out   ;
extern PLL_MODEL_t pll_hsrc8_116M_out   ;
extern PLL_MODEL_t pll_hsrc8_120M_out   ;
extern PLL_MODEL_t pll_hsrc8_124M_out   ;
extern PLL_MODEL_t pll_hsrc8_128M_out   ;
extern PLL_MODEL_t pll_hsrc8_132M_out   ;
extern PLL_MODEL_t pll_hsrc8_136M_out   ;
extern PLL_MODEL_t pll_hsrc8_140M_out   ;
extern PLL_MODEL_t pll_hsrc8_144M_out   ;
extern PLL_MODEL_t pll_hsrc8_148M_out   ;
extern PLL_MODEL_t pll_hsrc8_152M_out   ;
extern PLL_MODEL_t pll_hsrc8_156M_out   ;
extern PLL_MODEL_t pll_hsrc8_160M_out   ;
extern PLL_MODEL_t pll_hsrc8_164M_out   ;
extern PLL_MODEL_t pll_hsrc8_168M_out   ;
extern PLL_MODEL_t pll_hsrc8_172M_out   ;
extern PLL_MODEL_t pll_hsrc8_176M_out   ;
extern PLL_MODEL_t pll_hsrc8_180M_out   ;
extern PLL_MODEL_t pll_hsrc8_184M_out   ;
extern PLL_MODEL_t pll_hsrc8_188M_out   ;
extern PLL_MODEL_t pll_hsrc8_192M_out   ;
extern PLL_MODEL_t pll_hsrc8_196M_out   ;
extern PLL_MODEL_t pll_hsrc8_200M_out   ;

extern PLL_MODEL_t pll_hsrc4_160M_out   ;

extern PLL_MODEL_t pll_hsrc12_160M_out   ;


extern PLL_MODEL_t pll_xtal10_16M_out    ;
extern PLL_MODEL_t pll_xtal10_20M_out    ;
extern PLL_MODEL_t pll_xtal10_32M_out    ;
extern PLL_MODEL_t pll_xtal10_48M_out    ;
extern PLL_MODEL_t pll_xtal10_64M_out    ;
extern PLL_MODEL_t pll_xtal10_80M_out    ;
extern PLL_MODEL_t pll_xtal10_96M_out    ;
extern PLL_MODEL_t pll_xtal10_112M_out   ;
extern PLL_MODEL_t pll_xtal10_128M_out   ;
extern PLL_MODEL_t pll_xtal10_144M_out   ;
extern PLL_MODEL_t pll_xtal10_160M_out   ;

extern PLL_MODEL_t pll_xtal8_160M_out    ;

extern PLL_MODEL_t pll_xtal16_48M_out    ;
extern PLL_MODEL_t pll_xtal16_64M_out    ;
extern PLL_MODEL_t pll_xtal16_80M_out    ;
extern PLL_MODEL_t pll_xtal16_160M_out   ;


extern PLL_MODEL_t pll_lsrc32k_1p875M_out; // The min value PLL_cal can calculate
extern PLL_MODEL_t pll_lsrc32k_16M_out   ;
extern PLL_MODEL_t pll_lsrc32k_32M_out   ;
extern PLL_MODEL_t pll_lsrc32k_48M_out   ;
extern PLL_MODEL_t pll_lsrc32k_64M_out   ;
extern PLL_MODEL_t pll_lsrc32k_80M_out   ;
extern PLL_MODEL_t pll_lsrc32k_96M_out   ;
extern PLL_MODEL_t pll_lsrc32k_112M_out  ;
extern PLL_MODEL_t pll_lsrc32k_128M_out  ;
extern PLL_MODEL_t pll_lsrc32k_144M_out  ;
extern PLL_MODEL_t pll_lsrc32k_160M_out  ; 


void PLL_Init(PLL_MODEL_t pll_m);

#endif // __PLL_H__
