
#include "pll.h"


                                 //    PLLINSEL PLLINSEL_A  CLKR    CLKF      CLKOD     BWADJ   freq
PLL_MODEL_t pll_hsrc8_12M_out       = {0,        0,         20,     503,      15,       62,     12000000 };
PLL_MODEL_t pll_hsrc8_16M_out       = {0,        0,         0,      31,       15,       3,      16000000 };
PLL_MODEL_t pll_hsrc8_20M_out       = {0,        0,         0,      39,       15,       4,      20000000 };
PLL_MODEL_t pll_hsrc8_24M_out       = {0,        0,         0,      47,       15,       5,      24000000 };
PLL_MODEL_t pll_hsrc8_28M_out       = {0,        0,         0,      55,       15,       6,      28000000 };
PLL_MODEL_t pll_hsrc8_32M_out       = {0,        0,         0,      55,       13,       6,      32000000 };
PLL_MODEL_t pll_hsrc8_36M_out       = {0,        0,         0,      53,       11,       6,      36000000 };
PLL_MODEL_t pll_hsrc8_40M_out       = {0,        0,         0,      49,       9,        5,      40000000 };
PLL_MODEL_t pll_hsrc8_44M_out       = {0,        0,         0,      54,       9,        6,      44000000 };
PLL_MODEL_t pll_hsrc8_48M_out       = {0,        0,         0,      47,       7,        5,      48000000 };
PLL_MODEL_t pll_hsrc8_52M_out       = {0,        0,         0,      51,       7,        5,      52000000 };
PLL_MODEL_t pll_hsrc8_56M_out       = {0,        0,         0,      55,       7,        6,      56000000 };
PLL_MODEL_t pll_hsrc8_60M_out       = {0,        0,         0,      44,       5,        5,      60000000 };
PLL_MODEL_t pll_hsrc8_64M_out       = {0,        0,         0,      47,       5,        5,      64000000 };
PLL_MODEL_t pll_hsrc8_68M_out       = {0,        0,         0,      50,       5,        5,      68000000 };
PLL_MODEL_t pll_hsrc8_72M_out       = {0,        0,         0,      53,       5,        6,      72000000 };
PLL_MODEL_t pll_hsrc8_76M_out       = {0,        0,         0,      37,       3,        4,      76000000 };
PLL_MODEL_t pll_hsrc8_80M_out       = {0,        0,         0,      39,       3,        4,      80000000 };
PLL_MODEL_t pll_hsrc8_84M_out       = {0,        0,         0,      41,       3,        4,      84000000 };
PLL_MODEL_t pll_hsrc8_88M_out       = {0,        0,         0,      43,       3,        5,      88000000 };
PLL_MODEL_t pll_hsrc8_92M_out       = {0,        0,         0,      46,       3,        5,      92000000 };
PLL_MODEL_t pll_hsrc8_96M_out       = {0,        0,         0,      47,       3,        5,      96000000 };
PLL_MODEL_t pll_hsrc8_100M_out      = {0,        0,         0,      49,       3,        5,      100000000 };
PLL_MODEL_t pll_hsrc8_104M_out      = {0,        0,         0,      51,       3,        5,      104000000 };
PLL_MODEL_t pll_hsrc8_108M_out      = {0,        0,         0,      53,       3,        6,      108000000 };
PLL_MODEL_t pll_hsrc8_112M_out      = {0,        0,         0,      55,       3,        6,      112000000 };
PLL_MODEL_t pll_hsrc8_116M_out      = {0,        0,         0,      28,       1,        3,      116000000 };
PLL_MODEL_t pll_hsrc8_120M_out      = {0,        0,         0,      29,       1,        3,      120000000 };
PLL_MODEL_t pll_hsrc8_124M_out      = {0,        0,         0,      30,       1,        3,      124000000 };
PLL_MODEL_t pll_hsrc8_128M_out      = {0,        0,         0,      31,       1,        3,      128000000 };
PLL_MODEL_t pll_hsrc8_132M_out      = {0,        0,         0,      32,       1,        3,      132000000 };
PLL_MODEL_t pll_hsrc8_136M_out      = {0,        0,         0,      33,       1,        3,      136000000 };
PLL_MODEL_t pll_hsrc8_140M_out      = {0,        0,         0,      34,       1,        3,      140000000 };
PLL_MODEL_t pll_hsrc8_144M_out      = {0,        0,         0,      35,       1,        3,      144000000 };
PLL_MODEL_t pll_hsrc8_148M_out      = {0,        0,         0,      36,       1,        4,      148000000 };
PLL_MODEL_t pll_hsrc8_152M_out      = {0,        0,         0,      37,       1,        4,      152000000 };
PLL_MODEL_t pll_hsrc8_156M_out      = {0,        0,         0,      38,       1,        4,      156000000 };
PLL_MODEL_t pll_hsrc8_160M_out      = {0,        0,         0,      39,       1,        4,      160000000 };
PLL_MODEL_t pll_hsrc8_164M_out      = {0,        0,         0,      40,       1,        4,      164000000 };
PLL_MODEL_t pll_hsrc8_168M_out      = {0,        0,         0,      41,       1,        4,      168000000 };
PLL_MODEL_t pll_hsrc8_172M_out      = {0,        0,         0,      42,       1,        5,      172000000 };
PLL_MODEL_t pll_hsrc8_176M_out      = {0,        0,         0,      43,       1,        5,      176000000 };
PLL_MODEL_t pll_hsrc8_180M_out      = {0,        0,         0,      44,       1,        5,      180000000 };
PLL_MODEL_t pll_hsrc8_184M_out      = {0,        0,         0,      45,       1,        5,      184000000 };
PLL_MODEL_t pll_hsrc8_188M_out      = {0,        0,         0,      46,       1,        5,      188000000 };
PLL_MODEL_t pll_hsrc8_192M_out      = {0,        0,         0,      47,       1,        5,      192000000 };
PLL_MODEL_t pll_hsrc8_196M_out      = {0,        0,         0,      48,       1,        5,      196000000 };
PLL_MODEL_t pll_hsrc8_200M_out      = {0,        0,         0,      49,       1,        5,      200000000 };

PLL_MODEL_t pll_hsrc4_160M_out      = {0,        0,         0,      79,       1,        9,      160000000 };

PLL_MODEL_t pll_hsrc12_160M_out     = {0,        0,         2,      79,       1,        9,      160000000 };


                                 //    PLLINSEL PLLINSEL_A  CLKR    CLKF      CLKOD     BWADJ   freq
PLL_MODEL_t pll_xtal10_16M_out      = {1,        0,         4,      111,      13,       13,     16000000 };
PLL_MODEL_t pll_xtal10_20M_out      = {1,        0,         0,      31,       15,       3,      20000000 };
PLL_MODEL_t pll_xtal10_32M_out      = {1,        0,         0,      31,       9,        3,      32000000 };
PLL_MODEL_t pll_xtal10_48M_out      = {1,        0,         4,      143,      5,        17,     48000000 };
PLL_MODEL_t pll_xtal10_64M_out      = {1,        0,         4,      127,      3,        15,     64000000 };
PLL_MODEL_t pll_xtal10_80M_out      = {1,        0,         0,      31,       3,        3,      80000000 };
PLL_MODEL_t pll_xtal10_96M_out      = {1,        0,         4,      191,      3,        23,     96000000 };
PLL_MODEL_t pll_xtal10_112M_out     = {1,        0,         4,      223,      3,        27,     112000000 };
PLL_MODEL_t pll_xtal10_128M_out     = {1,        0,         4,      127,      1,        15,     128000000 };
PLL_MODEL_t pll_xtal10_144M_out     = {1,        0,         4,      143,      1,        17,     144000000 };
PLL_MODEL_t pll_xtal10_160M_out     = {1,        0,         0,      31,       1,        3,      160000000 };

PLL_MODEL_t pll_xtal16_48M_out      = {1,        0,         0,      23,       7,        2,      48000000 };
PLL_MODEL_t pll_xtal16_64M_out      = {1,        0,         0,      23,       5,        2,      64000000 };
PLL_MODEL_t pll_xtal16_80M_out      = {1,        0,         0,      19,       3,        1,      80000000 };
PLL_MODEL_t pll_xtal16_160M_out     = {1,        0,         0,      19,       1,        1,      160000000 };

PLL_MODEL_t pll_xtal8_160M_out      = {1,        0,         0,      39,       1,        4,      160000000 };


                                 //    PLLINSEL PLLINSEL_A  CLKR    CLKF      CLKOD     BWADJ   freq
PLL_MODEL_t pll_lsrc32k_1p875M_out  = {0,        1,         0,      937,      15,       116,    1875000 }; // The min value PLL_cal can calculate
PLL_MODEL_t pll_lsrc32k_16M_out     = {0,        1,         0,      7999,     15,       999,    16000000 };
PLL_MODEL_t pll_lsrc32k_32M_out     = {0,        1,         0,      7999,     7,        999,    32000000 };
PLL_MODEL_t pll_lsrc32k_48M_out     = {0,        1,         0,      5999,     3,        749,    48000000 };
PLL_MODEL_t pll_lsrc32k_64M_out     = {0,        1,         0,      7999,     3,        999,    64000000 };
PLL_MODEL_t pll_lsrc32k_80M_out     = {0,        1,         0,      4999,     1,        624,    80000000 };
PLL_MODEL_t pll_lsrc32k_96M_out     = {0,        1,         0,      5999,     1,        749,    96000000 };
PLL_MODEL_t pll_lsrc32k_112M_out    = {0,        1,         0,      6999,     1,        874,    112000000 };
PLL_MODEL_t pll_lsrc32k_128M_out    = {0,        1,         0,      7999,     1,        999,    128000000 };
PLL_MODEL_t pll_lsrc32k_144M_out    = {0,        1,         0,      4499,     0,        561,    144000000 };
PLL_MODEL_t pll_lsrc32k_160M_out    = {0,        1,         0,      4999,     0,        624,    160000000 };



//************************************************************************
// RCC config use PLL

void PLL_Init(PLL_MODEL_t pll_m) {
  SYSCTRL->PLLCON_b.PLLEN     = 0; // disable PLL
  
  SYSCTRL->PLLDIV_b.CLKR       = pll_m.CLKR;
  SYSCTRL->PLLDIV_b.CLKF       = pll_m.CLKF;
  SYSCTRL->PLLDIV_b.CLKOD      = pll_m.CLKOD;
  SYSCTRL->PLLCON_b.BWADJ      = pll_m.BWADJ;
  SYSCTRL->PLLCON_b.PLLINSEL   = pll_m.PLLINSEL;
  SYSCTRL->PLLCON_b.PLLINSEL_A = pll_m.PLLINSEL_A;

  SYSCTRL->PLLCON_b.PLLEN     = 1; // enable PLL

  while (SYSCTRL->PLLCON_b.LOCKED == 0)
    __NOP();
}