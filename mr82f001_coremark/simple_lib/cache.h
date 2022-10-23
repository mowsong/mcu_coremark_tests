
#ifndef __CACHE_H__
#define __CACHE_H__

#include "MR82F001.h"

#define CACHE_CACR_ADDR     (0xE001E000UL) 


typedef struct
{
 
  union {
    __IOM uint32_t DCADCRR;             // 0x00000000             

    struct {
      __IOM uint32_t TAG        : 22;   // [21:0]
      __IOM uint32_t VALID      :  1;   // [22:22]
      __IOM uint32_t NS         :  1;   // [23:23]
      __IOM uint32_t DIRTY      :  1;   // [24:24]
      __IM  uint32_t            :  7;   // [31:25]       
    } DCADCRR_b;
  } ;

  union {
    __IOM uint32_t DCAICRR;             // 0x00000004         

    struct {
      __IOM uint32_t TAG        : 21;   // [20:0]
      __IM  uint32_t            :  1;   // [21:21]
      __IOM uint32_t VALID      :  1;   // [22:22]
      __IM  uint32_t            :  9;   // [31:23]       
    } DCAICRR_b;
  } ;

  uint32_t _reserved08; 
  uint32_t _reserved0c; 

  union {
    __IOM uint32_t DCADCLR;             // 0x00000010             

    struct {
      __IOM uint32_t RAMTYPE    :  1;   // [0:0]
      __IM  uint32_t            :  1;   // [1:1]
      __IOM uint32_t OFFSET     :  3;   // [4:2]
      __IOM uint32_t SET        :  5;   // [9:5]
      __IM  uint32_t            : 20;   // [29:10]       
      __IOM uint32_t WAY        :  2;   // [31:30]
    } DCADCLR_b;
  } ;

 union {
    __IOM uint32_t DCAICLR;             // 0x00000014             

    struct {
      __IOM uint32_t RAMTYPE    :  1;   // [0:0]
      __IM  uint32_t            :  1;   // [1:1]
      __IOM uint32_t OFFSET     :  3;   // [4:2]
      __IOM uint32_t SET        :  6;   // [10:5]
      __IM  uint32_t            : 19;   // [29:11]       
      __IOM uint32_t WAY        :  1;   // [30:30]
      __IM  uint32_t            :  1;   // [31:31]       
    } DCAICLR_b;
  } ;
} CACHE_DCA_Type;


#define CACHE_DCA_BASE  (0xE001E200UL)

#define CACHE_DCA       ((CACHE_DCA_Type*)            CACHE_DCA_BASE)


void En_Star_Cache_WT(void);
void Dis_Star_Cache_WT(void);

void Star_DCache_Data_Write(uint8_t way, uint32_t set, uint8_t offset, uint32_t data);

uint32_t Star_DCache_Data_Read(uint8_t way, uint32_t set, uint8_t offset);
uint32_t Star_ICache_Data_Read(uint8_t way, uint32_t set, uint8_t offset);
uint32_t Star_DCache_Tag_Read(uint8_t way, uint32_t set);
uint32_t Star_ICache_Tag_Read(uint8_t way, uint32_t set);

#endif // __CACHE_H__
