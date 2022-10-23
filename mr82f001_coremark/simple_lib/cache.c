
#include "cache.h"


//************************************************************************
void En_Star_Cache_WT(void) {
  uint32_t CACR;

  MR32(CACHE_CACR_ADDR, CACR);
  
  // CACR
  // 2 FORCEWT
  MW32(CACHE_CACR_ADDR, CACR | 0x4);

  //SCB->CACR |= 0x4;

  __DSB();
  __ISB();
}


//************************************************************************
void Dis_Star_Cache_WT(void) {
  uint32_t CACR;

  MR32(CACHE_CACR_ADDR, CACR);
  
  // CACR
  // 2 FORCEWT
  MW32(CACHE_CACR_ADDR, CACR & 0xfffffffb);

  //SCB->CACR &= 0xfffffffb;
  
  
  __DSB();
  __ISB();
}


//************************************************************************
void Star_DCache_Data_Write(uint8_t way, uint32_t set, uint8_t offset, uint32_t data) {
  CACHE_DCA->DCADCLR_b.RAMTYPE  = 1;        // Data
  CACHE_DCA->DCADCLR_b.OFFSET   = offset;   // 
  CACHE_DCA->DCADCLR_b.SET      = set;      //
  CACHE_DCA->DCADCLR_b.WAY      = way;      //

  CACHE_DCA->DCADCRR = data;
}



//************************************************************************
uint32_t Star_DCache_Data_Read(uint8_t way, uint32_t set, uint8_t offset) {
  CACHE_DCA->DCADCLR_b.RAMTYPE  = 1;        // Data
  CACHE_DCA->DCADCLR_b.OFFSET   = offset;   // 
  CACHE_DCA->DCADCLR_b.SET      = set;      //
  CACHE_DCA->DCADCLR_b.WAY      = way;      //

  return CACHE_DCA->DCADCRR;
}


//************************************************************************
uint32_t Star_ICache_Data_Read(uint8_t way, uint32_t set, uint8_t offset) {
  CACHE_DCA->DCAICLR_b.RAMTYPE  = 1;        // Data
  CACHE_DCA->DCAICLR_b.OFFSET   = offset;   // 
  CACHE_DCA->DCAICLR_b.SET      = set;      //
  CACHE_DCA->DCAICLR_b.WAY      = way;      //

  return CACHE_DCA->DCAICRR;
}


//************************************************************************
uint32_t Star_DCache_Tag_Read(uint8_t way, uint32_t set) {
  CACHE_DCA->DCADCLR_b.RAMTYPE  = 0;        // Tag
  CACHE_DCA->DCADCLR_b.SET      = set;      //
  CACHE_DCA->DCADCLR_b.WAY      = way;      //

  return CACHE_DCA->DCADCRR;
}


//************************************************************************
uint32_t Star_ICache_Tag_Read(uint8_t way, uint32_t set) {
  CACHE_DCA->DCAICLR_b.RAMTYPE  = 0;        // Tag
  CACHE_DCA->DCAICLR_b.SET      = set;      //
  CACHE_DCA->DCAICLR_b.WAY      = way;      //

  return CACHE_DCA->DCAICRR;
}

