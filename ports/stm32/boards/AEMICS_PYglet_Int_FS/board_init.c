#include "py/mphal.h"

void board_early_init(void) {

}

//User and read protection, @ 0x1FFF 7800
const uint32_t optionbytes1  __attribute__(( section(".optionbytes1") )) = 0xFBEFF8AA; // default 0xFFEFF8AA
const uint32_t optionbytes1n  __attribute__(( section(".optionbytes1n") )) = ~optionbytes1;
//PCROP1 Start address, @ 0x1FFF 7808
const uint32_t optionbytes2  __attribute__(( section(".optionbytes2") )) = 0xFFFFFFFF;
const uint32_t optionbytes2n  __attribute__(( section(".optionbytes2n") )) = ~optionbytes2;
//PCROP1 End address, @ 0x1FFF 7810
const uint32_t optionbytes3  __attribute__(( section(".optionbytes3") )) = 0x00FF0000;
const uint32_t optionbytes3n  __attribute__(( section(".optionbytes3n") )) = ~optionbytes3;
//WRP1 Area A address, @ 0x1FFF 7818
const uint32_t optionbytes4  __attribute__(( section(".optionbytes4") )) = 0xFF00FFFF;
const uint32_t optionbytes4n  __attribute__(( section(".optionbytes4n") )) = ~optionbytes4;
//WRP2 Area A address, @ 0x1FFF 7820
const uint32_t optionbytes5  __attribute__(( section(".optionbytes5") )) = 0xFF00FFFF;
const uint32_t optionbytes5n  __attribute__(( section(".optionbytes5n") )) = ~optionbytes5;
//Securable memory area Bank 1, @ 0x1FFF 7828
const uint32_t optionbytes6  __attribute__(( section(".optionbytes6") )) = 0xFF00FF00;
const uint32_t optionbytes6n  __attribute__(( section(".optionbytes6n") )) = ~optionbytes6;

//PCROP2 Start address, @ 0x1FFFF808
const uint32_t optionbytes7  __attribute__(( section(".optionbytes7") )) = 0xFFFFFFFF;
const uint32_t optionbytes7n  __attribute__(( section(".optionbytes7n") )) = ~optionbytes7;
//PCROP2 End address, @ 0x1FFF F810
const uint32_t optionbytes8  __attribute__(( section(".optionbytes8") )) = 0xFFFFFFFF;
const uint32_t optionbytes8n  __attribute__(( section(".optionbytes8n") )) = ~optionbytes8;
//WRP1 Area B address, @ 0x1FFF F818
const uint32_t optionbytes9  __attribute__(( section(".optionbytes9") )) = 0xFF00FFFF;
const uint32_t optionbytes9n  __attribute__(( section(".optionbytes9n") )) = ~optionbytes9;
//WRP2 Area B address, @ 0x1FFF F820
const uint32_t optionbytes10 __attribute__(( section(".optionbytes10") )) = 0xFF00FFFF;
const uint32_t optionbytes10n __attribute__(( section(".optionbytes10n") )) = ~optionbytes10;
//Securable memory area Bank 2, @ 0x1FFF F828
const uint32_t optionbytes11 __attribute__(( section(".optionbytes11") )) = 0xFF00FF00;
const uint32_t optionbytes11n __attribute__(( section(".optionbytes11n") )) = ~optionbytes11;

