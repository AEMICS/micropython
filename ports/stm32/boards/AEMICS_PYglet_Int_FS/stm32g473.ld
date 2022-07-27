/* Specify the memory areas */
MEMORY
{
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 128K
/* FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K */
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 352K
/* FLASH_QSPI (rx) : ORIGIN = 0x90000000, LENGTH = 1M     /* external QSPI flash in XIP mode * /*/
FLASH_FS (rx)   : ORIGIN = 0x08058000, LENGTH = 160K  /* starting @ 352K */
FUSESB0 (r)     : ORIGIN = 0x1fff7800, LENGTH = 48 /* Option Bytes (configuration fuses) */
FUSESB1 (r)     : ORIGIN = 0x1ffff808, LENGTH = 40 /* Option Bytes (configuration fuses) */

}

/* Generate a link error if heap and stack don't fit into RAM */
_minimum_heap_size = 0x200;      /* required amount of heap  */
_minimum_stack_size = 0x400; /* required amount of stack */

_ram_fs_cache_end = ORIGIN(RAM) + LENGTH(RAM);
_ram_fs_cache_start = _ram_fs_cache_end - 2K; /* fs cache = 2K RAM */

/* Define the stack.  The stack is full descending so begins just above last byte
   of RAM.  Note that EABI requires the stack to be 8-byte aligned for a call. */
_estack = _ram_fs_cache_start - _estack_reserve;
_sstack = _estack - 8K; /* tunable */ 

/* RAM extents for the garbage collector */
_ram_start = ORIGIN(RAM);
_ram_end = ORIGIN(RAM) + LENGTH(RAM);

_heap_start = _ebss; /* heap starts just after statically allocated memory */
_heap_end = _sstack;

/* _flash_fs_start = ORIGIN(FLASH_QSPI);
_flash_fs_end   = ORIGIN(FLASH_QSPI) + LENGTH(FLASH_QSPI); */

_flash_fs_start = ORIGIN(FLASH_FS);
_flash_fs_end   = ORIGIN(FLASH_FS) + LENGTH(FLASH_FS);

/* Memory layout for basic configuration:

    FLASH       .isr_vector
    FLASH       .text
    FLASH       .data

    RAM         .data
    RAM         .bss
    RAM         .heap
    RAM         .stack
*/

ENTRY(Reset_Handler)

/* define output sections */
SECTIONS
{
    /* The startup code goes first into FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Startup code */

        . = ALIGN(4);
    } >FLASH

    /* The program code and other data goes into FLASH */
    .text :
    {
        . = ALIGN(4);
        *(.text*)          /* .text* sections (code) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    /*  *(.glue_7)   */    /* glue arm to thumb code */
    /*  *(.glue_7t)  */    /* glue thumb to arm code */

        . = ALIGN(4);
        _etext = .;        /* define a global symbol at end of code */
    } >FLASH
	
	fuses_bank0 :
	{
	. = ALIGN(4);
	KEEP(*(.optionbytes1))
	KEEP(*(.optionbytes1n))
	KEEP(*(.optionbytes2))
	KEEP(*(.optionbytes2n))
	KEEP(*(.optionbytes3))
	KEEP(*(.optionbytes3n))
	KEEP(*(.optionbytes4))
	KEEP(*(.optionbytes4n))
	KEEP(*(.optionbytes5))
	KEEP(*(.optionbytes5n))
	KEEP(*(.optionbytes6))
	KEEP(*(.optionbytes6n))
	. = ALIGN(4);
	} > FUSESB0
	fuses_bank1 :
	{
	. = ALIGN(4);
	KEEP(*(.optionbytes7))
	KEEP(*(.optionbytes7n))
	KEEP(*(.optionbytes8))
	KEEP(*(.optionbytes8n))
	KEEP(*(.optionbytes9))
	KEEP(*(.optionbytes9n))
	KEEP(*(.optionbytes10))
	KEEP(*(.optionbytes10n))
	KEEP(*(.optionbytes11))
	KEEP(*(.optionbytes11n))
	. = ALIGN(4);
	} >FUSESB1

    INCLUDE common_extratext_data_in_flash.ld
    INCLUDE common_bss_heap_stack.ld
}