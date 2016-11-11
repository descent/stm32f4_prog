/*
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 128K
  SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}
*/

/* Specify the memory areas */
/* form: STM32F4-Discovery_FW_V1.1.0/Project/Peripheral_Examples/IO_Toggle/TrueSTUDIO/IO_Toggle/stm32_flash.ld */
MEMORY
{
  FLASH (rx)      : ORIGIN = 0x00000000, LENGTH = 1024K
  CCM (rw)        : ORIGIN = 0x10000000, LENGTH = 64K /* 0x1000_0000 ~ 0x1000_FFFF. */
  SRAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 128K
}


SECTIONS
{
  .text :
  {
    KEEP(*(.isr_vector .isr_vector.*))
    *(.text .text.*)
    *(.rodata .rodata*)
    _etext = .;
  } > FLASH
  .data : AT (_etext)
  {
    _data = .;
    *(.data .data.*)
    _edata = .;
  } > SRAM
  .bss(NOLOAD) :
  {
    _bss = .;
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > SRAM
  .stackarea(NOLOAD) :
  {
    . = ALIGN(8);
    *(.stackarea .stackares.*)
    . = ALIGN(8);
  } > SRAM
  .ccm(NOLOAD):
  {
    . = ALIGN(8);
    *(.ccm)
    . = ALIGN(8);
  } > CCM

  . = ALIGN(4);
  _end = .;
}
