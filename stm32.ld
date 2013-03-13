MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 128K
  SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
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
  }
  . = ALIGN(4);
  _end = .;
}
