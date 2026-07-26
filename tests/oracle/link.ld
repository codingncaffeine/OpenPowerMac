OUTPUT_FORMAT("elf32-powerpc")
OUTPUT_ARCH(powerpc)
ENTRY(_start)

SECTIONS {
  . = 0x00100000;
  .text   : { *(.text*) }
  .rodata : { *(.rodata*) *(.got2) }
  .data   : { *(.data*) *(.sdata*) }
  .bss    : { *(.bss*) *(.sbss*) *(COMMON) }
}
