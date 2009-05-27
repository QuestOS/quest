OUTPUT_FORMAT("elf32-i386")
OUTPUT_ARCH(i386)
ENTRY(_start)
SEARCH_DIR(../libc)
GROUP(-lc)
SECTIONS
{
  . = 0x1000 + SIZEOF_HEADERS;
  .text : { *(.text) }
  .rodata : { *(.rodata) }
  . = DATA_SEGMENT_ALIGN( 0x1000, 0x1000 );
  .data : { *(.data) }
  .bss : { *(.bss) *(COMMON) }
}
