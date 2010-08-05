OUTPUT_FORMAT("elf32-i386")
OUTPUT_ARCH(i386)
ENTRY(_start)
PHDRS {
  physical PT_LOAD FILEHDR PHDRS;
  virtualtext PT_LOAD;
  virtualdata PT_LOAD;
}
SECTIONS {
  /* Place per-CPU data in phantom segment, to be dynamically allocated */
  . = 0x0;
  _percpustart = .;
  .percpu (NOLOAD) : { *(.percpu) } : NONE
  _percpuend = ALIGN(0x1000);
  _percpu_pages = (_percpuend - _percpustart) / 0x1000;

  /* Bootstrap physical section, at virtual and physical address 0x100000 */
  . = 0x100000 + SIZEOF_HEADERS;
  .physicaltext : {   boot/boot.o (.text) } : physical
  .physicalrodata : { boot/boot.o (.rodata) } : physical
  .physicaldata : {   boot/boot.o (.data) } : physical
  .physicalbss : {    boot/boot.o (.bss COMMON) } : physical
  _physicalkernelstart = ALIGN(0x1000);
  _bootstrap_pages = (_physicalkernelstart - 0x100000) / 0x1000;

  /* Permanent virtual section, at virtual address 0xFFC00000 */
  /* Maps kernel in upper 4MB of virtual address space on 32-bit machines */
  . = 0xFFC00000;
  _kernelstart = .;
  .text : AT ( _physicalkernelstart ) { *(.text) } : virtualtext
  . = ALIGN(4);
  .rodata : AT ( ( LOADADDR(.text) + SIZEOF(.text) + 3 ) & 0xFFFFFFFC )
    { *(.rodata*) } : virtualtext

  . = ALIGN(4);
  /* Insert a read-only null-terminated array of pointers to constructors */
  _percpu_ctor_list = .;
  .ctors : AT ( ( LOADADDR (.rodata) + SIZEOF (.rodata) + 3 ) & 0xFFFFFFFC )
              { *(.percpu.ctor); LONG(0) } : virtualtext

  . = ALIGN(0x1000);
   _kernel_readwrite = .;
  .data : AT ( ( LOADADDR(.ctors) + SIZEOF(.ctors) + 0xFFF ) & 0xFFFFF000 ) 
             { *(.data) } : virtualdata
  .bss : AT ( LOADADDR(.data) + SIZEOF(.data) )
            { *(.bss) *(COMMON) } : virtualdata
  _kernelend = ALIGN(0x1000);
  _readwrite_pages = ( _kernelend - _kernel_readwrite ) / 0x1000;
  _readonly_pages = ( _kernel_readwrite - _kernelstart ) / 0x1000;
}
