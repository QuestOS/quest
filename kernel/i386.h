#ifndef __I386_H__
#define __I386_H__
#include"types.h"

/* EFLAGS bits */
#define F_CF 0x01 /* carry */
#define F_1 0x02
#define F_PF 0x04 /* parity */
#define F_AF 0x10 /* auxiliary carry */
#define F_ZF 0x40 /* zero */
#define F_SF 0x80 /* sign */
#define F_TF 0x100 /* trap */
#define F_IF 0x200 /* interrupt enable */
#define F_DF 0x400 /* direction */
#define F_OF 0x800 /* overflow */
#define F_IOPL0 0x1000
#define F_IOPL1 0x2000
#define F_IOPL 0x3000 /* I/O privilege level */
#define F_NT 0x4000 /* nested task */
#define F_RF 0x10000 /* resume */
#define F_VM 0x20000 /* virtual 8086 mode */

/* Page table entry bits */
#define PTE_P 0x01 /* present */
#define PTE_RW 0x02 /* read/write */
#define PTE_US 0x04 /* user/supervisor */
#define PTE_A 0x20 /* accessed */
#define PTE_D 0x40 /* dirty */
#define PTE_AVAIL0 0x200
#define PTE_AVAIL1 0x400
#define PTE_AVAIL2 0x800
#define PTE_FRAME 0xFFFFF000 /* page frame address */

#define PORT1 0x03F8            /* COM1 */

#define PIC1_BASE_IRQ 0x20
#define PIC2_BASE_IRQ 0x28

typedef struct _tss {
    /* 80386 hardware data */
    uint16 usPrevious; /* previous task selector */
    uint16 usReserved0;
    uint32 ulESP0; /* ring 0 stack pointer */
    uint16 usSS0; /* ring 0 stack selector */
    uint16 usReserved1;
    uint32 ulESP1; /* ring 1 stack pointer */
    uint16 usSS1; /* ring 1 stack selector */
    uint16 usReserved2;
    uint32 ulESP2; /* ring 2 stack pointer */
    uint16 usSS2; /* ring 2 stack selector */
    uint16 usReserved3;
    void *pCR3; /* page directory */
    uint32 ulEIP; /* instruction pointer */
    uint32 ulEFlags;
    uint32 ulEAX, ulECX, ulEDX, ulEBX, ulESP, ulEBP, ulESI, ulEDI;
    uint16 usES, usReserved4, usCS, usReserved5, usSS, usReserved6,
	usDS, usReserved7, usFS, usReserved8, usGS, usReserved9;
    uint16 usLDT, usReserved10;
    unsigned int fTrap : 1;
    unsigned int uReserved11 : 15;
    uint16 usIOMap;
} tss;

/* Bit-field definitions for a segment descriptor */
typedef struct _descriptor {
    unsigned int uLimit0 : 16; /* limit (bits 0-15) */
    unsigned int pBase0 : 16; /* base (bits 0-15) */
    unsigned int pBase1 : 8; /* base (bits 16-23) */
    unsigned int uType : 5; /* type */
    unsigned int uDPL : 2; /* privilege level */
    unsigned int fPresent : 1; /* present */
    unsigned int uLimit1 : 4; /* limit (bits 16-19) */
    unsigned int f : 1; /* available */
    unsigned int f0 : 1; /* reserved */
    unsigned int fX : 1; /* varies (32-bit) */
    unsigned int fGranularity : 1; /* granularity */
    unsigned int pBase2 : 8; /* base (bits 24-31) */
} descriptor;

typedef struct _idt_descriptor {
  unsigned int pBase0 : 16;     /* Offset (bits 0-15) */
  unsigned int pSeg : 16;       /* Segment */
  unsigned int fReserved : 5;   /* reserved */
  unsigned int fZero0 : 3;      /* zeroed */
  unsigned int fType : 3;       /* 0x6 = interrupt, 0x7 = trap */
  unsigned int f32bit : 1;      /* 1 = 32 bit size of gate, 0 = 16 bit */
  unsigned int fZero1 : 1;      /* zeroed */
  unsigned int uDPL : 2;        /* Descriptor privilege level */
  unsigned int fPresent : 1;    /* present bit */
  unsigned int pBase1 : 16;     /* Offset (bits 16-31) */
} idt_descriptor;
  

static inline void cli( void ) {

    asm volatile( "cli" );
}

static inline void sti( void ) {

    asm volatile( "sti" );
}

__attribute__((noreturn)) static inline void hlt( void ) {

    for(;;)
	asm volatile( "hlt" );
}

/* Get physical address of the page directory in CR3 */
static inline void *get_pdbr( void ) {

    void *p;
    
    asm volatile( "movl %%cr3,%0" : "=r" (p) : );
    return p;
}


static inline void jmp_gate( uint16 us ) {

    uint16 gate[ 3 ];

    gate[ 2 ] = us;  /* Set segment selector -- ignore 4-byte offset */
    
    asm volatile( "ljmp *(%0)" : : "r" (gate) );
}


static inline void call_gate( uint16 us ) {

    uint16 gate[ 3 ];

    gate[ 2 ] = us;  /* Set segment selector -- ignore 4-byte offset */
    
    asm volatile( "lcall *(%0)" : : "r" (gate) );
}


static inline void bochs_instr_trace ( void ) {


  asm volatile( "outw %%ax, (%%dx)" : : "a" (0x8AE3), "d" (0x8A00));

}


static inline void bochs_regs_trace ( void ) {


  asm volatile( "outw %%ax, (%%dx)" : : "a" (0x8AE5), "d" (0x8A00));

}


static inline void invalidate_page ( void *va ) {


  asm volatile( "invlpg %0" : : "m" (*(char *) va) );

}


static inline void flush_tlb_all () {

  unsigned int tmpreg;

  asm volatile( "movl %%cr3, %0\n"
                "movl %0, %%cr3" : "=&r" (tmpreg) : );

}


static inline uint8 inb( uint16 usPort ) {

    uint8 uch;
    
    asm volatile( "inb %1,%0" : "=a" (uch) : "Nd" (usPort) );
    return uch;
}


static inline uint16 inw( uint16 usPort ) {

    uint16 us;
    
    asm volatile( "inw %1,%0" : "=a" (us) : "Nd" (usPort) );
    return us;
}

static inline uint32 inl( uint16 usPort ) {

    uint32 ul;
    
    asm volatile( "inl %1,%0" : "=a" (ul) : "Nd" (usPort) );
    return ul;
}


static inline void insw( uint16 usPort, void *buf, int count ) {

  asm volatile( "rep insw" : : "d" (usPort), "D" (buf), "c" (count) );
}


static inline void outsw( uint16 usPort, void *buf, int count ) {

  asm volatile( "rep outsw" : : "d" (usPort), "S" (buf), "c" (count) );
}


static inline void outb( uint8 uch, uint16 usPort ) {

    asm volatile( "outb %0,%1" : : "a" (uch), "Nd" (usPort) );
}


static inline void outw( uint16 us, uint16 usPort ) {

    asm volatile( "outw %0,%1" : : "a" (us), "Nd" (usPort) );
}

static inline void outl( uint32 ul, uint16 usPort ) {

    asm volatile( "outl %0,%1" : : "a" (ul), "Nd" (usPort) );
}


static inline uint16 str( void ) {

    uint16 us;

    asm volatile( "str %0" : "=r" (us) : );
    return us;
}


static inline void ltr( uint16 us ) {

    asm volatile( "ltr %0" : : "r" (us) );

}


static inline unsigned int ffs(unsigned int word)
{
  __asm__("bsfl %1,%0"
	  :"=r" (word)
	  :"rm" (word));
  return word;
}

#define RDTSC(var)                                              \
  {                                                             \
    uint32 var##_lo, var##_hi;                                   \
    asm volatile("rdtsc" : "=a"(var##_lo), "=d"(var##_hi));     \
    var = var##_hi;                                             \
    var <<= 32;                                                 \
    var |= var##_lo;                                            \
  }

#endif


