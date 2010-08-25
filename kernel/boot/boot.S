/* -*- Mode: asm; comment-start: "\/*"; comment-end: "*\/"; indent-tabs-mode: nil -*- */
/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * boot.S
 *
 *
 * NOTE:         All references to Intel documentation are from the manuals 
 *               dated 1997-2001. Updated book versions may be on-line.         
 */


#include "kernel.h"

        .globl initial_gdt
        
        .bss
        .align 0x1000 /* align page directory on 4K boundary - 2-12 Manual vol 3 */
        .comm pgd, 0x1000   /* setup 4Kbytes for page global directory */

        .align 0x1000
        .comm idt, 0x1000   /* setup 4Kbytes for IDT+GDT */

        .align 0x1000 
        .comm kern_pg_table, 0x1000 /* setup 4Kbytes for kernel page table */

        .align 0x1000
        .comm tmp_stack, 0x1000
        
        .data
        .align 4
initial_gdt:
        /* null descriptor -- 1st entry not used -- see Manual 3-16 vol 3 */
        .long 0
        .long 0

        /* Kernel CS */
        /* code descriptor - base 0, limit 0xFFFFFFFF, 32-bit, readable, ring 0 */
        .long 0x0000FFFF
        .long 0x00CF9A00

        /* Kernel DS */
        /* data descriptor - base 0, limit 0xFFFFFFFF, 32-bit, writable, ring 0 */
        .long 0x0000FFFF
        .long 0x00CF9200

        /* User CS */
        /* code descriptor - base 0, limit 0xBFFFFFFF, 32-bit, readable, ring 3 */
        .long 0x0000FFFF
        .long 0x00CBFA00
        /* User DS */
        /* data descriptor - base 0, limit 0xBFFFFFFF, 32-bit, writable, ring 3 */
        .long 0x0000FFFF
        .long 0x00CBF200

gdt_ptr:
        .short 0x7FF     /* length in bytes - 256 descriptors */
        .long KERN_GDT   /* linear address */

        .globl idt_ptr
idt_ptr:
        .short 0x7FF     /* length - 256 descriptors */
        .long KERN_IDT   /* linear address */

        
        .text
        .globl _start

_start:
        jmp real_start

        /* Multiboot header -- Safe to place this header in 1st page for GRUB */
        .align 4
        .long 0x1BADB002 /* Multiboot magic number */
        .long 0x00000003 /* Align modules to 4KB, req. mem size */
                         /* See 'info multiboot' for further info */
        .long 0xE4524FFB /* Checksum */

real_start:

        /* turn on Bochs debugging using I/O ports */
        movw $0x8A00, %ax
        movw $0x8A00, %dx
        outw %ax, (%dx)
        
        /* set up stack */
        movl $tmp_stack+0x1000, %esp /* setup temp stack */

        /* save multiboot parameter, for eventual call to C code */
        pushl %ebx

        /* initialise EFLAGS */
        pushl $2 /* Bit 1 always set to 1 in EFLAGS -- all other bits set to 0 */
        popf     /* See 2-8 Manual vol 3 */

        /* set up GDT */
        movl $idt, %edi
        addl $0x800, %edi /* 2KB offset of GDT from IDT */
        movl $initial_gdt, %esi
        movl $10, %ecx /* setup 5 segment descriptors of 64-bits each */
        rep movsl
        lgdt gdt_ptr /* pass address of 6-byte object */

        /* set up support for 4MB paging */
        movl %cr4, %eax /* EAX is temporary for CR4 */
        orl $0x10, %eax /* Set PSE bit of CR4 */
        movl %eax, %cr4

        /* Setup 1st page directory entry for super-paging */
        movl $0x83, %eax 
        movl %eax, pgd

        /* LAPIC address mapping - 4MB page @ 0xFEC00000 */
        movl $0xFEC00083, %eax
        movl %eax, pgd+0xFEC 
        
        /* Set 1024th (top) entry in PGD to reference kern_pg_table
         * Here, we have 1 4MB superpage for the entire kernel
         */
        movl $kern_pg_table+3, %eax /* +3=>present and r/w */
        movl %eax, pgd+0xFFC /* --WARN-- Hardcoded pgd offset */

        /* Add page table entries for kernel portion at high virtual addresses
         * Here, we have 1024x4KB page table entries for the kernel
         */
        movl $kern_pg_table, %edi
        movl $_physicalkernelstart, %eax
        orl $0x1, %eax /* present bit set in page */
        movl $_readonly_pages, %ecx
1:      stosl
        addl $0x1000, %eax
        loop 1b

        orl $0x3, %eax /* present and read/write bits set in each page */
        movl $_readwrite_pages, %ecx
1:      stosl
        addl $0x1000, %eax
        loop 1b
        
        /* Setup entry in kernel page table for KERNEL IDT/GDT */
        movl $idt+3, %eax /* idt is page aligned in bss - 
                           * set physical address in
                           * kernel page table:
                           * +3 indicates present/write-enabled 
                           */

        /* NOTE: shift 10 bits rather than 12 due to kern_pg_table being
           array of ints */
        movl %eax, kern_pg_table + ( ( KERN_IDT >> 10 ) & 0x0FFF )

        /* Here, remap kern_pg_table to high virtual memory to be
           accessible later by user-level code via syscalls */
        movl $kern_pg_table+3, %eax 
        movl %eax, kern_pg_table + ( ( KERN_PGT >> 10 ) & 0x0FFF )

        /* This is for mapping screen memory into kernel for screen dumps */
        movl $0x000B8003, %eax
        movl %eax, kern_pg_table + ( ( KERN_SCR >> 10 ) & 0x0FFF )

        /* enable paging */
        movl $pgd, %eax
        movl %eax, %cr3
        movl %cr0, %eax /* need to set bit 31 of CR0 - see 3-18 in Manual vol 3 */
        orl $0x80000000, %eax
        movl %eax, %cr0

        /* Manual vol 3 pg 8-14: Need a far jump after initializing CR0 */
        ljmp $0x08,$1f /* Seg selector: index=1,TI=0,RPL=00 for Kernel CS
                                        AT&T syntax req'res comma-separated offset */
1:              

        /* Setup remaining segment selectors */
        movw $0x10, %ax /* Set DS-SS to refer to KERNEL DS */
        movw %ax, %ds
        movw %ax, %es
        movw %ax, %fs
        movw %ax, %gs
        movw %ax, %ss

        /* no LDT */
        xorw %ax, %ax
        lldt %ax

        /* set up IDT by constructing 64-bit interrupt descriptors */
        movl $KERN_IDT, %edi
        movl $0x100, %ecx /* Set counter for 256 IDT entries */
        movl $0x080000,%edx /* Used to index KERNEL CS */
        movl $int_table,%esi

        /* For now populate int_table w/ interrupt gate descriptors.
         * These clear the IF flag when invoked
         */
1:      
        lodsl
        movw %ax,%dx
        movw $0xEE00,%ax /* DPL = 11 -- is that right? */
        movl %edx,(%edi) /* Store bottom 32-bits of descriptor in EDX into IDT */
        addl $4,%edi
        stosl /* Store top 32-bits of descriptor held in EAX into IDT */
        loop 1b
        
        lidt idt_ptr

        popl %ebx
        movl $stack+0x1000, %esp      /* setup real C stack */
        pushl %ebx                      /* pass GRUB multiboot info */
        call init /* start of C code */

int_table:
        /* interrupts 0x00-0x1F - processor traps */
        .long interrupt0
        .long interrupt1
        .long interrupt2
        .long interrupt3
        .long interrupt4
        .long interrupt5
        .long interrupt6
        .long interrupt7
        .long interrupt8
        .long interrupt9
        .long interrupta
        .long interruptb
        .long interruptc
        .long interruptd
        .long interrupte
        .long interruptIGN      /* spurious interrupt */
        .long interrupt10
        .long interrupt11
        .long interrupt12
        .long interrupt13
        .long interrupt14
        .long interrupt15
        .long interrupt16
        .long interrupt17
        .long interrupt18
        .long interrupt19
        .long interrupt1a
        .long interrupt1b
        .long interrupt1c
        .long interrupt1d
        .long interrupt1e
        .long interrupt1f

        /* interrupts 0x20-0x2F - PIC/IRQ interrupt requests */ 
        .long timer             /* PIT */
        .long interrupt21
        .long interrupt22
        .long interrupt23
        .long interrupt24
        .long soundcard         /* soundcard --??-- in future, 
                                   don't hardcode IRQ of soundcard */
        .long interrupt26
        .long interrupt27
        .long interrupt28
        .long interrupt29
        .long interrupt2a
        .long interrupt2b
        .long interrupt2c
        .long interrupt2d
        .long interrupt2e      /* not sure why 2E gets triggered sometimes */
        .long interrupt2f

        /* interrupts 0x30-0x3F - system calls */
        .long syscall0          /* putchar style call to video server */
        .long syscall1          /* fork */
        .long syscall2          /* switch_to -- co-routine-style context-switching */
        .long syscall3          /* exec */
        .long syscall4          /* getchar style call to keyboard driver */
        .long syscall5          /* open */
        .long syscall6          /* read */
        .long syscall7          /* uname */
        .long syscall8          /* meminfo */
        .long syscall9          /* time */
        .long syscalla          /* _exit */
        .long syscallb          /* waitpid */
        .long syscallc          /* sched_setparam -- not totally POSIX compliant */
        .long interrupt3d
        .long interrupt3e
        .long interrupt3f
#define INT(n)     .long interrupt##n
INT(40)
INT(41)
INT(42)
INT(43)
INT(44)
INT(45)
INT(46)
INT(47)
INT(48)
INT(49)
INT(4a)
INT(4b)
INT(4c)
INT(4d)
INT(4e)
INT(4f)
INT(50)
INT(51)
INT(52)
INT(53)
INT(54)
INT(55)
INT(56)
INT(57)
INT(58)
INT(59)
INT(5a)
INT(5b)
INT(5c)
INT(5d)
INT(5e)
INT(5f)
INT(60)
INT(61)
INT(62)
INT(63)
INT(64)
INT(65)
INT(66)
INT(67)
INT(68)
INT(69)
INT(6a)
INT(6b)
INT(6c)
INT(6d)
INT(6e)
INT(6f)
INT(70)
INT(71)
INT(72)
INT(73)
INT(74)
INT(75)
INT(76)
INT(77)
INT(78)
INT(79)
INT(7a)
INT(7b)
INT(7c)
INT(7d)
INT(7e)
INT(7f)
INT(80)
INT(81)
INT(82)
INT(83)
INT(84)
INT(85)
INT(86)
INT(87)
INT(88)
INT(89)
INT(8a)
INT(8b)
INT(8c)
INT(8d)
INT(8e)
INT(8f)
INT(90)
INT(91)
INT(92)
INT(93)
INT(94)
INT(95)
INT(96)
INT(97)
INT(98)
INT(99)
INT(9a)
INT(9b)
INT(9c)
INT(9d)
INT(9e)
INT(9f)
INT(a0)
INT(a1)
INT(a2)
INT(a3)
INT(a4)
INT(a5)
INT(a6)
INT(a7)
INT(a8)
INT(a9)
INT(aa)
INT(ab)
INT(ac)
INT(ad)
INT(ae)
INT(af)
INT(b0)
INT(b1)
INT(b2)
INT(b3)
INT(b4)
INT(b5)
INT(b6)
INT(b7)
INT(b8)
INT(b9)
INT(ba)
INT(bb)
INT(bc)
INT(bd)
INT(be)
INT(bf)
INT(c0)
INT(c1)
INT(c2)
INT(c3)
INT(c4)
INT(c5)
INT(c6)
INT(c7)
INT(c8)
INT(c9)
INT(ca)
INT(cb)
INT(cc)
INT(cd)
INT(ce)
INT(cf)
INT(d0)
INT(d1)
INT(d2)
INT(d3)
INT(d4)
INT(d5)
INT(d6)
INT(d7)
INT(d8)
INT(d9)
INT(da)
INT(db)
INT(dc)
INT(dd)
INT(de)
INT(df)
INT(e0)
INT(e1)
INT(e2)
INT(e3)
INT(e4)
INT(e5)
INT(e6)
INT(e7)
INT(e8)
INT(e9)
INT(ea)
INT(eb)
INT(ec)
INT(ed)
INT(ee)
INT(ef)
INT(f0)
INT(f1)
INT(f2)
INT(f3)
INT(f4)
INT(f5)
INT(f6)
INT(f7)
INT(f8)
INT(f9)
INT(fa)
INT(fb)
INT(fc)
INT(fd)
INT(fe)
INT(ff)
#undef INT

/* vi: set et sw=8 sts=8: */
