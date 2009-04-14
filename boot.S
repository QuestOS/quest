/*
 * boot.S
 *
 *
 * NOTE:	 All references to Intel documentation are from the manuals 
 *               dated 1997-2001. Updated book versions may be on-line.		
 */


#include "kernel.h"

	.globl stack /* export kernel stack symbol */
	.globl initial_gdt
        
	.bss
	.align 0x1000 /* align page directory on 4K boundary - 2-12 Manual vol 3 */
	.comm pgd, 0x1000   /* setup 4Kbytes for page global directory */

	.align 0x1000 
	.comm stack, 0x1000 /* setup 4Kbytes for stack in bss */

	.align 0x1000
	.comm idt, 0x1000   /* setup 4Kbytes for IDT+GDT */

        .align 0x1000
        .comm lapic_pg_table, 0x1000  /* page table for mapping LAPIC */
        
	.align 0x1000 
	.comm kern_pg_table, 0x1000 /* setup 4Kbytes for kernel page table */

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

idt_ptr:
	.short 0x1FF     /* length - 64 descriptors */
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
	movl $stack+0x1000, %esp /* setup 4Kbyte stack */
	

        /* Intel Floating Pointer structure search */
        /* For gaining info on multiple processors */

        /* MP Signature is:  ('_' | ('M'<<8) | ('P'<<16) | ('_'<<24)) */

        /*    = 0x5F504D5F */
        movl $0x5F504D5F, %eax

        /* Try end of lower memory */
        /* Strictly, I should check CMOS to find bound */
        /* 0x9F800 = 638kb */

        movl $0x0009F800, %edi
        movl $0x00000800, %ecx
        shr $2, %ecx
1:      scasl
        je found_mb
        loop 1b

        /* Try Extended BIOS Data Area */
        movl $0x0000040E, %edi /* read base addr */
        movw (%edi), %di
        shl $4, %edi 
        movl $0x00000400, %ecx
        shr $2, %ecx
1:      scasl
        je found_mb
        loop 1b

        /* Try BIOS ROM address space */
        movl $0x000F0000, %edi
        movl $0x00010000, %ecx
        shr $2, %ecx
1:      scasl
        je found_mb
        loop 1b

        /* Not found - indicate uniprocessor */
        xorl %edi, %edi
        jmp mb_search_done

found_mb:
        addl $-4, %edi /* since scasl incremented */
mb_search_done:

        /* save MP floating pointer struct address */
        pushl %edi
	/* save multiboot parameter, for eventual call to C code */
	pushl %ebx

	/* initialise EFLAGS */
	pushl $2 /* Bit 1 always set to 1 in EFLAGS -- all other bits set to 0 */
	popf     /* See 2-8 Manual vol 3 */

        /* test APIC */
/*        movl $0xFEE00280, %edi
        xorl %eax, %eax
        movl %eax, (%edi) */
        
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
1:	stosl
	addl $0x1000, %eax
	loop 1b

	orl $0x3, %eax /* present and read/write bits set in each page */
	movl $_readwrite_pages, %ecx
1:	stosl
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
	ljmp $0x08,$1f /* Seg selector:	index=1,TI=0,RPL=00 for Kernel CS
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
	movl $0x40, %ecx /* Set counter for 64 IDT entries */
	movl $0x080000,%edx /* Used to index KERNEL CS */
	movl $int_table,%esi

	/* For now populate int_table w/ interrupt gate descriptors.
	 * These clear the IF flag when invoked
	 */
1:	
	lodsl
	movw %ax,%dx
	movw $0xEE00,%ax 
	movl %edx,(%edi) /* Store bottom 32-bits of descriptor in EDX into IDT */
	addl $4,%edi
	stosl /* Store top 32-bits of descriptor held in EAX into IDT */
	loop 1b
	
	lidt idt_ptr

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
	.long interruptf
	.long interrupt10
	.long interrupt11
	.long interrupt12
	.long interrupt13
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt

	/* interrupts 0x20-0x2F - PIC/IRQ interrupt requests */	
	.long timer		/* PIT */
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long soundcard		/* soundcard --??-- in future, 
				   don't hardcode IRQ of soundcard */
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt
	.long interrupt

	/* interrupts 0x30-0x3F - system calls */
	.long syscall0		/* putchar style call to video server */
	.long syscall1		/* fork */
	.long syscall2		/* switch_to -- co-routine-style context-switching */
	.long syscall3		/* exec */
	.long syscall4		/* getchar style call to keyboard driver */
	.long syscall5		/* open */
	.long syscall6		/* read */
	.long syscall7		/* uname */
	.long syscall8		/* meminfo */
	.long syscall9		/* time */
	.long syscalla		/* _exit */
	.long syscallb		/* waitpid */
	.long syscallc		/* sched_setparam -- not totally POSIX compliant */
	.long interrupt
	.long interrupt
	.long interrupt

        .code16
        .text
/*        .section .ap_startup */
        .globl patch_code_start
        .globl patch_code_end
patch_code_start:
/*
        movw %cs, %ax
        movw %ax, %ds
        movw %ax, %es
        movw %ax, %ss
        */
        
        /* Write 'A' to screen */
        movw $0x0741, %ax
        movw $0xB800, %dx
        movw %dx, %ds
        xorw %dx, %dx
        /* Write 1 to status indicator */
        movw %ax, %ds:(%edx) 
        movw $0x1, %ax
        movw $0x6FFF, %dx
        movw %dx, %ds
        movw $0x000B, %dx
        movw %ax, %ds:(%edx)

        movw $0x7000, %dx
        movw %dx, %ds
        
/*        movl $0x70200, %eax
        lgdt gdt_ptr - patch_code_start */

        lgdtl patch_gdt_ptr - patch_code_start /* ds segment */

        xorw %dx, %dx
        movw %dx, %ds

	movl %cr4, %eax /* EAX is temporary for CR4 */
	orl $0x10, %eax /* Set PSE bit of CR4 */
	movl %eax, %cr4

	movl $pgd, %eax
	movl %eax, %cr3
	movl %cr0, %eax /* need to set bit 31 of CR0 - see 3-18 in Manual vol 3 */
	orl $0x80000001, %eax
	movl %eax, %cr0 /* and enter protected mode */

        /* flush icache */
        ljmpl $0x08, $0x70051   /* ugh, hardcoded for now */

        .code32
pmode:  
	movw $0x10, %ax /* Set DS-SS to refer to KERNEL DS */
	movw %ax, %ds
	movw %ax, %es
	movw %ax, %fs
	movw %ax, %gs
	movw %ax, %ss
                
	xorw %ax, %ax
	lldt %ax        

        movw $0x0742, %ax
        movl $0xB8002, %edx
        movw %ax, (%edx)
        
        cli
1:      hlt
        jmp 1b

patch_gdt_ptr:  
	.short 0x7FF     /* length in bytes - 256 descriptors */
	.long  KERN_GDT   /* linear address */
patch_code_end: 
