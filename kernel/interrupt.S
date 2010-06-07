/* -*- Mode: asm; comment-start: "\/*"; comment-end: "*\/"; indent-tabs-mode: nil -*- */

/*
 *
 * interrupt.S
 *
 */

/* The movw $0x10...in SREGS_SAVE forces subsequent operation on the
 * kernel data segment by loading the appropriate segment
 * registers */

#define SREGS_SAVE       \
        pushw %ds;       \
        pushw %es;       \
        pushw %fs;       \
        pushw %gs;       \
                         \
        pushl %edx;      \
        movw $0x10, %dx; \
        movw %dx, %ds;   \
        movw %dx, %es;   \
        movw %dx, %fs;   \
        movw %dx, %gs;   \
                         \
        popl %edx

#define REGS_RESTORE                                           \
        pushfl;                                                \
        popl %edx; /* Get eflags into edx and clear NT flag */ \
        andl $0xFFFFBFFF, %edx;                                \
        pushl %edx;                                            \
        popfl;                                                 \
                                                               \
        popw %gs;                                              \
        popw %fs;                                              \
        popw %es;                                              \
        popw %ds; 

#define SREGS_RESTORE \
        REGS_RESTORE  \
        iret;

        
        .text
        .globl interrupt0
        .globl interrupt1
        .globl interrupt2
        .globl interrupt3
        .globl interrupt4
        .globl interrupt5
        .globl interrupt6
        .globl interrupt7
        .globl interrupt8
        .globl interrupt9
        .globl interrupta
        .globl interruptb
        .globl interruptc
        .globl interruptd
        .globl interrupte
        .globl interruptf
        .globl interrupt10
        .globl interrupt11
        .globl interrupt12
        .globl interrupt13
        .globl interrupt
        .globl syscall0
        .globl syscall1
        .globl syscall2
        .globl syscall3
        .globl syscall4
        .globl syscall5
        .globl syscall6
        .globl syscall7
        .globl syscall8
        .globl syscall9
        .globl syscalla
        .globl syscallb
        .globl syscallc
        .globl timer
        .globl soundcard
        
        /* FIXME these are temporary debugging aids... ultimately these
        interrupts should be handled by task gates to the process server,
        and this code will go away */           
interrupt0:
        pushl $0
        pushl $0
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 


interrupt1:
        pushl $0
        pushl $1
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 
        
interrupt2:
        pushl $0
        pushl $2
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interrupt3:
        pushl $0
        pushl $3
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interrupt4:
        pushl $0
        pushl $4
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interrupt5:
        pushl $0
        pushl $5
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 


interrupt6:
        pushl $0
        pushl $6
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interrupt7:
        pushl $0
        pushl $7
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interrupt8:
        pushl $8
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interrupt9:
        pushl $0
        pushl $9
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interrupta:
        pushl $0xA
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interruptb:
        pushl $0xB
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interruptc:
        pushl $0xC
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interruptd:
        pushl $0xD
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interrupte:
        pushl $0xE
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 

interruptf:
        pushl $0
        pushl $0xF
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret

        
interrupt10:
        pushl $0
        pushl $0x10
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret

interrupt11:
        pushl $0x11
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 
        
interrupt12:
        pushl $0
        pushl $0x12
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret 
        
interrupt13:
        pushl $0
        pushl $0x13
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret                    

        
    /* Handler's Stack: (Figure 5-4, Intel Manual vol. 3)
     *
     * esp[6]: SS
     * esp[5]: ESP
     * esp[4]: EFLAGS
     * esp[3]: CS 
     * esp[2]: EIP 
     * esp[1]: ERROR CODE
     *
     */


/* Need to setup syscall stubs */
/* syscall0 is a putchar style call to a video server */ 
syscall0:
        SREGS_SAVE
        
        pushl %ebx
        pushl %eax
        call handle_syscall0
        popl  %eax
        popl  %ebx              /* preserve */

        SREGS_RESTORE

/* For now, this is the fork syscall - will move to a fork server */
syscall1:
        SREGS_SAVE
        pushl %esp  /* Will use this later to create a TSS with
                     * duplicate parent register values
                     * See DuplicateTSS() for more info
                     */
        

        pushl %ebp  /* Also needed to populate child TSS */
        call _fork
        addl $8, %esp
        
        SREGS_RESTORE

/* switch_to syscall supporting coroutines (voluntarily relinquishing CPU) */
syscall2:
        SREGS_SAVE

        pushl %eax /* Destination TSS argument */
        call _switch_to
        addl $4, %esp   
        
        SREGS_RESTORE


/* exec syscall wrapper */      
syscall3:
        SREGS_SAVE
        pushl %esp  /* 
                     *  Use this to patch up esp, eip for new program
                     */
        pushl %ebx
        pushl %eax
        call _exec
        addl $12, %esp
        
        SREGS_RESTORE

/* getchar from keyboard driver */      
syscall4:
        SREGS_SAVE

        pushl %ebx
        call _getchar
        popl %ebx
        
        SREGS_RESTORE

/* open */      
syscall5:
        SREGS_SAVE

        pushl %ebx              /* flags */
        pushl %eax              /* pathname */
        call _open
        addl $4, %esp
        popl %ebx               /* preserve */
        
        SREGS_RESTORE           
        
        
/* read */      
syscall6:
        SREGS_SAVE

        pushl %ecx              /* byte count */
        pushl %ebx              /* buf */
        pushl %eax              /* pathname */
        call _read
        addl $4, %esp
        popl %ebx
        popl %ecx               /* preserve */
        
        SREGS_RESTORE

/* uname */     
syscall7:
        SREGS_SAVE

        pushl %eax              /* address of resultant string in user-space */
        call _uname
        addl $4, %esp
        
        SREGS_RESTORE

/* meminfo */   
syscall8:
        SREGS_SAVE

        pushl %edx
        pushl %eax
        call _meminfo
        addl $4, %esp
        popl %edx               /* preserve */
        
        SREGS_RESTORE

/* time */      
syscall9:
        SREGS_SAVE

        call _time
        
        SREGS_RESTORE

/* _exit */     
syscalla:
        SREGS_SAVE

        pushl %eax
        call __exit
        addl $4, %esp           /* Shouldn't really get here... */
        
        SREGS_RESTORE

/* waitpid */   
syscallb:
        SREGS_SAVE

        pushl %eax
        call _waitpid
        addl $4, %esp                   
        
        SREGS_RESTORE


/* sched_setparam */    
syscallc:
        SREGS_SAVE

        pushl %ebx
        pushl %eax
        call _sched_setparam
        addl $4, %esp
        popl  %ebx              /* preserve */
        
        SREGS_RESTORE   

/* IRQ0 - system timer */
timer:  
        pushal
        SREGS_SAVE

        pushfl
        popl %edx
        andl $0x4000, %edx
        jne panic
        
        call _timer

        REGS_RESTORE
        popal

        iret
        
/* IRQ5 - soundcard --??-- in future, don't hardcode to specific IRQ */
soundcard:      
        pushal
        SREGS_SAVE

        pushfl
        popl %edx
        andl $0x4000, %edx
        jne panic
        
        call _soundcard

        REGS_RESTORE
        popal

        iret            
        
        
interrupt:
        pushl $0
        pushl $0xFFFFFFFF
        SREGS_SAVE
        call handle_interrupt
        REGS_RESTORE
        addl $8, %esp
        iret

        .globl interruptIGN
interruptIGN:
        iret

        .global interrupt29
interrupt29:
        pushal
        SREGS_SAVE
        call _interrupt29
        REGS_RESTORE
        popal
        iret
        
        .global interrupt3e
interrupt3e:
        pushal
        SREGS_SAVE
        call _interrupt3e
        REGS_RESTORE
        popal
        iret


#define EXC(n)                 \
        .globl interrupt##n;   \
interrupt##n:                  \
        pushl $0;              \
        pushl $0x##n;          \
        SREGS_SAVE;            \
        call handle_interrupt; \
        REGS_RESTORE;          \
        addl $8, %esp;         \
        iret

EXC(14)
EXC(15)
EXC(16)
EXC(17)
EXC(18)
EXC(19)
EXC(1a)
EXC(1b)
EXC(1c)
EXC(1d)
EXC(1e)
EXC(1f)
#undef EXC

#define INT(n)                \
        .globl interrupt##n;  \
interrupt##n:                 \
        pushal;               \
        SREGS_SAVE;           \
        pushl $0x##n;         \
        call dispatch_vector; \
        addl $4, %esp;        \
        REGS_RESTORE;         \
        popal;                \
        iret

INT(20)
INT(21)
INT(22)
INT(23)
INT(24)
INT(25)
INT(26)
INT(27)
INT(28)
INT(2a)
INT(2b)
INT(2c)
INT(2d)
INT(2e)
INT(2f)
INT(30)
INT(31)
INT(32)
INT(33)
INT(34)
INT(35)
INT(36)
INT(37)
INT(38)
INT(39)
INT(3a)
INT(3b)
INT(3c)
INT(3d)
INT(3f)

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
