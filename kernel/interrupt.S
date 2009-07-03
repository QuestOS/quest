/*
 *
 * interrupt.S
 *
 */

/* The movw $0x10...in SREGS_SAVE forces subsequent operation on the 
	kernel data segment by loading the appropriate segment registers */
#define SREGS_SAVE \
	pushw %ds; \
	pushw %es; \
	pushw %fs; \
	pushw %gs; \
		  \
	pushl %edx; \
	movw $0x10, %dx; \
	movw %dx, %ds; \
	movw %dx, %es; \
	movw %dx, %fs; \
	movw %dx, %gs; \
			\
	popl %edx

#define REGS_RESTORE \
	pushfl; \
	popl %edx; /* Get eflags into edx and clear NT flag */ \
	andl $0xFFFFBFFF, %edx; \
	pushl %edx; \
	popfl; \
			\
	popw %gs; \
	popw %fs; \
	popw %es; \
	popw %ds; 

#define SREGS_RESTORE \
	REGS_RESTORE \
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
        .globl keyboard
	.globl soundcard
	
	/* FIXME these are temporary debugging aids... ultimately these
	interrupts should be handled by task gates to the process server,
	and this code will go away */		
interrupt0:
	pushl $0
	pushl $0
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 


interrupt1:
	pushl $0
	pushl $1
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 
	
interrupt2:
	pushl $0
	pushl $2
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interrupt3:
	pushl $0
	pushl $3
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interrupt4:
	pushl $0
	pushl $4
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interrupt5:
	pushl $0
	pushl $5
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 


interrupt6:
	pushl $0
	pushl $6
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interrupt7:
	pushl $0
	pushl $7
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interrupt8:
	pushl $8
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interrupt9:
	pushl $0
	pushl $9
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interrupta:
	pushl $0xA
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interruptb:
	pushl $0xB
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interruptc:
	pushl $0xC
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interruptd:
	pushl $0xD
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interrupte:
	pushl $0xE
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 

interruptf:
	pushl $0
	pushl $0xF
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret

	
interrupt10:
	pushl $0
	pushl $0x10
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret

interrupt11:
	pushl $0x11
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 
	
interrupt12:
	pushl $0
	pushl $0x12
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 
	
interrupt13:
	pushl $0
	pushl $0x13
	SREGS_SAVE
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret 			
	

/* Need to setup syscall stubs */
/* syscall0 is a putchar style call to a video server */ 
syscall0:
	SREGS_SAVE
	
	pushl %ebx
	pushl %eax
	call HandleSyscall0
	addl $8, %esp

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

	call _getchar
	
	SREGS_RESTORE

/* open */	
syscall5:
	SREGS_SAVE

	pushl %ebx		/* flags */
	pushl %eax 		/* pathname */
	call _open
	addl $8, %esp	
	
	SREGS_RESTORE		
	
	
/* read */	
syscall6:
	SREGS_SAVE

	pushl %ecx		/* byte count */
	pushl %ebx		/* buf */
	pushl %eax 		/* pathname */
	call _read
	addl $12, %esp	
	
	SREGS_RESTORE

/* uname */	
syscall7:
	SREGS_SAVE

	pushl %eax		/* address of resultant string in user-space */
	call _uname
	addl $4, %esp
	
	SREGS_RESTORE

/* meminfo */	
syscall8:
	SREGS_SAVE

        pushl %edx
	pushl %eax
	call _meminfo
	addl $8, %esp
	
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
	addl $4, %esp		/* Shouldn't really get here... */
	
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
	addl $8, %esp		
	
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

/* IRQ1 - keyboard (bochs behavior is different from qemu, atm) */
keyboard:
        pushal
        SREGS_SAVE

        call _keyboard

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
	call HandleInterrupt
	REGS_RESTORE
	addl $8, %esp
	iret

        .globl interruptIGN
interruptIGN:
	iret

        .global interrupt3f
interrupt3f:
        pushal
	SREGS_SAVE
	call _interrupt3f
	REGS_RESTORE
        popal
        iret
        
#define INT(n)                \
        .globl interrupt##n;  \
interrupt##n:                 \
	pushl $0;             \
	pushl $0x##n;         \
	SREGS_SAVE;           \
	call HandleInterrupt; \
	REGS_RESTORE;         \
	addl $8, %esp;        \
	iret

INT(14)
INT(15)
INT(16)
INT(17)
INT(18)
INT(19)
INT(1a)
INT(1b)
INT(1c)
INT(1d)
INT(1e)
INT(1f)
INT(20)
INT(21)
INT(22)
INT(23)
INT(24)
INT(25)
INT(26)
INT(27)
INT(28)
INT(29)
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
INT(3e)
