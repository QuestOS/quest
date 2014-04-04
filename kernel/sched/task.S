/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
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

        /* segments */
#define USER_CS 0x1B
#define USER_DS 0x23

        /* offsets into TSS/TCB */
#define initial_EIP 8
#define EBP 4
#define EFLAGS 16
#define ESP 0
#define CR3 12
#define FPU_STATE 32

        /* Software task-switch */
        /* %esi: source TSS pointer
         * %edi: destination TSS pointer */
        .globl _sw_jmp_task
_sw_jmp_task:
        /* check if source is NULL */
        testl %esi, %esi
        jz srcNULL

        /* save EBP */
        movl %ebp, EBP(%esi)

        /* save EFLAGS */
        pushf
        popl %ebx
        movl %ebx, EFLAGS(%esi)

        /* Safe FPU state */
#ifdef NO_FPU
        /* Do nothing */
#else
        fxsave FPU_STATE(%esi)
#endif
        
        
        /* save stack */
        movl %esp, %ebp
        movl %ebp, ESP(%esi)

        jmp restore

        /* force CR3 load if source is NULL */
srcNULL:
        movl CR3(%edi), %ebx
        jmp stoCR3

        /* -------------------- */
        /* restore destination TSS */
restore:
        /* restore CR3 if necessary */
        movl CR3(%edi), %ebx

        /* skip if kernel thread */
        movl $pgd, %edx
        cmpl %ebx, %edx
        je skipCR3

        /* skip if already loaded */
        movl %cr3, %edx
        cmpl %ebx, %edx
        je skipCR3

stoCR3: movl %ebx, %cr3
skipCR3:

        /* restore stack */
        movl ESP(%edi), %ebp
        movl %ebp, %esp

#ifdef NO_FPU
        /* Do nothing */
#else
        fxrstor FPU_STATE(%edi)
#endif
        
        /* restore EBP */
        movl EBP(%edi), %ebp

        /* restore EFLAGS */
        movl EFLAGS(%edi), %ebx
        pushl %ebx
        popf

        ret


        /* Start initial task */
        /* %edi: TSS pointer */
        .globl _sw_init_task
_sw_init_task:
        /* setup EFLAGS */
        movl EFLAGS(%edi), %ebx
        pushl %ebx
        popf

        /* setup stack */
        movl ESP(%edi), %ebp
        movl %ebp, %esp

        /* setup CR3 */
        movl CR3(%edi), %ebx
        movl %ebx, %cr3

        /* invoke task */
        ret

        /* Start initial task in userspace */
        /* %edi: TSS pointer */
        .globl _sw_init_user_task
_sw_init_user_task:
        subl $20, %esp

        /* setup CR3 */
        movl CR3(%edi), %ebx
        movl %ebx, %cr3

        /* setup stack */
        movl $USER_DS, 16(%esp)
        movl ESP(%edi), %ebp
        movl %ebp, 12(%esp)

        /* setup EFLAGS */
        movl EFLAGS(%edi), %ebx
        movl %ebx, 8(%esp)

        /* setup CS:EIP */
        movl $USER_CS, 4(%esp)
        movl initial_EIP(%edi), %ebx
        movl %ebx, 0(%esp)

        /* setup data segments */
        movl $USER_DS, %ebx
        movw %bx, %ds
        movw %bx, %es
        movw %bx, %fs
        movw %bx, %gs

        /* so programs can assume zeroed registers */
        xorl %eax, %eax
        xorl %ebx, %ebx
        xorl %ecx, %ecx
        xorl %edx, %edx
        xorl %esi, %esi
        xorl %edi, %edi

        iret


        /* Fast-path IPC: pass registers EAX, EDX */
        /* %esi: source TSS pointer
         * %edi: destination TSS pointer */
        .globl _sw_ipc
_sw_ipc:
        /* save EBP */
        movl %ebp, EBP(%esi)

        /* save EFLAGS */
        pushf
        popl %ebx
        movl %ebx, EFLAGS(%esi)

        /* save stack */
        movl %esp, %ebp
        movl %ebp, ESP(%esi)

        /* restore CR3 if necessary */
        movl CR3(%edi), %ebx

        /* skip if kernel thread */
        movl $pgd, %ebp
        cmpl %ebx, %ebp
        je 2f

        /* skip if already loaded */
        movl %cr3, %ebp
        cmpl %ebx, %ebp
        je 2f

        movl %ebx, %cr3
2:

        /* restore stack */
        movl ESP(%edi), %ebp
        movl %ebp, %esp

        /* restore EBP */
        movl EBP(%edi), %ebp

        /* restore EFLAGS */
        movl EFLAGS(%edi), %ebx
        pushl %ebx
        popf

        ret

/*
 * Local Variables:
 * mode: asm
 * comment-start: "\/*"
 * comment-end: "*\/"
 * indent-tabs-mode: nil
 * End:
 */

/* vi: set et sw=8 sts=8: */
