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

        /* Software task-switch */
        /* %esi: source TSS pointer
         * %edi: destination TSS pointer */
        .globl _sw_jmp_task
_sw_jmp_task:
        /* check if source is NULL */
        testl %esi, %esi
        jz 1f

        /* save EIP */
        call 2f
        /* resume point */
        ret
2:      popl %eax
        movl %eax, 32(%esi)

        /* save EBP */
        movl %ebp, 60(%esi)

        /* save EFLAGS */
        pushf
        popl %eax
        movl %eax, 36(%esi)

        /* save CR3 */
        movl %cr3, %ebp
        movl %ebp, 28(%esi)

        /* save stack */
        movl %esp, %ebp
        movl %ebp, 56(%esi)

        /* -------------------- */
1:
        /* restore CR3 */
        movl 28(%edi), %eax
        movl %eax, %cr3

        /* restore stack */
        movl 56(%edi), %ebp
        movl %ebp, %esp

        /* restore EBP */
        movl 60(%edi), %ebp

        /* restore flags */
        movl 36(%edi), %eax
        pushl %eax
        popf

        /* restore EIP */
        leal 32(%edi), %eax
        jmp *(%eax)

        /* Start initial task */
        /* %edi: TSS pointer */
        .globl _sw_init_task
_sw_init_task:
        /* setup EFLAGS */
        movl 36(%edi), %eax
        pushl %eax
        popf

        /* setup stack */
        movl 56(%edi), %ebp
        movl %ebp, %esp

        /* setup CR3 */
        movl 28(%edi), %eax
        movl %eax, %cr3

        /* invoke task */
        leal 32(%edi), %eax
        jmp *(%eax)

        /* Start initial task in userspace */
        /* %edi: TSS pointer */
        .globl _sw_init_user_task
_sw_init_user_task:
        subl $20, %esp

        /* setup CR3 */
        movl 28(%edi), %eax
        movl %eax, %cr3

        /* setup stack */
        movl $0x23, 16(%esp)
        movl 56(%edi), %ebp
        movl %ebp, 12(%esp)

        /* setup EFLAGS */
        movl 36(%edi), %eax
        movl %eax, 8(%esp)

        /* setup CS:EIP */
        movl $0x1B, 4(%esp)
        movl 32(%edi), %eax
        movl %eax, 0(%esp)

        /* setup data segments */
        movl $0x23, %eax
        movw %ax, %ds
        movw %ax, %es
        movw %ax, %fs
        movw %ax, %gs

        /* so programs can assume zeroed registers */
        xorl %eax, %eax
        xorl %ebx, %ebx
        xorl %ecx, %ecx
        xorl %edx, %edx
        xorl %esi, %esi
        xorl %edi, %edi

        iret

/*
 * Local Variables:
 * mode: asm
 * comment-start: "\/*"
 * comment-end: "*\/"
 * indent-tabs-mode: nil
 * End:
 */

/* vi: set et sw=8 sts=8: */
