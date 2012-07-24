/* -*- Mode: asm; comment-start: "\/*"; comment-end: "*\/"; indent-tabs-mode: nil -*- */
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

        .section .text16, "ax"
        .align 0x1000
        .code16
        .globl vmx_code16_entry
        .type vmx_code16_entry,@function
vmx_code16_entry:
        push %bp
        movw %sp, %bp
test_outw:
        movw $0x8A00, %dx
        movw $0x8A00, %ax
        outw %ax, %dx
test_outsw:
        movw %cs, %ax
        movw %ax, %ds
        leaw testdata, %si
        movw $3, %cx
        cld
        rep outsw
        pushf
        popf
video_mode_13h:
        movw $0x13, %ax
        int $0x10
drawtest:
        movw $0xA000, %di
        movw %di, %es
        xorw %di, %di
        movw $0x6, %cx
2:
        movw %cx, %dx
        movw $0xF, %cx
1:
        movw %cx, %ax
        movw $640, %cx
        rep stosb
        movw %ax, %cx
        loop 1b
        movw %dx, %cx
        loop 2b

        movw $0xFFFF, %cx
3:
        movw $0x3C8, %dx
        movb %cl, %al
        andb $0x0F, %al
        outb %al, %dx
        movw $0x3C9, %dx
        movb %cl, %al
        outb %al, %dx
        movb %ch, %al
        outb %al, %dx
        movb %ch, %al
        outb %al, %dx
        loop 3b

finish:
        movw %bp, %sp
        pop %bp
        hlt
        jmp vmx_code16_entry
        .size vmx_code16_entry,.-vmx_code16_entry
testdata:
        .word 0x8A00
        .word 0x8A00
        .word 0x8A00

/* vi: set et sw=8 sts=8: */
