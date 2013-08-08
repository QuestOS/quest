#### Makefile for Z80Em
# Following #defines can be used:
# DEBUG			Compile with builtin debugger
# LSB_FIRST		Compile for a low-endian machine
# X86_ASM		Use inline 80x86 assembly (GCC only)

CC	= gcc	# C compiler used
LD	= gcc	# Linker used

CFLAGS	= -Wall -O2 -fomit-frame-pointer -DLSB_FIRST -DDEBUG

all:    Z80.o Z80Debug.o z80dasm

Z80Dasm.o:  Z80Dasm.c
z80dasm:    Z80Dasm.o
	$(LD) -s -o z80dasm Z80Dasm.o

Z80Debug.o: Z80Debug.c Z80.h Z80IO.h Z80Dasm.h
Z80.o:      Z80.c Z80.h Z80Codes.h Z80IO.h Z80DAA.h
