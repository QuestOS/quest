# *** Choose your compiler. Make it ANSI-compatible (e.g. 'cc -ansi').
CC	= gcc
LD	= ld -melf_i386 

ifndef MNT_POINT
MNT_POINT=/tftp
endif


LIBS   = -L../libc/ -L../libc/include -L../softfloat/ -lc -lsoftfloat
INCS   = -I../libc/include -I../quest/include
CFLAGS = -m32 -march=i586 -fno-builtin -fno-stack-protector -fno-strict-aliasing \
         -fno-delete-null-pointer-checks -nostdinc -g -Wall -Wno-attributes -O0 \
         -D_QUEST -Wa,--32 -MMD -msoft-float $(INCS)


PROGS = exec migtest race test1 test2 test3 test4 test5 \
	test6 test7 usb_test

.PHONY: all clean install

all: $(PROGS)

%: %.o
	$(LD) -o $@ $< -T../kernel/ldscripts/module.ld -Map $@.map

%: %.c

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	cp $(PROGS) $(MNT_POINT)/boot
	sync
