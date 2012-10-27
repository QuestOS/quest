# *** Choose your compiler. Make it ANSI-compatible (e.g. 'cc -ansi').
CC	= gcc
LD	= ld -melf_i386 

ifndef INSTALL_DIR
INSTALL_DIR=/tftp
endif

ifndef INSTALL_CMD
INSTALL_CMD = cp
endif

LIBS   = -L../libc2/ -L../libc2/include -lc
INCS   = -I../libc2/include -I../quest/include -I../libc2/newlib-1.20.0/newlib/libc/include/
CFLAGS = -m32 -march=i586 -fno-builtin -fno-stack-protector -fno-strict-aliasing \
         -fno-delete-null-pointer-checks -nostdinc -g -Wall -Wno-attributes -O0 \
         -D_QUEST -Wa,--32 -MMD $(INCS)


PROGS = exec migtest race test1 test2 test3 test4 test5 \
	test6 test7 usb_test usb_test_rtt

.PHONY: all clean install

all: $(PROGS)

%: %.o
	$(LD) -o $@ $< -T../kernel/ldscripts/module.ld -Map $@.map $(LIBS)

%: %.c

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)/boot
