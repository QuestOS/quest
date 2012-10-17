# *** Choose your compiler. Make it ANSI-compatible (e.g. 'cc -ansi').
include ../kernel/config.mk
CC	= gcc
LD	= ld -melf_i386 

ifndef INSTALL_DIR
INSTALL_DIR=/tftp
endif

ifndef INSTALL_CMD
INSTALL_CMD = cp
endif

LIBS   = -L../libc/ -L../libc/include -L../softfloat/ -lc -lsoftfloat
INCS   = -I../libc/include -I../quest/include
CFLAGS = -m32 -march=i586 -fno-builtin -fno-stack-protector -fno-strict-aliasing \
         -fno-delete-null-pointer-checks -nostdinc -g -Wall -Wno-attributes -O0 \
         -D_QUEST -Wa,--32 -MMD -msoft-float $(INCS) $(LIBS) $(CFG)


PROGS = shell iotest ipctest spinner tcpclient tcpserver  \
	udpclient udpserver webserver

.PHONY: all clean install


all: $(PROGS)

%: %.o
	$(LD) -o $@ $< -T../kernel/ldscripts/module.ld $(LIBS) -Map $@.map

%: %.c

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) index.html default.css quest.png $(INSTALL_DIR)/boot
