# *** Choose your compiler. Make it ANSI-compatible (e.g. 'cc -ansi').
include ../kernel/config.mk

ifndef TARGET
TARGET = i586-pc-quest
endif

CC = $(TARGET)-gcc
LD = $(TARGET)-ld
AR = $(TARGET)-ar

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

ifndef TOOLCHAIN-INSTALL-DIR
TOOLCHAIN-INSTALL-DIR=$(HOME)/bin/cross
endif

ifndef INSTALL_CMD
INSTALL_CMD = cp
endif

LIBS   = 
INCS   = 
CFLAGS = $(CFG)


PROGS = shell iotest ipctest spinner tcpclient tcpserver  \
	udpclient udpserver webserver

.PHONY: all clean install


all: $(PROGS)

%: %.o
	$(LD)  -o $@ $< $(TOOLCHAIN-INSTALL-DIR)/$(TARGET)/lib/crt0.o -lc -Map $@.map

%: %.c

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) index.html default.css quest.png $(INSTALL_DIR)
