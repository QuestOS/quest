# *** Choose your compiler. Make it ANSI-compatible (e.g. 'cc -ansi').
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

LIBS   = $(TOOLCHAIN-INSTALL-DIR)/$(TARGET)/lib/crt0.o -lc
INCS   = 
CFLAGS = -g



PROGS = exec migtest race test1 test2 test3 test4 test5 \
	test6 test7 usb_test usb_test_rtt float_test

.PHONY: all clean install

all: $(PROGS)

%: %.o
	$(LD)  -o $@ $< $(LIBS) -Map $@.map

%: %.c

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)
