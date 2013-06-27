include ../config.mk
include ../kernel/config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

LIBS   = $(LIB_DEST)/crt0.o -lc
INCS   = 
CFLAGS = $(CFG) -g


PROGS = shell iotest ipctest spinner tcpclient tcpserver  \
	udpclient udpserver webserver

.PHONY: all clean install

all: $(PROGS)

%: %.o
	$(LD)  -o $@ $< $(LIBS) -Map $@.map

%: %.c

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) index.html default.css quest.png $(INSTALL_DIR)

$(PROGS): ../kernel/config.mk
