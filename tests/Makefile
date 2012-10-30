# *** Choose your compiler. Make it ANSI-compatible (e.g. 'cc -ansi').
CC	= i586-pc-quest-gcc
LD	= i586-pc-quest-ld 

ifndef INSTALL_DIR
INSTALL_DIR=/tftp
endif

ifndef INSTALL_CMD
INSTALL_CMD = cp
endif

LIBS   =
INCS   = 
CFLAGS =


PROGS = exec migtest race test1 test2 test3 test4 test5 \
	test6 test7 usb_test usb_test_rtt

.PHONY: all clean install

#test2.o : test2.c
#	$(CC) test2.c -c -o test2.o

all: $(PROGS)

%: %.o
	$(LD)  -o $@ $< /home/missimer/bin/cross/i586-pc-quest/lib/crt0.o -lc -Map $@.map

%: %.c

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)/boot
