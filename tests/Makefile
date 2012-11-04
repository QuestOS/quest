include ../config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

LIBS   = $(LIB_DEST)/crt0.o -lc
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
