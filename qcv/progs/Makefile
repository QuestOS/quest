include ../../config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

LIBS   = $(LIB_DEST)/crt0.o -lc -lqcv
INCS   = 
CFLAGS = -g



PROGS = camera

#usb_test usb_test_rtt

.PHONY: all clean install

all: $(PROGS)

%: %.o
	$(LD)  -o $@ $< $(LIBS) -Map $@.map

%: %.c

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)
