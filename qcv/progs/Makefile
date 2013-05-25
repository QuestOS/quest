include ../../config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

LIBS   = $(LIB_DEST)/crt0.o -lqcv -ljpeg -lc
INCS   = 
CFLAGS = -g



PROGS = camera img_mjpeg_dec

#usb_test usb_test_rtt

.PHONY: all clean install

all: $(PROGS)

%: %.o $(LIB_DEST)/libqcv.a
	$(LD)  -o $@ $< $(LIBS) -Map $@.map

%: %.c 

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)
	$(INSTALL_CMD) test.jpg $(INSTALL_DIR)
