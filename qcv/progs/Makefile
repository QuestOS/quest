include ../../config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

LIBS   = -lqcv -ljpeg -lavdevice -lavfilter -lavformat -lavcodec -lz -lswresample -lswscale -lavutil -lm
INCS   = 
CFLAGS = -g

EXTRA_FILES = test.jpg white.jpg black.jpg half.jpg test.mpg


PROGS = camera img_mjpeg_dec camera_canny img_canny mpeg_test

#usb_test usb_test_rtt

.PHONY: all clean install

all: $(PROGS)

%: %.c $(LIB_DEST)/libqcv.a
	$(CC) -o $@ $< $(LIBS)
	cp $@ $@-unstripped
	$(STRIP) $@

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)
	$(INSTALL_CMD) $(EXTRA_FILES) $(INSTALL_DIR)
