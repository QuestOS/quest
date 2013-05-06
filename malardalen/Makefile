include ../config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

LIBS   = $(LIB_DEST)/crt0.o -lc
INCS   = 
CFLAGS = -g -O2 -fno-builtin -fomit-frame-pointer



PROGS = adpcm bs bsort100 cnt compress cover crc duff \
	edn expint fac fdct fft1 fibcall fir insertsort \
	janne_complex jfdctint lcdnum lms ludcmp matmult \
	minver ndes ns nsichneu prime qsort-exam qurt \
	recursion select st ud

#usb_test usb_test_rtt

.PHONY: all clean install

all: $(PROGS)

st: sqrt.o st.o
	$(LD)  -o $@ st.o $(LIBS) sqrt.o -Map $@.map
%: %.o
	$(LD)  -o $@ $< $(LIBS) -Map $@.map

%: %.c

clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)
