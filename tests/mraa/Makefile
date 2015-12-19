include ../../config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

LIBS   = $(LIB_DEST)/crt0.o -lc
INCS   = -I../../libmraa
CFLAGS = -Wall 

PROGS = i2c_test gpio_test

.PHONY: all clean install

all: $(PROGS)

%: %.c $(LIB_DEST)/libc.a
	$(CC) $(INCS) $(CFLAGS) $< -o $@

clean: 
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)
