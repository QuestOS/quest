include ../../config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

LIBS   = $(LIB_DEST)/crt0.o -lc
INCS   = -I../../libarduino/
CFLAGS =

SKETCH_MAIN = ../../libarduino/main.c

PROGS = multithread/multi_blink blink fade prime_sketch \
				gettimeofday_test digital_io_freq rdtsc_test \
				interrupt prime_single interrupt_freq prime_interrupt

.PHONY: all clean install

all: $(PROGS)


%: %.c $(SKETCH_MAIN) $(LIB_DEST)/libc.a
	$(CC) $(INCS) $(CFLAGS) $< $(SKETCH_MAIN) -o $@


clean:
	rm -f *.o *.d *.map *~ $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)
