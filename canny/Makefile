include ../config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

.PHONY: all clean install

LIBS   = -lc -lm -lsoftfloat $(LIB_DEST)/crt0.o
INCS   = 
CFLAGS =  -g -Wall -O0 -MMD -msoft-float $(INCS)

OBJS = canny.o test.o

PROGS = canny

all:  $(PROGS)

canny: $(OBJS)
	$(LD) -o canny $(OBJS) $(LIBS)


%.o: %.c
	$(CC)  $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o *.d *~ canny

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)
