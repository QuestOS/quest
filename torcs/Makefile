ifndef TARGET
TARGET = i586-pc-quest
endif

CC = $(TARGET)-gcc
LD = $(TARGET)-ld
AR = $(TARGET)-ar

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

ifndef TOOLCHAIN-INSTALL-DIR
TOOLCHAIN-INSTALL-DIR=$(HOME)/bin/cross
endif

ifndef INSTALL_CMD
INSTALL_CMD = cp
endif

LIBS   = -lc -lm -lsoftfloat $(TOOLCHAIN-INSTALL-DIR)/$(TARGET)/lib/crt0.o
INCS   = 
CFLAGS =  -g -Wall -O0 -MMD -msoft-float $(INCS)

OBJS = torcs.o

all:  torcs

torcs: $(OBJS) 
	$(LD) -o torcs $(OBJS) $(LIBS)


%.o: %.c
	$(CC)  $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o *.d *~ torcs

install: torcs
	$(INSTALL_CMD) torcs $(INSTALL_DIR)/boot
