ifndef TARGET
TARGET = i586-pc-quest
endif

CC = $(TARGET)-gcc
LD = $(TARGET)-ld
AR = $(TARGET)-ar

LIB_DEST = $(TOOLCHAIN-INSTALL-DIR)/$(TARGET)/lib
INC_DEST = $(TOOLCHAIN-INSTALL-DIR)/$(TARGET)/include


ifndef TOOLCHAIN-INSTALL-DIR
TOOLCHAIN-INSTALL-DIR=$(HOME)/bin/cross
endif

CFLAGS    = -msoft-float -g -I./include
OBJS = src/softfloat.o src/helper.o

AR_FILE = libsoftfloat.a

all: $(LIB_DEST)/$(AR_FILE)


$(LIB_DEST)/$(AR_FILE): $(AR_FILE)
	cp $(AR_FILE) $(TOOLCHAIN-INSTALL-DIR)/$(TARGET)/lib/

$(AR_FILE): $(OBJS)
	$(AR) rcs $@ $(OBJS)

clean:
	rm -f *~ $(OBJS) src/*~ include/*~ $(AR_FILE)
	-rm $(TOOLCHAIN-INSTALL-DIR)/$(TARGET)/lib/$(AR_FILE)

install:
	@echo "libsoftfloat.a is installed via make all"
