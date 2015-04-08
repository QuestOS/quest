include ../../config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

LIBS   = $(LIB_DEST)/crt0.o -lc
INCS   = -I../../libarduino/
CFLAGS = -Os -Wall -Werror

SKETCH_MAIN = ../../libarduino/main.c
MODULES := multithreading time digital standard_arduino interrupt \
	temporal_isolation demo
SRC_DIR := $(MODULES)
BUILD_DIR := $(MODULES)
SRC := $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.c))
PROGS := $(patsubst %.c,%,$(SRC))

vpath %.c $(SRC_DIR)

define make-goal
$1/%: %.c $(SKETCH_MAIN) $(LIB_DEST)/libc.a
	$(CC) $(INCS) $(CFLAGS) $$< $(SKETCH_MAIN) -o $$@
endef

.PHONY: all clean install

all: $(PROGS)

clean:
	rm -f $(PROGS)

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)

$(foreach bdir,$(BUILD_DIR),$(eval $(call make-goal,$(bdir))))
