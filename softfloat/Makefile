CFLAGS    = -m32 -nostdinc -I../libc/include -I./include -I../kernel -msoft-float -fno-builtin -fno-stack-protector -g
OBJS = src/softfloat.o src/helper.o

AR_FILE = libsoftfloat.a

all: $(AR_FILE)

$(AR_FILE) : $(OBJS)
	ar rcs $@ $(OBJS)

clean :
	rm -f *~ $(OBJS) src/*~ include/*~ $(AR_FILE)

install:
	@echo "Nothing to install for softfloat"
