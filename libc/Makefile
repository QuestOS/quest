CFLAGS    = -m32 -nostdinc -I../libc/include -I../kernel -fno-builtin -g
OBJS = src/stdio.o

all: libc.a

libc.a : $(OBJS)
	ar rcs $@ $(OBJS)

clean :
	rm -f $(OBJS) src/*~ include/*~ libc.a
