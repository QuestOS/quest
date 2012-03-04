CFLAGS    = -m32 -nostdinc -I../libc/include -I../kernel -fno-builtin -fno-stack-protector -g
OBJS = src/stdio.o src/socket.o src/inet.o src/select.o

all: libc.a

libc.a : $(OBJS)
	ar rcs $@ $(OBJS)

clean :
	rm -f $(OBJS) src/*~ include/*~ libc.a
