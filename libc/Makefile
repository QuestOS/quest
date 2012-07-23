CFLAGS    = -m32 -nostdinc -I../libc/include -msoft-float -L../softfloat -lsoftfloat -I../kernel -fno-builtin -fno-stack-protector -g
OBJS = src/stdio.o src/socket.o src/inet.o src/select.o src/math.o

all: libc.a

libc.a : $(OBJS)
	ar rcs $@ $(OBJS)

clean :
	rm -f $(OBJS) src/*~ include/*~ libc.a
