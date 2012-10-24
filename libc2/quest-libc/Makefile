CFLAGS    = -m32 -nostdinc -I../include -I./include -I../newlib-1.20.0/newlib/libc/include/sys -I../newlib-1.20.0/newlib/libc/include/ -msoft-float -L../softfloat -lsoftfloat -fno-builtin -fno-stack-protector -g
OBJS =  src/socket.o src/inet.o src/select.o src/syscall.o 

all: quest-libc.a

quest-libc.a : $(OBJS)
	ar rcs $@ $(OBJS)

clean :
	rm -f $(OBJS) src/*~ include/*~ quest-libc.a *~

install:
	@echo "Nothing to install for libc"
