CFLAGS    = -m32 -nostdinc -I../include -msoft-float  -fno-builtin -fno-stack-protector -g
OBJS =  src/socket.o src/inet.o src/select.o src/syscall.o 

all: quest-libc.a

quest-libc.a : $(OBJS)
	ar rcs $@ $(OBJS)

clean :
	rm -f $(OBJS) src/*~ include/*~ quest-libc.a *~

install:
	@echo "Nothing to install for libc"
