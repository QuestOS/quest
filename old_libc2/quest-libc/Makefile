CFLAGS    = -nostdinc -I../include -msoft-float  -fno-builtin -fno-stack-protector -g
OBJS =  src/socket.o src/inet.o src/select.o src/syscall.o 
CC = i586-elf-gcc

all: quest-libc.a

quest-libc.a : $(OBJS)
	i586-elf-ar rcs $@ $(OBJS)

clean :
	rm -f $(OBJS) src/*~ include/*~ quest-libc.a *~

install:
	@echo "Nothing to install for libc"
