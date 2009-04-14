CC = gcc
CFLAGS = -m32 -fno-builtin -nostdinc -I. -I../libc/include -g -Wall -O0
CPPFLAGS = -Wa,--32 -MMD 
OBJS = boot.o init.o smp.o interrupt.o interrupt_handler.o kernel.o diskio.o \
	fsys_ext2fs.o sched.o sound.o
MNT_POINT=/mnt/quest
PROGS = quest terminal_server shell exec race test

all: $(PROGS)

quest: $(OBJS)
	$(LD) -T quest.ld -o $@ $^

%: %.o
	$(LD) -o $@ $< -T module.ld

%: %.c

install: $(PROGS)
	cp $(PROGS) $(MNT_POINT)/boot
	sync

clean:
	-rm -f *.o *.d $(PROGS) *~

-include *.d
