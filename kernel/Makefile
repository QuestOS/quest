CC = gcc
CFLAGS = -m32 -fno-builtin -nostdinc -Iacpi/include -I. -I../libc/include -g -Wall 
CPPFLAGS = -D_QUEST -Wa,--32 -MMD 
OBJS = boot.o init.o boot-smp.o smp.o interrupt.o interrupt_handler.o \
	kernel.o diskio.o cpuid.o \
	fsys_ext2fs.o sched.o sound.o \
	quest-acpica.o printf.o mem.o
ACPI_SRCS = $(shell ls acpi/*.c)
ACPI_OBJS = $(patsubst %.c,%.o,$(ACPI_SRCS))
MNT_POINT=/mnt/quest
PROGS = quest terminal_server shell exec race test1 test2 test3 test4 test5 test6

.PHONY: all tags clean cleanacpi vmware cleanall

all: $(PROGS)

quest: $(OBJS) $(ACPI_OBJS)
	rm -f quest
	$(LD) -T quest.ld -o $@ $^ 

%: %.o
	$(LD) -o $@ $< -T module.ld

%: %.c

install: $(PROGS)
	cp $(PROGS) $(MNT_POINT)/boot
	sync

vmware: install
	qemu-img convert c.img -O vmdk c-img.vmdk
	cp c-img.vmdk /mnt/hgfs/shared/quest

tags:
	etags -o TAGS *.c acpi/*.c 
	ctags -o tags *.c acpi/*.c

clean:
	-rm -f *.o *.d $(PROGS) *~

cleanacpi:
	-rm -f $(ACPI_OBJS)

cleanall: clean cleanacpi

define ACPI_template
$(1): $(patsubst %.o,%.c,$(1))
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $$@ -c $$^
endef 

$(foreach obj,$(ACPI_OBJS),$(eval $(call ACPI_template,$(obj))))



-include *.d
