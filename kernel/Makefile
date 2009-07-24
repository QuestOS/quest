GRUBLIB = /usr/lib/grub/*-pc/
CC = gcc
ATSCC = atscc 
ATSOPT = atsopt
CFLAGS = -m32 -fno-builtin -nostdinc -Iacpi/include -I. -I../libc/include -g -Wall $(EXTRA_CFLAGS)
CPPFLAGS = -D_QUEST -Wa,--32 -MMD 
ATSFLAGS = -I. -Iacpi/include -I../libc/include -g -D_ATS_PRELUDE_NONE -D_ATS_DYNLOADFUN_NONE -D_ATS_STALOADFUN_NONE -DATS_TYPES_H -DATS_EXCEPTION_H -DATS_MEMORY_H -D_QUEST
INDENT = indent
INDENTFLAGS = -gnu -br -ce --no-tabs
OBJS = boot.o init.o boot-smp.o smp.o interrupt.o interrupt_handler.o \
	kernel.o diskio.o cpuid.o \
	fsys.o fsys_ext2fs.o fsys_iso9660.o sched.o sound.o \
	quest-acpica.o printf.o mem.o ata.o 
ACPI_SRCS = $(shell ls acpi/*.c)
ACPI_OBJS = $(patsubst %.c,%.o,$(ACPI_SRCS))
MNT_POINT=/mnt/quest
PROGS = quest terminal_server shell exec race test1 test2 test3 test4 test5 test6

.PHONY: all tags clean cleanacpi vmware cleanall

all: $(PROGS)

quest: $(OBJS) $(ACPI_OBJS)
	rm -f quest
	$(LD) -Map quest.map -T quest.ld -o $@ $^ 

%: %.o
	$(LD) -o $@ $< -T module.ld -Map $@.map

%: %.c

%.o: %.dats
	$(ATSCC) $(ATSFLAGS) -fno-builtin -m32 -o $@ -c $^

install: $(PROGS)
	cp $(PROGS) $(MNT_POINT)/boot
	sync

vmware: install
	qemu-img convert c.img -O vmdk c-img.vmdk
	cp c-img.vmdk /mnt/hgfs/shared/quest

quest.iso: quest $(PROGS)
	mkdir -p iso/boot/grub 
	cp $(GRUBLIB)/stage2_eltorito iso/boot/grub/ 
	cp iso-menu.lst iso/boot/grub/menu.lst 
	cp $(PROGS) iso/boot/ 
	mkisofs -quiet $(MSINFO) \
		-R -b boot/grub/stage2_eltorito \
		-no-emul-boot -boot-load-size 4 \
		-boot-info-table -o $@ iso

indent:
	$(INDENT) $(INDENTFLAGS) *.c *.h

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
