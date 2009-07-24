GRUBLIB = /usr/lib/grub/*-pc/
CC = gcc
ATSCC = atscc 
ATSOPT = atsopt
CFLAGS = -m32 -fno-builtin -nostdinc -g -Wall $(EXTRA_CFLAGS)
CPPFLAGS = -D_QUEST -Wa,--32 -MMD -Iinclude -Iinclude/drivers/acpi -I. -I../libc/include 
ATSFLAGS = -I. -Iinclude -Iinclude/drivers/acpi -I../libc/include -g -D_ATS_PRELUDE_NONE -D_ATS_DYNLOADFUN_NONE -D_ATS_STALOADFUN_NONE -DATS_TYPES_H -DATS_EXCEPTION_H -DATS_MEMORY_H -D_QUEST
INDENT = indent
INDENTFLAGS = -gnu -br -ce --no-tabs

OBJS =  boot/boot.o boot/init.o kernel.o \
	interrupt.o interrupt_handler.o \
	smp/boot-smp.o smp/smp.o \
	sched/sched.o \
	mem/mem.o \
	util/cpuid.o util/printf.o \
	drivers/ata/ata.o drivers/ata/diskio.o \
	fs/fsys.o \
	fs/ext2/fsys_ext2fs.o \
	fs/iso9660/fsys_iso9660.o \
	drivers/sb16/sound.o \
	drivers/acpi/quest-acpica.o  

CFILES = $(patsubst %.o,%.c,$(OBJS))
DFILES = $(patsubst %.o,%.d,$(OBJS))
MAPFILES = $(patsubst %.o,%.map,$(OBJS))

ACPI_SRCS = $(shell ls drivers/acpica/*.c)
ACPI_OBJS = $(patsubst %.c,%.o,$(ACPI_SRCS))
MNT_POINT=/mnt/quest
PROGS = quest \
	sysprogs/terminal_server sysprogs/shell \
	tests/exec tests/race tests/test1 tests/test2 \
	tests/test3 tests/test4 tests/test5 tests/test6

.PHONY: all tags clean cleanacpi vmware cleanall indent

all: $(PROGS)

quest: $(OBJS) $(ACPI_OBJS)
	rm -f quest
	$(LD) -Map quest.map -T ldscripts/quest.ld -o $@ $^ 

%: %.o
	$(LD) -o $@ $< -T ldscripts/module.ld -Map $@.map

%: %.c

%.o: %.dats
	$(ATSCC) $(ATSFLAGS) -fno-builtin -m32 -o $@ -c $^

install: $(PROGS)
	cp $(PROGS) welcome.raw $(MNT_POINT)/boot
	sync

vmware: install
	qemu-img convert c.img -O vmdk c-img.vmdk
	cp c-img.vmdk /mnt/hgfs/shared/quest

quest.iso: quest $(PROGS)
	mkdir -p iso/boot/grub 
	cp $(GRUBLIB)/stage2_eltorito iso/boot/grub/ 
	cp iso-menu.lst iso/boot/grub/menu.lst 
	cp $(PROGS) welcome.raw iso/boot/ 
	mkisofs -quiet $(MSINFO) \
		-R -b boot/grub/stage2_eltorito \
		-no-emul-boot -boot-load-size 4 \
		-boot-info-table -o $@ iso

indent:
	(for f in $(CFILES); do [ -f "$$f" ] && $(INDENT) $(INDENTFLAGS) "$$f"; done)

tags:
	etags -o TAGS *.c acpi/*.c 
	ctags -o tags *.c acpi/*.c

clean:
	-rm -f $(OBJS) $(DFILES) $(MAPFILES) $(PROGS) quest.iso quest.map
	-rm -rf iso

cleanacpi:
	-rm -f $(ACPI_OBJS)

cleanall: clean cleanacpi

define ACPI_template
$(1): $(patsubst %.o,%.c,$(1))
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $$@ -c $$^
endef 

$(foreach obj,$(ACPI_OBJS),$(eval $(call ACPI_template,$(obj))))



-include *.d
