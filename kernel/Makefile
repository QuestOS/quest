GRUBLIB = grub/x86_64-pc
TFTPDIR=tftp
LD = ld -melf_i386
CC = gcc
ATSCC = atscc 
ATSOPT = atsopt
OPT = 0
CFLAGS = -m32 -fno-builtin -fno-stack-protector -fno-strict-aliasing -nostdinc -g -Wall -Wno-attributes -O$(OPT) $(EXTRA_CFLAGS)
CPPFLAGS = -D_QUEST -Wa,--32 -MMD -Iinclude -Iinclude/drivers/acpi -I. -I../libc/include 
ATSFLAGS = -IATS include -I. -Iinclude -Iinclude/drivers/acpi -I../libc/include -g -O$(OPT) -D_ATS_PRELUDE_NONE -D_ATS_DYNLOADFUN_NONE -D_ATS_STALOADFUN_NONE -DATS_TYPES_H -DATS_EXCEPTION_H -DATS_MEMORY_H -D_QUEST
INDENT = indent
INDENTFLAGS = -gnu -br -ce --no-tabs

##################################################

MNT_POINT=/mnt/quest
PROGS = quest \
	sysprogs/terminal_server sysprogs/shell \
	tests/exec tests/race tests/test1 tests/test2 \
	tests/test3 tests/test4 tests/test5 tests/test6

##################################################

OBJS =  boot/boot.o boot/init.o kernel.o \
	interrupt.o interrupt_handler.o \
	smp/boot-smp.o smp/smp.o smp/intel.o smp/acpi.o smp/apic.o \
	sched/sched.o \
	mem/physical.o mem/virtual.o mem/pow2.o \
	util/cpuid.o util/printf.o util/screen.o util/debug.o util/circular.o \
	drivers/ata/ata.o drivers/ata/diskio.o \
	drivers/input/keyboard_8042.o drivers/input/keymap.o \
	drivers/pci/pci.o \
	drivers/net/ethernetif.o drivers/net/pcnet.o \
	drivers/net/e1000.o drivers/net/e1000e.o \
	fs/fsys.o \
	fs/ext2/fsys_ext2fs.o \
	fs/iso9660/fsys_iso9660.o \
	drivers/sb16/sound.o \
	drivers/acpi/quest-acpica.o \
	drivers/usb/uhci_hcd.o drivers/usb/uvc.o drivers/usb/umsc.o \
	drivers/usb/hub.o \
	drivers/usb/usb_tests.o \
	kdb/i386-stub.o

CFILES = $(patsubst %.o,%.c,$(OBJS))
DFILES = $(patsubst %.o,%.d,$(OBJS))
MAPFILES = $(patsubst %.o,%.map,$(OBJS))

##################################################

ACPI_SRCS = $(shell ls drivers/acpica/*.c)
ACPI_OBJS = $(patsubst %.c,%.o,$(ACPI_SRCS))
ACPI_DEPS = $(patsubst %.c,%.d,$(ACPI_SRCS))

##################################################

LIBLWIP = lwip.a

LWIP_CFILES = \
  lwip/api/netbuf.c \
  lwip/api/api_msg.c \
  lwip/api/tcpip.c \
  lwip/api/sockets.c \
  lwip/api/netdb.c \
  lwip/api/netifapi.c \
  lwip/api/err.c \
  lwip/api/api_lib.c \
  lwip/core/dhcp.c \
  lwip/core/tcp_in.c \
  lwip/core/pbuf.c \
  lwip/core/netif.c \
  lwip/core/tcp.c \
  lwip/core/dns.c \
  lwip/core/stats.c \
  lwip/core/tcp_out.c \
  lwip/core/raw.c \
  lwip/core/sys.c \
  lwip/core/mem.c \
  lwip/core/ipv4/inet.c \
  lwip/core/ipv4/ip.c \
  lwip/core/ipv4/icmp.c \
  lwip/core/ipv4/ip_addr.c \
  lwip/core/ipv4/igmp.c \
  lwip/core/ipv4/autoip.c \
  lwip/core/ipv4/ip_frag.c \
  lwip/core/ipv4/inet_chksum.c \
  lwip/core/init.c \
  lwip/core/memp.c \
  lwip/core/udp.c \
  lwip/netif/ethernetif.c \
  lwip/netif/slipif.c \
  lwip/netif/loopif.c \
  lwip/netif/etharp.c

LWIP_OBJS = $(patsubst %.c,%.o,$(LWIP_CFILES))
LWIP_DEPS = $(patsubst %.c,%.d,$(LWIP_CFILES))

##################################################

.PHONY: all tags clean cleanacpi cleanlwip vmware cleanall indent netboot

all: $(PROGS)

quest: $(OBJS) $(ACPI_OBJS) $(LWIP_OBJS)
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

vmware: install quest.iso
	qemu-img convert c.img -O vmdk c-img.vmdk
	cp c-img.vmdk /mnt/hgfs/shared/quest/
	cp quest.iso /mnt/hgfs/shared/quest/

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
	find . -name '*.[ch]' | xargs etags -o TAGS 
	find . -name '*.[ch]' | xargs ctags -o tags

clean:
	-rm -f $(OBJS) $(DFILES) $(MAPFILES) $(PROGS) quest.iso quest.map
	-rm -rf iso tftp

cleanacpi:
	-rm -f $(ACPI_OBJS) $(ACPI_DEPS)

cleanall: clean cleanacpi cleanlwip

define ACPI_template
$(1): $(patsubst %.o,%.c,$(1))
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $$@ -c $$^
endef 

$(foreach obj,$(ACPI_OBJS),$(eval $(call ACPI_template,$(obj))))

netboot: quest.iso
	mkdir -p $(TFTPDIR)/boot/grub
	cp $(GRUBLIB)/nbgrub quest shell terminal_server $(TFTPDIR)
	cp netboot/menu.lst $(TFTPDIR)/boot/grub

##################################################
# LWIP

$(LIBLWIP): $(LWIP_OBJS)
	$(AR) r $@ $(LWIP_OBJS)
	ranlib $@

cleanlwip:
	-rm -f $(LWIP_OBJS) $(LWIP_DEPS) $(LIBLWIP)

##################################################


#-include $(shell find . -name '*.d')
