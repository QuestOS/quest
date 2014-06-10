include config.mk
include ../config.mk
GRUBLIB = grub/x86_64-pc
TFTPDIR=tftp

TAR = tar
CFLAGS = -fno-builtin -fno-stack-protector -fno-strict-aliasing -fno-delete-null-pointer-checks -nostdinc -g -Wall -Wno-attributes -O$(OPT) $(CFG) $(EXTRA_CFLAGS)
CPPFLAGS = $(CFG) -D_QUEST -Wa,--32 -MMD -Iinclude -Iinclude/drivers/acpi -I.
INDENT = indent
INDENTFLAGS = -gnu -br -ce --no-tabs

##################################################

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

PROGS = quest

##################################################


# -- EM --The following I removed because I changed the USB interface and broke them
# drivers/usb/ftdi.o  
# drivers/usb/rtl8187b.o 
# drivers/usb/net.o drivers/usb/asix.o
# drivers/usb/uhci_hcd.o
# drivers/usb/usb_test.o

OBJS =  boot/boot.o boot/init.o kernel.o module/loader.o \
	boot/multiboot.o \
	interrupt.o interrupt_handler.o socket.o \
	smp/boot-smp.o smp/smp.o smp/intel.o smp/acpi.o smp/apic.o smp/semaphore.o \
	arch/i386/percpu.o arch/i386/measure.o \
	vm/vmx.o vm/ept.o vm/shm.o vm/shdr.o vm/migration.o vm/hypercall.o vm/fault_detection.o \
	vm/linux_boot.o \
	sched/task.o sched/sched.o sched/sleep.o sched/vcpu.o sched/ipc.o sched/msgt.o sched/proc.o \
	mem/physical.o mem/virtual.o mem/$(KMALLOC).o mem/malloc.o mem/dma_pool.o \
	util/cpuid.o util/printf.o util/screen.o util/debug.o util/circular.o \
	util/crc32.o util/bitrev.o util/logger.o util/perfmon.o util/sort.o \
	drivers/ata/ata.o drivers/ata/diskio.o \
	drivers/input/keyboard_8042.o drivers/input/keymap.o \
	drivers/pci/pci.o drivers/pci/pci_irq.o \
	drivers/i2c/galileo_i2c.o drivers/gpio/cy8c9540a.o \
	drivers/net/ethernetif.o drivers/net/pcnet.o \
	drivers/net/e1000.o drivers/net/e1000e.o \
	drivers/net/bnx2.o \
	drivers/net/mac80211.o drivers/net/netsetup.o \
	drivers/serial/mcs9922.o \
	drivers/video/vga.o \
	fs/fsys.o \
	fs/ext2/fsys_ext2fs.o \
	fs/iso9660/fsys_iso9660.o \
	fs/vfat/fsys_vfat.o \
	fs/tftp/fsys_tftp.o \
	fs/ram_romfs/fsys_ram_romfs.o \
	drivers/sb16/sound.o \
	drivers/acpi/quest-acpica.o \
	drivers/eeprom/93cx6.o \
	drivers/usb/umsc.o \
	drivers/usb/gadget/net2280.o \
	drivers/usb/usb-keyboard.o drivers/usb/sidewinder2_joystick.o \
	drivers/usb/pl2303.o \
	drivers/usb/usb-migration.o \
	drivers/usb/hub.o drivers/usb/uvc.o drivers/usb/beagle_gadget.o drivers/usb/beagle_gadget2.o \
	drivers/usb/ehci_hcd.o drivers/usb/ehci_mem.o drivers/usb/ehci_debug.o \
	drivers/usb/usb.o drivers/usb/enumeration.o \
	drivers/serial/mmio32_uart.o \
	kdb/i386-stub.o

CFILES = $(patsubst %.o,%.c,$(OBJS))
DFILES = $(patsubst %.o,%.d,$(OBJS))
DFILES += $(patsubst %,%.d,$(PROGS))
MAPFILES = $(patsubst %.o,%.map,$(OBJS))
MAPFILES += $(patsubst %,%.map,$(PROGS))


##################################################

ACPI_SRCS = $(shell ls drivers/acpica/*.c)
ACPI_OBJS = $(patsubst %.c,%.o,$(ACPI_SRCS))
ACPI_DEPS = $(patsubst %.c,%.d,$(ACPI_SRCS))

##################################################

SHARED_COMPS = shared.o

SHARED_COMPS_CFILES = \
  vm/spow2.c \
  drivers/net/r8169.c

SHARED_COMPS_OBJS = $(patsubst %.c,%.o,$(SHARED_COMPS_CFILES))
SHARED_COMPS_DEPS = $(patsubst %.c,%.d,$(SHARED_COMPS_CFILES))

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

.PHONY: all tags clean cleanacpi cleanlwip vmware indent netboot

all: $(PROGS)

quest: $(OBJS) $(ACPI_OBJS) $(LWIP_OBJS) $(SHARED_COMPS)
	rm -f quest
	$(LD) -Map quest.map -T ldscripts/quest.ld -o $@ $^ 

%: %.o
	$(LD) -o $@ $< -T ldscripts/module.ld -Map $@.map

%: %.c

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) welcome.raw $(INSTALL_DIR)

vmware: install quest.iso
	qemu-img convert c.img -O vmdk c-img.vmdk
	cp c-img.vmdk /mnt/hgfs/shared/quest/
	cp quest.iso /mnt/hgfs/shared/quest/

indent:
	(for f in $(CFILES); do [ -f "$$f" ] && $(INDENT) $(INDENTFLAGS) "$$f"; done)

tags:
	find . -name '*.[ch]' | xargs etags -o TAGS 
	find . -name '*.[ch]' | xargs ctags -o tags

clean: cleanacpi cleanlwip cleanshared
	-rm -f $(OBJS) $(DFILES) $(MAPFILES) $(PROGS) $(PROG_OBJS) quest.map
	-rm -rf tftp

cleanacpi:
	-rm -f $(ACPI_OBJS) $(ACPI_DEPS)

define ACPI_template
$(1): $(patsubst %.o,%.c,$(1))
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $$@ -c $$<
endef

$(foreach obj,$(ACPI_OBJS),$(eval $(call ACPI_template,$(obj))))

netboot: ../quest.iso
	mkdir -p $(TFTPDIR)/boot/grub
	cp quest ../sysprogs/shell $(TFTPDIR)/boot
	cp netboot/grub.cfg $(TFTPDIR)/boot/grub
	cp ../grub2/grub2pxe $(TFTPDIR)
	$(TAR) -C $(TFTPDIR)/boot/grub -jxf ../grub2/mods.tar.bz2

##################################################
# LWIP

$(LIBLWIP): $(LWIP_OBJS)
	$(AR) r $@ $(LWIP_OBJS)
	$(RANLIB) $@

cleanlwip:
	-rm -f $(LWIP_OBJS) $(LWIP_DEPS) $(LIBLWIP)

##################################################
# Shared System Components

$(SHARED_COMPS): $(SHARED_COMPS_OBJS)
	$(LD) -r $(SHARED_COMPS_OBJS) -o $@

cleanshared:
	-rm -f $(SHARED_COMPS_OBJS) $(SHARED_COMPS) $(SHARED_COMPS_DEPS)

##################################################

config.mk: default-config.mk
	cp -f default-config.mk config.mk

-include $(DFILES)
-include $(SHARED_COMPS_DEPS)

$(SHARED_COMPS_OBJS) $(OBJS) $(LWIP_OBJS) $(ACPI_OBJS) : config.mk
