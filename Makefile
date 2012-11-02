include config.mk
GRUB2 = grub2
PWD := $(shell pwd)
MNT_POINT = /tftp/boot
ISO_DIR = $(PWD)/iso/boot
TAR = tar
SYNC = sync




DIRS = kernel canny netperf softfloat sysprogs tests libc torcs

# the sets of directories to do various things in
BUILDDIRS   = $(DIRS:%=build-%)
CLEANDIRS   = $(DIRS:%=clean-%)
INSTALLDIRS = $(DIRS:%=install-%)

# Uncomment the line below for parallel builds
#MAKEFLAGS += -j
MAKEFLAGS += -k


install: export INSTALL_DIR = $(MNT_POINT)
quest.iso: export INSTALL_DIR = $(ISO_DIR)

all: $(BUILDDIRS)

$(DIRS):
	$(MAKE) build-$@

$(BUILDDIRS):
	$(MAKE) -C $(@:build-%=%)

# set dependencies
build-canny: build-softfloat build-libc
build-sysprogs: build-softfloat build-libc
build-tests: build-softfloat build-libc
build-netperf: build-softfloat build-libc
build-torcs: build-softfloat build-libc
build-softfloat: build-libc


install: $(INSTALLDIRS)
	$(SYNC)

$(INSTALLDIRS) : all
	$(MAKE) -C $(@:install-%=%) install


clean: $(CLEANDIRS)
	rm -rf quest.iso iso

$(CLEANDIRS): 
	$(MAKE) -C $(@:clean-%=%) clean

$(ISO_DIR)/boot/grub/eltorito.img:  iso-grub.cfg 
	mkdir -p iso/boot/grub 
	cp iso-grub.cfg iso/boot/grub/grub.cfg
	cp $(GRUB2)/eltorito.img iso/boot/grub/
	$(TAR) -C iso/boot/grub -jxf $(GRUB2)/mods.tar.bz2

quest.iso: $(ISO_DIR)/grub/eltorito.img all
	$(MAKE) $(INSTALLDIRS);
	mkisofs -quiet $(MSINFO) \
		-R -b boot/grub/eltorito.img \
		-no-emul-boot -boot-load-size 4 \
		-boot-info-table -o $@ iso

.PHONY: subdirs $(DIRS)
.PHONY: subdirs $(BUILDDIRS)
.PHONY: subdirs $(CLEANDIRS)
.PHONY: subdirs $(INSTALLDIRS)
.PHONY: all clean install
