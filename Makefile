include config.mk
GRUB2 = grub2
PWD := $(shell pwd)
MNT_POINT = /tftp/boot
ISO_DIR = $(PWD)/iso/boot
TAR = tar
SYNC = sync

QUEST_USER_PROGS_DIRS = canny netperf sysprogs tests torcs malardalen \
			zlib-1.2.7 qcv/progs
QUEST_LIB_DIRS = libc libjpeg qcv/lib
DIRS = $(QUEST_USER_PROGS_DIRS) $(QUEST_LIB_DIRS) kernel

# the sets of directories to do various things in
BUILDDIRS   = $(DIRS:%=build-%)
CLEANDIRS   = $(DIRS:%=clean-%)
INSTALLDIRS = $(DIRS:%=install-%)
CLEANUSERPROGS = $(QUEST_USER_PROGS_DIRS:%=clean-%)

# Uncomment the line below for parallel builds
#MAKEFLAGS += -j
MAKEFLAGS += -k


install: export INSTALL_DIR = $(MNT_POINT)
quest.iso: export INSTALL_DIR = $(ISO_DIR)

all: $(BUILDDIRS)

$(DIRS):
	$(MAKE) -C $@

$(BUILDDIRS):
	$(MAKE) -C $(@:build-%=%)

# set dependencies
build-canny:  build-libc
build-sysprogs:  build-libc
build-tests:  build-libc
build-netperf:  build-libc
build-torcs:  build-libc
build-zlib-1.2.7: build-libc
build-malardalen: build-libc
build-sysprogs: build-kernel
build-libjpeg: build-libc
build-qcv/lib: build-libc
build-qcv/progs: build-libc build-qcv/lib


install: $(INSTALLDIRS)
	$(SYNC)

$(INSTALLDIRS) : all
	$(MAKE) -C $(@:install-%=%) install


clean: $(CLEANDIRS)
	rm -rf quest.iso iso

$(CLEANDIRS): 
	$(MAKE) -C $(@:clean-%=%) clean

clean-user-progs: $(CLEANUSERPROGS)

$(ISO_DIR)/grub/eltorito.img:  iso-grub.cfg 
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
.PHONY: all clean clean-user-progs install
