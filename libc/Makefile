include ../config.mk

LIBC = newlib-1.18.0
LIBC_BUILD_DIR = build-$(LIBC)
LIBC_WEB_LOCATION = ftp://sources.redhat.com/pub/newlib/$(LIBC).tar.gz

QUEST_LIBC_FILES = $(shell find newlib-files -type f)


all: $(LIB_DEST)/libc.a

install:
	@echo "libc.a is installed via make all"

$(LIB_DEST)/libc.a: $(LIBC_BUILD_DIR)/Makefile $(QUEST_LIBC_FILES)
	cd newlib-files; find . -type d -exec mkdir -p ../$(LIBC)/newlib/libc/sys/quest/{} \;
	cd newlib-files; find . -type f -name "*.[ch]" \
		-exec cp {} ../$(LIBC)/newlib/libc/sys/quest/{} \;
	cp newlib-files/vanilla-syscalls.c $(LIBC)/newlib/libc/sys/quest/syscalls.c
	cd $(LIBC_BUILD_DIR); $(MAKE)
	cp newlib-files/syscalls.c $(LIBC)/newlib/libc/sys/quest/syscalls.c
	cd $(LIBC_BUILD_DIR); $(MAKE)
# Install portion
	mkdir -p $(TOOLCHAIN_INSTALL_DIR)
	cd $(LIBC_BUILD_DIR); $(MAKE) install
	cd newlib-files; find . -type d -exec mkdir -p $(INC_DEST)/{} \;
	cd newlib-files; find . -type f -name "*.[h]" \
		-exec cp {} $(INC_DEST)/{} \;


$(LIBC_BUILD_DIR)/Makefile:
	wget -c $(LIBC_WEB_LOCATION)
	mkdir -p $(LIBC_BUILD_DIR)
	tar zxf $(LIBC).tar.gz
	patch -p0 -d $(LIBC) < newlib.patch
	mkdir -p $(LIBC)/newlib/libc/sys/quest/
	cp -r newlib-files/* $(LIBC)/newlib/libc/sys/quest
	cd $(LIBC)/newlib/libc/sys; autoconf
	cd $(LIBC)/newlib/libc/sys/quest; autoreconf
	cd $(LIBC_BUILD_DIR); ../$(LIBC)/configure --target=$(TARGET) \
		--prefix=$(TOOLCHAIN_INSTALL_DIR)

clean:
	-make -C $(LIBC_BUILD_DIR) clean
	rm -f $(LIB_DEST)/libc.a
	rm -f $(LIB_DEST)/libm.a
	find newlib-files/ -name "*~" -exec rm {} \;

uninstall:
	rm -rf $(LIBC_BUILD_DIR)
	rm -rf $(LIBC)
