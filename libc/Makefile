include ../config.mk

LIBC = newlib-1.18.0
LIBC-BUILD-DIR = build-$(LIBC)
LIBC-WEB-LOCATION = ftp://sources.redhat.com/pub/newlib/$(LIBC).tar.gz

QUEST-LIBC-FILES = $(shell find newlib-files -type f)


all: $(LIB_DEST)/libc.a

install:
	@echo "libc.a is installed via make all"

$(LIB_DEST)/libc.a: $(LIBC-BUILD-DIR)/Makefile $(QUEST-LIBC-FILES)
	cd newlib-files; find . -type d -exec mkdir -p ../$(LIBC)/newlib/libc/sys/quest/{} \;
	cd newlib-files; find . -type f -name "*.[ch]" \
		-exec cp {} ../$(LIBC)/newlib/libc/sys/quest/{} \;
	cp newlib-files/vanilla-syscalls.c $(LIBC)/newlib/libc/sys/quest/syscalls.c
	cd $(LIBC-BUILD-DIR); $(MAKE)
	cp newlib-files/syscalls.c $(LIBC)/newlib/libc/sys/quest/syscalls.c
	cd $(LIBC-BUILD-DIR); $(MAKE)
# Install portion
	mkdir -p $(TOOLCHAIN-INSTALL-DIR)
	cd $(LIBC-BUILD-DIR); $(MAKE) install
	cd newlib-files; find . -type d -exec mkdir -p $(INC_DEST)/{} \;
	cd newlib-files; find . -type f -name "*.[h]" \
		-exec cp {} $(INC_DEST)/{} \;


$(LIBC-BUILD-DIR)/Makefile:
	wget -c $(LIBC-WEB-LOCATION)
	mkdir -p $(LIBC-BUILD-DIR)
	tar zxf $(LIBC).tar.gz
	patch -p0 -d $(LIBC) < newlib.patch
	mkdir -p $(LIBC)/newlib/libc/sys/quest/
	cp -r newlib-files/* $(LIBC)/newlib/libc/sys/quest
	cd $(LIBC)/newlib/libc/sys; autoconf
	cd $(LIBC)/newlib/libc/sys/quest; autoreconf
	cd $(LIBC-BUILD-DIR); ../$(LIBC)/configure --target=$(TARGET) \
		--prefix=$(TOOLCHAIN-INSTALL-DIR)

clean:
	make -C $(LIBC-BUILD-DIR) clean
	rm -f $(LIB_DEST)/libc.a
	rm -f $(LIB_DEST)/libm.a
	find newlib-files/ -name "*~" -exec rm {} \;

uninstall:
	rm -rf $(LIBC-BUILD-DIR)
	rm -rf $(LIBC)
