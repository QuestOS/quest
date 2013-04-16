include ../config.mk

LIBC = newlib-1.18.0
LIBC_BUILD_DIR = build-$(LIBC)
LIBC_WEB_LOCATION = ftp://sources.redhat.com/pub/newlib/$(LIBC).tar.gz
ARCH_INDEPENDENT_QUEST_FILES = quest-files
ARCH_INDEPENDENT_QUEST_LIBC_FILES = $(shell find $(ARCH_INDEPENDENT_QUEST_FILES) -type f)
ARCH_DEPENDENT_QUEST_FILES = arch/$(TARGET)
ARCH_DEPENDENT_QUEST_LIBC_FILES = $(shell find $(ARCH_DEPENDENT_QUEST_FILES) -type f)


all: $(LIB_DEST)/libc.a

install:
	@echo "libc.a is installed via make all"

$(LIB_DEST)/libc.a: $(LIBC_BUILD_DIR)/Makefile $(ARCH_INDEPENDENT_QUEST_LIBC_FILES) $(ARCH_DEPENDENT_QUEST_LIBC_FILES)
	cd $(ARCH_INDEPENDENT_QUEST_FILES); \
		find . -type d -exec mkdir -p ../$(LIBC)/newlib/libc/sys/quest/{} \;
	cd $(ARCH_INDEPENDENT_QUEST_FILES); find . -type f -name "*.[ch]" \
		-exec cp {} ../$(LIBC)/newlib/libc/sys/quest/{} \;
	cd $(ARCH_DEPENDENT_QUEST_FILES); \
		find . -type d -exec mkdir -p ../../$(LIBC)/newlib/libc/sys/quest/{} \;
	cd $(ARCH_DEPENDENT_QUEST_FILES); find . -type f -name "*.[ch]" \
		-exec cp {} ../../$(LIBC)/newlib/libc/sys/quest/{} \;
	cp $(ARCH_INDEPENDENT_QUEST_FILES)/vanilla-syscalls.c $(LIBC)/newlib/libc/sys/quest/syscalls.c
	cd $(LIBC_BUILD_DIR); $(MAKE)
	cp $(ARCH_DEPENDENT_QUEST_FILES)/syscalls.c $(LIBC)/newlib/libc/sys/quest/syscalls.c
	cd $(LIBC_BUILD_DIR); $(MAKE)
# Install portion
	mkdir -p $(TOOLCHAIN_INSTALL_DIR)
	cd $(LIBC_BUILD_DIR); $(MAKE) install
	cd $(ARCH_INDEPENDENT_QUEST_FILES); find . -type d -exec mkdir -p $(INC_DEST)/{} \;
	cd $(ARCH_INDEPENDENT_QUEST_FILES); find . -type f -name "*.[h]" \
		-exec cp {} $(INC_DEST)/{} \;
	cd $(ARCH_DEPENDENT_QUEST_FILES); find . -type d -exec mkdir -p $(INC_DEST)/{} \;
	cd $(ARCH_DEPENDENT_QUEST_FILES); find . -type f -name "*.[h]" \
		-exec cp {} $(INC_DEST)/{} \;


$(LIBC_BUILD_DIR)/Makefile:
	wget -c $(LIBC_WEB_LOCATION)
	mkdir -p $(LIBC_BUILD_DIR)
	tar zxf $(LIBC).tar.gz
	patch -p0 -d $(LIBC) < newlib.patch
	mkdir -p $(LIBC)/newlib/libc/sys/quest/
	cp -r $(ARCH_INDEPENDENT_QUEST_FILES)/* $(LIBC)/newlib/libc/sys/quest
	cp -r $(ARCH_DEPENDENT_QUEST_FILES)/* $(LIBC)/newlib/libc/sys/quest
	cd $(LIBC)/newlib/libc/sys; autoconf
	cd $(LIBC)/newlib/libc/sys/quest; autoreconf
	cd $(LIBC_BUILD_DIR); ../$(LIBC)/configure --target=$(TARGET) \
		--prefix=$(TOOLCHAIN_INSTALL_DIR)

clean:
	-make -C $(LIBC_BUILD_DIR) clean
	rm -f $(LIB_DEST)/libc.a
	rm -f $(LIB_DEST)/libm.a
	find $(ARCH_INDEPENDENT_QUEST_FILES)/ -name "*~" -exec rm {} \;
	find $(ARCH_DEPENDENT_QUEST_FILES)/ -name "*~" -exec rm {} \;

uninstall:
	rm -rf $(LIBC_BUILD_DIR)
	rm -rf $(LIBC)
