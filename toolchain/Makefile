TARGET=i586-pc-quest
TOOLCHAIN_INSTALL_DIR=$(HOME)/bin/cross


COMPILER = gcc-4.7.2
COMPILER_BUILD_DIR = build-$(COMPILER)
COMPILER_WEB_LOCATION = http://ftp.gnu.org/gnu/gcc/$(COMPILER)/$(COMPILER).tar.gz
BINUTILS = binutils-2.23
BINUTILS_BUILD_DIR = build-$(BINUTILS)
BINUTILS_WEB_LOCATION = http://ftp.gnu.org/gnu/binutils/$(BINUTILS).tar.gz

all:
	$(MAKE) binutils
	$(MAKE) compiler

binutils: $(BINUTILS_BUILD_DIR)/Makefile
	mkdir -p $(TOOLCHAIN_INSTALL_DIR)
	cd $(BINUTILS_BUILD_DIR); $(MAKE) all; $(MAKE) install;

$(BINUTILS_BUILD_DIR)/Makefile:
	wget -c $(BINUTILS_WEB_LOCATION)
	mkdir -p $(BINUTILS_BUILD_DIR)
	tar xf $(BINUTILS).tar.gz
	cd binutils-files; find . -type f -name "*" -exec cp {} ../$(BINUTILS)/{} \;
	cd $(BINUTILS_BUILD_DIR); ../$(BINUTILS)/configure --target=$(TARGET) \
		--prefix=$(TOOLCHAIN_INSTALL_DIR) --disable-nls

compiler: $(COMPILER_BUILD_DIR)/Makefile
	mkdir -p $(TOOLCHAIN_INSTALL_DIR)
	cd $(COMPILER_BUILD_DIR); $(MAKE) all-gcc; $(MAKE) install-gcc; 
	-cd ${HOME}/bin/cross/bin; ln -s $(TARGET)-gcc $(TARGET)-cc
	$(MAKE) -C ../libc/ all;
	cd $(COMPILER_BUILD_DIR); $(MAKE) all; $(MAKE) install


$(COMPILER_BUILD_DIR)/Makefile:
	wget -c $(COMPILER_WEB_LOCATION)
	mkdir -p $(COMPILER_BUILD_DIR)
	tar xf $(COMPILER).tar.gz
	cd compiler-files; find . -type f -name "*" -exec cp {} ../$(COMPILER)/{} \;
	cp compiler-files/gcc/config/quest.h $(COMPILER)/gcc/config/quest.h
	cd $(COMPILER_BUILD_DIR); ../$(COMPILER)/configure --target=$(TARGET) \
		--prefix=$(TOOLCHAIN_INSTALL_DIR) \
		--disable-nls --enable-languages=c --with-newlib

clean:
	rm -rf $(BINUTILS_BUILD_DIR)
	rm -rf $(COMPILER_BUILD_DIR)
	rm -rf $(BINUTILS)
	rm -rf $(COMPILER)

# Need to fix uninstall to not delete everything
uninstall:
	rm -rf $(TOOLCHAIN_INSTALL_DIR)/*
	$(MAKE) -C ../libc uninstall
