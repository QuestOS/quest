TARGET=i586-pc-quest
TOOLCHAIN-INSTALL-DIR=$(HOME)/bin/cross


COMPILER = gcc-4.7.2
COMPILER-BUILD-DIR = build-$(COMPILER)
COMPILER-WEB-LOCATION = http://ftp.gnu.org/gnu/gcc/$(COMPILER)/$(COMPILER).tar.gz
BINUTILS = binutils-2.23
BINUTILS-BUILD-DIR = build-$(BINUTILS)
BINUTILS-WEB-LOCATION = http://ftp.gnu.org/gnu/binutils/$(BINUTILS).tar.gz

all:
	$(MAKE) binutils
	$(MAKE) compiler

binutils: $(BINUTILS-BUILD-DIR)/Makefile
	mkdir -p $(TOOLCHAIN-INSTALL-DIR)
	cd $(BINUTILS-BUILD-DIR); $(MAKE) all; $(MAKE) install;

$(BINUTILS-BUILD-DIR)/Makefile:
	wget -c $(BINUTILS-WEB-LOCATION)
	mkdir -p $(BINUTILS-BUILD-DIR)
	tar xf $(BINUTILS).tar.gz
	cd binutils-files; find . -type f -name "*" -exec cp {} ../$(BINUTILS)/{} \;
	cd $(BINUTILS-BUILD-DIR); ../$(BINUTILS)/configure --target=$(TARGET) \
		--prefix=$(TOOLCHAIN-INSTALL-DIR) --disable-nls

compiler: $(COMPILER-BUILD-DIR)/Makefile
	mkdir -p $(TOOLCHAIN-INSTALL-DIR)
	cd $(COMPILER-BUILD-DIR); $(MAKE) all-gcc; $(MAKE) install-gcc; 
	-cd ${HOME}/bin/cross/bin; ln -s $(TARGET)-gcc $(TARGET)-cc
	$(MAKE) -C ../libc/ all;
	cd $(COMPILER-BUILD-DIR); $(MAKE) all; $(MAKE) install


$(COMPILER-BUILD-DIR)/Makefile:
	wget -c $(COMPILER-WEB-LOCATION)
	mkdir -p $(COMPILER-BUILD-DIR)
	tar xf $(COMPILER).tar.gz
	cd compiler-files; find . -type f -name "*" -exec cp {} ../$(COMPILER)/{} \;
	cp compiler-files/gcc/config/quest.h $(COMPILER)/gcc/config/quest.h
	cd $(COMPILER-BUILD-DIR); ../$(COMPILER)/configure --target=$(TARGET) \
		--prefix=$(TOOLCHAIN-INSTALL-DIR) \
		--disable-nls --enable-languages=c --with-newlib

clean:
	rm -rf $(BINUTILS-BUILD-DIR)
	rm -rf $(COMPILER-BUILD-DIR)
	rm -rf $(BINUTILS)
	rm -rf $(COMPILER)

# Need to fix uninstall to not delete everything
uninstall:
	rm -rf $(TOOLCHAIN-INSTALL-DIR)/*
	$(MAKE) -C ../libc uninstall
