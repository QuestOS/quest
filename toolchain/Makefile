#TARGET=i586-pc-quest
TARGET=arm-pc-quest
TOOLCHAIN_INSTALL_DIR=$(HOME)/bin/cross


COMPILER = gcc-4.7.2
COMPILER_BUILD_DIR = build-$(COMPILER)
COMPILER_WEB_LOCATION = http://ftp.gnu.org/gnu/gcc/$(COMPILER)/$(COMPILER).tar.gz

BINUTILS = binutils-2.23
BINUTILS_BUILD_DIR = build-$(BINUTILS)
BINUTILS_WEB_LOCATION = http://ftp.gnu.org/gnu/binutils/$(BINUTILS).tar.gz

GMP = gmp-5.0.1
GMP_BUILD_DIR = build-$(GMP)
GMP_WEB_LOCATION = http://ftp.gnu.org/gnu/gmp/$(GMP).tar.gz

MPFR = mpfr-2.4.2
MPFR_BUILD_DIR = build-$(MPFR)
MPFR_WEB_LOCATION = http://ftp.gnu.org/gnu/mpfr/$(MPFR).tar.gz

MPC = mpc-0.8.1
MPC_BUILD_DIR = build-$(MPC)
MPC_WEB_LOCATION = http://www.multiprecision.org/mpc/download/$(MPC).tar.gz


all: toolchain

toolchain:
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
	#$(MAKE) -C ../libc/ all;
	#cd $(COMPILER_BUILD_DIR); $(MAKE) all; $(MAKE) install

gmp: $(GMP_BUILD_DIR)/Makefile
	$(MAKE) -C $(GMP_BUILD_DIR)
	$(MAKE) -C $(GMP_BUILD_DIR) check
	$(MAKE) -C $(GMP_BUILD_DIR) install

$(GMP_BUILD_DIR)/Makefile:
	wget -c $(GMP_WEB_LOCATION)
	mkdir -p $(GMP_BUILD_DIR)
	tar xf $(GMP).tar.gz
	cd $(GMP_BUILD_DIR); ../$(GMP)/configure --prefix=$(TOOLCHAIN_INSTALL_DIR) --disable-shared

mpfr: $(MPFR_BUILD_DIR)/Makefile
	$(MAKE) -C $(MPFR_BUILD_DIR)
	$(MAKE) -C $(MPFR_BUILD_DIR) check
	$(MAKE) -C $(MPFR_BUILD_DIR) install

$(MPFR_BUILD_DIR)/Makefile:
	wget -c $(MPFR_WEB_LOCATION)
	mkdir -p $(MPFR_BUILD_DIR)
	tar xf $(MPFR).tar.gz
	cd $(MPFR_BUILD_DIR); ../$(MPFR)/configure --prefix=$(TOOLCHAIN_INSTALL_DIR) --with-gmp=$(TOOLCHAIN_INSTALL_DIR) --disable-shared

mpc: $(MPC_BUILD_DIR)/Makefile
	$(MAKE) -C $(MPC_BUILD_DIR)
	$(MAKE) -C $(MPC_BUILD_DIR) check
	$(MAKE) -C $(MPC_BUILD_DIR) install

$(MPC_BUILD_DIR)/Makefile:
	wget -c $(MPC_WEB_LOCATION)
	mkdir -p $(MPC_BUILD_DIR)
	tar xf $(MPC).tar.gz
	cd $(MPC_BUILD_DIR); ../$(MPC)/configure --prefix=$(TOOLCHAIN_INSTALL_DIR) --with-gmp=$(TOOLCHAIN_INSTALL_DIR) --with-mpfr=$(TOOLCHAIN_INSTALL_DIR) --disable-shared

$(COMPILER_BUILD_DIR)/Makefile:
	$(MAKE) gmp
	$(MAKE) mpfr
	$(MAKE) mpc
	wget -c $(COMPILER_WEB_LOCATION)
	mkdir -p $(COMPILER_BUILD_DIR)
	tar xf $(COMPILER).tar.gz
	cd compiler-files; find . -type f -name "*" -exec cp {} ../$(COMPILER)/{} \;
	cp compiler-files/gcc/config/quest.h $(COMPILER)/gcc/config/quest.h
	cd $(COMPILER_BUILD_DIR); ../$(COMPILER)/configure --target=$(TARGET) \
		--prefix=$(TOOLCHAIN_INSTALL_DIR) \
		--with-gmp=$(TOOLCHAIN_INSTALL_DIR) --with-mpfr=$(TOOLCHAIN_INSTALL_DIR) --with-mpc=$(TOOLCHAIN_INSTALL_DIR) \
		--disable-nls --enable-languages=c --with-newlib

clean:
	rm -rf $(GMP_BUILD_DIR)
	rm -rf $(MPFR_BUILD_DIR)
	rm -rf $(MPC_BUILD_DIR)
	rm -rf $(BINUTILS_BUILD_DIR)
	rm -rf $(COMPILER_BUILD_DIR)
	rm -rf $(GMP)
	rm -rf $(MPFR)
	rm -rf $(MPC)
	rm -rf $(BINUTILS)
	rm -rf $(COMPILER)

# Need to fix uninstall to not delete everything
uninstall:
	rm -rf $(TOOLCHAIN_INSTALL_DIR)/*
	$(MAKE) -C ../libc uninstall
