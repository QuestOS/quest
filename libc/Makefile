TARGET := i586-pc-quest
PREFIX := ${HOME}/bin/cross

LIBC = newlib-1.18.0
LIBC-BUILD-DIR = build-$(LIBC)
LIBC-WEB-LOCATION = ftp://sources.redhat.com/pub/newlib/$(LIBC).tar.gz

all: libc

install: 
	mkdir -p ${PREFIX}
	cd $(LIBC-BUILD-DIR); make install

libc: $(LIBC-BUILD-DIR)/Makefile
	cp newlib-files/vanilla-syscalls.c $(LIBC)/newlib/libc/sys/quest/syscalls.c
	cd $(LIBC-BUILD-DIR); $(MAKE)
	cp newlib-files/syscalls.c $(LIBC)/newlib/libc/sys/quest/syscalls.c
	cd $(LIBC-BUILD-DIR); $(MAKE)


$(LIBC-BUILD-DIR)/Makefile:
	wget -c $(LIBC-WEB-LOCATION)
	mkdir -p $(LIBC-BUILD-DIR)
	tar zxf $(LIBC).tar.gz
	patch -p0 -d $(LIBC) < newlib.patch
	mkdir -p $(LIBC)/newlib/libc/sys/quest/
	cp -r newlib-files/* $(LIBC)/newlib/libc/sys/quest
	cd $(LIBC)/newlib/libc/sys; autoconf
	cd $(LIBC)/newlib/libc/sys/quest; autoreconf
	cd $(LIBC-BUILD-DIR); ../$(LIBC)/configure --target=${TARGET} --prefix=${PREFIX}

clean:
	rm -rf $(LIBC-BUILD-DIR)
	rm -rf $(LIBC)
	cd newlib-files; rm -rf *~ *.o

uninstall:
	rm -rf ${PREFIX}/*
