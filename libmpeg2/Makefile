include ../config.mk

LIBMPEG2_VERSION=0.5.1
LIBMPEG2=libmpeg2-$(LIBMPEG2_VERSION)

LIBMPEG2_FILES = $(shell find $(LIBMPEG2) -type f | grep -v $(LIBMPEG2)/Makefile)

all: make_guard

$(LIBMPEG2_FILES): $(LIBMPEG2)/Makefile

make_guard: $(LIBMPEG2)/Makefile $(LIBMPEG2_FILES)
	$(MAKE) -C $(LIBMPEG2)
	$(MAKE) -C $(LIBMPEG2) install 
	touch $@

$(LIBMPEG2)/Makefile: $(LIBMPEG2).tar.gz libmpeg2.patch
	$(MAKE) clean
	rm -rf $(LIBMPEG2) make_guard
	tar xf $< 
	patch -p1 -d $(LIBMPEG2) < libmpeg2.patch
	cd $(LIBMPEG2); ./configure -host=$(TARGET) --disable-sdl --disable-shared --prefix=$(TARGET_DEST)

clean:
	-$(MAKE) -C $(LIBMPEG2) clean
	-$(MAKE) -C $(LIBMPEG2) uninstall
	rm -f make_guard

install:
	@echo "libmpeg2 is installed via make all"

cleanall: clean
	rm -rf $(LIBMPEG2)


.PHONY: all clean install
