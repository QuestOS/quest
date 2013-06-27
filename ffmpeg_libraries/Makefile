include ../config.mk

FFMPEG_VERSION=1.2.1
FFMPEG=ffmpeg-$(FFMPEG_VERSION)

all: make_guard

$(FFMPEG_FILES): $(FFMPEG)/config.h

make_guard: $(FFMPEG)/config.h
	$(MAKE) -C $(FFMPEG)
	$(MAKE) -C $(FFMPEG) install 
	touch $@

$(FFMPEG)/config.h: $(FFMPEG).tar.gz ffmpeg.patch
	$(MAKE) clean
	rm -rf $(FFMPEG) make_guard
	tar xf $< 
	cd $(FFMPEG); ./configure --disable-programs --disable-pthreads --disable-dxva2 \
		--disable-vaapi --disable-vdpau --disable-doc --cross-prefix=i586-pc-quest- \
		--enable-cross-compile --arch=x86 --target-os=none \
		--prefix=$(TARGET_DEST)
	patch -p1 -d $(FFMPEG) < ffmpeg.patch

clean:
	-test -d $(FFMPEG) && $(MAKE) -C $(FFMPEG) clean
	-test -d $(FFMPEG) && $(MAKE) -C $(FFMPEG) uninstall
	rm -f make_guard

install:
	@echo "ffmpeg is installed via make all"

cleanall: clean
	rm -rf $(FFMPEG)


.PHONY: all clean install


