include ../config.mk

ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

LIBS   = $(LIB_DEST)/crt0.o -lc
INCS   = 
CFLAGS = -g

OTHER_FILES = sample.txt

SUBDIRS = sketches

PROGS = exec migtest race test1 test2 test3 test4 test5 \
	test6 test7  float_test \
	usb_gadget_test joystick_test \
	cl-param-test vcputest vga seek getpid \
	vshm_test vshm_circ_buf vshm_async \
	fault_detection fault_detection_sink \
	vshm_linux pololu thread thread1 matrix \
	find_prime 

#usb_test usb_test_rtt

.PHONY: all clean install

all: $(PROGS)
	@for i in $(SUBDIRS); do \
		echo "make all in $$i..."; \
		(cd $$i; $(MAKE) all); done


%: %.c $(LIB_DEST)/libc.a
	$(CC) $(INCS) $(CFLAGS) $< -o $@


clean: 
	rm -f *.o *.d *.map *~ $(PROGS)
	@for i in $(SUBDIRS); do \
		echo "make clean in $$i..."; \
		(cd $$i; $(MAKE) clean); done

install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(OTHER_FILES) $(INSTALL_DIR)
	@for i in $(SUBDIRS); do \
		echo "make install in $$i..."; \
		(cd $$i; $(MAKE) install); done
