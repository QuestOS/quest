include ../../config.mk

CFLAGS = -g -I./include -MMD -Wall
OBJS   = src/capture.o src/mjpeg.o src/frame.o

AR_FILE = libqcv.a
INC_DIR_NAME=qcv
HEADER_FILES = $(shell find include -type f)


DFILES = $(patsubst %.o,%.d,$(OBJS))

all: $(LIB_DEST)/$(AR_FILE)

$(LIB_DEST)/$(AR_FILE): $(AR_FILE)
	rm -rf $(INC_DEST)/$(INC_DIR_NAME)
	cd include; find . -type d -exec mkdir -p $(INC_DEST)/$(INC_DIR_NAME)/{} \;
	cd include; find . -type f -name "*.[h]" \
		-exec cp {} $(INC_DEST)/$(INC_DIR_NAME)/{} \;
	cp $(AR_FILE) $(LIB_DEST)

$(AR_FILE): $(OBJS) $(HEADER_FILES)
	$(AR) rcs $@ $(OBJS)

clean:
	rm -f *~ $(OBJS) src/*~ include/*~ $(AR_FILE) $(DFILES)
	rm -rf $(INC_DEST)/$(INC_DIR_NAME)
	-rm $(LIB_DEST)/$(AR_FILE)

install:
	@echo "libqcv.a is installed via make all"


-include $(DFILES)

.PHONY: all clean install
