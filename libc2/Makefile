NEWLIB = newlib-1.20.0
BUILD_DIR = newlib-build
QUEST_CONFIG_SCRIPT = quest_newlib_configure
NEWLIB_TARGET = i586-elf
BROKEN_CONFIG_FILE = ./newlib-1.20.0/newlib/libc/include/sys/config.h

all: libc

libc: 
	$(MAKE) newlib-libc.a
	cd quest-libc; $(MAKE) quest-libc.a
	$(MAKE) libc.a

libc.a: newlib-libc.a quest-libc/quest-libc.a
	rm -rf temp_objects
	mkdir temp_objects
	cp newlib-libc.a quest-libc/quest-libc.a temp_objects
	cd temp_objects; ar -x newlib-libc.a
	cd temp_objects; ar -x quest-libc.a
	cd temp_objects; ar rcs libc.a *.o
	cd temp_objects; cp libc.a ..
	rm -rf temp_objects


$(NEWLIB)/$(BUILD_DIR)/Makefile:
	tar xzf $(NEWLIB).tar.gz
	mkdir $(NEWLIB)/$(BUILD_DIR)
	cp $(QUEST_CONFIG_SCRIPT) $(NEWLIB)/$(BUILD_DIR)/
	cd $(NEWLIB)/$(BUILD_DIR); ./$(QUEST_CONFIG_SCRIPT);
	sed -i 's/#define __LARGE64_FILES 1/\/\/#define __LARGE64_FILES 1/' $(BROKEN_CONFIG_FILE)


newlib_make: $(NEWLIB)/$(BUILD_DIR)/Makefile
	cd $(NEWLIB)/$(BUILD_DIR); $(MAKE)

quest-libc/quest-libc.a: $(NEWLIB)/$(BUILD_DIR)/$(NEWLIB_TARGET)/newlib/libc.a
	cd quest-libc/; $(MAKE)


newlib_libs: $(NEWLIB)/$(BUILD_DIR)/$(NEWLIB_TARGET)/newlib/libc.a


$(NEWLIB)/$(BUILD_DIR)/$(NEWLIB_TARGET)/newlib/libc.a:
	$(MAKE) newlib_make

libm.a: $(NEWLIB)/$(BUILD_DIR)/$(NEWLIB_TARGET)/newlib/libc.a
	cp $(NEWLIB)/$(BUILD_DIR)/$(NEWLIB_TARGET)/newlib/libm.a libm.a 

newlib-libc.a: $(NEWLIB)/$(BUILD_DIR)/$(NEWLIB_TARGET)/newlib/libc.a
	cp $(NEWLIB)/$(BUILD_DIR)/$(NEWLIB_TARGET)/newlib/libc.a newlib-libc.a

clean:
	-cd $(NEWLIB)/$(BUILD_DIR)/ && $(MAKE) clean
	cd quest-libc/; $(MAKE) clean
	rm -rf *~ libc.a libm.a newlib-libc.a temp_objects

cleanall: 
	rm -rf $(NEWLIB)
	$(MAKE) clean

.PHONY: all newlib_make newlib_libc clean cleanall libc
