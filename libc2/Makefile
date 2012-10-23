NEWLIB = newlib-1.20.0
BUILD_DIR = newlib-build
QUEST_CONFIG_SCRIPT = quest_newlib_configure
NEWLIB_TARGET = i586-elf

all: newlib_libs


$(NEWLIB)/$(BUILD_DIR)/Makefile:
	tar xzf $(NEWLIB).tar.gz
	mkdir $(NEWLIB)/$(BUILD_DIR)
	cp $(QUEST_CONFIG_SCRIPT) $(NEWLIB)/$(BUILD_DIR)/
	cd $(NEWLIB)/$(BUILD_DIR); ./$(QUEST_CONFIG_SCRIPT); 

newlib_make: $(NEWLIB)/$(BUILD_DIR)/Makefile
	cd $(NEWLIB)/$(BUILD_DIR); $(MAKE)


newlib_libs: $(NEWLIB)/$(BUILD_DIR)/$(NEWLIB_TARGET)/newlib/libc.a


$(NEWLIB)/$(BUILD_DIR)/$(NEWLIB_TARGET)/newlib/libc.a:
	$(MAKE) newlib_make

cleanall:
	rm -rf $(NEWLIB)

.PHONY: all newlib_make newlib_libc clean cleanall
