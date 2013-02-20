include $(dir $(lastword $(MAKEFILE_LIST)))target-config.mk

INSTALL_CMD = cp
CC = $(TARGET)-gcc
LD = $(TARGET)-ld
AR = $(TARGET)-ar
RANLIB = $(TARGET)-ranlib
LIB_DEST = $(TOOLCHAIN_INSTALL_DIR)/$(TARGET)/lib
INC_DEST = $(TOOLCHAIN_INSTALL_DIR)/$(TARGET)/include
