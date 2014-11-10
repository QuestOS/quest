#include "kernel.h"

u8 i2c_read_byte_data(u8 command);
s32 i2c_write_byte_data(u8 command, u8 value);
s32 i2c_read_block_data(u8 command, u8 length, u8 *values);
s32 i2c_write_block_data(u8 command, u8 length, u8 *values);
void i2c_xfer_init(u32 slave_addr);
void i2c_init();
void i2c_remove();

