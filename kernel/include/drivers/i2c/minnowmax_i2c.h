#include "kernel.h"

u8 i2c_read_byte_data(u8 command);
s32 i2c_write_byte_data(u8 value);
void i2c_xfer_init(u32 slave_addr);
bool i2c_init();
uint32 i2c_irq_handler(uint8);

