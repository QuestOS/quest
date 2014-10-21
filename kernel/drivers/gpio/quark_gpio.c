#include "drivers/pci/pci.h"
#include "util/printf.h"
#include "mem/mem.h"

#define DEBUG_QGPIO

#ifdef DEBUG_QGPIO
#define DLOG(fmt,...) DLOG_PREFIX("Quark GPIO",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif
#define DERR(fmt,...) DLOG_PREFIX("Quark GPIO",fmt,##__VA_ARGS__)

#define GALILEO_QGPIO_VID			0x8086
#define	GALILEO_QGPIO_DID			 	0x0934

#define QGPIO_PORT_A_DATA 0x0
#define QGPIO_PORT_A_DIR 	0x04

static u32 qgpio_base;

static inline void
qgpio_write_r(u32 val , u32 reg)
{
  *(u32 *)(qgpio_base + reg) = val;
}

static inline u32
qgpio_read_r(u32 reg)
{
  return *(u32 *)(qgpio_base+ reg);
}

void 
quark_gpio8_high()
{
	qgpio_write_r(0x1UL | qgpio_read_r(QGPIO_PORT_A_DATA), QGPIO_PORT_A_DATA);
}

void 
quark_gpio8_low()
{
	qgpio_write_r((~0x1UL) & qgpio_read_r(QGPIO_PORT_A_DATA), QGPIO_PORT_A_DATA);
}

u32 
quark_gpio8_read()
{
	return qgpio_read_r(QGPIO_PORT_A_DATA);
}

bool
quark_gpio_init()
{
	uint device_index;
	uint mem_addr;

	if (!pci_find_device(GALILEO_QGPIO_VID, GALILEO_QGPIO_DID,
				0xFF, 0xFF, 0, &device_index))
		return FALSE;
	if (device_index == (uint)(~0)) {
    DERR ("Unable to detect compatible device.");
    return FALSE;
  }
	DLOG("device_index is %u", device_index);
	if (!pci_decode_bar (device_index, 1, &mem_addr, NULL, NULL)) {
    DERR ("Bar decoding failed!");
    return FALSE;
  } 
	DLOG("Initializing Quark GPIO...");
	DLOG("mem_addr is 0x%X", mem_addr);

	/* map the physical page of memory-mapped registers in page table */
	qgpio_base = (u32)map_virtual_page(mem_addr | 0x3); // PTE_P | PTE_W

	/* test */
	//DLOG("DIR=0x%X", qgpio_read_r(QGPIO_PORT_A_DIR));
	//DLOG("DATA=0x%X", qgpio_read_r(QGPIO_PORT_A_DATA));

	return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = quark_gpio_init
};

DEF_MODULE (galileo_quark_gpio, "Galileo Quark GPIO driver", &mod_ops, {"pci"});


