/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "drivers/pci/pci.h"
#include "drivers/gpio/gpio.h"
#include "util/printf.h"
#include "mem/mem.h"
#include "drivers/acpi/acpixf.h"

#define DEBUG_MGPIO

#ifdef DEBUG_MGPIO
#define DLOG(fmt,...) DLOG_PREFIX("MINNOWMAX GPIO",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define MINNOWMAX_LPC_VID			0x8086
#define	MINNOWMAX_LPC_DID		 	0x0f1c
#define BIT(nr)                 (1UL << (nr))

/* BYT_CONF0_REG register bits */
#define BYT_IODEN		BIT(31)
#define BYT_DIRECT_IRQ_EN	BIT(27)
#define BYT_TRIG_NEG		BIT(26)
#define BYT_TRIG_POS		BIT(25)
#define BYT_TRIG_LVL		BIT(24)
#define BYT_PULL_STR_SHIFT	9
#define BYT_PULL_STR_MASK	(3 << BYT_PULL_STR_SHIFT)
#define BYT_PULL_STR_2K		(0 << BYT_PULL_STR_SHIFT)
#define BYT_PULL_STR_10K	(1 << BYT_PULL_STR_SHIFT)
#define BYT_PULL_STR_20K	(2 << BYT_PULL_STR_SHIFT)
#define BYT_PULL_STR_40K	(3 << BYT_PULL_STR_SHIFT)
#define BYT_PULL_ASSIGN_SHIFT	7
#define BYT_PULL_ASSIGN_MASK	(3 << BYT_PULL_ASSIGN_SHIFT)
#define BYT_PULL_ASSIGN_UP	(1 << BYT_PULL_ASSIGN_SHIFT)
#define BYT_PULL_ASSIGN_DOWN	(2 << BYT_PULL_ASSIGN_SHIFT)
#define BYT_PIN_MUX		0x07

/* BYT_VAL_REG register bits */
#define BYT_INPUT_EN		BIT(2)  /* 0: input enabled (active low)*/
#define BYT_OUTPUT_EN		BIT(1)  /* 0: output enabled (active low)*/
#define BYT_LEVEL		BIT(0)

#define BYT_DIR_MASK		(BIT(1) | BIT(2))
#define BYT_TRIG_MASK		(BIT(26) | BIT(25) | BIT(24))

#define BYT_INT_STAT_REG0	0x800

/* map pin number to memory mapped register offset */
int pin_mapping[27] = { -1/* no pin 0 */, -1/* GND */, -1/* GND */,
	-1/* +5V */, -1/* +3.3V */, 0x110, 0x10, 0x120, 0x20, 0x130, 0x40,
	0x100, 0x0, 0x140, 0xd0, 0x150, 0xc0, 0x70, 0xe0, 0x60, 0xf0,
	0x21d0, 0xa0, 0x2210, 0xb0, 0x21e0, 0x670/* not confirmed */ };

static pci_device lpc_pci_device;
static void *gpio_virt_base;
void * ilb_virt_base; 

static inline void
minnowmax_gpio_write_r(u32 val , u32 reg)
{
  *(u32 *)((u32)gpio_virt_base + reg) = val;
}

static inline u32
minnowmax_gpio_read_r(u32 reg)
{
  return *(u32 *)((u32)gpio_virt_base + reg);
}

static uint32
dummy_irq_handler(uint8 vec)
{
	DLOG("interrupt");
	return 0;
}

int
byt_gpio_get(uint32 pin)
{
	int conf_reg = pin_mapping[pin];
	int reg = conf_reg + 0x8;

	if (conf_reg < 0) {
		logger_printf("this is not a GPIO pin\n");
		return -1;
	}

	return minnowmax_gpio_read_r(reg) & BYT_LEVEL;
}

void
byt_gpio_set(uint32 pin, int value)
{
	int conf_reg = pin_mapping[pin];
	int reg = conf_reg + 0x8;
	u32 reg_val;

	if (conf_reg < 0) {
		logger_printf("this is not a GPIO pin\n");
		return;
	}

	reg_val = minnowmax_gpio_read_r(reg);

	if (value)
		minnowmax_gpio_write_r(reg_val | BYT_LEVEL, reg);
	else
		minnowmax_gpio_write_r(reg_val & ~BYT_LEVEL, reg);
}

int 
byt_gpio_direction_input(uint32 pin)
{
	int conf_reg = pin_mapping[pin];
	int reg = conf_reg + 0x8;
	u32 reg_val;

	if (conf_reg < 0) {
		logger_printf("this is not a GPIO pin\n");
		return -1;
	}

	/* set pin function to GPIO */
	reg_val = minnowmax_gpio_read_r(conf_reg);
	DLOG("CONF REG is 0x%x", reg_val);
	minnowmax_gpio_write_r(reg_val & ~BIT(0), conf_reg);

	/* enable input alone */
	reg_val = minnowmax_gpio_read_r(reg) | BYT_DIR_MASK;
	reg_val &= ~BYT_INPUT_EN;		/* active low */

	minnowmax_gpio_write_r(reg_val, reg);

	return 0;
}

int
byt_gpio_direction_output(u32 pin, u32 value)
{
	int conf_reg = pin_mapping[pin];
	int reg = conf_reg + 0x8;
	u32 reg_val;

	if (conf_reg < 0) {
		logger_printf("this is not a GPIO pin\n");
		return -1;
	}

	/* set pin function to GPIO */
	reg_val = minnowmax_gpio_read_r(conf_reg);
	DLOG("CONF REG is 0x%x", reg_val);
	minnowmax_gpio_write_r(reg_val & ~BIT(0), conf_reg);

	/* enable output alone */
	reg_val = minnowmax_gpio_read_r(reg) | BYT_DIR_MASK;
	//reg_val &= ~(BYT_OUTPUT_EN | BYT_INPUT_EN);
	reg_val &= ~BYT_OUTPUT_EN; 

	if (value)
		minnowmax_gpio_write_r(reg_val | BYT_LEVEL, reg);
	else
		minnowmax_gpio_write_r(reg_val & ~BYT_LEVEL, reg);

	return 0;
}

typedef enum {
	LEVEL = 0,
	EDGE,
} interrupt_type;

typedef enum {
	ACTIVE_LOW = 0,
	ACTIVE_HIGH,
	FALLING_EDGE,
	RISING_EDGE,
} interrupt_polarity;

int 
byt_gpio_set_interrupt_type(int pin, interrupt_type type)
{
	int conf_reg = pin_mapping[pin];
	u32 reg_val;

	if (conf_reg < 0) {
		logger_printf("this is not a GPIO pin\n");
		return -1;
	}

	reg_val = minnowmax_gpio_read_r(conf_reg);
	DLOG("CONF REG is 0x%x", reg_val);
	if (type)
		reg_val &= ~(1 << 24);
	else 
		reg_val |= (1 << 24);
	minnowmax_gpio_write_r(reg_val, conf_reg);

	return 0;
}

int
byt_gpio_set_interrupt_polarity(int pin, interrupt_polarity polarity)
{
	int conf_reg = pin_mapping[pin];
	u32 reg_val;

	if (conf_reg < 0) {
		logger_printf("this is not a GPIO pin\n");
		return -1;
	}

	reg_val = minnowmax_gpio_read_r(conf_reg);
	DLOG("CONF REG is 0x%x", reg_val);

	switch (polarity) {
		case RISING_EDGE:
		case ACTIVE_HIGH:
			reg_val |= (1 << 25);
			break;
		case FALLING_EDGE:
		case ACTIVE_LOW:
			reg_val |= (1 << 26);
			break;
	}
	minnowmax_gpio_write_r(reg_val, conf_reg);

	return 0;
}

bool
minnowmax_gpio_init()
{
	uint device_index, irq_line, irq_pin;
	pci_irq_t irq;

	if (!pci_find_device(MINNOWMAX_LPC_VID, MINNOWMAX_LPC_DID,
				0xFF, 0xFF, 0, &device_index))
		return FALSE;
	if (device_index == (uint)(~0)) {
    DLOG ("Unable to detect compatible device.");
    return FALSE;
  }
	DLOG ("Found device_index=%d", device_index);

	if (!pci_get_device(device_index, &lpc_pci_device)) {
		DLOG("Unable to get PCI device from PCI subsystem");
		return FALSE;
	}
  DLOG ("Using PCI bus=%x dev=%x func=%x",
        lpc_pci_device.bus,
        lpc_pci_device.slot,
        lpc_pci_device.func);

#if 0
	irq_pin = 2;
	if (pci_irq_find(lpc_pci_device.bus, lpc_pci_device.slot,
				irq_pin, &irq)) {
		/* use PCI routing table */
    DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi);
		if (!pci_irq_map_handler(&irq, dummy_irq_handler, get_logical_dest_addr(0),
					IOAPIC_DESTINATION_LOGICAL,
					IOAPIC_DELIVERY_FIXED))
			goto abort;
		irq_line = irq.gsi;
	}
  DLOG ("Using IRQ line=%.02X pin=%X", irq_line, irq_pin);
#endif

	//get GPIO memory-mapped BAR
	uint gpio_mem_addr = READ(lpc_pci_device.bus, lpc_pci_device.slot,
			lpc_pci_device.func, 0x4C, dword);
	if (gpio_mem_addr == 0) {
    DLOG ("GPIO unable to detect memory mapped IO region");
    return FALSE;
  } 
	gpio_mem_addr &= 0xFFFFC000;
	gpio_virt_base = map_virtual_page (gpio_mem_addr | 0x3);
  if (gpio_virt_base == NULL) {
    DLOG ("GPIO unable to map page to phys=%p", gpio_mem_addr);
    return FALSE;
  }
  DLOG ("GPIO using memory mapped IO at phys=%p virt=%p", gpio_mem_addr, gpio_virt_base);

#if 0
	//get GPIO IO port 
	uint gpio_io_port = READ(lpc_pci_device.bus, lpc_pci_device.slot,
			lpc_pci_device.func, 0x48, dword);
	gpio_io_port &= 0x0000FF00;
  DLOG ("GPIO using IO port=0x%x", gpio_io_port);

	//get PMC BAR
	uint pmc_mem_addr = READ(lpc_pci_device.bus, lpc_pci_device.slot,
			lpc_pci_device.func, 0x44, dword);
	if (pmc_mem_addr == 0) {
    DLOG ("PMC unable to detect memory mapped IO region");
    return FALSE;
  } 
	pmc_mem_addr &= 0xFFFFFE00;
	void * pmc_virt_base = map_virtual_page (pmc_mem_addr | 0x3);
  if (pmc_virt_base == NULL) {
    DLOG ("PMC unable to map page to phys=%p", pmc_mem_addr);
    return FALSE;
  }
  DLOG ("PMC using memory mapped IO at phys=%p virt=%p", pmc_mem_addr, pmc_virt_base);

	//set GPE to edge trigger
	*(uint *)((uint)pmc_virt_base + 0xC4) = 0x0;
	
	//set GPIO_ROUT
	*(uint *)((uint)pmc_virt_base + 0x58) = 0xaaaaaaaa;

	//get ACPI BAR
	uint acpi_io_port;
	acpi_io_port = READ(lpc_pci_device.bus, lpc_pci_device.slot,
			lpc_pci_device.func, 0x40, dword);
	if (acpi_io_port == 0) {
    DLOG ("APCI unable to detect memory mapped IO region");
    return FALSE;
  } 
	acpi_io_port &= 0x0000FF80;
  DLOG ("ACPI using IO port at phys=%p", acpi_io_port);

	//set CORE GPIO ENable
	uint core_gpio_en = inl(acpi_io_port + 0x28);
	outl(0xFF000000 | core_gpio_en, acpi_io_port + 0x28);

	//set PM1_CNT.SCI_EN
	uint pm1_cnt = inl(acpi_io_port + 0x4);
	outl(0x1 | pm1_cnt, acpi_io_port + 0x4);
	//set GPE to EDGE trigger, polarity LOW 
	*(uint *)((uint)pmc_virt_base + 0xC4) = 0x0;
	*(uint *)((uint)pmc_virt_base + 0xC8) = 0x0;
	
	//get iLB BAR
	uint32 ilb_mem_addr = READ(lpc_pci_device.bus, lpc_pci_device.slot,
			lpc_pci_device.func, 0x50, dword);
	if (ilb_mem_addr == 0) {
    DLOG ("ILB unable to detect memory mapped IO region");
    return FALSE;
  } 
	ilb_mem_addr &= 0xFFFFFE00;
	ilb_virt_base = map_virtual_page (ilb_mem_addr | 0x3);
	if (ilb_virt_base == NULL) {
    DLOG ("ILB unable to map page to phys=%p", ilb_mem_addr);
    return FALSE;
  }
  DLOG ("ILB using memory mapped IO at phys=%p virt=%p", ilb_mem_addr, ilb_virt_base);

	//set actl to 0b101
	*(uint *)((uint)ilb_virt_base + 0x00) = 0x4;
	
	//disable 8259
	*(uint *)((uint)ilb_virt_base + 0x04) = 0xE;

	//diable IOAPIC
	//uint32 ioapic_en = *(uint *)((uint)ilb_virt_base + 0x60);
	//*(uint *)((uint)ilb_virt_base + 0x60) = ioapic_en & 0xFFFFFEFF;

	//stop triggering IRQ11 and IRQ12
	*(uint *)((uint)ilb_virt_base + 0x70) = 0x0;
	//uint32 uart_en = *(uint *)((uint)ilb_virt_base + 0x88);
	//printf("uart_en is 0x%x\n", uart_en);
	//*(uint *)((uint)ilb_virt_base + 0x88) = 0xFFFFFFFF;
#endif

#if 0
	/* clear interrupt status register */
	minnowmax_gpio_write_r(0xFFFFFFFF, BYT_INT_STAT_REG0);
	minnowmax_gpio_write_r(0xFFFFFFFF, BYT_INT_STAT_REG0 + 0x4);
	minnowmax_gpio_write_r(0xFFFFFFFF, BYT_INT_STAT_REG0 + 0x8);
	minnowmax_gpio_write_r(0xFFFFFFFF, BYT_INT_STAT_REG0 + 0xc);

	/* enable and use I/O mapped registers */
	/* select IO-mapped regs */
	uint reg_val;
	outl(0x1000, gpio_io_port + 0x40); 
	reg_val = inl(gpio_io_port + 0x40);
  DLOG ("reg_val=0x%x", reg_val);

	/* set to input mode */
	/* enable UART1_CTS */
	outl(0x1000, gpio_io_port + 0x44); 
	reg_val = inl(gpio_io_port + 0x44);
  DLOG ("reg_val=0x%x", reg_val);

	/* drive high */
	//outl(0x00000000, gpio_io_port + 0x48); 
	
	/* enable trigger positive edge */
	outl(0x1000, gpio_io_port + 0x4C); 
	reg_val = inl(gpio_io_port + 0x4C);
  DLOG ("reg_val=0x%x", reg_val);

	/* enable trigger negative edge */
	outl(0x1000, gpio_io_port + 0x50); 
	reg_val = inl(gpio_io_port + 0x50);
  DLOG ("reg_val=0x%x", reg_val);


	//byt_gpio_direction_input(10);
	//byt_gpio_set_interrupt_type(10, EDGE);
	//byt_gpio_set_interrupt_polarity(10, RISING_EDGE);
	//byt_gpio_set_interrupt_polarity(10, FALLING_EDGE);
#endif
	
#if 0
	//this function call alone is sufficient to change the value of a pin
	//byt_gpio_direction_output(10, 1);
	while(1);
#endif

	/* register functions to gpio framwork */
	extern struct gpio_ops gops;
	gops.set_value = byt_gpio_set;
	gops.get_value = byt_gpio_get;
	gops.set_output = byt_gpio_direction_output;
	gops.set_input = byt_gpio_direction_input;

	return TRUE;

abort:
	unmap_virtual_page(gpio_virt_base);
	return FALSE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = minnowmax_gpio_init
};

#ifdef MINNOWMAX
DEF_MODULE (minnowmax_quark_gpio, "MinnowBoard Max GPIO driver", &mod_ops, {"pci"});
#endif



