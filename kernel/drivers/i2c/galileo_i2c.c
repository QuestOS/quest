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

/* some parts are copied from linux
 * i2c-designware-core.c
 * i2c-core.c */

#include <drivers/pci/pci.h>
#include <util/printf.h>
#include <mem/mem.h>
#include <sched/sched.h>
#include <sched/vcpu.h>

//#define DEBUG_I2C 

#ifdef DEBUG_I2C
#define DLOG(fmt,...) DLOG_PREFIX("I2C",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/* Device */

/*
 * Registers offset
 */
#define DW_IC_CON		0x0
#define DW_IC_TAR		0x4
#define DW_IC_DATA_CMD		0x10
#define DW_IC_SS_SCL_HCNT	0x14
#define DW_IC_SS_SCL_LCNT	0x18
#define DW_IC_FS_SCL_HCNT	0x1c
#define DW_IC_FS_SCL_LCNT	0x20
#define DW_IC_INTR_STAT		0x2c
#define DW_IC_INTR_MASK		0x30
#define DW_IC_RAW_INTR_STAT	0x34
#define DW_IC_RX_TL		0x38
#define DW_IC_TX_TL		0x3c
#define DW_IC_CLR_INTR		0x40
#define DW_IC_CLR_RX_UNDER	0x44
#define DW_IC_CLR_RX_OVER	0x48
#define DW_IC_CLR_TX_OVER	0x4c
#define DW_IC_CLR_RD_REQ	0x50
#define DW_IC_CLR_TX_ABRT	0x54
#define DW_IC_CLR_RX_DONE	0x58
#define DW_IC_CLR_ACTIVITY	0x5c
#define DW_IC_CLR_STOP_DET	0x60
#define DW_IC_CLR_START_DET	0x64
#define DW_IC_CLR_GEN_CALL	0x68
#define DW_IC_ENABLE		0x6c
#define DW_IC_STATUS		0x70
#define DW_IC_TXFLR		0x74
#define DW_IC_RXFLR		0x78
#define DW_IC_TX_ABRT_SOURCE	0x80

static u32 tx_fifo_depth = 16;
static u32 rx_fifo_depth = 1;
static u32 clk_khz = 33000;
static void *mmio_base;

static inline void
i2c_write_r(u32 val , u32 reg)
{
  *(u32 *)((u32)mmio_base + reg) = val;
}

static inline u32
i2c_read_r(u32 reg)
{
  return *(u32 *)((u32)mmio_base + reg);
}

/* copied from clanton to configure clock */
static u32
i2c_dw_scl_hcnt(u32 ic_clk, u32 tSYMBOL, u32 tf, int cond, int offset)
{
	/*
	 * DesignWare I2C core doesn't seem to have solid strategy to meet
	 * the tHD;STA timing spec.  Configuring _HCNT based on tHIGH spec
	 * will result in violation of the tHD;STA spec.
	 */
	if (cond)
		/*
		 * Conditional expression:
		 *
		 *   IC_[FS]S_SCL_HCNT + (1+4+3) >= IC_CLK * tHIGH
		 *
		 * This is based on the DW manuals, and represents an ideal
		 * configuration.  The resulting I2C bus speed will be
		 * faster than any of the others.
		 *
		 * If your hardware is free from tHD;STA issue, try this one.
		 */
		return (ic_clk * tSYMBOL + 5000) / 10000 - 8 + offset;
	else
		/*
		 * Conditional expression:
		 *
		 *   IC_[FS]S_SCL_HCNT + 3 >= IC_CLK * (tHD;STA + tf)
		 *
		 * This is just experimental rule; the tHD;STA period turned
		 * out to be proportinal to (_HCNT + 3).  With this setting,
		 * we could meet both tHIGH and tHD;STA timing specs.
		 *
		 * If unsure, you'd better to take this alternative.
		 *
		 * The reason why we need to take into account "tf" here,
		 * is the same as described in i2c_dw_scl_lcnt().
		 */
		return (ic_clk * (tSYMBOL + tf) + 5000) / 10000 - 3 + offset;
}

static u32 i2c_dw_scl_lcnt(u32 ic_clk, u32 tLOW, u32 tf, int offset)
{
	/*
	 * Conditional expression:
	 *
	 *   IC_[FS]S_SCL_LCNT + 1 >= IC_CLK * (tLOW + tf)
	 *
	 * DW I2C core starts counting the SCL CNTs for the LOW period
	 * of the SCL clock (tLOW) as soon as it pulls the SCL line.
	 * In order to meet the tLOW timing spec, we need to take into
	 * account the fall time of SCL signal (tf).  Default tf value
	 * should be 0.3 us, for safety.
	 */
	return ((ic_clk * (tLOW + tf) + 5000) / 10000) - 1 + offset;
}

/* IC_CON register */
#define DW_IC_CON_MASTER    0x1
#define DW_IC_CON_SPEED_STD   0x2
#define DW_IC_CON_SPEED_FAST    0x4
#define DW_IC_CON_10BITADDR_MASTER  0x10
#define DW_IC_CON_RESTART_EN    0x20

static void
i2c_config()
{
  u32 input_clock_khz = clk_khz;

  /* set standard and fast speed deviders for high/low periods */
  u32 hcnt, lcnt;

	/* Standard-mode */
  hcnt = i2c_dw_scl_hcnt(input_clock_khz,
				40,	/* tHD;STA = tHIGH = 4.0 us */
				3,	/* tf = 0.3 us */
				0,	/* 0: DW default, 1: Ideal */
				0);	/* No offset */
	lcnt = i2c_dw_scl_lcnt(input_clock_khz,
				47,	/* tLOW = 4.7 us */
				3,	/* tf = 0.3 us */
				0);	/* No offset */
  i2c_write_r(hcnt, DW_IC_SS_SCL_HCNT);
  i2c_write_r(lcnt, DW_IC_SS_SCL_LCNT);

	/* Fast-mode */
  hcnt = i2c_dw_scl_hcnt(input_clock_khz,
				6,	/* tHD;STA = tHIGH = 0.6 us */
				3,	/* tf = 0.3 us */
				0,	/* 0: DW default, 1: Ideal */
				0);	/* No offset */
	lcnt = i2c_dw_scl_lcnt(input_clock_khz,
				13,	/* tLOW = 1.3 us */
				3,	/* tf = 0.3 us */
				0);	/* No offset */
  i2c_write_r(hcnt, DW_IC_FS_SCL_HCNT);
	i2c_write_r(lcnt, DW_IC_FS_SCL_LCNT);

	/* Configure Tx/Rx FIFO threshold levels */
	/* XXX: currently we only allow byte_write and byte_read.
	 * So each transaction has 2 bytes to write.
	 * We set threshold = (tx_fifo_depth - 2) so that when TX_EMPTY
	 * interrupt occurs, we are guaranteed to have enough tx fifo slot
	 * for one transaction */
  i2c_write_r(tx_fifo_depth - 2, DW_IC_TX_TL); 
  i2c_write_r(rx_fifo_depth - 1, DW_IC_RX_TL); 

  u32 cfg = DW_IC_CON_MASTER | DW_IC_CON_RESTART_EN | DW_IC_CON_SPEED_STD;
  /* configure the i2c master */
  i2c_write_r(cfg, DW_IC_CON);
}

#define DW_IC_INTR_RX_UNDER	0x001
#define DW_IC_INTR_RX_OVER	0x002
#define DW_IC_INTR_RX_FULL	0x004
#define DW_IC_INTR_TX_OVER	0x008
#define DW_IC_INTR_TX_EMPTY	0x010
#define DW_IC_INTR_RD_REQ	0x020
#define DW_IC_INTR_TX_ABRT	0x040
#define DW_IC_INTR_RX_DONE	0x080
#define DW_IC_INTR_ACTIVITY	0x100
#define DW_IC_INTR_STOP_DET	0x200
#define DW_IC_INTR_START_DET	0x400
#define DW_IC_INTR_GEN_CALL	0x800

#define DW_IC_INTR_DEFAULT_MASK		(DW_IC_INTR_RX_FULL | \
					 DW_IC_INTR_TX_EMPTY | \
					 DW_IC_INTR_TX_ABRT)

static inline void
i2c_disable_int()
{
  i2c_write_r(0, DW_IC_INTR_MASK);
}

static inline void
i2c_clear_int()
{
  i2c_read_r(DW_IC_CLR_INTR);
}

static inline void
i2c_disable()
{
  i2c_write_r(0, DW_IC_ENABLE);
}

static inline void
i2c_enable()
{
  i2c_write_r(1, DW_IC_ENABLE);
}

static inline u32
i2c_int_stat()
{
	return i2c_read_r(DW_IC_INTR_STAT);
}

#define DW_IC_CMD_WRITE 		0x000
#define DW_IC_CMD_READ			0x100
#define DW_IC_CMD_STOP			0x200
#define DW_IC_CMD_RESTART		0x400

#define STATUS_RFNE 0x8
#define STATUS_TFNF 0x2

static void wait_rx()
{
	u32 val;
	do {
		val = i2c_read_r(DW_IC_STATUS);
		DLOG("waiting rx...");
	} while (!(val & STATUS_RFNE));
}

static void wait_tx()
{
	u32 val;
	do {
		val = i2c_read_r(DW_IC_STATUS);
		DLOG("waiting tx...");
	} while (!(val & STATUS_TFNF));
}

typedef enum {
	READ = 0,
	WRITE,
} i2c_trans_type;

typedef enum {
	WRITE_PENDING = 0,
	READ_PENDING,
	DONE,
} i2c_trans_status;

struct i2c_trans {
	i2c_trans_type type;
	i2c_trans_status status;
	u32 ispending;
	u32 data_w[2];
	u32 data_r;
};

static task_id i2c_sleep_queue = 0;
struct i2c_trans i2c_dev_buffer;
semaphore i2c_dev_mtx;
#define _mutex_init(mtx) semaphore_init(mtx, 1, 1) 
#define _mutex_destory(mtx) semaphore_destroy(mtx) 
#define _mutex_lock(mtx) semaphore_wait(mtx, 1, -1) 
#define _mutex_unlock(mtx) semaphore_signal(mtx, 1) 

u8 i2c_read_byte_data(u8 reg)
{
	u32 val1, val2, retval;

	val1 = reg | DW_IC_CMD_WRITE | DW_IC_CMD_RESTART;
	val2 = DW_IC_CMD_READ | DW_IC_CMD_STOP | DW_IC_CMD_RESTART;
	/* --TOM--
	 * mp_enabled can infer weather we are here from
	 * user process or still doing kernel
	 * initialization. If from user process, do a sleep
	 * to block the process and wait for interrupts.
	 * Otherwise, do a polling */
	if (!mp_enabled) {
		wait_tx();
		i2c_write_r(val1, DW_IC_DATA_CMD);
		wait_tx();
		i2c_write_r(val2, DW_IC_DATA_CMD);
		wait_rx();
		retval = i2c_read_r(DW_IC_DATA_CMD);
	} else {
		//lock i2c device and write device-global buffer
		_mutex_lock(&i2c_dev_mtx);
		i2c_dev_buffer.type = READ;
		i2c_dev_buffer.status = WRITE_PENDING,
		i2c_dev_buffer.data_w[0] = val1;
		i2c_dev_buffer.data_w[1] = val2;
		queue_append(&i2c_sleep_queue, str());
		i2c_write_r(DW_IC_INTR_DEFAULT_MASK, DW_IC_INTR_MASK);
		/* We won't receive interrupts until schedule() is called
		 * because kernel will never be interrupted */
		DLOG("about to sleep");
		schedule();
		retval = i2c_dev_buffer.data_r;
		_mutex_unlock(&i2c_dev_mtx);
	}
	return (retval & 0xFF);
}

s32 i2c_write_byte_data(u8 reg, u8 data)
{
	u32 val1, val2;

	val1 = reg | DW_IC_CMD_WRITE | DW_IC_CMD_RESTART;
	val2 = data | DW_IC_CMD_WRITE | DW_IC_CMD_STOP;
	if (!mp_enabled) {
		wait_tx();
		i2c_write_r(val1, DW_IC_DATA_CMD);
		wait_tx();
		i2c_write_r(val2, DW_IC_DATA_CMD);
	} else {
		//lock i2c device
		_mutex_lock(&i2c_dev_mtx);
		i2c_dev_buffer.type = WRITE;
		i2c_dev_buffer.status = WRITE_PENDING,
		i2c_dev_buffer.data_w[0] = val1;
		i2c_dev_buffer.data_w[1] = val2;
		queue_append(&i2c_sleep_queue, str());
		i2c_write_r(DW_IC_INTR_DEFAULT_MASK, DW_IC_INTR_MASK);
		DLOG("about to sleep");
		schedule();
		_mutex_unlock(&i2c_dev_mtx);
	}
	return 0;
}

static inline void
i2c_write()
{
	if (i2c_dev_buffer.status != WRITE_PENDING) {
		return;
	}
	/* we set DW_IC_TX_TL to the value that we are guaranteed
	 * to finish a transaction's write in one TX_EMPTY interrupt.
	 * So mask TX_EMPTY out. It is the next transaction who
	 * will set it on again */
	i2c_write_r((DW_IC_INTR_DEFAULT_MASK & ~DW_IC_INTR_TX_EMPTY),
			DW_IC_INTR_MASK);
	DLOG("data_w[0] is 0x%x", i2c_dev_buffer.data_w[0]);
	DLOG("data_w[1] is 0x%x", i2c_dev_buffer.data_w[1]);
	i2c_write_r(i2c_dev_buffer.data_w[0], DW_IC_DATA_CMD);
	i2c_write_r(i2c_dev_buffer.data_w[1], DW_IC_DATA_CMD);
	i2c_dev_buffer.status = (i2c_dev_buffer.type == READ) ?
		READ_PENDING : DONE;
}

static inline void
i2c_read()
{
	if (i2c_dev_buffer.status != READ_PENDING)
		return;
	i2c_dev_buffer.data_r = i2c_read_r(DW_IC_DATA_CMD);
	i2c_dev_buffer.status = DONE;
}

static uint32
i2c_irq_handler(uint8 vec)
{
	u32 int_stat = i2c_int_stat();
	DLOG("IRQ coming..., int_status is 0x%x", int_stat);
	if (int_stat & DW_IC_INTR_RX_FULL) {
		i2c_read();
		goto done;
	}
	else if (int_stat & DW_IC_INTR_TX_EMPTY) {
		DLOG("type is %d", i2c_dev_buffer.type);
		i2c_write();
		/* we are probably not done yet.
		 * If we are doing a read, we need to
		 * keep interrupt enabled so that
		 * RX_FULL will be handled */
		if (i2c_dev_buffer.type == READ)
			return 0;
		goto done;
	}
	else {
		i2c_clear_int();
		return 0;
	}

done:
	i2c_disable_int();
	DLOG("about to wakeup");
	wakeup_queue(&i2c_sleep_queue);
	return 0;
}

/* It will be called by cy8c9540a intialization code
 * So polling will be able to work. But interrupt will
 * be enabled only when user programs call i2c_read_byte_data()
 * or i2c_write_byte_data() 
 */
void i2c_xfer_init(u32 slave_addr)
{
  i2c_write_r(slave_addr, DW_IC_TAR);
	i2c_enable();
}

#define GALILEO_I2C_VID		 		0x8086
#define	GALILEO_I2C_DID			 	0x0934

static pci_device i2c_pci_device;

bool i2c_init()
{
	uint device_index, irq_line, irq_pin;
	uint mem_addr;
	pci_irq_t irq;

	if (!pci_find_device(GALILEO_I2C_VID, GALILEO_I2C_DID,
				0xFF, 0xFF, 0, &device_index))
		return FALSE;
	if (device_index == (uint)(~0)) {
    DLOG ("Unable to detect compatible device.");
    return FALSE;
  }
  DLOG ("Found device_index=%d", device_index);

	if (!pci_get_device(device_index, &i2c_pci_device)) {
		DLOG("Unable to get PCI device from PCI subsystem");
		return FALSE;
	}
  DLOG ("Using PCI bus=%x dev=%x func=%x",
        i2c_pci_device.bus,
        i2c_pci_device.slot,
        i2c_pci_device.func);

	if (!pci_decode_bar (device_index, 0, &mem_addr, NULL, NULL)) {
    DLOG ("Invalid PCI configuration or BAR0 not found");
    return FALSE;
  } 
  if (mem_addr == 0) {
    DLOG ("Unable to detect memory mapped IO region");
    return FALSE;
  }
  mmio_base = map_virtual_page (mem_addr | 0x3);
  if (mmio_base == NULL) {
    DLOG ("Unable to map page to phys=%p", mem_addr);
    return FALSE;
  }
  DLOG ("Using memory mapped IO at phys=%p virt=%p", mem_addr, mmio_base);

	if (!pci_get_interrupt(device_index, &irq_line, &irq_pin)) {
		DLOG("Unable to get IRQ");
		goto abort;
	}
	if (pci_irq_find(i2c_pci_device.bus, i2c_pci_device.slot,
				irq_pin, &irq)) {
		/* use PCI routing table */
    DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi);
		if (!pci_irq_map_handler(&irq, i2c_irq_handler, get_logical_dest_addr(0),
					IOAPIC_DESTINATION_LOGICAL,
					IOAPIC_DELIVERY_FIXED))
			goto abort;
		irq_line = irq.gsi;
	}
  DLOG ("Using IRQ line=%.02X pin=%X", irq_line, irq_pin);

	_mutex_init(&i2c_dev_mtx);
	i2c_disable();
  i2c_config();
  i2c_disable_int();
  i2c_clear_int();
	return TRUE;

abort:
	unmap_virtual_page(mmio_base);
	return FALSE;
}

void i2c_remove()
{
	i2c_disable();
	_mutex_destory(&i2c_dev_mtx);
  unmap_virtual_page (mmio_base);
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = i2c_init
};

DEF_MODULE (galileo_i2c, "Galileo I2C driver", &mod_ops, {"pci"});

