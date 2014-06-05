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
#include "drivers/pci/pcidb.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "kernel.h"
#include "mem/mem.h"

#define DEBUG_PCI 

#ifdef DEBUG_PCI
#define DLOG(fmt,...) DLOG_PREFIX("PCI",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif


#define READ(bus, slot, func, reg, type) \
  pci_read_##type (pci_addr (bus, slot, func, reg))
#define WRITE(bus, slot, func, reg, type, val) \
  pci_write_##type (pci_addr (bus, slot, func, reg), val)


bool
pci_search_ven_table (uint32 vendor, PCI_VENTABLE* e)
{
  uint32 i;
  for (i=0; i<PCI_VENTABLE_LEN; i++)
    if (PciVenTable[i].VenId == vendor) {
      *e = PciVenTable[i];
      return TRUE;
    }
  return FALSE;
}

bool
pci_search_dev_table (uint32 vendor, uint32 dev, PCI_DEVTABLE* e)
{
  uint32 i;
  for (i=0; i<PCI_DEVTABLE_LEN; i++)
    if (PciDevTable[i].VenId == vendor && PciDevTable[i].DevId == dev) {
      *e = PciDevTable[i];
      return TRUE;
    }
  return FALSE;
}

bool
pci_search_class_code_table (uint32 base, uint32 sub, uint32 prog,
                             PCI_CLASSCODETABLE* e)
{
  uint32 i;
  for (i=0; i<PCI_CLASSCODETABLE_LEN; i++)
    if (PciClassCodeTable[i].BaseClass == base &&
        PciClassCodeTable[i].SubClass == sub &&
        PciClassCodeTable[i].ProgIf == prog) {
      *e = PciClassCodeTable[i];
      return TRUE;
    }
  return FALSE;
}


/* store table of PCI devices */
static pci_device devices[PCI_MAX_DEVICES];
static uint32 num_devices = 0;


/******************************************************************/
/* i2c master driver */
/* XXX: definitely need a better error handling machanism
 * DLOG() is awful
 */
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

u32 i2c_base;

struct i2c_dev {
  u32 tx_fifo_depth;
  u32 rx_fifo_depth;
  u32 clk_khz;
  u32 rx_outstanding;
};

static struct i2c_dev dev_i2c = {
  .tx_fifo_depth = 16,
  .rx_fifo_depth = 16,
  .clk_khz = 33000,
  .rx_outstanding = 0,
};

static inline void
i2c_write_r(u32 val , u32 reg)
{
  *(u32 *)(i2c_base + reg) = val;
}

static inline u32
i2c_read_r(u32 reg)
{
  return *(u32 *)(i2c_base + reg);
}

/* copy from linux to configure clock */
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

#define DW_IC_CON_MASTER    0x1
#define DW_IC_CON_SPEED_STD   0x2
#define DW_IC_CON_SPEED_FAST    0x4
#define DW_IC_CON_10BITADDR_MASTER  0x10
#define DW_IC_CON_RESTART_EN    0x20
#define DW_IC_CON_SLAVE_DISABLE   0x40

#define INTEL_CLN_STD_CFG  (DW_IC_CON_MASTER |      \
    DW_IC_CON_SLAVE_DISABLE | \
    DW_IC_CON_RESTART_EN)

static void
i2c_config()
{
  u32 input_clock_khz = dev_i2c.clk_khz;

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
  i2c_write_r(dev_i2c.tx_fifo_depth - 1, DW_IC_TX_TL); 
  i2c_write_r(0, DW_IC_RX_TL); 

  /* 
   * copy from clanton. In comment, it says:
   * Clanton default configuration is fast mode, unless otherwise asked.
   * XXX: this is not true... setting fast mode makes it not working
   * XXX: Donno what is DW_IC_CON_SLAVE_DISABLE...
   */
  //u32 cfg = INTEL_CLN_STD_CFG | DW_IC_CON_SPEED_FAST;
  u32 cfg = INTEL_CLN_STD_CFG | DW_IC_CON_SPEED_STD;
  /* configure the i2c master */
  i2c_write_r(cfg, DW_IC_CON);
}

static void
i2c_disable_int()
{
  i2c_write_r(0, DW_IC_INTR_MASK);
}

static void
i2c_clear_int()
{
  i2c_read_r(DW_IC_CLR_INTR);
}

static void
i2c_disable()
{
  i2c_write_r(0, DW_IC_ENABLE);
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
					 DW_IC_INTR_TX_ABRT | \
					 DW_IC_INTR_STOP_DET)

static void i2c_xfer_init(u32 slave_addr)
{
  i2c_disable();

  /* set the slave (target) address */
  i2c_write_r(slave_addr, DW_IC_TAR);

	/* Enable the adapter */
	i2c_write_r(1, DW_IC_ENABLE);

	/* XXX: Enable interrupts */
	/* i2c_write_r(DW_IC_INTR_DEFAULT_MASK, DW_IC_INTR_MASK); */
}

static u32
i2c_status()
{
  return i2c_read_r(DW_IC_STATUS);
}

struct i2c_msg {
	u16 flags;
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_STOP		0x8000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_NOSTART */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
	u16 len;		/* msg length				*/
	u8 *buf;		/* pointer to msg data			*/
};

#define DW_IC_CMD_READ			0x01
#define DW_IC_CMD_STOP			0x02
#define DW_IC_CMD_RESTART		0x04

/*
 * Define the IC_DATA_CMD format.
 */
static union i2c_dw_data_cmd {
	struct fields {
		u8 data;
		u8 cmd;
	} fields;
	u16 value;
} data_cmd;

static void
i2c_write(struct i2c_msg *msgs, int msg_num)
{
  u32 buf_len;
  u8 *buf;
  int i, segment_start;
  /* reset pending RX */
  dev_i2c.rx_outstanding = 0;

  for (i = 0; i < msg_num; i++) {
    segment_start = 1;
    buf_len = msgs[i].len;
    buf = msgs[i].buf;

    while (buf_len > 0) {
      data_cmd.fields.data = 0x00;
      data_cmd.fields.cmd = 0x00;

      if (msgs[i].flags & I2C_M_RD) {
        data_cmd.fields.cmd = DW_IC_CMD_READ;
        dev_i2c.rx_outstanding++;
      } else {
        data_cmd.fields.data = *buf;
        buf++;
      }

      if (segment_start) {
        /*
         * First byte of a transaction segment for a
         * device requiring explicit transaction
         * termination: generate (re)start symbol.
         */
        segment_start = 0;
        data_cmd.fields.cmd |= DW_IC_CMD_RESTART;
      }

      if (i == msg_num - 1 && buf_len == 1) {
        /*
         * Last byte of last transction segment for a
         * device requiring explicit transaction
         * termination: generate stop symbol.
         */
        data_cmd.fields.cmd |= DW_IC_CMD_STOP;
      }

      DLOG("data_cmd.value is %x\n", data_cmd.value);
      i2c_write_r(data_cmd.value, DW_IC_DATA_CMD);
      buf_len--;
    }
  }
}

static void
i2c_read(struct i2c_msg *msgs, int msg_num)
{
  int i, rx_valid;
  u32 buf_len;
  u8 *buf;

  for (i = 0; i < msg_num; i++) {
    if (!(msgs[i].flags & I2C_M_RD)) 
      continue;

    buf_len = msgs[i].len;
    buf = msgs[i].buf;

    while (dev_i2c.rx_outstanding > 0) {
      /* more read requests need to be handled */
      rx_valid = i2c_read_r(DW_IC_RXFLR);

      for (; buf_len > 0 && rx_valid > 0; buf_len--, rx_valid--) {
        u8 val = (u8)i2c_read_r(DW_IC_DATA_CMD);
        *buf++ = val;
        dev_i2c.rx_outstanding--;
      }
    }
  }
}

/******************************************************************/

/******************************************************************/
/* i2c protocol */

/*
 * Data for Messages
 * copy from clanton
 */
#define I2C_BLOCK_MAX	32	/* As specified in SMBus standard */
union i2c_data {
	u8 byte;
	u16 word;
	u8 block[I2C_BLOCK_MAX + 2]; /* block[0] is used for length */
			       /* and one more for user-space compatibility */
};

/* transaction types */
#define I2C_BYTE_DATA	    0
#define I2C_WORD_DATA	    1
#define I2C_BLOCK_DATA    2

/* i2c_xfer read or write markers */
#define I2C_READ	1
#define I2C_WRITE	0

static s32 i2c_xfer(unsigned short flags, char read_write,
    u8 command, int size, union i2c_data *data)
{
  unsigned char msgbuf0[I2C_BLOCK_MAX+3];
  unsigned char msgbuf1[I2C_BLOCK_MAX+2];
	int num = read_write == I2C_READ ? 2 : 1;
	int i;
  struct i2c_msg msg[2] = {
    {
      .flags = flags,
      .len = 1,
      .buf = msgbuf0,
    }, {
      .flags = flags | I2C_M_RD,
      .len = 0,
      .buf = msgbuf1,
    },
  };

	msgbuf0[0] = command;
  switch (size) {
  case I2C_BYTE_DATA:
		if (read_write == I2C_READ)
			msg[1].len = 1;
		else {
			msg[0].len = 2;
			msgbuf0[1] = data->byte;
		}
		break;
  case I2C_BLOCK_DATA:
		if (read_write == I2C_READ) {
			msg[1].len = data->block[0];
		} else {
			msg[0].len = data->block[0] + 1;
			if (msg[0].len > I2C_BLOCK_MAX + 1) {
        DLOG("Invalid block write size %d\n",
             data->block[0]);
        return -1;
			}
			for (i = 1; i <= data->block[0]; i++)
				msgbuf0[i] = data->block[i];
		}
		break;
  case I2C_WORD_DATA:
		if (read_write == I2C_READ)
			msg[1].len = 2;
		else {
			msg[0].len = 3;
			msgbuf0[1] = data->word & 0xff;
			msgbuf0[2] = data->word >> 8;
		}
		break;
	default:
    DLOG("Unsupported transaction%d\n",
         size);
    return -1;
	}

  i2c_write(msg, num);

  if (read_write == I2C_READ)
    i2c_read(msg, num);

		switch (size) {
		case I2C_BYTE_DATA:
			data->byte = msgbuf1[0];
			break;
		case I2C_WORD_DATA:
			data->word = msgbuf1[0] | (msgbuf1[1] << 8);
			break;
		case I2C_BLOCK_DATA:
			for (i = 0; i < data->block[0]; i++)
				data->block[i+1] = msgbuf1[i];
			break;
		}

	return 0;
}

s32 i2c_read_byte_data(u8 command)
{
  union i2c_data data;
  s32 status = i2c_xfer(0, I2C_READ, command, I2C_BYTE_DATA, &data);
  if (status < 0) {
    DLOG("i2c_read_byte_data error");
    return -1;
  }
  return data.byte;
}

/* return 0 on success, -1 on failure */
s32 i2c_write_byte_data(u8 command, u8 value)
{
  union i2c_data data;
  data.byte = value;
  s32 status = i2c_xfer(0, I2C_WRITE, command, I2C_BYTE_DATA, &data);
  if (status < 0) {
    DLOG("i2c_write_byte_data error");
    return -1;
  }
  return 0;
}

s32 i2c_read_block_data(u8 command, u8 length, u8 *values)
{
  union i2c_data data;
  int status;

  if (length > I2C_BLOCK_MAX)
    length = I2C_BLOCK_MAX;
  data.block[0] = length;

  status = i2c_xfer(0, I2C_READ, command, I2C_BLOCK_DATA, &data);
  if (status < 0)
    return status;

  memcpy(values, &data.block[1], data.block[0]);
  return data.block[0];
}

s32 i2c_write_block_data(u8 command, u8 length, u8 *values)
{
  union i2c_data data;
  int status;

  if (length > I2C_BLOCK_MAX)
    length = I2C_BLOCK_MAX;
  data.block[0] = length;

  memcpy(&data.block[1], values, length);
  status = i2c_xfer(0, I2C_WRITE, command, I2C_BLOCK_DATA, &data);

  return status;
}
/******************************************************************/

/******************************************************************/
/* cy8c9540 driver */

#define NPORTS				6
#define NPWM          8
#define PWM_CLK				0x00	/* see resulting PWM_TCLK_NS */

/* Register offset  */
#define REG_INPUT_PORT0			0x00
#define REG_OUTPUT_PORT0		0x08
#define REG_INTR_STAT_PORT0		0x10
#define REG_PORT_SELECT			0x18
#define REG_INTR_MASK			0x19
#define REG_SELECT_PWM			0x1a
#define REG_PIN_DIR			0x1c
#define REG_DRIVE_PULLUP		0x1d
#define REG_PWM_SELECT			0x28
#define REG_PWM_CLK			0x29
#define REG_PWM_PERIOD			0x2a
#define REG_PWM_PULSE_W			0x2b
#define REG_ENABLE			0x2d
#define REG_DEVID_STAT			0x2e

#define BIT(nr)			(1UL << (nr))

#define GPIOF_DRIVE_PULLUP	(1 << 6)
#define GPIOF_DRIVE_PULLDOWN	(1 << 7)
#define GPIOF_DRIVE_STRONG	(1 << 8)
#define GPIOF_DRIVE_HIZ		(1 << 9)

/* Per-port GPIO offset */
static const u8 cy8c9540a_port_offs[] = {
	0,
	8,
	16,
	20,
	28,
	36,
};

/* XXX: need to change name */
struct cy8c9540a {
	/* cached output registers */
	u8 outreg_cache[NPORTS];
	/* cached IRQ mask */
	u8 irq_mask_cache[NPORTS];
	/* IRQ mask to be applied */
	u8 irq_mask[NPORTS];
  u32 addr;
};

static struct cy8c9540a dev_c = {
  .addr = 0x20,
};

static inline u8
cypress_get_port(unsigned gpio)
{
  u8 i = 0;
  for (i = 0; i < sizeof(cy8c9540a_port_offs) - 1; i++) {
    if (!(gpio / cy8c9540a_port_offs[i + 1]))
      break;
  }
  return i;
}

static inline u8 cypress_get_offs(unsigned gpio, u8 port)
{
	return gpio - cy8c9540a_port_offs[port];
}

static int
cy8c9540a_gpio_get_value(unsigned gpio)
{
  s32 ret = 0;
  u8 port = 0, pin = 0, in_reg = 0;
  port = cypress_get_port(gpio);
  pin = cypress_get_offs(gpio, port);
  in_reg = REG_INPUT_PORT0 + port;

  ret = i2c_read_byte_data(in_reg);
  if (ret < 0) {
    DLOG("can't read input port%u\n", port);
  }

  return !!(ret & BIT(pin));
}

static void
cy8c9540a_gpio_set_value(unsigned gpio, int val)
{
  s32 ret = 0;
  u8 port = 0, pin = 0, out_reg = 0;
  port = cypress_get_port(gpio);
  pin = cypress_get_offs(gpio, port);
  out_reg = REG_OUTPUT_PORT0 + port;

  if (val) {
    dev_c.outreg_cache[port] |= BIT(pin);
  } else {
    dev_c.outreg_cache[port] &= ~BIT(pin);
  }

  ret = i2c_write_byte_data(out_reg, dev_c.outreg_cache[port]);
  DLOG("here\n");

  if (ret < 0) {
    DLOG("can't read output port%u\n", port);
  }
}

static int cy8c9540a_gpio_set_drive(unsigned gpio, unsigned mode)
{
  s32 ret = 0;
  u8 port = 0, pin = 0, offs = 0, val = 0;
  port = cypress_get_port(gpio);
  pin = cypress_get_offs(gpio, port);

  switch(mode) {
    case GPIOF_DRIVE_PULLUP:
      offs = 0x0;
      break;
    case GPIOF_DRIVE_STRONG:
      offs = 0x4;
      break;
    case GPIOF_DRIVE_HIZ:
      offs = 0x6;
      break;
    default:
      return -1;
  }

  ret = i2c_write_byte_data(REG_PORT_SELECT, port);
  if (ret < 0) {
    DLOG("can't select port%u\n", port);
    return ret;
  }

  ret = i2c_read_byte_data(REG_DRIVE_PULLUP + offs);
  if (ret < 0) {
    DLOG("can't read drive mode port%u\n", port);
    return ret;
  }

  val = (u8)(ret | BIT(pin));

  ret = i2c_write_byte_data(REG_DRIVE_PULLUP + offs, val);
  if (ret < 0) {
    DLOG("can't write drive mode port%u\n", port);
    return ret;
  }

  return 0;
}

static int
cy8c9540a_gpio_direction(unsigned gpio, int out, int val)
{
  s32 ret = 0;
  u8 pins = 0, port = 0, pin = 0;
  port = cypress_get_port(gpio);

  if (out) {
    cy8c9540a_gpio_set_value(gpio, val);
  }

  ret = i2c_write_byte_data(REG_PORT_SELECT, port);
  if (ret < 0) {
    DLOG("can't select port%u\n", port);
    return ret;
  }

  ret = i2c_read_byte_data(REG_PIN_DIR);
  if (ret < 0) {
    DLOG("can't read pin direction\n", port);
    return ret;
  }

  pin = cypress_get_offs(gpio, port);

  pins = (u8)ret & 0xff;
  if (out) {
    pins &= ~BIT(pin);
  } else {
    pins |= BIT(pin);
  }
  
  ret = i2c_write_byte_data(REG_PIN_DIR, pins);
  if (ret < 0) {
    DLOG("can't write pin direction\n", port);
    return ret;
  }

  return 0;
}

static int cy8c9540a_gpio_direction_output(unsigned gpio, int val)
{
  return cy8c9540a_gpio_direction(gpio, 1, val);
}

static int cy8c9540a_gpio_direction_input(unsigned gpio)
{
  return cy8c9540a_gpio_direction(gpio, 0, 0);
}

static s32 cypress_get_id()
{
  s32 dev_id = i2c_read_byte_data(REG_DEVID_STAT);
  return dev_id & 0xf0;
}

/* called when initializing */
static int cy8c9540a_setup()
{
	int ret = 0;
	int i = 0;
  s32 dev_id;
	const u8 eeprom_enable_seq[] = {0x43, 0x4D, 0x53, 0x2};

  dev_id = cypress_get_id();
  DLOG("dev_id is 0x%x\n", dev_id);

	/* Disable PWM, set all GPIOs as input.  */
	for (i = 0; i < NPORTS; i++) {
		ret = i2c_write_byte_data(REG_PORT_SELECT, i);
		if (ret < 0) {
			DLOG("can't select port %u\n", i);
      return ret;
		}

		ret = i2c_write_byte_data(REG_SELECT_PWM, 0x00);
		if (ret < 0) {
			DLOG("can't write to SELECT_PWM\n");
      return ret;
		}

		ret = i2c_write_byte_data(REG_PIN_DIR, 0xff);
		if (ret < 0) {
			DLOG("can't write to PIN_DIR\n");
      return ret;
		}
	}

#if 0
	/* Cache the output registers */
	ret = i2c_read_block_data(REG_OUTPUT_PORT0,
            sizeof(dev_c.outreg_cache),
            dev_c.outreg_cache);
	if (ret < 0) {
    DLOG("can't cache output registers\n");
    return ret;
	}

	/* Set default PWM clock source.  */
	for (i = 0; i < NPWM; i ++) {
		ret = i2c_write_byte_data(REG_PWM_SELECT, i);
		if (ret < 0) {
			DLOG("can't select pwm %u\n", i);
      return ret;
		}

		ret = i2c_write_byte_data(REG_PWM_CLK, PWM_CLK);
		if (ret < 0) {
			DLOG("can't write to REG_PWM_CLK\n");
      return ret;
		}
	}

	/* Enable the EEPROM */
	ret = i2c_write_block_data(REG_ENABLE,
					     sizeof(eeprom_enable_seq),
					     eeprom_enable_seq);
	if (ret < 0) {
		DLOG("can't enable EEPROM\n");
    return ret;
	}
#endif

	return 0;
}

static void
cy8c9540a_test()
{
  unsigned gpio = 11;

  cy8c9540a_setup();
  cy8c9540a_gpio_direction_output(gpio, 0);
  cy8c9540a_gpio_set_drive(gpio, GPIOF_DRIVE_STRONG);
  cy8c9540a_gpio_set_value(gpio, 1);
}

static void
i2c_test()
{
  i2c_base = map_virtual_page (0x90007000 | 0x3);

  i2c_disable();
  i2c_config();
  i2c_disable_int();
  i2c_clear_int();
  /* enable i2c master */
  i2c_xfer_init(dev_c.addr);
  cy8c9540a_test();
  i2c_disable();

  unmap_virtual_page ((void *) i2c_base);
}
/******************************************************************/

static void
probe (void)
{
  uint32 bus, slot, func, i;
#ifdef DEBUG_PCI
  PCI_VENTABLE ven;
  PCI_DEVTABLE dev;
  PCI_CLASSCODETABLE cc;
#endif

  for (bus=0; bus <= PCI_MAX_BUS_NUM; bus++)
    for (slot=0; slot <= PCI_MAX_DEV_NUM; slot++)
      for (func=0; func <= PCI_MAX_FUNC_NUM; func++)
        if (READ (bus, slot, func, 0x00, dword) != 0xFFFFFFFF) {

          uint16 vendorID = READ (bus, slot, func, 0x00, word);
          uint16 deviceID = READ (bus, slot, func, 0x02, word);
          uint8 classID   = READ (bus, slot, func, 0x0B, byte);
          uint8 subclID   = READ (bus, slot, func, 0x0A, byte);
          uint8 prgIFID   = READ (bus, slot, func, 0x09, byte);
          uint8 header    = READ (bus, slot, func, 0x0E, byte);

          for (i=0; i<0x10; i++)
            devices[num_devices].data[i] =
              /* BTW, misaligned read from PCI config space causes
               * VMware to crash. :) */
              READ (bus, slot, func, i<<2, dword);

#ifdef DEBUG_PCI
          DLOG ("%.02x:%.02x.%x", bus, slot, func);
          DLOG ("  %.08x %.08x %.08x %.08x",
                devices[num_devices].data[0],
                devices[num_devices].data[1],
                devices[num_devices].data[2],
                devices[num_devices].data[3]);
          DLOG ("  %.08x %.08x %.08x %.08x",
                devices[num_devices].data[4],
                devices[num_devices].data[5],
                devices[num_devices].data[6],
                devices[num_devices].data[7]);
          DLOG ("  %.08x %.08x %.08x %.08x",
                devices[num_devices].data[8],
                devices[num_devices].data[9],
                devices[num_devices].data[10],
                devices[num_devices].data[11]);
          DLOG ("  %.08x %.08x %.08x %.08x",
                devices[num_devices].data[12],
                devices[num_devices].data[13],
                devices[num_devices].data[14],
                devices[num_devices].data[15]);
#endif

#ifdef DEBUG_PCI
          if (classID == 0x02 && subclID == 0)
            printf ("PCI ethernet: %x %x\n", vendorID, deviceID);
          if (classID == 0x0C && subclID == 0x03)
            printf ("USB host controller: %x %x\n", vendorID, deviceID);
          if (pci_search_ven_table (vendorID, &ven))
            DLOG ("  %s (0x%x)", ven.VenFull, vendorID);
          else
            DLOG ("  0x%x", vendorID);
          if (pci_search_dev_table (vendorID, deviceID, &dev))
            DLOG ("  %s (0x%x)", dev.ChipDesc, deviceID);
          else
            DLOG ("  0x%x", deviceID);
          if (pci_search_class_code_table (classID, subclID, prgIFID, &cc))
            DLOG ("  %s (%x) %s (%x) %s (%x)",
                  cc.BaseDesc, classID,
                  cc.SubDesc, subclID,
                  cc.ProgDesc, prgIFID);
#endif

          devices[num_devices].vendor = vendorID;
          devices[num_devices].device = deviceID;
          devices[num_devices].bus = bus;
          devices[num_devices].slot = slot;
          devices[num_devices].func = func;
          devices[num_devices].classcode = classID;
          devices[num_devices].subclass = subclID;
          devices[num_devices].progIF = prgIFID;
          devices[num_devices].headerType = header;
          devices[num_devices].index = num_devices;

          if ((header & 0x7F) == 0) {
            /* 6 BARs */
            for (i=0; i<6; i++) {
              devices[num_devices].bar[i].raw = devices[num_devices].data[4+i];
              if (devices[num_devices].bar[i].raw != 0) {
                /* Save raw data */
                uint32 raw = devices[num_devices].bar[i].raw;
#ifdef DEBUG_PCI
                DLOG ("  BAR%d raw: %p", i, devices[num_devices].bar[i].raw);
#endif
                /* Fill with 1s */
                WRITE (bus, slot, func, 0x10 + i*4, dword, ~0);
                /* Read back mask */
                uint32 mask = READ (bus, slot, func, 0x10 + i*4, dword);
#ifdef DEBUG_PCI
                DLOG ("  BAR%d mask: %p", i, mask);
#endif
                devices[num_devices].bar[i].mask = mask;
                /* Restore raw data */
                WRITE (bus, slot, func, 0x10 + i*4, dword, raw);
              }
            }
          }
          num_devices++;
          if (num_devices >= PCI_MAX_DEVICES) /* reached our limit */
            return;
        }
}

bool
pci_init (void)
{
  DLOG("init");
  probe ();
  i2c_test();
  return TRUE;
}

bool
pci_get_device (uint n, pci_device *dev)
{
  if (n < num_devices) {
    memcpy (dev, &devices[n], sizeof (pci_device));
    return TRUE;
  }
  return FALSE;
}

bool
pci_find_device (uint16 vendor, uint16 device,
                 uint8 classcode, uint8 subclass,
                 uint start_index,
                 uint* index)
{
#ifdef DEBUG_PCI
  DLOG ("find_device (%p,%p,%p,%p,%p) num_devices=%d",
        vendor, device, classcode, subclass, start_index, num_devices);
#endif
  uint i;
  for (i=start_index; i<num_devices; i++) {
    if ((vendor == 0xFFFF    || vendor == devices[i].vendor) &&
        (device == 0xFFFF    || device == devices[i].device) &&
        (classcode == 0xFF   || classcode == devices[i].classcode) &&
        (subclass == 0xFF    || subclass == devices[i].subclass)) {
      *index = i;
      return TRUE;
    }
  }
  return FALSE;
}

bool
pci_decode_bar (uint index, uint bar_index,
                uint* mem_addr, uint* io_addr, uint* mask)
{
  pci_device *dev;
  pci_bar *bar;

  if (index >= num_devices) return FALSE;

  dev = &devices[index];

  if ((dev->headerType & 0x7F) == 0) {
    /* there are 6 BARs */
    if (bar_index >= 6) return FALSE;

    bar = &dev->bar[bar_index];

    DLOG ("pci_decode_bar (%d, %d) bar->raw=%p", index, bar_index, bar->raw);
    if (bar->raw & 0x1) {
      /* IO port */
      if (io_addr)
        *io_addr = bar->ioBAR.ioPort << 2;
      if (mem_addr)
        *mem_addr = 0;
      if (mask)
        *mask = bar->mask;
    } else {
      /* Memory-mapped */
      if (io_addr)
        *io_addr = 0;
      if (mem_addr)
        *mem_addr = bar->memBAR.baseAddr << 4;
      if (mask)
        *mask = bar->mask;
    }

    return TRUE;
  }

  return FALSE;
}

bool
pci_get_interrupt (uint index, uint* line, uint* pin)
{
  if (index >= num_devices) return FALSE;

  *line = devices[index].data[0xF] & 0xFF;
  *pin = (devices[index].data[0xF] >> 8) & 0xFF;
  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = pci_init
};

DEF_MODULE (pci, "PCI bus driver", &mod_ops, {});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
