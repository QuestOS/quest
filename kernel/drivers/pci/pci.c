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

static uint32 I2C_BASE;
static uint32 IO_EXP_ADDR = 0x20;

#define WRITEL(val, reg) do {*(uint32 *)reg = val;} while(0)
#define READL(reg) *(uint32 *)reg

/* copy from linux to configure clock */
static uint32
i2c_dw_scl_hcnt(uint32 ic_clk, uint32 tSYMBOL, uint32 tf, int cond, int offset)
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

static uint32 i2c_dw_scl_lcnt(uint32 ic_clk, uint32 tLOW, uint32 tf, int offset)
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
  /*
   * copy from clanton
   * intel_cln_gip_i2c.c : line 59
   */
  /* XXX: divided by 8 */
  uint32 input_clock_khz = 33000;
  /*
   * copy from clanton
   * intel_cln_gip_i2c.c : line 53
   */
  uint32 tx_fifo_depth = 16;

  /* registers */
  uint32 I2C_SS_SCL_HCNT = I2C_BASE + 0x14;
  uint32 I2C_SS_SCL_LCNT = I2C_BASE + 0x18;
  uint32 I2C_FS_SCL_HCNT = I2C_BASE + 0x1c;
  uint32 I2C_FS_SCL_LCNT = I2C_BASE + 0x20;
  uint32 I2C_TX_TL = I2C_BASE + 0x3c;
  uint32 I2C_RX_TL = I2C_BASE + 0x38;
  uint32 I2C_CON = I2C_BASE + 0x0;

  /* set standard and fast speed deviders for high/low periods */
  uint32 hcnt, lcnt;

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
  WRITEL(hcnt, I2C_SS_SCL_HCNT);
  WRITEL(lcnt, I2C_SS_SCL_LCNT);

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
  WRITEL(hcnt, I2C_FS_SCL_HCNT);
	WRITEL(lcnt, I2C_FS_SCL_LCNT);

	/* Configure Tx/Rx FIFO threshold levels */
  WRITEL(tx_fifo_depth - 1, I2C_TX_TL); 
  WRITEL(0, I2C_RX_TL); 

  /* 
   * copy from clanton. In comment, it says:
   * Clanton default configuration is fast mode, unless otherwise asked.
   * XXX: Donno what is DW_IC_CON_SLAVE_DISABLE...
   */
  //uint32 cfg = INTEL_CLN_STD_CFG | DW_IC_CON_SPEED_FAST;
  uint32 cfg = INTEL_CLN_STD_CFG | DW_IC_CON_SPEED_STD;
  /* configure the i2c master */
  WRITEL(cfg, I2C_CON);
}

static void
i2c_disable_int()
{
  uint32 I2C_INTR_MASK = I2C_BASE + 0x30;
  WRITEL(0, I2C_INTR_MASK);
}

static void
i2c_clear_int()
{
  uint32 I2C_CLR_INTR = I2C_BASE + 0x40;
  uint32 val = READL(I2C_CLR_INTR);
}

static void
i2c_disable()
{
  uint32 I2C_ENABLE = I2C_BASE + 0x6c;
  WRITEL(0, I2C_ENABLE);
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

static void i2c_xfer_init()
{
  uint32 I2C_TAR = I2C_BASE + 0x4;
  uint32 I2C_ENABLE = I2C_BASE + 0x6c;
  uint32 I2C_INTR_MASK = I2C_BASE + 0x30;

  i2c_disable();

  /* set the slave (target) address */
  WRITEL(IO_EXP_ADDR, I2C_TAR);

	/* Enable the adapter */
	WRITEL(1, I2C_ENABLE);

	/* XXX: Enable interrupts */
	/* WRITEL(DW_IC_INTR_DEFAULT_MASK, I2C_INTR_MASK); */
}

static uint32
i2c_enable_status()
{
  uint32 I2C_ENABLE_STATUS = I2C_BASE + 0x9c;
  return READL(I2C_ENABLE_STATUS);
}

static uint32
i2c_status()
{
  uint32 I2C_STATUS = I2C_BASE + 0x70;
  return READL(I2C_STATUS);
}

struct i2c_msg {
	uint16 flags;
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_STOP		0x8000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_NOSTART */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
	uint16 len;		/* msg length				*/
	uint8 *buf;		/* pointer to msg data			*/
};

#define DW_IC_CMD_READ			0x01
#define DW_IC_CMD_STOP			0x02
#define DW_IC_CMD_RESTART		0x04

/*
 * Define the IC_DATA_CMD format.
 */
static union i2c_dw_data_cmd {
	struct fields {
		uint8 data;
		uint8 cmd;
	} fields;
	uint16 value;
} data_cmd;

/* XXX: status may need to be returned */
static void
i2c_xfer(struct i2c_msg *msgs, int msg_num)
{
  uint32 I2C_DATA_CMD = I2C_BASE + 0x10;
  uint32 I2C_STATUS = I2C_BASE + 0x70;
  uint32 buf_len;
  uint8 *buf;
  int i, explicit_stop = 1, segment_start;

  for (i = 0; i < msg_num; i++) {
    /* XXX: indicates if this is the first byte of a msg 
     * right now it is always turned on since both msgs we have 
     * are of 1 byte */
    segment_start = 1;
    buf_len = msgs[i].len;
    buf = msgs[i].buf;
    DLOG("buf: %x", *buf);
    data_cmd.fields.data = 0x00;
    data_cmd.fields.cmd = 0x00;

    if (msgs[i].flags & I2C_M_RD)
      data_cmd.fields.cmd = DW_IC_CMD_READ;
    else {
      data_cmd.fields.data = *buf;
      buf++;
    }

    if (1 == explicit_stop
        && 1 == segment_start) {
      /*
       * First byte of a transaction segment for a
       * device requiring explicit transaction
       * termination: generate (re)start symbol.
       */
      segment_start = 0;
      data_cmd.fields.cmd |= DW_IC_CMD_RESTART;
    }

    if (1 == explicit_stop
        && i == msg_num - 1
        && 1 == buf_len) {
      /*
       * Last byte of last transction segment for a
       * device requiring explicit transaction
       * termination: generate stop symbol.
       */
      data_cmd.fields.cmd |= DW_IC_CMD_STOP;
    }

    DLOG("data_cmd.value is %x", data_cmd.value);
    uint32 status = READL(I2C_STATUS);
    DLOG("status1 is %x", status);
    WRITEL(data_cmd.value, I2C_DATA_CMD);
    status = READL(I2C_STATUS);
    DLOG("status2 is %x", status);
  }
}

static void
i2c_read(struct i2c_msg *msgs, int msg_num)
{
  uint32 I2C_DATA_CMD = I2C_BASE + 0x10;
  uint32 I2C_RXFLR = I2C_BASE + 0x78;
  uint32 I2C_STATUS = I2C_BASE + 0x70;
  int i, rx_valid;
  uint32 buf_len;
  uint8 *buf;
  uint32 status;

  for (i = 0; i < msg_num; i++) {
    if (!(msgs[i].flags & I2C_M_RD)) 
      continue;

    buf_len = msgs[i].len;
    buf = msgs[i].buf;

    /* XXX: hack */
    //while (!((status = READL(I2C_STATUS)) & 0x8));
    status = READL(I2C_STATUS);
    DLOG("status is %d", status);
		rx_valid = READL(I2C_RXFLR);
    DLOG("rx_valid is %d", rx_valid);
		for (; buf_len > 0 && rx_valid > 0; buf_len--, rx_valid--) {
      uint8 val = (uint8)READL(I2C_DATA_CMD);
      DLOG("val is %d", val);
			*buf++ = val;
    }
    uint8 val = (uint8)READL(I2C_DATA_CMD);
    DLOG("val is %d", val);
  }
}

/******************************************************************/

/******************************************************************/
/* i2c protocol */

/*
 * Data for SMBus Messages
 * copy from clanton
 */
#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */
union i2c_smbus_data {
	uint8 byte;
	uint16 word;
	uint8 block[I2C_SMBUS_BLOCK_MAX + 2]; /* block[0] is used for length */
			       /* and one more for user-space compatibility */
};

/* SMBus transaction types (size parameter in the above functions)
   Note: these no longer correspond to the (arbitrary) PIIX4 internal codes! */
#define I2C_SMBUS_QUICK		    0
#define I2C_SMBUS_BYTE		    1
#define I2C_SMBUS_BYTE_DATA	    2
#define I2C_SMBUS_WORD_DATA	    3
#define I2C_SMBUS_PROC_CALL	    4
#define I2C_SMBUS_BLOCK_DATA	    5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7		/* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8

/* i2c_smbus_xfer read or write markers */
#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

static sint32 i2c_proto_xfer(unsigned short flags,
    char read_write, uint8 command, int size,
    union i2c_smbus_data *data)
{
  unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX+3];
  unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX+2];
	int num = read_write == I2C_SMBUS_READ ? 2 : 1;
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
	case I2C_SMBUS_QUICK:
		msg[0].len = 0;
		/* Special case: The read/write field is used as data */
		msg[0].flags = flags | (read_write == I2C_SMBUS_READ ?
					I2C_M_RD : 0);
		num = 1;
		break;
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_READ) {
			/* Special case: only a read! */
			msg[0].flags = I2C_M_RD | flags;
			num = 1;
		}
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_READ)
			msg[1].len = 1;
		else {
			msg[0].len = 2;
			msgbuf0[1] = data->byte;
		}
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_READ)
			msg[1].len = 2;
		else {
			msg[0].len = 3;
			msgbuf0[1] = data->word & 0xff;
			msgbuf0[2] = data->word >> 8;
		}
		break;
	case I2C_SMBUS_PROC_CALL:
		num = 2; /* Special case */
		read_write = I2C_SMBUS_READ;
		msg[0].len = 3;
		msg[1].len = 2;
		msgbuf0[1] = data->word & 0xff;
		msgbuf0[2] = data->word >> 8;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			msg[1].flags |= I2C_M_RECV_LEN;
			msg[1].len = 1; /* block length will be added by
					   the underlying bus driver */
		} else {
			msg[0].len = data->block[0] + 2;
			if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 2) {
      /*
				dev_err(&adapter->dev,
					"Invalid block write size %d\n",
					data->block[0]);
				return -EINVAL;
      */
        return -1;
			}
			for (i = 1; i < msg[0].len; i++)
				msgbuf0[i] = data->block[i-1];
		}
		break;
	case I2C_SMBUS_BLOCK_PROC_CALL:
		num = 2; /* Another special case */
		read_write = I2C_SMBUS_READ;
		if (data->block[0] > I2C_SMBUS_BLOCK_MAX) {
    /*
			dev_err(&adapter->dev,
				"Invalid block write size %d\n",
				data->block[0]);
			return -EINVAL;
    */
      return -1;
		}
		msg[0].len = data->block[0] + 2;
		for (i = 1; i < msg[0].len; i++)
			msgbuf0[i] = data->block[i-1];
		msg[1].flags |= I2C_M_RECV_LEN;
		msg[1].len = 1; /* block length will be added by
				   the underlying bus driver */
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			msg[1].len = data->block[0];
		} else {
			msg[0].len = data->block[0] + 1;
			if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 1) {
      /*
				dev_err(&adapter->dev,
					"Invalid block write size %d\n",
					data->block[0]);
				return -EINVAL;
      */
        return -1;
			}
			for (i = 1; i <= data->block[0]; i++)
				msgbuf0[i] = data->block[i];
		}
		break;
	default:
    /*
		dev_err(&adapter->dev, "Unsupported transaction %d\n", size);
		return -EOPNOTSUPP;
    */
    return -1;
	}

  i2c_xfer(msg, num);
  i2c_read(msg, num);

  if (read_write == I2C_SMBUS_READ)
		switch (size) {
		case I2C_SMBUS_BYTE:
			data->byte = msgbuf0[0];
			break;
		case I2C_SMBUS_BYTE_DATA:
			data->byte = msgbuf1[0];
			break;
		case I2C_SMBUS_WORD_DATA:
		case I2C_SMBUS_PROC_CALL:
			data->word = msgbuf1[0] | (msgbuf1[1] << 8);
			break;
		case I2C_SMBUS_I2C_BLOCK_DATA:
			for (i = 0; i < data->block[0]; i++)
				data->block[i+1] = msgbuf1[i];
			break;
		case I2C_SMBUS_BLOCK_DATA:
		case I2C_SMBUS_BLOCK_PROC_CALL:
			for (i = 0; i < msgbuf1[0] + 1; i++)
				data->block[i] = msgbuf1[i];
			break;
		}

	return 0;
}

static sint32
i2c_read_byte_data(uint8 command)
{
  union i2c_smbus_data data;
  sint32 status = i2c_proto_xfer(0, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data);
  if (status < 0) {
    DLOG("i2c_read_byte_data error");
    return -1;
  }
  return data.byte;
}

static sint32
i2c_write_byte_data(uint8 command, uint8 value)
{
  union i2c_smbus_data data;
  data.byte = value;
  sint32 status = i2c_proto_xfer(0, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
  if (status < 0) {
    DLOG("i2c_write_byte_data error");
    return -1;
  }
  return 0;
}

#define REG_DEVID_STAT			0x2e
/* XXX: this function should be in cy8c9540 driver */
static sint32
cy_dev_id()
{
  sint32 dev_id = i2c_read_byte_data(REG_DEVID_STAT);
  return dev_id & 0xf0;
}

static void
i2c_test()
{
  I2C_BASE = map_virtual_page (0x90007000 | 0x3);

  i2c_disable();
  uint32 enabled = i2c_enable_status();
  DLOG("1: enabled is %x", enabled);
  i2c_config();
  i2c_disable_int();
  i2c_clear_int();

  /* XXX: do I need to wait_bus_not_busy */
  /* enable i2c master */
  i2c_xfer_init();
  enabled = i2c_enable_status();
  DLOG("2: enabled is %x", enabled);

  /* read cy8c9540's device id */
  sint32 val = cy_dev_id();

	/* XXX: wait for tx to complete */
	/* ret = wait_for_completion_interruptible_timeout(&dev->cmd_complete, HZ); */

  i2c_disable();
  DLOG("dev_id is %x", val);

  unmap_virtual_page ((void *) I2C_BASE);
}

/******************************************************************/

/******************************************************************/
/* cy8c9540 driver */

#define NPORTS				6

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

struct cy8c9540a {
	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	struct pwm_chip pwm_chip;
	struct mutex lock;
	/* protect serialized access to the interrupt controller bus */
	struct mutex irq_lock;
	/* cached output registers */
	u8 outreg_cache[NPORTS];
	/* cached IRQ mask */
	u8 irq_mask_cache[NPORTS];
	/* IRQ mask to be applied */
	u8 irq_mask[NPORTS];
} cy_dev;

/* called when initializing */
static int cy8c9540a_setup()
{
	int ret = 0;
	int i = 0;
	const uint8 eeprom_enable_seq[] = {0x43, 0x4D, 0x53, 0x2};

	/* Disable PWM, set all GPIOs as input.  */
	for (i = 0; i < NPORTS; i ++) {
		ret = i2c_write_byte_data(REG_PORT_SELECT, i);
		if (ret < 0) {
			DLOG("can't select port %u\n", i);
			goto end;
		}

		ret = i2c_smbus_write_byte_data(REG_SELECT_PWM, 0x00);
		if (ret < 0) {
			DLOG("can't write to SELECT_PWM\n");
			goto end;
		}

		ret = i2c_smbus_write_byte_data(REG_PIN_DIR, 0xff);
		if (ret < 0) {
			DLOG("can't write to PIN_DIR\n");
			goto end;
		}
	}

  gc->direction_input = cy8c9540a_gpio_direction_input;
	gc->direction_output = cy8c9540a_gpio_direction_output;
	gc->get = cy8c9540a_gpio_get_value;
	gc->set = cy8c9540a_gpio_set_value;
	gc->set_drive = cy8c9540a_gpio_set_drive;

	gc->can_sleep = 1;

	gc->base = GPIO_BASE_ID;
	gc->ngpio = NGPIO;
	gc->label = client->name;
	gc->owner = THIS_MODULE;
	gc->to_irq = cy8c9540a_gpio_to_irq;


#if 0
	/* Cache the output registers */
	ret = i2c_smbus_read_i2c_block_data(dev->client, REG_OUTPUT_PORT0,
					    sizeof(dev->outreg_cache),
					    dev->outreg_cache);
	if (ret < 0) {
		dev_err(&client->dev, "can't cache output registers\n");
		goto end;
	}

	/* Set default PWM clock source.  */
	for (i = 0; i < NPWM; i ++) {
		ret = i2c_smbus_write_byte_data(client, REG_PWM_SELECT, i);
		if (ret < 0) {
			dev_err(&client->dev, "can't select pwm %u\n", i);
			goto end;
		}

		ret = i2c_smbus_write_byte_data(client, REG_PWM_CLK, PWM_CLK);
		if (ret < 0) {
			dev_err(&client->dev, "can't write to REG_PWM_CLK\n");
			goto end;
		}
	}

	/* Enable the EEPROM */
	ret = i2c_smbus_write_i2c_block_data(client, REG_ENABLE,
					     sizeof(eeprom_enable_seq),
					     eeprom_enable_seq);
	if (ret < 0) {
		dev_err(&client->dev, "can't enable EEPROM\n");
		goto end;
	}
#endif

end:
	return ret;
}

/******************************************************************/

/******************************************************************/
/* gpio driver */
/* copy from include/asm-generic/gpio.h */
/* XXX: need to get rid of some unused members */
struct gpio_chip {
	const char		*label;
	struct device		*dev;
	struct module		*owner;

	int			(*request)(struct gpio_chip *chip,
						unsigned offset);
	void			(*free)(struct gpio_chip *chip,
						unsigned offset);
	int			(*get_direction)(struct gpio_chip *chip,
						unsigned offset);
	int			(*direction_input)(struct gpio_chip *chip,
						unsigned offset);
	int			(*get)(struct gpio_chip *chip,
						unsigned offset);
	int			(*direction_output)(struct gpio_chip *chip,
						unsigned offset, int value);
	int			(*set_debounce)(struct gpio_chip *chip,
						unsigned offset, unsigned debounce);
	int			(*set_drive)(struct gpio_chip *chip,
					     unsigned offset, unsigned mode);

	void			(*set)(struct gpio_chip *chip,
						unsigned offset, int value);

	int			(*to_irq)(struct gpio_chip *chip,
						unsigned offset);

	void			(*dbg_show)(struct seq_file *s,
						struct gpio_chip *chip);
	int			base;
	u16			ngpio;
	const char		*const *names;
	unsigned		can_sleep:1;
	unsigned		exported:1;
} gc;

static void
gpio_chip_setup()
{
  gc.direction_input = cy8c9540a_gpio_direction_input; 

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
