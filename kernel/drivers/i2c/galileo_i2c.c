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

/* I2C driver */
/* Based on Claton
 * i2c-designware-core.c
 * i2c-core.c */

#include "drivers/pci/pci.h"
#include "util/printf.h"
#include "mem/mem.h"

#define DEBUG_I2C 

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

struct i2c_dev {
  u32 tx_fifo_depth;
  u32 rx_fifo_depth;
  u32 clk_khz;
  u32 rx_outstanding;
	u32 i2c_base;
};

static struct i2c_dev dev = {
  .tx_fifo_depth = 16,
  .rx_fifo_depth = 16,
  .clk_khz = 33000,
  .rx_outstanding = 0,
};

static inline void
i2c_write_r(u32 val , u32 reg)
{
  *(u32 *)(dev.i2c_base + reg) = val;
}

static inline u32
i2c_read_r(u32 reg)
{
  return *(u32 *)(dev.i2c_base + reg);
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
  u32 input_clock_khz = dev.clk_khz;

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
  i2c_write_r(dev.tx_fifo_depth - 1, DW_IC_TX_TL); 
  i2c_write_r(0, DW_IC_RX_TL); 

  /* 
   * copy from clanton. In comment, it says:
   * Clanton default configuration is fast mode, unless otherwise asked.
   * XXX: this is not true... setting fast mode makes it not work
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

static u32
i2c_status()
{
  return i2c_read_r(DW_IC_STATUS);
}
/* ************************************************************ */

/* Protocol */

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
  dev.rx_outstanding = 0;
	int tx_limit;

  for (i = 0; i < msg_num; i++) {
    segment_start = 1;
    buf_len = msgs[i].len;
    buf = msgs[i].buf;

    while (buf_len > 0) {
			do {
				tx_limit = dev.tx_fifo_depth - i2c_read_r(DW_IC_TXFLR);
			} while (tx_limit <= 0);

      data_cmd.fields.data = 0x00;
      data_cmd.fields.cmd = 0x00;

      if (msgs[i].flags & I2C_M_RD) {
        data_cmd.fields.cmd = DW_IC_CMD_READ;
        dev.rx_outstanding++;
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

    while (dev.rx_outstanding > 0) {
      /* more read requests need to be handled */
      rx_valid = i2c_read_r(DW_IC_RXFLR);

      for (; buf_len > 0 && rx_valid > 0; buf_len--, rx_valid--) {
        u8 val = (u8)i2c_read_r(DW_IC_DATA_CMD);
        *buf++ = val;
        dev.rx_outstanding--;
      }
    }
  }
}

/* transaction types */
#define I2C_BYTE_DATA	    0
#define I2C_WORD_DATA	    1
#define I2C_BLOCK_DATA    2

/* i2c_xfer read or write markers */
#define I2C_READ	1
#define I2C_WRITE	0

#define I2C_BLOCK_MAX	32	/* As specified in SMBus standard */
union i2c_data {
	u8 byte;
	u16 word;
	u8 block[I2C_BLOCK_MAX + 2]; /* block[0] is used for length */
			       /* and one more for user-space compatibility */
};

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
        DLOG("Invalid block write size %d",
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
    DLOG("Unsupported transaction%d",
         size);
    return -1;
	}

  i2c_write(msg, num);

  if (read_write == I2C_READ) {
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
	} 

	return 0;
}
/* ************************************************************ */

/* APIs */

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

void i2c_xfer_init(u32 slave_addr)
{
  i2c_disable();
  /* set the slave (target) address */
  i2c_write_r(slave_addr, DW_IC_TAR);
	/* Enable the adapter */
	i2c_write_r(1, DW_IC_ENABLE);
	/* XXX: Enable interrupts */
	/* i2c_write_r(DW_IC_INTR_DEFAULT_MASK, DW_IC_INTR_MASK); */
}

#define GALILEO_I2C_VID		 		0x8086
#define	GALILEO_I2C_DID			 	0x0934

bool i2c_init()
{
	uint device_index;
	uint mem_addr;

	if (!pci_find_device(GALILEO_I2C_VID, GALILEO_I2C_DID,
				0xFF, 0xFF, 0, &device_index))
		return FALSE;
	if (device_index == (uint)(~0)) {
    DLOG ("Unable to detect compatible device.");
    return FALSE;
  }
	if (!pci_decode_bar (device_index, 0, &mem_addr, NULL, NULL)) {
    DLOG ("Bar decoding failed!");
    return FALSE;
  } 

  u32 addr = (u32)map_virtual_page (mem_addr | 0x3);
	dev.i2c_base = addr;
	i2c_disable();
  i2c_config();
  i2c_disable_int();
  i2c_clear_int();

	return TRUE;
}

void i2c_remove()
{
	i2c_disable();
  unmap_virtual_page ((void *) dev.i2c_base);
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = i2c_init
};

DEF_MODULE (galileo_i2c, "Galileo I2C driver", &mod_ops, {"pci"});


