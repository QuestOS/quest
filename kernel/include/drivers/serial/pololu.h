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
#ifndef _POLOLU_H_
#define _POLOLU_H_

#include "types.h"

#define POLOLU_CMD_SETPARAM    0x0
#define POLOLU_CMD_SETSPD      0x1
#define POLOLU_CMD_SETPOS1     0x2
#define POLOLU_CMD_SETPOS2     0x3
#define POLOLU_CMD_SETPOSABS   0x4
#define POLOLU_CMD_SETNEUTRAL  0x5

/* Pololu Mode Command Format */
typedef struct {
  uint8_t start_byte;  /* 0x80 */
  uint8_t dev_id;      /* 0x01 */
  uint8_t cmd;
  uint8_t servo_num;
  uint8_t data1;
  uint8_t data2;
} pololu_cmd_t;

typedef struct {
  uint8_t start_byte;  /* 0xFF */
  uint8_t servo_num;   /* 0x00 - 0xFE */
  uint8_t servo_pos;   /* 0x00 - 0xFE */
} minissc2_cmd_t;

extern int pololu_send_cmd (uint32_t, uint8_t, uint8_t, uint8_t, uint8_t);
extern int pololu_enable (uint32_t, uint8_t);
extern int pololu_disable (uint32_t, uint8_t);
extern int pololu_set_abspos (uint32_t, uint8_t, uint32_t);

#endif

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
