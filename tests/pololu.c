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

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <vshm.h>

#define POLOLU_CMD_SETPARAM    0x0
#define POLOLU_CMD_SETSPD      0x1
#define POLOLU_CMD_SETPOS1     0x2
#define POLOLU_CMD_SETPOS2     0x3
#define POLOLU_CMD_SETPOSABS   0x4
#define POLOLU_CMD_SETNEUTRAL  0x5
#define POLOLU_CMD_INIT        0x6

#define LINUX_SANDBOX    0x3

typedef struct pololu_command {
  uint32_t ssc;
  uint32_t index;
  uint8_t servo;
  uint8_t cmd;
  uint8_t data1;
  uint8_t data2;
} pololu_command_t;

pololu_command_t pcmd;

uint32_t prev_index = 0;

int
pololu_set_abspos (uint32_t ssc, uint8_t servo, uint32_t pos)
{
  if ((pos < 500) || (pos > 5500)) {
    return -1;
  }

  return syscall_pololu_send_cmd (ssc, servo, POLOLU_CMD_SETPOSABS,
                                  (uint8_t) (pos / 128), (uint8_t) (pos % 128));
}

int
pololu_enable (uint32_t ssc, uint8_t servo)
{
  /* Set range to 15, this is the default. */
  return syscall_pololu_send_cmd (ssc, servo, POLOLU_CMD_SETPARAM, 0x4F, 0x0);
}

int
main ()
{
  uint sandbox = socket_get_sb_id();
  vshm_async_channel_t async_channel;
  printf("In %s sandbox %u\n", __FILE__, sandbox);
  if (sandbox == 0) {
    int res = mk_vshm_async_channel (&async_channel, 1234, sizeof(pololu_command_t),
                                     0x1 << LINUX_SANDBOX,
                                     VSHM_CREATE | VSHM_ALL_ACCESS);
    printf("%d: res = %d\n", sandbox, res);
    pcmd.index = 0;
    pololu_enable (0, 0);
    pololu_set_abspos (0, 0, 1500);
    usleep(3000000);
    pololu_set_abspos (0, 0, 4500);
    printf ("Waiting for new commands...\n");
    while(1) {
      vshm_async_channel_read (&async_channel, &pcmd);
      if (pcmd.index != prev_index) {
        printf ("Got new command (%d,%d,%d,%d,%d)!\n", pcmd.ssc, pcmd.servo, pcmd.cmd,
                pcmd.data1, pcmd.data2);
        syscall_pololu_send_cmd (pcmd.ssc, pcmd.servo, pcmd.cmd, pcmd.data1, pcmd.data2);
        prev_index = pcmd.index;
        printf ("Command index=%d\n", prev_index);
      }
    }
    exit (EXIT_SUCCESS);
  }

  exit (EXIT_FAILURE);
}

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
