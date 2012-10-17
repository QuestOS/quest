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

/* Based on the Linux driver:
 *     Copyright (C) 2004 - 2006 rt2x00 SourceForge Project
 *       <http://rt2x00.serialmonkey.com>
 *
 *       This program is free software; you can redistribute it and/or modify
 *       it under the terms of the GNU General Public License as published by
 *       the Free Software Foundation; either version 2 of the License, or
 *       (at your option) any later version.
 *
 */

/* Abstract: EEPROM reader routines for 93cx6 chipsets. */


#ifndef _93CX6_H_
#define _93CX6_H_

/*
 * EEPROM operation defines.
 */
#define PCI_EEPROM_WIDTH_93C46	6
#define PCI_EEPROM_WIDTH_93C56	8
#define PCI_EEPROM_WIDTH_93C66	8
#define PCI_EEPROM_WIDTH_OPCODE	3
#define PCI_EEPROM_WRITE_OPCODE	0x05
#define PCI_EEPROM_READ_OPCODE	0x06
#define PCI_EEPROM_EWDS_OPCODE	0x10
#define PCI_EEPROM_EWEN_OPCODE	0x13

/**
 * struct eeprom_93cx6 - control structure for setting the commands
 * for reading the eeprom data.
 * @data: private pointer for the driver.
 * @register_read(struct eeprom_93cx6 *eeprom): handler to
 * read the eeprom register, this function should set all reg_* fields.
 * @register_write(struct eeprom_93cx6 *eeprom): handler to
 * write to the eeprom register by using all reg_* fields.
 * @width: eeprom width, should be one of the PCI_EEPROM_WIDTH_* defines
 * @reg_data_in: register field to indicate data input
 * @reg_data_out: register field to indicate data output
 * @reg_data_clock: register field to set the data clock
 * @reg_chip_select: register field to set the chip select
 *
 * This structure is used for the communication between the driver
 * and the eeprom_93cx6 handlers for reading the eeprom.
 */
struct eeprom_93cx6 {
        void *data;

        void (*register_read)(struct eeprom_93cx6 *eeprom);
        void (*register_write)(struct eeprom_93cx6 *eeprom);

        int width;

        char reg_data_in;
        char reg_data_out;
        char reg_data_clock;
        char reg_chip_select;
};

extern void eeprom_93cx6_read(struct eeprom_93cx6 *eeprom,
        const uint8 word, uint16 *data);
extern void eeprom_93cx6_multiread(struct eeprom_93cx6 *eeprom,
        const uint8 word, uint16 *data, const uint16 words);

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
