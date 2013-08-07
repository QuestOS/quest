#ifndef DRIVER_H
#define DRIVER_H


#include "common.h"
#include "mame.h"
#include "cpuintrf.h"


/***************************************************************************

Note that the memory hooks are not passed the actual memory address where
the operation takes place, but the offset from the beginning of the block
they are assigned to. This makes handling of mirror addresses easier, and
makes the handlers a bit more "object oriented". If you handler needs to
read/write the main memory area, provide a "base" pointer: it will be
initialized by the main engine to point to the beginning of the memory block
assigned to the handler.

***************************************************************************/
struct MemoryReadAddress
{
	int start,end;
	int (*handler)(int offset);	/* see special values below */
	unsigned char **base;
};

#define MRA_NOP 0	/* don't care, return 0 */
#define MRA_RAM ((int(*)())-1)	/* plain RAM location (return its contents) */
#define MRA_ROM ((int(*)())-2)	/* plain ROM location (return its contents) */


struct MemoryWriteAddress
{
	int start,end;
	void (*handler)(int offset,int data);	/* see special values below */
	unsigned char **base;
};

#define MWA_NOP 0	/* do nothing */
#define MWA_RAM ((void(*)())-1)	/* plain RAM location (store the value) */
#define MWA_ROM ((void(*)())-2)	/* plain ROM location (do nothing) */


/***************************************************************************

IN and OUT ports are handled like memory accesses, the hook template is the
same so you can interchange them. Of course there is no 'base' pointer for
IO ports.

***************************************************************************/
struct IOReadPort
{
	int start,end;
	int (*handler)(int offset);	/* see special values below */
};

#define IORP_NOP 0	/* don't care, return 0 */


struct IOWritePort
{
	int start,end;
	void (*handler)(int offset,int data);	/* see special values below */
};

#define IOWP_NOP 0	/* do nothing */



/***************************************************************************

Don't confuse this with the I/O ports above. This is used to handle game
inputs (joystick, coin slots, etc). Typically, you will read them using
input_port_[n]_r(), which you will associate to the appropriate memory
address or I/O port.

***************************************************************************/
struct InputPort
{
	int default_value;	/* default value for the input port */
	int keyboard[8];	/* keys affecting the 8 bits of the input port (0 means none) */
	int joystick[8];	/* same for joystick */
};



/* dipswitch setting definition */
struct DSW
{
	int num;	/* input port affected */
				/* -1 terminates the array */
	int mask;	/* bits affected */
	const char *name;	/* name of the setting */
	const char *values[16];/* null terminated array of names for the values */
									/* the setting can have */
	int reverse; 	/* set to 1 to display values in reverse order */
};



struct GfxDecodeInfo
{
	int memory_region;	/* memory region where the data resides (usually 1) */
						/* -1 marks the end of the array */
	int start;	/* beginning of data to decode */
	struct GfxLayout *gfxlayout;
	int color_codes_start;	/* offset in the color lookup table where color codes start */
	int total_color_codes;	/* total number of color codes */
};



struct MachineCPU
{
	int cpu_type;	/* see #defines below. */
	int cpu_clock;	/* in Hertz */
	int memory_region;	/* number of the memory region (allocated by loadroms()) where */
						/* this CPU resides */
	const struct MemoryReadAddress *memory_read;
	const struct MemoryWriteAddress *memory_write;
	const struct IOReadPort *port_read;
	const struct IOWritePort *port_write;
	int (*interrupt)(void);
	int interrupts_per_frame;	/* usually 1 */
};

#define CPU_Z80 1
#define CPU_M6502 2


#define MAX_CPU 4


struct MachineDriver
{
	/* basic machine hardware */
	struct MachineCPU cpu[MAX_CPU];
	int frames_per_second;
	int (*init_machine)(const char *gamename);

	/* video hardware */
	int screen_width,screen_height;
	struct rectangle visible_area;
	struct GfxDecodeInfo *gfxdecodeinfo;
	int total_colors;	/* palette is 3*total_colors bytes long */
	int color_table_len;	/* length in bytes of the color lookup table */
	void (*vh_convert_color_prom)(unsigned char *palette, unsigned char *colortable,const unsigned char *color_prom);

	int (*vh_init)(const char *gamename);
	int (*vh_start)(void);
	void (*vh_stop)(void);
	void (*vh_update)(struct osd_bitmap *bitmap);

	/* sound hardware */
	unsigned char *samples;
	int (*sh_init)(const char *gamename);
	int (*sh_start)(void);
	void (*sh_stop)(void);
	void (*sh_update)(void);
};



struct GameDriver
{
	const char *name;
	const struct MachineDriver *drv;

	const struct RomModule *rom;
	unsigned (*rom_decode)(int A);	/* used to decrypt the ROMs after loading them */
	unsigned (*opcode_decode)(int A);	/* used to decrypt the ROMs when the CPU fetches an opcode */

	struct InputPort *input_ports;
	const struct DSW *dswsettings;

		/* if they are available, provide a dump of the color proms (there is no */
		/* copyright infringement in that, since you can't copyright a color scheme) */
		/* and a function to convert them to a usable palette and colortable (the */
		/* function pointer is in the MachineDriver, not here) */
		/* Otherwise, leave this field null and provide palette and colortable. */
	const unsigned char *color_prom;
	const unsigned char *palette;
	const unsigned char *colortable;
	int	numbers_start;	/* start of numbers and letters in the character roms */
	int letters_start;	/* (used by displaytext() ) */
	int white_text,yellow_text;	/* used by the dip switch menu */
	int paused_x,paused_y,paused_color;	/* used to print PAUSED on the screen */

	int (*hiscore_load)(const char *name);	/* will be called every vblank until it */
											/* returns nonzero */
	void (*hiscore_save)(const char *name);	/* will not be loaded if hiscore_load() hasn't yet */
										/* returned nonzero, to avoid saving an invalid table */
};



extern const struct GameDriver *drivers[];


#endif
