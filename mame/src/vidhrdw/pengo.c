/***************************************************************************

  vidhrdw.c

  Functions to emulate the video hardware of the machine.

***************************************************************************/

#include "driver.h"
#include "vidhrdw/generic.h"



#define VIDEO_RAM_SIZE 0x400

static int gfx_bank;



static struct rectangle spritevisiblearea =
{
	0, 28*8-1,
	2*8, 34*8-1
};



/***************************************************************************

  Convert the color PROMs into a more useable format.

  Pengo has a 32 bytes palette PROM and a 256 bytes color lookup table PROM
  (actually that's 512 bytes, but the high address bit is grounded).
  The palette PROM is connected to the RGB output this way:

  bit 7 -- 220 ohm resistor  -- BLUE
        -- 470 ohm resistor  -- BLUE
        -- 220 ohm resistor  -- GREEN
        -- 470 ohm resistor  -- GREEN
        -- 1  kohm resistor  -- GREEN
        -- 220 ohm resistor  -- RED
        -- 470 ohm resistor  -- RED
  bit 0 -- 1  kohm resistor  -- RED

***************************************************************************/
void pengo_vh_convert_color_prom(unsigned char *palette, unsigned char *colortable,const unsigned char *color_prom)
{
	int i;


	for (i = 0;i < 32;i++)
	{
		int bit0,bit1,bit2;


		bit0 = (color_prom[i] >> 0) & 0x01;
		bit1 = (color_prom[i] >> 1) & 0x01;
		bit2 = (color_prom[i] >> 2) & 0x01;
		palette[3*i] = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		bit0 = (color_prom[i] >> 3) & 0x01;
		bit1 = (color_prom[i] >> 4) & 0x01;
		bit2 = (color_prom[i] >> 5) & 0x01;
		palette[3*i + 1] = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		bit0 = 0;
		bit1 = (color_prom[i] >> 6) & 0x01;
		bit2 = (color_prom[i] >> 7) & 0x01;
		palette[3*i + 2] = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
	}

	for (i = 0;i < 4 * 32;i++)
		colortable[i] = color_prom[i + 32];
	for (i = 4 * 32;i < 8 * 32;i++)
	{
		if (color_prom[i + 32]) colortable[i] = color_prom[i + 32] + 0x10;
		else colortable[i] = 0;
	}
}



int pengo_vh_start(void)
{
	gfx_bank = 0;

	return generic_vh_start();
}



void pengo_gfxbank_w(int offset,int data)
{
	/* the Pengo hardware can set independently the palette bank, color lookup */
	/* table, and chars/sprites. However the game always set them together (and */
	/* the only place where this is used is the intro screen) so I don't bother */
	/* emulating the whole thing. */
	gfx_bank = data;
}



/***************************************************************************

  Draw the game screen in the given osd_bitmap.
  Do NOT call osd_update_display() from this function, it will be called by
  the main emulation engine.

***************************************************************************/
void pengo_vh_screenrefresh(struct osd_bitmap *bitmap)
{
	int i,offs;


	/* for every character in the Video RAM, check if it has been modified */
	/* since last time and update it accordingly. */
	for (offs = 0;offs < VIDEO_RAM_SIZE;offs++)
	{
		if (dirtybuffer[offs])
		{
			int sx,sy,mx,my;


			dirtybuffer[offs] = 0;

	/* Even if Pengo's screen is 28x36, the memory layout is 32x32. We therefore */
	/* have to convert the memory coordinates into screen coordinates. */
	/* Note that 32*32 = 1024, while 28*36 = 1008: therefore 16 bytes of Video RAM */
	/* don't map to a screen position. We don't check that here, however: range */
	/* checking is performed by drawgfx(). */

			mx = offs / 32;
			my = offs % 32;

			if (mx <= 1)
			{
				sx = 29 - my;
				sy = mx + 34;
			}
			else if (mx >= 30)
			{
				sx = 29 - my;
				sy = mx - 30;
			}
			else
			{
				sx = 29 - mx;
				sy = my + 2;
			}

			drawgfx(tmpbitmap,Machine->gfx[gfx_bank ? 2 : 0],
					videoram[offs],
					colorram[offs],
					0,0,8*sx,8*sy,
					&Machine->drv->visible_area,TRANSPARENCY_NONE,0);
		}
	}

	/* copy the character mapped graphics */
	copybitmap(bitmap,tmpbitmap,0,0,0,0,&Machine->drv->visible_area,TRANSPARENCY_NONE,0);

	/* Draw the sprites. Note that it is important to draw them exactly in this */
	/* order, to have the correct priorities. */
	/* sprites #0 and #7 are not used */
	for (i = 6;i > 0;i--)
	{
		drawgfx(bitmap,Machine->gfx[gfx_bank ? 3 : 1],
				spriteram[2*i] >> 2,spriteram[2*i + 1],
				spriteram[2*i] & 2,spriteram[2*i] & 1,
				239 - spriteram_2[2*i],272 - spriteram_2[2*i + 1],
				&spritevisiblearea,TRANSPARENCY_COLOR,Machine->background_pen);
	}
}
