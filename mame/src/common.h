/*********************************************************************

  common.h

  Generic functions used in different emulators.
  There's not much for now, but it could grow in the future... ;-)

*********************************************************************/

#ifndef COMMON_H
#define COMMON_H

#include "osdepend.h"


struct RomModule
{
	const char *name;	/* name of the file to load */
	int offset;			/* offset to load it to */
	int length;			/* length of the file */
};

/* there are some special cases for the above. name, offset and size all set to 0 */
/* mark the end of the aray. If name is 0 and the others aren't, that means "continue */
/* reading the previous from from this address". If length is 0 and offset is not 0, */
/* that marks the start of a new memory region. Confused? Well, don't worry, just use */
/* the macros below. */

#define ROM_START(name) static struct RomModule name[] = {	/* start of table */
#define ROM_REGION(length) { 0, length, 0 },	/* start of memory region */
#define ROM_LOAD(name,offset,length) { name, offset, length },	/* ROM to load */
#define ROM_CONTINUE(offset,length) { 0, offset, length },	/* continue loading the previous ROM */
#define ROM_END { 0, 0, 0 }	}; /* end of table */


struct GfxLayout
{
	int width,height;	/* width and height of chars/sprites */
	int total;	/* total numer of chars/sprites in the rom */
	int planes;	/* number of bitplanes */
	int planeoffset[8];	/* start of every bitplane */
	int xoffset[64];	/* coordinates of the bit corresponding to the pixel */
	int yoffset[64];	/* of the given coordinates */
	int charincrement;	/* distance between two consecutive characters/sprites */
};



struct GfxElement
{
	int width,height;

	struct osd_bitmap *gfxdata;	/* graphic data */
	int total_elements;	/* total number of characters/sprites */

	int color_granularity;	/* number of colors for each color code */
							/* (for example, 4 for 2 bitplanes gfx) */
	unsigned char *colortable;	/* map color codes to screen pens */
								/* if this is 0, the function does a verbatim copy */
	int total_colors;
};



struct rectangle
{
	int min_x,max_x;
	int min_y,max_y;
};



struct DisplayText
{
	const char *text;	/* 0 marks the end of the array */
	int color;
	int x;
	int y;
};

#define TRANSPARENCY_NONE 0
#define TRANSPARENCY_PEN 1
#define TRANSPARENCY_COLOR 2

int readroms(const struct RomModule *romp,const char *basename);
void decodechar(struct GfxElement *gfx,int num,const unsigned char *src,const struct GfxLayout *gl);
struct GfxElement *decodegfx(const unsigned char *src,const struct GfxLayout *gl);
void freegfx(struct GfxElement *gfx);
void drawgfx(struct osd_bitmap *dest,const struct GfxElement *gfx,
		unsigned int code,unsigned int color,int flipx,int flipy,int sx,int sy,
		const struct rectangle *clip,int transparency,int transparent_color);
void copybitmap(struct osd_bitmap *dest,struct osd_bitmap *src,int flipx,int flipy,int sx,int sy,
		const struct rectangle *clip,int transparency,int transparent_color);
void clearbitmap(struct osd_bitmap *bitmap);
int setdipswitches(void);
void displaytext(const struct DisplayText *dt,int erase);
int showcharset(void);


#endif
