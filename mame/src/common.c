/***************************************************************************

  common.c

  Generic functions used in different emulators.

***************************************************************************/

#include "driver.h"


/***************************************************************************

  Read ROMs into memory.

  Arguments:
  const struct RomModule *romp - pointer to an array of Rommodule structures,
                                 as defined in common.h.

  const char *basename - Name of the directory where the files are
                         stored. The function also supports the
						 control sequence %s in file names: for example,
						 if the RomModule gives the name "%s.bar", and
						 the basename is "foo", the file "foo/foo.bar"
						 will be loaded.

***************************************************************************/
int readroms(const struct RomModule *rommodule,const char *basename)
{
	int region;
	const struct RomModule *romp;


	romp = rommodule;

	for (region = 0;region < MAX_MEMORY_REGIONS;region++)
		Machine->memory_region[region] = 0;

	region = 0;

	while (romp->name || romp->offset || romp->length)
	{
		int region_size;


		if (romp->name || romp->length)
		{
			printf("Error in RomModule definition: expecting ROM_REGION\n");
			goto getout;
		}

		region_size = romp->offset;
		if ((Machine->memory_region[region] = malloc(region_size)) == 0)
		{
			printf("Unable to allocate %d bytes of RAM\n",region_size);
			goto getout;
		}

		/* some games (i.e. Pleiades) want the memory clear on startup */
		memset(Machine->memory_region[region],0,region_size);

		romp++;

		while (romp->length)
		{
			FILE *f;
			char buf[100];
			char name[100];


			if (romp->name == 0)
			{
				printf("Error in RomModule definition: ROM_CONTINUE not preceded by ROM_LOAD\n");
				goto getout;
			}

			sprintf(buf,romp->name,basename);
			sprintf(name,"%s/%s",basename,buf);

			if ((f = fopen(name,"rb")) == 0)
			{
				printf("Unable to open ROM %s\n",name);
				goto printromlist;
			}

			do
			{
				if (romp->offset + romp->length > region_size)
				{
					printf("Error in RomModule definition: %s out of memory region space\n",name);
					fclose(f);
					goto getout;
				}

				if (fread(Machine->memory_region[region] + romp->offset,1,romp->length,f) != romp->length)
				{
					printf("Unable to read ROM %s\n",name);
					fclose(f);
					goto printromlist;
				}

				romp++;
			} while (romp->length && romp->name == 0);

			fclose(f);
		}

		region++;
	}

	return 0;


printromlist:
	romp = rommodule;
	printf( "\nMAME is an emulator: it reproduces, more or less faithfully, the behaviour of\n"
			"several arcade machines. But hardware is useless without software, so an image\n"
			"of the ROMs which run on that hardware is required. Such ROMs, like any other\n"
			"commercial software, are copyrighted material and it is therefore illegal to\n"
			"use them if you don't own the original arcade machine. Needless to say, ROMs\n"
			"are not distributed together with MAME. Distribution of MAME together with ROM\n"
			"images is a violation of copyright law and should be promptly reported to the\n"
			"author so that appropriate legal action can be taken.\n\nPress return to continue\n");
	getchar();
	printf("This is the list of the ROMs required.\n"
			"All the ROMs must reside in a subdirectory called \"%s\".\n"
			"Name             Size\n",basename);
	while (romp->name || romp->offset || romp->length)
	{
		romp++;	/* skip memory region definition */

		while (romp->length)
		{
			char name[100];
			int length;


			sprintf(name,romp->name,basename);

			length = 0;

			do
			{
				length += romp->length;

				romp++;
			} while (romp->length && romp->name == 0);

			printf("%-12s %5d bytes\n",name,length);
		}
	}

getout:
	for (region = 0;region < MAX_MEMORY_REGIONS;region++)
	{
		free(Machine->memory_region[region]);
		Machine->memory_region[region] = 0;
	}
	return 1;
}



/***************************************************************************

  Function to convert the information stored in the graphic roms into a
  more usable format.

  Back in the early '80s, arcade machines didn't have the memory or the
  speed to handle bitmaps like we do today. They used "character maps",
  instead: they had one or more sets of characters (usually 8x8 pixels),
  which would be placed on the screen in order to form a picture. This was
  very fast: updating a character mapped display is, rougly speaking, 64
  times faster than updating an equivalent bitmap display, since you only
  modify groups of 8x8 pixels and not the single pixels. However, it was
  also much less versatile than a bitmap screen, since with only 256
  characters you had to do all of your graphics. To overcome this
  limitation, some special hardware graphics were used: "sprites". A sprite
  is essentially a bitmap, usually larger than a character, which can be
  placed anywhere on the screen (not limited to character boundaries) by
  just telling the hardware the coordinates. Moreover, sprites can be
  flipped along the major axis just by setting the appropriate bit (some
  machines can flip characters as well). This saves precious memory, since
  you need only one copy of the image instead of four.

  What about colors? Well, the early machines had a limited palette (let's
  say 16-32 colors) and each character or sprite could not use all of them
  at the same time. Characters and sprites data would use a limited amount
  of bits per pixel (typically 2, which allowed them to address only four
  different colors). You then needed some way to tell to the hardware which,
  among the available colors, were the four colors. This was done using a
  "color attribute", which typically was an index for a lookup table.

  OK, after this brief and incomplete introduction, let's come to the
  purpose of this section: how to interpret the data which is stored in
  the graphic roms. Unfortunately, there is no easy answer: it depends on
  the hardware. The easiest way to find how data is encoded, is to start by
  making a bit by bit dump of the rom. You will usually be able to
  immediately recognize some pattern: if you are lucky, you will see
  letters and numbers right away, otherwise you will see something which
  looks like letters and numbers, but with halves switched, dilated, or
  something like that. You'll then have to find a way to put it all
  together to obtain our standard one byte per pixel representation. Two
  things to remember:
  - keep in mind that every pixel has typically two (or more) bits
    associated with it, and they are not necessarily near to each other.
  - characters might be rotated 90 degrees. That's because many games used a
    tube rotated 90 degrees. Think how your monitor would look like if you
	put it on its side ;-)

  After you have successfully decoded the characters, you have to decode
  the sprites. This is usually more difficult, because sprites are larger,
  maybe have more colors, and are more difficult to recognize when they are
  messed up, since they are pure graphics without letters and numbers.
  However, with some work you'll hopefully be able to work them out as
  well. As a rule of thumb, the sprites should be encoded in a way not too
  dissimilar from the characters.

***************************************************************************/
int readbit(const unsigned char *src,int bitnum)
{
	int bit;


	bit = src[bitnum / 8] << (bitnum % 8);

	if (bit & 0x80) return 1;
	else return 0;
}



void decodechar(struct GfxElement *gfx,int num,const unsigned char *src,const struct GfxLayout *gl)
{
	int plane;


	for (plane = 0;plane < gl->planes;plane++)
	{
		int offs,y;


		offs = num * gl->charincrement + gl->planeoffset[plane];
		for (y = 0;y < gl->height;y++)
		{
			int x;


			for (x = 0;x < gl->width;x++)
			{
				unsigned char *dp;


				dp = gfx->gfxdata->line[num * gl->height + y];
				if (plane == 0) dp[x] = 0;
				else dp[x] <<= 1;

				dp[x] += readbit(src,offs + gl->yoffset[y] + gl->xoffset[x]);
			}
		}
	}
}



struct GfxElement *decodegfx(const unsigned char *src,const struct GfxLayout *gl)
{
	int c;
	struct osd_bitmap *bm;
	struct GfxElement *gfx;


	if ((bm = osd_create_bitmap(gl->width,gl->total * gl->height)) == 0)
		return 0;
	if ((gfx = malloc(sizeof(struct GfxElement))) == 0)
	{
		osd_free_bitmap(bm);
		return 0;
	}
	gfx->width = gl->width;
	gfx->height = gl->height;
	gfx->total_elements = gl->total;
	gfx->color_granularity = 1 << gl->planes;
	gfx->gfxdata = bm;

	for (c = 0;c < gl->total;c++)
		decodechar(gfx,c,src,gl);

	return gfx;
}



void freegfx(struct GfxElement *gfx)
{
	if (gfx)
	{
		osd_free_bitmap(gfx->gfxdata);
		free(gfx);
	}
}



/***************************************************************************

  Draw graphic elements in the specified bitmap.

  transparency == TRANSPARENCY_NONE - no transparency.
  transparency == TRANSPARENCY_PEN - bits whose _original_ value is == transparent_color
                                     are transparent. This is the most common kind of
									 transparency.
  transparency == TRANSPARENCY_COLOR - bits whose _remapped_ value is == transparent_color
                                     are transparent. This is used by e.g. Pac Man.

***************************************************************************/
void drawgfx(struct osd_bitmap *dest,const struct GfxElement *gfx,
		unsigned int code,unsigned int color,int flipx,int flipy,int sx,int sy,
		const struct rectangle *clip,int transparency,int transparent_color)
{
	int ox,oy,ex,ey,x,y,start;
	const unsigned char *sd;
	unsigned char *bm;
	int col;


	if (!gfx) return;

	/* check bounds */
	ox = sx;
	oy = sy;
	ex = sx + gfx->width-1;
	if (sx < 0) sx = 0;
	if (clip && sx < clip->min_x) sx = clip->min_x;
	if (ex >= dest->width) ex = dest->width-1;
	if (clip && ex > clip->max_x) ex = clip->max_x;
	if (sx > ex) return;
	ey = sy + gfx->height-1;
	if (sy < 0) sy = 0;
	if (clip && sy < clip->min_y) sy = clip->min_y;
	if (ey >= dest->height) ey = dest->height-1;
	if (clip && ey > clip->max_y) ey = clip->max_y;
	if (sy > ey) return;

	start = (code % gfx->total_elements) * gfx->height;
	if (gfx->colortable)	/* remap colors */
	{
		const unsigned char *paldata;


		paldata = &gfx->colortable[gfx->color_granularity * (color % gfx->total_colors)];

		switch (transparency)
		{
			case TRANSPARENCY_NONE:
				if (flipx)
				{
					if (flipy)	/* XY flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + gfx->height-1 - (y-oy)] + gfx->width-1 - (sx-ox);
							for (x = sx;x <= ex;x++)
								*(bm++) = paldata[*(sd--)];
						}
					}
					else 	/* X flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + (y-oy)] + gfx->width-1 - (sx-ox);
							for (x = sx;x <= ex;x++)
								*(bm++) = paldata[*(sd--)];
						}
					}
				}
				else
				{
					if (flipy)	/* Y flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + gfx->height-1 - (y-oy)] + (sx-ox);
							for (x = sx;x <= ex;x++)
								*(bm++) = paldata[*(sd++)];
						}
					}
					else		/* normal */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + (y-oy)] + (sx-ox);
							for (x = sx;x <= ex;x++)
								*(bm++) = paldata[*(sd++)];
						}
					}
				}
				break;

			case TRANSPARENCY_PEN:
				if (flipx)
				{
					if (flipy)	/* XY flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + gfx->height-1 - (y-oy)] + gfx->width-1 - (sx-ox);
							for (x = sx;x <= ex;x++)
							{
								col = *(sd--);
								if (col != transparent_color) *bm = paldata[col];
								bm++;
							}
						}
					}
					else 	/* X flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + (y-oy)] + gfx->width-1 - (sx-ox);
							for (x = sx;x <= ex;x++)
							{
								col = *(sd--);
								if (col != transparent_color) *bm = paldata[col];
								bm++;
							}
						}
					}
				}
				else
				{
					if (flipy)	/* Y flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + gfx->height-1 - (y-oy)] + (sx-ox);
							for (x = sx;x <= ex;x++)
							{
								col = *(sd++);
								if (col != transparent_color) *bm = paldata[col];
								bm++;
							}
						}
					}
					else		/* normal */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + (y-oy)] + (sx-ox);
							for (x = sx;x <= ex;x++)
							{
								col = *(sd++);
								if (col != transparent_color) *bm = paldata[col];
								bm++;
							}
						}
					}
				}
				break;

			case TRANSPARENCY_COLOR:
				if (flipx)
				{
					if (flipy)	/* XY flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + gfx->height-1 - (y-oy)] + gfx->width-1 - (sx-ox);
							for (x = sx;x <= ex;x++)
							{
								col = paldata[*(sd--)];
								if (col != transparent_color) *bm = col;
								bm++;
							}
						}
					}
					else 	/* X flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + (y-oy)] + gfx->width-1 - (sx-ox);
							for (x = sx;x <= ex;x++)
							{
								col = paldata[*(sd--)];
								if (col != transparent_color) *bm = col;
								bm++;
							}
						}
					}
				}
				else
				{
					if (flipy)	/* Y flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + gfx->height-1 - (y-oy)] + (sx-ox);
							for (x = sx;x <= ex;x++)
							{
								col = paldata[*(sd++)];
								if (col != transparent_color) *bm = col;
								bm++;
							}
						}
					}
					else		/* normal */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + (y-oy)] + (sx-ox);
							for (x = sx;x <= ex;x++)
							{
								col = paldata[*(sd++)];
								if (col != transparent_color) *bm = col;
								bm++;
							}
						}
					}
				}
				break;
		}
	}
	else
	{
		switch (transparency)
		{
			case TRANSPARENCY_NONE:		/* do a verbatim copy (faster) */
				if (flipx)
				{
					if (flipy)	/* XY flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + gfx->height-1 - (y-oy)] + gfx->width-1 - (sx-ox);
							for (x = sx;x <= ex;x++)
								*(bm++) = *(sd--);
						}
					}
					else 	/* X flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + (y-oy)] + gfx->width-1 - (sx-ox);
							for (x = sx;x <= ex;x++)
								*(bm++) = *(sd--);
						}
					}
				}
				else
				{
					if (flipy)	/* Y flip */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + gfx->height-1 - (y-oy)] + (sx-ox);
							memcpy(bm,sd,ex-sx+1);
						}
					}
					else		/* normal */
					{
						for (y = sy;y <= ey;y++)
						{
							bm = dest->line[y] + sx;
							sd = gfx->gfxdata->line[start + (y-oy)] + (sx-ox);
							memcpy(bm,sd,ex-sx+1);
						}
					}
				}
				break;

			case TRANSPARENCY_PEN:
			case TRANSPARENCY_COLOR:
				{
					int *sd4,x1;
					int trans4;


					trans4 = transparent_color * 0x01010101;

					if (flipx)
					{
						if (flipy)	/* XY flip */
						{
							for (y = sy;y <= ey;y++)
							{
								bm = dest->line[y] + sx;
								sd4 = (int *)(gfx->gfxdata->line[start + gfx->height-1 - (y-oy)] + gfx->width-1 - (sx-ox) - 3);
								for (x = sx;x <= ex;x+=4)
								{
		/* WARNING: if the width of the area to copy is not a multiple of sizeof(int), this */
		/* might access memory outside it. The copy will be executed correctly, though. */
									if (*sd4 == trans4)
									{
										bm += 4;
									}
									else
									{
										sd = ((unsigned char *)sd4) + 3;
										x1 = ex - x;
										if (x1 > 3) x1 = 3;
										while (x1 >= 0)
										{
											col = *(sd--);
											if (col != transparent_color) *bm = col;
											bm++;
											x1--;
										}
									}
									sd4--;
								}
							}
						}
						else 	/* X flip */
						{
							for (y = sy;y <= ey;y++)
							{
								bm = dest->line[y] + sx;
								sd4 = (int *)(gfx->gfxdata->line[start + (y-oy)] + gfx->width-1 - (sx-ox) - 3);
								for (x = sx;x <= ex;x+=4)
								{
		/* WARNING: if the width of the area to copy is not a multiple of sizeof(int), this */
		/* might access memory outside it. The copy will be executed correctly, though. */
									if (*sd4 == trans4)
									{
										bm += 4;
									}
									else
									{
										sd = ((unsigned char *)sd4) + 3;
										x1 = ex - x;
										if (x1 > 3) x1 = 3;
										while (x1 >= 0)
										{
											col = *(sd--);
											if (col != transparent_color) *bm = col;
											bm++;
											x1--;
										}
									}
									sd4--;
								}
							}
						}
					}
					else
					{
						if (flipy)	/* Y flip */
						{
							for (y = sy;y <= ey;y++)
							{
								bm = dest->line[y] + sx;
								sd4 = (int *)(gfx->gfxdata->line[start + gfx->height-1 - (y-oy)] + (sx-ox));
								for (x = sx;x <= ex;x+=4)
								{
		/* WARNING: if the width of the area to copy is not a multiple of sizeof(int), this */
		/* might access memory outside it. The copy will be executed correctly, though. */
									if (*sd4 == trans4)
									{
										bm += 4;
									}
									else
									{
										sd = (unsigned char *)sd4;
										x1 = ex - x;
										if (x1 > 3) x1 = 3;
										while (x1 >= 0)
										{
											col = *(sd++);
											if (col != transparent_color) *bm = col;
											bm++;
											x1--;
										}
									}
									sd4++;
								}
							}
						}
						else		/* normal */
						{
							for (y = sy;y <= ey;y++)
							{
								bm = dest->line[y] + sx;
								sd4 = (int *)(gfx->gfxdata->line[start + (y-oy)] + (sx-ox));
								for (x = sx;x <= ex;x+=4)
								{
		/* WARNING: if the width of the area to copy is not a multiple of sizeof(int), this */
		/* might access memory outside it. The copy will be executed correctly, though. */
									if (*sd4 == trans4)
									{
										bm += 4;
									}
									else
									{
										sd = (unsigned char *)sd4;
										x1 = ex - x;
										if (x1 > 3) x1 = 3;
										while (x1 >= 0)
										{
											col = *(sd++);
											if (col != transparent_color) *bm = col;
											bm++;
											x1--;
										}
									}
									sd4++;
								}
							}
						}
					}
				}
				break;
		}
	}
}



/***************************************************************************

  Use drawgfx() to copy a bitmap onto another at the given position.
  This function will very likely change in the future.

***************************************************************************/
void copybitmap(struct osd_bitmap *dest,struct osd_bitmap *src,int flipx,int flipy,int sx,int sy,
		const struct rectangle *clip,int transparency,int transparent_color)
{
	static struct GfxElement mygfx =
	{
		0,0,0,	/* filled in later */
		1,1,0,1
	};

	mygfx.width = src->width;
	mygfx.height = src->height;
	mygfx.gfxdata = src;
	drawgfx(dest,&mygfx,0,0,flipx,flipy,sx,sy,clip,transparency,transparent_color);
}



void clearbitmap(struct osd_bitmap *bitmap)
{
	int i;


	for (i = 0;i < bitmap->height;i++)
		memset(bitmap->line[i],Machine->background_pen,bitmap->width);
}



int setdipswitches(void)
{
	struct DisplayText dt[40];
	int settings[20];
	int i,s,key;
	int total;
	const struct DSW *dswsettings;


	dswsettings = Machine->gamedrv->dswsettings;

	total = 0;
	while (dswsettings[total].num != -1)
	{
		int msk,val;


		msk = dswsettings[total].mask;
		if (msk == 0) return 0;	/* error in DSW definition, quit */
		val = Machine->gamedrv->input_ports[dswsettings[total].num].default_value;
		while ((msk & 1) == 0)
		{
			val >>= 1;
			msk >>= 1;
		}
		settings[total] = val & msk;

		total++;
	}

	s = 0;
	do
	{
		for (i = 0;i < total;i++)
		{
			dt[2 * i].color = (i == s) ? Machine->gamedrv->yellow_text : Machine->gamedrv->white_text;
			dt[2 * i].text = dswsettings[i].name;
			dt[2 * i].x = 2*8;
			dt[2 * i].y = 2*8 * i + (Machine->drv->screen_height - 2*8 * (total - 1)) / 2;
			dt[2 * i + 1].color = (i == s) ? Machine->gamedrv->yellow_text : Machine->gamedrv->white_text;
			dt[2 * i + 1].text = dswsettings[i].values[settings[i]];
			dt[2 * i + 1].x = Machine->drv->screen_width - 2*8 - 8*strlen(dt[2 * i + 1].text);
			dt[2 * i + 1].y = dt[2 * i].y;
		}
		dt[2 * i].text = 0;	/* terminate array */

		displaytext(dt,1);

		key = osd_read_key();

		switch (key)
		{
			case OSD_KEY_DOWN:
				if (s < total - 1) s++;
				break;

			case OSD_KEY_UP:
				if (s > 0) s--;
				break;

			case OSD_KEY_RIGHT:
				if (dswsettings[s].reverse == 0)
				{
					if (dswsettings[s].values[settings[s] + 1] != 0) settings[s]++;
				}
				else
				{
					if (settings[s] > 0) settings[s]--;
				}
				break;

			case OSD_KEY_LEFT:
				if (dswsettings[s].reverse == 0)
				{
					if (settings[s] > 0) settings[s]--;
				}
				else
				{
					if (dswsettings[s].values[settings[s] + 1] != 0) settings[s]++;
				}
				break;
		}
	} while (key != OSD_KEY_TAB && key != OSD_KEY_ESC);

	while (osd_key_pressed(key));	/* wait for key release */

	while (--total >= 0)
	{
		int msk;


		msk = dswsettings[total].mask;
		while ((msk & 1) == 0)
		{
			settings[total] <<= 1;
			msk >>= 1;
		}

		Machine->gamedrv->input_ports[dswsettings[total].num].default_value =
				(Machine->gamedrv->input_ports[dswsettings[total].num].default_value
				& ~dswsettings[total].mask) | settings[total];
	}

	/* clear the screen before returning */
	clearbitmap(Machine->scrbitmap);

	if (key == OSD_KEY_ESC) return 1;
	else return 0;
}






/***************************************************************************

  Display text on the screen. If erase is 0, it superimposes the text on
  the last frame displayed.

***************************************************************************/
void displaytext(const struct DisplayText *dt,int erase)
{
	if (erase) clearbitmap(Machine->scrbitmap);

	while (dt->text)
	{
		int x,y;
		const char *c;


		x = dt->x;
		y = dt->y;
		c = dt->text;

		while (*c)
		{
			if (*c == '\n')
			{
				x = dt->x;
				y += Machine->gfx[0]->height + 1;
			}
			else if (*c == ' ')
			{
				/* don't try to word wrap at the beginning of a line (this would cause */
				/* an endless loop if a word is longer than a line) */
				if (x == dt->x)
					x += Machine->gfx[0]->width;
				else
				{
					int nextlen;
					const char *nc;


					x += Machine->gfx[0]->width;
					nc = c+1;
					while (*nc && *nc != ' ' && *nc != '\n')
						nc++;

					nextlen = nc - c - 1;

					/* word wrap */
					if (x + nextlen * Machine->gfx[0]->width >= Machine->drv->screen_width)
					{
						x = dt->x;
						y += Machine->gfx[0]->height + 1;
					}
				}
			}
			else
			{
				if (*c >= '0' && *c <= '9')
					drawgfx(Machine->scrbitmap,Machine->gfx[0],*c - '0' + Machine->gamedrv->numbers_start,dt->color,0,0,x,y,0,TRANSPARENCY_NONE,0);
				else drawgfx(Machine->scrbitmap,Machine->gfx[0],*c - 'A' + Machine->gamedrv->letters_start,dt->color,0,0,x,y,0,TRANSPARENCY_NONE,0);

				x += Machine->gfx[0]->width;
			}

			c++;
		}

		dt++;
	}

	osd_update_display();
}



int showcharset(void)
{
	int i,key,cpl;
	struct DisplayText dt[2];
	char buf[80];
	int bank,color;


	bank = 0;
	color = 0;

	do
	{
		clearbitmap(Machine->scrbitmap);

		cpl = Machine->scrbitmap->width / Machine->gfx[bank]->width;

		for (i = 0;i < Machine->drv->gfxdecodeinfo[bank].gfxlayout->total;i++)
		{
			drawgfx(Machine->scrbitmap,Machine->gfx[bank],
					i,color,
					0,0,
					(i % cpl) * Machine->gfx[bank]->width,
					Machine->gfx[0]->height+1 + (i / cpl) * Machine->gfx[bank]->height,
					0,TRANSPARENCY_NONE,0);
		}

		sprintf(buf,"GFXSET %d  COLOR %d",bank,color);
		dt[0].text = buf;
		dt[0].color = Machine->gamedrv->paused_color;
		dt[0].x = 0;
		dt[0].y = 0;
		dt[1].text = 0;
		displaytext(dt,0);


		key = osd_read_key();

		switch (key)
		{
			case OSD_KEY_RIGHT:
				if (Machine->gfx[bank + 1]) bank++;
				break;

			case OSD_KEY_LEFT:
				if (bank > 0) bank--;
				break;

			case OSD_KEY_UP:
				if (color < Machine->drv->gfxdecodeinfo[bank].total_color_codes - 1)
					color++;
				break;

			case OSD_KEY_DOWN:
				if (color > 0) color--;
				break;
		}
	} while (key != OSD_KEY_F4 && key != OSD_KEY_ESC);

	while (osd_key_pressed(key));	/* wait for key release */

	/* clear the screen before returning */
	clearbitmap(Machine->scrbitmap);

	if (key == OSD_KEY_ESC) return 1;
	else return 0;
}
