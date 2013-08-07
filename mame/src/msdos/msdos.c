/***************************************************************************

  osdepend.c

  OS dependant stuff (display handling, keyboard scan...)
  This is the only file which should me modified in order to port the
  emulator to a different system.

***************************************************************************/

#include <pc.h>
#include <sys/farptr.h>
#include <go32.h>
#include "TwkUser.c"
#include "driver.h"
#include "allegro.h"


#define VESA_SCREEN_WIDTH 640
#define VESA_SCREEN_HEIGHT 480
#define VESASCAN_MAX_WIDTH 800
#define SCREEN_MODE GFX_VESA1


struct osd_bitmap *bitmap;
int first_free_pen;
int use_vesa;
int use_vesascan;
int use_vesaskip;
int noscanlines;
int videofreq;
int video_sync;
int play_sound;
int use_joystick;

int vs_width;
int vs_height;
int vs_skiplines;
int vs_xoffset;
int vs_yoffset;
int vs_lastline;


/* audio related stuff */
#define NUMVOICES 6
#define SAMPLE_RATE 44100
#define SAMPLE_BUFFER_LENGTH 50000
HAC hVoice[NUMVOICES];
LPAUDIOWAVE lpWave[NUMVOICES];


/* put here anything you need to do when the program is started. Return 0 if */
/* initialization was successful, nonzero otherwise. */
int osd_init(int argc,char **argv)
{
	int i;
	int soundcard;


	use_vesa = 0;
	use_vesascan = 0;
	use_vesaskip = 0;
	noscanlines = 0;
	videofreq = 0;
	video_sync = 0;
	play_sound = 1;
	use_joystick = 1;
	soundcard = -1;

	/* --??-- Need keyboard support */
	first_free_pen = 0;

	return 0;
}



/* put here cleanup routines to be executed when the program is terminated. */
void osd_exit(void)
{

}



/* Create a bitmap. Also call clearbitmap() to appropriately initialize it to */
/* the background color. */
struct osd_bitmap *osd_create_bitmap(int width,int height)
{
	struct osd_bitmap *bitmap;


	if ((bitmap = malloc(sizeof(struct osd_bitmap) + (height-1)*sizeof(unsigned char *))) != 0)
	{
		int i;
		unsigned char *bm;


		bitmap->width = width;
		bitmap->height = height;
		if ((bm = malloc(width * height * sizeof(unsigned char))) == 0)
		{
			free(bitmap);
			return 0;
		}

		for (i = 0;i < height;i++)
			bitmap->line[i] = &bm[i * width];

		bitmap->private = bm;

		clearbitmap(bitmap);
	}

	return bitmap;
}



void osd_free_bitmap(struct osd_bitmap *bitmap)
{
	if (bitmap)
	{
		free(bitmap->private);
		free(bitmap);
	}
}



Register scr224x288[] =
{
	{ 0x3c2, 0x00, 0xe3},{ 0x3d4, 0x00, 0x5f},{ 0x3d4, 0x01, 0x37},
	{ 0x3d4, 0x02, 0x38},{ 0x3d4, 0x03, 0x82},{ 0x3d4, 0x04, 0x4a},
	{ 0x3d4, 0x05, 0x9a},{ 0x3d4, 0x06, 0x55},{ 0x3d4, 0x07, 0xf0},
	{ 0x3d4, 0x08, 0x00},{ 0x3d4, 0x09, 0x61},{ 0x3d4, 0x10, 0x40},
	{ 0x3d4, 0x11, 0xac},{ 0x3d4, 0x12, 0x3f},{ 0x3d4, 0x13, 0x1c},
	{ 0x3d4, 0x14, 0x40},{ 0x3d4, 0x15, 0x40},{ 0x3d4, 0x16, 0x4a},
	{ 0x3d4, 0x17, 0xa3},{ 0x3c4, 0x01, 0x01},{ 0x3c4, 0x04, 0x0e},
	{ 0x3ce, 0x05, 0x40},{ 0x3ce, 0x06, 0x05},{ 0x3c0, 0x10, 0x41},
	{ 0x3c0, 0x13, 0x0}
};

Register scr224x288scanlines[] =
{
	{ 0x3c2, 0x00, 0xe3},{ 0x3d4, 0x00, 0x5f},{ 0x3d4, 0x01, 0x37},
	{ 0x3d4, 0x02, 0x38},{ 0x3d4, 0x03, 0x82},{ 0x3d4, 0x04, 0x4a},
	{ 0x3d4, 0x05, 0x9a},{ 0x3d4, 0x06, 0x43},{ 0x3d4, 0x07, 0x1f},
	{ 0x3d4, 0x08, 0x00},{ 0x3d4, 0x09, 0x60},{ 0x3d4, 0x10, 0x2a},
	{ 0x3d4, 0x11, 0xac},{ 0x3d4, 0x12, 0x1f},{ 0x3d4, 0x13, 0x1c},
	{ 0x3d4, 0x14, 0x40},{ 0x3d4, 0x15, 0x27},{ 0x3d4, 0x16, 0x3a},
	{ 0x3d4, 0x17, 0xa3},{ 0x3c4, 0x01, 0x01},{ 0x3c4, 0x04, 0x0e},
	{ 0x3ce, 0x05, 0x40},{ 0x3ce, 0x06, 0x05},{ 0x3c0, 0x10, 0x41},
	{ 0x3c0, 0x13, 0x00}
};

Register scr256x256[] =
{
	{ 0x3c2, 0x00, 0xe3},{ 0x3d4, 0x00, 0x5f},{ 0x3d4, 0x01, 0x3f},
	{ 0x3d4, 0x02, 0x40},{ 0x3d4, 0x03, 0x82},{ 0x3d4, 0x04, 0x4A},
	{ 0x3d4, 0x05, 0x9A},{ 0x3d4, 0x06, 0x23},{ 0x3d4, 0x07, 0xb2},
	{ 0x3d4, 0x08, 0x00},{ 0x3d4, 0x09, 0x61},{ 0x3d4, 0x10, 0x0a},
	{ 0x3d4, 0x11, 0xac},{ 0x3d4, 0x12, 0xff},{ 0x3d4, 0x13, 0x20},
	{ 0x3d4, 0x14, 0x40},{ 0x3d4, 0x15, 0x07},{ 0x3d4, 0x16, 0x1a},
	{ 0x3d4, 0x17, 0xa3},{ 0x3c4, 0x01, 0x01},{ 0x3c4, 0x04, 0x0e},
	{ 0x3ce, 0x05, 0x40},{ 0x3ce, 0x06, 0x05},{ 0x3c0, 0x10, 0x41},
	{ 0x3c0, 0x13, 0x00}
};

Register scr256x256scanlines[] =
{
	{ 0x3c2, 0x00, 0xe3},{ 0x3d4, 0x00, 0x5f},{ 0x3d4, 0x01, 0x3f},
	{ 0x3d4, 0x02, 0x40},{ 0x3d4, 0x03, 0x82},{ 0x3d4, 0x04, 0x4a},
	{ 0x3d4, 0x05, 0x9a},{ 0x3d4, 0x06, 0x23},{ 0x3d4, 0x07, 0x1d},
	{ 0x3d4, 0x08, 0x00},{ 0x3d4, 0x09, 0x60},{ 0x3d4, 0x10, 0x0a},
	{ 0x3d4, 0x11, 0xac},{ 0x3d4, 0x12, 0xff},{ 0x3d4, 0x13, 0x20},
	{ 0x3d4, 0x14, 0x40},{ 0x3d4, 0x15, 0x07},{ 0x3d4, 0x16, 0x1a},
	{ 0x3d4, 0x17, 0xa3},{ 0x3c4, 0x01, 0x01},{ 0x3c4, 0x04, 0x0e},
	{ 0x3ce, 0x05, 0x40},{ 0x3ce, 0x06, 0x05},{ 0x3c0, 0x10, 0x41},
	{ 0x3c0, 0x13, 0x00}
};

Register scr288x224[] =
{
	{ 0x3c2, 0x0, 0xe3},{ 0x3d4, 0x0, 0x5f},{ 0x3d4, 0x1, 0x47},
	{ 0x3d4, 0x2, 0x50},{ 0x3d4, 0x3, 0x82},{ 0x3d4, 0x4, 0x50},
	{ 0x3d4, 0x5, 0x80},{ 0x3d4, 0x6, 0xb},{ 0x3d4, 0x7, 0x3e},
	{ 0x3d4, 0x8, 0x0},{ 0x3d4, 0x9, 0x41},{ 0x3d4, 0x10, 0xda},
	{ 0x3d4, 0x11, 0x9c},{ 0x3d4, 0x12, 0xbf},{ 0x3d4, 0x13, 0x24},
	{ 0x3d4, 0x14, 0x40},{ 0x3d4, 0x15, 0xc7},{ 0x3d4, 0x16, 0x4},
	{ 0x3d4, 0x17, 0xa3},{ 0x3c4, 0x1, 0x1},{ 0x3c4, 0x4, 0xe},
	{ 0x3ce, 0x5, 0x40},{ 0x3ce, 0x6, 0x5},{ 0x3c0, 0x10, 0x41},
	{ 0x3c0, 0x13, 0x0}
};

Register scr288x224scanlines[] =
{
	{ 0x3c2, 0x0, 0xe3},{ 0x3d4, 0x0, 0x5f},{ 0x3d4, 0x1, 0x47},
	{ 0x3d4, 0x2, 0x47},{ 0x3d4, 0x3, 0x82},{ 0x3d4, 0x4, 0x50},
	{ 0x3d4, 0x5, 0x9a},{ 0x3d4, 0x6, 0xb},{ 0x3d4, 0x7, 0x19},
	{ 0x3d4, 0x8, 0x0},{ 0x3d4, 0x9, 0x40},{ 0x3d4, 0x10, 0xf5},
	{ 0x3d4, 0x11, 0xac},{ 0x3d4, 0x12, 0xdf},{ 0x3d4, 0x13, 0x24},
	{ 0x3d4, 0x14, 0x40},{ 0x3d4, 0x15, 0xc7},{ 0x3d4, 0x16, 0x4},
	{ 0x3d4, 0x17, 0xa3},{ 0x3c4, 0x1, 0x1},{ 0x3c4, 0x4, 0xe},
	{ 0x3ce, 0x5, 0x40},{ 0x3ce, 0x6, 0x5},{ 0x3c0, 0x10, 0x41},
	{ 0x3c0, 0x13, 0x0}
};

Register scr320x204[] =
{
	{ 0x3c2, 0x00, 0xe3},{ 0x3d4, 0x00, 0x5f},{ 0x3d4, 0x01, 0x4f},
	{ 0x3d4, 0x02, 0x50},{ 0x3d4, 0x03, 0x82},{ 0x3d4, 0x04, 0x54},
	{ 0x3d4, 0x05, 0x80},{ 0x3d4, 0x06, 0xbf},{ 0x3d4, 0x07, 0x1f},
	{ 0x3d4, 0x08, 0x00},{ 0x3d4, 0x09, 0x41},{ 0x3d4, 0x10, 0x9c},
	{ 0x3d4, 0x11, 0x8e},{ 0x3d4, 0x12, 0x97},{ 0x3d4, 0x13, 0x28},
	{ 0x3d4, 0x14, 0x40},{ 0x3d4, 0x15, 0x96},{ 0x3d4, 0x16, 0xb9},
	{ 0x3d4, 0x17, 0xa3},{ 0x3c4, 0x01, 0x01},{ 0x3c4, 0x04, 0x0e},
	{ 0x3ce, 0x05, 0x40},{ 0x3ce, 0x06, 0x05},{ 0x3c0, 0x10, 0x41},
	{ 0x3c0, 0x13, 0x00}
};



/* Create a display screen, or window, large enough to accomodate a bitmap */
/* of the given dimensions. I don't do any test here (224x288 will just do */
/* for now) but one could e.g. open a window of the exact dimensions */
/* provided. Return a osd_bitmap pointer or 0 in case of error. */
struct osd_bitmap *osd_create_display(int width,int height)
{
	Register *reg = 0;
	int reglen = 0;


	if (!(width == 224 && height == 288) &&
			!(width == 256 && height == 256) &&
			!(width == 288 && height == 224) &&
			!(width == 320 && height == 204))
		use_vesa = 1;

	if (use_vesa)
	{
		if (set_gfx_mode(SCREEN_MODE,VESA_SCREEN_WIDTH,VESA_SCREEN_HEIGHT,0,0) != 0)
			return 0;
	}
	else if (use_vesascan)
	{
		/* Perhaps it fits into a 640x480 screen? No need to use 800x600 then. */

		if (height <= 240 || use_vesaskip == 1)
		{
			vs_height = 480;
			vs_width = 640;
		}
		else
		{
			vs_height = 600;
			vs_width = 800;
			vs_skiplines = 0;
		}

		if (set_gfx_mode(SCREEN_MODE,vs_width,vs_height,0,0) != 0)
			return 0;


		/* Calculate the offsets to center the image */
		vs_yoffset = (vs_height - height * 2) / 2;
		if (vs_yoffset < 0) vs_yoffset = 0;
		vs_xoffset = (vs_width - width * 2) / 2;
		if (vs_xoffset < 0) vs_xoffset = 0;

		vs_lastline = height - vs_skiplines;
		if (vs_height/2 < vs_lastline)
			vs_lastline = vs_height / 2;
	}
	else
	{
		/* big hack: open a mode 13h screen using Allegro, then load the custom screen */
		/* definition over it. */
		if (set_gfx_mode(GFX_VGA,320,200,0,0) != 0)
			return 0;

		if (width == 224 && height == 288)
		{
			if (noscanlines)
			{
				reg = scr224x288;
				reglen = sizeof(scr224x288)/sizeof(Register);
			}
			else
			{
				reg = scr224x288scanlines;
				reglen = sizeof(scr224x288scanlines)/sizeof(Register);
			}
		}
		else if (width == 256 && height == 256)
		{
			if (noscanlines)
			{
				reg = scr256x256;
				reglen = sizeof(scr256x256)/sizeof(Register);
			}
			else
			{
				reg = scr256x256scanlines;
				reglen = sizeof(scr256x256scanlines)/sizeof(Register);
			}
		}
		else if (width == 288 && height == 224)
		{
			if (noscanlines)
			{
				reg = scr288x224;
				reglen = sizeof(scr288x224)/sizeof(Register);
			}
			else
			{
				reg = scr288x224scanlines;
				reglen = sizeof(scr288x224scanlines)/sizeof(Register);
			}
		}
		else if (width == 320 && height == 204)
		{
			reg = scr320x204;
			reglen = sizeof(scr320x204)/sizeof(Register);
		}

		/* set the VGA clock */
		reg[0].value = (reg[0].value & 0xf3) | (videofreq << 2);

		outRegArray(reg,reglen);
	}

	bitmap = osd_create_bitmap(width,height);

	return bitmap;
}



/* shut up the display */
void osd_close_display(void)
{
	set_gfx_mode(GFX_TEXT,80,25,0,0);
	osd_free_bitmap(bitmap);
}



int osd_obtain_pen(unsigned char red, unsigned char green, unsigned char blue)
{
	RGB rgb;


	rgb.r = red >> 2;
	rgb.g = green >> 2;
	rgb.b = blue >> 2;
	set_color(first_free_pen,&rgb);

	return first_free_pen++;
}



/* Update the display. */
/* As an additional bonus, this function also saves the screen as a PCX file */
/* when the user presses F5. This is not required for porting. */
void osd_update_display(void)
{
	if (video_sync) vsync();

	if (use_vesa)
	{
		int y;
		int width4 = bitmap->width / 4;
		unsigned long *lb = (unsigned long *)bitmap->private;


		for (y = 0;y < bitmap->height;y++)
		{
			unsigned long address;


			address = bmp_write_line(screen,y + (VESA_SCREEN_HEIGHT - bitmap->height) / 2)
					+ (VESA_SCREEN_WIDTH - bitmap->width) / 2;
			_dosmemputl(lb,width4,address);
			lb += width4;
		}
	}
 	else if (use_vesascan)
   	{
		int x,y;
		int width4 = bitmap->width / 4;
		unsigned char vesa_line[VESASCAN_MAX_WIDTH];

		unsigned long *lb = (unsigned long *) vesa_line;
		unsigned char *line = (unsigned char *) bitmap->private;
		unsigned char *p;

		line += bitmap->width*vs_skiplines;

		for (y = 0; y < vs_lastline; y++)
		{
			unsigned long address;
			register unsigned char pixel;

			p = vesa_line;

			/* For some strange reason, counting down and comparing
			to zero is way faster. "Vesascan" is almost
			as fast as tweaked modes now. Can this little
			pixel-doubling loop be sped up some more? */

			for (x = bitmap->width ; x != 0 ; x--)
			{
				pixel =*(line++);
				*(p++)=pixel;
				*(p++)=pixel;
			}

			address = bmp_write_line(screen, 2*y + vs_yoffset) + vs_xoffset;

			_dosmemputl(lb,width4*2,address);
		}


		/* Check for PGUP, PGDN and scroll screen */

		if (osd_key_pressed(OSD_KEY_PGDN))
		{
			if (vs_height/2 + vs_skiplines < bitmap->height)
				vs_skiplines++;
		}

		if (osd_key_pressed(OSD_KEY_PGUP))
		{
			if (vs_skiplines > 0) vs_skiplines--;
		}
 	}
	else
	{
		/* copy the bitmap to screen memory */
		_dosmemputl(bitmap->private,bitmap->width * bitmap->height / 4,0xa0000);
	}
}



void osd_update_audio(void)
{
}



void osd_play_sample(int channel,unsigned char *data,int len,int freq,int volume,int loop)
{
}



void osd_play_streamed_sample(int channel,unsigned char *data,int len,int freq,int volume)
{
}



void osd_adjust_sample(int channel,int freq,int volume)
{
}



void osd_stop_sample(int channel)
{
}



/* check if a key is pressed. The keycode is the standard PC keyboard code, as */
/* defined in osdepend.h. Return 0 if the key is not pressed, nonzero otherwise. */
int osd_key_pressed(int keycode)
{
	return key[keycode];
}



/* wait for a key press and return the keycode */
int osd_read_key(void)
{
	clear_keybuf();
	return readkey() >> 8;
}



int joy_b3, joy_b4;

void osd_poll_joystick(void)
{
}



/* check if the joystick is moved in the specified direction, defined in */
/* osdepend.h. Return 0 if it is not pressed, nonzero otherwise. */
int osd_joy_pressed(int joycode)
{
  return 0;
}
