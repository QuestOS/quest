/*
 * An adaptation of MAME 0.2 for X11 / 0.81 for DOS.
 * Can be played. Has only been tested on Linux, but should port easily.
 *
 * All thanks go to Nicola Salmoria for building a clean cut MSDOS emu.
 * Most of the adaptation of XMultiPac (a previous version of the
 * X-interface) was a copy/paste from his 'msdos.c'.
 *
 * ----- HELP! ---------------------------------------------------------------
 *
 * We've had terrible problems implementing sound. If there's anybody out
 * there who thinks she/he can do better: you're welcome. We just don't know
 * enough about it. You can test sound on your machine (Linux only, for
 * now) by specifying -sound on the command-line.
 *
 * Using the SEAL lib (which is available for Windoze 95, NT, Dos & Linux)
 * does not seem to be a good idea. Sound is terrible. Right now, a
 * primitive mixing scheme is implemented using the timer - see the code.
 * Our problem is, that we either have
 * - ticking sound (when we do not pump enough samples to /dev/dsp), or
 * - sound which is 1-5 seconds too slow (when we pump too much samples).
 * If we use the Buffers (now quoted) we get strange echoing sounds running
 * a few seconds after a sample is played. Note: we're testing on a GUS.
 *
 * ---------------------------------------------------------------------------
 *
 * By the way:
 *
 * - Joystick support should be possible in XFree 3.2 using the
 *   extension. Haven't looked at it yet.
 *
 * Created: March, 5th
 *          Allard van der Bas	avdbas@wi.leidenuniv.nl
 *          Dick de Ridder			dick@ph.tn.tudelft.nl
 *
 * Visit the arcade emulator programming repository at
 *   http://valhalla.ph.tn.tudelft.nl/emul8
 *
 * Revisions: #define DR1, #define DR2, Dick de Ridder, 8-5-97
 *            Some bugfixes after profiling with Purify on SunOS.
 *
 * Some fixes by Jack Patton, 9-5-97.
 * Some fixes by Allard van der Bas 10-9-97 (who also broke the already
 * fragile sound under linux :-) ).
 *
 */

/*
 * Include files.
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>

#ifdef linux
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifndef FREEBSD
#include <sys/soundcard.h>
#else
#include <machine/soundcard.h>
#endif
#include <sys/ioctl.h>
#include <errno.h>
#endif

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>

#include "osdepend.h"

#ifdef UNIX
#define stricmp         strcasecmp
#define strnicmp        strncasecmp
#endif

/*
 * Definitions.
 *
 * Define MITSHM to use the MIT Shared Memory extension. This
 * will speed things up considerably. Should really be a command line switch
 * or auto-detection thing (XQueryExtensions).
 */

#define DR1
#define DR2

typedef unsigned char		byte;

#define OSD_NUMKEYS		(100)
#define OSD_NUMPENS		(256)

#define	TRUE			(1)
#define FALSE			(0)

#define OSD_OK			(0)
#define OSD_NOT_OK		(1)

#define DEBUG(x)

#ifdef MITSHM
#include <sys/ipc.h>
#include <sys/shm.h>
#include <X11/extensions/XShm.h>

XShmSegmentInfo 	shm_info;
#endif

/*
 * Global variables.
 */

Display 			*display;
Window 		  	 	 window;
Colormap 	 		 colormap;
XImage 				*image;
GC 				 gc;
/* Just the default GC - we don't really draw. */

unsigned long  		 	White,
			 	Black;

unsigned long 	*xpixel = NULL;	/* Pixel index values for the colors we need. */


/* Fixed title to say Mame - Jack Patton (jpatton@intserv.com) */
char 	*title 		= "X-MAME v0.9";

int 	 snapshot_no	=	0;
byte	*buffer_ptr;

struct osd_bitmap	*bitmap;
int			 use_joystick;
int			 play_sound;
int			 first_free_pen;

/* Not implemented yet: */

int 			 osd_joy_up,
			 osd_joy_down,
			 osd_joy_left,
			 osd_joy_right;
int 			 osd_joy_b1,
			 osd_joy_b2,
			 osd_joy_b3,
			 osd_joy_b4;

/*
 * Workaround: in DO$, the Allegro library maintains this array.
 * In X, we'll have to do it ourselves. Bummer.
 *
 * Note: this can be speeded up considerably (see the enormous switch
 * statement in osd_key_pressed).
 */

byte			 key[OSD_NUMKEYS];

/*
 * Audio variables.
 */

#define AUDIO_SAMPLE_FREQ			(22050)
#define AUDIO_SAMPLE_BITS			(8)
#define AUDIO_NUM_BUFS				(0)
#define AUDIO_BUFSIZE_BITS			(8)
#define AUDIO_MIX_BUFSIZE			(128)
#define AUDIO_NUM_VOICES			(6)
#define AUDIO_TIMER_FREQ			(48)

int	   audio_fd;

int		audio_vol[AUDIO_NUM_VOICES];
int		audio_dvol[AUDIO_NUM_VOICES];
int		audio_freq[AUDIO_NUM_VOICES];
int		audio_dfreq[AUDIO_NUM_VOICES];
int		audio_len[AUDIO_NUM_VOICES];
byte 		*audio_data[AUDIO_NUM_VOICES];
int		audio_on[AUDIO_NUM_VOICES];

#ifdef linux

void audio_timer (int arg)
{
	static int	in = FALSE;
	int		i, j;
	static byte	buf[AUDIO_MIX_BUFSIZE];
	static int	intbuf[AUDIO_MIX_BUFSIZE];

	if (!in)
	{
		in = TRUE;

/* Do your thang. */

	  for (j = 0; j < AUDIO_MIX_BUFSIZE; j++)
		{
			intbuf[j] = 32768;
		}

		for (i = 0; i < AUDIO_NUM_VOICES; i++)
		{
			if (audio_on[i])
			{
				for (j = 0; j < AUDIO_MIX_BUFSIZE; j++)
				{
   				intbuf[j] |= 32768 + (audio_vol[i]/2) * audio_data[i][((j * audio_freq[i]) / AUDIO_SAMPLE_FREQ) % audio_len[i]];
   			}

   			audio_dvol[i]  = FALSE;
   			audio_dfreq[i] = FALSE;
   		}
		}

	  for (j = 0; j < AUDIO_MIX_BUFSIZE; j++)
		{
			buf[j] = intbuf[j];
			if (intbuf[j] < 0)   	 buf[j] = 0;
			if (intbuf[j] > 65535) buf[j] = 255;
		}

		if (write (audio_fd, buf, AUDIO_MIX_BUFSIZE) <= 0)
		{
			/* OKEE */
		}
		in = FALSE;
	}
}

#endif

/*
 * Put anything here you need to do when the program is started.
 * Return 0 if initialization was successful, nonzero otherwise.
 */

int osd_init (int argc, char *argv[])
{
	int			i;

#ifdef linux
	int			dspfreq, dspbyte, dspstereo;
	struct sigaction 	sig_action;
	struct itimerval	timer_value;
#endif

	play_sound 		= FALSE;

	for (i = 1;i < argc;i++)
	{
#ifdef linux
		if (stricmp(argv[i],"-sound") == 0)
			play_sound = TRUE;
#endif
		if (stricmp(argv[i],"-nojoy") == 0)
			use_joystick = FALSE;
	}

#ifdef linux
	if (play_sound)
	{
	  	dspbyte 	= AUDIO_SAMPLE_BITS;
 		dspfreq 	= AUDIO_SAMPLE_FREQ;
		dspstereo	= 0;

    printf ("Linux sound device initialization...\n");

    if ((audio_fd = open ("/dev/dsp", O_WRONLY | O_NONBLOCK)) < 0)
    {
      printf ("couldn't open audio device\n");
      exit(1);
    }

    if (ioctl (audio_fd, SNDCTL_DSP_SETFMT, &dspbyte))
    {
      printf ("can't set DSP number of bits\n");
      exit (1);
    }
  	else
  	{
  		printf ("DSP set to %d bits\n", dspbyte);
  	}

    if (ioctl (audio_fd, SNDCTL_DSP_STEREO, &dspstereo))
    {
    	printf ("can't set DSP to mono\n");
    	exit (1);
    }
		else
		{
			printf ("DSP set to %s\n", dspstereo ? "stereo":"mono");
		}

    if (ioctl (audio_fd, SNDCTL_DSP_SPEED, &dspfreq))
    {
      printf ("can't set DSP frequency\n");
      exit(1);
    }
		else
		{
			printf ("DSP set to %d Hz\n", dspfreq);
		}
/*

  	i = (AUDIO_BUFSIZE_BITS | (AUDIO_NUM_BUFS << 16));
  	if (ioctl (audio_fd, SNDCTL_DSP_SETFRAGMENT, &i) == -1)
  	{
  		printf ("Can't set DSP buffers\n");
  		exit (1);
  	}
*/
		for (i = 0; i < AUDIO_NUM_VOICES; i++)
		{
			audio_on[i] = FALSE;
		}

		sig_action.sa_handler = audio_timer;

/* I didn't have time to check this out but I get an error under FreeBSD */
/* with this compiling. Sound is broken anyway *shrug* */
/* Jack Patton (jpatton@intserv.com) */

#ifndef FREEBSD_SOUND_WORKAROUND
		sig_action.sa_flags   = SA_NOMASK | SA_RESTART;
#endif
		sigaction (SIGALRM, &sig_action, NULL);

	  	timer_value.it_interval.tv_sec =
 	 	timer_value.it_value.tv_sec    = 0L;
 	 	timer_value.it_interval.tv_usec=
  		timer_value.it_value.tv_usec   = 1000000L / AUDIO_TIMER_FREQ;

	  if (setitimer (ITIMER_REAL, &timer_value, NULL))
	  {
 	 		printf ("Setting the timer failed.\n");
  		return (TRUE);
  	}
	}
#endif

	first_free_pen = 0;

	/* Initialize key array - no keys are pressed. */

	for (i = 0; i < OSD_NUMKEYS; i++)
	{
		key[i] = FALSE;
	}

	return (OSD_OK);
}


/*
 * Cleanup routines to be executed when the program is terminated.
 */

void osd_exit (void)
{
#ifdef linux
  if (play_sound)
  {
	close (audio_fd);
  }
#endif
}


/*
 * Make a snapshot of the screen. Not implemented yet.
 * Current bug : because the path is set to the rom directory, PCX files
 * will be saved there.
 */

int osd_snapshot(void)
{
	return (OSD_OK);
}

struct osd_bitmap *osd_create_bitmap (int width, int height)
{
  struct osd_bitmap  	*bitmap;
  int 			i;
  unsigned char  	*bm;

#ifndef DR1
  if ((bitmap = malloc (sizeof (struct osd_bitmap) +
												(height-1) * sizeof (byte *))) != NULL)
#else
  if ((bitmap = malloc (sizeof (struct osd_bitmap) +
												(height) * sizeof (byte *))) != NULL)
#endif
  {
    bitmap->width = width;
    bitmap->height = height;

    if ((bm = malloc (width * height * sizeof(unsigned char))) == NULL)
    {
      free (bitmap);
      return (NULL);
    }

    for (i = 0;i < height;i++)
		{
      			bitmap->line[i] = &bm[i * width];
		}

		bitmap->private = bm;
	}

	return (bitmap);
}

void osd_free_bitmap (struct osd_bitmap *bitmap)
{
	if (bitmap)
	{
		free (bitmap->private);
		free (bitmap);
		bitmap = NULL;
	}
}

/*
 * Create a display screen, or window, large enough to accomodate a bitmap
 * of the given dimensions. I don't do any test here (640x480 will just do
 * for now) but one could e.g. open a window of the exact dimensions
 * provided. Return 0 if the display was set up successfully, nonzero
 * otherwise.
 *
 * Added: let osd_create_display allocate the actual display buffer. This
 * seems a bit dirty, but is more or less essential for X implementations
 * to avoid a lengthy memcpy().
 */

struct osd_bitmap *osd_create_display (int width, int height)
{
	Screen	 	*screen;
	XEvent		 event;
  	XSizeHints 	 hints;
  	XWMHints 	 wm_hints;
	int		 i;

	/* Allocate the bitmap and set the image width and height. */

	if ((bitmap = malloc (sizeof (struct osd_bitmap) +
												(height-1) * sizeof (unsigned char *))) == NULL)
	{
		return (NULL);
	}

	bitmap->width 	= width;
	bitmap->height =	height;

	/* Open the display. */

  display = XOpenDisplay (NULL);
  if(!display)
  {
  	printf ("OSD ERROR: failed to open the display.\n");
  	return (NULL);
  }

  screen 		= DefaultScreenOfDisplay (display);
  White			= WhitePixelOfScreen (screen);
  Black			= BlackPixelOfScreen (screen);

  colormap		= DefaultColormapOfScreen (screen);
  gc			= DefaultGCOfScreen (screen);

	/* Create the window. No buttons, no fancy stuff. */

  window= XCreateSimpleWindow (display, RootWindowOfScreen (screen), 0, 0,
    			 	bitmap->width, bitmap->height, 0, White, Black);

  if (!window)
  {
  	printf ("OSD ERROR: failed to open a window.\n");
  	return (NULL);
  }

	/*  Placement hints etc. */

  hints.flags		= PSize | PMinSize | PMaxSize;
  hints.min_width	= hints.max_width=hints.base_width= bitmap->width;
  hints.min_height	= hints.max_height=hints.base_height=bitmap->height;
  wm_hints.input	= TRUE;
  wm_hints.flags	= InputHint;

  XSetWMHints 		(display, window, &wm_hints);
  XSetWMNormalHints 	(display, window, &hints);
  XStoreName 		(display, window, title);

/* Map and expose the window. */

  XSelectInput   (display, window, FocusChangeMask | ExposureMask | KeyPressMask | KeyReleaseMask);
  XMapRaised     (display, window);
  XClearWindow   (display, window);
  XAutoRepeatOff (display);
  XWindowEvent   (display, window, ExposureMask, &event);

#ifdef MITSHM

	/* Create a MITSHM image. */

  image= XShmCreateImage (display, DefaultVisualOfScreen (screen), 8,
  		  ZPixmap, NULL, &shm_info, bitmap->width, bitmap->height);

  if (!image)
  {
  	printf ("OSD ERROR: failed to create a MITSHM image.\n");
  	return (NULL);
  }

  shm_info.shmid = shmget (IPC_PRIVATE, image->bytes_per_line * image->height,
  			 (IPC_CREAT | 0777));

  if (shm_info.shmid < 0)
  {
  	printf ("OSD ERROR: failed to create MITSHM block.\n");
  	return (NULL);
  }

	/* And allocate the bitmap buffer. */

  buffer_ptr = (byte *)(image->data = shm_info.shmaddr =
			 shmat (shm_info.shmid, 0, 0));

  if (!buffer_ptr)
  {
  	printf ("OSD ERROR: failed to allocate MITSHM bitmap buffer.\n");
  	return (NULL);
  }

  shm_info.readOnly = FALSE;

	/* Attach the MITSHM block. */

  if(!XShmAttach (display, &shm_info))
  {
  	printf ("OSD ERROR: failed to attach MITSHM block.\n");
  	return (NULL);
  }

#else

	/* Allocate a bitmap buffer. */

  buffer_ptr = (byte *) malloc (sizeof(byte) * bitmap->width * bitmap->height);

  if (!buffer_ptr)
  {
  	printf ("OSD ERROR: failed to allocate bitmap buffer.\n");
  	return (NULL);
  }

	/* Create the image. It's a ZPixmap, which means that the buffer will
	 * contain a byte for each pixel. This also means that it will only work
	 * on PseudoColor visuals (i.e. 8-bit/256-color displays).
	 */

  image = XCreateImage (display, DefaultVisualOfScreen (screen), 8,
      			ZPixmap, 0, (char *) buffer_ptr,
      			bitmap->width, bitmap->height, 8, 0);

  if (!image)
  {
    printf ("OSD ERROR: could not create image.\n");
    return (NULL);
  }

#endif

	for (i = 0;i < height; i++)
	{
		bitmap->line[i] = &buffer_ptr[i * width];
	}

	bitmap->private = buffer_ptr;

  return (bitmap);
}

/*
 * Shut down the display.
 */

void osd_close_display (void)
{
	int		i;
	unsigned long	l;

  if (display && window)
  {
#ifdef MITSHM
			/* Detach the SHM. */

      XShmDetach (display, &shm_info);

      if (shm_info.shmaddr)
      {
      	shmdt (shm_info.shmaddr);
      }

      if (shm_info.shmid >= 0)
      {
      	shmctl (shm_info.shmid, IPC_RMID, 0);
      }
#else
	/* Memory will be deallocated shortly (== buffer_ptr). */
#endif

	/* Destroy the image and free the colors. */

    if (image)
    {
    	XDestroyImage (image);
    }

    for (i = 0; i < 16; i++)
    {
      if (xpixel[i] != Black)
      {
      	l = (unsigned long) xpixel[i];
      	XFreeColors (display, colormap, &l, 1, 0);
      }

    	XAutoRepeatOn(display);
  	}

  	if (display)
  	{
  		XAutoRepeatOn (display);
  		XCloseDisplay (display);
  	}
	}

	if (xpixel)
	{
		free (xpixel);
		xpixel = NULL;
		first_free_pen = 0;
	}

/*
	if (bitmap)
	{
		if (bitmap->private)
		{
			free (bitmap->private);
		}
		free (bitmap);
		bitmap = NULL;
	}
*/
}

/*
 * Set the screen colors using the given palette.
 *
 * This function now also sets the palette map. The reason for this is that
 * in X we cannot tell in advance which pixel value (index) will be assigned
 * to which color. To avoid translating the entire bitmap each time we update
 * the screen, we'll have to use the color indices supplied by X directly.
 * This can only be done if we let this function update the color values used
 * in the calling program.... (hack).
 */

int osd_obtain_pen (byte red, byte green, byte blue)
{
	int				i;
	XColor		color;

	/* First time ? */
	if (xpixel == NULL)
	{
		if ((xpixel = (unsigned long *) malloc (sizeof (unsigned long) * OSD_NUMPENS)) == NULL)
		{
			printf ("OSD ERROR: failed to allocate pixel index table.\n");
			return (OSD_NOT_OK);
		}

		for (i = 0; i < OSD_NUMPENS; i++)
		{
			xpixel[i] = Black;
		}
	}

  /* Translate VGA 0-63 values to X 0-65536 values. */

  color.flags	= (DoRed | DoGreen | DoBlue);
  color.red	= 256 * (int) red;
  color.green	= 256 * (int) green;
  color.blue	= 256 * (int) blue;

  if (XAllocColor (display, colormap, &color))
  {
    xpixel[first_free_pen] = color.pixel;
  }

	first_free_pen++;

	return (color.pixel);
}

/*
 * Since *we* control the bitmap buffer and a pointer to the data is stored
 * in 'image', all we need to do is a XPutImage.
 *
 * For compatibility, we accept a parameter 'bitmap' (not used).
 */

void osd_update_display (void)
{
#ifdef MITSHM

  XShmPutImage (display, window, gc, image, 0, 0, 0, 0,
   		bitmap->width, bitmap->height, FALSE);

#else

  XPutImage (display, window, gc, image, 0, 0, 0, 0,
  		 bitmap->width, bitmap->height);

#endif

  XFlush (display);
}

/*
 * Wait until player wants to resume game.
 */

/* Key handling was way to fast using this routine */
int slowdown = 30;

int osd_read_key (void)
{
	int	i;

	i = OSD_NUMKEYS;

	while (i == OSD_NUMKEYS)
	{
		osd_key_pressed (-1);

		i = 0;
		while (!key[i++] && (i < OSD_NUMKEYS));
	}
        if (slowdown == 0) {
          slowdown = 30;
	  return (i-1);
        }
	else {
	  slowdown--;
	  return (0);
	}

}

/*
 * Check if a key is pressed. The keycode is the standard PC keyboard code,
 * as defined in osdepend.h.
 *
 * Handles all incoming keypress/keyrelease events and updates the 'key'
 * array accordingly.
 *
 * This poses some problems, since keycodes returned from X aren't quite the
 * same as normal PC keyboard codes. For now, we only catch the keys we're
 * interested in in a switch-statement. Could be improved so that programs
 * don't expect osdepend.c to map keys to a certain value.
 *
 * Return 0 if the key is not pressed, nonzero otherwise.
 *
 */

int osd_key_pressed (int request)
{
  XEvent 		E;
  int	 		keycode;
  int			mask;

  while (XCheckWindowEvent (display,window, KeyPressMask | KeyReleaseMask, &E))
  {
  	mask = TRUE; 	/* In case of KeyPress. */

    if ((E.type == KeyPress) || (E.type == KeyRelease))
    {
			if (E.type == KeyRelease)
			{
				mask = FALSE;
			}

	    keycode = XLookupKeysym ((XKeyEvent *) &E, 0);

      switch (keycode)
      {
        case XK_Escape:
        {
        	key[OSD_KEY_ESC] = mask;
        	break;
        }
        case XK_Down:
        {
        	key[OSD_KEY_DOWN] = mask;
       		break;
       	}
       	case XK_Up:
       	{
       		key[OSD_KEY_UP] = mask;
       		break;
       	}
       	case XK_Left:
       	{
       		key[OSD_KEY_LEFT] = mask;
       		break;
       	}
       	case XK_Right:
       	{
       		key[OSD_KEY_RIGHT] = mask;
       		break;
       	}
       	case XK_Control_L:
       	case XK_Control_R:
       	{
       		key[OSD_KEY_CONTROL] = mask;
       		break;
       	}
       	case XK_Tab:
       	{
       		key[OSD_KEY_TAB] = mask;
       		break;
       	}
      	case XK_F1:
      	{
      		key[OSD_KEY_F1] = mask;
      		break;
      	}
      	case XK_F2:
      	{
      		key[OSD_KEY_F2] = mask;
      		break;
      	}
      	case XK_F3:
      	{
       		key[OSD_KEY_F3] = mask;
      		break;
      	}
      	case XK_F4:
      	{
      		key[OSD_KEY_F4] = mask;
      		break;
      	}
      	case XK_F5:
      	{
      		key[OSD_KEY_F5] = mask;
      		break;
      	}
      	case XK_F6:
      	{
      		key[OSD_KEY_F6] = mask;
      		break;
      	}
      	case XK_F7:
      	{
      		key[OSD_KEY_F7] = mask;
      		break;
      	}
      	case XK_F8:
      	{
      		key[OSD_KEY_F8] = mask;
      		break;
      	}
      	case XK_F9:
      	{
      		key[OSD_KEY_F9] = mask;
      		break;
      	}
        case XK_F10:
        {
        	key[OSD_KEY_F10] = mask;
        	break;
        }
        case XK_F11:
        {
        	key[OSD_KEY_F11] = mask;
        	break;
        }
        case XK_F12:
        {
        	key[OSD_KEY_F12] = mask;
        	break;
        }
      	case XK_0:
      	{
      		key[OSD_KEY_0] = mask;
      		break;
      	}
      	case XK_1:
      	{
      		key[OSD_KEY_1] = mask;
      		break;
      	}
      	case XK_2:
      	{
      		key[OSD_KEY_2] = mask;
      		break;
      	}
      	case XK_3:
      	{
      		key[OSD_KEY_3] = mask;
      		break;
      	}
      	case XK_4:
      	{
      		key[OSD_KEY_4] = mask;
      		break;
      	}
      	case XK_5:
      	{
      		key[OSD_KEY_5] = mask;
      		break;
      	}
      	case XK_6:
      	{
      		key[OSD_KEY_6] = mask;
      		break;
      	}
      	case XK_7:
      	{
      		key[OSD_KEY_7] = mask;
      		break;
      	}
      	case XK_8:
      	{
      		key[OSD_KEY_8] = mask;
      		break;
      	}
      	case XK_9:
      	{
      		key[OSD_KEY_9] = mask;
      		break;
      	}

/* Mike Zimmerman noticed this one */
/* Pause now works */
	case XK_p:
	case XK_P:
	{
		key[OSD_KEY_P]=mask;
		break;
	}
/* For Crazy Climber - Jack Patton (jpatton@intserv.com)*/
/* Based on this stuff from ../drivers/cclimber.c */
/* Left stick: OSD_KEY_E, OSD_KEY_D, OSD_KEY_S, OSD_KEY_F, */
/* Right stick: OSD_KEY_I, OSD_KEY_K, OSD_KEY_J, OSD_KEY_L */
/* and values from the keycode variable */

      	case 101:
      	{
      		key[OSD_KEY_E] = mask;
      		break;
      	}
      	case 100:
      	{
      		key[OSD_KEY_D] = mask;
      		break;
      	}
      	case 115:
      	{
      		key[OSD_KEY_S] = mask;
      		break;
      	}
      	case 102:
      	{
      		key[OSD_KEY_F] = mask;
      		break;
      	}
      	case 105:
      	{
      		key[OSD_KEY_I] = mask;
      		break;
      	}
      	case 106:
      	{
      		key[OSD_KEY_J] = mask;
      		break;
      	}
      	case 107:
      	{
      		key[OSD_KEY_K] = mask;
      		break;
      	}
      	case 108:
      	{
      		key[OSD_KEY_L] = mask;
      		break;
      	}
/* Scramble and Space Cobra bomb key OSD_KEY_ALT*/
/* from one of the keysym includes */
/* Jack Patton (jpatton@intserv.com) */
      	case XK_Alt_L:
      	{
      		key[OSD_KEY_ALT] = mask;
      		break;
      	}
      	case XK_Alt_R:
      	{
      		key[OSD_KEY_ALT] = mask;
      		break;
      	}
				default:
				{
				}
      }
		}
	}

#ifndef DR2
		return key[request];
#else
	if (request >= 0)
		return key[request];
	else
		return (FALSE);
#endif
}

void osd_poll_joystick (void)
{
	/* Not implemented yet. */
}

int osd_joy_pressed (int joycode)
{
	/* Not implemented yet. */
	return (FALSE);
}

/*
 * Audio shit.
 */

void osd_update_audio (void)
{
	/* Not used. */
}

void osd_play_sample (int channel, byte *data, int len, int freq,
			int volume, int loop)
{
	if (!play_sound || (channel >= AUDIO_NUM_VOICES))
	{
		return;
 	}

	audio_on[channel]	= TRUE;
	audio_data[channel] 	= data;
	audio_len[channel]  	= len;
	audio_freq[channel]	= freq;
	audio_vol[channel]  	= volume;
}

void osd_play_streamed_sample (int channel, byte *data, int len, int freq, int volume)
{
	/* Not used. */
}

void osd_adjust_sample (int channel, int freq, int volume)
{
	if (play_sound && (channel < AUDIO_NUM_VOICES))
	{
		audio_dfreq[channel] |= (freq    != audio_freq[channel]);
		audio_dvol[channel]  |= (volume  != audio_vol[channel]);
		audio_freq[channel] = freq;
		audio_vol[channel]  = volume;
	}
}

void osd_stop_sample (int channel)
{
	if (play_sound && (channel < AUDIO_NUM_VOICES))
	{
		audio_on[channel] = FALSE;
	}
}
