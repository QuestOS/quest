#include "driver.h"
#include <time.h>


#if defined (UNIX) || defined (__MWERKS__)
#define uclock_t clock_t
#define	uclock clock
#define UCLOCKS_PER_SEC CLOCKS_PER_SEC
#endif


static struct RunningMachine machine;
struct RunningMachine *Machine = &machine;
static const struct GameDriver *gamedrv;
static const struct MachineDriver *drv;

static int hiscoreloaded;
static char hiscorename[50];


int frameskip;


#define MAX_COLORS 256	/* can't handle more than 256 colors on screen */
#define MAX_COLOR_TUPLE 16	/* no more than 4 bits per pixel, for now */
#define MAX_COLOR_CODES 256	/* no more than 256 color codes, for now */


unsigned char *RAM;
unsigned char *ROM;


static unsigned char remappedtable[MAX_COLOR_TUPLE*MAX_COLOR_CODES];


#define DEFAULT_NAME "pacman"


FILE *errorlog;


int main(int argc,char **argv)
{
  int i,log,success;

  success = 1;

  log = 0;
  for (i = 1;i < argc;i++)
    {
      if (strcmp(argv[i],"-log") == 0)
	log = 1;
    }

  if (log) errorlog = fopen("error.log","wa");

  if (init_machine(argc > 1 && argv[1][0] != '-' ? argv[1] : DEFAULT_NAME,argc,argv) == 0)
    {
      if (osd_init(argc,argv) == 0)
	{
	  if (run_machine(argc > 1 && argv[1][0] != '-' ? argv[1] : DEFAULT_NAME) == 0)
	    success = 0;
	  else printf("Unable to start emulation\n");
	  
	  osd_exit();
	}
      else printf("Unable to initialize system\n");

#if 0				/* --RW-- */
      /* free the memory allocated for ROM and RAM */
      for (i = 0;i < MAX_MEMORY_REGIONS;i++)
	{
	  free(Machine->memory_region[i]);
	  Machine->memory_region[i] = 0;
	}
#endif
    }
  else printf("Unable to initialize machine emulation\n");

  if (errorlog) fclose(errorlog);

  return success;
}



/***************************************************************************

  Initialize the emulated machine (load the roms, initialize the various
  subsystems...). Returns 0 if successful.

***************************************************************************/
int init_machine(const char *gamename,int argc,char **argv)
{
  int i;
  const struct MemoryReadAddress *mra;
  const struct MemoryWriteAddress *mwa;


  frameskip = 0;
  for (i = 1;i < argc;i++)
    {
      if (strcmp(argv[i],"-frameskip") == 0)
	{
	  i++;
	  if (i < argc)
	    {
	      frameskip = atoi(argv[i]);
	      if (frameskip < 0) frameskip = 0;
	      if (frameskip > 3) frameskip = 3;
	    }
	}
    }
  printf("gamename = %s\n", gamename);
  i = 0;
  while (drivers[i] && strcmp(gamename,drivers[i]->name) != 0)
    i++;

  if (drivers[i] == 0)
    {
      printf("game \"%s\" not supported\n",gamename);
      return 1;
    }

  Machine->gamedrv = gamedrv = drivers[i];
  Machine->drv = drv = gamedrv->drv;

  if (readroms(gamedrv->rom,gamename) != 0)
    return 1;

  RAM = Machine->memory_region[drv->cpu[0].memory_region];
  ROM = RAM;

  /* decrypt the ROMs if necessary */
  if (gamedrv->rom_decode)
    {
      int j;


      for (j = 0;j < 0x10000;j++)
	RAM[j] = (*gamedrv->rom_decode)(j);
    }

  if (gamedrv->opcode_decode)
    {
      int j;


      /* find the first avaialble memory region pointer */
      j = 0;
      while (Machine->memory_region[j]) j++;

      if ((ROM = malloc(0x10000)) == 0)
	return 1;

      Machine->memory_region[j] = ROM;

      for (j = 0;j < 0x10000;j++)
	ROM[j] = (*gamedrv->opcode_decode)(j);
    }


  /* initialize the memory base pointers for memory hooks */
  mra = drv->cpu[0].memory_read;
  while (mra->start != -1)
    {
      if (mra->base) *mra->base = &RAM[mra->start];
      mra++;
    }
  mwa = drv->cpu[0].memory_write;
  while (mwa->start != -1)
    {
      if (mwa->base) *mwa->base = &RAM[mwa->start];
      mwa++;
    }

  if (*drv->init_machine && (*drv->init_machine)(gamename) != 0)
    return 1;

  if (*drv->vh_init && (*drv->vh_init)(gamename) != 0)
    return 1;

  if (*drv->sh_init && (*drv->sh_init)(gamename) != 0)
    return 1;

  return 0;
}



void vh_close(void)
{
  int i;

  /* --RW-- */
  clearbitmap(Machine->scrbitmap); 
  osd_update_display(); 

  for (i = 0;i < MAX_GFX_ELEMENTS;i++) freegfx(Machine->gfx[i]);
  osd_close_display();

}



int vh_open(void)
{
  int i;
  unsigned char pens[MAX_COLORS];
  const unsigned char *palette,*colortable;
  unsigned char convpalette[3 * MAX_COLORS];
  unsigned char convtable[MAX_COLOR_TUPLE*MAX_COLOR_CODES];
  struct DisplayText dt[2];


  if ((Machine->scrbitmap = osd_create_display(drv->screen_width,drv->screen_height)) == 0)
    return 1;

  if (drv->vh_convert_color_prom)
    {
      (*drv->vh_convert_color_prom)(convpalette,convtable,gamedrv->color_prom);
      palette = convpalette;
      colortable = convtable;
    }
  else
    {
      palette = gamedrv->palette;
      colortable = gamedrv->colortable;
    }

  for (i = 0;i < drv->total_colors;i++)
    pens[i] = osd_obtain_pen(palette[3*i],palette[3*i+1],palette[3*i+2]);

  Machine->background_pen = pens[0];

  for (i = 0;i < drv->color_table_len;i++)
    remappedtable[i] = pens[colortable[i]];


  for (i = 0;i < MAX_GFX_ELEMENTS;i++) Machine->gfx[i] = 0;

  for (i = 0;i < MAX_GFX_ELEMENTS && drv->gfxdecodeinfo[i].memory_region != -1;i++)
    {
      if ((Machine->gfx[i] = decodegfx(Machine->memory_region[drv->gfxdecodeinfo[i].memory_region]
				       + drv->gfxdecodeinfo[i].start,
				       drv->gfxdecodeinfo[i].gfxlayout)) == 0)
	{
	  vh_close();
	  return 1;
	}

      Machine->gfx[i]->colortable = &remappedtable[drv->gfxdecodeinfo[i].color_codes_start];
      Machine->gfx[i]->total_colors = drv->gfxdecodeinfo[i].total_color_codes;
    }


  /* free the graphics ROMs, they are no longer needed */
  free(Machine->memory_region[1]);
  Machine->memory_region[1] = 0;


  dt[0].text = "\n\n\nFOR THOSE ABOUT TO PLAY WE SALUTE YOU\n\n"
    "PRESS ESC TO RETURN TO THE SHELL OR ANY "
    "OTHER KEY TO LET THE GAME BEGIN";

  dt[0].color = gamedrv->paused_color;
  dt[0].x = 0;
  dt[0].y = 0;
  dt[1].text = 0;
  displaytext(dt,0);

  i = osd_read_key();
  while (osd_key_pressed(i));	/* wait for key release */
  if (i == OSD_KEY_ESC) return 1;

  clearbitmap(Machine->scrbitmap);	/* initialize the bitmap to the correct background color */
  osd_update_display();

  return 0;
}



/***************************************************************************

  This function takes care of refreshing the screen, processing user input,
  and throttling the emulation speed to obtain the required frames per second.

***************************************************************************/
int updatescreen(void)
{
  static int framecount = 0;


  /* read hi scores from disk */
  if (hiscoreloaded == 0 && *gamedrv->hiscore_load)
    hiscoreloaded = (*gamedrv->hiscore_load)(hiscorename);

  /* if the user pressed ESC, stop the emulation */
  
  if (osd_key_pressed(OSD_KEY_ESC)) return 1;
  

  /* if the user pressed F3, reset the emulation */
  if (osd_key_pressed(OSD_KEY_F3))
    {
      /* write hi scores to disk */
      if (hiscoreloaded != 0 && *gamedrv->hiscore_save)
	(*gamedrv->hiscore_save)(hiscorename);
      hiscoreloaded = 0;

      return 2;
    }

  if (osd_key_pressed(OSD_KEY_P)) /* pause the game */
    {
      struct DisplayText dt[2];
      int key;


      dt[0].text = "PAUSED";
      dt[0].color = gamedrv->paused_color;
      dt[0].x = gamedrv->paused_x;
      dt[0].y = gamedrv->paused_y;
      dt[1].text = 0;
      displaytext(dt,0);

      while (osd_key_pressed(OSD_KEY_P));	/* wait for key release */
      do
	{
	  key = osd_read_key();

	  if (key == OSD_KEY_ESC) return 1;
	  else if (key == OSD_KEY_TAB)
	    {
	      if (setdipswitches()) return 1;
	      (*drv->vh_update)(Machine->scrbitmap);	/* redraw screen */
	      displaytext(dt,0);
	    }
	} while (key != OSD_KEY_P);
      while (osd_key_pressed(key));	/* wait for key release */
    }

  /* if the user pressed TAB, go to dipswitch setup menu */
  if (osd_key_pressed(OSD_KEY_TAB))
    {
      if (setdipswitches()) return 1;
    }

  /* if the user pressed F4, show the character set */
  if (osd_key_pressed(OSD_KEY_F4))
    {
      if (showcharset()) return 1;
    }

  if (*drv->sh_update)
    {
      (*drv->sh_update)();	/* update sound */
      osd_update_audio();
    }

  if (++framecount > frameskip)
    {
      static int showfps,f11pressed;
      static int throttle = 1,f10pressed;
      uclock_t curr,mtpf;
#define MEMORY 10
      static uclock_t prev[MEMORY];
      static int i,fps;


      framecount = 0;

      if (osd_key_pressed(OSD_KEY_F11))
	{
	  if (f11pressed == 0)
	    {
	      showfps ^= 1;
	      if (showfps == 0) clearbitmap(Machine->scrbitmap);
	    }
	  f11pressed = 1;
	}
      else f11pressed = 0;

      if (osd_key_pressed(OSD_KEY_F10))
	{
	  if (f10pressed == 0) throttle ^= 1;
	  f10pressed = 1;
	}
      else f10pressed = 0;


      (*drv->vh_update)(Machine->scrbitmap);	/* update screen */

      if (showfps)
	{
	  drawgfx(Machine->scrbitmap,Machine->gfx[0],fps/100 + gamedrv->numbers_start,gamedrv->white_text,0,0,0,0,0,TRANSPARENCY_NONE,0);
	  drawgfx(Machine->scrbitmap,Machine->gfx[0],(fps%100)/10 + gamedrv->numbers_start,gamedrv->white_text,0,0,8,0,0,TRANSPARENCY_NONE,0);
	  drawgfx(Machine->scrbitmap,Machine->gfx[0],fps%10 + gamedrv->numbers_start,gamedrv->white_text,0,0,16,0,0,TRANSPARENCY_NONE,0);
	}

      osd_update_display();

      osd_poll_joystick();

      /* now wait until it's time to trigger the interrupt */
      do
	{
	  
	  curr = uclock();
	} while (/*video_sync == 0 && --??--*/ throttle != 0 && (curr - prev[i]) < (frameskip+1) * UCLOCKS_PER_SEC/drv->frames_per_second);

      i = (i+1) % MEMORY;

      mtpf = ((curr - prev[i])/(MEMORY))/2;
      if (mtpf) fps = (UCLOCKS_PER_SEC+mtpf)/2/mtpf;

      prev[i] = curr;
    }
  //printf("Returning from %s\n", __FUNCTION__);

  return 0;
}



/***************************************************************************

  Run the emulation. Start the various subsystems and the CPU emulation.
  Returns non zero in case of error.

***************************************************************************/
int run_machine(const char *gamename)
{
  int res = 1;


  if (vh_open() == 0)
    {
      if (*drv->vh_start == 0 || (*drv->vh_start)() == 0)	/* start the video hardware */
	{
	  if (*drv->sh_start == 0 || (*drv->sh_start)() == 0)	/* start the audio hardware */
	    {
	      FILE *f;
	      char name[100];
	      int i,incount;


	      incount = 0;
	      while (gamedrv->input_ports[incount].default_value != -1) incount++;

	      /* read dipswitch settings from disk */
	      sprintf(name,"%s/%s.dsw",gamename,gamename);
	      if ((f = fopen(name,"rb")) != 0)
		{
		  /* use name as temporary buffer */
		  if (fread(name,1,incount,f) == incount)
		    {
		      for (i = 0;i < incount;i++)
			gamedrv->input_ports[i].default_value = name[i];
		    }
		  fclose(f);
		}

	      /* we have to load the hi scores, but this will be done while */
	      /* the game is running */
	      hiscoreloaded = 0;
	      sprintf(hiscorename,"%s/%s.hi",gamename,gamename);
	      
	      cpu_run();	/* run the emulation! */
	      
	      if (*drv->sh_stop) (*drv->sh_stop)();
	      if (*drv->vh_stop) (*drv->vh_stop)();

	      /* write hi scores to disk */
	      if (hiscoreloaded != 0 && *gamedrv->hiscore_save)
		(*gamedrv->hiscore_save)(hiscorename);

	      /* write dipswitch settings to disk */
	      sprintf(name,"%s/%s.dsw",gamename,gamename);
	      if ((f = fopen(name,"wb")) != 0)
		{
		  /* use name as temporary buffer */
		  for (i = 0;i < incount;i++)
		    name[i] = gamedrv->input_ports[i].default_value;

		  fwrite(name,1,incount,f);
		  fclose(f);
		}
				
	      res = 0;
	    }
	  else printf("Unable to start audio emulation\n");
	}
      else printf("Unable to start video emulation\n");

      vh_close();
    }
  else { printf("Unable to initialize display\n"); 

    /* --RW-- */
    vh_close(); 
  }

  return res;
}
