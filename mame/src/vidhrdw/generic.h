#include "driver.h"


extern unsigned char *videoram;
extern unsigned char *colorram;
extern unsigned char *spriteram;
extern unsigned char *spriteram_2;
extern unsigned char *dirtybuffer;
extern struct osd_bitmap *tmpbitmap;

extern int generic_vh_start(void);
extern void generic_vh_stop(void);
extern int videoram_r(int offset);
extern int colorram_r(int offset);
extern void videoram_w(int offset,int data);
extern void colorram_w(int offset,int data);
