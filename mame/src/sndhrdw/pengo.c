#include "driver.h"


#define SND_CLOCK 3072000	/* 3.072 Mhz */


unsigned char *pengo_soundregs;
static int sound_enable;
static int sound_changed;



void pengo_sound_enable_w(int offset,int data)
{
	sound_enable = data;
	if (sound_enable == 0)
	{
		osd_adjust_sample(0,1000,0);
		osd_adjust_sample(1,1000,0);
		osd_adjust_sample(2,1000,0);
	}
}



void pengo_sound_w(int offset,int data)
{
	data &= 0x0f;

	if (pengo_soundregs[offset] != data)
	{
		sound_changed = 1;
		pengo_soundregs[offset] = data;
	}
}



void pengo_sh_update(void)
{
	if (play_sound == 0) return;

	if (sound_enable && sound_changed)
	{
		int voice;
		static int currwave[3] = { -1,-1,-1 };


		sound_changed = 0;

		for (voice = 0;voice < 3;voice++)
		{
			int freq,volume,wave;


			freq = pengo_soundregs[0x14 + 5 * voice];	/* always 0 */
			freq = freq * 16 + pengo_soundregs[0x13 + 5 * voice];
			freq = freq * 16 + pengo_soundregs[0x12 + 5 * voice];
			freq = freq * 16 + pengo_soundregs[0x11 + 5 * voice];
			if (voice == 0)
				freq = freq * 16 + pengo_soundregs[0x10 + 5 * voice];
			else freq = freq * 16;

			freq = (SND_CLOCK / 2048) * freq / 512;

			volume = pengo_soundregs[0x15 + 5 * voice];
			volume = (volume << 4) | volume;

			if (freq == 0)
			{
				freq = 1000;
				volume = 0;
			}

			wave = pengo_soundregs[0x05 + 5 * voice] & 7;
			if (wave != currwave[voice])
			{
				osd_play_sample(voice,&Machine->drv->samples[wave * 32],32,freq,volume,1);
				currwave[voice] = wave;
			}
			else osd_adjust_sample(voice,freq,volume);
		}
	}
}
