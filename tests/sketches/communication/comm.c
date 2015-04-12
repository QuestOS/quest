#include <arducomm.h>
#include <ardutime.h>
#include <arduthread.h>
#include <stdio.h>

channel chan;

void setup()
{
	chan = channelInit();
}

void loop(1,20,100)
{
	int i;
	for (i = 0; i < 1000; i++) {
		channelWrite(chan, i);
		printf("writing %d\n", i);
		delay(500);
	}
	while(1);
}

void loop(2,20,100)
{
	int i, res;
	for (i = 0; i < 1000; i++) {
		res = channelRead(chan);
		printf("reading %d\n", res);
		delay(300);
	}
	while(1);
}
