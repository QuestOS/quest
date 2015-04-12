#include <stdlib.h>

/* Simpson's four slot asynchronous communication mechanism */
typedef struct {
  unsigned int latest, reading;
  unsigned int slot[2];
  int data[4];
} four_slot_shared_t;

typedef four_slot_shared_t* channel;

void channelWrite(channel chan, int item)
{
  unsigned int pair = !(chan->reading);
  unsigned int index = !(chan->slot[pair]);
	chan->data[pair + 2 * index] = item;
  chan->slot[pair] = index;
  chan->latest = pair;
}

int channelRead(channel chan)
{
  unsigned int index;
  unsigned int pair = !! (chan->latest);
	chan->reading = pair;
  index = !! (chan->slot[pair]);
	return chan->data[pair + 2 * index];
}

channel channelInit()
{
	return malloc(sizeof (four_slot_shared_t));
}

