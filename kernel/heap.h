#ifndef heap_h
#define heap_h
#endif

#include <math.h>

#define PARENT(i) (int)floor((double)(i)/2)
#define LEFT(i) (2*(i))
#define RIGHT(i) (2*(i)+1)
#define MAX_HEAP_SIZE 128

typedef struct _heap_item
{
  int key;                      /* Priority key for heap item. */
  void *obj;                    /* Ordered object type. */
  long secondary_key;           /* If two heap entries have the same
                                   primary 'key', order on the
                                   secondary-key.  */
} heap_item;


typedef struct _heap_t
{
  int size;                     /* Size of heap. */
  heap_item item[MAX_HEAP_SIZE];        /* Array of heaped items. */
} heap_t;


void heapify (heap_t * heap, int i);
void build_heap (heap_t * heap);
heap_item *heap_extract_max (heap_t * heap, heap_item * max_item);
heap_t *heap_insert (heap_t * heap, heap_item * item);
void heap_init (heap_t * heap);
