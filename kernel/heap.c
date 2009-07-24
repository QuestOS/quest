#include <stdlib.h>
#include "heap.h"

/* 
 * Sample heap routines by Rich West.
 *
 * NOTES: - All heap items are array-indexed by the heap-index - 1. 
 *          This is because heap-indices start at 1 and heap items are 
 *          accessed from array-index 0.
 *        - To maintain heap with min-key at root of heap, simply add
 *          the new routines to build heap, insert into heap, and remove
 *          minimum key item from heap. This is trvial to do by changing
 *           > to < and vice versa in the routines that are given.
 *        - The heap items can have any object stored in the heap. 
 */


/* Routine to maintain property of valid heap. */
void
heapify (heap_t * heap, int i)
{

  int left, right;              /* Indices of left and right children. */
  int largest;                  /* Index of heap item with largest key. */
  heap_item tmp_item;           /* Place holder for temporary item. */

  left = LEFT (i);
  right = RIGHT (i);

  if ((left <= heap->size) &&
      ((heap->item[left - 1].key > heap->item[i - 1].key) ||
       ((heap->item[left - 1].key == heap->item[i - 1].key) &&
        (heap->item[left - 1].secondary_key <
         heap->item[i - 1].secondary_key))))
    largest = left;
  else
    largest = i;

  if ((right <= heap->size) &&
      ((heap->item[right - 1].key > heap->item[i - 1].key) ||
       ((heap->item[right - 1].key == heap->item[i - 1].key) &&
        (heap->item[right - 1].secondary_key <
         heap->item[i - 1].secondary_key))))
    largest = right;

  if (largest != i) {
    /* Exchange ith item with largest. */
    tmp_item = heap->item[i - 1];
    heap->item[i - 1] = heap->item[largest - 1];
    heap->item[largest - 1] = tmp_item;

    heapify (heap, largest);
  }
}


/* Reconstruct heap to satisfy heap property. */
void
build_heap (heap_t * heap)
{

  int i;

  for (i = (int) floor ((double) (heap->size) / 2); i >= 1; i--)
    heapify (heap, i);

}


/* Return item from heap with max key. */
heap_item *
heap_extract_max (heap_t * heap, heap_item * max_item)
{

  if (heap == NULL)
    return NULL;

  if (heap->size < 1)
    return NULL;

  *max_item = heap->item[0];
  heap->item[0] = heap->item[heap->size - 1];
  heap->size--;
  if (heap->size > 0)
    heapify (heap, 1);

  return (max_item);
}


/* Insert heap_item into heap at appropriate location. */
heap_t *
heap_insert (heap_t * heap, heap_item * item)
{

  int i = ++(heap->size);

  while ((i > 1) &&
         ((heap->item[PARENT (i) - 1].key < item->key) ||
          ((heap->item[PARENT (i) - 1].key == item->key) &&
           (heap->item[PARENT (i) - 1].secondary_key >
            item->secondary_key)))) {
    heap->item[i - 1] = heap->item[PARENT (i) - 1];
    i = PARENT (i);
  }

  heap->item[i - 1] = *item;

  return (heap);
}


void
heap_init (heap_t * heap)
{

  int i;

  heap->size = 0;
  for (i = 0; i < MAX_HEAP_SIZE; i++) {
    heap->item[i].key = 0;
    heap->item[i].obj = NULL;
    heap->item[i].secondary_key = 0;
  }
}
