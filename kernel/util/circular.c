/* -*- Mode: C -*- */

#include "arch/i386.h"
#include "sched/sched.h"
#include "util/circular.h"

static inline void
circular_lock (circular *c)
{
  spinlock_lock (&c->lock);
}

static inline void
circular_unlock (circular *c)
{
  spinlock_unlock (&c->lock);
}

/* Insert an element of any size into the circular buffer.  If buffer
 * is full, check if CIRCULAR_FLAG_NOWAIT is set.  If so, return -1,
 * else block. */
static sint32
generic_circular_insert (circular *c, void *elt, uint32 flags)
{
  sint32 ret;

  circular_lock (c);

  while (c->cur_count == c->num_elts) {
    if (flags & CIRCULAR_FLAG_NOWAIT) {
      ret = -1;
      goto finish;
    } else {
      /* place on waitqueue */
      queue_append (&c->ins_waitq, str ());
      circular_unlock (c);
      schedule ();              /* block */
      circular_lock (c);
    }
  }

  /* copy the elt into the insertion point and advance the insertion
   * pointer to the next slot. */
  memcpy (c->insert_ptr, elt, c->elt_size);
  c->insert_ptr += c->elt_size;
  if (c->insert_ptr >= c->buffer_end)
    c->insert_ptr = c->buffer;
  ret = ++c->cur_count;
  wakeup_queue (&c->rem_waitq);

 finish:
  circular_unlock (c);
  return ret;
}

/* Remove an element of any size from the circular buffer and copy to
 * the pointer out_elt.  If buffer is empty, check if
 * CIRCULAR_FLAG_NOWAIT is set.  If so, return -1, else block. */
static sint32
generic_circular_remove (circular *c, void *out_elt, uint32 flags)
{
  sint32 ret;

  circular_lock (c);

  while (c->cur_count == 0) {
    if (flags & CIRCULAR_FLAG_NOWAIT) {
      ret = -1;
      goto finish;
    } else {
      /* place on waitqueue */
      queue_append (&c->rem_waitq, str ());
      circular_unlock (c);
      schedule ();              /* block */
      circular_lock (c);
    }
  }

  /* copy from the removal point into out_elt and advance the removal
   * pointer to the next slot. */
  memcpy (out_elt, c->remove_ptr, c->elt_size);
  c->remove_ptr += c->elt_size;
  if (c->remove_ptr >= c->buffer_end)
    c->remove_ptr = c->buffer;
  ret = --c->cur_count;
  wakeup_queue (&c->ins_waitq);

 finish:
  circular_unlock (c);
  return ret;
}

/* Instead of templates, use explicit element size parameter. */

void
circular_init (circular *c, void *buffer, sint32 num_elts, sint32 elt_size)
{
  c->buffer     = buffer;
  c->insert_ptr = buffer;
  c->remove_ptr = buffer;
  c->buffer_end = buffer + num_elts * elt_size;
  c->num_elts   = num_elts;
  c->elt_size   = elt_size;
  c->cur_count  = 0;
  /* Leave possibility of specialized routines being used here. */
  c->insert     = generic_circular_insert;
  c->remove     = generic_circular_remove;
  spinlock_init (&c->lock);
  c->ins_waitq  = 0;
  c->rem_waitq  = 0;
}

/* vi: set et sw=2 sts=2: */
