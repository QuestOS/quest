#ifndef _CIRCULAR_H_
#define _CIRCULAR_H_
#include "types.h"
#include "smp/spinlock.h"
#include "util/circular-defs.h"

struct _circular 
{
  void *buffer, *insert_ptr, *remove_ptr, *buffer_end;
  sint32 num_elts, elt_size, cur_count;
  sint32 (*insert)(struct _circular *, void *, uint32);
  sint32 (*remove)(struct _circular *, void *, uint32);
  spinlock lock;
  uint16 ins_waitq, rem_waitq;
};

typedef struct _circular circular;

static inline sint32
circular_insert (circular *c, void *elt)
{
  return c->insert (c, elt, 0);
}

static inline sint32
circular_insert_nowait (circular *c, void *elt)
{
  return c->insert (c, elt, CIRCULAR_FLAG_NOWAIT);
}

static inline sint32
circular_remove (circular *c, void *out_elt)
{
  return c->remove (c, out_elt, 0);
}

void circular_init (circular *c, void *buffer, sint32 num_elts, sint32 elt_size);

#endif

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
