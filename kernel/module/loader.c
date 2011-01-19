/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "types.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "module/header.h"
#include "util/debug.h"
#include "arch/i386.h"

#define DEBUG_MODULE

#ifdef DEBUG_MODULE
#define DLOG(fmt,...) DLOG_PREFIX("module",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

extern const struct module *_module_ptr_list;

typedef struct modruntime {
  const struct module *mod;
  union {
    u32 flags;
    struct {
      u32 loaded:1;
    };
  };
} modruntime_t;

typedef struct modsystem {
  modruntime_t *mr;
  u32 cnt;
} modsystem_t;

static modsystem_t module_system;

/* Find index of module in runtime table.  i, j are indices into the
 * name string provided. */
static int
module_find (const char *name, int i, int j)
{
  int n;
  for (n=0; n<module_system.cnt; n++) {
    const char *modname = module_system.mr[n].mod->name;
    if (strlen (modname) == j - i && strncmp (modname, name + i, j - i) == 0)
      return n;
  }
  return -1;
}

static bool module_load_i (const int);

/* Initialize a single disjunction of modules by |-separated names.
 * Iff any succeeds, then return TRUE.  Try them all, regardless. */
static bool
module_load_disj (const char *names)
{
  int i, j;
  bool success = FALSE;
  for (i=0, j=0; names[j]; i=j+1) {
    j=i;
    while (names[j] && names[j]!='|') j++;
    success |= module_load_i (module_find (names, i, j));
  }
  return success;
}

/* Initialize a single module by index */
static bool
module_load_i (const int modi)
{
  int depi;
  modruntime_t *mr = module_system.mr;

  if (modi < 0) return FALSE;
  if (mr[modi].loaded) return TRUE;
  for (depi=0; depi<mr[modi].mod->num_dependencies; depi++)
    if (!module_load_disj (mr[modi].mod->dependencies[depi]))
      return FALSE;
  if (mr[modi].mod->ops->init ()) {
    mr[modi].loaded = 1;
    DLOG ("initialized module \"%s\"", mr[modi].mod->name);
    return TRUE;
  } else {
    DLOG ("failed to initialize module \"%s\"", mr[modi].mod->name);
    return FALSE;
  }
}

/* Initialize module runtime information and load all modules */
extern bool
module_load_all (void)
{
  u32 count = 0, pages = 0, i;
  u32 phys;
  modruntime_t *mr;
  DLOG ("loading all modules");
  const struct module **mod;
  for (mod = &_module_ptr_list; *mod; mod++) {
    DLOG ("found name=\"%s\" desc: %s", (*mod)->name, (*mod)->desc);
    count++;
  }
  if (!count) return TRUE;
  pages = ((count * sizeof (modruntime_t) - 1) >> 12) + 1;
  phys = alloc_phys_frames (pages);
  if (phys == (u32) -1) goto abort;
  mr = map_contiguous_virtual_pages (phys | 3, pages);
  if (!mr) goto abort_phys;
  memset (mr, 0, pages << 12);

  module_system.mr = mr;
  module_system.cnt = count;
  for (i=0, mod = &_module_ptr_list; *mod; i++, mod++) {
    mr[i].mod = *mod;
  }

  for (i=0; i<count; i++)
    module_load_i (i);

  return TRUE;
 abort_phys:
  free_phys_frames (phys, pages);
 abort:
  return FALSE;
}

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
