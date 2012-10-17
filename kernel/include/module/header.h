/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
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

#ifndef _MODULE_HEADER_H_
#define _MODULE_HEADER_H_

#include "util/stringify.h"

struct module_ops {
  bool (*init) (void);
};

struct module {
  const char *name, *desc;
  const struct module_ops *ops;
  const u32 num_dependencies;
  const char **dependencies;
};

#define DEF_MODULE(n,d,o,...)                                           \
  static const char *_module_##n##_deps[] = __VA_ARGS__;                \
  static struct module _module_##n = {                                  \
    .name = __stringify(n),                                             \
    .desc = d,                                                          \
    .num_dependencies = sizeof (_module_##n##_deps) / sizeof (s8 *),    \
    .ops = o,                                                           \
    .dependencies = _module_##n##_deps                                  \
  };                                                                    \
  _MODULE_PTR_ATTR const struct module *_module_##n##_ptr = &_module_##n

#define _MODULE_PTR_ATTR __attribute__((section(".module.ptrs"), unused))

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
