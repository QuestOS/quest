/* -*- Mode: C; c-file-style: "gnu"; c-basic-offset: 2 -*- */

#ifndef _POW2_H_
#define _POW2_H_
#include "types.h"

void pow2_init (void);
int pow2_alloc (uint32 size, uint8 ** ptr);
void pow2_free (uint8 * ptr);

#endif

/* vi: set et sw=2 sts=2: */
