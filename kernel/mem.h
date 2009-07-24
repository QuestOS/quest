#ifndef _MEM_H_
#define _MEM_H_

#include "types.h"
void pow2_init (void);
int pow2_alloc (uint16 size, uint8 ** ptr);
void pow2_free (uint8 * ptr);


#endif
