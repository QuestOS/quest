#ifndef _MEM_H_
#define _MEM_H_

#include "types.h"
void pow2_init(void);
int pow2_alloc(WORD size, BYTE **ptr);
void pow2_free(BYTE *ptr);


#endif
