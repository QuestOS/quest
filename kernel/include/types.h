/* -*- Mode: C -*- */

#ifndef __TYPES_H__
#define __TYPES_H__

#define INT_MAX 0xFFFFFFFF

#ifndef __ASSEMBLER__

#define TRUE 1
#define FALSE 0
#define PRIVATE static
#define PACKED __attribute__ ((packed))

typedef unsigned char uint8;
typedef unsigned short int uint16;
typedef unsigned long int uint32;
typedef unsigned long long int uint64;

typedef signed char sint8;
typedef signed short int sint16;
typedef signed long int sint32;
typedef signed long long int sint64;

typedef signed char bool;
#endif // [#ifndef __ASSEMBLER__]

#endif


/* vi: set et sw=2 sts=2: */
