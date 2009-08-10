/* -*- Mode: C -*- */

#include "syscall.h"

#ifndef NULL
#define NULL 0
#endif 

#ifndef _SIZE_T
typedef int size_t;
#define _SIZE_T 1
#endif

typedef struct _file {
  int fd;

} FILE;

extern FILE *fopen( const char *filename, const char *mode );
extern int printf( const char *format, ... );
extern int fprintf( FILE *stream, const char *format, ... );
extern int sprintf( char *str, const char *format, ... );

extern void itoa( char *buf, int base, int d );

extern int fclose( FILE *stream );

extern size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream);
extern size_t   fwrite(const  void  *ptr,  size_t  size,  size_t  nmemb,  FILE *stream);

extern int puts( const char *s );

/* vi: set et sw=2 sts=2: */
