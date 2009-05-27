
#ifndef _SIZE_T
typedef int size_t;
#define _SIZE_T 1
#endif

extern int strcmp ( const char *s1, const char *s2 );
extern void *memset( void *s, int c, size_t n );
extern void *memcpy( void *dest, const void *src, size_t n );
extern size_t strlen( const char *s );
extern int memcmp( const void *s1, const void *s2, size_t n );
extern char *strcpy(char *dest, const char *src);
extern char *strncpy( char *s1, const char *s2, int length );
