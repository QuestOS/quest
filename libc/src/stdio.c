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


/* --??-- Future development: possbily break this file into separate
   files for time, string maipulation, etc etc functionality */


#include "stdio.h"
#include "string.h"
#include "ctype.h"
#include "stdlib.h"
#include "buffer.h"
#include "time.h"

#define ALIGN 8

#define ceil(size, align) (( size ) + ( align ) - 1 ) / ( align )
#define rdtsc(x)      __asm__ __volatile__("rdtsc \n\t" : "=A" (*(x)))

static frec_p frhead;           /* Free record-list head.                  */
static frec_p frecs;            /* Allocatable records from free-list.     */

static void MergeRecords (frec_p frp);
static void DelRecord (frec_p prev_frp, frec_p frp);

/* Convert the integer D to a string and save the string in BUF. If
   BASE is equal to 'd', interpret that D is decimal, and if BASE is
   equal to 'x', interpret that D is hexadecimal. */
void itoa (char *buf, int base, int d)
{
  char *p = buf;
  char *p1, *p2;
  unsigned long ud = d;
  int divisor = 10;
     
  /* If %d is specified and D is minus, put `-' in the head. */
  if (base == 'd' && d < 0)
    {
      *p++ = '-';
      buf++;
      ud = -d;
    }
  else if (base == 'x')
    divisor = 16;
     
  /* Divide UD by DIVISOR until UD == 0. */
  do
    {
      int remainder = ud % divisor;
     
      *p++ = (remainder < 10) ? remainder + '0' : remainder + 'a' - 10;
    }
  while (ud /= divisor);
     
  /* Terminate BUF. */
  *p = 0;
     
  /* Reverse BUF. */
  p1 = buf;
  p2 = p - 1;
  while (p1 < p2)
    {
      char tmp = *p1;
      *p1 = *p2;
      *p2 = tmp;
      p1++;
      p2--;
    }
}


int printf (const char *format, ...) {
  char **arg = (char **) &format;
  int c;
  char buf[20];
  int count = 0;
  
  arg++;
  
  while ((c = *format++) != 0)
    {
      if (c != '%') {
        putchar (c);
        count++;
      }
      else
        {
          char *p;
          
          c = *format++;
          switch (c)
            {
            case 'd':
            case 'u':
            case 'x':
              itoa (buf, c, *((int *) arg++));
              p = buf;
              goto string;
              break;

            case 's':
              p = *arg++;
              if (! p)
                p = "(null)";

            string:
              while (*p) {
                putchar (*p++);
                count++;
              }
              break;

            default:
              putchar (*((int *) arg++));
              count++;
              break;
            }
        }
    }
  return count;
}


int sprintf (char *str, const char *format, ...) {
  char **arg = (char **) &format;
  int c;
  char buf[20];
  int count = 0;
  
  arg++;
  
  while ((c = *format++) != 0)
    {
      if (c != '%') {
        *str++ = c;
        count++;
      }
      else
        {
          char *p;
          
          c = *format++;
          switch (c)
            {
            case 'd':
            case 'u':
            case 'x':
              itoa (buf, c, *((int *) arg++));
              p = buf;
              goto string;
              break;

            case 's':
              p = *arg++;
              if (! p)
                p = "(null)";

            string:
              while (*p) {
                *str++ = *p++;
                count++;
              }
              break;

            default:
              *str++ = (*((int *) arg++));
              count++;
              break;
            }
        }
    }
  
  *str = '\0';

  return count;
}


static char arena[1000000];     /* --??-- To configure later for d.m.a. */

void mem_init ( void ) {
  void *memset( void *p, int ch, size_t cb );
  addrs_t baseptr;
  frec_p frp;

  int size = 800000;            /* --??-- (See above) */

  memset(arena, 0, 1000000);

  baseptr = arena; 
  frhead = (frec_p)(baseptr + size); 

  for (frp = frhead; frp < frhead + MAXBLKS; frp++)
    frp->next = frp + 1;

  frhead->next = NULL;
  (frhead + MAXBLKS - 1)->next = NULL;
  frecs = frhead + 1;
  frhead->fbp = baseptr;
  frhead->size = size;
}


void *malloc (size_t size) {

  frec_p frp, prev_frp;
  addrs_t frstart;

  size += 8;                    /* Allocate space for tracking size 
                                   of actual allocation */

  prev_frp = frp = frhead;

  while (frp) {
    if (frp->size >= (int)(ALIGN * ceil(size, ALIGN))) {

      /* Have found space. */
      frstart = frp->fbp;
      frp->fbp += (int)(ALIGN * ceil(size, ALIGN));
      frp->size -= (int)(ALIGN * ceil(size, ALIGN));
      *((int *)frstart) = size; /* Store size allocated */
      
      /* If block is only partially allocated then return. */
      if (frp->size)
        return (frstart+8);

      /* Complete block is allocated. Adjust free record list. */
      DelRecord (prev_frp, frp);
      return (frstart+8);

    }
    prev_frp = frp;
    frp = frp->next;
  }
  return NULL;
}


static void DelRecord (frec_p prev_frp, frec_p frp) {

  if (frp == frhead)
    frhead = frp->next;
  else
    prev_frp->next = frp->next;

  frp->next = frecs;
  frecs = frp;
}


void free (void *addr) {

  frec_p frp, new_frp, prev_frp;
  size_t size;

  addr -= 8;                    /* Decrement address to find size field */

  size = *((int *) addr );

  if ((new_frp = frecs) == NULL) {
    /* fprintf (stderr, "No free records.\n"); --??-- */
    exit (1);
  }

  new_frp->fbp = (addrs_t) addr;
  new_frp->size = (int)(ALIGN * ceil(size, ALIGN));
  frecs = new_frp->next;
  frp = frhead;

  /* If this block's address is less than lowest block address
     currently available, or if no block records are available at this
     time, put this block on the front of the free record list. */

  if (frp == NULL || (addrs_t)addr <= frp->fbp) {
    new_frp->next = frp;
    frhead = new_frp;
    MergeRecords (new_frp);
    return;
  }

  /* Deallocated block does not go on front of free record list. */

  while (frp && (addrs_t)addr > frp->fbp) {
    prev_frp = frp;
    frp = frp->next;
  }

  new_frp->next = prev_frp->next;
  prev_frp->next = new_frp;
  MergeRecords (prev_frp);
}


static void MergeRecords (frec_p frp) {

  frec_p next_frp;

  /* Merge contiguous records if possible. */
  
  if ((next_frp = frp->next) == NULL)
    return;

  if (frp->fbp + frp->size == next_frp->fbp) {
    frp->size += next_frp->size;
    DelRecord (frp, next_frp);
  }
  else
    frp = next_frp;

  if ((next_frp = frp->next) == NULL)
    return;

  if (frp->fbp + frp->size == next_frp->fbp) {
    frp->size += next_frp->size;
    DelRecord (frp, next_frp);
  }
}

__attribute__((noreturn)) void exit( int status ) {

  /* --??--Add functionality here for cleaning up user-level resources e.g., in
     UNIX this would be stdio-related buffers etc, followed by calling exit
     handlers... */

  _exit( status );

}


static unsigned long int next = 1;

int rand(void) { /* RAND_MAX assumed to be 32767 */

  next = next * 1103515245 + 12345;
  return (unsigned int)(next/65536) % 32768;
}

void srand(unsigned int seed) {
  next = seed;
}

/*
 * --!!-- not a full implementation but good enough for most cases
 * hasn't been tested
 */
int atoi(const char *nptr) {
  int negative = FALSE;
  int result = 0;

  while(*nptr == ' ') ++nptr;

  if(*nptr == '-') {
    negative = TRUE;
    nptr++;
  }

  while(*nptr == ' ') ++nptr;

  while(*nptr >= '0' && *nptr <= '9')
    result = result * 10 + *nptr++ - '0';

  return negative ? -1 * result : result;
}

clock_t clock( void ) {

  return( time() * 10000 );     /* Time returned in microseconds */
}

size_t strlen( const char *s ) {

  int i;

  i = 0;
  while( s[i] != '\0' )
    ++i;
  return i;

}


char *strcpy( char *s1, const char *s2 ) {

  while( *s1++ = *s2++ );
  
  return s1;
}


char *strncpy( char *s1, const char *s2, int length ) {

  while( ( length-- ) && ( *s1++ = *s2++ ) );
  
  if( length < 0 )
    *(s1-1) = '\0';
  
  return s1;
}


extern char *strdup(const char *s) {
  char* dup;
  size_t len = strlen(s);
  dup = malloc(len);

  if(dup) {
    memcpy(dup, s, len);
  }
  
  return dup;
}


int strcmp(const char *s1, const char *s2) {

  for( ; *s1 == *s2; s1++, s2++ )
    if( *s1 == '\0' )
      return 0;
  return *s1 - *s2;

}


void *memset( void *p, int ch, size_t cb ) {

    asm volatile( "rep stosb" : : "D" (p), "a" (ch), "c" (cb) );
    return p;
}


void *memcpy( void *pDest, const void *pSrc, size_t cb ) {

    asm volatile( "rep movsb" : : "D" (pDest), "S" (pSrc), "c" (cb) );
    return pDest;
}


int memcmp( const void *p1, const void *p2, size_t cb ) {

  const char *s1 = p1, *s2 = p2;

  for( ; cb && *s1 == *s2; s1++, s2++, cb-- );
  if( !cb )
    return 0;
  
  return *s1 - *s2;
}


/* --??-- Needs future enhancement. Right now, strongly tied to fopen */
static int fp;
static int filesize;
static char tmp_buf[8192];              /* Ugly! */

size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream) {
  
  /* Don't actually read -- just index into tmp_buf */
  if( size * nmemb + fp > filesize )
    nmemb = ( filesize - fp ) / size;

  memcpy( ptr, tmp_buf + fp, size * nmemb );
  fp += size * nmemb;
  
  return nmemb;
}


/* --??-- Needs future enhancement */
FILE *fopen(const char *path, const char *mode) {
  
  static FILE f;

  if( ( filesize = open( path, 0 ) ) < 0 )
    return NULL;
  read( (char *)path, tmp_buf, filesize );
  fp = 0;

  return &f;
  
}


/* --??-- To be implemented */
size_t   fwrite(const  void  *ptr,  size_t  size,  size_t  nmemb,  FILE *stream) {


  return 0;
}


/* --??-- To be implemented */
int fclose(FILE *stream) {

  return 0;
}


/* --??-- To be implemented */
int fprintf(FILE *stream, const char *format, ...) {

}


int puts(const char *s) {
  
  while( *s )
    putchar( *s++ );

  putchar( '\n' );
  
  return( 1 );
}


void _start ( int argc, char *argv[] ) {

  mem_init();
  exit ( main( argc, argv ) );
}


/*
 * Taken from http://bytes.com/topic/c/answers/223610-comparing-two-strcasecmp-stricmp-implementations
 */
int strcasecmp(const char *s1, const char *s2)
{
  unsigned char c1,c2;
  do {
    c1 = *s1++;
    c2 = *s2++;
    c1 = (unsigned char) tolower( (unsigned char) c1);
    c2 = (unsigned char) tolower( (unsigned char) c2);
  }
  while((c1 == c2) && (c1 != '\0'));
  return (int) c1-c2;
}


int tolower(int c)
{
  return ('A' <= c) && (c <= 'Z') ? 'a' + (c - 'A') : c;
}

char *strchr(const char *s, int c)
{
  unsigned int i;
  size_t len = strlen(s);

  for(i = 0; i <= len; ++i) {
    if(s[i] == c) return (char *)&s[i];
  }

  return NULL;
}

// s1 = haystack, s2 = needle
char *strstr(const char *s1, const char *s2)
{
  int i, j, diff;
  size_t haystack_len, needle_len;

  haystack_len = strlen(s1);
  needle_len = strlen(s2);
  char needle_start = s2[0];
  diff = haystack_len - needle_len;
  
  for(i = 0; i < diff; ++i) {
    if(needle_start == s1[i]) {
      for(j = 1; j < needle_len; ++j) {
        if(s2[j] != s1[j+i]) break;
      }
      if(j == needle_len) return (char *)&s1[i];
    }
  }

  return NULL;
}


void bzero(void *s, size_t n){

  memset(s, 0, n);
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
