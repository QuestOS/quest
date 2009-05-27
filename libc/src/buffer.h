#define MAXBLKS 128       /* --??-- Max blocks (holes) in free memory.   */

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

typedef struct frec *frec_p;
typedef char *addrs_t;
typedef void *any_t;

typedef struct frec {
  addrs_t fbp;			/* Free block pointer.                    */  
  size_t size;
  frec_p next;
} frec_t;			/* Free record type.                      */

