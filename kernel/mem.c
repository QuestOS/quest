#include "kernel.h"
#include "mem.h"
#include "spinlock.h"

/* power-of-2 memory allocator works with blocks in increasing sizes
 * in powers of 2, from 2^POW2_MIN_POW to 2^POW2_MAX_POW */

#define POW2_MIN_POW 5
#define POW2_MAX_POW 16
/* NB: the value of POW2_MAX_POW has to be smaller than 2^POW2_MIN_POW
 * because it needs to be able to fit within the POW2_MASK_POW.  Also
 * keep in mind it needs to be able to find a contiguous virtual
 * address range every time it allocates one of these large
 * blocks. */

#define POW2_MIN_SIZE (2<<POW2_MIN_POW)
#define POW2_MAX_SIZE (2<<POW2_MAX_POW)

/* Length of the central header table */
#define POW2_TABLE_LEN ((POW2_MAX_POW - POW2_MIN_POW)+1)

/* How many frames to allocate for a max-size block: */
#define POW2_MAX_POW_FRAMES (1 << (POW2_MAX_POW - 12))

/* Mask the index from a descriptor: */
#define POW2_MASK_POW ((1<<POW2_MIN_POW)-1)

struct _POW2_HEADER {
  struct _POW2_HEADER *next;
  DWORD count;
  BYTE *ptrs[0];
} PACKED;
typedef struct _POW2_HEADER POW2_HEADER;
#define POW2_MAX_COUNT ((0x1000 - sizeof(POW2_HEADER)) >> 2)

static POW2_HEADER *pow2_table[POW2_TABLE_LEN];

static DWORD *pow2_used_table;  /* array of descriptors:
                                 * pointer | index */
static DWORD pow2_used_count, pow2_used_table_pages;

static struct spinlock pow2_lock = SPINLOCK_INIT;

static void pow2_add_free_block(BYTE *ptr, BYTE index) {
  POW2_HEADER *hdr = pow2_table[index - POW2_MIN_POW];

  for(;;) {
    if (hdr->count < POW2_MAX_COUNT) {
      /* There is room in the current header's block */
      hdr->ptrs[hdr->count++] = ptr;
      return;
    } else {
      /* There is not enough room */
      if (hdr->next) {
        /* Chase the next pointer */
        hdr = hdr->next;
      } else {
        /* End of the list -- make a new header */
        hdr->next = MapVirtualPage(AllocatePhysicalPage() | 3);
        memset(hdr->next, 0, 0x1000);
        hdr = hdr->next;
      }
    }
  }
}

static BYTE *pow2_get_free_block(BYTE index);

/* mutual recursion -- lets try it.
 * I'm calculating 20-24 bytes per frame and a maximum of
 * 7*2 calls, so 308 bytes of stack needed at most */
static void pow2_split_block(BYTE index) {
  BYTE *ptr1, *ptr2;
  
  ptr1 = pow2_get_free_block(index);
  ptr2 = ptr1 + (1 << (index-1));

  pow2_add_free_block(ptr1, index-1);
  pow2_add_free_block(ptr2, index-1);
}

/* Trying to avoid making the frame-size of pow2_get_free_block any
 * larger than it is -- because of mutual recursion -- and this is
 * used in a re-entrantly safe fashion. */
static unsigned pow2_tmp_phys_frames[POW2_MAX_POW_FRAMES];

static BYTE *pow2_get_free_block(BYTE index) {
  POW2_HEADER *hdr = pow2_table[index - POW2_MIN_POW], *prev = NULL;
  BYTE *ptr;

  for(;;) {
    if (hdr->count == 0) {
      /* no free blocks -- get one */
      /* hdr->count can only be 0 for first in chain */
      if(index < POW2_MAX_POW) {
        pow2_split_block(index+1);
      } else {
        /* grab new pages */
        int i;
        for(i=0;i<POW2_MAX_POW_FRAMES;i++)
          pow2_tmp_phys_frames[i] = AllocatePhysicalPage() | 3;
        return MapVirtualPages(pow2_tmp_phys_frames, POW2_MAX_POW_FRAMES);
      }
    } else if (hdr->count < POW2_MAX_COUNT || hdr->next == NULL) {
      /* There are free blocks ready to go */
      ptr = hdr->ptrs[--hdr->count];
      if (prev && hdr->count == 0) {
        /* We followed a next pointer to get here. */
        DWORD frame = (DWORD)get_phys_addr((void *)hdr);
        prev->next = NULL;
        UnmapVirtualPage((void *)hdr);
        FreePhysicalPage(frame);
      }
      return ptr;
    } else {
      /* hdr->count == POW2_MAX_COUNT && hdr->next != NULL */
      /* chase next pointer and try to use that header instead */
      prev = hdr;
      hdr = hdr->next;
    }
  }
}

static void pow2_insert_used_table(BYTE *ptr, BYTE index) {
  if(pow2_used_count >= (pow2_used_table_pages * 0x400)) {
    unsigned count = pow2_used_table_pages + 1;
    DWORD frames = AllocatePhysicalPages(count), old_frames;
    void *virt = MapContiguousVirtualPages(frames | 3, count);
    memcpy(virt, pow2_used_table, sizeof(DWORD) * pow2_used_count);
    old_frames = (DWORD)get_phys_addr(pow2_used_table);
    UnmapVirtualPages((void *)pow2_used_table, pow2_used_table_pages);
    FreePhysicalPages(old_frames, pow2_used_table_pages);
    pow2_used_table = (unsigned *)virt;
    pow2_used_table_pages = count;
  }
  pow2_used_table[pow2_used_count++] = (DWORD)ptr | (DWORD)index;
}

static int pow2_remove_used_table(BYTE *ptr, BYTE *index) {
  int i;
  for(i=0;i<pow2_used_count;i++) {
    if ((DWORD)ptr == (pow2_used_table[i] & (~ POW2_MASK_POW))) {
      /* found it */
      *index = pow2_used_table[i] & POW2_MASK_POW;
      pow2_used_table[i] = pow2_used_table[--pow2_used_count];
      pow2_used_table[pow2_used_count] = 0;
      return 0;      
    }
  }
  return -1;
}

static BYTE pow2_compute_index(WORD size) {
  int i;
  if (size <= POW2_MIN_SIZE) return POW2_MIN_POW;
  else if (size >= POW2_MAX_SIZE) return POW2_MAX_POW;
  else {
    size--;
    /* bit scan reverse -- find most significant set bit */
    asm volatile("bsr %1,%0" : "=r"(i) : "r"(size));
    return i;
  }
}

int pow2_alloc(WORD size, BYTE **ptr) {
  BYTE index = pow2_compute_index(size);
  spinlock_lock(&pow2_lock);
  *ptr = pow2_get_free_block(index);
  pow2_insert_used_table(*ptr, index);
  spinlock_unlock(&pow2_lock);
  return index;
}

void pow2_free(BYTE *ptr) {
  BYTE index;
  spinlock_lock(&pow2_lock);
  if(pow2_remove_used_table(ptr, &index) == 0) {
    pow2_add_free_block(ptr, index);
  }
  spinlock_unlock(&pow2_lock);
}
  
void pow2_init(void) {
  int i;
  for(i=0; i<POW2_TABLE_LEN; i++) {
    pow2_table[i] = MapVirtualPage(AllocatePhysicalPage() | 3);
    memset(pow2_table[i], 0, 0x1000);
  }
  pow2_used_table = MapVirtualPage(AllocatePhysicalPage() | 3);
  memset(pow2_used_table, 0, 0x1000);
  pow2_used_count = 0;
  pow2_used_table_pages = 1;
}

