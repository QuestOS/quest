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

#include "kernel.h"
#include "mem/mem.h"
#include "fs/filesys.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "drivers/ata/ata.h"

void
iso9660_date_record (uint8 * buf)
{
  com1_printf ("%.4s-%.2s-%.2s %.2s:%.2s:%.2s.%.2s %02d",
               buf, buf + 4, buf + 6,
               buf + 8, buf + 10, buf + 12, buf + 14,
               *((signed char *) buf + 16) / 4);
}


void
iso9660_show_dir_record (uint8 * r)
{
  iso9660_dir_record *d = (iso9660_dir_record *) r;
  int i;
  com1_printf ("dir_record\n  length=%d first_sector=%X data_length=%d\n",
               d->length, d->first_sector, d->data_length);
  com1_printf ("  %d-%02d-%02d %02d:%02d:%02d\n",
               d->years_since_1900 + 1900, d->month, d->day,
               d->hour, d->minute, d->second);
  com1_printf ("  flags: %s %s %s %s %s %s\n",
               d->flag_hidden ? "hidden" : "regular",
               d->flag_dir ? "dir" : "file",
               d->flag_assoc ? "assoc" : "noassoc",
               d->flag_recfmtspec ? "recfmtspec" : "norecfmtspec",
               d->flag_permspec ? "permspec" : "nopermspec",
               d->flag_notfinal ? "notfinal" : "final");
  com1_printf ("  id len=%d: ", d->identifier_length);
  for (i = 0; i < d->identifier_length; i++)
    if (d->identifier[i])
      com1_putc (d->identifier[i]);
  com1_putc ('\n');
}

void
iso9660_walk_tree (uint32 bus, uint32 drive,
                   iso9660_dir_record * d, int depth)
{
  int i, len;

  for (i = 0; i < depth; i++)
    com1_puts ("  ");

  if (d->identifier[0] == 0) {
    com1_puts (".\n");
    if (depth > 0)
      return;
  } else if (d->identifier[0] == 1) {
    com1_puts ("..\n");
    if (depth > 0)
      return;
  } else {
    for (i = 0; i < d->identifier_length; i++)
      if (d->identifier[i])
        com1_putc (d->identifier[i]);
    com1_puts ("\n");
  }

  if (d->flag_dir) {
    uint32 frame = alloc_phys_frame ();
    uint8 *sector = map_virtual_page (frame | 3);

    uint32 secnum = d->first_sector;
    uint32 num_bytes = d->data_length;
    uint32 count;

    for (count = 0; count < num_bytes;) {
      if (count % ATAPI_SECTOR_SIZE == 0) {
        // if 2048-byte aligned get next sector
        len = atapi_drive_read_sector (bus, drive, secnum, sector);
        if (len < 0) {
          panic ("CD ROM READ ERROR\n");
        }
        secnum++;
        d = (iso9660_dir_record *) sector;
      }

      iso9660_walk_tree (bus, drive, d, depth + 1);

      count += d->length;
      d = (iso9660_dir_record *) ((uint8 *) d + d->length);

      // skip zeroes
      while (*((uint8 *)d) == 0 && (count % ATAPI_SECTOR_SIZE) != 0) {
        d = (iso9660_dir_record *) (((uint8 *)d) + 1);
        count++;
      }
    }
    unmap_virtual_page (sector);
    free_phys_frame (frame);
  }
}


int
iso9660_mount (uint32 bus, uint32 drive, iso9660_mounted_info * mi)
{
  uint32 frame = alloc_phys_frame ();
  uint8 *page = map_virtual_page (frame | 3);
  int len;

  /* The first 16 sectors (0-15) are empty. */

  /* Primary Volume descriptor */
  len = atapi_drive_read_sector (bus, drive, 16, page);

  if (len < 0) {
    com1_printf ("CD-ROM read error\n");
    return -1;
  }
#if 0
  {
    int i, j;
    com1_printf ("Read %d bytes.\n", len);
    /* dump to com1 */
    for (i = 0; i < 128; i++) {
      for (j = 0; j < 16; j++) {
        com1_printf ("%.2X ", page[i * 16 + j]);
      }
      com1_printf ("\n");
    }
  }

  com1_printf ("Sys ID: %.32s\n", page + 8);
  com1_printf ("Vol ID: %.32s\n", page + 40);
  com1_printf ("# Sectors: %.8X\n", *(uint32 *) (page + 80));
  com1_printf ("Path Table: len=%.8X fst=", *(uint32 *) (page + 132));
  com1_printf ("%.8X snd=%.8X\n", *(uint32 *) (page + 140),
               *(uint32 *) (page + 144));
  /* root dir @ 156 */
  iso9660_show_dir_record (page + 156);
  com1_printf ("Vol Set ID: %.128s\n", page + 190);
  com1_printf ("Pub ID: %.128s\n", page + 318);
  com1_printf ("Preparer ID: %.128s\n", page + 318 + 128);
  com1_printf ("Application ID: %.128s\n", page + 318 + 256);
  com1_printf ("Copyright File ID: %.37s\n", page + 318 + 384);
  com1_printf ("Abstract File ID: %.37s\n", page + 318 + 384 + 37);
  com1_printf ("Bib File ID: %.37s\n", page + 318 + 384 + 37 + 37);
  com1_printf ("Creation Date: ");
  iso9660_date_record (page + 318 + 384 + 37 + 37 + 37);
  com1_printf ("\n");
  com1_printf ("Modification Date: ");
  iso9660_date_record (page + 318 + 384 + 37 + 37 + 37 + 17);
  com1_printf ("\n");
  com1_printf ("Expiration Date: ");
  iso9660_date_record (page + 318 + 384 + 37 + 37 + 37 + 34);
  com1_printf ("\n");
  com1_printf ("Effective Date: ");
  iso9660_date_record (page + 318 + 384 + 37 + 37 + 37 + 51);
  com1_printf ("\n");

  iso9660_walk_tree (bus, drive, (iso9660_dir_record *) (page + 156), 0);

#endif

  mi->bus = bus;
  mi->drive = drive;
  mi->root_dir_sector = ((iso9660_dir_record *) (page + 156))->first_sector;
  mi->root_dir_data_length =
    ((iso9660_dir_record *) (page + 156))->data_length;

  unmap_virtual_page (page);
  free_phys_frame (frame);
  return 0;
}

/* Parse out the first component of a pathname: the first string
 * portion between the (optional) first slash and the second slash.
 * Uses in/out parameters start and end.  On input they define the
 * bounds of the search.  On output they define the bounds of the
 * first component.  Return value is 0 if this is also the last
 * component, or 1 if there are other components remaining. */
static int
iso9660_parse_first_component (char *pathname, int *start, int *end)
{
  int i;

  /* Skip leading slashes */
  for (; pathname[*start] == PATHSEP; (*start)++);
  /* Find next slash or nul within bounds */
  for (i = *start; pathname[i] != PATHSEP && pathname[i] != '\0' && i < *end;
       i++);
  *end = i;

  return (pathname[*end] == '\0' ? 0 : 1);
}

#define TOUPPER(c) ('a' <= c && c <= 'z' ? c + ('A' - 'a') : c)

static int
iso9660_filename_compare (char *a, int a_len, char *b, int b_len)
{
  int i = 0;
  while (i < a_len && i < b_len) {
    if (TOUPPER (a[i]) != TOUPPER (b[i]))
      return -1;
    i++;
    if (a[i] == '.' && i == b_len)
      return 0;
    else if (i == a_len && b[i] == '.')
      return 0;
    else if (a[i] == ';' && i == b_len)
      return 0;
    else if (i == a_len && b[i] == ';')
      return 0;
    else if (i == a_len && i < b_len)
      return -1;
    else if (i < a_len && i == b_len)
      return -1;
  }
  return 0;
}


static int
iso9660_search_dir (iso9660_mounted_info * mi,
                    iso9660_dir_record * d,
                    char *pathname, int start, int end,
                    iso9660_dir_record * de)
{
  uint32 frame;
  uint8 *page;
  int len = end - start;

  frame = alloc_phys_frame ();
  page = map_virtual_page (frame | 3);

  uint32 secnum = d->first_sector;
  uint32 num_bytes = d->data_length;
  uint32 count;

  for (count = 0; count < num_bytes;) {
    if (count % ATAPI_SECTOR_SIZE == 0) {
      // if 2048-byte aligned get next sector
      if (atapi_drive_read_sector (mi->bus, mi->drive, secnum, page) < 0) {
        panic ("CD ROM READ ERROR\n");
      }
      secnum++;
      d = (iso9660_dir_record *) page;
    }

    if (iso9660_filename_compare (pathname + start, len,
                                  (char *) d->identifier,
                                  d->identifier_length) == 0)
      break;                    /* found it */


    count += d->length;
    d = (iso9660_dir_record *) ((uint8 *) d + d->length);

    // skip zeroes
    while (*((uint8 *)d) == 0 && (count % ATAPI_SECTOR_SIZE) != 0) {
      d = (iso9660_dir_record *) (((uint8 *)d) + 1);
      count++;
    }
  }

  if (count >= num_bytes)
    goto error;

  *de = *d;

  unmap_virtual_page (page);
  free_phys_frame (frame);
  return 0;
 error:
  unmap_virtual_page (page);
  free_phys_frame (frame);
  return -1;
}


int
iso9660_open (iso9660_mounted_info * mi, char *pathname, iso9660_handle * h)
{
  int len = 0, start = 0, end;
  iso9660_dir_record d, de;
  d.first_sector = mi->root_dir_sector;
  d.data_length = mi->root_dir_data_length;

  if (pathname[0] != PATHSEP)
    return -1;
  while (pathname[len])
    len++;

  for (;;) {
    end = len;
    if (iso9660_parse_first_component (pathname, &start, &end) == 0) {
      if (start == end)
        return -1;

      if ((iso9660_search_dir (mi, &d, pathname, start, end, &de)) < 0) {
        return -1;
      }

      h->mount = mi;
      h->sector = de.first_sector;
      h->offset = 0;
      h->length = de.data_length;
      return 0;
    } else {
      /* This component is a directory name. */
      if ((iso9660_search_dir (mi, &d, pathname, start, end, &de)) < 0) {
        return -1;
      } else {
        d = de;
      }
      start = end;              /* move forward to next component */
    }
  }
}

int
iso9660_read (iso9660_handle * h, uint8 * buf, uint32 len)
{
  int n, count = 0, rem = len, curlen;
  iso9660_mounted_info *mi = h->mount;
  uint32 frame;
  uint8 *page;

  frame = alloc_phys_frame ();
  page = map_virtual_page (frame | 3);

  while (count < len) {
    n = atapi_drive_read_sector (mi->bus, mi->drive, h->sector, page);
    if (n < 0)
      goto error;

    curlen = rem > (n - h->offset) ? (n - h->offset) : rem;

    memcpy (buf + count, page + h->offset, curlen);

    rem -= curlen;
    count += curlen;
    h->offset += curlen;
    h->length -= curlen;
    if (h->offset == 2048) {
      h->sector++;
      h->offset = 0;
    }
  }


  unmap_virtual_page (page);
  free_phys_frame (frame);
  return count;
error:
  unmap_virtual_page (page);
  free_phys_frame (frame);
  return -1;
}

static iso9660_mounted_info eziso_mount_info;

#if 0
uint8 test1_buf[2958];
#endif

int
eziso_mount (uint32 bus, uint32 drive)
{
  int v;

  v = iso9660_mount (bus, drive, &eziso_mount_info);

#if 0
  if (iso9660_open (&eziso_mount_info, "/boot/test1", &h) == 0) {
    int i, j;
    iso9660_handle h;
    com1_printf ("OPEN SUCCEEDED\n");

    if (iso9660_read (&h, test1_buf, 2958) < 0) {
      com1_printf ("READ FAILED\n");
    } else {
      for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++)
          com1_printf (" %.2X", (test1_buf + (2958 - 64))[i * 8 + j]);
        com1_printf ("\n");
      }
    }

  } else
    com1_printf ("OPEN FAILED\n");
#endif

  if (v == 0)
    return 1;
  else
    return 0;
}

static iso9660_handle eziso_handle;
int
eziso_dir (char *pathname)
{
  if (iso9660_open (&eziso_mount_info, pathname, &eziso_handle) == 0)
    return eziso_handle.length;
  else
    return -1;
}

int
eziso_read (char *buf, int len)
{
  int n;
  if ((n = iso9660_read (&eziso_handle, (uint8 *) buf, len)) < 0)
    return 0;
  return n;
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
