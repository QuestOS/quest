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

/*
 * Based on:
 *
 *  GRUB  --  GRand Unified Bootloader
 *  Copyright (C) 2000,2001,2005   Free Software Foundation, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "fs/filesys.h"
#include "drivers/usb/umsc.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "types.h"

#define UMSC_DEVICE_INDEX 0
static int
devread_vfat (int sector, int byte_offset, int byte_len, char *buf)
{
  uint8 s[512];
  int len = byte_len;
  com1_printf ("fsys_vfat: devread_vfat (%d, %d, %d, %p)\n",
               sector, byte_offset, byte_len, buf);
  while (len > 0) {
    if (umsc_read_sector (UMSC_DEVICE_INDEX, sector, s, sizeof (s)) != sizeof (s))
      return 0;
    memcpy (buf, &s[byte_offset], len > sizeof (s) ? sizeof (s) : len);
    len -= sizeof (s);
    buf += sizeof (s);
    sector++;
  }
  return byte_len;
}

static int
tolower (int c)
{
  if (c >= 'A' && c <= 'Z')
    return (c + ('a' - 'A'));

  return c;
}

static int
isspace (int c)
{
  switch (c)
    {
    case ' ':
    case '\t':
    case '\r':
    case '\n':
      return 1;
    default:
      break;
    }

  return 0;
}

static int
substring (const char *s1, const char *s2)
{
  while (*s1 == *s2)
    {
      /* The strings match exactly. */
      if (! *(s1++))
	return 0;
      s2 ++;
    }

  /* S1 is a substring of S2. */
  if (*s1 == 0)
    return -1;

  /* S1 isn't a substring. */
  return 1;
}

#ifndef NULL
#define NULL ((void *)0)
#endif

#define MAXINT     0x7FFFFFFF

/********************
 * #include "fat.h" *
 ********************/

/*
 *  Defines for the FAT BIOS Parameter Block (embedded in the first block
 *  of the partition.
 */

typedef __signed__ char __s8;
typedef unsigned char __u8;
typedef __signed__ short __s16;
typedef unsigned short __u16;
typedef __signed__ int __s32;
typedef unsigned int __u32;

/* Note that some shorts are not aligned, and must therefore
 * be declared as array of two bytes.
 */
struct fat_bpb {
        __s8    ignored[3];     /* Boot strap short or near jump */
        __s8    system_id[8];   /* Name - can be used to special case
                                   partition manager volumes */
        __u8    bytes_per_sect[2];      /* bytes per logical sector */
        __u8    sects_per_clust;/* sectors/cluster */
        __u8    reserved_sects[2];      /* reserved sectors */
        __u8    num_fats;       /* number of FATs */
        __u8    dir_entries[2]; /* root directory entries */
        __u8    short_sectors[2];       /* number of sectors */
        __u8    media;          /* media code (unused) */
        __u16   fat_length;     /* sectors/FAT */
        __u16   secs_track;     /* sectors per track */
        __u16   heads;          /* number of heads */
        __u32   hidden;         /* hidden sectors (unused) */
        __u32   long_sectors;   /* number of sectors (if short_sectors == 0) */

        /* The following fields are only used by FAT32 */
        __u32   fat32_length;   /* sectors/FAT */
        __u16   flags;          /* bit 8: fat mirroring, low 4: active fat */
        __u8    version[2];     /* major, minor filesystem version */
        __u32   root_cluster;   /* first cluster in root directory */
        __u16   info_sector;    /* filesystem info sector */
        __u16   backup_boot;    /* backup boot sector */
        __u16   reserved2[6];   /* Unused */
};

#define FAT_CVT_U16(bytarr) (* (__u16*)(bytarr))

/*
 *  Defines how to differentiate a 12-bit and 16-bit FAT.
 */

#define FAT_MAX_12BIT_CLUST       4087  /* 4085 + 2 */

/*
 *  Defines for the file "attribute" byte
 */

#define FAT_ATTRIB_OK_MASK        0x37
#define FAT_ATTRIB_NOT_OK_MASK    0xC8
#define FAT_ATTRIB_DIR            0x10
#define FAT_ATTRIB_LONGNAME       0x0F

/*
 *  Defines for FAT directory entries
 */

#define FAT_DIRENTRY_LENGTH       32

#define FAT_DIRENTRY_ATTRIB(entry) \
  (*((unsigned char *) (entry+11)))
#define FAT_DIRENTRY_VALID(entry) \
  ( ((*((unsigned char *) entry)) != 0) \
    && ((*((unsigned char *) entry)) != 0xE5) \
    && !(FAT_DIRENTRY_ATTRIB(entry) & FAT_ATTRIB_NOT_OK_MASK) )
#define FAT_DIRENTRY_FIRST_CLUSTER(entry) \
  ((*((unsigned short *) (entry+26)))+(*((unsigned short *) (entry+20)) << 16))
#define FAT_DIRENTRY_FILELENGTH(entry) \
  (*((unsigned long *) (entry+28)))

#define FAT_LONGDIR_ID(entry) \
  (*((unsigned char *) (entry)))
#define FAT_LONGDIR_ALIASCHECKSUM(entry) \
  (*((unsigned char *) (entry+13)))

struct fat_superblock
{
  int fat_offset;
  int fat_length;
  int fat_size;
  int root_offset;
  int root_max;
  int data_offset;

  int num_sectors;
  int num_clust;
  int clust_eof_marker;
  int sects_per_clust;
  int sectsize_bits;
  int clustsize_bits;
  int root_cluster;

  int cached_fat;
  int file_cluster;
  int current_cluster_num;
  int current_cluster;
};

static int errnum;
static char fsys_buf[0x8000];
static int filepos;
static int filemax;


/* pointer(s) into filesystem info buffer for DOS stuff */
#define FAT_SUPER ( (struct fat_superblock *) \
                    ( fsys_buf + 32256) )/* 512 bytes long */
#define FAT_BUF   ( fsys_buf + 30208 )  /* 4 sector FAT buffer */
#define NAME_BUF  ( fsys_buf + 29184 )  /* Filename buffer (833 bytes) */

#define FAT_CACHE_SIZE 2048

static __inline__ unsigned long
log2 (unsigned long word)
{
  __asm__ ("bsfl %1,%0"
           : "=r" (word)
           : "r" (word));
  return word;
}

int
vfat_mount (void)
{
  struct fat_bpb bpb;
  __u32 magic, first_fat;

  /* Read bpb */
  if (! devread_vfat (0, 0, sizeof (bpb), (char *) &bpb))
    return 0;

  /* Check if the number of sectors per cluster is zero here, to avoid
     zero division.  */
  if (bpb.sects_per_clust == 0)
    return 0;

  FAT_SUPER->sectsize_bits = log2 (FAT_CVT_U16 (bpb.bytes_per_sect));
  FAT_SUPER->clustsize_bits
    = FAT_SUPER->sectsize_bits + log2 (bpb.sects_per_clust);

  /* Fill in info about super block */
  FAT_SUPER->num_sectors = FAT_CVT_U16 (bpb.short_sectors)
    ? FAT_CVT_U16 (bpb.short_sectors) : bpb.long_sectors;

  /* FAT offset and length */
  FAT_SUPER->fat_offset = FAT_CVT_U16 (bpb.reserved_sects);
  FAT_SUPER->fat_length =
    bpb.fat_length ? bpb.fat_length : bpb.fat32_length;

  /* Rootdir offset and length for FAT12/16 */
  FAT_SUPER->root_offset =
    FAT_SUPER->fat_offset + bpb.num_fats * FAT_SUPER->fat_length;
  FAT_SUPER->root_max = FAT_DIRENTRY_LENGTH * FAT_CVT_U16(bpb.dir_entries);

  /* Data offset and number of clusters */
  FAT_SUPER->data_offset =
    FAT_SUPER->root_offset
    + ((FAT_SUPER->root_max - 1) >> FAT_SUPER->sectsize_bits) + 1;
  FAT_SUPER->num_clust =
    2 + ((FAT_SUPER->num_sectors - FAT_SUPER->data_offset)
         / bpb.sects_per_clust);
  FAT_SUPER->sects_per_clust = bpb.sects_per_clust;

  if (!bpb.fat_length)
    {
      /* This is a FAT32 */
      if (FAT_CVT_U16(bpb.dir_entries))
        return 0;

      if (bpb.flags & 0x0080)
        {
          /* FAT mirroring is disabled, get active FAT */
          int active_fat = bpb.flags & 0x000f;
          if (active_fat >= bpb.num_fats)
            return 0;
          FAT_SUPER->fat_offset += active_fat * FAT_SUPER->fat_length;
        }

      FAT_SUPER->fat_size = 8;
      FAT_SUPER->root_cluster = bpb.root_cluster;

      /* Yes the following is correct.  FAT32 should be called FAT28 :) */
      FAT_SUPER->clust_eof_marker = 0xffffff8;
    }
  else
    {
      if (!FAT_SUPER->root_max)
        return 0;

      FAT_SUPER->root_cluster = -1;
      if (FAT_SUPER->num_clust > FAT_MAX_12BIT_CLUST)
        {
          FAT_SUPER->fat_size = 4;
          FAT_SUPER->clust_eof_marker = 0xfff8;
        }
      else
        {
          FAT_SUPER->fat_size = 3;
          FAT_SUPER->clust_eof_marker = 0xff8;
        }
    }

  /* Now do some sanity checks */

  if (FAT_CVT_U16(bpb.bytes_per_sect) != (1 << FAT_SUPER->sectsize_bits)
      || FAT_CVT_U16(bpb.bytes_per_sect) != SECTOR_SIZE
      || bpb.sects_per_clust != (1 << (FAT_SUPER->clustsize_bits
                                       - FAT_SUPER->sectsize_bits))
      || FAT_SUPER->num_clust <= 2
      || (FAT_SUPER->fat_size * FAT_SUPER->num_clust / (2 * SECTOR_SIZE)
          > FAT_SUPER->fat_length))
    return 0;

  /* kbs: Media check on first FAT entry [ported from PUPA] */

  if (!devread_vfat(FAT_SUPER->fat_offset, 0,
               sizeof(first_fat), (char *)&first_fat))
    return 0;

  if (FAT_SUPER->fat_size == 8)
    {
      first_fat &= 0x0fffffff;
      magic = 0x0fffff00;
    }
  else if (FAT_SUPER->fat_size == 4)
    {
      first_fat &= 0x0000ffff;
      magic = 0xff00;
    }
  else
    {
      first_fat &= 0x00000fff;
      magic = 0x0f00;
    }

  /* Ignore the 3rd bit, because some BIOSes assigns 0xF0 to the media
     descriptor, even if it is a so-called superfloppy (e.g. an USB key).
     The check may be too strict for this kind of stupid BIOSes, as
     they overwrite the media descriptor.  */
  if ((first_fat | 0x8) != (magic | bpb.media | 0x8))
    return 0;

  FAT_SUPER->cached_fat = - 2 * FAT_CACHE_SIZE;
  return 1;
}

int
vfat_read (char *buf, int len)
{
  int logical_clust;
  int offset;
  int ret = 0;
  int size;

  errnum=0;

  if (FAT_SUPER->file_cluster < 0)
    {
      /* root directory for fat16 */
      size = FAT_SUPER->root_max - filepos;
      if (size > len)
        size = len;
      if (!devread_vfat(FAT_SUPER->root_offset, filepos, size, buf))
        return 0;
      filepos += size;
      return size;
    }

  logical_clust = filepos >> FAT_SUPER->clustsize_bits;
  offset = (filepos & ((1 << FAT_SUPER->clustsize_bits) - 1));
  if (logical_clust < FAT_SUPER->current_cluster_num)
    {
      FAT_SUPER->current_cluster_num = 0;
      FAT_SUPER->current_cluster = FAT_SUPER->file_cluster;
    }

  while (len > 0)
    {
      int sector;
      while (logical_clust > FAT_SUPER->current_cluster_num)
        {
          /* calculate next cluster */
          int fat_entry =
            FAT_SUPER->current_cluster * FAT_SUPER->fat_size;
          int next_cluster;
          int cached_pos = (fat_entry - FAT_SUPER->cached_fat);

          if (cached_pos < 0 ||
              (cached_pos + FAT_SUPER->fat_size) > 2*FAT_CACHE_SIZE)
            {
              FAT_SUPER->cached_fat = (fat_entry & ~(2*SECTOR_SIZE - 1));
              cached_pos = (fat_entry - FAT_SUPER->cached_fat);
              sector = FAT_SUPER->fat_offset
                + FAT_SUPER->cached_fat / (2*SECTOR_SIZE);
              if (!devread_vfat (sector, 0, FAT_CACHE_SIZE, (char*) FAT_BUF))
                return 0;
            }
          next_cluster = * (unsigned long *) (FAT_BUF + (cached_pos >> 1));
          if (FAT_SUPER->fat_size == 3)
            {
              if (cached_pos & 1)
                next_cluster >>= 4;
              next_cluster &= 0xFFF;
            }
          else if (FAT_SUPER->fat_size == 4)
            next_cluster &= 0xFFFF;

          if (next_cluster >= FAT_SUPER->clust_eof_marker)
            return ret;
          if (next_cluster < 2 || next_cluster >= FAT_SUPER->num_clust)
            {
              errnum = ERR_FSYS_CORRUPT;
              return 0;
            }

          FAT_SUPER->current_cluster = next_cluster;
          FAT_SUPER->current_cluster_num++;
        }

      sector = FAT_SUPER->data_offset +
        ((FAT_SUPER->current_cluster - 2) << (FAT_SUPER->clustsize_bits
                                              - FAT_SUPER->sectsize_bits));
      size = (1 << FAT_SUPER->clustsize_bits) - offset;
      if (size > len)
        size = len;

      devread_vfat(sector, offset, size, buf);

      len -= size;
      buf += size;
      ret += size;
      filepos += size;
      logical_clust++;
      offset = 0;
    }
  return errnum ? 0 : ret;
}

int
vfat_dir (char *dirname)
{
  char *rest, ch, dir_buf[FAT_DIRENTRY_LENGTH];
  char *filename = (char *) NAME_BUF;
  int attrib = FAT_ATTRIB_DIR;

  /* XXX I18N:
   * the positions 2,4,6 etc are high bytes of a 16 bit unicode char
   */
  static unsigned char longdir_pos[] =
  { 1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30 };
  int slot = -2;
  int alias_checksum = -1;

  FAT_SUPER->file_cluster = FAT_SUPER->root_cluster;
  filepos = 0;
  FAT_SUPER->current_cluster_num = MAXINT;

  /* main loop to find desired directory entry */
 loop:

  /* if we have a real file (and we're not just printing possibilities),
     then this is where we want to exit */

  if (!*dirname || isspace (*dirname))
    {
      if (attrib & FAT_ATTRIB_DIR)
        {
          errnum = ERR_BAD_FILETYPE;
          return 0;
        }

      return filemax;
    }

  /* continue with the file/directory name interpretation */

  while (*dirname == '/')
    dirname++;

  if (!(attrib & FAT_ATTRIB_DIR))
    {
      errnum = ERR_BAD_FILETYPE;
      return 0;
    }
  /* Directories don't have a file size */
  filemax = MAXINT;

  for (rest = dirname; (ch = *rest) && !isspace (ch) && ch != '/'; rest++);

  *rest = 0;

  while (1)
    {
      if (vfat_read (dir_buf, FAT_DIRENTRY_LENGTH) != FAT_DIRENTRY_LENGTH
          || dir_buf[0] == 0)
        {
          if (!errnum)
            {
              errnum = ERR_FILE_NOT_FOUND;
              *rest = ch;
            }

          return 0;
        }

      if (FAT_DIRENTRY_ATTRIB (dir_buf) == FAT_ATTRIB_LONGNAME)
        {
          /* This is a long filename.  The filename is build from back
           * to front and may span multiple entries.  To bind these
           * entries together they all contain the same checksum over
           * the short alias.
           *
           * The id field tells if this is the first entry (the last
           * part) of the long filename, and also at which offset this
           * belongs.
           *
           * We just write the part of the long filename this entry
           * describes and continue with the next dir entry.
           */
          int i, offset;
          unsigned char id = FAT_LONGDIR_ID(dir_buf);

          if ((id & 0x40))
            {
              id &= 0x3f;
              slot = id;
              filename[slot * 13] = 0;
              alias_checksum = FAT_LONGDIR_ALIASCHECKSUM(dir_buf);
            }

          if (id != slot || slot == 0
              || alias_checksum != FAT_LONGDIR_ALIASCHECKSUM(dir_buf))
            {
              alias_checksum = -1;
              continue;
            }

          slot--;
          offset = slot * 13;

          for (i=0; i < 13; i++)
            filename[offset+i] = dir_buf[longdir_pos[i]];
          continue;
        }

      if (!FAT_DIRENTRY_VALID (dir_buf))
        continue;

      if (alias_checksum != -1 && slot == 0)
        {
          int i;
          unsigned char sum;

          slot = -2;
          for (sum = 0, i = 0; i< 11; i++)
            sum = ((sum >> 1) | (sum << 7)) + dir_buf[i];

          if (sum == alias_checksum)
            {
              if (substring (dirname, filename) == 0)
                break;
            }
        }

      /* XXX convert to 8.3 filename format here */
      {
        int i, j, c;

        for (i = 0; i < 8 && (c = filename[i] = tolower (dir_buf[i]))
               && !isspace (c); i++);

        filename[i++] = '.';

        for (j = 0; j < 3 && (c = filename[i + j] = tolower (dir_buf[8 + j]))
               && !isspace (c); j++);

        if (j == 0)
          i--;

        filename[i + j] = 0;
      }

      if (substring (dirname, filename) == 0)
        break;
    }

  *(dirname = rest) = ch;

  attrib = FAT_DIRENTRY_ATTRIB (dir_buf);
  filemax = FAT_DIRENTRY_FILELENGTH (dir_buf);
  filepos = 0;
  FAT_SUPER->file_cluster = FAT_DIRENTRY_FIRST_CLUSTER (dir_buf);
  FAT_SUPER->current_cluster_num = MAXINT;

  /* go back to main loop at top of function */
  goto loop;
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
