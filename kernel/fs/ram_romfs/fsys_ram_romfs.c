/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
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

#include "fs/ram_romfs.h"
#include "mem/mem.h"
#include "kernel.h"
#include "util/printf.h"

#ifdef USE_VMX
#include "vm/shm.h"
#include "vm/spow2.h"
#endif

#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

uint32 ramdisk_phys = 0xFFFFFFFF;
romfs_file_header_t* cur_file_header = NULL;
size_t cur_file_phys_offset = 0;
romfs_t* romfs_header;



bool copy_ramdisk_module(multiboot_module *pmm, int mod_num)
{
  uint pages_needed = DIV_ROUND_UP( ((char*)pmm->mod_end) - ((char*)pmm->pe), 0x1000);
  char* ramdisk_module_start = (char*)pmm->pe;
  int i;
  ramdisk_phys = alloc_phys_frames(pages_needed);
  
  if(ramdisk_phys == 0xFFFFFFFF) {
    com1_printf("Failed to allocate physical memory for ramdisk copy\n");
    return FALSE;
  }
  
  /* Map and copy one page at a time as the ramdisk might be very
     large */
  for(i = 0; i < pages_needed; ++i) { 
    void* virt_des = map_virtual_page((ramdisk_phys + 0x1000 * i) | 0x3);
    void* virt_src = map_virtual_page((((uint32)ramdisk_module_start) + 0x1000 * i) | 0x3);
    if(!virt_src || !virt_des) {
      com1_printf("Failed to map virtual page for ramdisk copy\n");
      return FALSE;
    }
    memcpy(virt_des, virt_src, 0x1000);
    unmap_virtual_page(virt_des);
    unmap_virtual_page(virt_src);
  }
  return TRUE;
}

bool ramdisk_mount()
{
  if(ramdisk_phys == 0xFFFFFFFF) {
    return FALSE;
  }
   
#ifdef USE_VMX
  ramdisk_phys += SANDBOX_KERN_OFFSET * get_pcpu_id();
#endif
  
  romfs_header = map_virtual_page(ramdisk_phys | 0x3);
  if(!romfs_header) return FALSE;
  
  if(strncmp(romfs_header->name, "-rom1fs-", 8)) {
    return FALSE;
  }
  return TRUE;
}

static romfs_file_header_t* get_file_header(size_t offset)
{
  static romfs_file_header_t* temp_file_header = NULL;
  
  if(temp_file_header) {
    unmap_virtual_pages(temp_file_header, 2);
  }
  /* Just to make sure file offsets should always be 16 byte
     aligned */
  offset &= ~0xF;
  cur_file_phys_offset = offset;
  /* We map two virtual pages in case the file header cross a page
     boundary */
  
  temp_file_header = map_contiguous_virtual_pages(((ramdisk_phys + offset) & 0xFFFFF000) | 0x3, 2);
  return (romfs_file_header_t*)(((char*)temp_file_header) + (offset % 0x1000));
}

int ramdisk_dir (char *pathname)
{
  romfs_file_header_t* file_header;

  cur_file_header = NULL;

  file_header = get_file_header(FIRST_FILE_HEADER_OFFSET);
  if(!file_header) return -1;

  while(1) {
    char cur_filename[17];
    int cur_filename_size = 0;
    bool directory;
    
    if(*pathname == '/') pathname++;
    
    while(*pathname != '/' && *pathname != 0) {
      if(cur_filename_size >= 16) return -1;
      cur_filename[cur_filename_size++] = *pathname++;
    }
    cur_filename[cur_filename_size] = 0;

    if(cur_filename_size == 0) return -1;

    directory = *pathname == '/';
    
    while(1) {
      uint file_header_name_len = 0;
      while(file_header_name_len < 16) {
        if(!file_header->filename[file_header_name_len]) break;
        file_header_name_len++;
      }
      if( ((directory && (romfs_get_file_type(file_header) == ROMFS_DIRECTORY)) ||
           (!directory && (romfs_get_file_type(file_header) == ROMFS_REGULAR_FILE))) &&
          (!strncmp(file_header->filename, cur_filename, cur_filename_size) &&
           cur_filename_size == file_header_name_len) ) {
        /* Found the match */
        if(directory) {
          file_header = get_file_header(be32toh(file_header->spec_info) & ~0xF);
          if(!file_header) return -1;
          break;
        }
        else {
          cur_file_header = file_header;
          return be32toh(cur_file_header->size);
        }
      }
      else {
        if((be32toh(file_header->next) & ~0xF) == 0) {
          return -1;
        }
        else {
          file_header = get_file_header(be32toh(file_header->next) & ~0xF);
          if(!file_header) return -1;
        }
      }
    }
  }
}

int ramdisk_read (char *buf, int len)
{
  size_t phys_addr_data;
  size_t page_offset;
  int bytes_copied = 0;
  char* temp;

  if(!cur_file_header) return -1;

  if(len > be32toh(cur_file_header->size)) len = be32toh(cur_file_header->size);

  phys_addr_data = ramdisk_phys + cur_file_phys_offset + sizeof(romfs_file_header_t);

  while(bytes_copied < len) {
    size_t bytes_to_copy;
    page_offset = (phys_addr_data % 0x1000);

    temp = map_virtual_page((phys_addr_data & 0xFFFFF000) | 0x3);
    if(!temp) return -1;

    bytes_to_copy = 0x1000 - page_offset;
    if(bytes_to_copy > (len - bytes_copied)) {
      bytes_to_copy = len - bytes_copied;
    }
    memcpy(&buf[bytes_copied], &temp[page_offset], bytes_to_copy);
    unmap_virtual_page(temp);
    bytes_copied += bytes_to_copy;
    phys_addr_data += bytes_to_copy;
  }

  return bytes_copied;
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
