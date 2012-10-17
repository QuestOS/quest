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

/* Adopted from Linux */

/*
 * Oct 15, 2000 Matt Domsch <Matt_Domsch@dell.com>
 * Nicer crc32 functions/docs submitted by linux@horizon.com.  Thanks!
 * Code was from the public domain, copyright abandoned.  Code was
 * subsequently included in the kernel, thus was re-licensed under the
 * GNU GPL v2.
 *
 * Oct 12, 2000 Matt Domsch <Matt_Domsch@dell.com>
 * Same crc32 function was used in 5 other places in the kernel.
 * I made one version, and deleted the others.
 * There are various incantations of crc32().  Some use a seed of 0 or ~0.
 * Some xor at the end with ~0.  The generic crc32() function takes
 * seed as an argument, and doesn't xor at the end.  Then individual
 * users can do whatever they need.
 *   drivers/net/smc9194.c uses seed ~0, doesn't xor with ~0.
 *   fs/jffs2 uses seed 0, doesn't xor with ~0.
 *   fs/partitions/efi.c uses seed ~0, xor's with ~0.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */

#include <kernel.h>
#include <util/crc32.h>
#include <arch/i386.h>

/*
 * There are multiple 16-bit CRC polynomials in common use, but this is
 * *the* standard CRC-32 polynomial, first popularized by Ethernet.
 * x^32+x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x^1+x^0
 */
#define CRCPOLY_LE 0xedb88320
#define CRCPOLY_BE 0x04c11db7

/* How many bits at a time to use.  Requires a table of 4<<CRC_xx_BITS bytes. */
/* For less performance-sensitive, use 4 */
#ifndef CRC_LE_BITS
# define CRC_LE_BITS 8
#endif
#ifndef CRC_BE_BITS
# define CRC_BE_BITS 8
#endif

/*
 * Little-endian CRC computation.  Used with serial bit streams sent
 * lsbit-first.  Be sure to use cpu_to_le32() to append the computed CRC.
 */
#if CRC_LE_BITS > 8 || CRC_LE_BITS < 1 || CRC_LE_BITS & CRC_LE_BITS-1
# error CRC_LE_BITS must be a power of 2 between 1 and 8
#endif

/*
 * Big-endian CRC computation.  Used with serial bit streams sent
 * msbit-first.  Be sure to use cpu_to_be32() to append the computed CRC.
 */
#if CRC_BE_BITS > 8 || CRC_BE_BITS < 1 || CRC_BE_BITS & CRC_BE_BITS-1
# error CRC_BE_BITS must be a power of 2 between 1 and 8
#endif


#if CRC_LE_BITS == 8
#define tole(x) __constant_cpu_to_le32(x)
#define tobe(x) __constant_cpu_to_be32(x)
#else
#define tole(x) (x)
#define tobe(x) (x)
#endif

/* generated */
static const u32 crc32table_le[] = {
  tole(0x00000000L), tole(0x77073096L), tole(0xee0e612cL), tole(0x990951baL),
  tole(0x076dc419L), tole(0x706af48fL), tole(0xe963a535L), tole(0x9e6495a3L),
  tole(0x0edb8832L), tole(0x79dcb8a4L), tole(0xe0d5e91eL), tole(0x97d2d988L),
  tole(0x09b64c2bL), tole(0x7eb17cbdL), tole(0xe7b82d07L), tole(0x90bf1d91L),
  tole(0x1db71064L), tole(0x6ab020f2L), tole(0xf3b97148L), tole(0x84be41deL),
  tole(0x1adad47dL), tole(0x6ddde4ebL), tole(0xf4d4b551L), tole(0x83d385c7L),
  tole(0x136c9856L), tole(0x646ba8c0L), tole(0xfd62f97aL), tole(0x8a65c9ecL),
  tole(0x14015c4fL), tole(0x63066cd9L), tole(0xfa0f3d63L), tole(0x8d080df5L),
  tole(0x3b6e20c8L), tole(0x4c69105eL), tole(0xd56041e4L), tole(0xa2677172L),
  tole(0x3c03e4d1L), tole(0x4b04d447L), tole(0xd20d85fdL), tole(0xa50ab56bL),
  tole(0x35b5a8faL), tole(0x42b2986cL), tole(0xdbbbc9d6L), tole(0xacbcf940L),
  tole(0x32d86ce3L), tole(0x45df5c75L), tole(0xdcd60dcfL), tole(0xabd13d59L),
  tole(0x26d930acL), tole(0x51de003aL), tole(0xc8d75180L), tole(0xbfd06116L),
  tole(0x21b4f4b5L), tole(0x56b3c423L), tole(0xcfba9599L), tole(0xb8bda50fL),
  tole(0x2802b89eL), tole(0x5f058808L), tole(0xc60cd9b2L), tole(0xb10be924L),
  tole(0x2f6f7c87L), tole(0x58684c11L), tole(0xc1611dabL), tole(0xb6662d3dL),
  tole(0x76dc4190L), tole(0x01db7106L), tole(0x98d220bcL), tole(0xefd5102aL),
  tole(0x71b18589L), tole(0x06b6b51fL), tole(0x9fbfe4a5L), tole(0xe8b8d433L),
  tole(0x7807c9a2L), tole(0x0f00f934L), tole(0x9609a88eL), tole(0xe10e9818L),
  tole(0x7f6a0dbbL), tole(0x086d3d2dL), tole(0x91646c97L), tole(0xe6635c01L),
  tole(0x6b6b51f4L), tole(0x1c6c6162L), tole(0x856530d8L), tole(0xf262004eL),
  tole(0x6c0695edL), tole(0x1b01a57bL), tole(0x8208f4c1L), tole(0xf50fc457L),
  tole(0x65b0d9c6L), tole(0x12b7e950L), tole(0x8bbeb8eaL), tole(0xfcb9887cL),
  tole(0x62dd1ddfL), tole(0x15da2d49L), tole(0x8cd37cf3L), tole(0xfbd44c65L),
  tole(0x4db26158L), tole(0x3ab551ceL), tole(0xa3bc0074L), tole(0xd4bb30e2L),
  tole(0x4adfa541L), tole(0x3dd895d7L), tole(0xa4d1c46dL), tole(0xd3d6f4fbL),
  tole(0x4369e96aL), tole(0x346ed9fcL), tole(0xad678846L), tole(0xda60b8d0L),
  tole(0x44042d73L), tole(0x33031de5L), tole(0xaa0a4c5fL), tole(0xdd0d7cc9L),
  tole(0x5005713cL), tole(0x270241aaL), tole(0xbe0b1010L), tole(0xc90c2086L),
  tole(0x5768b525L), tole(0x206f85b3L), tole(0xb966d409L), tole(0xce61e49fL),
  tole(0x5edef90eL), tole(0x29d9c998L), tole(0xb0d09822L), tole(0xc7d7a8b4L),
  tole(0x59b33d17L), tole(0x2eb40d81L), tole(0xb7bd5c3bL), tole(0xc0ba6cadL),
  tole(0xedb88320L), tole(0x9abfb3b6L), tole(0x03b6e20cL), tole(0x74b1d29aL),
  tole(0xead54739L), tole(0x9dd277afL), tole(0x04db2615L), tole(0x73dc1683L),
  tole(0xe3630b12L), tole(0x94643b84L), tole(0x0d6d6a3eL), tole(0x7a6a5aa8L),
  tole(0xe40ecf0bL), tole(0x9309ff9dL), tole(0x0a00ae27L), tole(0x7d079eb1L),
  tole(0xf00f9344L), tole(0x8708a3d2L), tole(0x1e01f268L), tole(0x6906c2feL),
  tole(0xf762575dL), tole(0x806567cbL), tole(0x196c3671L), tole(0x6e6b06e7L),
  tole(0xfed41b76L), tole(0x89d32be0L), tole(0x10da7a5aL), tole(0x67dd4accL),
  tole(0xf9b9df6fL), tole(0x8ebeeff9L), tole(0x17b7be43L), tole(0x60b08ed5L),
  tole(0xd6d6a3e8L), tole(0xa1d1937eL), tole(0x38d8c2c4L), tole(0x4fdff252L),
  tole(0xd1bb67f1L), tole(0xa6bc5767L), tole(0x3fb506ddL), tole(0x48b2364bL),
  tole(0xd80d2bdaL), tole(0xaf0a1b4cL), tole(0x36034af6L), tole(0x41047a60L),
  tole(0xdf60efc3L), tole(0xa867df55L), tole(0x316e8eefL), tole(0x4669be79L),
  tole(0xcb61b38cL), tole(0xbc66831aL), tole(0x256fd2a0L), tole(0x5268e236L),
  tole(0xcc0c7795L), tole(0xbb0b4703L), tole(0x220216b9L), tole(0x5505262fL),
  tole(0xc5ba3bbeL), tole(0xb2bd0b28L), tole(0x2bb45a92L), tole(0x5cb36a04L),
  tole(0xc2d7ffa7L), tole(0xb5d0cf31L), tole(0x2cd99e8bL), tole(0x5bdeae1dL),
  tole(0x9b64c2b0L), tole(0xec63f226L), tole(0x756aa39cL), tole(0x026d930aL),
  tole(0x9c0906a9L), tole(0xeb0e363fL), tole(0x72076785L), tole(0x05005713L),
  tole(0x95bf4a82L), tole(0xe2b87a14L), tole(0x7bb12baeL), tole(0x0cb61b38L),
  tole(0x92d28e9bL), tole(0xe5d5be0dL), tole(0x7cdcefb7L), tole(0x0bdbdf21L),
  tole(0x86d3d2d4L), tole(0xf1d4e242L), tole(0x68ddb3f8L), tole(0x1fda836eL),
  tole(0x81be16cdL), tole(0xf6b9265bL), tole(0x6fb077e1L), tole(0x18b74777L),
  tole(0x88085ae6L), tole(0xff0f6a70L), tole(0x66063bcaL), tole(0x11010b5cL),
  tole(0x8f659effL), tole(0xf862ae69L), tole(0x616bffd3L), tole(0x166ccf45L),
  tole(0xa00ae278L), tole(0xd70dd2eeL), tole(0x4e048354L), tole(0x3903b3c2L),
  tole(0xa7672661L), tole(0xd06016f7L), tole(0x4969474dL), tole(0x3e6e77dbL),
  tole(0xaed16a4aL), tole(0xd9d65adcL), tole(0x40df0b66L), tole(0x37d83bf0L),
  tole(0xa9bcae53L), tole(0xdebb9ec5L), tole(0x47b2cf7fL), tole(0x30b5ffe9L),
  tole(0xbdbdf21cL), tole(0xcabac28aL), tole(0x53b39330L), tole(0x24b4a3a6L),
  tole(0xbad03605L), tole(0xcdd70693L), tole(0x54de5729L), tole(0x23d967bfL),
  tole(0xb3667a2eL), tole(0xc4614ab8L), tole(0x5d681b02L), tole(0x2a6f2b94L),
  tole(0xb40bbe37L), tole(0xc30c8ea1L), tole(0x5a05df1bL), tole(0x2d02ef8dL)
};

static const u32 crc32table_be[] = {
  tobe(0x00000000L), tobe(0x04c11db7L), tobe(0x09823b6eL), tobe(0x0d4326d9L),
  tobe(0x130476dcL), tobe(0x17c56b6bL), tobe(0x1a864db2L), tobe(0x1e475005L),
  tobe(0x2608edb8L), tobe(0x22c9f00fL), tobe(0x2f8ad6d6L), tobe(0x2b4bcb61L),
  tobe(0x350c9b64L), tobe(0x31cd86d3L), tobe(0x3c8ea00aL), tobe(0x384fbdbdL),
  tobe(0x4c11db70L), tobe(0x48d0c6c7L), tobe(0x4593e01eL), tobe(0x4152fda9L),
  tobe(0x5f15adacL), tobe(0x5bd4b01bL), tobe(0x569796c2L), tobe(0x52568b75L),
  tobe(0x6a1936c8L), tobe(0x6ed82b7fL), tobe(0x639b0da6L), tobe(0x675a1011L),
  tobe(0x791d4014L), tobe(0x7ddc5da3L), tobe(0x709f7b7aL), tobe(0x745e66cdL),
  tobe(0x9823b6e0L), tobe(0x9ce2ab57L), tobe(0x91a18d8eL), tobe(0x95609039L),
  tobe(0x8b27c03cL), tobe(0x8fe6dd8bL), tobe(0x82a5fb52L), tobe(0x8664e6e5L),
  tobe(0xbe2b5b58L), tobe(0xbaea46efL), tobe(0xb7a96036L), tobe(0xb3687d81L),
  tobe(0xad2f2d84L), tobe(0xa9ee3033L), tobe(0xa4ad16eaL), tobe(0xa06c0b5dL),
  tobe(0xd4326d90L), tobe(0xd0f37027L), tobe(0xddb056feL), tobe(0xd9714b49L),
  tobe(0xc7361b4cL), tobe(0xc3f706fbL), tobe(0xceb42022L), tobe(0xca753d95L),
  tobe(0xf23a8028L), tobe(0xf6fb9d9fL), tobe(0xfbb8bb46L), tobe(0xff79a6f1L),
  tobe(0xe13ef6f4L), tobe(0xe5ffeb43L), tobe(0xe8bccd9aL), tobe(0xec7dd02dL),
  tobe(0x34867077L), tobe(0x30476dc0L), tobe(0x3d044b19L), tobe(0x39c556aeL),
  tobe(0x278206abL), tobe(0x23431b1cL), tobe(0x2e003dc5L), tobe(0x2ac12072L),
  tobe(0x128e9dcfL), tobe(0x164f8078L), tobe(0x1b0ca6a1L), tobe(0x1fcdbb16L),
  tobe(0x018aeb13L), tobe(0x054bf6a4L), tobe(0x0808d07dL), tobe(0x0cc9cdcaL),
  tobe(0x7897ab07L), tobe(0x7c56b6b0L), tobe(0x71159069L), tobe(0x75d48ddeL),
  tobe(0x6b93dddbL), tobe(0x6f52c06cL), tobe(0x6211e6b5L), tobe(0x66d0fb02L),
  tobe(0x5e9f46bfL), tobe(0x5a5e5b08L), tobe(0x571d7dd1L), tobe(0x53dc6066L),
  tobe(0x4d9b3063L), tobe(0x495a2dd4L), tobe(0x44190b0dL), tobe(0x40d816baL),
  tobe(0xaca5c697L), tobe(0xa864db20L), tobe(0xa527fdf9L), tobe(0xa1e6e04eL),
  tobe(0xbfa1b04bL), tobe(0xbb60adfcL), tobe(0xb6238b25L), tobe(0xb2e29692L),
  tobe(0x8aad2b2fL), tobe(0x8e6c3698L), tobe(0x832f1041L), tobe(0x87ee0df6L),
  tobe(0x99a95df3L), tobe(0x9d684044L), tobe(0x902b669dL), tobe(0x94ea7b2aL),
  tobe(0xe0b41de7L), tobe(0xe4750050L), tobe(0xe9362689L), tobe(0xedf73b3eL),
  tobe(0xf3b06b3bL), tobe(0xf771768cL), tobe(0xfa325055L), tobe(0xfef34de2L),
  tobe(0xc6bcf05fL), tobe(0xc27dede8L), tobe(0xcf3ecb31L), tobe(0xcbffd686L),
  tobe(0xd5b88683L), tobe(0xd1799b34L), tobe(0xdc3abdedL), tobe(0xd8fba05aL),
  tobe(0x690ce0eeL), tobe(0x6dcdfd59L), tobe(0x608edb80L), tobe(0x644fc637L),
  tobe(0x7a089632L), tobe(0x7ec98b85L), tobe(0x738aad5cL), tobe(0x774bb0ebL),
  tobe(0x4f040d56L), tobe(0x4bc510e1L), tobe(0x46863638L), tobe(0x42472b8fL),
  tobe(0x5c007b8aL), tobe(0x58c1663dL), tobe(0x558240e4L), tobe(0x51435d53L),
  tobe(0x251d3b9eL), tobe(0x21dc2629L), tobe(0x2c9f00f0L), tobe(0x285e1d47L),
  tobe(0x36194d42L), tobe(0x32d850f5L), tobe(0x3f9b762cL), tobe(0x3b5a6b9bL),
  tobe(0x0315d626L), tobe(0x07d4cb91L), tobe(0x0a97ed48L), tobe(0x0e56f0ffL),
  tobe(0x1011a0faL), tobe(0x14d0bd4dL), tobe(0x19939b94L), tobe(0x1d528623L),
  tobe(0xf12f560eL), tobe(0xf5ee4bb9L), tobe(0xf8ad6d60L), tobe(0xfc6c70d7L),
  tobe(0xe22b20d2L), tobe(0xe6ea3d65L), tobe(0xeba91bbcL), tobe(0xef68060bL),
  tobe(0xd727bbb6L), tobe(0xd3e6a601L), tobe(0xdea580d8L), tobe(0xda649d6fL),
  tobe(0xc423cd6aL), tobe(0xc0e2d0ddL), tobe(0xcda1f604L), tobe(0xc960ebb3L),
  tobe(0xbd3e8d7eL), tobe(0xb9ff90c9L), tobe(0xb4bcb610L), tobe(0xb07daba7L),
  tobe(0xae3afba2L), tobe(0xaafbe615L), tobe(0xa7b8c0ccL), tobe(0xa379dd7bL),
  tobe(0x9b3660c6L), tobe(0x9ff77d71L), tobe(0x92b45ba8L), tobe(0x9675461fL),
  tobe(0x8832161aL), tobe(0x8cf30badL), tobe(0x81b02d74L), tobe(0x857130c3L),
  tobe(0x5d8a9099L), tobe(0x594b8d2eL), tobe(0x5408abf7L), tobe(0x50c9b640L),
  tobe(0x4e8ee645L), tobe(0x4a4ffbf2L), tobe(0x470cdd2bL), tobe(0x43cdc09cL),
  tobe(0x7b827d21L), tobe(0x7f436096L), tobe(0x7200464fL), tobe(0x76c15bf8L),
  tobe(0x68860bfdL), tobe(0x6c47164aL), tobe(0x61043093L), tobe(0x65c52d24L),
  tobe(0x119b4be9L), tobe(0x155a565eL), tobe(0x18197087L), tobe(0x1cd86d30L),
  tobe(0x029f3d35L), tobe(0x065e2082L), tobe(0x0b1d065bL), tobe(0x0fdc1becL),
  tobe(0x3793a651L), tobe(0x3352bbe6L), tobe(0x3e119d3fL), tobe(0x3ad08088L),
  tobe(0x2497d08dL), tobe(0x2056cd3aL), tobe(0x2d15ebe3L), tobe(0x29d4f654L),
  tobe(0xc5a92679L), tobe(0xc1683bceL), tobe(0xcc2b1d17L), tobe(0xc8ea00a0L),
  tobe(0xd6ad50a5L), tobe(0xd26c4d12L), tobe(0xdf2f6bcbL), tobe(0xdbee767cL),
  tobe(0xe3a1cbc1L), tobe(0xe760d676L), tobe(0xea23f0afL), tobe(0xeee2ed18L),
  tobe(0xf0a5bd1dL), tobe(0xf464a0aaL), tobe(0xf9278673L), tobe(0xfde69bc4L),
  tobe(0x89b8fd09L), tobe(0x8d79e0beL), tobe(0x803ac667L), tobe(0x84fbdbd0L),
  tobe(0x9abc8bd5L), tobe(0x9e7d9662L), tobe(0x933eb0bbL), tobe(0x97ffad0cL),
  tobe(0xafb010b1L), tobe(0xab710d06L), tobe(0xa6322bdfL), tobe(0xa2f33668L),
  tobe(0xbcb4666dL), tobe(0xb8757bdaL), tobe(0xb5365d03L), tobe(0xb1f740b4L)
};


#define __pure

/**
 * crc32_le() - Calculate bitwise little-endian Ethernet AUTODIN II CRC32
 * @crc: seed value for computation.  ~0 for Ethernet, sometimes 0 for
 *	other uses, or the previous crc32 value if computing incrementally.
 * @p: pointer to buffer over which CRC is run
 * @len: length of buffer @p
 */
u32 __pure crc32_le(u32 crc, unsigned char const *p, size_t len);

#if CRC_LE_BITS == 1
/*
 * In fact, the table-based code will work in this case, but it can be
 * simplified by inlining the table in ?: form.
 */

u32 __pure crc32_le(u32 crc, unsigned char const *p, size_t len)
{
	int i;
	while (len--) {
		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? CRCPOLY_LE : 0);
	}
	return crc;
}
#else				/* Table-based approach */

u32 __pure crc32_le(u32 crc, unsigned char const *p, size_t len)
{
# if CRC_LE_BITS == 8
	const u32      *b =(u32 *)p;
	const u32      *tab = crc32table_le;

# ifdef __LITTLE_ENDIAN
#  define DO_CRC(x) crc = tab[ (crc ^ (x)) & 255 ] ^ (crc>>8)
# else
#  define DO_CRC(x) crc = tab[ ((crc >> 24) ^ (x)) & 255] ^ (crc<<8)
# endif

	crc = __cpu_to_le32(crc);
	/* Align it */
	if(unlikely(((long)b)&3 && len)){
		do {
			u8 *p = (u8 *)b;
			DO_CRC(*p++);
			b = (void *)p;
		} while ((--len) && ((long)b)&3 );
	}
	if(likely(len >= 4)){
		/* load data 32 bits wide, xor data 32 bits wide. */
		size_t save_len = len & 3;
	        len = len >> 2;
		--b; /* use pre increment below(*++b) for speed */
		do {
			crc ^= *++b;
			DO_CRC(0);
			DO_CRC(0);
			DO_CRC(0);
			DO_CRC(0);
		} while (--len);
		b++; /* point to next byte(s) */
		len = save_len;
	}
	/* And the last few bytes */
	if(len){
		do {
			u8 *p = (u8 *)b;
			DO_CRC(*p++);
			b = (void *)p;
		} while (--len);
	}

	return __le32_to_cpu(crc);
#undef ENDIAN_SHIFT
#undef DO_CRC

# elif CRC_LE_BITS == 4
	while (len--) {
		crc ^= *p++;
		crc = (crc >> 4) ^ crc32table_le[crc & 15];
		crc = (crc >> 4) ^ crc32table_le[crc & 15];
	}
	return crc;
# elif CRC_LE_BITS == 2
	while (len--) {
		crc ^= *p++;
		crc = (crc >> 2) ^ crc32table_le[crc & 3];
		crc = (crc >> 2) ^ crc32table_le[crc & 3];
		crc = (crc >> 2) ^ crc32table_le[crc & 3];
		crc = (crc >> 2) ^ crc32table_le[crc & 3];
	}
	return crc;
# endif
}
#endif

/**
 * crc32_be() - Calculate bitwise big-endian Ethernet AUTODIN II CRC32
 * @crc: seed value for computation.  ~0 for Ethernet, sometimes 0 for
 *	other uses, or the previous crc32 value if computing incrementally.
 * @p: pointer to buffer over which CRC is run
 * @len: length of buffer @p
 */
u32 __pure crc32_be(u32 crc, unsigned char const *p, size_t len);

#if CRC_BE_BITS == 1
/*
 * In fact, the table-based code will work in this case, but it can be
 * simplified by inlining the table in ?: form.
 */

u32 __pure crc32_be(u32 crc, unsigned char const *p, size_t len)
{
	int i;
	while (len--) {
		crc ^= *p++ << 24;
		for (i = 0; i < 8; i++)
			crc =
			    (crc << 1) ^ ((crc & 0x80000000) ? CRCPOLY_BE :
					  0);
	}
	return crc;
}

#else				/* Table-based approach */
u32 __pure crc32_be(u32 crc, unsigned char const *p, size_t len)
{
# if CRC_BE_BITS == 8
	const u32      *b =(u32 *)p;
	const u32      *tab = crc32table_be;

# ifdef __LITTLE_ENDIAN
#  define DO_CRC(x) crc = tab[ (crc ^ (x)) & 255 ] ^ (crc>>8)
# else
#  define DO_CRC(x) crc = tab[ ((crc >> 24) ^ (x)) & 255] ^ (crc<<8)
# endif

	crc = __cpu_to_be32(crc);
	/* Align it */
	if(unlikely(((long)b)&3 && len)){
		do {
			u8 *p = (u8 *)b;
			DO_CRC(*p++);
			b = (u32 *)p;
		} while ((--len) && ((long)b)&3 );
	}
	if(likely(len >= 4)){
		/* load data 32 bits wide, xor data 32 bits wide. */
		size_t save_len = len & 3;
	        len = len >> 2;
		--b; /* use pre increment below(*++b) for speed */
		do {
			crc ^= *++b;
			DO_CRC(0);
			DO_CRC(0);
			DO_CRC(0);
			DO_CRC(0);
		} while (--len);
		b++; /* point to next byte(s) */
		len = save_len;
	}
	/* And the last few bytes */
	if(len){
		do {
			u8 *p = (u8 *)b;
			DO_CRC(*p++);
			b = (void *)p;
		} while (--len);
	}
	return __be32_to_cpu(crc);
#undef ENDIAN_SHIFT
#undef DO_CRC

# elif CRC_BE_BITS == 4
	while (len--) {
		crc ^= *p++ << 24;
		crc = (crc << 4) ^ crc32table_be[crc >> 28];
		crc = (crc << 4) ^ crc32table_be[crc >> 28];
	}
	return crc;
# elif CRC_BE_BITS == 2
	while (len--) {
		crc ^= *p++ << 24;
		crc = (crc << 2) ^ crc32table_be[crc >> 30];
		crc = (crc << 2) ^ crc32table_be[crc >> 30];
		crc = (crc << 2) ^ crc32table_be[crc >> 30];
		crc = (crc << 2) ^ crc32table_be[crc >> 30];
	}
	return crc;
# endif
}
#endif

/*
 * A brief CRC tutorial.
 *
 * A CRC is a long-division remainder.  You add the CRC to the message,
 * and the whole thing (message+CRC) is a multiple of the given
 * CRC polynomial.  To check the CRC, you can either check that the
 * CRC matches the recomputed value, *or* you can check that the
 * remainder computed on the message+CRC is 0.  This latter approach
 * is used by a lot of hardware implementations, and is why so many
 * protocols put the end-of-frame flag after the CRC.
 *
 * It's actually the same long division you learned in school, except that
 * - We're working in binary, so the digits are only 0 and 1, and
 * - When dividing polynomials, there are no carries.  Rather than add and
 *   subtract, we just xor.  Thus, we tend to get a bit sloppy about
 *   the difference between adding and subtracting.
 *
 * A 32-bit CRC polynomial is actually 33 bits long.  But since it's
 * 33 bits long, bit 32 is always going to be set, so usually the CRC
 * is written in hex with the most significant bit omitted.  (If you're
 * familiar with the IEEE 754 floating-point format, it's the same idea.)
 *
 * Note that a CRC is computed over a string of *bits*, so you have
 * to decide on the endianness of the bits within each byte.  To get
 * the best error-detecting properties, this should correspond to the
 * order they're actually sent.  For example, standard RS-232 serial is
 * little-endian; the most significant bit (sometimes used for parity)
 * is sent last.  And when appending a CRC word to a message, you should
 * do it in the right order, matching the endianness.
 *
 * Just like with ordinary division, the remainder is always smaller than
 * the divisor (the CRC polynomial) you're dividing by.  Each step of the
 * division, you take one more digit (bit) of the dividend and append it
 * to the current remainder.  Then you figure out the appropriate multiple
 * of the divisor to subtract to being the remainder back into range.
 * In binary, it's easy - it has to be either 0 or 1, and to make the
 * XOR cancel, it's just a copy of bit 32 of the remainder.
 *
 * When computing a CRC, we don't care about the quotient, so we can
 * throw the quotient bit away, but subtract the appropriate multiple of
 * the polynomial from the remainder and we're back to where we started,
 * ready to process the next bit.
 *
 * A big-endian CRC written this way would be coded like:
 * for (i = 0; i < input_bits; i++) {
 *	multiple = remainder & 0x80000000 ? CRCPOLY : 0;
 *	remainder = (remainder << 1 | next_input_bit()) ^ multiple;
 * }
 * Notice how, to get at bit 32 of the shifted remainder, we look
 * at bit 31 of the remainder *before* shifting it.
 *
 * But also notice how the next_input_bit() bits we're shifting into
 * the remainder don't actually affect any decision-making until
 * 32 bits later.  Thus, the first 32 cycles of this are pretty boring.
 * Also, to add the CRC to a message, we need a 32-bit-long hole for it at
 * the end, so we have to add 32 extra cycles shifting in zeros at the
 * end of every message,
 *
 * So the standard trick is to rearrage merging in the next_input_bit()
 * until the moment it's needed.  Then the first 32 cycles can be precomputed,
 * and merging in the final 32 zero bits to make room for the CRC can be
 * skipped entirely.
 * This changes the code to:
 * for (i = 0; i < input_bits; i++) {
 *      remainder ^= next_input_bit() << 31;
 *	multiple = (remainder & 0x80000000) ? CRCPOLY : 0;
 *	remainder = (remainder << 1) ^ multiple;
 * }
 * With this optimization, the little-endian code is simpler:
 * for (i = 0; i < input_bits; i++) {
 *      remainder ^= next_input_bit();
 *	multiple = (remainder & 1) ? CRCPOLY : 0;
 *	remainder = (remainder >> 1) ^ multiple;
 * }
 *
 * Note that the other details of endianness have been hidden in CRCPOLY
 * (which must be bit-reversed) and next_input_bit().
 *
 * However, as long as next_input_bit is returning the bits in a sensible
 * order, we can actually do the merging 8 or more bits at a time rather
 * than one bit at a time:
 * for (i = 0; i < input_bytes; i++) {
 *	remainder ^= next_input_byte() << 24;
 *	for (j = 0; j < 8; j++) {
 *		multiple = (remainder & 0x80000000) ? CRCPOLY : 0;
 *		remainder = (remainder << 1) ^ multiple;
 *	}
 * }
 * Or in little-endian:
 * for (i = 0; i < input_bytes; i++) {
 *	remainder ^= next_input_byte();
 *	for (j = 0; j < 8; j++) {
 *		multiple = (remainder & 1) ? CRCPOLY : 0;
 *		remainder = (remainder << 1) ^ multiple;
 *	}
 * }
 * If the input is a multiple of 32 bits, you can even XOR in a 32-bit
 * word at a time and increase the inner loop count to 32.
 *
 * You can also mix and match the two loop styles, for example doing the
 * bulk of a message byte-at-a-time and adding bit-at-a-time processing
 * for any fractional bytes at the end.
 *
 * The only remaining optimization is to the byte-at-a-time table method.
 * Here, rather than just shifting one bit of the remainder to decide
 * in the correct multiple to subtract, we can shift a byte at a time.
 * This produces a 40-bit (rather than a 33-bit) intermediate remainder,
 * but again the multiple of the polynomial to subtract depends only on
 * the high bits, the high 8 bits in this case.
 *
 * The multiple we need in that case is the low 32 bits of a 40-bit
 * value whose high 8 bits are given, and which is a multiple of the
 * generator polynomial.  This is simply the CRC-32 of the given
 * one-byte message.
 *
 * Two more details: normally, appending zero bits to a message which
 * is already a multiple of a polynomial produces a larger multiple of that
 * polynomial.  To enable a CRC to detect this condition, it's common to
 * invert the CRC before appending it.  This makes the remainder of the
 * message+crc come out not as zero, but some fixed non-zero value.
 *
 * The same problem applies to zero bits prepended to the message, and
 * a similar solution is used.  Instead of starting with a remainder of
 * 0, an initial remainder of all ones is used.  As long as you start
 * the same way on decoding, it doesn't make a difference.
 */

#ifdef UNITTEST

#include <stdlib.h>
#include <stdio.h>

#if 0				/*Not used at present */
static void
buf_dump(char const *prefix, unsigned char const *buf, size_t len)
{
	fputs(prefix, stdout);
	while (len--)
		printf(" %02x", *buf++);
	putchar('\n');

}
#endif

static void bytereverse(unsigned char *buf, size_t len)
{
	while (len--) {
		unsigned char x = bitrev8(*buf);
		*buf++ = x;
	}
}

static void random_garbage(unsigned char *buf, size_t len)
{
	while (len--)
		*buf++ = (unsigned char) random();
}

#if 0				/* Not used at present */
static void store_le(u32 x, unsigned char *buf)
{
	buf[0] = (unsigned char) x;
	buf[1] = (unsigned char) (x >> 8);
	buf[2] = (unsigned char) (x >> 16);
	buf[3] = (unsigned char) (x >> 24);
}
#endif

static void store_be(u32 x, unsigned char *buf)
{
	buf[0] = (unsigned char) (x >> 24);
	buf[1] = (unsigned char) (x >> 16);
	buf[2] = (unsigned char) (x >> 8);
	buf[3] = (unsigned char) x;
}

/*
 * This checks that CRC(buf + CRC(buf)) = 0, and that
 * CRC commutes with bit-reversal.  This has the side effect
 * of bytewise bit-reversing the input buffer, and returns
 * the CRC of the reversed buffer.
 */
static u32 test_step(u32 init, unsigned char *buf, size_t len)
{
	u32 crc1, crc2;
	size_t i;

	crc1 = crc32_be(init, buf, len);
	store_be(crc1, buf + len);
	crc2 = crc32_be(init, buf, len + 4);
	if (crc2)
		printf("\nCRC cancellation fail: 0x%08x should be 0\n",
		       crc2);

	for (i = 0; i <= len + 4; i++) {
		crc2 = crc32_be(init, buf, i);
		crc2 = crc32_be(crc2, buf + i, len + 4 - i);
		if (crc2)
			printf("\nCRC split fail: 0x%08x\n", crc2);
	}

	/* Now swap it around for the other test */

	bytereverse(buf, len + 4);
	init = bitrev32(init);
	crc2 = bitrev32(crc1);
	if (crc1 != bitrev32(crc2))
		printf("\nBit reversal fail: 0x%08x -> 0x%08x -> 0x%08x\n",
		       crc1, crc2, bitrev32(crc2));
	crc1 = crc32_le(init, buf, len);
	if (crc1 != crc2)
		printf("\nCRC endianness fail: 0x%08x != 0x%08x\n", crc1,
		       crc2);
	crc2 = crc32_le(init, buf, len + 4);
	if (crc2)
		printf("\nCRC cancellation fail: 0x%08x should be 0\n",
		       crc2);

	for (i = 0; i <= len + 4; i++) {
		crc2 = crc32_le(init, buf, i);
		crc2 = crc32_le(crc2, buf + i, len + 4 - i);
		if (crc2)
			printf("\nCRC split fail: 0x%08x\n", crc2);
	}

	return crc1;
}

#define SIZE 64
#define INIT1 0
#define INIT2 0

int main(void)
{
	unsigned char buf1[SIZE + 4];
	unsigned char buf2[SIZE + 4];
	unsigned char buf3[SIZE + 4];
	int i, j;
	u32 crc1, crc2, crc3;

	for (i = 0; i <= SIZE; i++) {
		printf("\rTesting length %d...", i);
		fflush(stdout);
		random_garbage(buf1, i);
		random_garbage(buf2, i);
		for (j = 0; j < i; j++)
			buf3[j] = buf1[j] ^ buf2[j];

		crc1 = test_step(INIT1, buf1, i);
		crc2 = test_step(INIT2, buf2, i);
		/* Now check that CRC(buf1 ^ buf2) = CRC(buf1) ^ CRC(buf2) */
		crc3 = test_step(INIT1 ^ INIT2, buf3, i);
		if (crc3 != (crc1 ^ crc2))
			printf("CRC XOR fail: 0x%08x != 0x%08x ^ 0x%08x\n",
			       crc3, crc1, crc2);
	}
	printf("\nAll test complete.  No failures expected.\n");
	return 0;
}

#endif				/* UNITTEST */

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
