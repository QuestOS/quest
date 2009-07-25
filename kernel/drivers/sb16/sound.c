#include "types.h"
#include "drivers/sb16/sound.h"
#include "kernel.h"
#include "fs/filesys.h"
#include "smp/smp.h"

PRIVATE uint16 dsp_version;     // Version of the Digital Sound Processor

// Driver capabilities
PRIVATE SB_CAPABILITY driver_capability;        // Depends on the SB card capability

PRIVATE uint16 dsp_base_address = 0x220;        // I/O Base Address of the DSP
PRIVATE uint8 dsp_irq_number = 5;       // IRQ used for DMA transfer
PRIVATE uint8 dsp_dma_channel_8 = 1;    // DMA channel in 8-bit mode
PRIVATE uint8 dsp_dma_channel_16 = 5;   // DMA channel in 16-bit mode

#ifdef __8_BIT__
/* Port settings for page, count & memory address DMA registers (8-bit mode) */
PRIVATE uint8 driver_dma_page[4] = { 0x87, 0x83, 0x81, 0x82 };
PRIVATE uint8 driver_dma_length[4] = { 0x01, 0x03, 0x05, 0x07 };
PRIVATE uint8 driver_dma_address[4] = { 0x00, 0x02, 0x04, 0x06 };
#endif

/* Port settings for page, count & memory address DMA registers (16-bit mode) */
PRIVATE uint8 driver_dma_page[4] = { 0x8F, 0x8B, 0x89, 0x8A };
PRIVATE uint8 driver_dma_length[4] = { 0xC2, 0xC6, 0xCA, 0xCE };
PRIVATE uint8 driver_dma_address[4] = { 0xC0, 0xC4, 0xC8, 0xCC };

/* Virtual address of DMA buffer */
static uint32 dma_buffer_virt_base;
/* Base physical address of a 64KB DMA buffer */
uint32 dma_buffer_phys_base;

static int filesize;
static char filebuffer[0x10000];

bool
sb_dsp_reset (uint16 base_address)
{
  int i;
  // int      start_time;

  // Write a 1 to the SB RESET port
  outb (1, base_address + SB_DSP_RESET);

  // Wait for >= 3 microseconds 
  for (i = 0; i < 6; i++)
    inb (base_address + SB_DSP_WRITE_BUFFER_STATUS);

  // Write a 0 to the SB RESET port
  outb (0, base_address + SB_DSP_RESET);

  // Read the byte from the DATA_AVAILABLE port until bit 7 = 1
  while ((inb (base_address + SB_DSP_READ_BUFFER_STATUS) < 0x80));

  // Poll for a ready byte (AAh) from the READ DATA port
  while (((i = inb (base_address + SB_DSP_READ_DATA)) != 0xAA));

  if (i != 0xAA)
    return (SB_RESET_DSP);
  return (SB_OK);
}


bool
sb_dsp_detect_base_address (uint16 * base_address)
{

  uint16 address;

  for (address = 0x220; address <= 0x280; address += 0x020)
    if (!sb_dsp_reset (address)) {
      *base_address = address;
      dsp_base_address = address;
      return (SB_OK);
    }

  return (SB_DETECT_DSP);
}


bool
sb_dsp_detect_irq_number (uint16 base_address, uint8 * irq_number)
{

  uint8 b;

  outb (INTERRUPT_SETUP, base_address + MIXER_REG_ADDR_PORT);
  b = inb (base_address + MIXER_DATA_PORT);

  switch (b) {                  /* Possible IRQ settings for sound card */
  case 1:
    dsp_irq_number = 2;
    break;                      /* IRQ 2 */
  case 2:
    dsp_irq_number = 5;
    break;                      /* IRQ 5 */
  case 4:
    dsp_irq_number = 7;
    break;                      /* IRQ 7 */
  case 8:
    dsp_irq_number = 10;
    break;                      /* IRQ 10 */
  default:
    dsp_irq_number = 0;         /* Error */
  }

  if ((*irq_number = dsp_irq_number))
    return (SB_OK);
  else
    return (SB_INVALID_IRQ);
}


bool
sb_dsp_detect_dma (uint16 base_address, uint8 * dma8, uint8 * dma16)
{

  uint8 b;

  outb (DMA_SETUP, base_address + MIXER_REG_ADDR_PORT);
  b = inb (base_address + MIXER_DATA_PORT);

  *dma8 = b & 0x08 ? 3 : b & 0x02 ? 1 : 0;
  *dma16 = b >> 7 ? 7 : b >> 6 ? 6 : 5;

  if (*dma8 && *dma16)
    return (SB_OK);
  else
    return (SB_INVALID_DMA);
}



bool
sb_dsp_write (uint8 value)
{

  if (!dsp_base_address)
    return (SB_NOT_INITIALIZED);

  // Read the DSP's BUFFER_STATUS port until bit 7 = 0
  while (inb (dsp_base_address + SB_DSP_WRITE_BUFFER_STATUS) & 0x80);

  // Write the value to the WRITE command/data port
  outb (value, dsp_base_address + SB_DSP_WRITE);
  return (SB_OK);
}


bool
sb_dsp_read (uint8 * value)
{

  if (!dsp_base_address)
    return (SB_NOT_INITIALIZED);

  // Read the DSP's DATA_AVAILABLE port until bit 7 = 1
  while (inb (dsp_base_address + SB_DSP_READ_BUFFER_STATUS) < 0x80);

  // Read the data from the READ_DATA port
  *value = inb (dsp_base_address + SB_DSP_READ_DATA);
  return (SB_OK);
}


bool
sb_speaker_on (void)
{

  return (sb_dsp_write (SB_SPEAKER_ON));
}

bool
sb_speaker_off (void)
{

  return (sb_dsp_write (SB_SPEAKER_OFF));
}


// Read the DSP version and sets capabilities
//
// Sound-Blaster AWE32   >=0x040C
// Sound-Blaster 16      >=0x0400
// Sound-Blaster Pro     >=0x0300
// Sound-Blaster 2.0     >=0x0201
// Sound-Blaster 1.0/1.5 else
bool
sb_dsp_get_version (uint16 * version)
{
  uint8 value;
  bool result;

  if ((result = sb_dsp_write (SB_DSP_VERSION)))
    return (result);

  if ((result = sb_dsp_read (&value)))
    return (result);

  dsp_version = (uint16) value << 8;
  if ((result = sb_dsp_read (&value)))
    return (result);
  dsp_version += value;
  *version = dsp_version;

  if (dsp_version >= 0x040C)
    driver_capability = capability_sb_awe32;
  else if (dsp_version >= 0x0400)
    driver_capability = capability_sb_16;
  else if (dsp_version >= 0x0300)
    driver_capability = capability_sb_pro;
  else if (dsp_version >= 0x0201)
    driver_capability = capability_sb_20;
  else
    driver_capability = capability_sb_10;
  return (SB_OK);
}


PRIVATE bool
driver_set_time_constant (uint16 frequency)
{

  // Reset the mixer
  sb_mixer_register_set (MIXER_RESET, 0);

  // Set mono mode
  sb_mixer_register_set (MIXER_OUTPUT, 0x13);

  /* Sets the time constant / sampling rate for SB16 and above, DSP version
   * 4.xx */
  sb_dsp_write (0x41);
  sb_dsp_write (frequency >> 8);
  sb_dsp_write (frequency & 0xFF);

  return (SB_OK);
}


PRIVATE void
driver_setup_dma_transfer (int memory_size)
{

  /* Disable DMA */
  outb (0x04 + dsp_dma_channel_8, SB_DMA_MASK_16);

  /* Clear byte pointer flip-flop ready for DMA reprogramming */
  outb (0, SB_DMA_CLEAR_16);

  /* Single-cycle, addr increment, single-mode, read transfer */
  outb (0x48 + dsp_dma_channel_8, SB_DMA_MODE_16);

  /* 64K aligned DMA buffer physical "page" info */
  outb ((dma_buffer_phys_base >> 16) & 0xFF,
        driver_dma_page[dsp_dma_channel_8]);
  outb (0 /* offset (low byte) in 64K "page" --??-- for now, 0 */ ,
        driver_dma_address[dsp_dma_channel_8]);
  outb (0 /* offset (high byte, >>8) in 64K "page" --??-- for now, 0 */ ,
        driver_dma_address[dsp_dma_channel_8]);
  outb ((memory_size - 1) & 0xFF /* (DMA memory size-1) low byte */ ,
        driver_dma_length[dsp_dma_channel_8]);
  outb ((memory_size - 1) >> 8 /* (DMA memory size-1) high byte, >>8 */ ,
        driver_dma_length[dsp_dma_channel_8]);

  /* Enable DMA */
  outb (0x00 + dsp_dma_channel_8, SB_DMA_MASK_16);
}

PRIVATE void
driver_setup_dsp_transfer (int memory_size)
{

  memory_size >>= 1;

  /* 16-bit DMA mode digitized sound I/O */
  sb_dsp_write (0xB0);          /* command */
  sb_dsp_write (0x00);          /* mode: mono signal, unsigned data */
  sb_dsp_write ((memory_size - 1) & 0xFF);      /* length - low byte */
  sb_dsp_write ((memory_size - 1) >> 8);        /* length - high byte */
}


// Write a value to the mixer register
bool
sb_mixer_register_set (uint8 index, uint8 value)
{
  if (!dsp_base_address)
    return (SB_NOT_INITIALIZED);

  // Set the register index
  outb (index, dsp_base_address + MIXER_REG_ADDR_PORT);

  // Write the value to the mixer register
  outb (value, dsp_base_address + MIXER_DATA_PORT);
  return (SB_OK);
}


// Read a value from the mixer register
bool
sb_mixer_register_get (uint8 index, uint8 * value)
{
  if (!dsp_base_address)
    return (SB_NOT_INITIALIZED);

  // Set the register index
  outb (index, dsp_base_address + MIXER_REG_ADDR_PORT);

  // Read the value of the mixer register
  *value = inb (dsp_base_address + MIXER_DATA_PORT);
  return (SB_OK);
}


static void
play_sample (void)
{

  static int fileoffset;
  int chunksize;

  if (!filesize)
    /* all finished */
    return;

  chunksize = filesize > SB_MEMORY_SIZE ? SB_MEMORY_SIZE : filesize;

  memcpy ((void *) dma_buffer_virt_base, filebuffer + fileoffset, chunksize);
  fileoffset += chunksize;

  /* Single-cycle mode: need to reinitialise dma/dsp for next block */
  driver_setup_dma_transfer (chunksize);
  driver_setup_dsp_transfer (chunksize);

  filesize -= chunksize;
}


/* IRQ5 soundcard interrupt handler: --??-- in future, should not be hardcoded
   to specific IRQ */
void
_soundcard (void)
{

  inb (dsp_base_address + SB_IRQ_ACK_16);

  play_sample ();

  /* Acknowledge the interrupt */
    /****************************************
     * if (dsp_irq_number == 10)            *
     *     outb (0x20, SB_PIC2_EOI);        *
     * outb (0x20, SB_PIC1_EOI);            *
     ****************************************/
  send_eoi ();
}


// Install the Sound Blaster Driver
// 'frequency': sound driver frequency (Hz)
// 'stereo': use stereo mode (not currently supported)
bool
sb_install_driver (uint16 frequency, bool use_stereo)
{
  bool result;

  if ((result = driver_set_time_constant (frequency)))
    return (result);

  play_sample ();

  return (SB_OK);
}


bool
sb_read_raw (char *pathname)
{

  filesize = vfs_dir (pathname);        /* --??-- Need error checking */

  vfs_read (filebuffer, filesize);

  return (SB_OK);
}


void
initialise_sound (void)
{

  int i;

  /* Search for an unallocated contiguous aligned 64K block.  Each 32-bit
     section of the bitmap describes 32 4K pages, i.e. 128K.  We want to
     stay under the 16MB 24-bit DMA boundary, so we can scan as far as the
     16M/4K = 4096th bit. */
  for (i = 0; i < 0x80; i++)
    if ((mm_table[i] & 0xFFFF) == 0xFFFF) {
      /* found a free 64K region on a 128K boundary */
      mm_table[i] &= 0xFFFF0000;
      dma_buffer_phys_base = i << 17;
      break;
    } else if ((mm_table[i] & 0xFFFF0000) == 0xFFFF0000) {
      /* found a free 64K region halfway through a 128K boundary */
      mm_table[i] &= 0xFFFF;
      dma_buffer_phys_base = (i << 17) | 0x10000;
      break;
    }

  if (i >= 0x80)
    panic ("No suitable DMA buffer found");

  sb_read_raw ("/boot/welcome.raw");

  /* For now, just 4KB for buffer */
  dma_buffer_virt_base = (uint32) MapVirtualPage (dma_buffer_phys_base | 3);
  sb_dsp_detect_base_address (&dsp_base_address);
  sb_dsp_detect_irq_number (dsp_base_address, &dsp_irq_number);
  sb_dsp_detect_dma (dsp_base_address,
                     &dsp_dma_channel_8, &dsp_dma_channel_16);
  sb_dsp_get_version (&dsp_version);
  sb_speaker_on ();
  sb_install_driver (11025, FALSE);
}
