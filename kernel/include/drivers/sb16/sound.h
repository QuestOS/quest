/* -*- Mode: C; c-file-style: "gnu"; c-basic-offset: 2; indent-tabs-mode: nil -*- */

#ifndef __SOUND_H__
#define __SOUND_H__

// Error costants
#define SB_OK                   0       // No errors
#define SB_RESET_DSP            1       // Cannot reset the SB DSP
#define SB_DETECT_DSP           2       // Cannot detect the SB DSP I/O base address
#define SB_NOT_INITIALIZED      3       // SB Library not initialized
#define SB_INVALID_IRQ          4       // Invalid IRQ number
#define SB_INVALID_DMA          5       // Invalid DMA channel settings
#define SB_SAMPLE_RATE          6       // Sample rate out of range
#define SB_TOO_MANY_VOICES      7       // Driver is playing too many voices
#define SB_USE_STEREO           8       // Stereo mode not supported
#define SB_INVALID_FILE         9       // Invalid (sound) file

// Defining the offset of the SB I/O ports
#define SB_DSP_RESET            0x06    // DSP Reset Port
#define SB_DSP_READ_DATA        0x0A    // DSP Read Data Port
#define SB_DSP_WRITE            0x0C    // DSP Write Command/Data (out)
#define SB_DSP_WRITE_BUFFER_STATUS 0x0C // DSP Write Buffer Status (in)
#define SB_DSP_READ_BUFFER_STATUS  0x0E // DSP Data Available

#define INTERRUPT_SETUP         0x80
#define DMA_SETUP               0x81

// Other definitions
#define SB_RESET_TIMEOUT        5000000
#define SB_IRQ_DETECT_TIMEOUT   5000000
#define SB_HIGH_SPEED_FREQUENCY 23 * 1024
#define SB_MEMORY_SIZE          4096
#define SB_MAX_VOICES           8
#define WAV_CHUNK_LENGTH        12
#define SB_DRIVER_VERSION_HI    0x01
#define SB_DRIVER_VERSION_LO    0x05

// Mixer ports and registers
#define MIXER_REG_ADDR_PORT     0x04
#define MIXER_DATA_PORT         0x05
#define MIXER_RESET             0x00
#define MIXER_OUTPUT            0x0E

// Supported sound files extensions
#define SB_EXTENSION_RAW        ".RAW"
#define SB_EXTENSION_WAV        ".WAV"


// DSP Commands
#define SB_SPEAKER_ON        0xD1       // Turn Speaker on
#define SB_SPEAKER_OFF       0xD3       // Turn Speaker off
#define SB_DSP_VERSION       0xE1       // Gets the DSP version
#define SB_DMA_MODE_8_SC     0x14       // DMA DAC, 8-bit (Single Cycle, Norm Speed)
#define SB_DMA_MODE_8_AI_NS  0x1C       // Auto-Init DMA DAC, 8-bit (Normal Speed)
#define SB_TIME_CONSTANT     0x40       // Set Time Constant
#define SB_DMA_BLOCK_SIZE    0x48       // Set DMA Block Size
#define SB_DMA_MODE_8_AI_HS  0x90       // Auto-Init DMA DAC, 8-bit (High Speed)
#define SB_HALT_DMA          0xD0       // Halt DMA Operation, 8-bit
#define SB_EXIT_DMA          0xDA       // Exit Auto-Init DMA Operation, 8-bit
#define SB_IRQ_TRIGGER       0xF2       // Triggers SB interrupt

// PIC ports addresses
#define SB_PIC1_EOI          0x20       // PIC 1 EOI (End Of Interrupt)
#define SB_PIC2_EOI          0xA0       // PIC 2 EOI
#define SB_PIC_MASK_1        0x21       // PIC 1 port (master)
#define SB_PIC_MASK_2        0xA1       // PIC 2 port (slave)

// IRQ ACK ports
#define SB_IRQ_ACK_8         0x0E       // ACK of the 8 bit IRQ
#define SB_IRQ_ACK_16        0x0F       // ACK of the 16 bit IRQ

// DMA Ports
#define SB_DMA_MASK_8   0x0A    // 8-bit DMA mask register
#define SB_DMA_MODE_8   0x0B    // 8-bit DMA mode register
#define SB_DMA_CLEAR_8  0x0C    // 8-bit DMA clear byte ptr
#define SB_DMA_MASK_16  0xD4    // 16-bit DMA mask port
#define SB_DMA_MODE_16  0xD6    // 16-bit DMA mode register
#define SB_DMA_CLEAR_16 0xD8    // 16-bit DMA clear byte ptr

typedef struct
{
  char riffsign[4] /*PACKED*/;  // The RIFF signature (should be 'RIFF')
  int length PACKED;            // The length of the data in the next chunk
  char wavesign[4] /*PACKED*/;  // The WAVE signature (should be 'WAVE')
  char ftmsign[4] /*PACKED*/;   // Contains the characters 'fmt'
  int formatlength PACKED;      // Length of the data in the format chunk
  uint16 waveformat PACKED;     // Wave Format
  uint16 channels PACKED;       // Number of channels (1=mono, 2=stereo)
  uint16 samplespersec PACKED;  // Playback Frequency
  uint16 averagebytes PACKED;   // average number of bytes a second
  uint16 blockalign PACKED;     // block alignment of the data
  uint16 formatspecific PACKED; // Format specific data area
} WAVCHUNK;

typedef struct
{
  char sign[4] /*PACKED*/;      // Contains the characters 'data'
  int length PACKED;            // Data length
} DATACHUNK;


typedef struct
{
  uint16 min_mono_8;            // Min mono sample for 8 bits
  uint16 max_mono_8;            // Max mono sample for 8 bits
  uint16 min_stereo_8;          // Min stereo sample for 8 bits
  uint16 max_stereo_8;          // Max stereo sample for 8 bits
  uint16 min_mono_16;           // Min mono sample for 16 bits
  uint16 max_mono_16;           // Max mono sample for 16 bits
  uint16 min_stereo_16;         // Min stereo sample for 16 bits
  uint16 max_stereo_16;         // Max stereo sample for 16 bits
  bool auto_dma;                // Can use auto dma
  bool stereo;                  // Can use stereo mode
  bool _16_bit;                 // Can use 16-bit
} SB_CAPABILITY;

// Capabilities of the Sound Blaster cards
// Sound Blaster 1.0/1.5: no A/I DMA, no High Speed, no stereo
PRIVATE SB_CAPABILITY capability_sb_10 = {
min_mono_8:4000,
max_mono_8:22222,
min_stereo_8:0,
max_stereo_8:0,
min_mono_16:0,
max_mono_16:0,
min_stereo_16:0,
max_stereo_16:0,
auto_dma:FALSE,
stereo:FALSE,
_16_bit:FALSE
};

// Sound Blaster 2.0: support A/I DMA, High Speed. No stereo
PRIVATE SB_CAPABILITY capability_sb_20 = {
min_mono_8:4000,
max_mono_8:45454,
min_stereo_8:0,
max_stereo_8:0,
min_mono_16:0,
max_mono_16:0,
min_stereo_16:0,
max_stereo_16:0,
auto_dma:TRUE,
stereo:FALSE,
_16_bit:FALSE
};

// Sound Blaster Pro: support A/I DMA, High Speed, Stereo
PRIVATE SB_CAPABILITY capability_sb_pro = {
min_mono_8:4000,
max_mono_8:45454,
min_stereo_8:4000,
max_stereo_8:22727,
min_mono_16:0,
max_mono_16:0,
min_stereo_16:0,
max_stereo_16:0,
auto_dma:TRUE,
stereo:TRUE,
_16_bit:FALSE
};

// Sound Blaster 16: support A/I DMA, High Speed, Stereo, 16 bit
PRIVATE SB_CAPABILITY capability_sb_16 = {
min_mono_8:4000,
max_mono_8:45454,
min_stereo_8:4000,
max_stereo_8:45454,
min_mono_16:4000,
max_mono_16:45454,
min_stereo_16:4000,
max_stereo_16:45454,
auto_dma:TRUE,
stereo:TRUE,
_16_bit:TRUE
};

// Sound Blaster AWE32: support A/I DMA, High Speed, Stereo, 16 bit
PRIVATE SB_CAPABILITY capability_sb_awe32 = {
min_mono_8:5000,
max_mono_8:45454,
min_stereo_8:5000,
max_stereo_8:45454,
min_mono_16:5000,
max_mono_16:45454,
min_stereo_16:5000,
max_stereo_16:45454,
auto_dma:TRUE,
stereo:TRUE,
_16_bit:TRUE
};

#define SAMPLE_uint8     uint8
#define SAMPLE_CLIP_MIN 0
#define SAMPLE_CLIP_MAX 255

typedef struct
{
  int size;                     // Size of the sound data
  SAMPLE_uint8 *data;           // Data memory area
  bool stereo;                  // Mono or stereo sample data
} SAMPLE;

bool sb_dsp_reset (uint16 base_address);
bool sb_dsp_write (uint8 value);
bool sb_dsp_read (uint8 * value);
bool sb_speaker_on (void);
bool sb_speaker_off (void);
bool sb_dsp_get_version (uint16 * version);
bool sb_get_model_name (char *name);
bool sb_dsp_detect_base_address (uint16 * base_address);
bool sb_dsp_detect_irq_number (uint16 base_address, uint8 * irq_number);
bool sb_dsp_detect_dma (uint16 base_address, uint8 * dma8, uint8 * dma16);
bool sb_play_sample (SAMPLE * sample);
bool sb_mixer_register_set (uint8 index, uint8 value);
bool sb_mixer_register_get (uint8 index, uint8 * value);
bool sb_install_driver (uint16 frequency, bool use_stereo);
// bool sb_read_raw (char *filename, SAMPLE *sample);
#endif

/* vi: set et sw=2 sts=2: */
