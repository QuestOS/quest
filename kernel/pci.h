#ifndef _PCI_H_
#define _PCI_H_

#include"types.h"

#define PCI_CONFIG_ADDRESS 0xCF8
#define PCI_CONFIG_DATA    0xCFC

typedef struct _pci_config_addr {
  unsigned regNum : 8;
  unsigned funcNum : 3;
  unsigned devNum : 5;
  unsigned busNum : 8;
  unsigned reserved : 7;
  unsigned enable : 1;
} pci_config_addr;

static inline void 
pci_config_addr_init(pci_config_addr *a, 
                     WORD bus,
                     WORD dev,
                     WORD func,
                     WORD reg) {
  a->regNum   = reg;
  a->funcNum  = func;
  a->devNum   = dev;
  a->busNum   = bus;
  a->reserved = 0;
  a->enable   = 1;
}

static inline void
pci_read_byte(pci_config_addr *a, BYTE *v) {
  WORD offs = a->regNum & 0x3;
  DWORD a32, i32;
  a32 = *((DWORD *)a) & (~0x3); /* zero out lowest 2 bits */
  outl(a32, PCI_CONFIG_ADDRESS);
  i32 = inl(PCI_CONFIG_DATA);
  /* offs selects the first, second, third, or fourth byte in i32. */
  *v = (i32 >> (offs << 3)) & 0xFF;
}

static inline void
pci_read_word(pci_config_addr *a, WORD *v) {
  WORD offs = a->regNum & 0x2;
  DWORD a32, i32;
  a32 = *((DWORD *)a) & (~0x3); /* zero out lowest 2 bits */
  outl(a32, PCI_CONFIG_ADDRESS);
  i32 = inl(PCI_CONFIG_DATA);
  /* offs selects the first or second word in i32. */
  *v = (i32 >> (offs << 3)) & 0xFFFF;
}

static inline void
pci_read_dword(pci_config_addr *a, DWORD *v) {
  DWORD a32, i32;
  a32 = *((DWORD *)a) & (~0x3); /* zero out lowest 2 bits */
  outl(a32, PCI_CONFIG_ADDRESS);
  i32 = inl(PCI_CONFIG_DATA);
  *v = i32;
}

static inline void
pci_write_byte(pci_config_addr *a, BYTE v) {
  WORD offs = a->regNum & 0x3;
  DWORD a32;
  a32 = *((DWORD *)a) & (~0x3); /* zero out lowest 2 bits */
  outl(a32, PCI_CONFIG_ADDRESS);
  /* offs selects the first, second, third, or fourth byte in i32. */
  outb(v, PCI_CONFIG_DATA+offs);

}

static inline void
pci_write_word(pci_config_addr *a, WORD v) {
  WORD offs = a->regNum & 0x2;
  DWORD a32;
  a32 = *((DWORD *)a) & (~0x3); /* zero out lowest 2 bits */
  outl(a32, PCI_CONFIG_ADDRESS);
  /* offs selects the first or second word in i32. */
  outw(v, PCI_CONFIG_DATA+offs);
}

static inline void
pci_write_dword(pci_config_addr *a, DWORD v) {
  DWORD a32;
  a32 = *((DWORD *)a) & (~0x3); /* zero out lowest 2 bits */
  outl(a32, PCI_CONFIG_ADDRESS);
  outl(v, PCI_CONFIG_DATA);
}

#endif