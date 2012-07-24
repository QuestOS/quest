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

#include "arch/i386.h"
#include "vm/vm86.h"
#include "kernel.h"
#include "util/printf.h"

#define DEBUG_VM86 4

#if DEBUG_VM86 > 0
#define DLOG(fmt,...) DLOG_PREFIX("vm86",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define com1_printf logger_printf

/* Stored BIOS interrupt vector table */
vm86_farptr vmx_vm86_IVT[256];

void
vmx_vm86_global_init (void)
{
  extern uint32 vmx_vm86_pgt[1024];
  /* Temporarily re-map page 0 */
  vmx_vm86_pgt[0] = 7;
  flush_tlb_all ();
  /* Yes, I'm really dereferencing the address "0x0": back-up the BIOS
   * interrupt vector table. */
  memcpy ((uint8 *) vmx_vm86_IVT, (uint8 *) 0x0, 256*sizeof(vm86_farptr));
#if DEBUG_VM86 > 3
  int i,j;
  /* print the first 32 entries of the IVT */
  for (i=0;i<4;i++) {
    logger_printf ("  ");
    for (j=0;j<8;j++) {
      logger_printf ("IVT[0x%.02X]=%.04X:%.04X ",
                     i*8 + j, vmx_vm86_IVT[i*8+j].segm, vmx_vm86_IVT[i*8+j].offs);
    }
    logger_printf ("\n");
  }
#endif
  /* FIXME: Leave page 0 mapped until I implement a vm86 PF handler. */
  //  vmx_vm86_pgt[0] = 0;
  flush_tlb_all ();
}

/* Update the virtual machine with a new CS:IP */
static inline void
inc_ip (uint8 *eip, uint32 amount)
{
  vm86_farptr new_ip;
  new_ip = LIN32_TO_FP (eip + amount);
  vmwrite (new_ip.offs, VMXENC_GUEST_RIP);
  vmwrite (new_ip.segm, VMXENC_GUEST_CS_SEL);
  vmwrite (new_ip.segm << 4, VMXENC_GUEST_CS_BASE);
}

static inline void
upd_ip (uint32 amount)
{
  uint16 ip = vmread (VMXENC_GUEST_RIP) & 0xFFFF;
  uint16 cs = vmread (VMXENC_GUEST_CS_SEL) & 0xFFFF;
  uint8 *eip = REAL_TO_LIN32 (cs, ip, uint8);
  inc_ip (eip, amount);
}

struct _modrm_byte
{
   uint8 rm  : 3;
   uint8 reg : 3;
   uint8 mod : 2;
};

static void
decode_modrm_32 (uint8 *eip, uint8 *reg_op, bool *has_sib, uint8 *rm,
                 uint8 *disp_size, uint32 *disp)
{
  struct _modrm_byte modrm = *((struct _modrm_byte *) eip);

  /* Check if Scale-Index-Base byte present */
  *has_sib = modrm.mod < 3 && modrm.rm == 4;

  /* Obtain displacement size */
  switch (modrm.mod) {
  case 0:
    if (modrm.rm == 5) {
      *disp_size = 4;
      *disp = *((uint32 *) (eip + (*has_sib ? 2 : 1)));
    } else {
      *disp_size = 0;
      *disp = 0;
    }
    break;
  case 1:
    *disp_size = 1;
    *disp = *((eip + (*has_sib ? 2 : 1)));
    break;
  case 2:
    *disp_size = 4;
    *disp = *((uint32 *) (eip + (*has_sib ? 2 : 1)));
    break;
  case 3:
    *disp_size = 0;
    *disp = 0;
    break;
  }

  *reg_op = modrm.reg;
  *rm = modrm.rm;
}

/* Using the current Guest GDTR, try to set the segment registers to
 * legal working values.  */
static sint32
pick_segment_regs (void)
{
  uint32 i, cs_i = 0, ds_i = 0;
  descriptor *ad;
  descriptor *ad_bas = (descriptor *) vmread (VMXENC_GUEST_GDTR_BASE);
  descriptor *ad_lim =
    (descriptor *)(((uint8 *)ad_bas) + vmread (VMXENC_GUEST_GDTR_LIMIT) + 1);

#if DEBUG_VM86 > 1
  com1_printf ("  pick_segment_regs: ad_bas = %p ad_lim = %p\n",
               ad_bas, ad_lim);
#endif
  for (i = 0, ad = ad_bas; ad < ad_lim; i++, ad++) {
#if DEBUG_VM86 > 1
    com1_printf ("    i=%.02X dpl=%.01X type=%.01X\n", i, ad->uDPL, ad->uType);
#endif
    if (!cs_i) {
      /* pick a code segment */
      if (ad->uDPL == 0 && (ad->uType & 0x8))
        cs_i = i;
    }
    if (!ds_i) {
      /* pick a data segment */
      if (ad->uDPL == 0 && !(ad->uType & 0x8))
        ds_i = i;
    }
  }

  ad = ad_bas;
  if (cs_i && ds_i) {
    uint32 cs_base, ds_base;
    uint32 cs_limit, ds_limit;
    /* found both code and data segments */
#define ACCESS(ad)                              \
  (( 0x01            << 0x00 ) |                \
   ( (ad).uType        << 0x00 ) |              \
   ( (ad).uDPL         << 0x05 ) |              \
   ( (ad).fPresent     << 0x07 ) |              \
   ( (ad).f            << 0x0C ) |              \
   ( (ad).f0           << 0x0D ) |              \
   ( (ad).fX           << 0x0E ) |              \
   ( (ad).fGranularity << 0x0F ))
    vmwrite (ACCESS (ad[cs_i]), VMXENC_GUEST_CS_ACCESS);
    vmwrite (ACCESS (ad[ds_i]), VMXENC_GUEST_SS_ACCESS);
    vmwrite (ACCESS (ad[ds_i]), VMXENC_GUEST_DS_ACCESS);
    vmwrite (ACCESS (ad[ds_i]), VMXENC_GUEST_ES_ACCESS);
    vmwrite (ACCESS (ad[ds_i]), VMXENC_GUEST_FS_ACCESS);
    vmwrite (ACCESS (ad[ds_i]), VMXENC_GUEST_GS_ACCESS);

    cs_base = (ad[cs_i].pBase0 |
               (ad[cs_i].pBase1 << 16) |
               (ad[cs_i].pBase2 << 24));
    ds_base = (ad[ds_i].pBase0 |
               (ad[ds_i].pBase1 << 16) |
               (ad[ds_i].pBase2 << 24));

    cs_limit = (ad[cs_i].uLimit0 | (ad[cs_i].uLimit1 << 16));
    if (ad[cs_i].fGranularity) {
      cs_limit <<= 12;
      cs_limit |= 0xFFF;
    }
    ds_limit = (ad[ds_i].uLimit0 | (ad[ds_i].uLimit1 << 16));
    if (ad[ds_i].fGranularity) {
      ds_limit <<= 12;
      ds_limit |= 0xFFF;
    }

    vmwrite (cs_i << 3, VMXENC_GUEST_CS_SEL);
    vmwrite (ds_i << 3, VMXENC_GUEST_SS_SEL);
    vmwrite (ds_i << 3, VMXENC_GUEST_DS_SEL);
    vmwrite (ds_i << 3, VMXENC_GUEST_ES_SEL);
    vmwrite (ds_i << 3, VMXENC_GUEST_FS_SEL);
    vmwrite (ds_i << 3, VMXENC_GUEST_GS_SEL);

    vmwrite (cs_base, VMXENC_GUEST_CS_BASE);
    vmwrite (ds_base, VMXENC_GUEST_SS_BASE);
    vmwrite (ds_base, VMXENC_GUEST_DS_BASE);
    vmwrite (ds_base, VMXENC_GUEST_ES_BASE);
    vmwrite (ds_base, VMXENC_GUEST_FS_BASE);
    vmwrite (ds_base, VMXENC_GUEST_GS_BASE);

    vmwrite (cs_limit, VMXENC_GUEST_CS_LIMIT);
    vmwrite (ds_limit, VMXENC_GUEST_SS_LIMIT);
    vmwrite (ds_limit, VMXENC_GUEST_DS_LIMIT);
    vmwrite (ds_limit, VMXENC_GUEST_ES_LIMIT);
    vmwrite (ds_limit, VMXENC_GUEST_FS_LIMIT);
    vmwrite (ds_limit, VMXENC_GUEST_GS_LIMIT);

#if DEBUG_VM86 > 1
    com1_printf ("    CS=%.04X CS.base=%p CS.limit=%p CS.access=%.02X\n"
                 "    DS=%.04X DS.base=%p DS.limit=%p DS.access=%.02X\n",
                 cs_i << 3, cs_base, cs_limit, vmread (VMXENC_GUEST_CS_ACCESS),
                 ds_i << 3, ds_base, ds_limit, vmread (VMXENC_GUEST_DS_ACCESS));
#endif

    return 0;
  } else
    return -1;
}

sint32
vmx_vm86_handle_GPF (virtual_machine *vm)
{
  bool a32 = FALSE, o32 = FALSE, rep = FALSE, repnz = FALSE;
  uint16 ip = vmread (VMXENC_GUEST_RIP) & 0xFFFF;
  uint16 cs = vmread (VMXENC_GUEST_CS_SEL) & 0xFFFF;
  uint16 sp = vmread (VMXENC_GUEST_RSP) & 0xFFFF;
  uint16 ss = vmread (VMXENC_GUEST_SS_SEL) & 0xFFFF;
  uint32 eflags = vmread (VMXENC_GUEST_RFLAGS) & 0xFFFFFFFF;
  uint8 *eip = REAL_TO_LIN32 (cs, ip, uint8);
  uint16 *stk = REAL_TO_LIN32 (ss, sp, uint16);

#if DEBUG_VM86 > 3
  com1_printf ("vmx_vm86_handle_GPF: CS:IP = %.04X:%.04X (I=%0.02X) SS:SP = %.04X:%.04X (ESP=%.08X)\n", cs, ip, eip[0], ss, sp, (uint32) stk);
#endif

 parse:
  switch (eip[0]) {
  case 0xF0:                    /* LOCK prefix */
    break;
  case 0x26:                    /* segment-override-prefix: ES */
  case 0x2E:                    /* segment-override-prefix: CS */
  case 0x36:                    /* segment-override-prefix: SS */
  case 0x3E:                    /* segment-override-prefix: DS */
  case 0x64:                    /* segment-override-prefix: FS */
  case 0x65:                    /* segment-override-prefix: GS */
    break;
  case 0x66:                    /* indicate 32-bit operand */
    o32 = TRUE;
    eip++;
    goto parse;
  case 0x67:
    a32 = TRUE;                 /* indicate 32-bit address */
    eip++;
    goto parse;
  case 0xF2:                    /* REPNZ prefix */
    repnz = TRUE;
    eip++;
    goto parse;
  case 0xF3:                    /* REP prefix */
    rep = TRUE;
    eip++;
    goto parse;
  case 0xCD:                  /* INT instruction */
    {
      uint8 vec = eip[1];
      vm86_farptr dest = vmx_vm86_IVT[vec];
      vm86_farptr new_stk;
#if DEBUG_VM86 > 3
      com1_printf ("  interrupt to vector %.02X farjump to CS:IP = %.04X:%.04X\n",
                   vec, dest.segm, dest.offs);
#endif
      /* Simulate an interrupt dispatch by the CPU in real-mode */
      stk -= 3;                 /* push 3 values on the stack */
      new_stk = LIN32_TO_FP (stk);
      /* Write the new stack pointer to the guest */
      vmwrite (new_stk.offs, VMXENC_GUEST_RSP);
      vmwrite (new_stk.segm, VMXENC_GUEST_SS_SEL);
      vmwrite (new_stk.segm << 4, VMXENC_GUEST_SS_BASE);
      /* Set the IRETurn CS:IP address to the next instruction */
      stk[0] = LIN32_TO_FP (eip + 2).offs;
      stk[1] = LIN32_TO_FP (eip + 2).segm;
      stk[2] = (uint16) eflags;
      /* Write stored flags to memory but change "IF" according to VIF. */
      if (eflags & F_VIF)
        stk[2] |= F_IF;
      else
        stk[2] &= ~F_IF;
#if DEBUG_VM86 > 3
      com1_printf ("  New SS:SP = %.04X:%.04X  IRET CS:IP = %.04X:%.04X FL = %.04X\n",
                   new_stk.segm, new_stk.offs,
                   stk[1], stk[0], stk[2]);
#endif
      /* Clear the virtual IF */
      eflags &= ~F_VIF;
      vmwrite (eflags, VMXENC_GUEST_RFLAGS);
      /* Now, dispatch to dest far pointer */
      vmwrite (dest.offs, VMXENC_GUEST_RIP);
      vmwrite (dest.segm, VMXENC_GUEST_CS_SEL);
      vmwrite (dest.segm << 4, VMXENC_GUEST_CS_BASE);
      return 0;                 /* continue guest */
    }
  case 0x9C:                    /* PUSHF */
    if (o32) {                  /* 32-bit PUSHF */
      stk -= 2;
      *((uint32 *) stk) = eflags & 0xDFF; /* hide the IF flag */
      if (eflags & F_VIF)                 /* set it according to VIF */
        *((uint32 *) stk) |= F_IF;
      else
        *((uint32 *) stk) &= ~F_IF;
    } else {                    /* 16-bit PUSHF */
      stk--;
      *stk = (uint16) eflags & 0xDFF; /* hide the IF flag */
      if (eflags & F_VIF)             /* set it according to VIF */
        *stk |= F_IF;
      else
        *stk &= ~F_IF;
    }
#if DEBUG_VM86 > 3
    com1_printf ("  PUSHF New SS:SP = %.04X:%.04X wrote FL = %.08X\n",
                 LIN32_TO_FP (stk).segm, LIN32_TO_FP (stk).offs,
                 *((uint32 *) stk) & (o32 ? ~0 : 0xFFFF));
#endif
    /* write new CS:IP to guest */
    vmwrite (LIN32_TO_FP (eip + 1).offs, VMXENC_GUEST_RIP);
    vmwrite (LIN32_TO_FP (eip + 1).segm, VMXENC_GUEST_CS_SEL);
    vmwrite ((LIN32_TO_FP (eip + 1).segm) << 4, VMXENC_GUEST_CS_BASE);
    /* write new SS:SP to guest */
    vmwrite (LIN32_TO_FP (stk).offs, VMXENC_GUEST_RSP);
    vmwrite (LIN32_TO_FP (stk).segm, VMXENC_GUEST_SS_SEL);
    vmwrite ((LIN32_TO_FP (stk).segm) << 4, VMXENC_GUEST_SS_BASE);
    return 0;                   /* continue guest */
  case 0x9D:                    /* POPF */
    {
      uint32 new_eflags;
      uint32 f_if = *stk & F_IF;
      if (o32) {                  /* 32-bit POPF */
        /* Basically, allow the guest to control the bits in the mask 0xDFF */
        new_eflags = *((uint32 *) stk) & 0xDFF;
        stk += 2;
      } else {                    /* 16-bit POPF */
        new_eflags = *stk & 0xDFF;
        stk++;
      }
      /* And take the other bit settings from eflags. */
      new_eflags |= (eflags & (~0xDFF));
      /* Set the virtual IF according to what the guest thinks is the IF. */
      if (f_if)
        new_eflags |= F_VIF;
      else
        new_eflags &= ~F_VIF;
      vmwrite (new_eflags, VMXENC_GUEST_RFLAGS);
#if DEBUG_VM86 > 3
      com1_printf ("  POPF New SS:SP = %.04X:%.04X wrote FL = %.08X\n",
                   LIN32_TO_FP (stk).segm, LIN32_TO_FP (stk).offs,
                   new_eflags & (o32 ? ~0 : 0xFFFF));
#endif
      /* Move IP to next instruction */
      inc_ip (eip, 1);
      /* write new SS:SP to guest */
      vmwrite (LIN32_TO_FP (stk).offs, VMXENC_GUEST_RSP);
      vmwrite (LIN32_TO_FP (stk).segm, VMXENC_GUEST_SS_SEL);
      vmwrite ((LIN32_TO_FP (stk).segm) << 4, VMXENC_GUEST_SS_BASE);
      return 0;           /* continue guest */
    }
  case 0xCF:                    /* IRET */
    {
      /* stk[0] = IP; stk[1] = CS; stk[2] = FLAGS */
      uint32 new_eflags = stk[2] & 0xDFF;
      uint32 f_if = stk[2] & F_IF;
      vm86_farptr new_eip = { .segm = stk[1], .offs = stk[0] };

      /* And take the other bit settings from eflags. */
      new_eflags |= (eflags & (~0xDFF));
      /* Set the virtual IF according to what the guest thinks is the IF. */
      if (f_if)
        new_eflags |= F_VIF;
      else
        new_eflags &= ~F_VIF;

      /* write new CS:IP to guest */
      vmwrite (new_eip.offs, VMXENC_GUEST_RIP);
      vmwrite (new_eip.segm, VMXENC_GUEST_CS_SEL);
      vmwrite (new_eip.segm << 4, VMXENC_GUEST_CS_BASE);
      /* write new FLAGS to guest */
      vmwrite (new_eflags, VMXENC_GUEST_RFLAGS);
      /* write new SS:SP to guest */
      stk += 3;
      vmwrite (LIN32_TO_FP (stk).offs, VMXENC_GUEST_RSP);
      vmwrite (LIN32_TO_FP (stk).segm, VMXENC_GUEST_SS_SEL);
      vmwrite ((LIN32_TO_FP (stk).segm) << 4, VMXENC_GUEST_SS_BASE);
#if DEBUG_VM86 > 3
      com1_printf ("  IRET to CS:IP = %.04X:%.04X  SS:SP = %.04X:%.04X  FL = %.08X\n",
                   new_eip.segm, new_eip.offs,
                   LIN32_TO_FP (stk).segm, LIN32_TO_FP (stk).offs,
                   new_eflags);
#endif
      return 0;                 /* continue guest */
    }
  case 0xFA:                    /* CLI */
#if DEBUG_VM86 > 3
    com1_printf ("  CLI\n");
#endif
    eflags &= ~F_VIF;           /* Clear the virtual IF instead */
    /* write new CS:IP to guest */
    vmwrite (LIN32_TO_FP (eip + 1).offs, VMXENC_GUEST_RIP);
    vmwrite (LIN32_TO_FP (eip + 1).segm, VMXENC_GUEST_CS_SEL);
    vmwrite ((LIN32_TO_FP (eip + 1).segm) << 4, VMXENC_GUEST_CS_BASE);
    vmwrite (eflags, VMXENC_GUEST_RFLAGS);
    return 0;                   /* continue guest */
  case 0xFB:                    /* STI */
#if DEBUG_VM86 > 3
    com1_printf ("  STI\n");
#endif
    eflags |= F_VIF;            /* Set the virtual IF instead */
    /* write new CS:IP to guest */
    vmwrite (LIN32_TO_FP (eip + 1).offs, VMXENC_GUEST_RIP);
    vmwrite (LIN32_TO_FP (eip + 1).segm, VMXENC_GUEST_CS_SEL);
    vmwrite ((LIN32_TO_FP (eip + 1).segm) << 4, VMXENC_GUEST_CS_BASE);
    vmwrite (eflags, VMXENC_GUEST_RFLAGS);
    return 0;                   /* continue guest */
  case 0xE4:                    /* inb imm8, %al */
  case 0xE5:                    /* inw imm8, %ax */
  case 0xEC:                    /* inb %dx, %al */
  case 0xED:                    /* inw %dx, %ax */
  case 0x6C:                    /* insb %dx, m8 */
  case 0x6D:                    /* insw %dx, m16 */
    {
      uint16 port;
      uint8 isize = 1;

      if (*eip == 0xE4 || *eip == 0xE5) {
        /* Port specified as imm8 */
        port = eip[1];
        isize++;
      } else
        /* Port specified in %dx register */
        port = vm->guest_regs.edx & 0xFFFF;

      if (o32 &&                /* o32 prefix indicate double-words */
          (*eip == 0xE5 ||      /* inl imm8, %eax */
           *eip == 0xED)) {     /* inl %dx, %eax */
        /* 32-bit value */
        vm->guest_regs.eax = inl(port);
        inc_ip (eip, isize);
#if DEBUG_VM86 > 3
        com1_printf ("  %.08X = inl (%.04X)\n", vm->guest_regs.eax, port);
#endif
        return 0;               /* continue guest */
      } else if (*eip == 0xE5 || *eip == 0xED) {
        /* 16-bit value */
        vm->guest_regs.eax |= 0xFFFF & inw (port);
        inc_ip (eip, isize);
#if DEBUG_VM86 > 3
        com1_printf ("  %.04X = inw (%.04X)\n", vm->guest_regs.eax & 0xFFFF, port);
#endif
        return 0;               /* continue guest */
      } else if (*eip == 0xE4 || *eip == 0xEC) {
        /* 8-bit value */
        vm->guest_regs.eax |= 0xFF & inb (port);
        inc_ip (eip, isize);
#if DEBUG_VM86 > 3
        com1_printf ("  %.02X = inb (%.04X)\n", vm->guest_regs.eax & 0xFF, port);
#endif
        return 0;               /* continue guest */
      } else {
        uint8 *dst;
        uint16 cx = 1;
        /* String port input */

        if (a32) {
          /* 32-bit address */
          dst = (uint8 *) vm->guest_regs.edi; /* Q: Does this need to
                                               * examine the GDT? */
        } else {
          /* 16-bit address */
          dst = REAL_TO_LIN32 (vmread (VMXENC_GUEST_ES_SEL),
                               vm->guest_regs.edi & 0xFFFF,
                               uint8);
        }

        if (rep) {
          cx = vm->guest_regs.ecx & 0xFFFF;
        }

        if (o32 && *eip == 0x6D) { /* insl %dx, m32 */
          /* Transfer double-words */
          if (eflags & F_DF)
            insl_rev (port, dst, cx);
          else
            insl (port, dst, cx);
#if DEBUG_VM86 > 3
          com1_printf ("  insl (%.04X, %p, %.04X) DF=%.01X\n",
                       port, dst, cx, !!(eflags & F_DF));
#endif
        } else if (*eip == 0x6D) { /* insw %dx, m16 */
          /* Transfer words */
          if (eflags & F_DF)
            insw_rev (port, dst, cx);
          else
            insw (port, dst, cx);
#if DEBUG_VM86 > 3
          com1_printf ("  insw (%.04X, %p, %.04X) DF=%.01X\n",
                       port, dst, cx, !!(eflags & F_DF));
#endif

        } else if (*eip == 0x6C) { /* insb %dx, m8 */
          /* Transfer bytes */
          if (eflags & F_DF)
            insb_rev (port, dst, cx);
          else
            insb (port, dst, cx);
#if DEBUG_VM86 > 3
          com1_printf ("  insb (%.04X, %p, %.04X) DF=%.01X\n",
                       port, dst, cx, !!(eflags & F_DF));
#endif
        }

        inc_ip (eip, isize);
        return 0;               /* continue guest */
      }
      break;
    }
  case 0xE6:                    /* outb %al, imm8 */
  case 0xE7:                    /* outw %ax, imm8 */
  case 0xEE:                    /* outb %al, %dx */
  case 0xEF:                    /* outw %ax, %dx */
  case 0x6E:                    /* outsb m8, %dx */
  case 0x6F:                    /* outsw m16, %dx */
    {
      uint16 port;
      uint8 isize = 1;

      if (*eip == 0xE6 || *eip == 0xE7) {
        /* Port specified as imm8 */
        port = eip[1];
        isize++;
      } else
        /* Port specified in %dx register */
        port = vm->guest_regs.edx & 0xFFFF;

      if (o32 &&                /* o32 prefix indicates double-words */
          (*eip == 0xE7 ||      /* outl %eax, imm8 */
           *eip == 0xEF)) {     /* outl %eax, %dx */
        /* 32-bit value */
        outl (vm->guest_regs.eax, port);
        inc_ip (eip, isize);
#if DEBUG_VM86 > 3
        com1_printf ("  outl (%.08X, %.04X)\n", vm->guest_regs.eax, port);
#endif
        return 0;               /* continue guest */
      } else if (*eip == 0xE7 || *eip == 0xEF) {
        /* 16-bit value */
        outw (vm->guest_regs.eax & 0xFFFF, port);
        inc_ip (eip, isize);
#if DEBUG_VM86 > 3
        com1_printf ("  outw (%.04X, %.04X)\n", vm->guest_regs.eax & 0xFFFF, port);
#endif
        return 0;               /* continue guest */
      } else if (*eip == 0xE6 || *eip == 0xEE) {
        /* 8-bit value */
        outb (vm->guest_regs.eax & 0xFF, port);
        inc_ip (eip, isize);
#if DEBUG_VM86 > 3
        com1_printf ("  outb (%.02X, %.04X)\n", vm->guest_regs.eax & 0xFF, port);
#endif
        return 0;               /* continue guest */
      } else {
        uint8 *src;
        uint16 cx = 1;
        /* String port output */

        if (a32) {
          /* 32-bit address */
          src = (uint8 *) vm->guest_regs.esi; /* Q: Does this need to
                                               * examine the GDT? */
        } else {
          /* 16-bit address */
          src = REAL_TO_LIN32 (vmread (VMXENC_GUEST_DS_SEL),
                               vm->guest_regs.esi & 0xFFFF,
                               uint8);
        }

        if (rep) {
          cx = vm->guest_regs.ecx & 0xFFFF;
        }

        if (o32 && *eip == 0x6F) { /* outsl m32, %dx */
          /* Transfer double-words */
          if (eflags & F_DF)
            outsl_rev (port, src, cx);
          else
            outsl (port, src, cx);
#if DEBUG_VM86 > 3
          com1_printf ("  outsl (%.04X, %p, %.04X) DF=%.01X\n",
                       port, src, cx, !!(eflags & F_DF));
#endif
        } else if (*eip == 0x6F) { /* outsw m16, %dx */
          /* Transfer words */
          if (eflags & F_DF)
            outsw_rev (port, src, cx);
          else
            outsw (port, src, cx);
#if DEBUG_VM86 > 3
          com1_printf ("  outsw (%.04X, %p, %.04X) DF=%.01X\n",
                       port, src, cx, !!(eflags & F_DF));
#endif

        } else if (*eip == 0x6E) { /* outsb m8, %dx */
          /* Transfer bytes */
          if (eflags & F_DF)
            outsb_rev (port, src, cx);
          else
            outsb (port, src, cx);
#if DEBUG_VM86 > 3
          com1_printf ("  outsb (%.04X, %p, %.04X) DF=%.01X\n",
                       port, src, cx, !!(eflags & F_DF));
#endif
        }

        inc_ip (eip, isize);
        return 0;               /* continue guest */
      }

      break;
    }
  case 0xF4:                    /* HLT */
#if DEBUG_VM86 > 3
    com1_printf ("  HLT\n");
#endif
    return -1;


  case 0x0F:                    /* ESCAPE to 2-byte opcodes */
    eip++;
    switch (*eip) {
    case 0x01:                  /* LGDT / LIDT */
      {
        uint8 reg_op, disp_size, rm;
        uint16 limit;
        uint32 disp, base;
        bool has_sib;
        eip++;
        decode_modrm_32 (eip, &reg_op, &has_sib, &rm, &disp_size, &disp);
        if (reg_op == 2 &&      /* LGDT */
            a32 && o32) {
          /* example: 67660F011530910000 o32 lgdt [dword 0x9130]  */
          if (rm == 5) {
            /* disp is the address of the gdt ptr */
            limit = *((uint16 *) disp);
            base = *((uint32 *) (disp + 2));
          } else {
            limit = 0;
            base = 0;
          }
#if DEBUG_VM86 > 1
          com1_printf ("  LGDT base=%.08X limit=%.04X\n", base, limit);
#endif
          vmwrite (base, VMXENC_GUEST_GDTR_BASE);
          vmwrite (limit, VMXENC_GUEST_GDTR_LIMIT);
          inc_ip (eip, 1 + (has_sib ? 1 : 0) + disp_size);
          return 0;               /* continue guest */
        } else if (reg_op == 3) { /* LIDT */
#if DEBUG_VM86 > 1
          com1_printf ("  LIDT\n");
#endif
        }
        break;
      }
#define VM86_CR0_MASK (~0x80000001)
    case 0x20:                  /* MOVL %CR0, reg */
      {
        uint8 reg_op, disp_size, rm;
        uint32 disp;
        bool has_sib;
        eip++;
        decode_modrm_32 (eip, &reg_op, &has_sib, &rm, &disp_size, &disp);
#if DEBUG_VM86 > 1
        com1_printf ("  MOVL %%CR0, %%%s  CR0=%.08X\n",
                     VM_REG_NAME (rm), vmread (VMXENC_GUEST_CR0) & (VM86_CR0_MASK));
#endif
        VM_REG (rm) = (vmread (VMXENC_GUEST_CR0) & VM86_CR0_MASK);
        inc_ip (eip, 1);
        return 0;               /* continue guest */
      }
    case 0x22:                  /* MOVL reg, %CR0 */
      {
        uint8 reg_op, disp_size, rm;
        uint32 disp, new_cr0;
        bool has_sib;

        eip++;
        decode_modrm_32 (eip, &reg_op, &has_sib, &rm, &disp_size, &disp);
        new_cr0 = VM_REG (rm);
        if (new_cr0 & 0x1) {
          /* Guest has decided to enable P-mode, therefore we must
           * disable our virtual real-mode emulation. */
          vm->realmode = FALSE;
          vmwrite (vmread (VMXENC_GUEST_RFLAGS) & (~F_VM), VMXENC_GUEST_RFLAGS);
          if (pick_segment_regs () < 0) {
#if DEBUG_VM86 > 1
            com1_printf ("  failed to pick segment registers.\n");
#endif
            return -1;
          }

          /* Technically it doesn't have to be this way, but let's
           * assume for now that a FAR JUMP immediately follows any
           * enabling of P-mode. */
          eip++;
          if (eip[0] == 0x66 && eip[1] == 0xEA) {
            uint32 offs = *((uint32 *) (eip + 2));
            uint16 segm = *((uint16 *) (eip + 6));
            descriptor *ad = (descriptor *) vmread (VMXENC_GUEST_GDTR_BASE);
            uint32 cs_access = ACCESS (ad[segm >> 3]);
            uint32 cs_base =
              ad[segm >> 3].pBase0 |
              (ad[segm >> 3].pBase1 << 8) |
              (ad[segm >> 3].pBase2 << 16);
            uint32 cs_limit = (ad[segm >> 3].uLimit0 | (ad[segm >> 3].uLimit1 << 16));
            if (ad[segm >> 3].fGranularity) {
              cs_limit <<= 12;
              cs_limit |= 0xFFF;
            }
            /* Jump to new offset and set CS. */
            vmwrite (offs, VMXENC_GUEST_RIP);
            vmwrite (segm, VMXENC_GUEST_CS_SEL);
            vmwrite (cs_access, VMXENC_GUEST_CS_ACCESS);
            vmwrite (cs_base, VMXENC_GUEST_CS_BASE);
            vmwrite (cs_limit, VMXENC_GUEST_CS_LIMIT);
#if DEBUG_VM86 > 1
            com1_printf ("  P-mode enabled: FAR JUMP to %.04X:%p"
                         " access=%.02X base=%p limit=%p\n",
                         segm, offs, cs_access, cs_base, cs_limit);
#endif
          } else {
#if DEBUG_VM86 > 1
            com1_printf ("  invalid P-mode toggle: not followed by FAR JUMP.\n");
#endif
            return -1;
          }

        } else
          inc_ip (eip, 1);
        new_cr0 |= ~VM86_CR0_MASK;
        vmwrite (new_cr0, VMXENC_GUEST_CR0);
#if DEBUG_VM86 > 1
        com1_printf ("  MOVL %%%s, %%CR0  new CR0=%.08X CS=%.04X GDTR=%.08X:%.04X\n",
                     VM_REG_NAME (rm), new_cr0, vmread (VMXENC_GUEST_CS_SEL),
                     vmread (VMXENC_GUEST_GDTR_BASE), vmread (VMXENC_GUEST_GDTR_LIMIT));
#endif

        return 0;               /* continue guest */
      }
    }

  default:
#if DEBUG_VM86 > 1
    com1_printf ("  Unknown opcode %.02X\n", eip[0]);
#endif
    break;
  }

  return -1;
}
#undef ACCESS

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
