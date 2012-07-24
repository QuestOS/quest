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

#ifndef _VMX_DEFS_H_
#define _VMX_DEFS_H_

/* See Intel System Programming manual appendix H */


/* 16-bit Control Field */
#define VMXENC_VPID                           0x00000000


/* 16-bit Guest State Fields */
#define VMXENC_GUEST_ES_SEL                   0x00000800
#define VMXENC_GUEST_CS_SEL                   0x00000802
#define VMXENC_GUEST_SS_SEL                   0x00000804
#define VMXENC_GUEST_DS_SEL                   0x00000806
#define VMXENC_GUEST_FS_SEL                   0x00000808
#define VMXENC_GUEST_GS_SEL                   0x0000080A
#define VMXENC_GUEST_LDTR_SEL                 0x0000080C
#define VMXENC_GUEST_TR_SEL                   0x0000080E


/* 16-bit Host State Fields */
#define VMXENC_HOST_ES_SEL                    0x00000C00
#define VMXENC_HOST_CS_SEL                    0x00000C02
#define VMXENC_HOST_SS_SEL                    0x00000C04
#define VMXENC_HOST_DS_SEL                    0x00000C06
#define VMXENC_HOST_FS_SEL                    0x00000C08
#define VMXENC_HOST_GS_SEL                    0x00000C0A
#define VMXENC_HOST_TR_SEL                    0x00000C0C


/* 64-bit Control Fields */

/* Address of I/O bitmap A (full) */
#define VMXENC_IO_BITMAP_A                    0x00002000

/* Address of I/O bitmap A (high) */
#define VMXENC_IO_BITMAP_A_HI                 0x00002001

/* Address of I/O bitmap B (full) */
#define VMXENC_IO_BITMAP_B                    0x00002002

/* Address of I/O bitmap B (high) */
#define VMXENC_IO_BITMAP_B_HI                 0x00002003

/* Address of MSR bitmaps (full) */
#define VMXENC_MSR_BITMAPS                    0x00002004

/* Address of MSR bitmaps (high) */
#define VMXENC_MSR_BITMAPS_HI                 0x00002005

/* VM-exit MSR-store address (full) */
#define VMXENC_VM_EXIT_MSR_STORE_ADDR         0x00002006

/* VM-exit MSR-store address (high) */
#define VMXENC_VM_EXIT_MSR_STORE_ADDR_HI      0x00002007

/* VM-exit MSR-load address (full) */
#define VMXENC_VM_EXIT_MSR_LOAD_ADDR          0x00002008

/* VM-exit MSR-load address (high) */
#define VMXENC_VM_EXIT_MSR_LOAD_ADDR_HI       0x00002009

/* VM-entry MSR-load address (full) */
#define VMXENC_VM_ENTRY_MSR_LOAD_ADDR         0x0000200A

/* VM-entry MSR-load address (high) */
#define VMXENC_VM_ENTRY_MSR_LOAD_ADDR_HI      0x0000200B

/* Executive-VMCS pointer (full) */
#define VMXENC_EXECUTIVE_VMCS_PTR             0x0000200C

/* Executive-VMCS pointer (high) */
#define VMXENC_EXECUTIVE_VMCS_PTR_HI          0x0000200D

/* TSC offset (full) */
#define VMXENC_TSC_OFFSET                     0x00002010

/* TSC offset (high) */
#define VMXENC_TSC_OFFSET_HI                  0x00002011

/* Virtual-APIC address (full) */
#define VMXENC_VIRTUAL_APIC_ADDR              0x00002012

/* Virtual-APIC address (high) */
#define VMXENC_VIRTUAL_APIC_ADDR_HI           0x00002013

/* APIC-access address (full)3 */
#define VMXENC_APIC_ACCESS_ADDR               0x00002014

/* APIC-access address  (high) */
#define VMXENC_APIC_ACCESS_ADDR_HI            0x00002015

/* EPT pointer (EPTP; full) */
#define VMXENC_EPT_PTR                        0x0000201A

/* EPT pointer (EPTP; high) */
#define VMXENC_EPT_PTR_HI                     0x0000201B


/* 64-bit Read Only Data fields */
#define VMXENC_GUEST_PHYS_ADDR                0x00002400
#define VMXENC_GUEST_PHYS_ADDR_HI             0x00002401


/* 64-bit Guest State fields */

/* VMCS link pointer (full) */
#define VMXENC_VMCS_LINK_PTR                  0x00002800

/* VMCS link pointer (high) */
#define VMXENC_VMCS_LINK_PTR_HI               0x00002801

/* Guest IA32_DEBUGCTL (full) */
#define VMXENC_GUEST_IA32_DEBUGCTL            0x00002802

/* Guest IA32_DEBUGCTL (high) */
#define VMXENC_GUEST_IA32_DEBUGCTL_HI         0x00002803

/* Guest IA32_PAT (full) */
#define VMXENC_GUEST_IA32_PAT                 0x00002804

/* Guest IA32_PAT (high) */
#define VMXENC_GUEST_IA32_PAT_HI              0x00002805

/* Guest IA32_EFER (full) */
#define VMXENC_GUEST_IA32_EFER                0x00002806

/* Guest IA32_EFER (high) */
#define VMXENC_GUEST_IA32_EFER_HI             0x00002807

/* Guest IA32_PERF_GLOBAL_CTRL (full) */
#define VMXENC_GUEST_IA32_GLOBAL_CTRL         0x00002808

/* Guest IA32_PERF_GLOBAL_CTRL (high) */
#define VMXENC_GUEST_IA32_GLOBAL_CTRL_HI      0x00002809

/* Guest PDPTE0 (full) */
#define VMXENC_GUEST_PDPTE0                   0x0000280A

/* Guest PDPTE0 (high) */
#define VMXENC_GUEST_PDPTE0_HI                0x0000280B

/* Guest PDPTE1 (full) */
#define VMXENC_GUEST_PDPTE1                   0x0000280C

/* Guest PDPTE1 (high) */
#define VMXENC_GUEST_PDPTE1_HI                0x0000280D

/* Guest PDPTE2 (full) */
#define VMXENC_GUEST_PDPTE2                   0x0000280E

/* Guest PDPTE2 (high) */
#define VMXENC_GUEST_PDPTE2_HI                0x0000280F

/* Guest PDPTE3 (full) */
#define VMXENC_GUEST_PDPTE3                   0x00002810

/* Guest PDPTE3 (high) */
#define VMXENC_GUEST_PDPTE3_HI                0x00002811


/* 64-bit Host State fields */

/* Host IA32_PAT (full) */
#define VMXENC_HOST_IA32_PAT                  0x00002C00

/* Host IA32_PAT (high) */
#define VMXENC_HOST_IA32_PAT_HI               0x00002C01

/* Host IA32_EFER (full) */
#define VMXENC_HOST_IA32_EFER                 0x00002C02

/* Host IA32_EFER (high) */
#define VMXENC_HOST_IA32_EFER_HI              0x00002C03

/* Host IA32_PERF_GLOBAL_CTRL (full) */
#define VMXENC_HOST_IA32_GLOBAL_CTRL          0x00002C04

/* Host IA32_PERF_GLOBAL_CTRL (high) */


/* 32-bit Control fields */

#define VMXENC_HOST_IA32_GLOBAL_CTRL_HI       0x00002C05

/* Pin-based VM-execution controls */
#define VMXENC_PINBASED_VM_EXEC_CTRLS         0x00004000

/* Primary processor-based VM-execution controls */
#define VMXENC_PROCBASED_VM_EXEC_CTRLS        0x00004002

/* Exception bitmap */
#define VMXENC_EXCEPTION_BITMAP               0x00004004

/* Page-fault error-code mask */
#define VMXENC_PAGE_FAULT_ERRCODE_MASK        0x00004006

/* Page-fault error-code match */
#define VMXENC_PAGE_FAULT_ERRCODE_MATCH       0x00004008

/* CR3-target count */
#define VMXENC_CR3_TARGET_COUNT               0x0000400A

/* VM-exit controls */
#define VMXENC_VM_EXIT_CTRLS                  0x0000400C

/* VM-exit MSR-store count */
#define VMXENC_VM_EXIT_MSR_STORE_COUNT        0x0000400E

/* VM-exit MSR-load count */
#define VMXENC_VM_EXIT_MSR_LOAD_COUNT         0x00004010

/* VM-entry controls */
#define VMXENC_VM_ENTRY_CTRLS                 0x00004012

/* VM-entry MSR-load count */
#define VMXENC_VM_ENTRY_MSR_LOAD_COUNT        0x00004014

/* VM-entry interruption-information field */
#define VMXENC_VM_ENTRY_INTERRUPT_INFO        0x00004016

/* VM-entry exception error code */
#define VMXENC_VM_ENTRY_EXCEPTION_ERRCODE     0x00004018

/* VM-entry instruction length */
#define VMXENC_VM_ENTRY_INSTR_LEN             0x0000401A

/* TPR threshold */
#define VMXENC_TPR_THRESHOLD                  0x0000401C

/* Secondary processor-based VM-execution controls */
#define VMXENC_PROCBASED_VM_EXEC_CTRLS2       0x0000401E


/* 32-bit Read Only Data fields */

/* VM-instruction error */
#define VMXENC_VM_INSTR_ERROR                 0x00004400

/* Exit reason */
#define VMXENC_EXIT_REASON                    0x00004402

/* VM-exit interruption information */
#define VMXENC_VM_EXIT_INTERRUPT_INFO         0x00004404

/* VM-exit interruption error code */
#define VMXENC_VM_EXIT_INTERRUPT_ERRCODE      0x00004406

/* IDT-vectoring information field */
#define VMXENC_IDT_VECTORING_INFO             0x00004408

/* IDT-vectoring error code */
#define VMXENC_IDT_VECTORING_ERRCODE          0x0000440A

/* VM-exit instruction length */
#define VMXENC_VM_EXIT_INSTR_LEN              0x0000440C

/* VM-exit instruction information */
#define VMXENC_VM_EXIT_INSTR_INFO             0x0000440E


/* 32-bit Guest State fields */

/* Guest ES limit */
#define VMXENC_GUEST_ES_LIMIT                 0x00004800

/* Guest CS limit */
#define VMXENC_GUEST_CS_LIMIT                 0x00004802

/* Guest SS limit */
#define VMXENC_GUEST_SS_LIMIT                 0x00004804

/* Guest DS limit */
#define VMXENC_GUEST_DS_LIMIT                 0x00004806

/* Guest FS limit */
#define VMXENC_GUEST_FS_LIMIT                 0x00004808

/* Guest GS limit */
#define VMXENC_GUEST_GS_LIMIT                 0x0000480A

/* Guest LDTR limit */
#define VMXENC_GUEST_LDTR_LIMIT               0x0000480C

/* Guest TR limit */
#define VMXENC_GUEST_TR_LIMIT                 0x0000480E

/* Guest GDTR limit */
#define VMXENC_GUEST_GDTR_LIMIT               0x00004810

/* Guest IDTR limit */
#define VMXENC_GUEST_IDTR_LIMIT               0x00004812

/* Guest ES access rights */
#define VMXENC_GUEST_ES_ACCESS                0x00004814

/* Guest CS access rights */
#define VMXENC_GUEST_CS_ACCESS                0x00004816

/* Guest SS access rights */
#define VMXENC_GUEST_SS_ACCESS                0x00004818

/* Guest DS access rights */
#define VMXENC_GUEST_DS_ACCESS                0x0000481A

/* Guest FS access rights */
#define VMXENC_GUEST_FS_ACCESS                0x0000481C

/* Guest GS access rights */
#define VMXENC_GUEST_GS_ACCESS                0x0000481E

/* Guest LDTR access rights */
#define VMXENC_GUEST_LDTR_ACCESS              0x00004820

/* Guest TR access rights */
#define VMXENC_GUEST_TR_ACCESS                0x00004822

/* Guest interruptibility state */
#define VMXENC_GUEST_INTERRUPTIBILITY         0x00004824

/* Guest activity state */
#define VMXENC_GUEST_ACTIVITY                 0x00004826

/* Guest SMBASE */
#define VMXENC_GUEST_SMBASE                   0x00004828

/* Guest IA32_SYSENTER_CS */
#define VMXENC_GUEST_IA32_SYSENTER_CS         0x0000482A

/* VMX-preemption timer value */
#define VMXENC_VMX_PREEMPT_TIMER_VAL          0x0000482E


/* 32-bit Host State field */

/* Host IA32_SYSENTER_CS */
#define VMXENC_HOST_IA32_SYSENTER_CS          0x00004C00


/* Natural-width Control fields */

/* CR0 guest/host mask */
#define VMXENC_CR0_GUEST_HOST_MASK            0x00006000

/* CR4 guest/host mask */
#define VMXENC_CR4_GUEST_HOST_MASK            0x00006002

/* CR0 read shadow */
#define VMXENC_CR0_READ_SHADOW                0x00006004

/* CR4 read shadow */
#define VMXENC_CR4_READ_SHADOW                0x00006006

/* CR3-target value 0 */
#define VMXENC_CR3_TARGET_VAL0                0x00006008

/* CR3-target value 1 */
#define VMXENC_CR3_TARGET_VAL1                0x0000600A

/* CR3-target value 2 */
#define VMXENC_CR3_TARGET_VAL2                0x0000600C

/* CR3-target value 3 */
#define VMXENC_CR3_TARGET_VAL3                0x0000600E


/* Natural-width Read Only Data fields */

/* Exit qualification */
#define VMXENC_EXIT_QUAL                      0x00006400

/* I/O RCX */
#define VMXENC_IO_RCX                         0x00006402

/* I/O RSI */
#define VMXENC_IO_RSI                         0x00006404

/* I/O RDI */
#define VMXENC_IO_RDI                         0x00006406

/* I/O RIP */
#define VMXENC_IO_RIP                         0x00006408

/* Guest-linear address */
#define VMXENC_GUEST_LINEAR_ADDR              0x0000640A


/* Natural-width Guest State fields */

/* Guest CR0 */
#define VMXENC_GUEST_CR0                      0x00006800

/* Guest CR3 */
#define VMXENC_GUEST_CR3                      0x00006802

/* Guest CR4 */
#define VMXENC_GUEST_CR4                      0x00006804

/* Guest ES base */
#define VMXENC_GUEST_ES_BASE                  0x00006806

/* Guest CS base */
#define VMXENC_GUEST_CS_BASE                  0x00006808

/* Guest SS base */
#define VMXENC_GUEST_SS_BASE                  0x0000680A

/* Guest DS base */
#define VMXENC_GUEST_DS_BASE                  0x0000680C

/* Guest FS base */
#define VMXENC_GUEST_FS_BASE                  0x0000680E

/* Guest GS base */
#define VMXENC_GUEST_GS_BASE                  0x00006810

/* Guest LDTR base */
#define VMXENC_GUEST_LDTR_BASE                0x00006812

/* Guest TR base */
#define VMXENC_GUEST_TR_BASE                  0x00006814

/* Guest GDTR base */
#define VMXENC_GUEST_GDTR_BASE                0x00006816

/* Guest IDTR base */
#define VMXENC_GUEST_IDTR_BASE                0x00006818

/* Guest DR7 */
#define VMXENC_GUEST_DR7                      0x0000681A

/* Guest RSP */
#define VMXENC_GUEST_RSP                      0x0000681C

/* Guest RIP */
#define VMXENC_GUEST_RIP                      0x0000681E

/* Guest RFLAGS */
#define VMXENC_GUEST_RFLAGS                   0x00006820

/* Guest pending debug exceptions */
#define VMXENC_GUEST_PENDING_DEBUG_EXCEPTIONS 0x00006822

/* Guest IA32_SYSENTER_ESP */
#define VMXENC_GUEST_IA32_SYSENTER_ESP        0x00006824

/* Guest IA32_SYSENTER_EIP */
#define VMXENC_GUEST_IA32_SYSENTER_EIP        0x00006826


/* Natural-width Host State fields */

/* Host CR0 */
#define VMXENC_HOST_CR0                       0x00006C00

/* Host CR3 */
#define VMXENC_HOST_CR3                       0x00006C02

/* Host CR4 */
#define VMXENC_HOST_CR4                       0x00006C04

/* Host FS base */
#define VMXENC_HOST_FS_BASE                   0x00006C06

/* Host GS base */
#define VMXENC_HOST_GS_BASE                   0x00006C08

/* Host TR base */
#define VMXENC_HOST_TR_BASE                   0x00006C0A

/* Host GDTR base */
#define VMXENC_HOST_GDTR_BASE                 0x00006C0C

/* Host IDTR base */
#define VMXENC_HOST_IDTR_BASE                 0x00006C0E

/* Host IA32_SYSENTER_ESP */
#define VMXENC_HOST_IA32_SYSENTER_ESP         0x00006C10

/* Host IA32_SYSENTER_EIP */
#define VMXENC_HOST_IA32_SYSENTER_EIP         0x00006C12

/* Host RSP */
#define VMXENC_HOST_RSP                       0x00006C14

/* Host RIP */
#define VMXENC_HOST_RIP                       0x00006C16


#endif

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
