# Kernel configuration

# Compiler optimization level
OPT = 0

# Disable SMP
# CFG += -DNO_SMP

# Disable "logger" thread
CFG += -DNO_LOGGER

# Disable ACPI support
# CFG += -DNO_ACPI

# Disable Intel Multiprocessor Specification parsing
CFG += -DNO_INTEL_MPS

# Use VMX-based virtual machines for isolation
# CFG += -DUSE_VMX

# Use PL2303 usb-serial converter for serial output
# CFG += -DUSE_PL2303

# Share UHCI USB host controller driver
#CFG += -DSHARED_UHCI

# USB provides real-time guarantees for bulk and control transactions
#CFG += -DUSB_REALTIME_ASYNC 

# Use USB Migration for cross machine migration
#CFG += -DUSB_MIGRATION

# Change to use different allocator (default: tlsf)
KMALLOC = tlsf
#KMALLOC = pow2

# Read the entire file into memory on the call to open, makes
# subsequent read calls much faster but must store the entire file in
# the kernel heap
#CFG += -DONE_FILE_READ

# Enable Linux sandbox
# CFG += -DUSE_LINUX_SANDBOX

# Enable Quest-V multi-kernel without VMX enabled
# CFG += -DQUESTV_NO_VMX

# Enable Serial MMIO32 driver
# This will disable port based RS232
# Enable this flag for embedded platforms such as Galileo board
# CFG += -DSERIAL_MMIO32

# Minnowboard Max
CFG += -DMINNOWMAX

# Galileo
# CFG += -DGALILEO

# Disable FPU
CFG += -DNO_FPU

# No Pololu
CFG += -DNO_POLOLU

# No CY8C9540A
CFG += -DNO_CY8C9540A

# Disable attachInterrupt IOVCPU
# CFG += -DNO_GPIO_IOVCPU
