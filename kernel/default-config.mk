# Kernel configuration

# Compiler optimization level
OPT = 0

# Disable SMP
# CFG += -DNO_SMP

# Disable "logger" thread
# CFG += -DNO_LOGGER

# Disable ACPI support
# CFG += -DNO_ACPI

# Disable Intel Multiprocessor Specification parsing
# CFG += -DNO_INTEL_MPS

# Use VMX-based virtual machines for isolation
# CFG += -DUSE_VMX

# Use PL2303 usb-serial converter for serial output
#CFG += -DUSE_PL2303

# Share UHCI USB host controller driver
#CFG += -DSHARED_UHCI

# USB provides real-time guarantees for bulk and control transactions
CFG += -DUSB_REALTIME_ASYNC 

# Use USB Migration for cross machine migration
CFG += -DUSB_MIGRATION
