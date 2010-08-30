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

# Scheduler selection:
#   sprr    Static Priority Round Robin
#   mpq     Multiple Processor Queues
#   vcpu    Virtual CPU
#   vcpu_rr Virtual CPU Round-Robin
CFG += -DQUEST_SCHED=sprr
