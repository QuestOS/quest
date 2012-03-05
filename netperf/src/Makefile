# *** Choose your compiler. Make it ANSI-compatible (e.g. 'cc -ansi').
CC	= gcc
LD	= ld -melf_i386 -T../kernel/ldscripts/module.ld


# *** architecture.

# quest

LIBS   = -L../../libc/include -lc
INCS   = -I../../libc/include -I../../quest/include -I./mk_netperf_on_quest
CFLAGS = -m32 -march=i586 -fno-builtin -fno-stack-protector -fno-strict-aliasing \
         -fno-delete-null-pointer-checks -nostdinc -g -Wall -Wno-attributes -O0 \
         -D_QUEST -Wa,--32 -MMD $(INCS) $(LIBS)

# These are all the objs needed for netperf and netserver
#OBJS   = netcpu_procstat.o netperf.o netserver.o netsys_linux.o nettest_omni.o nettest_unix.o \
#         netdrv_ethtool.o netrt_rtnetlink.o netsh.o nettest_bsd.o nettest_sctp.o nettest_xti.o \
#         netlib.o netsec_linux.o netslot_linux.o nettest_dlpi.o nettest_sdp.o net_uuid.o


# These are only the ones needed for netperf
OBJS = netperf.o netlib.o netsh.o nettest_bsd.o nettest_dlpi.o nettest_unix.o nettest_xti.o \
       nettest_sctp.o nettest_sdp.o nettest_omni.o net_uuid.o netcpu_procstat.o netrt_rtnetlink.o \
       netdrv_ethtool.o netslot_linux.o netsys_linux.o netsec_linux.o



#mame:  $(OBJS) ../libc/libc.a
#	$(LD) -o mame $(OBJS) $(LIBS.$(ARCH))
#obj/%.o: src/%.c
#	 $(CC) $(DEFS.$(ARCH)) $(MS) $(CFLAGS) -O3 -o $@ -c $<

all: netperf

netperf:  $(OBJS) ../libc/libc.a
	$(LD) -o netperf $(OBJS) $(LIBS)

%.o: %.c
	$(CC)  $(CFLAGS) -o $@ -c $<