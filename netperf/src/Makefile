# *** Choose your compiler. Make it ANSI-compatible (e.g. 'cc -ansi').
CC	= gcc
LD	= ld -melf_i386 -T../../kernel/ldscripts/module.ld


# *** architecture.

ifndef MNT_POINT
MNT_POINT=/tftp
endif

# quest

PROGS = netperf netserver

LIBS   = -L../../libc/ -L../../libc/include -L../../softfloat/ -lc -lsoftfloat
INCS   = -I../../libc/include -I../../quest/include -I./mk_netperf_on_quest
CFLAGS = -m32 -march=i586 -fno-builtin -fno-stack-protector -fno-strict-aliasing \
         -fno-delete-null-pointer-checks -nostdinc -g -Wall -Wno-attributes -O0 \
         -D_QUEST -Wa,--32 -MMD -msoft-float $(INCS) $(LIBS)

# These are needed for both netperf and netserver
OBJS = netlib.o netsh.o nettest_bsd.o nettest_dlpi.o nettest_unix.o nettest_xti.o \
       nettest_sctp.o nettest_sdp.o nettest_omni.o net_uuid.o netcpu_procstat.o netrt_rtnetlink.o \
       netdrv_ethtool.o netslot_linux.o netsys_linux.o netsec_linux.o mk_netperf_on_quest/our_func.o


.PHONY: all clean install


all: $(PROGS)

netperf:  netperf.o $(OBJS) ../../libc/libc.a ../../softfloat/libsoftfloat.a
	$(LD) -o netperf netperf.o $(OBJS) $(LIBS)

netserver:  netserver.o $(OBJS) ../../libc/libc.a ../../softfloat/libsoftfloat.a
	$(LD) -o netserver netserver.o $(OBJS) $(LIBS)

%.o: %.c
	$(CC)  $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o *.d *~ netperf netserver
	cd mk_netperf_on_quest/; rm -f *.o *~ *.d *.map


install: $(PROGS)
	cp $(PROGS) $(MNT_POINT)/boot
	sync
