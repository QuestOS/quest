# *** Choose your compiler. Make it ANSI-compatible (e.g. 'cc -ansi').
CC	= gcc
LD	= ld -melf_i386 -T../../kernel/ldscripts/module.ld


# *** architecture.

# quest

LIBS   = -L../../libc/ -L../../libc/include -L../../softfloat/ -lc -lsoftfloat
INCS   = -I../../libc/include -I../../quest/include -I./mk_netperf_on_quest
CFLAGS = -m32 -march=i586 -fno-builtin -fno-stack-protector -fno-strict-aliasing \
         -fno-delete-null-pointer-checks -nostdinc -g -Wall -Wno-attributes -O0 \
         -D_QUEST -Wa,--32 -MMD -msoft-float $(INCS) $(LIBS)

# These are all the objs needed for netperf and netserver
#OBJS   = netcpu_procstat.o netperf.o netserver.o netsys_linux.o nettest_omni.o nettest_unix.o \
#         netdrv_ethtool.o netrt_rtnetlink.o netsh.o nettest_bsd.o nettest_sctp.o nettest_xti.o \
#         netlib.o netsec_linux.o netslot_linux.o nettest_dlpi.o nettest_sdp.o net_uuid.o


# These are needed for both netperf and netserver
OBJS = netlib.o netsh.o nettest_bsd.o nettest_dlpi.o nettest_unix.o nettest_xti.o \
       nettest_sctp.o nettest_sdp.o nettest_omni.o net_uuid.o netcpu_procstat.o netrt_rtnetlink.o \
       netdrv_ethtool.o netslot_linux.o netsys_linux.o netsec_linux.o mk_netperf_on_quest/our_func.o



#mame:  $(OBJS) /../libc/libc.a
#	$(LD) -o mame $(OBJS) $(LIBS.$(ARCH))
#obj/%.o: src/%.c
#	 $(CC) $(DEFS.$(ARCH)) $(MS) $(CFLAGS) -O3 -o $@ -c $<

all: netperf netserver

netperf:  netperf.o $(OBJS) ../../libc/libc.a ../../softfloat/libsoftfloat.a
	$(LD) -o netperf netperf.o $(OBJS) $(LIBS)

netserver:  netserver.o $(OBJS) ../../libc/libc.a ../../softfloat/libsoftfloat.a
	$(LD) -o netserver netserver.o $(OBJS) $(LIBS)

%.o: %.c
	$(CC)  $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o *.d *~ netperf netserver
	cd mk_netperf_on_quest/; rm -f *.o *~
	cd ../../libc; $(MAKE) clean
	cd ../../softfloat; $(MAKE) clean

../../libc/libc.a:
	cd ../../libc; $(MAKE)

../../softfloat/libsoftfloat.a:
	cd ../../softfloat; $(MAKE)

#install:
#	cp netperf /scratch/tftp/boot/
#	sync
#	quest_turn_on


on:
	quest_turn_on

off:
	quest_turn_off
