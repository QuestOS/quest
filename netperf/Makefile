include ../config.mk
ifndef INSTALL_DIR
INSTALL_DIR=/tftp/boot
endif

PROGS = netperf netserver

LIBS   = -lc -lm -lsoftfloat $(LIB_DEST)/crt0.o
INCS   = -I./mk_netperf_on_quest
CFLAGS =  -g -Wall -O0 -MMD -msoft-float $(INCS)

# These are needed for both netperf and netserver
OBJS = netlib.o netsh.o nettest_bsd.o nettest_dlpi.o nettest_unix.o nettest_xti.o \
       nettest_sctp.o nettest_sdp.o nettest_omni.o net_uuid.o netcpu_procstat.o netrt_rtnetlink.o \
       netdrv_ethtool.o netslot_linux.o netsys_linux.o netsec_linux.o mk_netperf_on_quest/our_func.o

.PHONY: all clean install

all: $(PROGS)

netperf:  netperf.o $(OBJS)
	$(LD) -o netperf netperf.o $(OBJS) $(LIBS)

netserver:  netserver.o $(OBJS)
	$(LD) -o netserver netserver.o $(OBJS) $(LIBS)

%.o: %.c
	$(CC)  $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o *.d *~ netperf netserver
	cd mk_netperf_on_quest/; rm -f *.o *~ *.d *.map


install: $(PROGS)
	$(INSTALL_CMD) $(PROGS) $(INSTALL_DIR)
