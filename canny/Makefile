# *** Choose your compiler. Make it ANSI-compatible (e.g. 'cc -ansi').
CC	= gcc
LD	= ld -melf_i386 -T../kernel/ldscripts/module.ld


# *** architecture.

ifndef MNT_POINT
MNT_POINT=/tftp
endif

# quest

LIBS   = -L../libc/ -L../libc/include -L../softfloat/ -lc -lsoftfloat
INCS   = -I../libc/include -I../quest/include -I.
CFLAGS = -m32 -march=i586 -fno-builtin -fno-stack-protector -fno-strict-aliasing \
         -fno-delete-null-pointer-checks -nostdinc -g -Wall -Wno-attributes -O0 \
         -D_QUEST -Wa,--32 -MMD -msoft-float $(INCS) $(LIBS)

OBJS = canny.o test.o

PROGS = canny

#mame:  $(OBJS) /../libc/libc.a
#	$(LD) -o mame $(OBJS) $(LIBS.$(ARCH))
#obj/%.o: src/%.c
#	 $(CC) $(DEFS.$(ARCH)) $(MS) $(CFLAGS) -O3 -o $@ -c $<

all:  $(PROGS)

canny: $(OBJS)
	$(LD) -o canny $(OBJS) $(LIBS)


%.o: %.c
	$(CC)  $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o *.d *~ canny

install: $(PROGS)
	cp $(PROGS) $(MNT_POINT)/boot
	sync
