/***************************************************************************

  cpuintrf.c

  Don't you love MS-DOS 8+3 names? That stands for CPU interface.
  Functions needed to interface the CPU emulator with the other parts of
  the emulation.

***************************************************************************/

#include "driver.h"
#include "Z80.h"
/* I can't include M6502.h because it redefines some types. Here are the things I need: */
/* Interrupt6502() returns:   */
#define INT_NONE  0            /* No interrupt required      */
#define INT_IRQ	  1            /* Standard IRQ interrupt     */
#define INT_NMI	  2            /* Non-maskable interrupt     */


static int activecpu;

static const struct MemoryReadAddress *memoryread;
static const struct MemoryWriteAddress *memorywrite;



struct z80context
{
  Z80_Regs regs;
  int icount;
  int iperiod;
  int irq;
};



void cpu_run(void)
{
  int totalcpu,usres;
  unsigned char cpucontext[MAX_CPU][100];	/* enough to accomodate the cpu status */
  unsigned char *ROM0;	/* opcode decryption is currently supported only for the first memory region */


  ROM0 = ROM;

  /* count how many CPUs we have to emulate */
  totalcpu = 0;
  while (totalcpu < MAX_CPU && Machine->drv->cpu[totalcpu].cpu_type != 0) totalcpu++;

 reset:
  for (activecpu = 0;activecpu < totalcpu;activecpu++)
    {
      int cycles;


      cycles = Machine->drv->cpu[activecpu].cpu_clock /
	(Machine->drv->frames_per_second * Machine->drv->cpu[activecpu].interrupts_per_frame);

      switch(Machine->drv->cpu[activecpu].cpu_type)
	{
	case CPU_Z80:
	  {
	    struct z80context *ctxt;


	    ctxt = (struct z80context *)cpucontext[activecpu];
	    Z80_Reset();
	    Z80_GetRegs(&ctxt->regs);
	    ctxt->icount = cycles;
	    ctxt->iperiod = cycles;
	    ctxt->irq = Z80_IGNORE_INT;
	  }
	  break;
	}
    }


  do
    {
      for (activecpu = 0;activecpu < totalcpu;activecpu++)
	{
	  int loops;


	  memoryread = Machine->drv->cpu[activecpu].memory_read;
	  memorywrite = Machine->drv->cpu[activecpu].memory_write;

	  RAM = Machine->memory_region[Machine->drv->cpu[activecpu].memory_region];
	  /* opcode decryption is currently supported only for the first memory region */
	  if (activecpu == 0) ROM = ROM0;
	  else ROM = RAM;

	  switch(Machine->drv->cpu[activecpu].cpu_type)
	    {
	    case CPU_Z80:
	      {
		struct z80context *ctxt;


		ctxt = (struct z80context *)cpucontext[activecpu];

		Z80_SetRegs(&ctxt->regs);
		Z80_ICount = ctxt->icount;
		Z80_IPeriod = ctxt->iperiod;
		Z80_IRQ = ctxt->irq;

		for (loops = 0;loops < Machine->drv->cpu[activecpu].interrupts_per_frame;loops++)
		  Z80_Execute();

		Z80_GetRegs(&ctxt->regs);
		ctxt->icount = Z80_ICount;
		ctxt->iperiod = Z80_IPeriod;
		ctxt->irq = Z80_IRQ;
	      }
	      break;

	    }
	}

      usres = updatescreen();
      
      if (usres == 2)	/* user asked to reset the machine */
	goto reset;
    } while (usres == 0);
}



/* some functions commonly used by emulators */

int readinputport(int port)
{
  int res,i;
  struct InputPort *in;


  in = &Machine->gamedrv->input_ports[port];

  res = in->default_value;

  for (i = 7;i >= 0;i--)
    {
      int c;


      c = in->keyboard[i];
      if (c && osd_key_pressed(c))
	res ^= (1 << i);
      else
	{
	  c = in->joystick[i];
	  if (c && osd_joy_pressed(c))
	    res ^= (1 << i);
	}
    }

  return res;
}



int input_port_0_r(int offset)
{
  return readinputport(0);
}



int input_port_1_r(int offset)
{
  return readinputport(1);
}



int input_port_2_r(int offset)
{
  return readinputport(2);
}



int input_port_3_r(int offset)
{
  return readinputport(3);
}



int input_port_4_r(int offset)
{
  return readinputport(4);
}



int input_port_5_r(int offset)
{
  return readinputport(5);
}




/***************************************************************************

  Interrupt handling

***************************************************************************/

int Z80_IRQ;	/* needed by the CPU emulation */


/* start with interrupts enabled, so the generic routine will work even if */
/* the machine doesn't have an interrupt enable port */
static int interrupt_enable = 1;
static int interrupt_vector = 0xff;

void interrupt_enable_w(int offset,int data)
{
  interrupt_enable = data;

  if (data == 0) Z80_IRQ = Z80_IGNORE_INT;	/* make sure there are no queued interrupts */
}



void interrupt_vector_w(int offset,int data)
{
  interrupt_vector = data;

  Z80_IRQ = Z80_IGNORE_INT;	/* make sure there are no queued interrupts */
}



/* If the game you are emulating doesn't have vertical blank interrupts */
/* (like Lady Bug) you'll have to provide your own interrupt function (set */
/* a flag there, and return the appropriate value from the appropriate input */
/* port when the game polls it) */
int interrupt(void)
{
  switch(Machine->drv->cpu[activecpu].cpu_type)
    {
    case CPU_Z80:
      if (interrupt_enable == 0) return Z80_IGNORE_INT;
      else return interrupt_vector;
      break;

    default:
      return -1;
      break;
    }
}



int nmi_interrupt(void)
{
  switch(Machine->drv->cpu[activecpu].cpu_type)
    {
    case CPU_Z80:
      if (interrupt_enable == 0) return Z80_IGNORE_INT;
      else return Z80_NMI_INT;
      break;

    default:
      return -1;
      break;
    }
}



/***************************************************************************

  Perform a memory read. This function is called by the CPU emulation.

***************************************************************************/
int cpu_readmem(register int A)
{
  const struct MemoryReadAddress *mra;


  mra = memoryread;
  while (mra->start != -1)
    {
      if (A >= mra->start && A <= mra->end)
	{
	  int (*handler)() = mra->handler;


	  if (handler == MRA_NOP) return 0;
	  else if (handler == MRA_RAM || handler == MRA_ROM) return RAM[A];
	  else return (*handler)(A - mra->start);
	}

      mra++;
    }

  if (errorlog) fprintf(errorlog,"CPU #%d PC %04x: warning - read unmapped memory address %04x\n",activecpu,Z80_GetPC(),A);
  return RAM[A];
}



/***************************************************************************

  Perform a memory write. This function is called by the CPU emulation.

***************************************************************************/
void cpu_writemem(register int A,register unsigned char V)
{
  const struct MemoryWriteAddress *mwa;


  mwa = memorywrite;
  while (mwa->start != -1)
    {
      if (A >= mwa->start && A <= mwa->end)
	{
	  void (*handler)() = mwa->handler;


	  if (handler == MWA_NOP) return;
	  else if (handler == MWA_RAM) RAM[A] = V;
	  else if (handler == MWA_ROM)
	    {
	      if (errorlog) fprintf(errorlog,"CPU #%d PC %04x: warning - write %02x to ROM address %04x\n",activecpu,Z80_GetPC(),V,A);
	    }
	  else (*handler)(A - mwa->start,V);

	  return;
	}

      mwa++;
    }

  if (errorlog) fprintf(errorlog,"CPU #%d PC %04x: warning - write %02x to unmapped memory address %04x\n",activecpu,Z80_GetPC(),V,A);
  RAM[A] = V;
}



byte Z80_In(byte Port)
{
  const struct IOReadPort *iorp;


  iorp = Machine->drv->cpu[activecpu].port_read;
  if (iorp)
    {
      while (iorp->start != -1)
	{
	  if (Port >= iorp->start && Port <= iorp->end)
	    {
	      int (*handler)() = iorp->handler;


	      if (handler == IORP_NOP) return 0;
	      else return (*handler)(Port - iorp->start);
	    }

	  iorp++;
	}
    }

  if (errorlog) fprintf(errorlog,"CPU #%d PC %04x: warning - read unmapped I/O port %02x\n",activecpu,Z80_GetPC(),Port);
  return 0;
}



void Z80_Out(byte Port,byte Value)
{
  const struct IOWritePort *iowp;


  iowp = Machine->drv->cpu[activecpu].port_write;
  if (iowp)
    {
      while (iowp->start != -1)
	{
	  if (Port >= iowp->start && Port <= iowp->end)
	    {
	      void (*handler)() = iowp->handler;


	      if (handler == IOWP_NOP) return;
	      else (*handler)(Port - iowp->start,Value);

	      return;
	    }

	  iowp++;
	}
    }

  if (errorlog) fprintf(errorlog,"CPU #%d PC %04x: warning - write %02x to unmapped I/O port %02x\n",activecpu,Z80_GetPC(),Value,Port);
}



/***************************************************************************

  Interrupt handler. This function is called at regular intervals
  (determined by IPeriod) by the CPU emulation.

***************************************************************************/

int cpu_interrupt(void)
{
  return (*Machine->drv->cpu[activecpu].interrupt)();
}



void Z80_Patch (Z80_Regs *Regs)
{
}



void Z80_Reti (void)
{
}



void Z80_Retn (void)
{
}
