#ifndef _VCPU_H_
#define _VCPU_H_

/* If BEST_EFFORT_VCPU, sched_param, vcpu_type or iovcpu_class are
   changed they must also be changed in the kernel's vcpu.h */

#define BEST_EFFORT_VCPU 0

typedef enum {
  MAIN_VCPU = 0, IO_VCPU
} vcpu_type;

typedef enum {
  IOVCPU_CLASS_ALL = 0,
  IOVCPU_CLASS_ATA = (1<<0),
  IOVCPU_CLASS_NET = (1<<1),
  IOVCPU_CLASS_USB = (1<<2),
  IOVCPU_CLASS_DISK = (1<<3),
  IOVCPU_CLASS_CDROM = (1<<4),
} iovcpu_class;

struct sched_param
{
  int sched_priority;

  vcpu_type type;
  iovcpu_class io_class;

  /* Below are paramters used for window-constrained scheduling */
  int C;                        /* service quantum */
  int T;                        /* period */
  int m;                        /* mandatory instance count in a window */
  int k;                        /* window of requests  */
  int affinity;                 /* CPU (or Quest-V sandbox affinity) */
  int machine_affinity;         /* -- EM -- Machine affinity hack
                                   right now is a just a bool to
                                   indicate stay (0) or move to other
                                   machine (1) */
};

static int vcpu_create_main(int C, int T)
{
  struct sched_param sp = { .type = MAIN_VCPU, .C = C, .T = T };
  return vcpu_create(&sp);
}


int vcpu_fork(uint vcpu_id);

#endif _VCPU_H_
