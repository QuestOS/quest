#ifndef _VCPU_H_
#define _VCPU_H_

/* If this is changed it must also be changed in kernel's vcpu.h */
#define BEST_EFFORT_VCPU 0

struct sched_param
{
  int sched_priority;

  /* Below are paramters used for window-constrained scheduling */
  int C;                        /* service quantum */
  int T;                        /* period */
  int m;                        /* mandatory instance count in a window */
  int k;                        /* window of requests  */
  int affinity;                 /* CPU (or Quest-V sandbox) affinity */
  int machine_affinity;         /* -- EM -- Machine affinity hack
                                   right now is a just a bool to
                                   indicate stay (0) or move to other
                                   machine (1) */
};


int vcpu_fork(uint vcpu_id);

#endif _VCPU_H_
