#ifndef _VCPU_H_
#define _VCPU_H_

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

#endif _VCPU_H_
