#ifndef __THREAD_H_
#define __THREAD_H_

#include <pthread.h>
#include <vcpu.h>

extern pthread_t thread[];

#define loop(i,c,t)  																										\
	dummy##i();																															\
	extern void _loop##i(); 																							\
																																				\
  void loop##i () {        																							\
		struct sched_param s_params = {.type = MAIN_VCPU, .C = c, .T = t};  \
		int new_vcpu = vcpu_create(&s_params);          										\
		if(new_vcpu < 0) {                              										\
			printf("Failed to create vcpu\n");           											\
			exit(1);                                    											\
		}                                          													\
		extern int vcpu_bind_task(vcpu_id_t);																					\
		vcpu_bind_task(new_vcpu);                   												\
		while (1) _loop##i ();                      												\
	}                                         														\
																																				\
  void loop##i##_init() {  																							\
		pthread_create(&thread[i], NULL, (void *)loop##i, NULL);  					\
	}                      																								\
																																				\
	void _loop##i ()

#endif

