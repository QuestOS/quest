#include <stdio.h>
#include <pthread.h>

#define WEAK_LOOP(i)	  \
	extern void loop##i##_init() __attribute__((weak))

WEAK_LOOP(1);
WEAK_LOOP(2);
WEAK_LOOP(3);
WEAK_LOOP(4);
WEAK_LOOP(5);
WEAK_LOOP(6);
WEAK_LOOP(7);
WEAK_LOOP(8);
WEAK_LOOP(9);
WEAK_LOOP(10);
WEAK_LOOP(11);
WEAK_LOOP(12);
WEAK_LOOP(13);
WEAK_LOOP(14);
WEAK_LOOP(15);
WEAK_LOOP(16);
WEAK_LOOP(17);
WEAK_LOOP(18);
WEAK_LOOP(19);
WEAK_LOOP(20);
WEAK_LOOP(21);
WEAK_LOOP(22);
WEAK_LOOP(23);
WEAK_LOOP(24);
WEAK_LOOP(25);
WEAK_LOOP(26);
WEAK_LOOP(27);
WEAK_LOOP(28);
WEAK_LOOP(29);
WEAK_LOOP(30);
WEAK_LOOP(31);
WEAK_LOOP(32);
extern int loop() __attribute__((weak));

#define MAX_THREAD_NUM 		32
pthread_t thread[MAX_THREAD_NUM];

void main()
{
	int i, res;

	setup();

	if (loop) {
		/* backward compatible */
		while(1) loop();
	} else {
		/* call user defined loop_init() to start thread */
		if (loop1_init) loop1_init();
		if (loop2_init) loop2_init();
		if (loop3_init) loop3_init();
		if (loop4_init) loop4_init();
		if (loop5_init) loop5_init();
		if (loop6_init) loop6_init();
		if (loop7_init) loop7_init();
		if (loop8_init) loop8_init();
		if (loop9_init) loop9_init();
		if (loop10_init) loop10_init();
		if (loop11_init) loop11_init();
		if (loop12_init) loop12_init();
		if (loop13_init) loop13_init();
		if (loop14_init) loop14_init();
		if (loop15_init) loop15_init();
		if (loop16_init) loop16_init();
		if (loop17_init) loop17_init();
		if (loop18_init) loop18_init();
		if (loop19_init) loop19_init();
		if (loop20_init) loop20_init();
		if (loop21_init) loop21_init();
		if (loop22_init) loop22_init();
		if (loop23_init) loop23_init();
		if (loop24_init) loop24_init();
		if (loop25_init) loop25_init();
		if (loop26_init) loop26_init();
		if (loop27_init) loop27_init();
		if (loop28_init) loop28_init();
		if (loop29_init) loop29_init();
		if (loop30_init) loop30_init();
		if (loop31_init) loop31_init();
		if (loop32_init) loop32_init();

		/* waiting for threads to finish */
		for (i = 0; i < MAX_THREAD_NUM; i++) {
			if (thread[i] != 0) {
				res = waitpid(thread[i]);
				printf("Return from waitpid %d\n", res);
			}
		}
		printf("pthread_exit in main!\n");
		pthread_exit(NULL);
	}
}
