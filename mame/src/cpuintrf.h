#ifndef CPUINTRF_H
#define CPUINTRF_H


void cpu_run(void);

void cpu_writemem(register int A,register unsigned char V);
int cpu_readmem(register int A);

/* some useful general purpose functions for the memory map */
int readinputport(int port);
int input_port_0_r(int offset);
int input_port_1_r(int offset);
int input_port_2_r(int offset);
int input_port_3_r(int offset);
int input_port_4_r(int offset);
int input_port_5_r(int offset);
void interrupt_enable_w(int offset,int data);
void interrupt_vector_w(int offset,int data);
int interrupt(void);
int nmi_interrupt(void);

#endif
