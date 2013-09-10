// cs means context switch

#include "type.h"

void proc_a()
{
  int i=0;
  __asm__ ("svc 0");
  ++i;
}

void proc_b()
{
  int i=2;
  __asm__ ("svc 0");
  ++i;
}

typedef struct Process_
{
  void (*exec)(void);
  char *stack_pointer;  
}Process;

#define STACK_SIZE 600
char stack[STACK_SIZE];

#define NUM 2
Process process[NUM];

void run()
{
  // get stack pointer to r0
  //__asm__ volatile ();
}

void mymain(void)
{
  process[0].exec = proc_a;
  process[0].stack_pointer = stack+STACK_SIZE;

  process[1].exec = proc_b;
  process[1].stack_pointer = stack + STACK_SIZE - (STACK_SIZE/NUM);
  run();

  while(1);
}
