#include "asm_syscall.h"

typedef void* SystemCall;
typedef int (*FuncPtr)();

struct Arg
{
};

int sys_get_ticks()
{
  extern int ticks;
  return ticks;
}

// ref: http://www.coactionos.com/embedded-design/133-effective-use-of-arm-cortex-m3-svcall.html
void service_call(FuncPtr func_ptr, Arg *arg)
{
  __asm__ volatile("svc 0");
}

int get_ticks()
{
  Arg *arg=0;

  service_call(sys_get_ticks, arg);
}

SystemCall sys_call_table[NR_SYS_CALL] = {(void*)sys_get_ticks};

