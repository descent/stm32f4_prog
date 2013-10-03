#include "asm_syscall.h"

typedef void* SystemCall;

int sys_get_ticks()
{
  extern int ticks;
  return ticks;
}

SystemCall sys_call_table[NR_SYS_CALL] = {(void*)sys_get_ticks};

