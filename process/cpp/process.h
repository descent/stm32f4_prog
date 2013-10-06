#ifndef PROCESS_H
#define PROCESS_H

class Process
{
  public:
    void (*exec)(void);
    char *stack_pointer;  
};

extern Process *ready_process;

#endif
