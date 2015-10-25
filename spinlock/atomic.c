// ref: arch/arm/include/asm/atomic.h
// ref: Linux 內核同步機制之 (一): 原子操作 http://www.wowotech.net/kernel_synchronization/atomic.html 

typedef struct
{
  int counter;
} atomic_t;

void atomic_add(int i, atomic_t *v)
{
        unsigned long tmp;
        int result;

//        prefetchw(&v->counter);
        __asm__ __volatile__("@ atomic_add\n"
"1:     ldrex   %0, [%3]\n"
"       add     %0, %0, %4\n"
"       strex   %1, %0, [%3]\n"
"       teq     %1, #0\n"
"       bne     1b"     
        : "=&r" (result), "=&r" (tmp), "+Qo" (v->counter)
        : "r" (&v->counter), "Ir" (i)
        : "cc");
}

int main(int argc, char *argv[])
{
  
  return 0;
}
