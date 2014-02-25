// http://wiki.csie.ncku.edu.tw/embedded/Lab2
/* use setjmp/longjmp to implement coroutines */

#include "stm32.h"
#include "my_setjmp.h"

#define setjmp my_setjmp
#define longjmp my_longjmp

#define STACK_SIZE 4096

char memarea1[128];
char memarea2[128];

/*
 * Change SP prior to calling setjmp so that longjmp will
 * start the routine with 'stackptr'.
 */
#if defined(__i386__)
#define SAVE_STACK_POINTER_ASM(savedstack, stackptr) \
	"movl %%esp, %[savedstack]\n" /* savedstack <- SP */ \
	"movl %[stackptr], %%esp"    /* SP <- stackptr */
#elif defined(__x86_64__)
#define SAVE_STACK_POINTER_ASM(savedstack, stackptr) \
	"movq %%rsp, %[savedstack]\n" /* savedstack <- SP */ \
	"movq %[stackptr], %%rsp"    /* SP <- stackptr */
#else
//#error "Unsupported architecture!"
#define SAVE_STACK_POINTER_ASM(savedstack, stackptr) \
	"mov %[savedstack], %%r7\n" /* savedstack <- SP */ \
	"mov %%r7, %[stackptr]"    /* SP <- stackptr */
#endif


#define SAVE_STACK_POINTER(savedstack, stackptr) \
do { \
	asm volatile ( SAVE_STACK_POINTER_ASM(savedstack, stackptr) \
	: [savedstack] "=r" (savedstack): [stackptr] "r" (stackptr) ); \
} while (0)

/* Restore "normal" stack prior to return */
#if defined(__i386__)
#define RESTORE_STACK_ASM(savedstack) \
	"movl %[savedstack],%%esp"
#elif defined(__x86_64__)
#define RESTORE_STACK_ASM(savedstack) \
	"movq %[savedstack],%%rsp"
#else
#define RESTORE_STACK_ASM(savedstack) \
	"mov %%r7, %[savedstack]"
//#error "Unsupported architecture!"
#endif


#define RESTORE_STACK(savedstack) \
do { \
	asm volatile ( RESTORE_STACK_ASM(savedstack) \
	: : [savedstack] "r" (savedstack)); \
} while (0)	

jmp_buf routine1_buf;
jmp_buf routine2_buf;

int exit(int i)
{
}

void *malloc(int size)
{
}

void printf(char *str, ...)
{
}

void routine1()
{
	volatile unsigned count = 0;

	/* Start of Routine #1 */
	setjmp(routine1_buf);

	printf("[Routine 1] pass %d\n", ++count);

	longjmp(routine2_buf, 1);
}

void create_routine1(const void *stackptr)
{
	register unsigned long savedstack;

	SAVE_STACK_POINTER(savedstack, stackptr);

	if (setjmp(routine1_buf) == 0) {
		RESTORE_STACK(savedstack);
	}
	else {
		/* We got here through longjmp */
		routine1();
	}
}

#define FINAL_NUM 5
void routine2()
{
	volatile unsigned count = 0;

	/* Start of Routine #2 */
	setjmp(routine2_buf);

	printf("[Routine 2] pass %d\n", (FINAL_NUM - ++count));

	if (count < FINAL_NUM)
		longjmp(routine1_buf, 1);

	exit(0);
}

void create_routine2(const void *stackptr)
{
	register unsigned long savedstack;

	SAVE_STACK_POINTER(savedstack, stackptr);

	if (setjmp(routine2_buf) == 0) {
		RESTORE_STACK(savedstack);
	}
	else {
		routine2();
	}
}


int main(void)
{
  //create_routine1((char *) malloc(STACK_SIZE) + STACK_SIZE);
  //create_routine2((char *) malloc(STACK_SIZE) + STACK_SIZE);
  create_routine1(memarea1);
  create_routine2(memarea2);
  longjmp(routine1_buf, 1);

  return 0;
}
