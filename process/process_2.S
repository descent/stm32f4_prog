# not process switch, this is context switch

@.equ STACK_TOP, 0x20000800
.text
.global _start
.code 16
.syntax unified
_start:
  .word STACK_TOP, start 
  .type start, function @ let lsb to 1

  .word int_isr+1
  .word hardf_isr+1
  .word mmf_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word svc_isr+1 @ svc isr
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1

start:
  @ init a proc
  ldr r0, =proc_a_stack

  @ setup proc a stack
  ldr r1, =proc_a_stack
  str r1, [r0]

  @ setup proc a psr
  mov r1, #0x21000000
  str r1, [r0, #(-4)]

  @ setup proc a pc
  ldr r1, =proc_a
  str r1, [r0, #(-8)]

  @@@@@@@@@@@@@@@@@@@

  @ init b proc
  ldr r0, =proc_b_stack

  @ setup proc b stack
  ldr r1, =proc_b_stack
  str r1, [r0]

  @ setup proc b psr
  mov r1, #0x21000000
  str r1, [r0, #(-4)]

  @ setup proc b pc
  ldr r1, =proc_b
  str r1, [r0, #(-8)]

  @@@@@@@@@@@@@@@@@@@

  ldr r0, =cur_proc
  ldr r1, =proc_a_ptr
  str r1, [r0]

  ldr r0, =proc_a_ptr
  @ ref: scmrtos-code/Ports/CortexM3/GCC/OS_Target_asm.S
  LDR     r4, [r0, #(4 * 14)]     @ Load process entry point into R4
  add     r0, #(4 * 16)           @ emulate context restore
  ldr r1, [r0]
  mov sp, r1
@  dsb
@  isb                             @ Insert a barrier

  bx r4                           @ run proc_a

.type proc_a, function @ let lsb to 1
proc_a:
  movs r0, #1
@  movs r4, #2
  svc 0
  nop
  b proc_a

.type proc_b, function @ let lsb to 1
proc_b:
  movs r0, #8
@  movs r4, #9
  nop
  svc 0
  b proc_b


.type del_func, function @ let lsb to 1
del_func:
  mov r5, #5
  b del_func

int_isr:
  mov r0, #67
  bx lr

hardf_isr:
  mov r0, #0x3b
  bx lr
@ memory manage fault
mmf_isr:
  mov r0, #0x3a
  bx lr

svc_isr:
@  cpsid i @Prevent interruption during context switch
  mov r0, sp
  stmdb   r0!, {r4-r11}
  push {lr}

  @ choice another process
  ldr r1, =cur_proc
  ldr r3, [r1]
  ldr r2, =proc_a_ptr
  ldr r12, =proc_b_ptr

  cmp r3, r12
  beq.n 1f
  @ set to proc_b_ptr
  str r12, [r1]
  b 2f
1:
  @ set to proc_a_ptr
  str r2, [r1]
2:
  ldr r0, [r1]
  ldr r12, [r0, #(4*16)]
  mov sp, r12

@  cpsie i
  pop {pc}


.data
#if 0
proc_a_ptr:
.word 1 @r11
.word 1 @r10
.word 1 @r9
.word 1 @r8
.word 1 @r7
.word 1 @r6
.word 1 @r5
.word 1 @r4

.word 1 @r0
.word 2 @r1
.word 3 @r2
.word 0 @r3
.word 0 @r12
.word del_func @lr @ del_func
.word proc_b @pc @ proc_b
.word 0x21000000 @psr
.word =proc_a_stack @currnet process stack pointer
stack_frame_a:

proc_b_ptr:
.word 1 @r11
.word 1 @r10
.word 1 @r9
.word 1 @r8
.word 1 @r7
.word 1 @r6
.word 1 @r5
.word 1 @r4

.word 1 @r0
.word 2 @r1
.word 3 @r2
.word 0 @r3
.word 0 @r12
.word del_func @lr @ del_func
.word proc_a @pc @ proc_a
.word 0x21000000 @psr
.word =proc_b_stack @currnet process stack pointer
stack_frame_b:
#endif

proc_a_ptr:
.space  0x300, 0
proc_a_stack:

proc_b_ptr:
.space  0x300, 0
proc_b_stack:
cur_proc:
.word 

.space  0x300, 0
STACK_TOP:
