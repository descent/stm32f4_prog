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

  @ setup proc a psr
  mov r1, #0x21000000
  str r1, [r0, #(-4)]

  @ setup proc a pc
  ldr r1, =proc_a
  str r1, [r0, #(-8)]

  @@@@@@@@@@@@@@@@@@@

  @ init b proc
  ldr r0, =proc_b_stack

  @ setup proc b psr
  mov r1, #0x21000000
  str r1, [r0, #(-4)]

  @ setup proc b pc
  ldr r1, =proc_b
  str r1, [r0, #(-8)]

  @@@@@@@@@@@@@@@@@@@

  ldr r0, =proc_a_sp
  ldr r1, =proc_a_stack
  sub r1, #(16*4)
  str r1, [r0]

  ldr r0, =proc_b_sp
  ldr r1, =proc_b_stack
  sub r1, #(16*4)
  str r1, [r0]

  ldr r0, =cur_proc
  mov r1, #0
  str r1, [r0]

  ldr r1, =proc_a_sp
  ldr r0, [r1]

  @ ref: scmrtos-code/Ports/CortexM3/GCC/OS_Target_asm.S
  LDR     r4, [r0, #(4 * 14)]     @ Load process entry point into R4
  add     r0, #(4 * 16)           @ emulate context restore
  mov sp, r0
@  dsb
  isb                             @ Insert a barrier

  bx r4                           @ run proc_a

.type proc_a, function @ let lsb to 1
proc_a:
  movs r0, #0x10
  movs r1, #0x11
  movs r2, #0x12
  movs r3, #0x13
  movs r4, #0x14
  movs r5, #0x15
  movs r6, #0x16
  movs r7, #0x17
  movs r8, #0x18
  movs r9, #0x19
  movs r10, #0x1a
  movs r11, #0x1b
  movs r12, #0x1c
  push {r0}
  svc 0
  nop
  b proc_a

.type proc_b, function @ let lsb to 1
proc_b:
  movs r0, #0x20
  movs r1, #0x21
  movs r2, #0x22
  movs r3, #0x23
  movs r4, #0x24
  movs r5, #0x25
  movs r6, #0x26
  movs r7, #0x27
  movs r8, #0x28
  movs r9, #0x29
  movs r10, #0x2a
  movs r11, #0x2b
  movs r12, #0x2c
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
  cpsid i @Prevent interruption during context switch

  push {r4-r11}

  @ choice another process
  ldr r1, =cur_proc
  ldr r0, [r1]

  cmp r0, #1
  beq.n 1f
  @ current proc a, switch to proc b
  ldr r2, =proc_a_sp
  str sp, [r2] @ save cur process sp

  mov r3, #1
  str r3, [r1]

  ldr r0, =proc_b_sp

  b 2f
1:
  @ current proc b, switch to proc a
  ldr r2, =proc_b_sp
  str sp, [r2] @ save cur process sp

  mov r3, #0
  str r3, [r1]

  ldr r0, =proc_a_sp

2:

  ldr r0, [r0]

  mov sp, r0
  pop {R4-R11}     // Restore r4-11 from new process stack

  cpsie i
  bx lr


.data

proc_a_sp:
.word 1
proc_b_sp:
.word 2

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
