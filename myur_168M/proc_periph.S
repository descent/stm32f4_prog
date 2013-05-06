#if 0

@.equ STACK_TOP, 0x20000800
.text
.global _start
.code 16
.syntax unified
_start:
  .word STACK_TOP, start 
  .type start, function @ let lsb to 1

  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1 @ svc isr
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
#endif

#define ENABLE_SYSTICK

.syntax unified
.code 16
.text
.global init_proc
.type start, function @ let lsb to 1
init_proc:
@  bl init_c_runtime
@  bl init
  ldr r0, =cur_proc
  mov r1, #1
  str r1, [r0]


  ldr r0, =stack_frame_a

  ldr r1, =#0xffffffff
  str r1, [r0, #20] @ setup lr

  ldr r1, =proc_a
  str r1, [r0, #24] @ setup pc
@  mov r1, #0x21000000
  mov r1, #0x1000000
  str r1, [r0, #28] @ setup psr

  ldr r0, =a_sp
  ldr r1, =stack_frame_a
  sub r1, #4 @ save for r7
  str r1, [r0]

#ifdef ENABLE_SYSTICK
@ 0xe000ed24 check systick isr active
@ 0xe000e018 check systick count
  ldr r0, =0xe000e010 @ control reg
  mov r1, #0
  str r1, [r0]
  ldr r1, =0x3fff
  str r1, [r0, #4]
  str r1, [r0, #8]
  mov r1, #0x7
  str r1, [r0] @ enable systick
#else
  svc 0
#endif
  svc 0
@  bx lr

.type proc_b, function @ let lsb to 1
proc_b:
  movs r1, #2
  nop
  bl put_b
@  svc 0
  nop
  b proc_b



.type proc_a, function @ let lsb to 1
proc_a:
  movs r0, #1
  bl put_a
  nop
@  svc 0
  nop
  b proc_a


.type del_func, function @ let lsb to 1
del_func:
  mov r5, #5
  b del_func

#if 1
.global asm_set
.type asm_set, function @ let lsb to 1
asm_set:
#if 0
  @ldr r0, =0x22000008
  ldr r0, =0x22001a80
  mov r1, #1
  str r1, [r0]
#else
  ldr r0, =asm_bit_band
  sub r0, #0x20000000
  lsl r0, #5 @ x 32
  add r0, #0x22000000
  mov r1, #1
  str r1, [r0]
#endif
  bx lr

.global asm_clear
.type asm_clear, function @ let lsb to 1
asm_clear:
  ldr r0, =asm_bit_band
  sub r0, #0x20000000
  lsl r0, #5
  add r0, #0x22000000
  mov r1, #0
  str r1, [r0]
  bx lr
#endif
.global int_isr
.type int_isr, function @ let lsb to 1
int_isr:
  push {r7}
  mov r3, sp @ save cur sp
  @add     r3, sp, #0

  ldr r0, =cur_proc
  ldr r1, [r0]
  cmp r1, #1
  beq.n 1f
  @ not 1, cur proc is proc_a
  @ save sp to a_sp,
  @ set sp by b_sp
  @ set cur_proc to 1

  ldr r1, =a_sp
  str r3, [r1]
  ldr r1, =b_sp
  mov r2, #1
  b 2f

1:
  @ eq 1, cur proc is proc_b, save sp to b_sp,
  @ set sp by a_sp
  @ set cur_proc to 0
  ldr r1, =b_sp
  str r3, [r1]
  ldr r1, =a_sp
  mov r2, #0
2:

  str r2, [r0] @ write 1 or 0 to cur_proC

  ldr sp, [r1]
@  mov sp, r1
  pop {r7}
  bx lr
  #movs pc, proc_a


#.bss
#.section ".stackarea"
.data

sram_begin:

.space  0x40, 0
stack_frame_a:
.word 1 @r0
.word proc_a @r1
.word 3 @r2
.word 0 @r3
.word 0 @r12
.word del_func @lr @ del_func
.word proc_a @pc @ proc_a
.word 0x21000000 @psr

.space  0x40, 0
stack_frame_b:
.word 0 @r0
.word 0 @r1
.word 0 @r2
.word 0 @r3
.word 0 @r12
.word del_func @lr
.word proc_b @pc
.word 0x21000000 @psr

cur_proc:
.word 1

asm_bit_band:
.word 0

a_r7:
.word 0
b_r7:
.word 0

a_sp:
.word 0
b_sp:
.word 0

#if 0
.section ".stackarea"
.space  0x400, 0
STACK_TOP:
#endif
