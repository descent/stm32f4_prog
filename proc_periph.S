# process switch and blink led
# ref : http://tw.m.wretch.yahoo.com/blog/DreamYeh/888788
# comment: #, @

.equ STACK_TOP, 0x20000800
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
  .word int_isr+1 @ sysick isr
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1

start:
  ldr r5, =cur_proc
  mov r1, #0
  str r1, [r5]

  ldr r0, =0xe000e010 @ control reg
  mov r1, #0
  str r1, [r0]
  ldr r1, =0x3ff
  str r1, [r0, #4]
  str r1, [r0, #8]
  mov r1, #0x7
  str r1, [r0] @ enable systick

  # 0xe000e014
  # 0xe000e018
  # 0xe000e01c

loop:
  nop
  b loop

  movs r3, #3
  movs r5, #5
  @ldr r1, [r0, #4]!

  ldr r0, =stack_frame_a

@  ldr r1, =del_func
@  str r1, [r0, #20] @ setup lr

  ldr r1,=#0xffffffff
  str r1, [r0, #20] @ setup lr

  @ldr r1, =proc_a
  adr r1, proc_a
  str r1, [r0, #24] @ setup pc
@  mov r1, #0x21000000
  mov r1, #0x1000000
  str r1, [r0, #28] @ setup psr

@  ldr lr, =#0xfffffff9

@  ldr sp,=#0x20000060
@  ldr sp, [r0, #32]

@  bx lr





  ldr r0, =stack_frame_b

@  ldr r1, =del_func
@  str r1, [r0, #20] @ setup lr

  ldr r1,=#0xffffffff
  str r1, [r0, #20] @ setup lr

  ldr r1, =proc_b
  str r1, [r0, #24] @ setup pc
  mov r1, #0x1000000
  str r1, [r0, #28] @ setup psr


  @ldr sp, =stack_frame_a
  @mov r0, #0x20000100
  @ldr sp, [r0, #16]
@  ldr sp,=#0x20000040
  svc 0

deadloop:
  b deadloop

.type proc_a, function @ let lsb to 1
proc_a:
  movs r0, #1
  nop
  svc 0
  nop
  b proc_a

.type proc_b, function @ let lsb to 1
proc_b:
  movs r1, #2
  nop
  svc 0
  nop
  b proc_b

.type del_func, function @ let lsb to 1
del_func:
  mov r5, #5
  b del_func

int_isr:
  nop
  movs r5, #5
  ldr r5, =cur_proc
  ldr r0, [r5]
  cmp r0, #0
  ittee eq
  ldreq sp,=#stack_frame_a
  moveq r1, #1
  ldrne sp,=#stack_frame_b
  movne r1, #0

  str r1, [r5]
  bx lr
  #movs pc, proc_a


#.bss
#.section ".stackarea"
.data

sram_begin:
.space  0x40, 0
STACK_TOP:

stack_frame_a:
.word 1 @r0
.word proc_a @r1
.word 3 @r2
.word 0 @r3
.word 0 @r12
.word del_func @lr @ del_func
.word proc_a @pc @ proc_a
.word 0x21000000 @psr

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
