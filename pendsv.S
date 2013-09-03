# test pendSV handler

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
  .word pendsv_handle+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1
  .word int_isr+1

start:

  @ pendsv priority
  ldr r0, =0xe000ed22
  mov r1, #0xff
  str r1, [r0]

  @ systick priority
  ldr r0, =0xe000ed23
  mov r1, #0xff
  str r1, [r0]

  @cpsie i

  @ enable pendsv
  ldr r0, =0xe000ed04 
  mov r1, #0x10000000
  str r1, [r0]


  push {r0}

deadloop:
  nop
  push {r0}
  b deadloop

pendsv_handle:
  nop
  bx lr

int_isr:
  nop
  movs r1, #0xf
  push {r1}
  bx lr

.data
.space  0x200, 0
psp_stack_top:

.space  0x400, 0
msp_stack_top:
