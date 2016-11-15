#ifndef STM32_H
#define STM32_H

#include "type.h"
#include "stm32f4xx_usart.h"

#define STACK_SIZE 2048
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;

int main(void);

void ResetISR(void)
{
  unsigned long *pulSrc, *pulDest;

  pulSrc = &_etext;
  for (pulDest = &_data; pulDest < &_edata;)
    *pulDest++ = *pulSrc++;
  for (pulDest = &_bss; pulDest < &_ebss;)
    *pulDest++ = 0;

  main();
}

void int_isr(void)
{
}


typedef void (*pfnISR)(void);
__attribute__((section(".stackares")))
static u8 pulStack[STACK_SIZE];


__attribute__((section(".isr_vector")))
pfnISR VectorTable[]=
{
  (pfnISR)((unsigned long)pulStack+sizeof(pulStack)),
  ResetISR, // 1
  int_isr,
  int_isr,
  int_isr,
  int_isr,
  int_isr,
  int_isr,
  int_isr,
  int_isr,
  int_isr,
  int_isr,    // 11
  int_isr,
  int_isr,
  int_isr, // 14
  int_isr, // 15

#ifdef EXT_INT
  // External Interrupts
  wwdg_isr,                   // Window WatchDog
  pvd_isr,                   // PVD through EXTI Line detection                      
  tamp_stamp_isr,            // Tamper and TimeStamps through the EXTI line
  rtc_wkup_isr,              // RTC Wakeup through the EXTI line                     
  flash_isr,                 // FLASH                                           
  rcc_isr,                   // RCC                                             
  exti0_isr                  // EXTI Line0 
#endif
};

#endif
