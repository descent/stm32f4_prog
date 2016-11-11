#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
    
  /* Transmit Data */
  USARTx->DR = (Data & (uint16_t)0x01FF);
}


void ur_puts(USART_TypeDef* USARTx, volatile char *s)
{
  while(*s)
  {
    // wait until data register is empty
    while( !(USARTx->SR & 0x00000040) );
    USART_SendData(USARTx, *s);
    *s++;
  }
}

void init_bss()
{
}

int mymain(void)
{
  init_bss();
  char str[]= "from sd loaded ok\r\n";
  ur_puts(USART2, str);
  return 2;
}

