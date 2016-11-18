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
  extern unsigned long _bss;
  extern unsigned long _ebss;
  unsigned long *bss_dest;

  for (bss_dest = &_bss; bss_dest < &_ebss;)
  {
    //*bss_dest++ = 0xabcd9876;
    *bss_dest++ = 0x0;
  }
}

char* s32_itoa(uint32_t n, char* str, int radix)
{
  char digit[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  char* p=str;
  char* head=str;
  uint8_t count=0;
  //int radix = 10;

//  if(!p || radix < 2 || radix > 36)
//    return p;
  if (n==0)
  {
    *p++='0';
    *p++ = '0';
    *p=0;
    return str;
  }
  if (radix == 10 && n < 0)
  {
    *p++='-';
    n= -n;
  }

  while(n)
  {
    ++count;
    *p++=digit[n%radix];
    //s32_put_char(*(p-1), (u8*)(0xb8000+80*2));
    n/=radix;
  }
  if (count == 1)
    *p++ = '0';

  *p=0;
  #if 1
  for (--p; head < p ; ++head, --p)
  {
    char temp=*head;
    if (*(p-1) != '-')
    {
      *head=*p;
      *p=temp;
    }
  }
  #endif
  return str;
}

u8 buf[4];
int x=0x12345678;
int mymain(void)
{
  init_bss();
  char str[]= "from sd loaded ok\r\n";
  ur_puts(USART2, str);

  char fmt_str[20];
  s32_itoa(x, fmt_str, 16);
  ur_puts(USART2, fmt_str);
  ur_puts(USART2, "\r\n");
  for (int i=0 ; i < 4; ++i)
  {
    s32_itoa(buf[i], fmt_str, 16);
    ur_puts(USART2, fmt_str);
    ur_puts(USART2, "\r\n");
  }

  return 2;
}

