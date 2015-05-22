/*
  Serial debug output, on PB10.
*/

#include <math.h>
#include <stm32f4_discovery.h>

#include "dbg.h"

void
setup_serial(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* enable peripheral clock for USART3 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* GPIOB Configuration:  USART3 TX on PB10. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect USART3 pins to AF7 */
  // TX = PB10
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);

  USART_Cmd(USART3, ENABLE);
}


void
serial_putchar(uint32_t c)
{
  while(!(USART3->SR & USART_FLAG_TC));
  USART_SendData(USART3, c);
}


void
serial_puts(const char *s)
{
  while (*s)
    serial_putchar((uint8_t)*s++);
}


static void
serial_output_hexdig(uint32_t dig)
{
  serial_putchar((dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


void
serial_output_hexbyte(uint8_t byte)
{
  serial_output_hexdig(byte >> 4);
  serial_output_hexdig(byte & 0xf);
}


void
println_uint32(uint32_t val)
{
  char buf[13];
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(buf);
}


void
println_int32(int32_t val)
{
  if (val < 0)
  {
    serial_putchar('-');
    println_uint32((uint32_t)0 - (uint32_t)val);
  }
  else
    println_uint32(val);
}


void
print_uint32_hex(uint32_t val)
{
  serial_output_hexbyte(val >> 24);
  serial_output_hexbyte((val >> 16) & 0xff);
  serial_output_hexbyte((val >> 8) & 0xff);
  serial_output_hexbyte(val & 0xff);
}


static void
float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after)
{
  float a;
  uint32_t d;
  uint8_t leading_zero;

  if (f == 0.0f)
  {
    buf[0] = '0';
    buf[1] = '\0';
    return;
  }
  if (f < 0)
  {
    *buf++ = '-';
    f = -f;
  }
  a =  powf(10.0f, (float)dig_before);
  if (f >= a)
  {
    buf[0] = '#';
    buf[1] = '\0';
    return;
  }
  leading_zero = 1;
  while (dig_before)
  {
    a /= 10.0f;
    d = (uint32_t)(f / a);
    if (leading_zero && d == 0 && a >= 10.0f)
      *buf++ = ' ';
    else
    {
      leading_zero = 0;
      *buf++ = '0' + d;
      f -= d*a;
    }
    --dig_before;
  }
  if (!dig_after)
  {
    *buf++ = '\0';
    return;
  }
  *buf++ = '.';
  do
  {
    f *= 10.0f;
    d = (uint32_t)f;
    *buf++ = '0' + d;
    f -= (float)d;
    --dig_after;
  } while (dig_after);
  *buf++ = '\0';
}


void
println_float(float f, uint32_t dig_before, uint32_t dig_after)
{
  char buf[21];
  char *p = buf;

  float_to_str(p, f, dig_before, dig_after);
  while (*p)
    ++p;
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(buf);
}


void
serial_dump_buf(uint8_t *buf, uint32_t len)
{
  uint32_t i, j;

  for (i = 0; i < len; i += 16)
  {
    print_uint32_hex(i);
    serial_puts(" ");
    for (j = 0; j < 16 && (i+j) < len; ++j)
    {
      if (!(j % 4))
        serial_puts(" ");
      serial_output_hexbyte(buf[i+j]);
    }
    serial_puts("\r\n");
  }
}
