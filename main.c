/* POV3D. */

#include <stm32f4_discovery.h>

#include "dbg.h"
#include "led.h"


/* This is apparently needed for libc/libm (eg. powf()). */
int __errno;


static void
delay(__IO uint32_t nCount)
{
    while(nCount--)
        __asm("nop"); // do nothing
}


int
main(void)
{
  setup_led();
  setup_serial();

  serial_puts("\r\n\r\nSTM32F4 wireless bootloader\r\nCopyright 2015 Kristian Nielsen\r\n");
  for (;;)
  {
    led_on();
    delay(28000000);
    led_off();
    delay(28000000);
  }
}
