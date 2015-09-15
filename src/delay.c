#include "delay.h"

void Delay(uint32_t time)
{
  while (time--)
  {
    asm("nop");
  }
}
