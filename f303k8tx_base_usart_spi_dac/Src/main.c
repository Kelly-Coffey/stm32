#include "stm32f3xx_hal.h"
#include "sysconf.h"

int main(void)
{
  SetSysClock_PLL_HSI();
  
  Init_NVIC();
  
  Init_GPIO();
  
  Init_Peripherals();
  
  while (1) {
    HAL_Delay(500);
  }
}

