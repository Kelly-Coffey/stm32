#include "stm32f3xx_hal.h"
#include "sysconf.h"

void USART2_UART_DMA_Init(DMA_Channel_TypeDef *dmach);
void UART2_Transmit_DMA(DMA_Channel_TypeDef *dmach, uint8_t *pData, uint16_t Size);

int main(void)
{
  SetSysClock_PLL_HSI();
  
  Init_NVIC();
  
  Init_GPIO();
  
  Init_Peripherals();
  
  USART2_UART_DMA_Init(DMA1_Channel7);

  char buff[32];
  UART2_Transmit_DMA(DMA1_Channel7, buff, 16);
  
  while (1) {
    LED3_GPIO_Port->BSRR = LED3_Pin;
    HAL_Delay(1000);
    LED3_GPIO_Port->BRR = LED3_Pin;
    HAL_Delay(1000);
  }
}

