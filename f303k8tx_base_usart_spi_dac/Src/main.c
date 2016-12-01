#include "stm32f3xx_hal.h"
#include "sysconf.h"

#include <string.h>

void USART2_UART_DMA_Init(DMA_Channel_TypeDef *dmach);
void UART2_Transmit_DMA(DMA_Channel_TypeDef *dmach, uint8_t *pData, uint16_t Size);

const char msg[] = "STM32F303K8T6\r\n";

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

int main(void)
{
  SetSysClock_PLL_HSI();
  
  Init_NVIC();
  
  Init_GPIO();
  
  Init_Peripherals();
  
  USART2_UART_DMA_Init(DMA1_Channel7);

  huart2.Instance = USART2;

  hdma_usart2_tx.Instance = DMA1_Channel7;

  hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart2_tx.Init.Mode = DMA_NORMAL;
  hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;

  huart2.hdmatx = &hdma_usart2_tx;
  hdma_usart2_tx.Parent = &huart2;
    
  UART2_Transmit_DMA(DMA1_Channel7, (uint8_t *)msg, strlen(msg));
   
  while (1) {
    LED3_GPIO_Port->BSRR = LED3_Pin;
    HAL_Delay(1000);
    LED3_GPIO_Port->BRR = LED3_Pin;
    HAL_Delay(1000);
  }
}


