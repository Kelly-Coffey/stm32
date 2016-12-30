#include "stm32f3xx.h"
#include "main.h"

void SystemCoreClockUpdate(void);

void InitSystem(void);

void _init(void)
{
}

int main(void)
{
  InitSystem();
  
  while (1) {
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
    HAL_Delay(500);
  }
}

void InitSystem(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));


  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

	/* Select Clock Source  */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* Configure the SysTick to have interrupt in 1ms time basis */
  SysTick_Config(SystemCoreClock / 1000);
  
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);  
}

/**
  * @brief  SYSTICK callback.
  * @retval None
  */
__weak void SysTick_Callback(void)
{
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  SysTick_Callback();
}
  

