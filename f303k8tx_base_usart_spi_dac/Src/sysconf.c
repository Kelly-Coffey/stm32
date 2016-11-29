#include "stm32f3xx_hal.h"
#include "sysconf.h"

void SetSysClock_PLL_HSI(void)
{
  /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
  __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(16);

  /*
    Additional consideration on the SYSCLK based on Latency settings:
        +-----------------------------------------------+
        | Latency       | SYSCLK clock frequency (MHz)  |
        |---------------|-------------------------------|
        |0WS(1CPU cycle)|       0 < SYSCLK <= 24        |
        |---------------|-------------------------------|
        |1WS(2CPU cycle)|      24 < SYSCLK <= 48        |
        |---------------|-------------------------------|
        |2WS(3CPU cycle)|      48 < SYSCLK <= 72        |
        +-----------------------------------------------+
  */
  FLASH->ACR = FLASH_ACR_PRFTBE | (uint32_t)FLASH_LATENCY_0;

  /* Configure the main PLL clock source and multiplication factor. */
  MODIFY_REG(RCC->CFGR,
              RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2,
              RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL6
              | RCC_CFGR_HPRE_DIV1      /* HCLK   = 24MHz */
              | RCC_CFGR_PPRE1_DIV2     /* PCLK1  = 12MHz */
              | RCC_CFGR2_PREDIV_DIV1); /* PCLK2  = 24MHz */

  /* Enable the main PLL. */
  __HAL_RCC_PLL_ENABLE();
  
  /* Wait till PLL is ready */
  while( !(RCC->CR & RCC_CR_PLLRDY) ) ;

  /* Select PLL as System Clock Source */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_PLL, RCC_SYSCLKSOURCE_PLLCLK);

  /* Wait till PLL is used as System Clock Source */
  while ((uint32_t)(RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) ;
  
  /* Update the SystemCoreClock global variable */
  SystemCoreClockUpdate();

  /* Configure the SysTick to have interrupt in 1ms time basis*/
  SysTick_Config(SystemCoreClock / 1000);
  
  /* Configure the Systick */
  SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;  
}

void Init_NVIC(void)
{
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
                    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
}

void Init_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  /*Configure GPIOA pin Output Level */
  GPIOA->BSRR = SPI_CS_Pin|AMP_EN_Pin;

  /*Configure GPIOB pin Output Level */
  GPIOB->BSRR = LED3_Pin;

  /*Configure GPIOA pins : SPI_CS_Pin AMP_EN_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|AMP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIOB pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Init_Peripherals(void)
{
  /* To use IOREMAP */
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

void _init(void)
{
  return;
}

