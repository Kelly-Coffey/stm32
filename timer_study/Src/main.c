/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usart2.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const uint8_t digdata[] = {0x5f, 0x41, 0x9d, 0xcd, 0xc3, 0xce, 0xde, 0x45, 0xdf, 0xcf};
__IO uint8_t leddat[] = {0x00, 0x00, 0x00, 0x00};

__IO uint16_t count = 0;
__IO uint8_t tim_to_led = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void _init(void)
{
}

static void Output_Digit(uint8_t dat, __IO uint32_t* brr)
{
  LED_DAT_GPIO_Port->BSRR = (LED_DAT_Pin << 16) | ((dat & 0x80) << 1);
  LED_SCK_GPIO_Port->BSRR = LED_SCK_Pin;

  LED_DAT_GPIO_Port->BSRR = ((LED_DAT_Pin | LED_SCK_Pin)<< 16) | ((dat & 0x40) << 2);
  LED_SCK_GPIO_Port->BSRR = LED_SCK_Pin;

  LED_DAT_GPIO_Port->BSRR = ((LED_DAT_Pin | LED_SCK_Pin)<< 16) | ((dat & 0x20) << 3);
  LED_SCK_GPIO_Port->BSRR = LED_SCK_Pin;

  LED_DAT_GPIO_Port->BSRR = ((LED_DAT_Pin | LED_SCK_Pin)<< 16) | ((dat & 0x10) << 4);
  LED_SCK_GPIO_Port->BSRR = LED_SCK_Pin;

  LED_DAT_GPIO_Port->BSRR = ((LED_DAT_Pin | LED_SCK_Pin)<< 16) | ((dat & 0x08) << 5);
  LED_SCK_GPIO_Port->BSRR = LED_SCK_Pin;

  LED_DAT_GPIO_Port->BSRR = ((LED_DAT_Pin | LED_SCK_Pin)<< 16) | ((dat & 0x04) << 6);
  LED_SCK_GPIO_Port->BSRR = LED_SCK_Pin;

  LED_DAT_GPIO_Port->BSRR = ((LED_DAT_Pin | LED_SCK_Pin)<< 16) | ((dat & 0x02) << 7);
  LED_SCK_GPIO_Port->BSRR = LED_SCK_Pin;

  LED_DAT_GPIO_Port->BSRR = ((LED_DAT_Pin | LED_SCK_Pin)<< 16) | ((dat & 0x01) << 8);
  LED_SCK_GPIO_Port->BSRR = LED_SCK_Pin;

  LED_SCK_GPIO_Port->BRR = LED_SCK_Pin;

  *brr = LED_DAT_Pin;
  LED_RCK_GPIO_Port->BSRR = LED_RCK_Pin;
  LED_RCK_GPIO_Port->BRR  = LED_RCK_Pin;
}

static void setled(uint16_t d)
{
  leddat[0] = digdata[d % 10];
  leddat[1] = digdata[(d / 10) % 10];
  leddat[2] = digdata[(d / 100) % 10];
  leddat[3] = (d < 1000) ? 0x00 : digdata[(d / 1000) % 10];
}

/* TIM2 : 100Hz Trigger Output from Internal Clock */
/* OC1, OC2, OC3, OC4 : each 2.5ms outout compare */
void TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  TIM2->CR1  = TIM_COUNTERMODE_UP | TIM_CLOCKDIVISION_DIV1;
  TIM2->CR2  = TIM_TRGO_UPDATE;
  TIM2->SMCR = 0;           /* ClockSource Internal */
  TIM2->DIER = 0;           /* interrupt disable */

  /* Frozen, Preload disabled */
  TIM2->CCMR1 = 0;
  TIM2->CCMR2 = 0;

  TIM2->CCR1 = 1;
  TIM2->CCR2 = 21;
  TIM2->CCR3 = 41;
  TIM2->CCR4 = 61;

  /* CCxP = 0: active high, CCxE = 0: output compare enable */
  TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

  TIM2->PSC = 1000-1;
  TIM2->ARR = 80-1;
  TIM2->EGR = TIM_EGR_UG;
  do {} while ( !(TIM2->SR & TIM_SR_UIF) );

  /* clear all flag */
  TIM2->SR = 0;
}

/* TIM3 : salve counter (use TIM2 as a presacler) */
void TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();

  TIM3->CR1  = TIM_COUNTERMODE_UP | TIM_CLOCKDIVISION_DIV1;
  TIM3->CR2  = 0;
  /* ClockSource External1, ITR1 (TIM2) */
  TIM3->SMCR = TIM_SLAVEMODE_EXTERNAL1 | TIM_TS_ITR1;

  TIM3->DIER = 0;           /* interrupt disable */

  TIM3->PSC = 0;
  TIM3->ARR = 60000-1;
  TIM3->EGR = TIM_EGR_UG;
  do {} while ( !(TIM3->SR & TIM_SR_UIF) );

  /* clear all flag */
  TIM3->SR = 0;
}

void init_start_led(void)
{
  Output_Digit(0x00, &LED_DAT_GPIO_Port->BSRR);
  Output_Digit(0x00, &LED_DAT_GPIO_Port->BSRR);
  Output_Digit(0x00, &LED_DAT_GPIO_Port->BSRR);
  Output_Digit(0x00, &LED_DAT_GPIO_Port->BSRR);

  TIM2_Init();
  TIM3_Init();

  TIM3->CR1 |= TIM_CR1_CEN;
  TIM2->CR1 |= TIM_CR1_CEN;
}

void led_drive(void)
{
  if (TIM3->SR & TIM_SR_UIF) {
    TIM3->SR = ~(TIM_SR_UIF);
    TIM3->CNT = TIM3->CNT + 10000;
  }

  if (TIM2->SR & TIM_SR_UIF) {
    TIM2->SR = ~(TIM_SR_UIF);
    if (tim_to_led) {
      count = TIM3->CNT;
      setled(count);
    }
  }

  if (TIM2->SR & TIM_SR_CC1IF) {
    TIM2->SR = ~(TIM_SR_CC1IF);
     Output_Digit(leddat[0], &LED_DAT_GPIO_Port->BRR);
  } else
  if (TIM2->SR & TIM_SR_CC2IF) {
    TIM2->SR = ~(TIM_SR_CC2IF);
    Output_Digit(leddat[1], &LED_DAT_GPIO_Port->BSRR);
  } else
  if (TIM2->SR & TIM_SR_CC3IF) {
    uint8_t dat;
    TIM2->SR = ~(TIM_SR_CC3IF);
    dat = leddat[2];
    if ((count % 100) < 50) {
      dat |= 0x20;
    }
    Output_Digit(dat, &LED_DAT_GPIO_Port->BSRR);
  } else
  if (TIM2->SR & TIM_SR_CC4IF) {
    TIM2->SR = ~(TIM_SR_CC4IF);
    Output_Digit(leddat[3], &LED_DAT_GPIO_Port->BSRR);
  }
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  count = 0;
  tim_to_led = 1;
  setled(count);
  init_start_led();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    led_drive();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
