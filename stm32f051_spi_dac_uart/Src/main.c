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
#include "stm32f0xx_hal.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "uart_serial.h"

#include "ff.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define FCC(c1,c2,c3,c4)  (((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)  /* FourCC */

const char INITMSG[] = "ST-Link UART V-Com output\n";
const char OPENDIROKMSG[] = "fatfs opendir OK\n";
const char OPENDIRNGMSG[] = "fatfs opendir FAILED\n";
const char FILENAMEMSG[] = "fatfs readdir : %s\n";
const char PLAYMSG[] = "play \n";
const char SIZEMSG[] = "size: %u\n";
const char PLAYENDMSG[] = "play end\n";
const char ENDMSG[] = "end\n";

volatile uint16_t freq;
volatile uint8_t channel;
volatile uint8_t resolution;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static
DWORD loadheader (FIL* fp)  /* 0:Invalid format, 1:I/O error, >=1024:Number of samples */
{
  DWORD sz, f;
  BYTE b;
  UINT rb;
  uint8_t buf[128];

  if (f_read(fp, buf, 12, &rb) != FR_OK) return 1;  /* Load file header (12 bytes) */

  if (rb != 12 || *(DWORD*)(&buf[8]) != FCC('W','A','V','E')) return 0;

  for (;;) {
    f_read(fp, buf, 8, &rb);                /* Get Chunk ID and size */
    if (rb != 8) return 0;
    sz = *(DWORD*)(&buf[4]);                /* Chunk size */

    switch (*(DWORD*)(&buf[0])) {           /* Switch by chunk ID */
    case FCC('f','m','t',' ') :             /* 'fmt ' chunk */
      if (sz & 1) sz++;                     /* Align chunk size */
      if (sz > 100 || sz < 16) return 0;    /* Check chunk size */
      f_read(fp, buf, sz, &rb);             /* Get content */
      if (rb != sz) return 0;
      if (buf[0] != 1) return 0;            /* Check coding type (LPCM) */
      b = buf[2];
      if (b != 1 && b != 2) return 0;       /* Mono Only */
      channel = b;
      b = buf[14];
      if (b != 8 && b != 16) return 0;      /* resolution 8bit only */
      resolution = b;

      f = *(DWORD*)(&buf[4]);               /* Check sampling freqency (8k-48k) */
      freq = f;
      break;

    case FCC('d','a','t','a') :             /* 'data' chunk */
      if (sz < 1024) return 0;              /* Check size */
      return sz;                            /* Start to play */

    case FCC('D','I','S','P') :             /* 'DISP' chunk */
    case FCC('L','I','S','T') :             /* 'LIST' chunk */
    case FCC('f','a','c','t') :             /* 'fact' chunk */
      if (sz & 1) sz++;                     /* Align chunk size */
      f_lseek(fp, fp->fptr + sz);           /* Skip this chunk */
      break;

    default :                               /* Unknown chunk */
      return 0;
    }
  }

  return 0;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  FATFS fs;
  DIR dir;
  FRESULT fres;
  FILINFO finf;

  FIL fp;
  uint16_t* dp;

  uint32_t size;
  uint32_t rb;
  uint8_t nextbuf;
  uint32_t nextsize;
  
  uint8_t buffer[2][1024];
  char text[128];
  
  char path[64] = "0:";
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
//  MX_SPI2_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  uart_serial_init(&huart1);

  printf(INITMSG);

  f_mount(&fs, "", 0);
  fres = f_opendir(&dir, path);

  if (fres == FR_OK) {
    printf(OPENDIROKMSG);
  } else {
    printf(OPENDIRNGMSG);
  }

  while (fres == FR_OK) {
    char* fn;
    int flen;
    
    fres = f_readdir(&dir, &finf);
    fn = finf.fname;
    if (fres != FR_OK || fn[0] == '\0') { break; }
    if (fn[0] == '.' || fn[0] == '_') { continue; }
    
    flen = strlen(fn);
    printf(FILENAMEMSG, fn);
    if (flen < 5 || fn[flen-4] != '.' || fn[flen-3] != 'W' || fn[flen-2] != 'A' || fn[flen-1] != 'V') {
      continue;
    }
    printf(PLAYMSG);
    
    strcpy(&(path[2]), fn);
    fres = f_open(&fp, path, FA_OPEN_EXISTING | FA_READ);
    if (fres == FR_OK) {
      size = loadheader(&fp);
      printf(SIZEMSG, size);
      break;
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
