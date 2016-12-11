/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "dac1_tim6_dma.h"
#include "spi1.h"
#include "usart2.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ff.h"
#include "string.h"

#include <errno.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

const char INITMSG[] = "ST-Link UART V-Com output\r\n";
const char OPENDIROKMSG[] = "fatfs opendir OK\r\n";
const char OPENDIRNGMSG[] = "fatfs opendir FAILED\r\n";
const char FILENAMEMSG[] = "fatfs readdir : %s\r\n";
const char PLAYMSG[] = "play \r\n";
const char SIZEMSG[] = "size: %u\r\n";
const char PLAYENDMSG[] = "play end\r\n";
const char ENDMSG[] = "end\r\n";

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
void _init(void)
{
}

caddr_t _sbrk_r (struct _reent *r, int incr)
{
  register char* stack_ptr asm ("sp");
  extern char    end asm ("end"); /* Defined by the linker.  */
  static char*   heap_end;
  char*          prev_heap_end;

  if (heap_end == NULL)
    heap_end = & end;
  
  prev_heap_end = heap_end;
  
  if (heap_end + incr > stack_ptr)
  {
      errno = ENOMEM;
      return (caddr_t) -1;
  }
  
  heap_end += incr;

  return (caddr_t) prev_heap_end;
}

static
DWORD loadheader (FIL* fp)	/* 0:Invalid format, 1:I/O error, >=1024:Number of samples */
{
  DWORD sz, f;
  BYTE b;
  UINT rb;
  uint8_t buf[128];

  if (f_read(fp, buf, 12, &rb) != FR_OK) return 1;	/* Load file header (12 bytes) */

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

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/**
  * @brief  Configures the System clock frequency, AHB/APBx prescalers and Flash
  *         settings.
  * @note   This function should be called only once the RCC clock configuration
  *         is reset to the default reset state (done in SystemInit() function).
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
/******************************************************************************/
/*            PLL (clocked by HSI) used as System clock source                */
/******************************************************************************/

  /* At this stage the HSI is already enabled and used as System clock source */
  
  /* SYSCLK, HCLK, PCLK configuration ----------------------------------------*/

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
  /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
  __HAL_FLASH_SET_LATENCY(FLASH_ACR_LATENCY_0);

  /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
  __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_HSICALIBRATION_DEFAULT);
 
  /* Configure the main PLL clock source and multiplication factor. */
   __HAL_RCC_PLL_CONFIG(RCC_PLLSOURCE_HSI, RCC_CFGR_PLLMUL6);


  /* Enable the main PLL. */
  __HAL_RCC_PLL_ENABLE();

  /* Wait till PLL is ready */
  while( !(RCC->CR & RCC_CR_PLLRDY) ) ;

  /* HCLK = SYSCLK */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_SYSCLK_DIV1);

  /* Select PLL as system clock source */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

  /* Wait till PLL is used as system clock source */
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) ;

  /* PCLK1 = HCLK / 2 */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV2);

  /* PCLK2 = HCLK */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_HCLK_DIV1) << 3));
    
  SystemCoreClock = (HSI_VALUE / 2) * 6;

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t tick;

  FATFS fs;
  DIR dir;
  FRESULT fres;
  FILINFO finf;

  FIL fp;
  uint16_t* dp;

  unsigned int size;
  unsigned int rb;
  int nextbuf;
  unsigned int nextsize;
  
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

  /* USER CODE BEGIN 2 */

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  MX_DAC1_Init();
  USART2_UART_Init();

  UART2_Transmit_String(INITMSG);
  
  f_mount(&fs, "", 0);
  fres = f_opendir(&dir, path);
  if (fres == FR_OK) {
    UART2_Transmit_String(OPENDIROKMSG);
  } else {
    UART2_Transmit_String(OPENDIRNGMSG);
  }
  
  while (fres == FR_OK) {
    char* fn;
    int flen;
    
    fres = f_readdir(&dir, &finf);
    fn = finf.fname;
    if (fres != FR_OK || fn[0] == '\0') { break; }
    if (fn[0] == '.' || fn[0] == '_') { continue; }
    
    flen = strlen(fn);
    sprintf(text, FILENAMEMSG, fn);
    UART2_Transmit_String(text);
    if (flen < 5 || fn[flen-4] != '.' || fn[flen-3] != 'W' || fn[flen-2] != 'A' || fn[flen-1] != 'V') {
      continue;
    }
    UART2_Transmit_String(PLAYMSG);
    
    strcpy(&(path[2]), fn);
    fres = f_open(&fp, path, FA_OPEN_EXISTING | FA_READ);
    if (fres == FR_OK) {
      size = loadheader(&fp);
      sprintf(text, SIZEMSG, size);
      UART2_Transmit_String(text);
      break;
    }
  }

  unsigned int pad = 512 - (fp.fptr % 512);
  if (pad == 0) pad = 512;

  f_read(&fp, (uint8_t*)(buffer[0]), pad, &rb);
  dp = (uint16_t*)(buffer[0]);
  for (int i=0; i<rb; i+=2, dp++) {
    *dp = (*dp & 0xfff0) + 0x8000;
  }
  size -= pad;
  f_read(&fp, (uint8_t*)(buffer[1]), 1024, &rb);
  dp = (uint16_t*)(buffer[1]);
  for (int i=0; i<rb; i+=2, dp++) {
    *dp = (*dp & 0xfff0) + 0x8000;
  }
  size -= 1024;
  nextbuf = 1;
  nextsize = 512;
  
  TIM6_Init(freq);

  /* Enable the TIM peripheral. */
  TIM6->CR1 |= TIM_CR1_CEN;

  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)(buffer[0]), (pad>>1), DAC_ALIGN_12B_L);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  tick = HAL_GetTick();
  
  while (1)
  {
    if (HAL_DMA_GetState(hdac1.DMA_Handle1) == HAL_DMA_STATE_READY) {
      if (nextsize) {
        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)(buffer[nextbuf]), nextsize, DAC_ALIGN_12B_L);
      } else {
        UART2_Transmit_String(PLAYENDMSG);
        break;
      }
      
      if (size) {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);

        nextbuf = 1-nextbuf;      
        nextsize = (size >= 1024) ? 1024 : size;
        f_read(&fp, (uint8_t*)(buffer[nextbuf]), nextsize, &rb);

        if (rb != nextsize) {
          sprintf(text, "size:%u nextsize:%u rb:%u\r\n", size, nextsize, rb);
          UART2_Transmit_String(text);
          break;
        }
        size -= rb;
        nextsize >>= 1;
      
        dp = (uint16_t*)(buffer[nextbuf]);
        for (int i=0; i<rb; i+=2, dp++) {
          *dp = (*dp & 0xfff0) + 0x8000;
        }
      } else {
        nextsize = 0;
      }
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
    }

    while (UART2_IsDataReceived()) {
      uint8_t c = UART2_Receive();
      UART2_Transmit(c);
      if (c == '\r') {
       UART2_Transmit('\n');
      }
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);

  /* Disable the TIM peripheral. */
  TIM6->CR1 &= ~TIM_CR1_CEN;

  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  
  HAL_DAC_DeInit(&hdac1);
  TIM6_DeInit();

  SPI1_DeInit();
  
  /* DMA controller clock disable */
  __HAL_RCC_DMA1_CLK_DISABLE();
  
  UART2_Transmit_String(ENDMSG);

  while (1)
  {}

#if 0
  /* USER CODE END 3 */

/* USER CODE BEGIN 4 */
#endif
}

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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
