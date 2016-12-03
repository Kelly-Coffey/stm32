#include "main.h"
#include "diskio.h"
#include "stm32f3xx_hal.h"
#include "ff.h"

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

extern TIM_HandleTypeDef htim6;
extern DAC_HandleTypeDef hdac1;
extern UART_HandleTypeDef huart2;

BYTE buffer[1024];

volatile UINT fifo_ct;
volatile UINT fifo_ri;

volatile uint16_t freq;
volatile uint8_t channel;
volatile uint8_t resolution;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim6.Instance) {
    if (fifo_ct == 0) {
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
      /* underflow */
      return;
    }

    UINT ri = fifo_ri;
    WORD dat = (*(WORD*)(&buffer[ri]) & 0xfff0) + 0x8000;
    
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_L, dat);
    
    fifo_ct -= 2;
    fifo_ri = (ri + 2) & (1024-1);
  }
}

void startplay(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  DAC_ChannelConfTypeDef sConfig;

  HAL_UART_Transmit(&huart2, (uint8_t*)"start play\r\n", 12, 5000);
  
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 24000000L / freq;
  HAL_TIM_Base_Init(&htim6);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  MX_DAC1_Init();
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0x80);

  HAL_TIM_Base_Start_IT(&htim6);
}

void stopplay(void)
{
  HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_MspDeInit(&hdac1);

  HAL_TIM_Base_Stop_IT(&htim6);
  HAL_TIM_Base_DeInit(&htim6);
}

static
DWORD loadheader (FIL* fp)	/* 0:Invalid format, 1:I/O error, >=1024:Number of samples */
{
  DWORD sz, f;
  BYTE b;
  UINT rb;

  if (f_read(fp, buffer, 12, &rb) != FR_OK) return 1;	/* Load file header (12 bytes) */

  if (rb != 12 || *(DWORD*)(&buffer[8]) != FCC('W','A','V','E')) return 0;

	for (;;) {
		f_read(fp, buffer, 8, &rb);					/* Get Chunk ID and size */
		if (rb != 8) return 0;
		sz = *(DWORD*)(&buffer[4]);					/* Chunk size */

		switch (*(DWORD*)(&buffer[0])) {				/* Switch by chunk ID */
		case FCC('f','m','t',' ') :					/* 'fmt ' chunk */
			if (sz & 1) sz++;						/* Align chunk size */
			if (sz > 100 || sz < 16) return 0;		/* Check chunk size */
			f_read(fp, buffer, sz, &rb);			/* Get content */
			if (rb != sz) return 0;
			if (buffer[0] != 1) return 0;			/* Check coding type (LPCM) */
			b = buffer[2];
			if (b != 1 && b != 2) return 0;			/* Mono Only */
			channel = b;
			b = buffer[14];
			if (b != 8 && b != 16) return 0;		/* resolution 8bit only */
			resolution = b;

			f = *(DWORD*)(&buffer[4]);				/* Check sampling freqency (8k-48k) */
			freq = f;
			break;

		case FCC('d','a','t','a') :				/* 'data' chunk */
			if (sz < 1024) return 0;				/* Check size */
			return sz;								/* Start to play */

		case FCC('D','I','S','P') :				/* 'DISP' chunk */
		case FCC('L','I','S','T') :				/* 'LIST' chunk */
		case FCC('f','a','c','t') :				/* 'fact' chunk */
			if (sz & 1) sz++;						/* Align chunk size */
			f_lseek(fp, fp->fptr + sz);				/* Skip this chunk */
			break;

		default :								/* Unknown chunk */
			return 0;
		}
	}

	return 0;
}

void playwav(FIL* fp)
{
  DWORD size;
  UINT rsize;
  UINT rb;
  UINT wi;

  size = loadheader(fp);

  if (size < 512) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"can't read wav\r\n", 16, 5000);
    return;
  }

  wi = 0;

  UINT pad = 512 - (fp->fptr % 512);
  if (pad == 0) pad = 512;
  fifo_ri = 1024 - pad;

  f_read(fp, &buffer[fifo_ri], pad, &rb);
  if (pad != rb) return;
  fifo_ct = rb;
  size -= rb;

  rsize = 512;

  f_read(fp, &buffer[0], rsize, &rb);
  if (rb != rsize) return;

  size -= rb;
  wi += rb;
  fifo_ct += rb;

  startplay();

  while (size || fifo_ct >= 4) {

    if (size && fifo_ct <= 512) {
      rsize = (size >= 512) ? 512 : size;
      f_read(fp, &buffer[wi], rsize, &rb);
      if (rb != rsize) break;
      size -= rb;
      wi = (wi + rb) & (1024 - 1);

      HAL_NVIC_DisableIRQ(TIM6_DAC1_IRQn);
      fifo_ct += rb;
      HAL_NVIC_EnableIRQ(TIM6_DAC1_IRQn);
		}
	}

	stopplay();
	f_close(fp);

	return;
}
