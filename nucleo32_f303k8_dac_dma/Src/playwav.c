#include "main.h"
#include "stm32f3xx_hal.h"
#include "dma.h"
#include "dac.h"
#include "diskio.h"
#include "ff.h"

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

extern TIM_HandleTypeDef htim6;
extern DAC_HandleTypeDef hdac1;
extern UART_HandleTypeDef huart2;

const uint16_t sin_table[36] = {
  0x8000, 0x963A, 0xABC7, 0xBFFF, 0xD246, 0xE20D, 0xEED9, 0xF847,
  0xFE0D, 0xFFFF, 0xFE0D, 0xF847, 0xEED9, 0xE20D, 0xD246, 0xBFFF,
  0xABC7, 0x963A, 0x8000, 0x69C6, 0x5439, 0x4000, 0x2DBA, 0x1DF3,
  0x1127, 0x07B9, 0x01F3, 0x0001, 0x01F3, 0x07B9, 0x1127, 0x1DF3,
  0x2DBA, 0x4000, 0x5439, 0x69C6
};

uint8_t buffer[2][1024];
UINT dsize[2];
volatile uint8_t bufsel;

volatile uint16_t freq;
volatile uint8_t channel;
volatile uint8_t resolution;

void startplay(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  HAL_UART_Transmit(&huart2, (uint8_t*)"start play\r\n", 12, 5000);
  
  MX_DAC1_Init();

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 12000000L / freq;
  HAL_TIM_Base_Init(&htim6);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_Base_Start(&htim6);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)(buffer[bufsel]), dsize[bufsel], DAC_ALIGN_12B_L);
}

void stopplay(void)
{
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_MspDeInit(&hdac1);

  HAL_TIM_Base_DeInit(&htim6);
}

static
DWORD loadheader (FIL* fp)	/* 0:Invalid format, 1:I/O error, >=1024:Number of samples */
{
  DWORD sz, f;
  BYTE b;
  UINT rb;

  if (f_read(fp, buffer[0], 12, &rb) != FR_OK) return 1;	/* Load file header (12 bytes) */

  if (rb != 12 || *(DWORD*)(&buffer[0][8]) != FCC('W','A','V','E')) return 0;

	for (;;) {
		f_read(fp, buffer[0], 8, &rb);					/* Get Chunk ID and size */
		if (rb != 8) return 0;
		sz = *(DWORD*)(&buffer[0][4]);					/* Chunk size */

		switch (*(DWORD*)(&buffer[0][0])) {				/* Switch by chunk ID */
		case FCC('f','m','t',' ') :					/* 'fmt ' chunk */
			if (sz & 1) sz++;						/* Align chunk size */
			if (sz > 100 || sz < 16) return 0;		/* Check chunk size */
			f_read(fp, buffer, sz, &rb);			/* Get content */
			if (rb != sz) return 0;
			if (buffer[0][0] != 1) return 0;			/* Check coding type (LPCM) */
			b = buffer[0][2];
			if (b != 1 && b != 2) return 0;			/* Mono Only */
			channel = b;
			b = buffer[0][14];
			if (b != 8 && b != 16) return 0;		/* resolution 8bit only */
			resolution = b;

			f = *(DWORD*)(&buffer[0][4]);				/* Check sampling freqency (8k-48k) */
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
  UINT i;

  uint16_t* p = (uint16_t*)(buffer[0]);
  for (int c = 0; c < 7; c++) {
    for (int a=0; a<36; a++) {
      *p++ = sin_table[a];
    }
  }
  dsize[0] = 36*7;
  
  size = loadheader(fp);

  if (size < 512) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"can't read wav\r\n", 16, 5000);
    return;
  }

/*
  UINT pad = 512 - (fp->fptr % 512);
  if (pad == 0) pad = 512;

  f_read(fp, buffer[0], pad, &rb);
  if (pad != rb) return;
  
  uint16_t *dp = (uint16_t*)(&buffer[0][0]);
  for (i=0; i<rb; i+=2, dp++) {
    *dp = (*dp & 0xfff0) + 0x8000;
  }

  size -= rb;
  dsize[0] = rb;

  rsize = 1024;

  f_read(fp, buffer[1], rsize, &rb);
  if (rb != rsize) return;

  dp = (uint16_t*)(&buffer[1][0]);
  for (i=0; i<rb; i+=2, dp++) {
    *dp = (*dp & 0xfff0) + 0x8000;
  }
  size -= rb;
  dsize[1] = rb;
*/
  bufsel = 0;
  startplay();

  while (size || (HAL_DMA_GetState(hdac1.DMA_Handle1) == HAL_DMA_STATE_BUSY)) {

    if (size && (HAL_DMA_GetState(hdac1.DMA_Handle1) == HAL_DMA_STATE_READY)) {
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
//      dsize[bufsel] = 0;
//      if (dsize[1-bufsel]) {
//        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)(buffer[bufsel]), dsize[bufsel], DAC_ALIGN_12B_L);
//        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)(buffer[1-bufsel]), dsize[1-bufsel], DAC_ALIGN_12B_L);
/*
        rsize = (size >= 1024) ? 1024 : size;
        f_read(fp, buffer[bufsel], rsize, &rb);
        if (rb != rsize) break;
        dsize[bufsel] = rb;
        dp = (uint16_t*)(&buffer[bufsel][0]);
        for (i=0; i<rb; i+=2, dp++) {
          *dp = (*dp & 0xfff0) + 0x8000;
        }
        size -= rb;
        bufsel = 1-bufsel;
      }
*/
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
		}
	}

	stopplay();
	f_close(fp);

	return;
}
