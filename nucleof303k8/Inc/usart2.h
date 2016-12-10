
#ifndef __usart_H
#define __usart_H

#include "stm32f3xx_hal.h"
#include "main.h"

extern void Error_Handler(void);

void USART2_UART_Init(void);
void UART2_Init(void);
void UART2_DeInit(void);

void UART2_Transmit(uint8_t ch);
void UART2_Transmit_String(const char *buff);
uint8_t UART2_IsDataReceived(void);
uint8_t UART2_Receive(void);
#endif /*__ usart_H */

