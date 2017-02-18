#ifndef __UART_SERIAL_H
#define __UART_SERIAL_H

#include "stm32f3xx_hal.h"

extern void uart_serial_init(UART_HandleTypeDef *huart);
extern void uart_transmit(void);
extern void uart_put(uint8_t c);
extern void uart_write(const uint8_t *buf);

#endif
