/*
 * module_uart.h
 *
 *  Created on: Oct 27, 2021
 *      Author: seung
 */

#ifndef INC_MODULE_UART_H_
#define INC_MODULE_UART_H_

#include "stdio.h"
#define MAX_BUFFER_SIZE    (255)

typedef struct{
  uint8_t head;
  uint8_t tail;
  uint8_t buffer[MAX_BUFFER_SIZE];
}uart_t;

void push(uart_t*, uint8_t);
uint8_t pop(uart_t*);
uint8_t isEmpty(uart_t*);

#endif /* INC_MODULE_UART_H_ */
