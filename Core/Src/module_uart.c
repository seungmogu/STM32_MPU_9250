/*
 * module_uart.c
 *
 *  Created on: Oct 25, 2021
 *      Author: seung
 */

#include "module_uart.h"
#include <string.h>
#include "stdio.h"

void init_uart(uart_t* u)
{
  u->head = 0;
  u->tail = 0;
  memset(u->buffer, 0, sizeof(u->buffer));
}

void push(uart_t* u, uint8_t data)
{
  u->buffer[u->head] = data;

  u->head++;

  if (u->head >= MAX_BUFFER_SIZE) {
    u->head = 0;
  }
}

uint8_t pop(uart_t* u)
{
  uint8_t data = u->buffer[u->tail];

  u->tail++;

  if (u->tail >= MAX_BUFFER_SIZE) {
    u->tail = 0;
  }

  return data;
}

uint8_t isEmpty(uart_t* u)
{
  return u->head == u->tail;
}
