/*
 * df4iah_serial.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_SERIAL_H_
#define DF4IAH_SERIAL_H_


/* UART Baudrate */
// #define BAUDRATE 9600
// #define BAUDRATE 19200
#define BAUDRATE 115200

/* use "Double Speed Operation" */
//#define UART_DOUBLESPEED

/* use second UART on mega128 / can128 / mega162 / mega324p / mega644p */
//#define UART_USE_SECOND


#ifdef UART_DOUBLESPEED
// #define UART_CALC_BAUDRATE(baudRate) (((F_CPU*10UL) / ((baudRate) *8UL) +5)/10 -1)
#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)((F_CPU) + ((uint32_t)baudRate * 4UL)) / ((uint32_t)(baudRate) * 8UL) - 1)

// DF4IAH  @see page 178ff
#define UART_CALC_BAUDRATE(baudRate) (((((uint32_t)(F_CPU)) >> 3 + ((uint32_t)baudRate >> 1)) / ((uint32_t)baudRate)) - 1)
#else
// #define UART_CALC_BAUDRATE(baudRate) (((F_CPU*10UL) / ((baudRate)*16UL) +5)/10 -1)
//#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)((F_CPU) + ((uint32_t)baudRate * 8UL)) / ((uint32_t)(baudRate) * 16UL) - 1)

// DF4IAH  @see page 178ff
#define UART_CALC_BAUDRATE(baudRate) ((((((uint32_t)(F_CPU)) >> 4) + ((uint32_t)baudRate >> 1)) / ((uint32_t)baudRate)) - 1)
#endif


#include "chipdef.h"


void init_serial();
void close_serial();
void sendchar_serial(uint8_t data);
uint8_t recvchar_serial(void);

#endif /* DF4IAH_SERIAL_H_ */
