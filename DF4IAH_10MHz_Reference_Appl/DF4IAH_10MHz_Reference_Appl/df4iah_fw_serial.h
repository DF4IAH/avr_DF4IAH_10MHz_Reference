/*
 * df4iah_fw_serial.h
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_SERIAL_H_
#define DF4IAH_FW_SERIAL_H_


#define SERIALCTXT_TX_BUFFER_SIZE									64
#define SERIALCTXT_RX_BUFFER_SIZE									120
#define SERIALCTXT_NMEA_RX_HOOK_SIZE								8

#define SERIAL_CTXT_BUFFER_STATE_BLOCK								1
#define SERIAL_CTXT_BUFFER_STATE_SEND								2

/* UART Baudrate */
#define DEFAULT_BAUDRATE 											9600
#define DEFAULT_DATABITS											8
#define DEFAULT_PARITY_N0_E2_O3										0
#define DEFAULT_STOPBITS											1

/* use "Double Speed Operation" */
//#define UART_DOUBLESPEED

/* use second UART on mega128 / can128 / mega162 / mega324p / mega644p */
//#define UART_USE_SECOND


#ifdef UART_DOUBLESPEED
// DF4IAH  @see page 178ff
# define UART_CALC_BAUDRATE(baudRate) ((((((uint32_t)(F_CPU)) >> 3) + ((uint32_t)baudRate >> 1)) / ((uint32_t)baudRate)) - 1)
#else
// DF4IAH  @see page 178ff
# define UART_CALC_BAUDRATE(baudRate) ((((((uint32_t)(F_CPU)) >> 4) + ((uint32_t)baudRate >> 1)) / ((uint32_t)baudRate)) - 1)
#endif


#include "chipdef.h"


void serial_fw_serRxIsrOn(uint8_t flag);

void serial_fw_init(void);
void serial_fw_close(void);

void serial_fw_setCommBaud(uint16_t baud);
uint8_t serial_fw_isTxRunning(void);
void serial_fw_copyAndSendNmea(uint8_t isPgm, const uchar inData[], uint8_t len);
void serial_fw_pullAndSendNmea_havingSemaphore(uint8_t isSend);

/* the following functions are direct __vector_xx calls to reduce some clocks */
//void serial_ISR_RXC0(void);
//void serial_ISR_UDRE0(void);
//void serial_ISR_TXC0(void);

#endif /* DF4IAH_FW_SERIAL_H_ */
