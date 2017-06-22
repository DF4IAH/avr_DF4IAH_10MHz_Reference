/*
 * df4iah_fw_twi.h
 *
 *  Created on: 01.03.2015
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_TWI_H_
#define DF4IAH_FW_TWI_H_


#include "chipdef.h"


#define TWICTXT_TX_BUFFER_SIZE									64
#define TWI_TWSR_PRESCALER_MASK									0x07
#define TWI_TWSR_STATE_MASK										0xF8
#define TWI_TWSR_START											0x08
#define TWI_TWSR_REPEATEDSTART									0x10
#define TWI_TWSR_M_SLAW_ADDR_ACK								0x18
#define TWI_TWSR_M_SLAW_ADDR_NACK								0x20
#define TWI_TWSR_M_SLAW_DATA_ACK								0x28
#define TWI_TWSR_M_SLAW_DATA_NACK								0x30
#define TWI_TWSR_M_SLAW_ARBIT_LOST								0x38
#define TWI_TWSR_M_SLAR_ADDR_ACK								0x40
#define TWI_TWSR_M_SLAR_ADDR_NACK								0x48
#define TWI_TWSR_M_SLAR_DATA_ACK								0x50
#define TWI_TWSR_M_SLAR_DATA_NACK								0x58

#define TWI_TWSR_S_SLAW_MYADDR_RECEIVED							0x60
#define TWI_TWSR_S_SLAW_MYADDR_ARBIT_LOST						0x68
#define TWI_TWSR_S_SLAW_OMNIADDR_RECEIVED						0x70
#define TWI_TWSR_S_SLAW_OMNIADDR_ARBIT_LOST						0x78
#define TWI_TWSR_S_SLAW_MYADDR_DATA_ACK							0x80
#define TWI_TWSR_S_SLAW_MYADDR_DATA_NACK						0x88
#define TWI_TWSR_S_SLAW_OMNIADDR_DATA_ACK						0x90
#define TWI_TWSR_S_SLAW_OMNIADDR_DATA_NACK						0x98
#define TWI_TWSR_S_SLAW_STOP_REPEATEDSTART_RECEIVED				0xA0

#define TWI_TWSR_S_SLAR_MYADDR_DATA_ACK							0xA8
#define TWI_TWSR_S_SLAR_MYADDR_ARBIT_LOST						0xB0
#define TWI_TWSR_S_SLAR_OMNIADDR_DATA_ACK						0xB8
#define TWI_TWSR_S_SLAR_OMNIADDR_DATA_NACK						0xC0
#define TWI_TWSR_S_SLAR_MYADDR_LASTDATA_ACK						0xC8


enum C_SMART_LCD_MODE__ENUM {
	C_SMART_LCD_MODE_UNIQUE											= 0x00,
	C_SMART_LCD_MODE_SMARTLCD										= 0x10,
	C_SMART_LCD_MODE_REFOSC											= 0x20,
};


void twi_fw_init(void);
#if 0
void twi_fw_close(void);
#endif

void twi_fw_waitUntilDone(uint8_t extraDelay);
void twi_fw_sendCmdSendData1(uint8_t addr, uint8_t cmd, uint8_t data1);
void twi_fw_sendCmdSendData1SendData2(uint8_t addr, uint8_t cmd, uint8_t data1, uint8_t data2);
void twi_fw_sendCmdSendData1SendDataVar(uint8_t addr, uint8_t cmd, uint8_t cnt, uint8_t data[]);
uint8_t twi_fw_sendCmdReadData1(uint8_t addr, uint8_t cmd);

void isr_sendStart(uint8_t sendSignal, uint8_t isRepeatedStart);
void isr_sendStop(uint8_t sendStopSignal);

/* the following functions are direct __vector_xx calls to reduce some clocks */
//void twi_ISR_TWI(void);

#endif /* DF4IAH_FW_TWI_H_ */
