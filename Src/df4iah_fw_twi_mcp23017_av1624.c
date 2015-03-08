/*
 * df4iah_fw_twi_mcp23017_av1624.c
 *
 *  Created on: 05.03.2015
 *      Author: DF4IAH, Ulrich Habel
 */
// tabsize: 4

#include <stdbool.h>
#include <util/delay.h>

#include <avr/wdt.h>

#include "df4iah_fw_main.h"
#include "df4iah_fw_twi_mcp23017.h"

#include "df4iah_fw_twi_mcp23017_av1624.h"


extern volatile main_bf_t main_bf;
//extern xxx twiMcpAvXxx;


static uint8_t recodeChar(uint8_t inChar)
{
	uint8_t outChar = inChar;

	return outChar;
}


void twi_mcp23017_av1624_fw_init()
{
	/*
	 * 	Port-A  LCD-assignment:
	 * 	PA0		LCD- DB0
	 * 	PA1		LCD- DB1
	 * 	PA2		LCD- DB2
	 * 	PA3		LCD- DB3
	 * 	PA4		LCD- DB4
	 * 	PA5		LCD- DB5
	 * 	PA6		LCD- DB6
	 * 	PA7		LCD- DB7
	 *
	 * 	Port-B  LCD-assignment:
	 * 	PB0		LCD- E
	 * 	PB1		LCD- R/!W
	 * 	PB2		LCD- RS
	 * 	PB3		LCD- Light
	 * 	PB4		(free)
	 * 	PB5		(free)
	 * 	PB6		(free)
	 * 	PB7		(free)
	 */

	// wait > 30 ms --> done

	twi_mcp23017_av1624_fw_waitUntilReady();
	if (!(main_bf.mainIsLcdAttached)) {
		return;
	}


	// set interface width to 8bits - (1)
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00111100);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00111100);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00111100);
	twi_mcp23017_fw_setPortA_DirOut(false);

	// > 4.1 µs
	_delay_us(4.1f);

	// set interface width to 8bits - (2)
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00111100);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00111100);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00111100);
	twi_mcp23017_fw_setPortA_DirOut(false);

	// > 100 µs
	_delay_us(100.0f);

	// set interface width to 8bits - (3)
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00111100);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00111100);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00111100);

	// set interface width to 8bits - (4)
	twi_mcp23017_av1624_fw_waitUntilReady();
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00111100);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00111100);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00111100);


	// display ON
	twi_mcp23017_av1624_fw_waitUntilReady();
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00001100);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00001100);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00001100);

	// display CLEAR
	twi_mcp23017_av1624_fw_waitUntilReady();
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00000001);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00000001);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00000001);

	// display Entry Mode Set
	twi_mcp23017_av1624_fw_waitUntilReady();
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00000110);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00000110);
	twi_mcp23017_fw_setPortBA(0b1000, 0b00000110);
	twi_mcp23017_fw_setPortA_DirOut(false);
}

void twi_mcp23017_av1624_fw_close()
{
	if (!(main_bf.mainIsLcdAttached)) {
		return;
	}


	// display OFF
	twi_mcp23017_av1624_fw_waitUntilReady();
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00001000);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00001000);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00001000);
	twi_mcp23017_fw_setPortA_DirOut(false);

	// display CLEAR
	twi_mcp23017_av1624_fw_waitUntilReady();
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00000001);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00000001);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00000001);
	twi_mcp23017_fw_setPortA_DirOut(false);
}


void twi_mcp23017_av1624_fw_waitUntilReady()
{
	twi_mcp23017_fw_setPortA_DirOut(false);

	for (;;) {
		twi_mcp23017_fw_setPortB(0b1010);
		twi_mcp23017_fw_setPortB(0b1011);
		uint8_t status = twi_mcp23017_fw_readPortA();
		twi_mcp23017_fw_setPortB(0b1010);

		if (!(status & 0x80)) {
			break;
		}

		_delay_us(10);
	    wdt_reset();
		usbPoll();
	}
}

void twi_mcp23017_av1624_fw_gotoPosition(uint8_t line, uint8_t column)
{
	uint8_t ddramAdrCmd = (0x80 | ((line % 2) << 6) | (column % 16));

	twi_mcp23017_av1624_fw_waitUntilReady();

	// set DDRAM address
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b1000, ddramAdrCmd);
	twi_mcp23017_fw_setPortBA(0b1001, ddramAdrCmd);
	twi_mcp23017_fw_setPortBA(0b1000, ddramAdrCmd);
	twi_mcp23017_fw_setPortA_DirOut(false);
}

void twi_mcp23017_av1624_fw_writeString(const uint8_t* buffer, uint8_t len)
{
	for (int idx = 0; idx < len; ++idx) {
		uint8_t c = buffer[idx];
		c = recodeChar(c);

		twi_mcp23017_av1624_fw_waitUntilReady();

		// write data
		twi_mcp23017_fw_setPortA_DirOut(true);
		twi_mcp23017_fw_setPortBA(0b1100, c);
		twi_mcp23017_fw_setPortBA(0b1101, c);
		twi_mcp23017_fw_setPortBA(0b1100, c);
		twi_mcp23017_fw_setPortBA(0b1000, c);
		twi_mcp23017_fw_setPortA_DirOut(false);
	}
}
