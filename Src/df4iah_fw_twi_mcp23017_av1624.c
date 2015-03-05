/*
 * df4iah_fw_twi_mcp23017_av1624.c
 *
 *  Created on: 05.03.2015
 *      Author: DF4IAH, Ulrich Habel
 */
// tabsize: 4

#include <avr/delay.h>

#include "df4iah_fw_main.h"
#include "df4iah_fw_twi_mcp23017.h"

#include "df4iah_fw_twi_mcp23017_av1624.h"


//extern xxx twiMcpAvXxx;


void twi_mcp23017_av1624_fw_init()
{
	// wait > 30 ms --> done

	// set interface width to 8bits - (1)
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00110000);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00110000);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00110000);
	twi_mcp23017_fw_setPortA_DirOut(false);

	// > 4.1 µs
	_delay_us(4.1f);

	// set interface width to 8bits - (2)
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00110000);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00110000);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00110000);
	twi_mcp23017_fw_setPortA_DirOut(false);

	// > 100 µs
	_delay_us(100.0f);


	// set interface width to 8bits - (3)
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00110000);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00110000);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00110000);
	twi_mcp23017_fw_setPortA_DirOut(false);

	// set interface width to 8bits - (4)
	twi_mcp23017_av1624_fw_waitUntilReady();
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00110000);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00110000);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00110000);
	twi_mcp23017_fw_setPortA_DirOut(false);

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

	// display Entry Mode Set
	twi_mcp23017_av1624_fw_waitUntilReady();
	twi_mcp23017_fw_setPortA_DirOut(true);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00000110);
	twi_mcp23017_fw_setPortBA(0b0001, 0b00000110);
	twi_mcp23017_fw_setPortBA(0b0000, 0b00000110);
	twi_mcp23017_fw_setPortA_DirOut(false);
}

void twi_mcp23017_av1624_fw_close()
{
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
		twi_mcp23017_fw_setPortB(0b0010);
		twi_mcp23017_fw_setPortB(0b0011);
		uint8_t status = twi_mcp23017_fw_readPortA(0b0011);
		twi_mcp23017_fw_setPortB(0b0010);

		if (!(status & 0x80)) {
			break;
		}

		_delay_us(10);
	}
}
