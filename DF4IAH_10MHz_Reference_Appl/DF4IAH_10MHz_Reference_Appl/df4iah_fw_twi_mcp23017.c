/*
 * df4iah_fw_twi_mcp23017.c
 *
 *  Created on: 05.03.2015
 *      Author: DF4IAH, Ulrich Habel
 */
// tabsize: 4

#include <stdbool.h>

#include "df4iah_fw_main.h"
#include "df4iah_fw_twi.h"

#include "df4iah_fw_twi_mcp23017.h"


extern volatile main_bf_t main_bf;


void twi_mcp23017_fw_init(void)
{
	uint8_t clr_data = 0x00;  // clear data
	uint8_t set_data = 0xff;  // set data
	uint8_t conData = _BV(IOCON_SEQOP) | _BV(IOCON_ODR);	// BANK=0, no MIRROR, BYTE mode, do not overwrite INT bits (ODR),

	/* IOCON */
	(void) twi_fw_sendCmdSendData1(TWI_MCP23017_ADDR, TWI_MCP23017_REG_IOCON, conData);
	twi_fw_waitUntilDone(0);

	if (!main_bf.mainIsLcdAttached) {
		return;
	}

	/* GPIO */

	/* GPPU */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_GPPUA, set_data, set_data);  // pull up all GPIO bits - setting port-A and port-B

	/* IODIR */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_IODIRA, set_data, clr_data);  // 0=output / 1=input - setting port-A and port-B

	/* IPOL */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_IPOLA, clr_data, clr_data);  // no pin inversion - setting port-A and port-B

	/* GPIO */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_GPIOA, clr_data, clr_data);  // all data out cleared - setting port-A and port-B

	/* OLAT */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_OLATA, clr_data, clr_data);  // all data cleared - setting port-A and port-B


	/* INTERRUPTS */

	/* GPINTEN */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_GPINTENA, clr_data, clr_data);  // no INT used - setting port-A and port-B

	/* DEFVAL */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_DEFVALA, clr_data, clr_data);  // unused - setting port-A and port-B

	/* INTCON */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_INTCONA, set_data, set_data);  // unused - setting port-A and port-B
}

#if 0
void twi_mcp23017_fw_close(void)
{
	uint8_t clr_data = 0x00;  // clear data
	uint8_t set_data = 0xff;  // set data

	/* INTERRUPTS */

	/* GPINTEN */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_GPINTENA, clr_data, clr_data);  // no INT enable - setting port-A and port-B


	/* GPIO */

	/* GPPU */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_GPPUA, set_data, set_data);  // setting port-A and port-B

	/* IODIR */
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_IODIRA, set_data, set_data);  // setting port-A and port-B
}
#endif

void twi_mcp23017_fw_setPortA_DirOut(uint8_t isOut)
{
	uint8_t clr_data = 0x00;  // clear data
	uint8_t set_data = 0xff;  // set data

	if (isOut) {
		(void) twi_fw_sendCmdSendData1(TWI_MCP23017_ADDR, TWI_MCP23017_REG_IODIRA, clr_data);  // 0=output - setting port-A

	} else {
		(void) twi_fw_sendCmdSendData1(TWI_MCP23017_ADDR, TWI_MCP23017_REG_IODIRA, set_data);  // 1=input  - setting port-A
	}
}

void twi_mcp23017_fw_setPortB(uint8_t portB)
{
	(void) twi_fw_sendCmdSendData1(TWI_MCP23017_ADDR, TWI_MCP23017_REG_OLATB, portB);
}

void twi_mcp23017_fw_setPortBA(uint8_t portB, uint8_t portA)
{
	portB |= (portA & 0x01) << 7;							// XXX defective MCP23017 work-around
	(void) twi_fw_sendCmdSendData1SendData2(TWI_MCP23017_ADDR, TWI_MCP23017_REG_OLATA, portA, portB);
}

uint8_t twi_mcp23017_fw_readPortA(void)
{
	return twi_fw_sendCmdReadData1(TWI_MCP23017_ADDR, TWI_MCP23017_REG_GPIOA);
}
