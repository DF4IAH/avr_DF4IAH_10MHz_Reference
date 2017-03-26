/*
 * df4iah_fw_twi_mcp23017_av1624.h
 *
 *  Created on: 05.03.2015
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_TWI_MCP23017_AV1624_H_
#define DF4IAH_FW_TWI_MCP23017_AV1624_H_


#include "chipdef.h"


void twi_mcp23017_av1624_fw_init(void);
void twi_mcp23017_av1624_fw_close(void);

void twi_mcp23017_av1624_fw_waitUntilReady(void);
void twi_mcp23017_av1624_fw_gotoPosition(uint8_t line, uint8_t column);
void twi_mcp23017_av1624_fw_writeString(const uint8_t* buffer, uint8_t len);


#endif /* DF4IAH_FW_TWI_MCP23017_AV1624_H_ */
