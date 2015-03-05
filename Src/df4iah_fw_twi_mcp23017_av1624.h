/*
 * df4iah_fw_twi_mcp23017_av1624.h
 *
 *  Created on: 05.03.2015
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_TWI_MCP23017_AV1624_H_
#define DF4IAH_FW_TWI_MCP23017_AV1624_H_


#include "chipdef.h"


#define TWI_MCP23017_AV1624_XXX								0


void twi_mcp23017_av1624_fw_init();
void twi_mcp23017_av1624_fw_close();

void twi_mcp23017_av1624_fw_waitUntilReady();

#endif /* DF4IAH_FW_TWI_MCP23017_AV1624_H_ */
