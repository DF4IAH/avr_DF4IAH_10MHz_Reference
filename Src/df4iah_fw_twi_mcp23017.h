/*
 * df4iah_fw_twi_mcp23017.h
 *
 *  Created on: 05.03.2015
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_TWI_MCP23017_H_
#define DF4IAH_FW_TWI_MCP23017_H_


#include "chipdef.h"


// I2C Address
#define TWI_MCP23017_ADDR									0x40

// IOCON.BANK = 0
#define TWI_MCP23017_REG_IODIRA								0x00
#define IODIRA_IO0											0
#define IODIRA_IO1											1
#define IODIRA_IO2											2
#define IODIRA_IO3											3
#define IODIRA_IO4											4
#define IODIRA_IO5											5
#define IODIRA_IO6											6
#define IODIRA_IO7											7

#define TWI_MCP23017_REG_IODIRB								0x01
#define IODIRB_IO0											0
#define IODIRB_IO1											1
#define IODIRB_IO2											2
#define IODIRB_IO3											3
#define IODIRB_IO4											4
#define IODIRB_IO5											5
#define IODIRB_IO6											6
#define IODIRB_IO7											7

#define TWI_MCP23017_REG_IPOLA								0x02
#define IPOLA_IP0											0
#define IPOLA_IP1											1
#define IPOLA_IP2											2
#define IPOLA_IP3											3
#define IPOLA_IP4											4
#define IPOLA_IP5											5
#define IPOLA_IP6											6
#define IPOLA_IP7											7

#define TWI_MCP23017_REG_IPOLB								0x03
#define IPOLB_IP0											0
#define IPOLB_IP1											1
#define IPOLB_IP2											2
#define IPOLB_IP3											3
#define IPOLB_IP4											4
#define IPOLB_IP5											5
#define IPOLB_IP6											6
#define IPOLB_IP7											7

#define TWI_MCP23017_REG_GPINTENA							0x04
#define GPINTENA_GPINT0										0
#define GPINTENA_GPINT1										1
#define GPINTENA_GPINT2										2
#define GPINTENA_GPINT3										3
#define GPINTENA_GPINT4										4
#define GPINTENA_GPINT5										5
#define GPINTENA_GPINT6										6
#define GPINTENA_GPINT7										7

#define TWI_MCP23017_REG_GPINTENB							0x05
#define GPINTENB_GPINT0										0
#define GPINTENB_GPINT1										1
#define GPINTENB_GPINT2										2
#define GPINTENB_GPINT3										3
#define GPINTENB_GPINT4										4
#define GPINTENB_GPINT5										5
#define GPINTENB_GPINT6										6
#define GPINTENB_GPINT7										7

#define TWI_MCP23017_REG_DEFVALA							0x06
#define DEFVALA_DEF0										0
#define DEFVALA_DEF1										1
#define DEFVALA_DEF2										2
#define DEFVALA_DEF3										3
#define DEFVALA_DEF4										4
#define DEFVALA_DEF5										5
#define DEFVALA_DEF6										6
#define DEFVALA_DEF7										7

#define TWI_MCP23017_REG_DEFVALB							0x07
#define DEFVALB_DEF0										0
#define DEFVALB_DEF1										1
#define DEFVALB_DEF2										2
#define DEFVALB_DEF3										3
#define DEFVALB_DEF4										4
#define DEFVALB_DEF5										5
#define DEFVALB_DEF6										6
#define DEFVALB_DEF7										7

#define TWI_MCP23017_REG_INTCONA							0x08
#define INTCONA_IOC0										0
#define INTCONA_IOC1										1
#define INTCONA_IOC2										2
#define INTCONA_IOC3										3
#define INTCONA_IOC4										4
#define INTCONA_IOC5										5
#define INTCONA_IOC6										6
#define INTCONA_IOC7										7


#define TWI_MCP23017_REG_INTCONB							0x09
#define INTCONB_IOC0										0
#define INTCONB_IOC1										1
#define INTCONB_IOC2										2
#define INTCONB_IOC3										3
#define INTCONB_IOC4										4
#define INTCONB_IOC5										5
#define INTCONB_IOC6										6
#define INTCONB_IOC7										7

/* bit assignments of TWI_MCP23017_REG_IOCON */
#define TWI_MCP23017_REG_IOCON								0x0A
#define TWI_MCP23017_REG_IOCON_2							0x0B
#define IOCON_INTPOL										1
#define IOCON_ODR											2
#define IOCON_HAEN											3
#define IOCON_DISSLW										4
#define IOCON_SEQOP											5
#define IOCON_MIRROR										6
#define IOCON_BANK											7

#define TWI_MCP23017_REG_GPPUA								0x0C
#define GPPUA_PU0											0
#define GPPUA_PU1											1
#define GPPUA_PU2											2
#define GPPUA_PU3											3
#define GPPUA_PU4											4
#define GPPUA_PU5											5
#define GPPUA_PU6											6
#define GPPUA_PU7											7

#define TWI_MCP23017_REG_GPPUB								0x0D
#define GPPUB_PU0											0
#define GPPUB_PU1											1
#define GPPUB_PU2											2
#define GPPUB_PU3											3
#define GPPUB_PU4											4
#define GPPUB_PU5											5
#define GPPUB_PU6											6
#define GPPUB_PU7											7

#define TWI_MCP23017_REG_INTFA								0x0E
#define INTFA_INT0											0
#define INTFA_INT1											1
#define INTFA_INT2											2
#define INTFA_INT3											3
#define INTFA_INT4											4
#define INTFA_INT5											5
#define INTFA_INT6											6
#define INTFA_INT7											7

#define TWI_MCP23017_REG_INTFB								0x0F
#define INTFB_INT0											0
#define INTFB_INT1											1
#define INTFB_INT2											2
#define INTFB_INT3											3
#define INTFB_INT4											4
#define INTFB_INT5											5
#define INTFB_INT6											6
#define INTFB_INT7											7

#define TWI_MCP23017_REG_INTCAPA							0x10
#define INTCAPA_ICP0										0
#define INTCAPA_ICP1										1
#define INTCAPA_ICP2										2
#define INTCAPA_ICP3										3
#define INTCAPA_ICP4										4
#define INTCAPA_ICP5										5
#define INTCAPA_ICP6										6
#define INTCAPA_ICP7										7

#define TWI_MCP23017_REG_INTCAPB							0x11
#define INTCAPB_ICP0										0
#define INTCAPB_ICP1										1
#define INTCAPB_ICP2										2
#define INTCAPB_ICP3										3
#define INTCAPB_ICP4										4
#define INTCAPB_ICP5										5
#define INTCAPB_ICP6										6
#define INTCAPB_ICP7										7

#define TWI_MCP23017_REG_GPIOA								0x12
#define GPIOA_GP0											0
#define GPIOA_GP1											1
#define GPIOA_GP2											2
#define GPIOA_GP3											3
#define GPIOA_GP4											4
#define GPIOA_GP5											5
#define GPIOA_GP6											6
#define GPIOA_GP7											7

#define TWI_MCP23017_REG_GPIOB								0x13
#define GPIOB_GP0											0
#define GPIOB_GP1											1
#define GPIOB_GP2											2
#define GPIOB_GP3											3
#define GPIOB_GP4											4
#define GPIOB_GP5											5
#define GPIOB_GP6											6
#define GPIOB_GP7											7

#define TWI_MCP23017_REG_OLATA								0x14
#define OLATA_OL0											0
#define OLATA_OL1											1
#define OLATA_OL2											2
#define OLATA_OL3											3
#define OLATA_OL4											4
#define OLATA_OL5											5
#define OLATA_OL6											6
#define OLATA_OL7											7

#define TWI_MCP23017_REG_OLATB								0x15
#define OLATB_OL0											0
#define OLATB_OL1											1
#define OLATB_OL2											2
#define OLATB_OL3											3
#define OLATB_OL4											4
#define OLATB_OL5											5
#define OLATB_OL6											6
#define OLATB_OL7											7


void twi_mcp23017_fw_init();
void twi_mcp23017_fw_close();

void twi_mcp23017_fw_setPortA_DirOut(uint8_t isOut);
void twi_mcp23017_fw_setPortB(uint8_t portB);
void twi_mcp23017_fw_setPortBA(uint8_t portB, uint8_t portA);
uint8_t twi_mcp23017_fw_readPortA();

#endif /* DF4IAH_FW_TWI_MCP23017_H_ */
