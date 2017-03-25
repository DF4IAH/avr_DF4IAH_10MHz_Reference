/*
 * df4iah_bl_probe.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_BL_PROBE_H_
#define DF4IAH_BL_PROBE_H_

#include "chipdef.h"


void probe_bl_init(void);
void probe_bl_close(void);
uint8_t probe_bl_checkJumper(void);

#endif /* DF4IAH_BL_PROBE_H_ */
