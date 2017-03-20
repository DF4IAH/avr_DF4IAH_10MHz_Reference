/*
 * df4iah_bl_probe.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_BL_PROBE_H_
#define DF4IAH_BL_PROBE_H_

#include "chipdef.h"


void probe_bl_init();
void probe_bl_close();
uint8_t probe_bl_checkJumper();

#endif /* DF4IAH_BL_PROBE_H_ */
