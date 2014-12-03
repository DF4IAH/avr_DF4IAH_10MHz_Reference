/*
 * df4iah_probe.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_PROBE_H_
#define DF4IAH_PROBE_H_

#include "chipdef.h"


void init_probe();
void close_probe();
uint8_t check_jumper();

#endif /* DF4IAH_PROBE_H_ */
