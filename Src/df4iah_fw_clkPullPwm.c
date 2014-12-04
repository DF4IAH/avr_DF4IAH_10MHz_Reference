/*
 * df4iah_clkPullPwm.c
 *
 *  Created on: 24.11.2014
 *      Author: espero
 */


#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "chipdef.h"
#include "df4iah_bl_clkPullPwm.h"
#include "df4iah_fw_clkPullPwm.h"


#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void init_fw_clkPullPwm()
{
	init_bl_clkPullPwm();
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void close_fw_clkPullPwm()
{
	close_bl_clkPullPwm();
}
