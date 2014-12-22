/*
 * df4iah_clkPullPwm.h
 *
 *  Created on: 24.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_CLKPULLPWM_H_
#define DF4IAH_FW_CLKPULLPWM_H_


enum ENUM_CLKOUT_t {
	ENUM_CLKOUT_OFF 										= 0,
	ENUM_CLKOUT_ON
};


void clkPullPwm_fw_init();
void clkPullPwm_fw_close();

void clkPullPwm_fw_setRatio(uint16_t ratio);

void clkPullPwm_fw_togglePin();
void clkPullPwm_fw_endlessTogglePin();

#endif /* DF4IAH_FW_CLKPULLPWM_H_ */
