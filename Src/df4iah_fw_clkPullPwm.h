/*
 * df4iah_fw_clkPullPwm.h
 *
 *  Created on: 24.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_CLKPULLPWM_H_
#define DF4IAH_FW_CLKPULLPWM_H_


/* this modules uses the T0 timer/counter/pwm-generator of the AVR controller as 8-bit PWM generator */


enum ENUM_CLKOUT_t {
	ENUM_CLKOUT_OFF 										= 0,
	ENUM_CLKOUT_ON
};


void clkPullPwm_fw_init();
void clkPullPwm_fw_close();

#if 0
void clkPullPwm_fw_setPin(uint8_t isSet);  // do not use when TWI is active
void clkPullPwm_fw_setPin_ID(uint8_t id);  // do not use when TWI is active
#endif

#endif /* DF4IAH_FW_CLKPULLPWM_H_ */
