/*
 * df4iah_fw_clkPullPwm.h
 *
 *  Created on: 24.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_CLKPULLPWM_H_
#define DF4IAH_FW_CLKPULLPWM_H_


/* this modules uses the T0 timer/counter/pwm-generator of the AVR controller as 8-bit PWM generator */


#ifndef F_CPU
# define F_CPU												20000000
#endif


void clkPullPwm_fw_init(void);
void clkPullPwm_fw_close(void);

#if 0
void clkPullPwm_fw_setPin(uint8_t isSet);  // do not use when TWI is active
void clkPullPwm_fw_setPin_ID(uint8_t id);  // do not use when TWI is active
#endif

#endif /* DF4IAH_FW_CLKPULLPWM_H_ */
