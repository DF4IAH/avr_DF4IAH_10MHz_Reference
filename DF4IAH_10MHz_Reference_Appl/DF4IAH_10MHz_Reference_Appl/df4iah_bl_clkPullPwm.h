/*
 * df4iah_bl_clkPullPwm.h
 *
 *  Created on: 24.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_BL_CLKPULLPWM_H_
#define DF4IAH_BL_CLKPULLPWM_H_


#define DEFAULT_PWM_COUNT  									90


void clkPullPwm_bl_init(void);
void clkPullPwm_bl_close(void);

void clkPullPwm_bl_togglePin(void);
void clkPullPwm_bl_endlessTogglePin(void);

#endif /* DF4IAH_BL_CLKPULLPWM_H_ */
