/*
 * df4iah_fw_clkSlowCtr.h
 *
 *  Created on: 27.12.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_CLKSLOWCTR_H_
#define DF4IAH_FW_CLKSLOWCTR_H_


/* this modules uses the T0 timer/counter/pwm-generator of the AVR controller as counter */

void clkSlowCtr_fw_init();
void clkSlowCtr_fw_close();

/* the following functions are direct __vector_xx calls to reduce some clocks */
//void clkSlowCtr_ISR_T0_OVL();
//void clkSlowCtr_ISR_PCI2();

#endif /* DF4IAH_FW_CLKSLOWCTR_H_ */
