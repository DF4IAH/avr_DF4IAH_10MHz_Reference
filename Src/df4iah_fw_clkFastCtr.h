/*
 * df4iah_fw_clkFastCtr.h
 *
 *  Created on: 26.12.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_CLKFASTCTR_H_
#define DF4IAH_FW_CLKFASTCTR_H_


/* this modules uses the T1 timer/counter/pwm-generator of the AVR controller as timer */


#define OCR1_TOP_VALUE 										19999
#define FAST_PWM_SUB_BITCNT									8


void clkFastCtr_fw_init();
void clkFastCtr_fw_close();

/* the following functions are direct __vector_xx calls to reduce some clocks */
//void clkFastCtr_ISR_T1_CompA();

#endif /* DF4IAH_FW_CLKFASTCTR_H_ */
