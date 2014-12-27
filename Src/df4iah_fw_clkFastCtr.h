/*
 * df4iah_fw_clkFastCtr.h
 *
 *  Created on: 26.12.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_CLKFASTCTR_H_
#define DF4IAH_FW_CLKFASTCTR_H_


/* this modules uses the T2 timer/counter/pwm-generator of the AVR controller as timer */

#define DEFAULT_OCR2A_VALUE									199


void clkFastCtr_fw_init();
void clkFastCtr_fw_close();

/* the following functions are direct __vector_xx calls to reduce some clocks */
//void clkFastCtr_ISR_T2_CompA();

#endif /* DF4IAH_FW_CLKFASTCTR_H_ */
