/*
 * df4iah_fw_anlgComp.h
 *
 *  Created on: 26.12.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_ANLGCOMP_H_
#define DF4IAH_FW_ANLGCOMP_H_


/* this modules uses the Analog Comparator to generate interrupts based on GPS (10 kHz) */

#define AC_ADC_CH_COUNT										2


void anlgComp_fw_init();
void anlgComp_fw_close();

/* the following functions are direct __vector_xx calls to reduce some clocks */
//void clkFastCtr_ISR_T1_Capt();
//void clkFastCtr_ISR_T1_CompA();

#endif /* DF4IAH_FW_ANLGCOMP_H_ */