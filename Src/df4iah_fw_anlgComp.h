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
enum ADC_CH_t {
	ADC_CH_PWMPULL											= 0,
	ADC_CH_PHASE,
	ADC_CH_TEMP
};


void anlgComp_fw_init();
void anlgComp_fw_close();

void anlgComp_fw_startAdcConvertion();


/* the following functions are direct __vector_xx calls to reduce some clocks */
//void clkFastCtr_ISR_T1_CompA();
//void anlgComp_fw_ISR_ADC();

#endif /* DF4IAH_FW_ANLGCOMP_H_ */
