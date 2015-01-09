/*
 * df4iah_fw_clkFastCtr.h
 *
 *  Created on: 26.12.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_CLKFASTCTR_H_
#define DF4IAH_FW_CLKFASTCTR_H_


/* this modules uses the T1 timer/counter/pwm-generator of the AVR controller as timer */

//#define EXPERIMENTAL										// ... or not

#define OCR1_TOP_VALUE 										19999
#define FAST_PWM_SUB_BITCNT									5													// highest X bits are relevant
#define FAST_PWM_SUB_INC_DEC								2													// minimal inc/dec step of sub PWM compare register
#define FAST_PWM_HIST_CNT									8
#define ADC_PWM_SAMPLING_CNT								16
#define ADC_PWM_CENTER_AREA									10
#define ADC_PWM_CENTER										(150 * ADC_PWM_SAMPLING_CNT)						// 0.65V (Center: 150)
#define ADC_PWM_LO											(ADC_PWM_CENTER - (120 * ADC_PWM_SAMPLING_CNT))		// 0.13V (Delta: -120)
#define ADC_PWM_HI											(ADC_PWM_CENTER + (120 * ADC_PWM_SAMPLING_CNT))		// 1.20V (Delta: +125)
#define ADC_PWM_SWITCH_SPEED								4
#define ADC_PWM_SWITCH_OUT									300
#define ADC_PWM_REDUCE_RANGE								75

#define DEBUG_UP_NR											4
#define DEBUG_UP_PIN										PINB
#define DEBUG_UP_DDR										DDRB
#define DEBUG_UP_PORT										PORTB
#define DEBUG_DN_NR											5
#define DEBUG_DN_PIN										PINB
#define DEBUG_DN_DDR										DDRB
#define DEBUG_DN_PORT										PORTB


void clkFastCtr_fw_init();
void clkFastCtr_fw_close();

/* the following functions are direct __vector_xx calls to reduce some clocks */
//void clkFastCtr_ISR_T1_CompA();

#endif /* DF4IAH_FW_CLKFASTCTR_H_ */
