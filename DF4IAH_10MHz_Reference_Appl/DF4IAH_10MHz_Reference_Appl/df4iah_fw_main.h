/*
 * df4iah_fw_main.h
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_MAIN_H_
#define DF4IAH_FW_MAIN_H_


#include "usbdrv_fw/usbdrv.h"

/* VERSION: YYM, MDD */
#define VERSION_HIGH										170
#define VERSION_LOW											408


/* GPS NMEA */
#define UART_PORT			PORTD						// port D register
#define UART_DDR			DDRD						// DDR for port D
#define UART_TX_PNUM		PIN1						// PIN number for TX pin
#define UART_RX_PNUM		PIN0						// PIN number for RX pin

/* Timer-n compare output */
#define DDR_OC0B_REG		DDRD
#define DDR_OC0B			PD5

/* TWI ports (I2C compatible) */
#define TWI_PORT			PORTC
#define TWI_DDR				DDRC
#define TWI_SDA_PNUM		PC4
#define TWI_SCL_PNUM		PC5


/*
 * Pin "STARTPIN" on port "STARTPORT" in this port has to grounded
 * (active low) to start the bootloader
 */
#define PROBE_DDR			DDRD
#define PROBE_PORT			PORTD
#define PROBE_PIN			PIND
#define PROBE_PNUM			PIN3						// JP3 BootLoader

/* BOOT token and place of BOOT token as offset before RAMEND */
# define BOOT_TOKEN			0xb00f
# define BOOT_TOKEN_EE_ADR	0x3fa						// @see df4iah_fw_memory.h


/* MCU frequency */
#ifndef F_CPU
// #define F_CPU 7372800
#define F_CPU (7372800 / 2)
#endif

/*
 * Define if Watchdog-Timer should be disable at startup
 */
#define DISABLE_WDT_AT_STARTUP

/*
 * Watchdog-reset is issued at exit
 * define the timeout-value here (see avr-libc manual)
 */
//#define EXIT_WDT_TIME										WDTO_250MS

#define DEBUG_DELAY_CNT										1

#define MAIN_PREPARE_BUFFER_SIZE							128
#define MAIN_FORMAT_BUFFER_SIZE								128

#define MAIN_STACK_CHECK_SIZE								0x0220

// PHASE-ADC: 1.00V --> ADC-Value = 229 /1024 (Full-Scale = 4.47V)
#define ADC_PHASE_LO_LOCKING  								 20		// @<0.15V
#define ADC_PHASE_LO_INSYNC   								 80		// @ 0.35V
#define ADC_PHASE_CENTER									137		// @ 0.60V
#define ADC_PHASE_HI_INSYNC   								195		// @ 0.85V
#define ADC_PHASE_HI_LOCKING  								241		// @ 1.05V

#define MEAN_QRG_CLOCK_STAGES_F								  5.0f;
#define MEAN_PHASE_CLOCK_STAGES_F							  3.0f;
#define MEAN_PHASE_PPM_STAGES_F								  7.0f;
#define PWM_COR_STEPS_COARSE_DIV_F							  1.1f;
#define PWM_COR_STEPS_FINE_DIV_F							 10.0f;
#define CLOCK_DIFF_OUT										100l
#define CLOCK_DIFF_COARSE_FINE								 20l
#define CLOCK_DIFF_FAST_FRAME								  3


typedef struct main_bf_struct
{
     uint8_t  mainIsAFC										: 1;
     uint8_t  mainIsAPC										: 1;
     uint8_t  mainIsTimerTest								: 1;
     uint8_t  mainIsSerComm									: 1;
     uint8_t  mainIsUsbCommTest 							: 1;
     uint8_t  mainStopAvr		 							: 1;
     uint8_t  mainStackCheck								: 1;
     uint8_t  mainIsLcdAttached								: 1;
     uint8_t  mainIsSmartAttached							: 1;
//     uint8_t  mainReserved01								: 0; // fill to 8 bits

     uint8_t  mainHelpConcatNr								: 4;
     uint8_t  mainLcdLedMode								: 3;
     uint8_t  mainReserved11								: 1; // fill to 8 bits
} main_bf_t;

enum REFCLK_STATE_t {
	REFCLK_STATE_NOSYNC										= 0b0000,
	REFCLK_STATE_SEARCH_QRG									= 0b0001,
	REFCLK_STATE_SEARCH_PHASE								= 0b0010,
	REFCLK_STATE_SEARCH_PHASE_CNTR_STABLIZED				= 0b0011,
	REFCLK_STATE_LOCKING_PHASE								= 0b0111,
	REFCLK_STATE_IN_SYNC									= 0b1111
};

enum ENTER_MODE_t {
	ENTER_MODE_SLEEP 										= 0,
	ENTER_MODE_BL,
	ENTER_MODE_FW
};

enum LCD_LED_MODE_t {
	LCD_LED_MODE_OFF 										= 0,
	LCD_LED_MODE_ON,
	LCD_LED_MODE_A1,
	LCD_LED_MODE_A2,
	LCD_LED_MODE_A3,
	LCD_LED_MODE_A4
};


typedef struct twiStatus_struct
{
     uint8_t  state											: 8;

     uint8_t  doStart										: 1;
     uint8_t  isProcessing									: 1;
     uint8_t  isRepeatedStart								: 1;
     uint8_t  reserved01									: 5; // fill to 8 bits
} twiStatus_t;


#define TWI_DATA_BUFFER_SIZE								16


float main_fw_calcTimerToFloat(uint8_t intVal, uint8_t intSubVal);
float main_fw_calcTimerAdj(float pwmAdjust, uint8_t* intVal, uint8_t* intSubVal);
int   main_fw_strncmp(const unsigned char* msg, const unsigned char* cmpProg, size_t size);
int   main_fw_memcmp(const unsigned char* msg, const unsigned char* cmpProg, size_t size);
void  main_fw_nmeaUtcPlusOneSec(void);
void  main_fw_parseNmeaLineData(void);
void  twi_mcp23017_av1624_fw_showStatus(void);
void  twi_smart_lcd_fw_showStatus(void);
void  main_fw_sendInitialHelp(void);
void  main_fw_giveAway(void);
int   main(void);


#endif /* DF4IAH_FW_MAIN_H_ */
