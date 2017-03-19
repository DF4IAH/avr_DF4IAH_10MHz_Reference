/*
 * df4iah_bl_probe.c
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */
// tabsize: 4

#include <avr/wdt.h>
#include <util/delay.h>

#include "df4iah_bl_main.h"
#include "df4iah_bl_probe.h"


extern uint8_t mainIsJumperBlSet;


#ifdef RELEASE
__attribute__((section(".df4iah_bl_probe"), aligned(2)))
#endif
void probe_bl_init()
{
	MCUCR &= ~(_BV(PUD));							// PullUp Disable off

	PROBE_DDR  &= ~(_BV(PROBE_PNUM));				// set probe line as input
	PROBE_PORT |=   _BV(PROBE_PNUM);				// and enable the PullUp

	uint8_t dlyCnt = 10;
	while (--dlyCnt) {								// delay for a reliable detection
        wdt_reset();
        _delay_ms(1);
    }
	mainIsJumperBlSet = probe_bl_checkJumper();
}

#ifdef RELEASE
__attribute__((section(".df4iah_bl_probe"), aligned(2)))
#endif
void probe_bl_close()
{
	PROBE_PORT &= ~(_BV(PROBE_PNUM));				// clear PULLUP to default
}

#ifdef RELEASE
__attribute__((section(".df4iah_bl_probe"), aligned(2)))
#endif
inline uint8_t probe_bl_checkJumper()
{
	if (PROBE_PIN & _BV(PROBE_PNUM)) {
		// pin is not grounded - JUMPER open
		return 0;
	}
	return 1;  // JUMPER closed
}
