/*
 * df4iah_probe.c
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#include <avr/wdt.h>
#include <util/delay.h>

#include "df4iah_probe.h"


#ifdef RELEASE
__attribute__((section(".df4iah_probe"), aligned(2)))
#endif
void init_probe()
{
	MCUCR &= ~(1<<PUD);								// PullUp Disable off

	PROBE_DDR  &= ~(_BV(PROBE_PNUM));				// set probe line as input
	PROBE_PORT |=  _BV(PROBE_PNUM);					// and enable the PullUp

	uint8_t dlyCnt = 10;
	while (--dlyCnt) {								// delay for a reliable detection
        wdt_reset();
        _delay_ms(1);
    }
}

#ifdef RELEASE
__attribute__((section(".df4iah_probe"), aligned(2)))
#endif
void close_probe()
{
	PROBE_PORT &= ~(_BV(PROBE_PNUM));				// clear PULLUP to default
}

#ifdef RELEASE
__attribute__((section(".df4iah_probe"), aligned(2)))
#endif
inline uint8_t check_jumper()
{
	if (PROBE_PIN & _BV(PROBE_PNUM)) {
		// pin is not grounded - JUMPER open
		return 0;
	}
	return 1;  // JUMPER closed
}
