/*
 * df4iah_usb.c
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#include <stdint.h>
#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include <avr/wdt.h>
#include <avr/boot.h>
#include <util/delay.h>

#include "df4iah_memory.h"
#include "df4iah_usbAsp.h"
#include "df4iah_usb.h"


static uchar replyBuffer[8];

static uchar prog_connected = PROG_UNCONNECTED;
static uchar prog_state = PROG_STATE_IDLE;

static uchar prog_address_newmode = 0;
static unsigned long prog_address;
static unsigned int prog_nbytes = 0;
static unsigned int prog_pagesize;
static uchar prog_blockflags;
static uchar prog_pagecounter;


#ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
#endif
void replyContent(uchar replyBuffer[], uchar data[])
{
	replyBuffer[0] = data[2];
	replyBuffer[1] = data[3];
	replyBuffer[2] = data[4];
	// replyBuffer[3] = 0;
}

#ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
#endif
void init_usb()
{
	usbInit();
	USB_INTR_ENABLE &= ~(_BV(USB_INTR_ENABLE_BIT));
	usbDeviceDisconnect();			/* enforce re-enumeration, do this while interrupts are disabled! */

    uint8_t i = 250;
    while (--i) {					/* fake USB disconnect for > 250 ms */
        _delay_ms(1);
        wdt_reset();
    }

    usbDeviceConnect();
	USB_INTR_ENABLE |= _BV(USB_INTR_ENABLE_BIT);
}

#ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
#endif
void close_usb()
{
	USB_INTR_ENABLE &= ~(_BV(USB_INTR_ENABLE_BIT));
	usbDeviceDisconnect();
}

#if 0
#ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
#endif
uint8_t recvchar_usb(void)
{
	return 0;  // XXX TODO
}

#ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
#endif
void sendchar_usb(uint8_t data)
{
	// XXX TODO
}

#ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
#endif
void recvBuffer_usb(uint16_t addr, uint8_t* dptr, uint8_t len)
{
#if 0
	pagebuf_t cnt;
	uint8_t *tmp = gBuffer;

	for (cnt = 0; cnt < sizeof(gBuffer); cnt++) {
		*tmp++ = (cnt < size) ? recvchar_usb() : 0xFF;
	}
#else
#endif
}

#ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
#endif
void sendBuffer_usb(uint16_t addr, const uint8_t* sptr, uint8_t len)
{
}
#endif

// -- 8< --


#ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
#endif
USB_PUBLIC usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	uchar len = 0;

	replyBuffer[3] = replyBuffer[2] = replyBuffer[1] = replyBuffer[0] = 0;

	if (data[1] == USBASP_FUNC_CONNECT) {
		prog_connected = PROG_CONNECTED;

		/* set compatibility mode of address delivering */
		prog_address_newmode = 0;

	} else if (data[1] == USBASP_FUNC_DISCONNECT) {
		prog_connected = PROG_UNCONNECTED;

	} else if (data[1] == USBASP_FUNC_TRANSMIT) {
		if ((data[2] == 0x30) && (data[3] == 0x00) && (data[4] < 3)) {
			// signature bytes
			replyContent(replyBuffer, data);
			replyBuffer[3] = boot_signature_byte_get(data[4] << 1);
			len = 4;

		} else if ((data[2] == 0x50) && (data[3] == 0x00)) {
			// lfuse bits - @see page 271f
			replyContent(replyBuffer, data);
			replyBuffer[3] = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
			len = 4;

		} else if ((data[2] == 0x58) && (data[3] == 0x08)) {
			// hfuse bits
			replyContent(replyBuffer, data);
			replyBuffer[3] = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
			len = 4;

		} else if ((data[2] == 0x50) && (data[3] == 0x08)) {
			// efuse bits
			replyContent(replyBuffer, data);
			replyBuffer[3] = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
			len = 4;

		} else if ((data[2] == 0x58) && (data[3] == 0x00)) {
			// lock bits
			replyContent(replyBuffer, data);
			replyBuffer[3] = boot_lock_fuse_bits_get(GET_LOCK_BITS);
			len = 4;
		}

	} else if ((data[1] == USBASP_FUNC_READFLASH) || (data[1] == USBASP_FUNC_READEEPROM)) {
		if (prog_connected > PROG_UNCONNECTED) {
			if (!prog_address_newmode) {
				prog_address = (data[3] << 8) | data[2];
			}

			prog_nbytes = (data[7] << 8) | data[6];
			prog_state = (data[1] == USBASP_FUNC_READFLASH) ?  PROG_STATE_READFLASH : PROG_STATE_READEEPROM;
			len = 0xff; /* multiple in */
		}

	} else if (data[1] == USBASP_FUNC_ENABLEPROG) {
		if (prog_connected == PROG_CONNECTED) {
			prog_connected = PROG_PROGENABLED;
			replyBuffer[0] = 0;
		} else {
			replyBuffer[0] = 1;
		}
		len = 1;

	} else if ((data[1] == USBASP_FUNC_WRITEFLASH) || (data[1] == USBASP_FUNC_WRITEEEPROM)) {
		if (prog_connected == PROG_PROGENABLED) {
			if (!prog_address_newmode) {
				prog_address = (data[3] << 8) | data[2];
			}

			if (data[1] == USBASP_FUNC_WRITEFLASH) {
				prog_blockflags = data[5] & 0x0F;
				prog_pagesize |= (((unsigned int) data[5] & 0xF0) << 4) | data[4];
				if (prog_blockflags & PROG_BLOCKFLAG_FIRST) {
					prog_pagecounter = prog_pagesize;
				}
				prog_state = PROG_STATE_WRITEFLASH;

			} else {  /* data[1] == USBASP_FUNC_WRITEEEPROM */
				prog_pagesize = 0;
				prog_blockflags = 0;
				prog_state = PROG_STATE_WRITEEEPROM;
			}

			prog_nbytes = (data[7] << 8) | data[6];
			len = 0xff; /* multiple out */
		}

	} else if (data[1] == USBASP_FUNC_SETLONGADDRESS) {
		if (prog_connected > PROG_UNCONNECTED) {
			/* set new mode of address delivering (ignore address delivered in commands) */
			prog_address_newmode = 1;
			/* set new address */
			prog_address = *((unsigned long*) &data[2]);
		}

	} else if (data[1] == USBASP_FUNC_SETISPSCK) {
		/* LOC does not implement that */
		replyBuffer[0] = 0;
		len = 1;

	} else if (data[1] == USBASP_FUNC_TPI_CONNECT) {
		/* Tiny Programming Interface is not supported */

	} else if (data[1] == USBASP_FUNC_TPI_DISCONNECT) {
		/* Tiny Programming Interface is not supported */

	} else if (data[1] == USBASP_FUNC_TPI_RAWREAD) {
		/* Tiny Programming Interface is not supported */
		replyBuffer[0] = 0;
		len = 1;

	} else if (data[1] == USBASP_FUNC_TPI_RAWWRITE) {
		/* Tiny Programming Interface is not supported */

	} else if (data[1] == USBASP_FUNC_TPI_READBLOCK) {
		/* Tiny Programming Interface is not supported */
		len = 0xff; /* multiple in */

	} else if (data[1] == USBASP_FUNC_TPI_WRITEBLOCK) {
		/* Tiny Programming Interface is not supported */
		len = 0xff; /* multiple out */

	} else if (data[1] == USBASP_FUNC_GETCAPABILITIES) {
		/* Tiny Programming Interface is not supported */
		replyBuffer[0] = 0;  // USBASP_CAP_0_TPI;
		replyBuffer[1] = 0;
		replyBuffer[2] = 0;
		replyBuffer[3] = 0;
		len = 4;
	}

	usbMsgPtr = (usbMsgPtr_t) replyBuffer;
	return len;
}

#ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
#endif
USB_PUBLIC uchar usbFunctionRead(uchar *data, uchar len)
{
	/* check if programmer is in correct read state */
	if ((prog_state != PROG_STATE_READFLASH) &&
		(prog_state	!= PROG_STATE_READEEPROM)) {
		return 0xff;
	}

	if (prog_state == PROG_STATE_READFLASH) {
		readFlashPage(data, len, prog_address);
	} else {
		readEEpromPage(data, len, prog_address);
	}
	prog_address += len;

	/* last packet? */
	if (len < 8) {
		prog_state = PROG_STATE_IDLE;
	}

	return len;
}

#ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
#endif
USB_PUBLIC uchar usbFunctionWrite(uchar *data, uchar len)
{
	/* check if programmer is in correct write state */
	if ((prog_state != PROG_STATE_WRITEFLASH) &&
		(prog_state	!= PROG_STATE_WRITEEEPROM)) {
		return 0xff;
	}

	if (prog_state == PROG_STATE_WRITEFLASH) {	// TODO create a job list and copy the data and return here fast
		writeFlashPage(data, len, prog_address);
	} else {
		writeEEpromPage(data, len, prog_address);
	}
	prog_address += len;

	prog_nbytes -= len;
	if (prog_nbytes <= 0) {
		prog_state = PROG_STATE_IDLE;
		return 1;
	}

	return 0;
}

#if USB_CFG_IMPLEMENT_FN_WRITEOUT
# ifdef RELEASE
__attribute__((section(".df4iah_usb"), aligned(2)))
# endif
USB_PUBLIC void usbFunctionWriteOut(uchar *data, uchar len)
{

}
#endif
