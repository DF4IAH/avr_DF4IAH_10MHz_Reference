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

#include "df4iah_fw_usb.h"


// static uchar replyBuffer[8];

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};
/* Since we define only one feature report, we don't use report-IDs (which
 * would be the first byte of the report). The entire report consists of 128
 * opaque data bytes.
 */



#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
void usb_fw_init()
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
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
void usb_fw_close()
{
	USB_INTR_ENABLE &= ~(_BV(USB_INTR_ENABLE_BIT));
	usbDeviceDisconnect();
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
uint8_t usb_fw_recvchar(void)
{
	return 0;  // TODO Missing implementation
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
void usb_fw_sendchar(uint8_t data)
{
	// TODO Missing implementation
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
void usb_fw_recvBuffer(uint16_t addr, uint8_t* dptr, uint8_t len)
{
#if 0
	pagebuf_t cnt;
	uint8_t *tmp = gBuffer;

	for (cnt = 0; cnt < sizeof(gBuffer); cnt++) {
		*tmp++ = (cnt < size) ? recvchar_usb() : 0xFF;
	}
#else
	// TODO Missing implementation
#endif
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
void usb_fw_sendBuffer(uint16_t addr, const uint8_t* sptr, uint8_t len)
{
	// TODO Missing implementation
}

// -- 8< --


#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
USB_PUBLIC usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	const usbRequest_t *rq = (void*) data;

	if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {					// HID class request
        if (rq->bRequest == USBRQ_HID_GET_REPORT) {										// wValue: ReportType (highbyte), ReportID (lowbyte)
            /* since we have only one report type, we can ignore the report-ID */
            return USB_NO_MSG;  														// use usbFunctionRead() to obtain data

        } else if (rq->bRequest == USBRQ_HID_SET_REPORT) {
            /* since we have only one report type, we can ignore the report-ID */
            return USB_NO_MSG;  														// use usbFunctionWrite() to receive data from host
        }

	} else {
        /* ignore vendor type requests, we don't use any */
    }

	return 0;
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
USB_PUBLIC uchar usbFunctionRead(uchar *data, uchar len)
{
#if 0
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
#else
	data[0] = 0x12;
	data[1] = 0x34;
	len = 2;
#endif
	return len;
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
USB_PUBLIC uchar usbFunctionWrite(uchar *data, uchar len)
{
#if 0
	/* check if programmer is in correct write state */
	if ((prog_state != PROG_STATE_WRITEFLASH) &&
		(prog_state	!= PROG_STATE_WRITEEEPROM)) {
		return 0xff;
	}

	if (prog_state == PROG_STATE_WRITEFLASH) {
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
#else
	return 0;
#endif
}
