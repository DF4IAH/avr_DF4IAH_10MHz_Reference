/*
 * df4iah_usb.c
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#include <stdint.h>
#include <string.h>
#include <util/delay.h>
#include <avr/pgmspace.h>									// required by usbdrv.h
#include <avr/wdt.h>
#include <avr/boot.h>

#include "df4iah_fw_memory.h"
#include "df4iah_fw_usb_requests.h"
#include "df4iah_fw_ringbuffer.h"

#include "df4iah_fw_usb.h"


#define min(a,b) ((a) < (b) ?  (a) : (b))


//extern uint16_t usbSetupCntr;
extern uint16_t cntRcv;
extern uint16_t cntSend;
extern uint8_t usbIsrCtxtBufferIdx;
extern uint8_t isUsbCommTest;

extern uchar usbIsrCtxtBuffer[USBISRCTXT_BUFFER_SIZE];
extern uchar usbCtxtSetupReplyBuffer[USBSETUPCTXT_BUFFER_SIZE];

static uint16_t doTestCntr 									= 0;


#if USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH
# if 1
PROGMEM const char usbDescriptorHidReport[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
	/* USB report descriptor */
	0x06, 0x00, 0xff,										// USAGE_PAGE (Generic Desktop)					- Data 2 Bytes, Global, Function: Usage Page, Size 2 Bytes: 0xff00 (vendor-defined)
	0x09, 0x01,												// USAGE (Vendor 0xff00 - Usage 1)				- Data 1 Byte, Local, Function: Usage, Size 1 Byte: Usage 1 of vendor-defined page 0xff00
	0xa1, 0x01,												// COLLECTION (Application)						- Data 1 Byte, Main, Function: Collection --> Application
	0x15, 0x00,												//   LOGICAL_MINIMUM (0)						- Data 1 Byte, Global, Function: Logical Minimum, Size 1 Byte: 0
	0x26, 0xff, 0x00,										//   LOGICAL_MAXIMUM (255)						- Data 2 Bytes, Global, Function: Logical Maximum, Size 2 Bytes: 255
	0x75, 0x08,												//   REPORT_SIZE (8)							- Data 1 Byte, Global, Function: Report Size, Size 1 Byte: uint value 8
	0x95, HIDSERIAL_INBUFFER_SIZE,							//   REPORT_COUNT (8)							- Data 1 Byte, Global, Function: Report Count, Size 1 Byte: uint value 8
	0x09, 0x01,												//   USAGE (vendor 1)							- Data 1 Byte, Local, Function: Usage, Size 1 Byte: Usage 1 of vendor-defined page 0xff00
	0x82, 0x32, 0x01,										//   INPUT (Data,Var,Abs,Buf)					- Data 2 Bytes, Main, Function: Input --> data, Variable, absolute, no wrap, Non-linear, Non-preferred, no null position, non volatile, (reserved), Buffered Bytes
	0x95, HIDSERIAL_OUTBUFFER_SIZE,							//   REPORT_COUNT (8)							- Data 1 Byte, Global, Function: Report Count, Size 1 Byte: uint value 8
	0x09, 0x02,												//   USAGE (vendor 2)							- Data 1 Byte, Local, Function: Usage, Size 1 Byte: Usage 2 of vendor-defined page 0xff00
	0x92, 0x32, 0x01,										//   OUTPUT (Data,Var,Abs,Buf)					- Data 2 Bytes, Main, Function: Input --> data, Variable, absolute, no wrap, Non-linear, Non-preferred, no null position, non volatile, (reserved), Buffered Bytes
	0x95, HIDSERIAL_FEATUREBUFFER_SIZE,						//   REPORT_COUNT (32)							- Data 1 Byte, Global, Function: Report Count, Size 1 Byte: uint value 32
	0x09, 0x03,												//   USAGE (vendor 3)							- Data 1 Byte, Local, Function: Usage, Size 1 Byte: Usage 3 of vendor-defined page 0xff00
	0xb2, 0x32, 0x01,										//   FEATURE (Data,Var,Abs,Buf)					- Data 2 Byte, Main, Function: Feature --> data, Variable, absolute, no wrap, Non-linear, Non-preferred, no null position, non volatile, (reserved), Buffered Bytes
	0xc0													// END_COLLECTION								- Data 0 Bytes, Main, Function: End Collection
};

# else

PROGMEM const char usbDescriptorHidReport[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
	/* OLD but BAD */
	/* USB report descriptor */
    0x06, 0x00, 0xff,										// USAGE_PAGE (Generic Desktop)
    0x09, 0x01,												// USAGE (Vendor Usage 1)
    0xa1, 0x01,												// COLLECTION (Application)
    0x15, 0x00,												//   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,										//   LOGICAL_MAXIMUM (255)
    0x75, 0x08,												//   REPORT_SIZE (8)
    0x95, 0x08,												//   REPORT_COUNT (8)
    0x09, 0x00,												//   USAGE (Undefined)
    0x82, 0x02, 0x01,										//   INPUT (Data,Var,Abs,Buf)
    0x95, HIDSERIAL_INBUFFER_SIZE,							//   REPORT_COUNT (32)
    0x09, 0x00,												//   USAGE (Undefined)
    0xb2, 0x02, 0x01,										//   FEATURE (Data,Var,Abs,Buf)
    0xc0													// END_COLLECTION
};
# endif
#endif


#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
void usb_fw_replyContent(uchar replyBuffer[], uchar data[])
{
	replyBuffer[0] = data[2];
	replyBuffer[1] = data[3];
	replyBuffer[2] = data[4];
	// replyBuffer[3] = 0;
}

/* Send IN-Interrupts to the host regular in case
 * any data is to be sent.
 */
# ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
# endif
void usb_fw_sendInInterrupt()
{
	static uchar bufferInt[5] = "<INT>";

	if (usbInterruptIsReady()) {
		/* send next packet if a new time-slot is ready to send */
		usbSetInterrupt(bufferInt, sizeof(bufferInt));
	}
}

/*  -- 8< -- */

#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
void usb_fw_init()
{
	usbInit();
	USB_INTR_ENABLE &= ~(_BV(USB_INTR_ENABLE_BIT));
	usbDeviceDisconnect();									// enforce re-enumeration, do this while interrupts are disabled!

    uint8_t i = 250;
    while (--i) {											// fake USB disconnect for > 250 ms
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

/*  -- 8< -- */

/* usbFunctionSetup() is called when the host does a setup of the USB function. For more
 * information see the documentation in usbdrv/usbdrv.h.
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
USB_PUBLIC usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	const usbRequest_t* rq = (usbRequest_t*) data;
	usbMsgLen_t len = 0;

    if (((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR) &&
    	((rq->bmRequestType & USBRQ_RCPT_MASK) == USBRQ_RCPT_DEVICE)) {
    	//usbSetupCntr++;

    	if (rq->bRequest == USBCUSTOMRQ_ECHO) {				// echo -- used for reliability tests
    		usbCtxtSetupReplyBuffer[0] = rq->wValue.bytes[0];
    		usbCtxtSetupReplyBuffer[1] = rq->wValue.bytes[1];
    		usbCtxtSetupReplyBuffer[2] = rq->wIndex.bytes[0];
    		usbCtxtSetupReplyBuffer[3] = rq->wIndex.bytes[1];
    		len = 4;

    	} else if (rq->bRequest == USBCUSTOMRQ_RECV) {		// receive data from this USB function
    		cntRcv = rq->wLength.word;
        	return USB_NO_MSG;								// use usbFunctionRead() to obtain the data

    	} else if (rq->bRequest == USBCUSTOMRQ_SEND) {		// send data from host to this USB function
    		cntSend = rq->wLength.word;
            return USB_NO_MSG;								// use usbFunctionWrite() to receive data from host
    	}
    }

	usbMsgPtr = (usbMsgPtr_t) usbCtxtSetupReplyBuffer;
    return len;
}

/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
USB_PUBLIC uchar usbFunctionRead(uchar *data, uchar len)
{
	const uint8_t isSend = false;
	uint8_t retLen = 0;

	if (isUsbCommTest) {
		/* special communication TEST */
		if (cntRcv) {
#if 1
			data[retLen++] = '0' + ((doTestCntr / 1000) % 10);
			data[retLen++] = '0' + ((doTestCntr /  100) % 10);
			data[retLen++] = '0' + ((doTestCntr /   10) % 10);
			data[retLen++] = '0' + ( doTestCntr++       % 10);
#elif 0
			data[retLen++] = '0' +  (       len /  100)      ;
			data[retLen++] = '0' + ((       len /   10) % 10);
			data[retLen++] = '0' + (        len         % 10);
#else
			data[retLen++] = '0' +  (    cntRcv /  100)      ;
			data[retLen++] = '0' + ((    cntRcv /   10) % 10);
			data[retLen++] = '0' + (     cntRcv         % 10);
#endif
			data[retLen++] = ':';
		}
	}

	signed int readCnt = min(cntRcv, len) + 1 - retLen;
	if (readCnt > 0) {
		if (getSemaphore(isSend)) {
			/* pull next message from the ring buffer and send it to the host IN */
			uint8_t pullLen = ringBufferPull(isSend, data + retLen, readCnt);	// we accept that the final null byte is written behind the buffer
			freeSemaphore(isSend);
			cntRcv -= retLen + pullLen;
			return retLen + pullLen;

		} else {
			strcpy((char*) data, "SemInUse");
			return min(len, 8);
		}

	} else {
		return retLen;
	}
}

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
USB_PUBLIC uchar usbFunctionWrite(uchar *data, uchar len)
{
	const uint8_t isSend = true;

	if (cntSend > len) {
		/* append first or any substring to the inBuffer */
		cntSend -= len;
		memcpy(&(usbIsrCtxtBuffer[usbIsrCtxtBufferIdx]), data, len);
		usbIsrCtxtBufferIdx += len;
		return 0;											// go ahead with more transfer requests

	} else {
		/* append last substring to the inBuffer and push it to the OUT ring buffer (host --> USB function) */
		if (cntSend > 0) {
			memcpy(&(usbIsrCtxtBuffer[usbIsrCtxtBufferIdx]), data, cntSend);
			usbIsrCtxtBufferIdx += cntSend;
		}

		/* push OUT string (send) from host to the USB function's ring buffer */
		ringBufferAppend(isSend, false, usbIsrCtxtBuffer, usbIsrCtxtBufferIdx);

		usbIsrCtxtBufferIdx = cntSend = 0;
		return 1;											// no more data transfers accepted
	}
}

#if USB_CFG_IMPLEMENT_FN_WRITEOUT
# ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
# endif
USB_PUBLIC void usbFunctionWriteOut(uchar *data, uchar len)
{
	// interrupt(/bulk) data comes in here

}
#endif
