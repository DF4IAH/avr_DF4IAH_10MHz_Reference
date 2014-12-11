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

#include "df4iah_fw_memory.h"
#include "df4iah_fw_usb.h"


static uchar reportId = 0;
static uchar received = 0;
static uchar bytesRemaining = 0;
static uchar* pos = 0;
static uchar inBuffer[HIDSERIAL_INBUFFER_SIZE] = { 0 };
static uchar replyBuffer[8] = { 0 };


PROGMEM const char usbDescriptorHidReport[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
	/* USB report descriptor */
	0x06, 0x00, 0xff,										// USAGE_PAGE (Generic Desktop)					- Data 2 Bytes, Global, Function: Usage Page, Size 2 Bytes: 0xff00 (vendor-defined)
	0x09, 0x01,												// USAGE (Vendor 0xff00 - Usage 1)				- Data 1 Byte, Local, Function: Usage, Size 1 Byte: Usage 1 of vendor-defined page 0xff00
	0xa1, 0x02,												// COLLECTION (Application)						- Data 1 Byte, Main, Function: Collection --> Application
	0x15, 0x00,												//   LOGICAL_MINIMUM (0)						- Data 1 Byte, Global, Function: Logical Minimum, Size 1 Byte: 0
	0x26, 0xff, 0x00,										//   LOGICAL_MAXIMUM (255)						- Data 2 Bytes, Global, Function: Logical Maximum, Size 2 Bytes: 255
	0x75, 0x08,												//   REPORT_SIZE (8)							- Data 1 Byte, Global, Function: Report Size, Size 1 Byte: uint value 8
	0x95, 0x08,												//   REPORT_COUNT (8)							- Data 1 Byte, Global, Function: Report Count, Size 1 Byte: uint value 8
	0x09, 0x01,												//   USAGE (Counted Buffer)						- Data 1 Byte, Local, Function: Usage, Size 1 Byte:
	0x82, 0x32, 0x01,										//   INPUT (Data,Var,Abs,Buf)					- Data 2 Bytes, Main, Function: Input --> data, Variable, absolute, no wrap, Non-linear, Non-preferred, no null position, non volatile, (reserved), Buffered Bytes
	0x09, 0x02,												//   USAGE (Counted Buffer)						- Data 1 Byte, Local, Function: Usage, Size 1 Byte:
	0x92, 0x32, 0x01,										//   OUTPUT (Data,Var,Abs,Buf)					- Data 2 Bytes, Main, Function: Input --> data, Variable, absolute, no wrap, Non-linear, Non-preferred, no null position, non volatile, (reserved), Buffered Bytes
	0x95, HIDSERIAL_INBUFFER_SIZE,							//   REPORT_COUNT (32)							- Data 1 Byte, Global, Function: Report Count, Size 1 Byte: uint value 8
	0x09, 0x03,												//   USAGE (Counted Buffer)						- Data 1 Byte, Local, Function: Usage, Size 1 Byte:
	0xb2, 0x32, 0x01,										//   FEATURE (Data,Var,Abs,Buf)					- Data 2 Byte, Main, Function: Feature --> data, Variable, absolute, no wrap, Non-linear, Non-preferred, no null position, non volatile, (reserved), Buffered Bytes
	0xc0													// END_COLLECTION								- Data 0 Bytes, Main, Function: End Collection
};

#if 0
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
#endif

// if (usbInterruptIsReady())
// usbSetInterrupt(uchar *data, uchar len);


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
    // const int reportId = rq->wValue.bytes[0];
    uchar len = 0;

    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {	// HID class request
    	if (rq->bRequest == USBRQ_HID_GET_REPORT) {
        	/* wValue: ReportType (highbyte), ReportID (lowbyte) */
        	/* since we have only one report type, we can ignore
        	 *  the report-ID */
        	return USB_NO_MSG;											// use usbFunctionRead() to obtain the data

        } else if (rq->bRequest == USBRQ_HID_SET_REPORT) {
        	/* since we have only one report type, we can ignore
        	 * the report-ID */
        	pos = inBuffer;
            bytesRemaining = rq->wLength.word;
            if (bytesRemaining > sizeof(inBuffer)) {
            	bytesRemaining = sizeof(inBuffer);
            }
            return USB_NO_MSG;											// use usbFunctionWrite() to receive data from host
        }

    } else {
    	/* ignore vendor type requests, we don't use any */

    	usb_fw_replyContent(replyBuffer, data);
    }

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
	// at the moment there is test data to be sent
	len = 0;

	data[len++] = '#';
	data[len++] = '\r';
	data[len++] = '\n';

	return len;
}

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
USB_PUBLIC uchar usbFunctionWrite(uchar *data, uchar len)
{
    if (reportId == 0) {
        if (len > bytesRemaining) {
            len = bytesRemaining;
        }
        bytesRemaining -= len;

        // int start = (pos == inBuffer) ?  1 : 0;

        for (int i = 0; i < len; i++) {
            if (data[i] != 0) {
                *pos++ = data[i];
             }
        }

        if (!bytesRemaining) {
            received = 1;
            *pos++ = 0;
            return 1;	// no more space available

        } else {
            return 0;	// go ahead with more transfer requests
        }

    } else {
        return 1;	// stop data transfer due to unsupported Report-ID
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
