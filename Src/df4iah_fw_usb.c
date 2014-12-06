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


uchar replyBuffer[8];
uchar prog_connected = 0;
uchar prog_state = 0;
uchar prog_address_newmode = 0;
unsigned int prog_nbytes = 0;


#if 0
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
#endif

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


#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
USB_PUBLIC usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	uchar len = 0;
	len = 0;

	return len;
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
USB_PUBLIC uchar usbFunctionRead(uchar *data, uchar len)
{

	return len;
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
#endif
USB_PUBLIC uchar usbFunctionWrite(uchar *data, uchar len)
{

	return 1;
}

#if USB_CFG_IMPLEMENT_FN_WRITEOUT
# ifdef RELEASE
__attribute__((section(".df4iah_fw_usb"), aligned(2)))
# endif
USB_PUBLIC void usbFunctionWriteOut(uchar *data, uchar len)
{

}
#endif
