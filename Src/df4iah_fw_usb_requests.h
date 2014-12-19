/*
 * df4iah_usb.h
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_USB_REQUESTS_H_
#define DF4IAH_FW_USB_REQUESTS_H_


/* Echo the data from host to function back to the host */
#define USBCUSTOMRQ_ECHO	0

/* Send the textual data from the host to the function */
#define USBCUSTOMRQ_SEND	1

/* Request the textual data from the function to the host */
#define USBCUSTOMRQ_RECV	2


#endif /* DF4IAH_FW_USB_REQUESTS_H_ */
