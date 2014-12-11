/*
 * df4iah_usb.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_USB_REQUESTS_H_
#define DF4IAH_FW_USB_REQUESTS_H_


/* Echo the data from host to function back to the host */
#define USB_RQ_ECHO_DATA	0

/* Send the textual data from the host to the function */
#define USB_RQ_SEND_DATA	1

/* Request the textual data from the function to the host */
#define USB_RQ_RECV_DATA	2


#endif /* DF4IAH_FW_USB_REQUESTS_H_ */
