/*
 * df4iah_usb.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_USB_H_
#define DF4IAH_FW_USB_H_

#include "chipdef.h"
#include "usbdrv_fw/usbdrv.h"


#ifndef true
# define true 1
#endif
#ifndef false
# define false 0
#endif


#define USBISRCTXT_BUFFER_SIZE								250
#define USBSETUPCTXT_BUFFER_SIZE							8
//#define HIDSERIAL_FEATUREBUFFER_SIZE 						32


void usb_fw_replyContent(uchar replyBuffer[], uchar data[]);
void usb_fw_sendInInterrupt();

void usb_fw_init();
void usb_fw_close();

#endif /* DF4IAH_FW_USB_H_ */
