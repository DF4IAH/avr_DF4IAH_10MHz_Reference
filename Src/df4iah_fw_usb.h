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


void usb_fw_replyContent(uchar replyBuffer[], uchar data[]);

void usb_fw_init();
void usb_fw_close();

#endif /* DF4IAH_FW_USB_H_ */
