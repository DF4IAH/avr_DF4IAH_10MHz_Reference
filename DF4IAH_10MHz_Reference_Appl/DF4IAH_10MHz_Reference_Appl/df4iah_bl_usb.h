/*
 * df4iah_bl_usb.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_BL_USB_H_
#define DF4IAH_BL_USB_H_


#include "chipdef.h"
#include "usbdrv_bl/usbdrv.h"


void usb_bl_replyContent(uchar replyBuffer[], uchar data[]);

void usb_bl_init();
void usb_bl_close();

#endif /* DF4IAH_BL_USB_H_ */
