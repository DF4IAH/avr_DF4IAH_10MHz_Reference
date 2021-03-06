/*
 * serout.h
 *
 *  Created on: 10.01.2016
 *      Author: espero
 */

#ifndef SEROUT_H_
#define SEROUT_H_


#define RINGBUFFER_SEND_SIZE	1024
#define RINGBUFFER_RCV_SIZE		1024
#define RINGBUFFER_HOOK_SIZE	1024
#define MSGBUFFER_SIZE			254

// #define TEST_DATATRANSFER_SLOW							// activate on request
// #define TEST_DATATRANSFER_USB_TEST2						// activate on request
// #define TEST_DATATRANSFER_PANEL							// activate on request
// #define TEST_DATATRANSFER_USB							// activate on request

typedef unsigned char  uchar;

int switchMode(int mode);


/* -- 8< --  SEROUT - LOOP */
void serout(int mode);


#endif /* SEROUT_H_ */
