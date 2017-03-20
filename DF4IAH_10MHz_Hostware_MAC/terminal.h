/*
 * terminal.h
 *
 *  Created on: 15.12.2014
 *      Author: espero
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include <ncurses.h>


#define RINGBUFFER_SEND_SIZE	1024
#define RINGBUFFER_RCV_SIZE		1024
#define RINGBUFFER_HOOK_SIZE	1024
#define MSGBUFFER_SIZE			254


typedef unsigned char  uchar;


/* -- 8< --  RINGBUFFERS */
int ringbuffer_fw_ringBufferPush(char isSend, uchar inData[], int len);
int ringBufferPull(char isSend, uchar outData[], int size);

void usb_do_transfers();

void terminal();


#endif /* TERMINAL_H_ */
