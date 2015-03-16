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

// #define TEST_DATATRANSFER_SLOW							// activate on request
// #define TEST_DATATRANSFER_USB_TEST2						// activate on request
// #define TEST_DATATRANSFER_PANEL							// activate on request
// #define TEST_DATATRANSFER_USB							// activate on request


typedef unsigned char  uchar;


/* -- 8< --  RINGBUFFERS */
int ringbuffer_fw_ringBufferPush(char isSend, uchar inData[], int len);
int ringBufferPull(char isSend, uchar outData[], int size);


/* -- 8< --  USB */
//static void usb_controlOut(char inLine[], int len);
//static int usb_controlIn(char outLine[], int size);


/* -- 8< --  NCURSES */
//static void ncurses_init(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx);
//static void ncurses_finish(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx);
//static void ncurses_rx_print(WINDOW** win_rx, const char string[], int colorPair, attr_t attributes);
//static void ncurses_tx_clear(WINDOW** win_tx);
//static void ncurses_tx_print(WINDOW** win_tx, const char string[]);
//static void ncurses_update(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx);


/* -- 8< --  TERMINAL - LOOP */
void terminal();


#endif /* TERMINAL_H_ */
