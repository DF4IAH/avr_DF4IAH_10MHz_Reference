/*
 * terminal.h
 *
 *  Created on: 15.12.2014
 *      Author: espero
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include <ncurses.h>


/* -- 8< --  RINGBUFFERS */
int ringBufferPush(char isSend, char inData[], int len);
int ringBufferPull(char isSend, char outData[], int size);

/* -- 8< --  USB */
static void usb_controlOut(char inLine[], int len);
static int usb_controlIn(char outLine[], int size);

/* -- 8< --  NCURSES */
static void ncurses_init(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx);
static void ncurses_finish(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx);
static void ncurses_rx_print(WINDOW** win_rx, const char string[], int colorPair, attr_t attributes);
static void ncurses_tx_clear(WINDOW** win_tx);
static void ncurses_tx_print(WINDOW** win_tx, const char string[]);
static void ncurses_update(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx);

/* -- 8< --  TERMINAL - LOOP */
void terminal();


#endif /* TERMINAL_H_ */
