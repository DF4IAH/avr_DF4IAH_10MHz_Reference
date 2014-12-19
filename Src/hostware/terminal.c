/*
 * terminal.c
 *
 *  Created on: 15.12.2014
 *      Author: espero
 */


#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>
#include <string.h>
#include <usb.h>											/* this is libusb */

//#include "firmware/df4iah_fw_usb_requests.h"				/* custom request numbers */
#include "firmware/usbconfig.h"								/* USB_CFG_INTR_POLL_INTERVAL */

#include "terminal.h"


#define min(a,b) ((a) < (b) ?  (a) : (b))


#define RINGBUFFER_RCV_SIZE		5
#define RINGBUFFER_SEND_SIZE	5

//#define TEST_DATARANSFER 1


/* the color pair index */
enum E_COLOR_PAIRS_t {
	E_COLOR_PAIR_TITLE = 1,
	E_COLOR_PAIR_SEND_MAIN,
	E_COLOR_PAIR_SEND_GPS,
	E_COLOR_PAIR_RCV_MAIN,
	E_COLOR_PAIR_RCV_GPS,
};


static char usbRingBufferRcv[RINGBUFFER_RCV_SIZE];
static char usbRingBufferSend[RINGBUFFER_SEND_SIZE];
static int usbRingBufferRcvPushIdx = 0;
static int usbRingBufferRcvPullIdx = 0;
static int usbRingBufferSendPushIdx = 0;
static int usbRingBufferSendPullIdx = 0;


/* -- 8< --  RINGBUFFERS */

int ringBufferPush(char isSend, char inData[], int len)
{
	int retLen = 0;
	int bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
	int pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	int pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if (!(((pushIdx + 1) == pullIdx) || (((pushIdx + 1) == bufferSize) && !pullIdx))) {
		char* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		int lenTop = min((pullIdx > pushIdx ?  (pullIdx - pushIdx - 1) : bufferSize - pushIdx - (!pullIdx ?  1 : 0)), len);
		int lenBot = min((pullIdx > pushIdx ?  0 : pullIdx - 1), len - lenTop);

		if (lenTop > 0) {
			memcpy(&(ringBuffer[pushIdx]), inData, lenTop);
			retLen += lenTop;
		}

		if (lenBot > 0) {
			memcpy(&(ringBuffer[0]), &(inData[lenTop]), lenBot);
			retLen += lenBot;
		}

		// advance the index
		if (isSend) {
			usbRingBufferSendPushIdx += retLen;
			usbRingBufferSendPushIdx %= bufferSize;
		} else {
			usbRingBufferRcvPushIdx += retLen;
			usbRingBufferRcvPushIdx %= bufferSize;
		}
	}
	return retLen;
}

int ringBufferPull(char isSend, char outData[], int size)
{
	int len = 0;
	int pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	int pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if (pushIdx != pullIdx) {
		char* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		int bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
		int lenTop = min((pushIdx > pullIdx ?  (pushIdx - pullIdx) : bufferSize - pullIdx), size);
		int lenBot = min((pushIdx > pullIdx ?  0 : pushIdx), size - lenTop);

		if (lenTop > 0) {
			memcpy(outData, &(ringBuffer[pullIdx]), lenTop);
			len += lenTop;
		}

		if (lenBot > 0) {
			memcpy(&(outData[lenTop]), &(ringBuffer[0]), lenBot);
			len += lenBot;
		}

		// advance the index
		if (isSend) {
			usbRingBufferSendPullIdx += len;
			usbRingBufferSendPullIdx %= bufferSize;
		} else {
			usbRingBufferRcvPullIdx += len;
			usbRingBufferRcvPullIdx %= bufferSize;
		}
	}
	return len;
}


/* -- 8< --  USB */

static void usb_controlOut(char inLine[], int len)
{
	ringBufferPush(true, inLine, len);
}

static int usb_controlIn(char outLine[], int size)
{
	return ringBufferPull(false, outLine, size);
}


/* -- 8< --  NCURSES */

static void ncurses_init(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx)
{
	/* ncurses init */
	initscr();

	/* Color handling */
	if (has_colors()) {
		start_color();
		use_default_colors();
		init_pair(E_COLOR_PAIR_TITLE, COLOR_WHITE, COLOR_RED);
		init_pair(E_COLOR_PAIR_SEND_MAIN, COLOR_YELLOW, COLOR_RED);
		init_pair(E_COLOR_PAIR_SEND_GPS, COLOR_WHITE, COLOR_RED);
		init_pair(E_COLOR_PAIR_RCV_MAIN, COLOR_BLUE, 252);	// 15, 252
		init_pair(E_COLOR_PAIR_RCV_GPS, COLOR_BLUE, COLOR_WHITE);
	}

	/* Key input */
	cbreak();
	noecho();
	keypad(stdscr, TRUE);
	timeout(0);

	/* Window creation */
	*win_rxborder = newwin(LINES - 14, COLS - 1, 2, 0);
	box(*win_rxborder, 0 , 0);
	*win_rx = newwin(LINES - 16, COLS - 3, 3, 1);
	scrollok(*win_rx, true);

	*win_tx = newwin(3, COLS - 1, LINES - 9, 0);
	box(*win_tx, 0 , 0);
	scrollok(*win_tx, false);

	/* Titles */
	attron(A_BOLD);
	attron(COLOR_PAIR(E_COLOR_PAIR_TITLE));
	mvprintw(1, (COLS >> 1 ) - 30, "       DF4IAH  10 MHz  Reference Oscillator  -  Terminal       ");
	attroff(COLOR_PAIR(E_COLOR_PAIR_TITLE));

	mvprintw(1, 1, " Transfer window ");
	mvprintw(LINES - 10, 1, " Send edit field ");
	attron(A_REVERSE);
	mvprintw(LINES - 2, COLS / 2 - 6, " Press F1 to exit ");
	attroff(A_BOLD | A_REVERSE);

	/* Cursor positioning */
	wmove(*win_tx, 1, 2);
	wclrtoeol(*win_tx);

	/* Updating */
	refresh();
	wrefresh(*win_rxborder);
	wrefresh(*win_rx);
	wrefresh(*win_tx);
}

static void ncurses_finish(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx)
{
	wborder(*win_rxborder, ' ', ' ', ' ',' ',' ',' ',' ',' ');
	wrefresh(*win_rxborder);
	delwin(*win_rx);
	delwin(*win_rxborder);

	wborder(*win_tx, ' ', ' ', ' ',' ',' ',' ',' ',' ');
	wrefresh(*win_tx);
	delwin(*win_tx);

	refresh();
	endwin();
}

static void ncurses_rx_print(WINDOW** win_rx, const char string[], int colorPair, attr_t attributes)
{
	scroll(*win_rx);
	if (attributes) {
		wattron(*win_rx, attributes);
	}
	if (colorPair) {
		wattron(*win_rx, COLOR_PAIR(colorPair));
	}
	mvwprintw(*win_rx, LINES - 18, 1, "%s\n", string);
	if (colorPair) {
		wattroff(*win_rx, COLOR_PAIR(colorPair));
	}
	if (attributes) {
		wattroff(*win_rx, attributes);
	}
}

static void ncurses_tx_clear(WINDOW** win_tx)
{
	wmove(*win_tx, 1, 2);
	wclrtoeol(*win_tx);
}

static void ncurses_tx_print(WINDOW** win_tx, const char string[])
{
	mvwprintw(*win_tx, 1, 2, "%s", string);
}

static void ncurses_update(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx)
{
	refresh();
	wrefresh(*win_rxborder);
	wrefresh(*win_rx);
	wrefresh(*win_tx);
}


/* -- 8< --  TERMINAL - LOOP */

void terminal()
{
	WINDOW* win_rxborder = NULL;
	WINDOW* win_rx = NULL;
	WINDOW* win_tx = NULL;
	struct timeval nowTime;
	char inLine[1024] = { 0 };
	char outLine[1024] = { 0 };
	int inLineCnt = 0;
	int outLineCnt = 0;
#if TEST_DATARANSFER
	int idleCnt = 0;
#endif

	/* timing init */
	gettimeofday(&nowTime, NULL);
	long long nextTime = nowTime.tv_sec * 1000000 + nowTime.tv_usec + ((USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000);

	ncurses_init(&win_rxborder, &win_rx, &win_tx);

	char loop = 1;
	do {
		/* transfer field */
#if TEST_DATARANSFER
		if (++idleCnt > 5) {
			static char toggle = 0;
			toggle = !toggle;
			strcpy(inLine, toggle ?  "test" : "$test");
			inLineCnt = strlen(inLine);
			idleCnt = 0;
		} else {
			inLineCnt = 0;
		}
#else
		inLineCnt = usb_controlIn(inLine, sizeof(inLine));
#endif
		if (inLineCnt) {
			enum E_COLOR_PAIRS_t thisColor = E_COLOR_PAIR_RCV_MAIN;
			int thisAttribute = A_BOLD;

			if (inLine[0] == '$') {							// NMEA messages are o be marked special
				thisColor = E_COLOR_PAIR_RCV_GPS;
				thisAttribute = 0;
			}
			ncurses_rx_print(&win_rx, inLine, thisColor, thisAttribute);
		}

		/* update terminal window */
		ncurses_update(&win_rxborder, &win_rx, &win_tx);

		/* edit field */
		int c = getch();
		switch (c) {
		case KEY_F(1):
			loop = 0;
			break;

		case 0x0d:
		case ERR:
			break;

		   case 0x0a:
			   if (outLineCnt > 0) {
				   if (!strcmp("exit", outLine) || !strcmp("quit", outLine)) {
					   loop = 0;
					   continue;
				   }

			   	   usb_controlOut(outLine, outLineCnt);

				   {
						enum E_COLOR_PAIRS_t thisColor = E_COLOR_PAIR_SEND_MAIN;
						int thisAttribute = A_BOLD;

						if (outLine[0] == '$') {							// NMEA messages are o be marked special
							thisColor = E_COLOR_PAIR_SEND_GPS;
							thisAttribute = 0;
						}
						ncurses_rx_print(&win_rx, outLine, thisColor, thisAttribute);
				   }
				   outLineCnt = 0;
				   outLine[outLineCnt] = 0;
				   ncurses_tx_clear(&win_tx);
			   }
			   break;

		   case 0x08:
		   case 0x7f:
		   case KEY_DC:
			   if (outLineCnt > 0) {
				   outLine[--outLineCnt] = 0;
				   ncurses_tx_clear(&win_tx);
				   ncurses_tx_print(&win_tx, outLine);
			   }
			   break;

		   default:
			   if ((outLineCnt + 1) < sizeof(outLine) && (c < 128)) {
				   outLine[outLineCnt++] = (char) (c >= 'a' && c <= 'z' ?  (c & ~0x20) : c);
				   outLine[outLineCnt] = 0;
				   ncurses_tx_print(&win_tx, outLine);
			   }
		   }

		   /* timer */
		   gettimeofday(&nowTime, NULL);
		   time_t deltaTime = (time_t) (nextTime - nowTime.tv_sec * 1000000 - nowTime.tv_usec);
#ifdef __APPLE_CC__
		   if (deltaTime > 0) {
			   usleep(deltaTime);
		   }
#else
		   if (deltaTime > 0) {
			   unsigned int usSleepTime = deltaTime * (1000000 / CLOCKS_PER_SEC);
			   nanosleep(usSleepTime * 1000);
		   }
#endif
		   nextTime += (USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000;
	   } while (loop);

	   ncurses_finish(&win_rxborder, &win_rx, &win_tx);
}
