/*
 * terminal.c
 *
 *  Created on: 15.12.2014
 *      Author: espero
 */


#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <usb.h>											/* this is libusb */

#include "firmware/df4iah_fw_usb_requests.h"				/* custom request numbers */
#include "firmware/usbconfig.h"								/* USB_CFG_INTR_POLL_INTERVAL */
#include "main.h"

#include "terminal.h"


#define min(a,b) ((a) < (b) ?  (a) : (b))


#ifdef DEBUG
uchar usbRingBufferHook[RINGBUFFER_HOOK_SIZE] = { 0 };
uint8_t usbRingBufferSendSemaphore = 0;
uint8_t usbRingBufferRcvSemaphore = 0;
uint8_t usbRingBufferHookLen = 0;
uint8_t usbRingBufferHookIsSend = 0;
#endif


/* the color pair index */
enum E_COLOR_PAIRS_t {
	E_COLOR_PAIR_TITLE = 1,
	E_COLOR_PAIR_SEND_MAIN,
	E_COLOR_PAIR_SEND_GPS,
	E_COLOR_PAIR_RCV_MAIN,
	E_COLOR_PAIR_RCV_GPS,
	E_COLOR_PAIR_DEBUGGING_IN,
	E_COLOR_PAIR_DEBUGGING_OUT
};


extern usb_dev_handle* handle;


static int usbRingBufferRcvPushIdx = 0;
static int usbRingBufferRcvPullIdx = 0;
static int usbRingBufferSendPushIdx = 0;
static int usbRingBufferSendPullIdx = 0;
static uchar usbRingBufferSend[RINGBUFFER_SEND_SIZE];
static uchar usbRingBufferRcv[RINGBUFFER_RCV_SIZE];
static uchar usbMsg[MSGBUFFER_SIZE];

#ifdef TEST_DATATRANSFER_USB
static int errLine = 0;
#endif

/* -- 8< --  RINGBUFFERS */

int ringBufferPush(char isSend, uchar inData[], int len)
{
	int retLen = 0;
	int bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
	int pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	int pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if (!(((pushIdx + 1) == pullIdx) || (((pushIdx + 1) == bufferSize) && !pullIdx))) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		int lenTop = min((pullIdx > pushIdx ?  (pullIdx - pushIdx - 1) : bufferSize - pushIdx - (!pullIdx ?  1 : 0)), len);
		int lenBot = min((((pullIdx > pushIdx) || !pullIdx) ?  0 : pullIdx - 1), len - lenTop);

		if (lenTop) {
			memcpy(&(ringBuffer[pushIdx]), inData, lenTop);
			retLen += lenTop;
		}

		if (lenBot) {
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

int ringBufferPull(char isSend, uchar outData[], int size)
{
	int len = 0;
	int pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	int pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if ((pushIdx != pullIdx) && (size > 1)) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		int bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
		int lenTop = min((pushIdx > pullIdx ?  (pushIdx - pullIdx) : bufferSize - pullIdx), size - 1);
		int lenBot = min((pushIdx > pullIdx ?  0 : pushIdx), size - 1 - lenTop);

		if (lenTop) {
			memcpy(outData, &(ringBuffer[pullIdx]), lenTop);
			len += lenTop;
		}

		if (lenBot) {
			memcpy(&(outData[lenTop]), &(ringBuffer[0]), lenBot);
			len += lenBot;
		}

		outData[len] = 0;

		// advance the index
		if (isSend) {
			usbRingBufferSendPullIdx += len;
			usbRingBufferSendPullIdx %= bufferSize;
		} else {
			usbRingBufferRcvPullIdx += len;
			usbRingBufferRcvPullIdx %= bufferSize;
		}
	} else if (!size) {
		outData[0] = 0;
	}
	return len;
}


/* -- 8< --  USB */

static void usb_buffer_controlOut(uchar inLine[], int len)
{
	ringBufferPush(true, inLine, len);
}

static int usb_buffer_controlIn(uchar outLine[], int size)
{
	return ringBufferPull(false, outLine, size);
}

void usb_do_transfers()
{
	/* USB OUT */
	if (handle && (usbRingBufferSendPushIdx != usbRingBufferSendPullIdx)) {
		int lenTx = ringBufferPull(true, usbMsg, sizeof(usbMsg));
#ifdef TEST_DATATRANSFER_USB
		int usbRetLen = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, USBCUSTOMRQ_SEND, 0, 0, usbMsg, lenTx, USB_CFG_INTR_POLL_INTERVAL - 5);
		mvhline(LINES - 7 + (errLine % 7), 20, ' ', 60);
		if (usbRetLen >= 0) {
			mvprintw(LINES - 7 + (errLine++ % 7), 20, "OUT Data: usbRetLen=%d of lenTx=%d .   ", usbRetLen, lenTx);
		} else if (usbRetLen < 0) {
        	mvprintw(LINES - 7 + (errLine++ % 7), 20, "USB error - OUT: %s\n", usb_strerror());
        }
#else
		int err = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, USBCUSTOMRQ_SEND, 0, 0, (char*) usbMsg, lenTx, USB_CFG_INTR_POLL_INTERVAL - 5);
		if (err < 0) {
			closeDevice();
		}
#endif
	}

	/* USB IN */
	if (handle && !(((usbRingBufferRcvPushIdx + 1) == usbRingBufferRcvPullIdx) || (((usbRingBufferRcvPushIdx + 1) == RINGBUFFER_RCV_SIZE) && !usbRingBufferRcvPullIdx))) {
        int usbRetLen = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USBCUSTOMRQ_RECV, 0, 0, (char*) usbMsg, sizeof(usbMsg), USB_CFG_INTR_POLL_INTERVAL - 5);
#ifdef TEST_DATATRANSFER_USB
        mvhline(LINES - 7 + (errLine % 7), 20, ' ', 60);
#endif
        if ((usbRetLen > 0) && (usbRetLen != sizeof(usbMsg))) {
        	ringBufferPush(false, usbMsg, usbRetLen);
#ifdef TEST_DATATRANSFER_USB
			mvprintw(LINES - 7 + (errLine++ % 7), 20, "IN  Data: usbRetLen=%d msg=%s.   ", usbRetLen, usbMsg);
#endif

        } else if (usbRetLen < 0) {
#ifdef TEST_DATATRANSFER_USB
        	mvprintw(LINES - 7 + (errLine++ % 7), 20, "USB error -  IN: %s\n", usb_strerror());
#endif
			closeDevice();
        }
	}
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
		init_pair(E_COLOR_PAIR_TITLE,			COLOR_WHITE,  COLOR_RED);
		init_pair(E_COLOR_PAIR_SEND_MAIN,		COLOR_BLUE,   511);
		init_pair(E_COLOR_PAIR_SEND_GPS,		COLOR_BLUE,   511);  // 15
		init_pair(E_COLOR_PAIR_RCV_MAIN,		COLOR_BLACK,  511);
		init_pair(E_COLOR_PAIR_RCV_GPS,			COLOR_BLACK,  511);  // 15
		init_pair(E_COLOR_PAIR_DEBUGGING_IN,	COLOR_YELLOW, COLOR_BLACK);
		init_pair(E_COLOR_PAIR_DEBUGGING_OUT,	COLOR_WHITE,  COLOR_BLACK);
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
	mvprintw(LINES - 2, COLS / 2 - 16, " Press F1 to exit, F2 to exit delayed ");
	attroff(A_BOLD | A_REVERSE);

	/* Cursor positioning */
	wmove(*win_tx, 1, 2);
	mvwhline(*win_tx, 1, 2, ' ', COLS - 4);

	/* Updating */
	refresh();
	wrefresh(*win_rxborder);
	wrefresh(*win_rx);
	wrefresh(*win_tx);
}

static void ncurses_finish(WINDOW* win_rxborder, WINDOW* win_rx, WINDOW* win_tx)
{
	wborder(win_rxborder, ' ', ' ', ' ',' ',' ',' ',' ',' ');
	wrefresh(win_rxborder);
	delwin(win_rx);
	delwin(win_rxborder);

	wborder(win_tx, ' ', ' ', ' ',' ',' ',' ',' ',' ');
	wrefresh(win_tx);
	delwin(win_tx);

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
	mvwprintw(*win_rx, LINES - 18, 1, "%s", string);
	if (colorPair) {
		wattroff(*win_rx, COLOR_PAIR(colorPair));
	}
	if (attributes) {
		wattroff(*win_rx, attributes);
	}
}

static void ncurses_tx_clear(WINDOW* win_tx)
{
	wmove(win_tx, 1, 2);
	mvwhline(win_tx, 1, 2, ' ', COLS - 4);
}

static void ncurses_tx_print(WINDOW* win_tx, const char string[])
{
	mvwprintw(win_tx, 1, 2, "%s", string);
}

static void ncurses_update(WINDOW* win_rxborder, WINDOW* win_rx, WINDOW* win_tx)
{
	refresh();
	wrefresh(win_rxborder);
	wrefresh(win_rx);
	wrefresh(win_tx);
}


/* -- 8< --  TERMINAL - LOOP */

void terminal()
{
	WINDOW* win_rxborder = NULL;
	WINDOW* win_rx = NULL;
	WINDOW* win_tx = NULL;
	struct timeval nowTime;
	uchar inLine[MSGBUFFER_SIZE] = { 0 };
	uchar outLine[MSGBUFFER_SIZE] = { 0 };
	int inLineCnt = 0;
	int outLineCnt = 0;
	uchar touched = false;
#ifdef TEST_DATATRANSFER_PANEL
	int idleCnt = 0;
#endif

	/* timing init */
	gettimeofday(&nowTime, NULL);
	long long nextTime = nowTime.tv_sec * 1000000 + nowTime.tv_usec + ((USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000);

	ncurses_init(&win_rxborder, &win_rx, &win_tx);

	char loop = 1;
	do {
		/* transfer field */
#ifdef TEST_DATATRANSFER_PANEL
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
		inLineCnt = usb_buffer_controlIn(inLine, sizeof(inLine));
# ifdef TEST_DATATRANSFER_USB_TEST2
		if (inLineCnt) {
			char debugBuffer[MSGBUFFER_SIZE] = { 0 };
			sprintf(debugBuffer, " usb_controlIn:   inLineCnt=%03d ", inLineCnt);
			ncurses_rx_print(&win_rx, debugBuffer, E_COLOR_PAIR_DEBUGGING_IN, 0);
		}
# endif
#endif
		if (inLineCnt) {
        	int oldIdx = 0;
        	for (int idx = 0; idx < inLineCnt; ++idx) {
        		if (inLine[idx] == '\r') {
        			inLine[idx] = '\n';

        		} else if ((inLine[idx] == '\n') || ((idx + 1) == inLineCnt)) {
					enum E_COLOR_PAIRS_t thisColor = E_COLOR_PAIR_RCV_MAIN;
					int thisAttribute = A_BOLD;

					if (inLine[idx] == '\n') {
						inLine[idx] = 0;
					} else {
						// do not eat last character if it ends without a limiting character
						inLine[idx + 1] = 0;
					}

					if (inLine[oldIdx] == '$') {			// NMEA messages are highlighted
						thisColor = E_COLOR_PAIR_RCV_GPS;
						thisAttribute = 0;
					}
					ncurses_rx_print(&win_rx, (char*) &(inLine[oldIdx]), thisColor, thisAttribute);
					touched = true;
					oldIdx = idx + 1;
				}
        	}
		}

		/* edit field */
		int c = getch();
		switch (c) {
		case KEY_F(1):
			loop = 0;
			break;

		case KEY_F(2):
			sleep(300);
			loop = 0;
			break;

		case ERR:											// no key hit
		case 0x0d:
			break;

		case 0x0a:
			if (outLineCnt > 0) {
				if (!strcmp("EXIT", (char*) outLine) || !strcmp("QUIT", (char*) outLine)) {
				   loop = 0;
				   continue;
				}

# ifdef TEST_DATATRANSFER_USB_TEST2
			char debugBuffer[MSGBUFFER_SIZE] = { 0 };
			sprintf(debugBuffer, " usb_controlOut: outLineCnt=%03d ", outLineCnt);
			ncurses_rx_print(&win_rx, debugBuffer, E_COLOR_PAIR_DEBUGGING_OUT, 0);
			touched = true;
# endif
				usb_buffer_controlOut(outLine, outLineCnt);

				{
					enum E_COLOR_PAIRS_t thisColor = E_COLOR_PAIR_SEND_MAIN;
					int thisAttribute = A_BOLD;

					if (outLine[0] == '$') {				// NMEA messages are o be marked special
						thisColor = E_COLOR_PAIR_SEND_GPS;
						thisAttribute = 0;
					}
					ncurses_rx_print(&win_rx, (char*) outLine, thisColor, thisAttribute);
				}
				outLineCnt = 0;
				outLine[outLineCnt] = 0;
				ncurses_tx_clear(win_tx);
				touched = true;
		    }
			break;

		case 0x08:
		case 0x7f:
		case KEY_DC:
			if (outLineCnt > 0) {
				outLine[--outLineCnt] = 0;
				ncurses_tx_clear(win_tx);
				ncurses_tx_print(win_tx, (char*) outLine);
				touched = true;
			}
			break;

		default:
			if ((outLineCnt + 1) < sizeof(outLine) && (c < 128)) {
				outLine[outLineCnt++] = (char) (c >= 'a' && c <= 'z' ?  (c & ~0x20) : c);
				outLine[outLineCnt] = 0;
				ncurses_tx_print(win_tx, (char*) outLine);
				touched = true;
			}
		}

		/* update terminal window */
		if (touched) {
			ncurses_update(win_rxborder, win_rx, win_tx);
			touched = false;
		}

		/* spare time for USB jobs to be done */
		usb_do_transfers();

		if (!handle) {
			sleep(1);
			openDevice(true);
		}

		/* timer */
		gettimeofday(&nowTime, NULL);
		time_t deltaTime = (time_t) (nextTime - nowTime.tv_sec * 1000000 - nowTime.tv_usec);
#ifdef __APPLE_CC__
		if (deltaTime > 0) {
			usleep(deltaTime);
		} else if (deltaTime < -1000) {
			/* adjust to now time */
			nextTime += -deltaTime;
		}
#else
		if (deltaTime > 0) {
		   unsigned int usSleepTime = deltaTime * (1000000 / CLOCKS_PER_SEC);
		   nanosleep(usSleepTime * 1000);
		}
#endif

#ifdef TEST_DATATRANSFER_SLOW
		nextTime += 250000;
#else
		nextTime += (USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000;
#endif
		mvprintw(LINES - 1, COLS - 30, "INFO: deltaTime=%09d", deltaTime);
		move(LINES - 8, outLineCnt + 2);
	} while (loop);

	ncurses_finish(win_rxborder, win_rx, win_tx);
	win_rxborder = win_rx = win_tx = NULL;
}
