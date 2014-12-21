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

#include "firmware/df4iah_fw_usb_requests.h"				/* custom request numbers */
#include "firmware/usbconfig.h"								/* USB_CFG_INTR_POLL_INTERVAL */

#include "terminal.h"


#define min(a,b) ((a) < (b) ?  (a) : (b))


#ifdef DEBUG
uchar usbRingBufferHook[RINGBUFFER_HOOK_SIZE] = { 0 };
uint8_t usbRingBufferSendSemaphore = 0;
uint8_t usbRingBufferRcvSemaphore = 0;
uint8_t usbRingBufferHookLen = 0;
uint8_t usbRingBufferHookIsSend = 0;
#endif


// #define TEST_DATATRANSFER_PANEL	1
// #define TEST_DATATRANSFER_USB	1
// #define TEST_DATATRANSFER_SLOW	1



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


static char usbRingBufferSend[RINGBUFFER_SEND_SIZE];
static char usbRingBufferRcv[RINGBUFFER_RCV_SIZE];
static char usbMsg[MSGBUFFER_SIZE];
static int usbRingBufferRcvPushIdx = 0;
static int usbRingBufferRcvPullIdx = 0;
static int usbRingBufferSendPushIdx = 0;
static int usbRingBufferSendPullIdx = 0;

#ifdef TEST_DATATRANSFER_USB
static int errLine = 0;
#endif

/* -- 8< --  RINGBUFFERS */

// vv  DEBUGGING SECTION FOR THE FIRMWARE
#ifdef DEBUG
inline uint8_t fw_getSemaphore(uint8_t isSend)
{
	uint8_t isLocked;
	uint8_t* semPtr = (isSend ?  &usbRingBufferSendSemaphore : &usbRingBufferRcvSemaphore);
	uint8_t sreg;

#if 0
	asm volatile
	(
		"ldi r19, 0x01\n\t"
		"in	 %1, 0x3f\n\t"
		"cli \n\t"
		"ld	%0, Z\n\t"
		"st	Z, r19\n\t"
		"out 0x3f, %1\n\t"
		: "=r" (isLocked),
		  "=r" (sreg)
		: "p" (semPtr)
		: "r19"
	);
#else
	isLocked = *semPtr;
	*semPtr = true;
#endif

	return !isLocked;
}

inline void fw_freeSemaphore(uint8_t isSend)
{
	/* check if the hook has a job attached to it */
	if (usbRingBufferHookLen) {
		(void) fw_ringBufferPush(usbRingBufferHookIsSend, usbRingBufferHook, usbRingBufferHookLen);
		usbRingBufferHookLen = 0;
	}

	/* free semaphore */
	{
		uint8_t* semPtr = (isSend ?  &usbRingBufferSendSemaphore : &usbRingBufferRcvSemaphore);
#if 0
		uint8_t sreg = SREG;
		cli();
		*semPtr = false;
		SREG = sreg;
#else
		*semPtr = false;
#endif
	}
}

uint8_t fw_ringBufferPush(uint8_t isSend, const uchar inData[], uint8_t len)
{
	uint8_t retLen = 0;
	uint8_t bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if (!(((pushIdx + 1) == pullIdx) || (((pushIdx + 1) == bufferSize) && !pullIdx))) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		uint8_t lenTop = min((pullIdx > pushIdx ?  (pullIdx - pushIdx - 1) : bufferSize - pushIdx - (!pullIdx ?  1 : 0)), len);
		uint8_t lenBot = min((pullIdx > pushIdx ?  0 : pullIdx - 1), len - lenTop);

		if (lenTop) {
			memcpy(&(ringBuffer[pushIdx]), inData, lenTop);
			retLen += lenTop;
		}

		if ((lenBot > 0) && (lenBot < 254)) {
			memcpy(&(ringBuffer[0]), &(inData[lenTop]), lenBot);
			retLen += lenBot;
		}

		printf("Push1: before usbRingBufferSendPushIdx=%d\tusbRingBufferRcvPushIdx=%d\tlenTop=%d\tlenBot=%d\n", usbRingBufferSendPushIdx, usbRingBufferRcvPushIdx, lenTop, lenBot);	// XXX REMOVE ME!
		// advance the index
		if (isSend) {
			usbRingBufferSendPushIdx += retLen;
			usbRingBufferSendPushIdx %= bufferSize;
		} else {
			usbRingBufferRcvPushIdx += retLen;
			usbRingBufferRcvPushIdx %= bufferSize;
		}
		printf("Push2: after  usbRingBufferSendPushIdx=%d\tusbRingBufferRcvPushIdx=%d\n\n", usbRingBufferSendPushIdx, usbRingBufferRcvPushIdx);										// XXX REMOVE ME!
	}
	return retLen;
}

void fw_ringBufferPushAddHook(uint8_t isSend, const uchar inData[], uint8_t len)
{
	/* copy data for the hooked job - hook needs to be unassigned before */
	if (!usbRingBufferHookLen) {
		usbRingBufferHookIsSend = isSend;
		memcpy(usbRingBufferHook, inData, len);
		usbRingBufferHookLen = len;							// this assignment last - since now ready to process
	}														// else: dismiss data - should not be the case anyway
}

uint8_t fw_ringBufferPull(uint8_t isSend, uchar outData[], uint8_t size)
{
	uint8_t len = 0;
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if ((pushIdx != pullIdx) && (size > 1)) {
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		uint8_t bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
		uint8_t lenTop = min((pushIdx > pullIdx ?  (pushIdx - pullIdx) : bufferSize - pullIdx), size - 1);
		uint8_t lenBot = min((pushIdx > pullIdx ?  0 : pushIdx), size - 1 - lenTop);

		if (lenTop) {
			memcpy(outData, &(ringBuffer[pullIdx]), lenTop);
			len += lenTop;
		}

		if ((lenBot > 0) && (lenBot < 254)) {
			memcpy(&(outData[lenTop]), &(ringBuffer[0]), lenBot);
			len += lenBot;
		}

		outData[len] = 0;									// due to the security ending 0, we have to reduce the usable len by one

		printf("Pull1: before usbRingBufferSendPullIdx=%d\tusbRingBufferRcvPullIdx=%d\tlenTop=%d\tlenBot=%d\n", usbRingBufferSendPullIdx, usbRingBufferRcvPullIdx, lenTop, lenBot);	// XXX REMOVE ME!
		// advance the index
		if (isSend) {
			usbRingBufferSendPullIdx += len;
			usbRingBufferSendPullIdx %= bufferSize;
		} else {
			usbRingBufferRcvPullIdx += len;
			usbRingBufferRcvPullIdx %= bufferSize;
		}
		printf("Pull2: after  usbRingBufferSendPullIdx=%d\tusbRingBufferRcvPullIdx=%d\t\t\t\t\tstring=%s\n\n", usbRingBufferSendPullIdx, usbRingBufferRcvPullIdx, outData);			// XXX REMOVE ME!
	} else if (!size) {
		outData[0] = 0;
	}
	return len;
}

enum RINGBUFFER_MSG_STATUS_t fw_getStatusNextMsg(uint8_t isSend)
{
	enum RINGBUFFER_MSG_STATUS_t status = 0;
	uint8_t pushIdx = (isSend ?  usbRingBufferSendPushIdx : usbRingBufferRcvPushIdx);
	uint8_t pullIdx = (isSend ?  usbRingBufferSendPullIdx : usbRingBufferRcvPullIdx);

	if (pullIdx != pushIdx) {
		status |= RINGBUFFER_MSG_STATUS_AVAIL;

		/* test for NMEA message */
		uchar* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		if (*(ringBuffer + pullIdx) == MSG_PATTERN_NMEA) {	// first character identifies message type
			status |= RINGBUFFER_MSG_STATUS_IS_NMEA;
		}
	}
	return status;
}
#endif
// ^^  DEBUGGING SECTION FOR THE FIRMWARE

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

		if (lenTop) {
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

	if ((pushIdx != pullIdx) && (size > 1)) {
		char* ringBuffer = (isSend ?  usbRingBufferSend : usbRingBufferRcv);
		int bufferSize = (isSend ?  RINGBUFFER_SEND_SIZE : RINGBUFFER_RCV_SIZE);
		int lenTop = min((pushIdx > pullIdx ?  (pushIdx - pullIdx) : bufferSize - pullIdx), size - 1);
		int lenBot = min((pushIdx > pullIdx ?  0 : pushIdx), size - 1 - lenTop);

		if (lenTop) {
			memcpy(outData, &(ringBuffer[pullIdx]), lenTop);
			len += lenTop;
		}

		if (lenBot > 0) {
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

static void usb_controlOut(char inLine[], int len)
{
	ringBufferPush(true, inLine, len);
}

static int usb_controlIn(char outLine[], int size)
{
	return ringBufferPull(false, outLine, size);
}

void usb_yield()
{
	/* USB OUT */
	if (usbRingBufferSendPushIdx != usbRingBufferSendPullIdx) {
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
		usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, USBCUSTOMRQ_SEND, 0, 0, usbMsg, lenTx, USB_CFG_INTR_POLL_INTERVAL - 5);
#endif
	}

	/* USB IN */
	if (!(((usbRingBufferRcvPushIdx + 1) == usbRingBufferRcvPullIdx) || (((usbRingBufferRcvPushIdx + 1) == RINGBUFFER_RCV_SIZE) && !usbRingBufferRcvPullIdx))) {
        int usbRetLen = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USBCUSTOMRQ_RECV, 0, 0, usbMsg, sizeof(usbMsg), USB_CFG_INTR_POLL_INTERVAL - 5);
#ifdef TEST_DATATRANSFER_USB
        mvhline(LINES - 7 + (errLine % 7), 20, ' ', 60);
#endif
        if (usbRetLen > 0) {
			for (int idx = usbRetLen - 1; idx; --idx) {
				if ((usbMsg[idx] == '\n') || (usbMsg[idx] == '\r')) {
					usbRetLen = idx;
				}
			}
			usbMsg[usbRetLen] = 0;
#ifdef TEST_DATATRANSFER_USB
			mvprintw(LINES - 7 + (errLine++ % 7), 20, "IN  Data: usbRetLen=%d msg=%s.   ", usbRetLen, usbMsg);
#endif
        	ringBufferPush(false, usbMsg, usbRetLen);
		} else if (usbRetLen < 0) {
#ifdef TEST_DATATRANSFER_USB
        	mvprintw(LINES - 7 + (errLine++ % 7), 20, "USB error -  IN: %s\n", usb_strerror());
#endif
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
		init_pair(E_COLOR_PAIR_TITLE,			COLOR_WHITE, COLOR_RED);
		init_pair(E_COLOR_PAIR_SEND_MAIN,		COLOR_YELLOW, COLOR_RED);
		init_pair(E_COLOR_PAIR_SEND_GPS,		COLOR_WHITE, COLOR_RED);
		init_pair(E_COLOR_PAIR_RCV_MAIN,		COLOR_BLUE, 252);	// 15, 252
		init_pair(E_COLOR_PAIR_RCV_GPS,			COLOR_BLUE, COLOR_WHITE);
		init_pair(E_COLOR_PAIR_DEBUGGING_IN,	COLOR_YELLOW, COLOR_BLACK);
		init_pair(E_COLOR_PAIR_DEBUGGING_OUT,	COLOR_WHITE, COLOR_BLACK);
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
	mvwhline(*win_tx, 1, 2, ' ', COLS - 4);
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
	char inLine[MSGBUFFER_SIZE] = { 0 };
	char outLine[MSGBUFFER_SIZE] = { 0 };
	int inLineCnt = 0;
	int outLineCnt = 0;
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
		inLineCnt = usb_controlIn(inLine, sizeof(inLine));
# ifdef TEST_DATATRANSFER_USB_TEST2
			char debugBuffer[MSGBUFFER_SIZE] = { 0 };
			sprintf(debugBuffer, " usb_controlIn:   inLineCnt=%03d ", inLineCnt);
			ncurses_rx_print(&win_rx, debugBuffer, E_COLOR_PAIR_DEBUGGING_IN, 0);
# endif
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

		case KEY_F(2):
			sleep(30);
			loop = 0;
			break;

		case ERR:											// no key hit
		case 0x0d:
			break;

		case 0x0a:
			if (outLineCnt > 0) {
				if (!strcmp("EXIT", outLine) || !strcmp("QUIT", outLine)) {
				   loop = 0;
				   continue;
				}

# ifdef TEST_DATATRANSFER_USB_TEST2
			char debugBuffer[MSGBUFFER_SIZE] = { 0 };
			sprintf(debugBuffer, " usb_controlOut: outLineCnt=%03d ", outLineCnt);
			ncurses_rx_print(&win_rx, debugBuffer, E_COLOR_PAIR_DEBUGGING_OUT, 0);
# endif
				usb_controlOut(outLine, outLineCnt);

				{
					enum E_COLOR_PAIRS_t thisColor = E_COLOR_PAIR_SEND_MAIN;
					int thisAttribute = A_BOLD;

					if (outLine[0] == '$') {				// NMEA messages are o be marked special
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

		/* spare time for USB jobs to be done */
		usb_yield();

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

#ifdef TEST_DATATRANSFER_SLOW
		nextTime += 250000;
#else
		nextTime += (USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000;
#endif
		mvprintw(LINES - 1, COLS - 30, "INFO: deltaTime=%06d", deltaTime);
	} while (loop);

	ncurses_finish(&win_rxborder, &win_rx, &win_tx);
}
