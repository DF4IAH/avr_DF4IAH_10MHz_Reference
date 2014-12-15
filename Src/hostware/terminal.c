/*
 * terminal.c
 *
 *  Created on: 15.12.2014
 *      Author: espero
 */


#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <ncurses.h>
#include <usb.h>											/* this is libusb */

//#include "firmware/df4iah_fw_usb_requests.h"				/* custom request numbers */
#include "firmware/usbconfig.h"								/* USB_CFG_INTR_POLL_INTERVAL */


static void sendInLine(char inLine[])
{

}

static void ncurses_init(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx)
{
	/* ncurses init */
	initscr();

	if (has_colors()) {
		start_color();
		use_default_colors();
		init_pair(1, COLOR_BLUE, 252);  // 15, 252
		init_pair(2, COLOR_RED, 252);
	}

	/* key input */
	cbreak();
	noecho();
	keypad(stdscr, TRUE);
	timeout(0);

	/* windows */
	*win_rxborder = newwin(LINES - 14, COLS - 1, 2, 0);
	box(*win_rxborder, 0 , 0);
	*win_rx = newwin(LINES - 16, COLS - 3, 3, 1);
	scrollok(*win_rx, true);

	*win_tx = newwin(3, COLS - 1, LINES - 9, 0);
	box(*win_tx, 0 , 0);
	scrollok(*win_tx, false);

	attron(A_BOLD);
//	attron(COLOR_PAIR(1));
	mvprintw(1, 1, " Transfer window ");
//	attroff(COLOR_PAIR(1));
	mvprintw(LINES - 10, 1, " Send edit field ");
	attron(A_REVERSE);
	mvprintw(LINES - 2, COLS / 2 - 6, " Press F1 to exit ");
	attroff(A_BOLD | A_REVERSE);

	wmove(*win_tx, 1, 2);
	wclrtoeol(*win_tx);

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

void terminal()
{
	WINDOW* win_rxborder = NULL;
	WINDOW* win_rx = NULL;
	WINDOW* win_tx = NULL;
	char inLine[1024] = { 0 };
	int inLineCnt = 0;
	struct timeval nowTime;
	gettimeofday(&nowTime, NULL);
	long long nextTime = nowTime.tv_sec * 1000000 + nowTime.tv_usec + ((USB_CFG_INTR_POLL_INTERVAL * CLOCKS_PER_SEC) / 1000);
	int idleCnt = 0;

	ncurses_init(&win_rxborder, &win_rx, &win_tx);

	char loop = 1;
	do {
		/* output field */
		if (++idleCnt > 5) {
			idleCnt = 0;
			ncurses_rx_print(&win_rx, "test", 1, A_BOLD);
		}

		/* update terminal window */
		ncurses_update(&win_rxborder, &win_rx, &win_tx);

		/* input field */
		int c = getch();
		switch (c) {
		case KEY_F(1):
			loop = 0;
			break;

		case 0x0d:
		case ERR:
			break;

		   case 0x0a:
			   if (inLineCnt > 0) {
				   if (!strcmp("exit", inLine) || !strcmp("quit", inLine)) {
					   loop = 0;
					   continue;
				   }

//			   	   sendInLine(inLine);
				   ncurses_rx_print(&win_rx, inLine, 2, A_REVERSE | A_BOLD);
				   inLineCnt = 0;
				   inLine[inLineCnt] = 0;
				   ncurses_tx_clear(&win_tx);
			   }
			   break;

		   case 0x08:
		   case 0x7f:
		   case KEY_DC:
			   if (inLineCnt > 0) {
				   inLine[--inLineCnt] = 0;
				   ncurses_tx_clear(&win_tx);
				   ncurses_tx_print(&win_tx, inLine);
			   }
			   break;

		   default:
			   if ((inLineCnt + 1) < sizeof(inLine) && (c <= 255)) {
				   inLine[inLineCnt++] = (char) c;
				   inLine[inLineCnt] = 0;
				   ncurses_tx_print(&win_tx, inLine);
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
