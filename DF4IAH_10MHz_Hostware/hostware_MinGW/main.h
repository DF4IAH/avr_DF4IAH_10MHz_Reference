/*
 * main.h
 *
 *  Created on: 24.12.2014
 *      Author: espero
 */

#ifndef MAIN_H_
#define MAIN_H_


#include <curses.h>

#ifndef true
# define true 1
#endif
#ifndef false
# define false 0
#endif


void openDevice(bool isReopening);
void closeDevice();

#endif /* MAIN_H_ */
