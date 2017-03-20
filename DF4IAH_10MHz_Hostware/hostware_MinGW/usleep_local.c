/*
 * usleep.c
 *
 *  Created on: 25.05.2015
 *      Author: espero
 */

#include "usleep_local.h"


void usleep_local(__int64 usec)
{
    HANDLE timer = CreateWaitableTimer(NULL, TRUE, NULL);
    LARGE_INTEGER ft;
    ft.QuadPart = -((10 * usec) / 5);				// Convert to 100 nanosecond interval, negative value indicates relative time

    SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
    WaitForSingleObject(timer, INFINITE);
    CloseHandle(timer);
}
