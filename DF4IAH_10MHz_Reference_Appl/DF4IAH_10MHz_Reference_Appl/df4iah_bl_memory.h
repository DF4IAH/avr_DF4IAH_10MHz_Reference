/*
 * df4iah_bl_memory.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_BL_MEMORY_H_
#define DF4IAH_BL_MEMORY_H_


/*
 * define the following if the bootloader should not output
 * itself at flash read (will fake an empty boot-section)
 */
#define READ_PROTECT_BOOTLOADER


#include <stdint.h>

#include "chipdef.h"


void memory_bl_eraseFlash(void);
void memory_bl_readFlashPage(uint8_t target[], pagebuf_t size, uint32_t baddr);
void memory_bl_readEEpromPage(uint8_t target[], pagebuf_t size, uint16_t baddr);
void memory_bl_writeFlashPage(uint8_t source[], pagebuf_t size, uint32_t baddr);
void memory_bl_writeEEpromPage(uint8_t source[], pagebuf_t size, uint16_t baddr);

#endif /* DF4IAH_BL_MEMORY_H_ */
