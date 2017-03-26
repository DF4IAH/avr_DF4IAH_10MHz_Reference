/*
 * df4iah_fw_twi_smart_lcd.c
 *
 * Created: 25.03.2017 20:13:50
 *  Author: DF4IAH
 */ 

#include <stdbool.h>

#include "df4iah_fw_main.h"
#include "df4iah_fw_twi.h"

#include "df4iah_fw_twi_smart_lcd.h"


extern volatile main_bf_t main_bf;


void twi_smart_lcd_fw_init(void)
{
	uint8_t ver = twi_smart_lcd_fw_get_version();

	if (!main_bf.mainIsSmartAttached) {
		return;
	}

	// TODO
	(void) ver;
}

void twi_smart_lcd_fw_close(void)
{
	if (!main_bf.mainIsSmartAttached) {
		return;
	}

	// TODO
}

uint8_t twi_smart_lcd_fw_get_version(void)
{
	return twi_fw_sendCmdReadData1(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_GETVER);		// within this function the main_bf.mainIsSmartAttached is being set
}
