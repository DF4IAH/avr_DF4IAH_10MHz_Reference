/*
 * df4iah_fw_twi_smart_lcd.h
 *
 * Created: 25.03.2017 20:11:11
 *  Author: espero
 */ 


#ifndef DF4IAH_FW_TWI_SMART_LCD_H_
#define DF4IAH_FW_TWI_SMART_LCD_H_


#include "chipdef.h"


// I2C Address
#define TWI_SMART_LCD_ADDR									0x22

// commands of the Smart-LCD device
#define TWI_SMART_LCD_CMD_GETVER							0x01


void twi_smart_lcd_fw_init(void);
uint8_t twi_smart_lcd_fw_get_version(void);
void twi_smart_lcd_fw_close(void);


#endif /* DF4IAH_FW_TWI_SMART_LCD_H_ */