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
#define TWI_SMART_LCD_CMD_NOOP								0x00

#define TWI_SMART_LCD_CMD_GETVER							0x01

#define TWI_SMART_LCD_CMD_SHOW_CLK_STATE					0x80
#define TWI_SMART_LCD_CMD_SHOW_YEAR_MON_DAY					0x81
#define TWI_SMART_LCD_CMD_SHOW_HR_MIN_SEC					0x82
#define TWI_SMART_LCD_CMD_SHOW_PPM_INT16_FRAC16				0x83

#define TWI_SMART_LCD_CMD_SHOW_TCXO_PWM						0x84
#define TWI_SMART_LCD_CMD_SHOW_TCXO_VC						0x85

#define TWI_SMART_LCD_CMD_SHOW_SATS							0x88
#define TWI_SMART_LCD_CMD_SHOW_DOP							0x89
#define TWI_SMART_LCD_CMD_SHOW_POS_STATE					0x8A
#define TWI_SMART_LCD_CMD_SHOW_POS_LAT						0x8B
#define TWI_SMART_LCD_CMD_SHOW_POS_LON						0x8C
#define TWI_SMART_LCD_CMD_SHOW_POS_HEIGHT					0x8D


void twi_smart_lcd_fw_init(void);
uint8_t twi_smart_lcd_fw_get_version(void);
void twi_smart_lcd_fw_close(void);


#endif /* DF4IAH_FW_TWI_SMART_LCD_H_ */