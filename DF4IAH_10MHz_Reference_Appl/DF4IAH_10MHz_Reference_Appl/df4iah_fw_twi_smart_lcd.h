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
#define TWI_SMART_LCD_CMD_SHOW_PPM							0x83

#define TWI_SMART_LCD_CMD_SHOW_TCXO_PWM						0x84
#define TWI_SMART_LCD_CMD_SHOW_TCXO_VC						0x85

#define TWI_SMART_LCD_CMD_SHOW_SATS							0x88
#define TWI_SMART_LCD_CMD_SHOW_DOP							0x89
#define TWI_SMART_LCD_CMD_SHOW_POS_STATE					0x8A
#define TWI_SMART_LCD_CMD_SHOW_POS_LAT						0x8B
#define TWI_SMART_LCD_CMD_SHOW_POS_LON						0x8C
#define TWI_SMART_LCD_CMD_SHOW_POS_HEIGHT					0x8D


void twi_smart_lcd_fw_init(void);
void twi_smart_lcd_fw_close(void);

uint8_t twi_smart_lcd_fw_get_version(void);
void twi_smart_lcd_fw_set_clk_state(uint8_t clk_state, int16_t phase100);
void twi_smart_lcd_fw_set_date(uint16_t year, uint8_t month, uint8_t day);
void twi_smart_lcd_fw_set_time(uint8_t hour, uint8_t minute, uint8_t second);
void twi_smart_lcd_fw_set_ppm(int16_t ppm_int, uint16_t ppm_frac);
void twi_smart_lcd_fw_set_pwm(uint8_t pwm_int, uint8_t pwm_frac);
void twi_smart_lcd_fw_set_pv(uint8_t pv_int, uint16_t pv_frac);
void twi_smart_lcd_fw_set_sat_use(uint8_t sat_west, uint8_t sat_east, uint8_t sat_used);
void twi_smart_lcd_fw_set_sat_dop(uint16_t sat_dop100);
void twi_smart_lcd_fw_set_pos_state(uint8_t state_fi, uint8_t state_m2);
void twi_smart_lcd_fw_set_pos_lat(uint8_t lat_sgn, uint8_t lat_deg, uint8_t lat_min_int, uint16_t lat_min_frac10000);
void twi_smart_lcd_fw_set_pos_lon(uint8_t lon_sgn, uint8_t lon_deg, uint8_t lon_min_int, uint16_t lon_min_frac10000);
void twi_smart_lcd_fw_set_pos_height(uint16_t height);


#endif /* DF4IAH_FW_TWI_SMART_LCD_H_ */