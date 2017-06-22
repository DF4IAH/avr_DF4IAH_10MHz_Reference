/*
 * df4iah_fw_twi_smart_lcd.h
 *
 * Created: 25.03.2017 20:11:11
 *  Author: espero
 */


#ifndef DF4IAH_FW_TWI_SMART_LCD_H_
#define DF4IAH_FW_TWI_SMART_LCD_H_


#include "chipdef.h"


/* Smart-LCD address and command-set */

// I2C Address
#define TWI_SMART_LCD_ADDR									0x22


// The unique commands of the Smart-LCD device for all modes
#define TWI_SMART_LCD_CMD_NOOP								0x00
#define TWI_SMART_LCD_CMD_GET_VER							0x01
#define TWI_SMART_LCD_CMD_SET_MODE							0x02
#define TWI_SMART_LCD_CMD_GET_STATE							0x03


// Mode 0x20 commands (10 MHz-Ref-Osc)
#define TWI_SMART_LCD_CMD_SHOW_CLK_STATE					0x80
#define TWI_SMART_LCD_CMD_SHOW_YEAR_MON_DAY					0x81
#define TWI_SMART_LCD_CMD_SHOW_HR_MIN_SEC					0x82
#define TWI_SMART_LCD_CMD_SHOW_PPB							0x83

#define TWI_SMART_LCD_CMD_SHOW_TCXO_PWM						0x84
#define TWI_SMART_LCD_CMD_SHOW_TCXO_VC						0x85

#define TWI_SMART_LCD_CMD_SHOW_SATS							0x88
#define TWI_SMART_LCD_CMD_SHOW_DOP							0x89
#define TWI_SMART_LCD_CMD_SHOW_POS_STATE					0x8A
#define TWI_SMART_LCD_CMD_SHOW_POS_LAT						0x8B
#define TWI_SMART_LCD_CMD_SHOW_POS_LON						0x8C
#define TWI_SMART_LCD_CMD_SHOW_POS_HEIGHT					0x8D


void twi_smart_lcd_fw_init(void);
#if 0
void twi_smart_lcd_fw_close(void);
#endif

uint8_t twi_smart_lcd_fw_get_version(void);
void twi_smart_lcd_fw_set_mode(uint8_t mode);
void twi_smart_lcd_fw_set_clkstate_phaseVolt__phaseDeg(uint8_t clk_state, uint16_t phaseVolt1000, int16_t phaseDeg100);
void twi_smart_lcd_fw_set_date(uint16_t year, uint8_t month, uint8_t day);
void twi_smart_lcd_fw_set_time(uint8_t hour, uint8_t minute, uint8_t second);
void twi_smart_lcd_fw_set_ppb(int16_t ppb_int, uint16_t ppb_frac1000);
void twi_smart_lcd_fw_set_pwm(uint8_t pwm_int, uint8_t pwm_frac256);
void twi_smart_lcd_fw_set_pv(uint8_t pv_int, uint16_t pv_frac1000);
void twi_smart_lcd_fw_set_sat_use(uint8_t sat_west, uint8_t sat_east, uint8_t sat_used);
void twi_smart_lcd_fw_set_sat_dop(uint16_t sat_dop100);
void twi_smart_lcd_fw_set_pos_state(uint8_t state_fi, uint8_t state_m2);
void twi_smart_lcd_fw_set_pos_lat(uint8_t lat_sgn, uint8_t lat_deg, uint8_t lat_min_int, uint16_t lat_min_frac10000);
void twi_smart_lcd_fw_set_pos_lon(uint8_t lon_sgn, uint8_t lon_deg, uint8_t lon_min_int, uint16_t lon_min_frac10000);
void twi_smart_lcd_fw_set_pos_height(uint16_t height_int, uint8_t height_frac10);


#endif /* DF4IAH_FW_TWI_SMART_LCD_H_ */