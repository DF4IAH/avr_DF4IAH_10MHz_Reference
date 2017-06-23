/*
 * df4iah_fw_twi_smart_lcd.c
 *
 * Created: 25.03.2017 20:13:50
 *  Author: DF4IAH
 */

#include <stdbool.h>
#include <util/delay.h>

#include "df4iah_fw_main.h"
#include "df4iah_fw_twi.h"

#include "df4iah_fw_twi_smart_lcd.h"


extern volatile main_bf_t main_bf;
extern uchar mainPrepareBuffer[MAIN_PREPARE_BUFFER_SIZE];


void twi_smart_lcd_fw_init(void)
{
	uint8_t ver = twi_smart_lcd_fw_get_version();
	if (ver >= 0x11 || main_bf.mainIsSmartAttached) {		// Smart-LCD detected
		twi_smart_lcd_fw_set_mode(C_SMART_LCD_MODE_REFOSC);
	}
}

uint8_t twi_smart_lcd_fw_get_version(void)
{
	uint8_t ver = twi_fw_sendCmdReadData1(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_GET_VER);	// within this function the main_bf.mainIsSmartAttached is being set;

	for (uint8_t i = 3; i; --i) {
		if (ver >= 0x10) {
			return ver;
		}
		_delay_ms(10);
		ver = twi_fw_sendCmdReadData1(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_GET_VER);
	}
	return ver;
}

void twi_smart_lcd_fw_set_mode(uint8_t mode)
{
	mainPrepareBuffer[0] = mode;
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SET_MODE, 1, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_clkstate_phaseVolt__phaseDeg(uint8_t clk_state, uint16_t phaseVolt1000, int16_t phaseDeg100)
{
	mainPrepareBuffer[0] = clk_state;
	mainPrepareBuffer[1] = (uint8_t) (phaseVolt1000 & 0xff);
	mainPrepareBuffer[2] = (uint8_t) (phaseVolt1000 >> 8);
	mainPrepareBuffer[3] = (uint8_t) (phaseDeg100   & 0xff);
	mainPrepareBuffer[4] = (uint8_t) (phaseDeg100   >> 8);
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_CLK_STATE, 5, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_date(uint16_t year, uint8_t month, uint8_t day)
{
	mainPrepareBuffer[0] = (uint8_t) (year & 0xff);
	mainPrepareBuffer[1] = (uint8_t) (year >>  8);
	mainPrepareBuffer[2] = month;
	mainPrepareBuffer[3] = day;
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_YEAR_MON_DAY, 4, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_time(uint8_t hour, uint8_t minute, uint8_t second)
{
	mainPrepareBuffer[0] = hour;
	mainPrepareBuffer[1] = minute;
	mainPrepareBuffer[2] = second;
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_HR_MIN_SEC, 3, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_ppb(int16_t ppb_int, uint16_t ppb_frac1000)
{
	mainPrepareBuffer[0] = (uint8_t) (ppb_int & 0xff);
	mainPrepareBuffer[1] = (uint8_t) (ppb_int >> 8);
	mainPrepareBuffer[2] = (uint8_t) (ppb_frac1000 & 0xff);
	mainPrepareBuffer[3] = (uint8_t) (ppb_frac1000 >> 8);
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_PPB, 4, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_pwm(uint8_t pwm_int, uint8_t pwm_frac256)
{
	mainPrepareBuffer[0] = pwm_int;
	mainPrepareBuffer[1] = pwm_frac256;
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_TCXO_PWM, 2, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_pv(uint8_t pv_int, uint16_t pv_frac)
{
	mainPrepareBuffer[0] = pv_int;
	mainPrepareBuffer[1] = (uint8_t) (pv_frac & 0xff);
	mainPrepareBuffer[2] = (uint8_t) (pv_frac >> 8);
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_TCXO_VC, 3, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_sat_use(uint8_t sat_west, uint8_t sat_east, uint8_t sat_used)
{
	mainPrepareBuffer[0] = sat_west;
	mainPrepareBuffer[1] = sat_east;
	mainPrepareBuffer[2] = sat_used;
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_SATS, 3, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_sat_dop(uint16_t sat_dop100)
{
	mainPrepareBuffer[0] = (uint8_t) (sat_dop100 & 0xff);
	mainPrepareBuffer[1] = (uint8_t) (sat_dop100 >> 8);
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_DOP, 2, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_pos_state(uint8_t state_fi, uint8_t state_m2)
{
	mainPrepareBuffer[0] = state_fi;
	mainPrepareBuffer[1] = state_m2;
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_POS_STATE, 2, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_pos_lat(uint8_t lat_sgn, uint8_t lat_deg, uint8_t lat_min_int, uint16_t lat_min_frac10000)
{
	mainPrepareBuffer[0] = (uint8_t) lat_sgn;
	mainPrepareBuffer[1] = (uint8_t) lat_deg;
	mainPrepareBuffer[2] = (uint8_t) lat_min_int;
	mainPrepareBuffer[3] = (uint8_t) (lat_min_frac10000 & 0xff);
	mainPrepareBuffer[4] = (uint8_t) (lat_min_frac10000 >> 8);
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_POS_LAT, 5, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_pos_lon(uint8_t lon_sgn, uint8_t lon_deg, uint8_t lon_min_int, uint16_t lon_min_frac10000)
{
	mainPrepareBuffer[0] = (uint8_t) lon_sgn;
	mainPrepareBuffer[1] = (uint8_t) lon_deg;
	mainPrepareBuffer[2] = (uint8_t) lon_min_int;
	mainPrepareBuffer[3] = (uint8_t) (lon_min_frac10000 & 0xff);
	mainPrepareBuffer[4] = (uint8_t) (lon_min_frac10000 >> 8);
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_POS_LON, 5, (uint8_t*) &mainPrepareBuffer);
}

void twi_smart_lcd_fw_set_pos_height(uint16_t height_int, uint8_t height_frac10)
{
	mainPrepareBuffer[0] = (uint8_t) (height_int & 0xff);
	mainPrepareBuffer[1] = (uint8_t) (height_int >> 8);
	mainPrepareBuffer[2] = (uint8_t)  height_frac10;
	twi_fw_sendCmdSendData1SendDataVar(TWI_SMART_LCD_ADDR, TWI_SMART_LCD_CMD_SHOW_POS_HEIGHT, 3, (uint8_t*) &mainPrepareBuffer);
}
