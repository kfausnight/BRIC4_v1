/*
 * main.h
 *
 * Created: 9/29/2019 8:29:48 PM
 *  Author: Kris Fausnight
 */ 


#ifndef MAIN_H_
#define MAIN_H_
//  USB funcations


#include <asf.h>

//#include "conf_usb.h"


// Physical Pin Defines
#define button1	IOPORT_CREATE_PIN(IOPORT_PORTB, 9)
#define button2	IOPORT_CREATE_PIN(IOPORT_PORTA, 4)
#define button3	IOPORT_CREATE_PIN(IOPORT_PORTA, 6)
#define button4	IOPORT_CREATE_PIN(IOPORT_PORTA, 7)
#define buttonE	IOPORT_CREATE_PIN(IOPORT_PORTA, 5)
#define SDA		IOPORT_CREATE_PIN(IOPORT_PORTA, 12)
#define SCL		IOPORT_CREATE_PIN(IOPORT_PORTA, 13)
#define MCU_TX1	IOPORT_CREATE_PIN(IOPORT_PORTA, 8)//bluetooth module TX
#define MCU_RX1	IOPORT_CREATE_PIN(IOPORT_PORTA, 9)//bluetooth module RX
#define MCU_RTS1		IOPORT_CREATE_PIN(IOPORT_PORTA, 10)//bluetooth module RTS
#define MCU_CTS1		IOPORT_CREATE_PIN(IOPORT_PORTA, 11)//bluetooth module CTS
#define MCU_TX2	IOPORT_CREATE_PIN(IOPORT_PORTA, 16)
#define MCU_RX2	IOPORT_CREATE_PIN(IOPORT_PORTA, 17)
#define mosi	IOPORT_CREATE_PIN(IOPORT_PORTB, 10)//SPI MOSI
#define miso	IOPORT_CREATE_PIN(IOPORT_PORTB, 8)//SPI MISO
#define sclk	IOPORT_CREATE_PIN(IOPORT_PORTB, 11)//SPI SCLK
#define lcd_SS	IOPORT_CREATE_PIN(IOPORT_PORTA, 23)//CS6
#define acc1_SS	IOPORT_CREATE_PIN(IOPORT_PORTA, 18)//CS1
#define acc2_SS	IOPORT_CREATE_PIN(IOPORT_PORTA, 19)//CS2
#define mag1_SS IOPORT_CREATE_PIN(IOPORT_PORTA, 21)//CS4
#define mag2_SS IOPORT_CREATE_PIN(IOPORT_PORTA, 20)//CS3
#define SD_CS	IOPORT_CREATE_PIN(IOPORT_PORTA, 15)//CS7
#define BLE_SS		IOPORT_CREATE_PIN(IOPORT_PORTA, 22)//CS5
#define V2_enable	IOPORT_CREATE_PIN(IOPORT_PORTB, 2)
#define laser_reset IOPORT_CREATE_PIN(IOPORT_PORTA, 2)
#define LCD_SPI_SS_PIN   IOPORT_CREATE_PIN(IOPORT_PORTA, 23)//LCD SS
#define LCD_SPI_DC_PIN   IOPORT_CREATE_PIN(IOPORT_PORTA, 27)//LCD A0
#define LCD_SPI_RST_PIN  IOPORT_CREATE_PIN(IOPORT_PORTB, 23)//LCD RST
#define BLE_ota  IOPORT_CREATE_PIN(IOPORT_PORTB, 22)//BL OTA
#define BLE_autorun	IOPORT_CREATE_PIN(IOPORT_PORTA, 14)//BL Auto Run
#define BLE_reset	IOPORT_CREATE_PIN(IOPORT_PORTA, 3)
//#define SD_CS		IOPORT_CREATE_PIN(IOPORT_PORTB, 3)



typedef enum {
	feet			= 0,
	meters			= 1,
	perc_grade		= 2,
	degrees			= 3,
	celsius			= 4,
	fahrenheit		= 5,
	make8bit1		= 0xff
} unit_type;


struct BACKLIGHT_SETTING{
	uint8_t brightness;
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t maxColor;
	uint8_t maxBrightness;
	uint8_t colorRef;
};


struct OPTIONS{
	unit_type current_unit_dist;
	unit_type current_unit_temp;
	uint8_t shot_delay;
	uint32_t chargeCurrent;
	float errorSensitivity;

	struct BACKLIGHT_SETTING backlight_setting;
	uint8_t Settings_Initialized_Key;
};


// Inputs
enum INPUT {
	input_none, 
	input_button1, 
	input_button2, 
	input_button3,
	input_button4, 
	input_buttonE, 
	input_powerdown, 
	input_1sec, 
	input_state_complete,
	input_set_clock, 
	input_set_bluetooth, 
	input_set_units, 
	input_error_info,
	input_cal_menu, 
	input_dist_calibration, 
	input_acc_comp_calibration, 
	input_disp_cal_report, 
	input_loop_test,
	input_debug_rawData, 
	input_debug_backlight, 
	input_debug_charger, 
	input_wakeup, 
	input_laser_timeout, 
	input_menu_debug,
	input_usb_transaction,
	};

// States
enum STATE{
	st_aim, 
	st_measure, 
	st_main_display, 
	st_menu1, 
	st_powerdown, 
	st_powerup,
	st_set_clock, 
	st_set_bluetooth, 
	st_set_options, 
	st_aim_abort,
	st_menu_cal, 
	st_acc_comp_calibration, 
	st_dist_calibration, 
	st_process_calibration, 
	st_disp_cal_report,
	st_loop_test, 
	st_disp_loop_report, 
	st_debug_rawData, 
	st_debug_backlight, 
	st_debug_charger,
	st_error_info, 
	st_menu_debug,
	st_usb_process
};












#include "glcd\glcd.h"
#include "glcd\fonts\font5x7.h"
//#include "FatFS_R13C\ff.h"
//#include "FatFS_R13C\diskio.h"
#include <arm_math.h>
#include <clockSetup.h>
#include <comms/comms.h>
#include <calibration.h>
#include <sensors.h>
#include <EEPROM.h>
#include <timers.h>
#include <backlight.h>
#include <batteryManagement.h>








//  State functions
void fn_main_display(void);
void fn_aim(void);
void fn_measure(void);
void fn_powerdown(void);
void fn_powerup(void);

void fn_set_clock(void);
void fn_set_bluetooth(void);
void fn_set_options(void);
void fn_menu1(void);
void fn_menu_cal(void);
void fn_menu_debug(void);
void fn_aim_abort(void);
void fn_dist_calibration(void);
void fn_acc_comp_calibration(void);
void fn_process_calibration(void);
void fn_disp_cal_report(void);
void fn_loop_test(void);
void fn_disp_loop_report(void);
void fn_debug_rawData(void);
void fn_debug_backlight(void);
void fn_debug_charger(void);
void fn_error_info(void);
void fn_usb_process(void);
// Other functions
void print_data_screen(void);
void input_handler(void);
void config_pins_powerup(void);
void config_pins_powerdown(void);
void cal_disp_message(void);
void draw_arrows(uint8_t);
void getDefaultOptions(struct OPTIONS *);
enum INPUT externalButtonRoutine(bool);

//USB



#endif /* MAIN_H_ */