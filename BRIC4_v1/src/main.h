/*
 * main.h
 *
 * Created: 9/29/2019 8:29:48 PM
 *  Author: Kris Fausnight
 */ 


#ifndef MAIN_H_
#define MAIN_H_


#include <asf.h>


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
	input_menu_debug
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
	st_menu_debug
};












#include "glcd\glcd.h"
#include "glcd\fonts\font5x7.h"
#include "FatFS_R13C\ff.h"
#include "FatFS_R13C\diskio.h"
#include <arm_math.h>
#include <extern_clock\extern_clock.h>
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
// Other functions
void print_data_screen(void);
void input_handler(void);
void config_pins_powerup(void);
void config_pins_powerdown(void);
void cal_disp_message(void);
void draw_arrows(uint8_t);
void getDefaultOptions(struct OPTIONS *);
void externalButtonRoutine(bool);


#endif /* MAIN_H_ */