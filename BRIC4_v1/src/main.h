/*
 * main.h
 *
 * Created: 9/29/2019 8:29:48 PM
 *  Author: Kris Fausnight
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#include <asf.h>


//Common Constants:
#define SOFTWARE_VERSION 4.0
#define NL	0x0a  //  New Line ("\n")
#define CR	0x0d   //  New Line ("\n")
#define DEG2RAD		(PI/180)//0.0174532925 // (pi/180) degrees to radians
#define MT2FT		3.28084 //  1 meter = 3.28084 feet
#define RAD2DEG		(180/PI)//57.2957795  // (180/pi) radians to degrees
#define MAX_ERRORS 4 //  Max number of error codes to store with each measurement
#define SHOT_DELAY_MAX 5 //seconds, max setting
#define MIN_ERROR_SENSITIVITY 0.2 // degrees; Error Checking Sensitivity
#define MAX_ERROR_SENSITIVITY 3  // degrees; Error Checking Sensitivity
#define STEP_ERROR_SENSITIVITY 0.2 // degrees; Error Checking Sensitivity
#define DELTA_ANG_MIN		8 // degrees;  Threshold to register as new 4-point group.
#define NBUFF_MEAS	6 //  Size of measurement buffer
#define NBUFF		100//  Size of data buffers
#define NGROUP		25 //  Max number of groups; should be NBUFF/GROUP_SIZE
#define NBUFFQAZM	5//  Quick AZM Calibration Temporary Buffer Length
#define QAZM_STDEV_MIN	0.005 // Quick AZM Calibration Stability Requirement Stdev
#define GROUP_SIZE	4// For Azm/Inc calibration
#define SHOT_SIZE   4// For Distance calibration
#define MIN_GROUPS	9//const uint8_t min_groups = 9;// Minimum groups to complete a calibration
#define MAX_BAD_GROUPS	3 //  Maximum groups that can be eliminated in calibration routine.
#define BAD_GROUP_THRESHOLD		0.5//  Degrees, will eliminate group if improvement greater than this
#define DIST_CAL_SETPOINT_FT	3 //  Distance to set marker during distance calibration, feet
#define DIST_CAL_SETPOINT_MT	1 // Distance to set marker during distance calibration, meters
#define UART_BUFFER_LENGTH		100
#define DEBUG_BUFFER_LENGTH	200
//  Timing Definitions
#define DEBOUNCE_MS	300		// ms.  No new button input within this interval
#define DEBOUNCE_MS_QUICK3	300		// ms.  No new button input within this interval during turn-on with external button
#define QUICK3_MS	3000	// ms.  Must press ext button 3 times in this interval
#define OFF_HOLD_MS	3000  // ms. Hold down ext button for this long to shut off device
#define IDLE_MAX_S 60 // seconds.  Produces input_powerdown after this many seconds
#define LASER_TIMEOUT_S	30 // seconds.  Shuts down laser rangefinder if idle for this many seconds
#define MEASUREMENT_TIMEOUT 5000 //  When taking a measurement, wait this long for laser data to arrive back
//  Clock Assignments
#define GCLK_FOR_32khz				GCLK_GENERATOR_2
#define GCLK_FOR_SPI				GCLK_GENERATOR_0
#define GCLK_FOR_USART_LASER		GCLK_GENERATOR_0
#define GCLK_FOR_USART_BLE			GCLK_GENERATOR_0
#define GCLK_FOR_I2C				GCLK_GENERATOR_0
#define GCLK_FOR_TIMERS				GCLK_GENERATOR_2
//  Note:  External Interrupts Clocked on ULP32K in "conf_extint.h":  #define EXTINT_CLOCK_SELECTION   EXTINT_CLK_ULP32K



enum CALTYPE{inc_azm_full, azm_quick, rangeFinder};





struct BACKLIGHT_COLOR
{
	char *colorStringPtr;
	uint8_t red;
	uint8_t blue;
	uint8_t green;
};

struct BACKLIGHT_SETTING{
	uint8_t brightness;
	uint8_t colorRef;
};



struct TIME {
	uint8_t	seconds;
	uint8_t	minutes;
	uint8_t	hours;
	uint8_t day; //day of the week
	uint8_t	date; //day of the month
	uint8_t	month;
	uint8_t year; // Last two digits only, i.e. "18" for 2018
	uint8_t control;
	uint8_t control_status;
	float temperatureC;
	float temperatureF;
};

enum MEAS_ERROR_TYPE{
	no_error =			0,
	accel1_mag_err =	1,
	accel2_mag_err =	2,
	comp1_mag_err =		3,
	comp2_mag_err =		4,
	accel_disp_err =	5,
	comp_disp_err =		6,
	laser_calc_err=			7,
	laser_weak_signal	=	8,
	laser_strong_signal	=	9,
	laser_pattern_error	=	10,
	laser_response_timeout= 11,
	laser_unknown=			12,
	laser_wrong_message =	13,
	inc_ang_err=			14,
	azm_ang_err=			15,
	make8bit_measurement_error_type		= 0xff
};

typedef enum {
	feet			= 0,
	meters			= 1,
	perc_grade		= 2,
	degrees			= 3,
	celsius			= 4,
	fahrenheit		= 5,
	make8bit1		= 0xff
} UNIT_TYPE;


struct MEASUREMENT{
	uint32_t index_ref;//
	uint32_t posix_time;//  POSIX time, number of seconds since Jan 1 1970
	float temperature; // Temperature in celsius
	float azimuth, inclination, roll, declination;// Processed readings
	float distRaw, distCal;
	float a1Raw[3];// raw data, accelerometer 1
	float a2Raw[3];// raw data, accelerometer 2
	float m1Raw[3];// raw data, compass 1
	float m2Raw[3];// raw data, compass 2
	float a1Cal[3];// raw data, accelerometer 1
	float a2Cal[3];// raw data, accelerometer 2
	float m1Cal[3];// raw data, compass 1
	float m2Cal[3];// raw data, compass 2
	uint32_t samples;
	UNIT_TYPE distance_units;
	UNIT_TYPE temp_units;
	//  Time to take measurement
	uint32_t readTimeMs;
	//  Measurement Error Data
	uint32_t num_errors;
	enum MEAS_ERROR_TYPE measurement_error[MAX_ERRORS];
	float measurement_error_data1[MAX_ERRORS];
	float measurement_error_data2[MAX_ERRORS];
	
};



//  Global Variables
extern uint32_t nGroups;//Counter for current added groups
extern uint32_t nPoints;// Counter for current # of points
extern float a1Raw[NBUFF][3], a2Raw[NBUFF][3], m1Raw[NBUFF][3], m2Raw[NBUFF][3];
extern float a1Cal[NBUFF][3], a2Cal[NBUFF][3], m1Cal[NBUFF][3], m2Cal[NBUFF][3];
extern float azimuth[NBUFF], inclination[NBUFF], roll[NBUFF];
// Distance Calibration
extern float dist_raw_buf[SHOT_SIZE];
extern float dist_disp_buf[SHOT_SIZE];
extern float temp_dist_offset;
// Loop Test
extern float loop_distance, loop_horizontal, loop_vertical, loop_azimuth, loop_error;
//   Calibration data structures
extern struct INST_CAL a1_calst, a2_calst, m1_calst, m2_calst, dist_calst;
extern struct CAL_REPORT cal_report;


//  Other global variables
extern volatile enum INPUT current_input, last_input; // Current and Last Inputs
extern volatile enum STATE current_state; //  Current State

extern struct OPTIONS options;
//  SD Card variables
extern FATFS FatFS;         /* Work area (file system object) for logical drives */
//extern FIL file1, file2, file_cal_report, file_cal_raw;      /* file objects */
extern FRESULT SD_status;
extern char filename[100];
extern char write_str1[400];
extern char write_str2[400];
// Buffer to hold characters for display
extern char display_str[200];
//  Time structures
extern struct TIME current_time, temp_time;
//  Status variables
extern volatile bool isCharging;//  Variable to track when plugged in and charging
extern volatile bool buttonE_triggered; // Variable to track when external button is triggered
//  Debug
extern uint32_t debug_ct1, debug_ct2;

//UART Buffers
extern volatile uint8_t rxBufferLaser[UART_BUFFER_LENGTH];
extern volatile uint8_t rxBufferLaserIndex;
extern volatile char rxBufferBle[UART_BUFFER_LENGTH];
extern volatile uint8_t rxBufferBleIndex;
extern volatile char debugBuff[DEBUG_BUFFER_LENGTH];
extern volatile uint32_t debugBuffIndex;
extern char *debugBuffPtr;

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
#define BLE_COMMAND_MODE		IOPORT_CREATE_PIN(IOPORT_PORTA, 22)//CS5
#define V2_enable	IOPORT_CREATE_PIN(IOPORT_PORTB, 2)
#define laser_reset IOPORT_CREATE_PIN(IOPORT_PORTA, 2)
#define LCD_SPI_SS_PIN   IOPORT_CREATE_PIN(IOPORT_PORTA, 23)//LCD SS
#define LCD_SPI_DC_PIN   IOPORT_CREATE_PIN(IOPORT_PORTA, 27)//LCD A0
#define LCD_SPI_RST_PIN  IOPORT_CREATE_PIN(IOPORT_PORTB, 23)//LCD RST
#define BLE_ota  IOPORT_CREATE_PIN(IOPORT_PORTB, 22)//BL OTA
#define BLE_autorun	IOPORT_CREATE_PIN(IOPORT_PORTA, 14)//BL Auto Run
#define BLE_reset	IOPORT_CREATE_PIN(IOPORT_PORTA, 3)


struct OPTIONS{
	uint32_t SerialNumber;
	UNIT_TYPE current_unit_dist;
	UNIT_TYPE current_unit_temp;
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
	input_inc_azm_full_calibration,
	input_disp_cal_report,
	input_azm_quick_calibration,
	input_loop_test,
	input_debug_rawData,
	input_debug_backlight,
	input_debug_charger,
	input_wakeup,
	input_laser_timeout,
	input_menu_debug,
	input_usb_transaction,
	input_reprocess_inc_azm_cal,
	input_reprocess_azm_quick_cal,
	input_BLE_message,
	
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
	st_inc_azm_full_calibration,
	st_azm_quick_calibration,
	st_dist_calibration,
	st_process_inc_azm_full_cal,
	st_process_azm_quick_cal,
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








#include <arm_math.h>
#include <timers.h>
#include <BLE_func.h>
#include <comms/comms.h>
#include <clockSetup.h>
#include <sensors.h>
#include <calibration.h>
#include <backlight.h>
#include <dispFunctions.h>
#include <mathBRIC.h>
#include <SDcardBRIC.h>

#include <EEPROM.h>


#include <batteryManagement.h>
#include "glcd\glcd.h"
#include "glcd\fonts\font5x7.h"





















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
void fn_inc_azm_full_calibration(void);
void fn_process_inc_azm_full_cal(void);
void fn_process_azm_quick_cal(void);
void fn_disp_cal_report(void);
void fn_azm_quick_calibration(void);
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
void getDefaultOptions(struct OPTIONS *);
enum INPUT externalButtonRoutine(bool, uint32_t);


//USB



#endif /* MAIN_H_ */