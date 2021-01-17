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
#define SOFTWARE_VERSION 5.5
#define HARDWARE_VERSION "A"
#define NL	0x0a  //  New Line ("\n")
#define CR	0x0d   //  New Line ("\n")
#define DEG2RAD		(PI/180)//0.0174532925 // (pi/180) degrees to radians
#define MT2FT		3.28084 //  1 meter = 3.28084 feet
#define RAD2DEG		(180/PI)//57.2957795  // (180/pi) radians to degrees
#define MAX_ERRORS 2 //  Max number of error codes to store with each measurement
#define SHOT_DELAY_MAX 5 //seconds, max setting
#define MIN_ERROR_SENSITIVITY 0.2 // degrees; Error Checking Sensitivity
#define MAX_ERROR_SENSITIVITY 3  // degrees; Error Checking Sensitivity
#define STEP_ERROR_SENSITIVITY 0.2 // degrees; Error Checking Sensitivity
#define DELTA_ANG_MIN		8 // degrees;  Threshold to register as new 4-point group.
#define N_MEASBUF	10 //  Size of measurement buffer
#define NBUFF		100//  Size of data buffers
#define NGROUP		25 //  Max number of groups; should be NBUFF/GROUP_SIZE
#define NBUFFQAZM	5//  Quick AZM Calibration Temporary Buffer Length
#define QAZM_STDEV_MIN	0.005 // Quick AZM Calibration Stability Requirement Stdev
#define GROUP_SIZE	4// For Azm/Inc calibration
#define SHOT_SIZE   4// For Distance calibration
#define MIN_GROUPS	14// Minimum groups to complete a calibration
#define MAX_BAD_GROUPS	3 //  Maximum groups that can be eliminated in calibration routine.
#define BAD_GROUP_THRESHOLD		0.4//  Degrees, will eliminate group if improvement greater than this
#define DIST_CAL_SETPOINT_FT	3.0 //  Distance to set marker during distance calibration, feet
#define DIST_CAL_SETPOINT_MT	1.0 // Distance to set marker during distance calibration, meters
#define REF_INDEX_MAX			9999  // Maximum reference index, limited by display
#define UART_BUFFER_LENGTH		100
#define DEBUG_BUFFER_LENGTH	100
//  Timing Definitions	
#define DEBOUNCE_MS 150		// ms.  No new button input within this interval
#define QUICK3_MS	1000	// ms.  Must press ext button 3 times each within this interval
#define OFF_HOLD_MS	3000  // ms. Hold down ext button for this long to shut off device
#define IDLE_MAX_S 90 // seconds.  Produces input_powerdown after this many seconds
#define LASER_TIMEOUT_S	30 // seconds.  Shuts down laser rangefinder if idle for this many seconds
#define MEASUREMENT_TIMEOUT 5000 //  Ms, When taking a measurement, wait this long for laser data to arrive back
//  Clock Assignments
#define GCLK_FOR_32khz				GCLK_GENERATOR_2
#define GCLK_FOR_SPI				GCLK_GENERATOR_0
#define GCLK_FOR_USART_LASER		GCLK_GENERATOR_0
#define GCLK_FOR_USART_BLE			GCLK_GENERATOR_0
#define GCLK_FOR_I2C				GCLK_GENERATOR_0
#define GCLK_FOR_TIMERS				GCLK_GENERATOR_2
//  Note:  External Interrupts Clocked on ULP32K in "conf_extint.h":  #define EXTINT_CLOCK_SELECTION   EXTINT_CLK_ULP32K



enum CALTYPE{inc_azm_full, azm_quick, rangeFinder};


enum MEAS_TYPE{measRegular, measScan, measQuick, measCal};


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
	uint16_t year;
	uint8_t	month;
	uint8_t	day; //day of the month
	uint8_t	hours;
	uint8_t	minutes;
	uint8_t	seconds;
	uint8_t centiseconds;
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



struct MEASUREMENT_FULL{
	uint32_t refIndex;//  Ref 0-REF_INDEX_MAX
	uint32_t posix_time;//  POSIX time, number of seconds since Jan 1 1970
	struct TIME measTime;
	float distMeters;  //  Distance in Meters
	float azimuth;
	float inclination;
	float dip;
	float roll;
	float temperatureC; // Temperature in celsius
	enum MEAS_TYPE meas_type;
	uint16_t samples;
	enum MEAS_ERROR_TYPE  errCode[2];
	float measurement_error_data1[2];
	float measurement_error_data2[2];
	
	//  Additional data
	float distRaw;
	
	//  Time to take measurement
	uint32_t readTimeMs;
	float a1Raw[3];// raw data, accelerometer 1
	float a2Raw[3];// raw data, accelerometer 2
	float m1Raw[3];// raw data, compass 1
	float m2Raw[3];// raw data, compass 2
	float a1Cal[3];// raw data, accelerometer 1
	float a2Cal[3];// raw data, accelerometer 2
	float m1Cal[3];// raw data, compass 1
	float m2Cal[3];// raw data, compass 2
	

	
};


struct MEASUREMENT{
	uint32_t refIndex;//  Ref 0-REF_INDEX_MAX
	uint32_t posix_time;//  POSIX time, number of seconds since Jan 1 1970
	struct TIME measTime;
	float distMeters;  //  Distance in Meters
	float azimuth;
	float inclination;
	float dip;
	float roll;
	float temperatureC; // Temperature in celsius
	enum MEAS_TYPE meas_type;
	uint16_t samples;
	enum MEAS_ERROR_TYPE  errCode[2];
	float measurement_error_data1[2];
	float measurement_error_data2[2];
	
};

struct BLE_SYNC_TRACKER{
	uint8_t keyMeasTrackerInitialized;
	struct TIME timeEnd;
	struct TIME timeSync;
	uint32_t refIndexEnd;
	uint32_t refIndexSync;
	uint32_t measStackEnd;
	uint32_t measStackSync;
	
	
};

// Measurement Display Data Buffers
extern struct MEASUREMENT measBuf[N_MEASBUF];//  Circular buffer
extern uint32_t measBufInd;
extern uint32_t refIndex ;


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
//  State machine variables
extern volatile enum INPUT current_input, last_input; // Current and Last Inputs
extern volatile enum STATE current_state, last_state; //  Current State
extern volatile bool state_change;

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
//  Measurement Tracker
extern struct BLE_SYNC_TRACKER bleSyncTracker;
//  Time structures
extern struct TIME current_time;
//  Temperature
extern float currentTempC;
//  Status variables
extern volatile bool isCharging;//  Variable to track when plugged in and charging
extern volatile bool SD_WriteLockout;
extern volatile bool buttonE_triggered; // Variable to track when external button is triggered
extern char BleClientMAC[20];
extern char BleDeviceMAC[20]; //  This device (BRIC4) MAC address
extern char BleDeviceName[20]; // BRIC4 device name (e.g. "BRIC4_0039")
extern char BleCommandQueue[20];//  Queued BLE command
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
extern char *bleBuffPtr;
extern volatile char laserDebugBuff[DEBUG_BUFFER_LENGTH];
extern volatile uint32_t laserDebugBuffIndex;
extern char *laserDebugBuffPtr;

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
#define V2_enable	IOPORT_CREATE_PIN(IOPORT_PORTB, 2)
#define laser_reset IOPORT_CREATE_PIN(IOPORT_PORTA, 2)
#define LCD_SPI_SS_PIN   IOPORT_CREATE_PIN(IOPORT_PORTA, 23)//LCD SS
#define LCD_SPI_DC_PIN   IOPORT_CREATE_PIN(IOPORT_PORTA, 27)//LCD A0
#define LCD_SPI_RST_PIN  IOPORT_CREATE_PIN(IOPORT_PORTB, 23)//LCD RST
//  Pin to indicate outgoing data
#define BLE_rcvMode_pin	 IOPORT_CREATE_PIN(IOPORT_PORTA, 22)//CS5
//  Auto-Run pin.  Low enables auto-run of downloaded applications.
#define BLE_nAUTORUN_pin	IOPORT_CREATE_PIN(IOPORT_PORTA, 14)//BL Auto Run
//  BL652 Over The Air Enable 
//  High enables OTA downloads
#define BLE_OTA_sendMode_pin  IOPORT_CREATE_PIN(IOPORT_PORTB, 22)//BL OTA
//  BL652 Reset Pin
//  Low holds in reset, high is run mode
#define BLE_nRESET_pin	IOPORT_CREATE_PIN(IOPORT_PORTA, 3)
#define BuzzerPin		IOPORT_CREATE_PIN(IOPORT_PORTB, 3)


struct OPTIONS{
	uint8_t keySettingsInitialized;
	uint32_t SerialNumber;
	UNIT_TYPE current_unit_dist;
	UNIT_TYPE current_unit_temp;
	uint8_t shot_delay;
	uint32_t chargeCurrent;
	float errorSensitivity;

	struct BACKLIGHT_SETTING backlight_setting;
	
};



#define N_GENERIC_INPUTS 6
// Inputs
enum INPUT {
	//  Generic	
	//  Must be first 6
	input_button1,
	input_button2,
	input_button3,
	input_button4,
	input_buttonE,
	input_pwrDown,
	
	//  Background Inputs	
	input_none,
	input_1sec,
	input_BLE_message,
	input_BLE_command,
	input_usb_transaction,
	
	//  Time related
	input_laser_timeout,
	input_state_complete,

};

// States
enum STATE{
	st_NULL,//  Must be first
	//st_aim,
	//st_measure,
	st_scan,
	st_main_display,
	st_menu1,
	st_powerdown,
	st_powerup,
	st_set_clock,
	st_menu_BLE,
	st_set_options,
	//st_aim_abort,
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
	st_usb_process,
	st_debug_BLE,
	st_firmware,
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
#include <Buzzer.h>
#include <errorsBRIC4.h>
#include <EEPROM.h>
#include <buttons.h>

#include <batteryManagement.h>
#include "glcd\glcd.h"
#include "glcd\fonts\font5x7.h"





















//  State functions
void fn_main_display(void);
void fn_aim(void);
void fn_measure(void);
void fn_scan(void);
void fn_powerdown(void);
void fn_powerup(void);
void fn_set_clock(void);
void fn_menu_BLE(void);
void fn_debug_BLE(void);
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
void fn_firmware(void);
// Other functions
void input_handler(void);
void config_pins_powerup(void);
void config_pins_powerdown(void);
void cal_disp_message(void);
void getDefaultOptions(struct OPTIONS *);
void processMeasurement(struct MEASUREMENT_FULL *);
void UsbHandleTransactions(void);
//USB



#endif /* MAIN_H_ */