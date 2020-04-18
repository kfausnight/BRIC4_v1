
// Survey Instrument Main Code
// Kris Fausnight
// kfausnight@gmail.com


/*
* Include header files for all drivers that have been imported from
* Atmel Software Framework (ASF).
*/
/*
* Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
*/


#include <main.h>

// Debug
volatile uint8_t debug1, debug2, debug3, debug4;
bool USART_BLE_enabled;
bool debug_ota;

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


//Common Constants:
const float software_version = 4.0;
const float rad2deg = 57.2957795; // (180/pi) radians to degrees
const float deg2rad  =   0.0174532925; // (pi/180) degrees to radians
const float mt2ft	=	3.28084; //  1 meter = 3.28084 feet
// Global variables
#define shot_delay_max 5 //seconds, max setting
#define minErrorSensitivity 0.2
#define maxErrorSensitivity 3
#define incErrorSensitivity 0.2
char display_str[254];

volatile enum INPUT current_input, last_input;
volatile bool buttonE_triggered=false;

volatile enum STATE current_state;
volatile bool state_change = true;

volatile bool laser_triggered;
volatile uint8_t click_counter;//Used to count external button clicks to wake up 
struct OPTIONS options;


//  Calibration Data Buffers
// Azm and Inc Calibration
#define nbuf		80//  Size of data buffers
#define group_size	4// For Azm/Inc calibration
uint32_t n_groups;//Counter for current added groups
uint32_t n_points;// Counter for current # of points
float a1raw[nbuf][3], a2raw[nbuf][3], c1raw[nbuf][3], c2raw[nbuf][3];
float a1cal[nbuf][3], a2cal[nbuf][3], c1cal[nbuf][3], c2cal[nbuf][3];
float azimuth[nbuf], inclination[nbuf], roll[nbuf];
const uint8_t min_groups = 9;// Minimum groups to complete a calibration
uint8_t ind_buf, ind_stack, buf_points;
// Distance Calibration
#define shot_size   4// For Distance calibration
const float dist_cal_setpoint_ft = 3;//  Distance to set marker during distance calibration, feet
const float dist_cal_setpoint_mt = 1;//  Distance to set marker during distance calibration, meters
float dist_raw_buf[shot_size];
float dist_disp_buf[shot_size];
float temp_dist_offset;
// All Azm/Inc/Dist Calibration

// Loop Test
float loop_distance, loop_horizontal, loop_vertical, loop_azimuth, loop_error;
//   Calibration data structures
struct INST_CAL a1_calst, a2_calst, c1_calst, c2_calst, dist_calst;
struct CAL_REPORT cal_report_azm_inc, cal_report_dist;

// Measurement Data Buffers
#define buf_length	10
struct MEASUREMENT data_buf[buf_length];
uint8_t data_buf_ind = 0;
uint32_t data_ref = 0;

//  Interrupt Management
void configure_extint_channel(void);
void configure_extint_callbacks(void);
void extint_routine(void);


//Clock functions
enum clock_type {clock_ext, clock_int, clock_low, clock_high} ;
void setup_XOSC32k(void);
void clock_32k_source(enum clock_type);
void clock_16M_source(enum clock_type);

//SD card functions
FATFS FatFS;         /* Work area (file system object) for logical drives */
FIL file1, file2, file_cal_report, file_cal_raw;      /* file objects */
FRESULT SD_status;
char filename[30];
FRESULT configure_SD(void);
FRESULT save_measurement(struct MEASUREMENT *meas_inst);


// Menu cursors
int cur_X;
int cur_Y;
int cur_X_low;
int cur_X_high;
int cur_Y_low;
int cur_Y_high;



typedef struct
{
	enum STATE current;
	enum INPUT input;
	enum STATE next;
} STATE_NEXTSTATE;

typedef void (*State_function)(void);
typedef struct
{
	enum STATE current;
	//State_function Initial_function;
	State_function Function;

} STATE_FUNCTIONS;


STATE_NEXTSTATE state_nextstate[] = {
	//  Current State		Input           Next State
	{st_main_display,	input_1sec,		st_main_display},
	{st_main_display,	input_button1,	st_menu1},
	{st_main_display,	input_button2,	st_main_display},
	{st_main_display,	input_button3,	st_main_display},
	{st_main_display,	input_button4,	st_menu1},
	{st_main_display,	input_buttonE,	st_aim},
	{st_main_display,	input_powerdown,	st_powerdown},
	
	
	{st_aim,			input_button1,	st_aim_abort},
	{st_aim,			input_button2,	st_aim_abort},
	{st_aim,			input_button3,	st_aim_abort},
	{st_aim,			input_button4,	st_aim_abort},
	{st_aim,			input_buttonE,	st_measure},
	{st_aim,			input_powerdown,	st_powerdown},
	{st_aim,			input_1sec,	st_aim},
	{st_aim,			input_laser_timeout, st_aim_abort},
		
	{st_aim_abort,		input_state_complete,	st_main_display},
	
	{st_measure,		input_state_complete,	st_main_display},

	{st_powerdown,		input_button1,	st_powerup},
	{st_powerdown,		input_button2,	st_powerup},
	{st_powerdown,		input_button3,	st_powerup},
	{st_powerdown,		input_button4,	st_powerup},
	{st_powerdown,		input_wakeup,	st_powerup},

	{st_powerup,		input_state_complete,	st_main_display},
	
	{st_menu1,			input_button4,			st_main_display},
	{st_menu1,			input_buttonE,			st_main_display},	
	{st_menu1,			input_set_clock,		st_set_clock},
	{st_menu1,			input_set_bluetooth,	st_set_bluetooth},
	{st_menu1,			input_set_units,		st_set_options},
	{st_menu1,			input_cal_menu,			st_menu_cal},
	{st_menu1,			input_error_info,		st_error_info},
	{st_menu1,			input_menu_debug,		st_menu_debug},
	{st_menu1,			input_powerdown,		st_powerdown},
	
	{st_set_clock,		input_button4,			st_menu1},
	{st_set_clock,		input_buttonE,			st_main_display},	
	{st_set_clock,		input_state_complete,	st_main_display},
	{st_set_clock,		input_powerdown,		st_powerdown},
	
	{st_set_bluetooth,		input_button4,			st_main_display},
	{st_set_bluetooth,		input_buttonE,			st_main_display},
	{st_set_bluetooth,		input_state_complete,	st_main_display},
	//{st_set_bluetooth,		input_powerdown,		st_powerdown},		
	
	{st_set_options,		input_button4,			st_menu1},
	{st_set_options,		input_buttonE,			st_main_display},
	{st_set_options,		input_state_complete,	st_main_display},
	{st_set_options,		input_powerdown,		st_powerdown},
		
	{st_menu_cal,		input_button4,				st_main_display},
	{st_menu_cal,		input_buttonE,				st_main_display},
	{st_menu_cal,		input_dist_calibration,		st_dist_calibration},
	{st_menu_cal,		input_acc_comp_calibration,	st_acc_comp_calibration},
	{st_menu_cal,		input_state_complete,		st_main_display},
	{st_menu_cal,		input_disp_cal_report,		st_disp_cal_report},	
	{st_menu_cal,		input_loop_test,			st_loop_test},
	{st_menu_cal,		input_powerdown,			st_powerdown},
		
	{st_menu_debug,		input_button4,				st_menu1},
	{st_menu_debug,		input_buttonE,				st_main_display},
	{st_menu_debug,		input_debug_rawData,		st_debug_rawData},
	{st_menu_debug,		input_debug_backlight,		st_debug_backlight},	
	{st_menu_debug,		input_debug_charger,		st_debug_charger},	
	{st_menu_debug,		input_powerdown,			st_powerdown},
		
	{st_acc_comp_calibration,		input_button4,				st_aim_abort},
	{st_acc_comp_calibration,		input_state_complete,		st_process_calibration},	
		
	{st_dist_calibration,		input_button4,				st_aim_abort},
	{st_dist_calibration,		input_state_complete,		st_disp_cal_report},
		
	{st_disp_cal_report,		input_state_complete,		st_main_display},
	{st_disp_cal_report,		input_buttonE,				st_main_display},
	{st_disp_cal_report,		input_button4,				st_main_display},
		
	{st_loop_test,				input_button4,				st_aim_abort},
	{st_loop_test,				input_state_complete,		st_disp_loop_report},
		
	{st_disp_loop_report,		input_button4,			st_main_display},
	{st_disp_loop_report,		input_button3,			st_main_display},
	{st_disp_loop_report,		input_button2,			st_main_display},
	{st_disp_loop_report,		input_button1,			st_main_display},
	{st_disp_loop_report,		input_buttonE,			st_main_display},
	{st_disp_loop_report,		input_powerdown,		st_powerdown},
		
	{st_process_calibration,		input_state_complete,	st_disp_cal_report},
		
	{st_error_info,		input_button4,				st_main_display},
	{st_error_info,		input_buttonE,				st_main_display},	
	{st_error_info,		input_powerdown,			st_powerdown},
	{st_error_info,		input_state_complete,	st_main_display},
		
		
	{st_debug_rawData,					input_button1,			st_menu1},
	{st_debug_rawData,					input_button4,			st_menu1},
	{st_debug_rawData,					input_powerdown,		st_powerdown},
		
	{st_debug_charger,					input_button1,			st_menu1},
	{st_debug_charger,					input_button4,			st_menu1},
	{st_debug_charger,					input_powerdown,		st_powerdown},
		
	{st_debug_backlight,					input_buttonE,			st_menu1},
	{st_debug_backlight,					input_powerdown,		st_powerdown},
				
	
	{0,0,0}
};




STATE_FUNCTIONS state_functions[]={
	//  Current State	Function
	{st_main_display,	fn_main_display},
	{st_aim,			fn_aim},
	{st_aim_abort,		fn_aim_abort},
	{st_measure,		fn_measure},
	{st_powerdown,		fn_powerdown},
	{st_powerup,		fn_powerup},
	{st_set_clock,		fn_set_clock	},
	{st_set_bluetooth,	fn_set_bluetooth},
	{st_set_options,	fn_set_options	},
	{st_menu1,			fn_menu1	},
	{st_menu_cal,		fn_menu_cal},
	{st_menu_debug,		fn_menu_debug},
	{st_acc_comp_calibration,		fn_acc_comp_calibration},
	{st_dist_calibration,			fn_dist_calibration},
	{st_process_calibration,		fn_process_calibration},
	{st_disp_cal_report,			fn_disp_cal_report},	
	{st_loop_test,					fn_loop_test},
	{st_disp_loop_report,			fn_disp_loop_report},	
	{st_debug_rawData,				fn_debug_rawData},
	{st_debug_backlight,			fn_debug_backlight},
	{st_debug_charger,				fn_debug_charger},
	{st_error_info,					fn_error_info},

};





int main (void)
{
	uint8_t i; // Iteration counter
	config_pins_powerup();
	system_init();
	delay_init();
	delay_ms(500);	
	wdt_enable();
	setup_spi();
	configure_i2c_master();	
	glcd_init();
	configure_extint_channel();
	configure_extint_callbacks();	
	setup_batt();
	setup_accel(&slave_acc1);
	setup_accel(&slave_acc2);	
	setup_mag(&slave_mag1);
	setup_mag(&slave_mag2);
	configure_usart();
	configure_usart_callbacks();	
	ext_osc_onoff(true);
	setup_XOSC32k();
	clock_32k_source(clock_ext);
	configure_timers(st_powerup);
	system_interrupt_enable_global();	
	sleepmgr_init();
	load_user_settings();
	load_calibration();
	setup_charger();
	backlightOn();
	
	configure_SD();	
	rangefinder_on_off(false);
	ioport_reset_pin_mode(BLE_ota);//  Needed to reset pin mode; set in some previous initialization
	ioport_set_pin_dir(BLE_ota, IOPORT_DIR_OUTPUT);//  Needed to reset pin mode; set in some previous initialization
	ioport_set_pin_level(BLE_ota, false);//  Needed to reset pin mode; set in some previous initialization
	
	current_state = st_main_display;
	current_input = input_1sec;
	
	while(1){
		if (current_input==input_none){
			clock_16M_source(clock_low);//  Set clock low to conserve power
			while(current_input == input_none);//hold here until an input
			clock_16M_source(clock_high);// Move clock back to high speed
		}
		wdt_reset_count();//
		//  Determine if idle powerdown will be performed
		idle_timeout();//Will produce input=idle_timeout if idle for too long
		
		//  Find state 
		//  State a function of current input		
		state_change = false;
		for (i=0; i<(sizeof(state_nextstate)/sizeof(STATE_NEXTSTATE));i++){
			if((current_state==state_nextstate[i].current) && (current_input==state_nextstate[i].input)){
				if(current_state!=state_nextstate[i].next){
					current_state = state_nextstate[i].next;
					state_change = true;
				}
				break;
			}
		}
		last_input = current_input;
		current_input = input_none;
		
		//  Find and run function for current state
		for(i=0;i<(sizeof(state_functions)/sizeof(STATE_FUNCTIONS));i++){
			if(current_state==state_functions[i].current){
				state_functions[i].Function();				
				break;
			}
		}

		
	}//End of main program while loop
}//end of main

void fn_debug_charger(void){
	uint8_t addressList[] = {
		0x00, 
		0x01,
		0x06,
		0x07,
		0x0B,
		0x0C,
		
		};
	uint8_t i;
	uint8_t data;
	char binStr[10]; 
	
	//  Set initial conditions
	if (state_change) {
		cur_Y = 2;
		cur_Y_low = 2;
		cur_Y_high = 5;
	}
		
	// Display
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"Charger Debug:");
	glcd_tiny_draw_string(0,0,display_str);
	
	for (i=0;i<sizeof(addressList);i++){
		data = getChargerRegister(addressList[i]);
		bin2str(data, binStr);
		sprintf(display_str, "Add:%02x = %s", 
			addressList[i], binStr);
		glcd_tiny_draw_string(10, i+1, display_str);
		
	}
	
	
	
	
	
	
	glcd_write();
	
}


void fn_debug_rawData(void){
	
	struct MEASUREMENT meas_debug;
	quick_measurement( &meas_debug);	
		

	glcd_clear_buffer();	
	sprintf(display_str, "Acc 1     Acc 2  Done");
	glcd_tiny_draw_string(0,0,display_str);
	
	sprintf(display_str, "X: %+0.3f %+0.3f", meas_debug.a1xyz[0], meas_debug.a2xyz[0]);
	glcd_tiny_draw_string(0,1,display_str);
	sprintf(display_str, "Y: %+0.3f %+0.3f", meas_debug.a1xyz[1], meas_debug.a2xyz[1]);
	glcd_tiny_draw_string(0,2,display_str);
	sprintf(display_str, "Z: %+0.3f %+0.3f", meas_debug.a1xyz[2], meas_debug.a2xyz[2]);
	glcd_tiny_draw_string(0,3,display_str);
	
	sprintf(display_str, "Comp 1    Comp 2  ");
	glcd_tiny_draw_string(0,4,display_str);
	
	sprintf(display_str, "X: %+0.3f %+0.3f", meas_debug.c1xyz[0], meas_debug.c2xyz[0]);
	glcd_tiny_draw_string(0,5,display_str);
	sprintf(display_str, "Y: %+0.3f %+0.3f", meas_debug.c1xyz[1], meas_debug.c2xyz[1]);
	glcd_tiny_draw_string(0,6,display_str);
	sprintf(display_str, "Z: %+0.3f %+0.3f Exit", meas_debug.c1xyz[2], meas_debug.c2xyz[2]);
	glcd_tiny_draw_string(0,7,display_str);
	glcd_write();	
}

void fn_process_calibration(void){
	uint8_t i, j;
	
	// Disable Watchdog Timer
	wdt_disable();
	
	// Start with empty cal structures
	cal_init_struct(&a1_calst);
	cal_init_struct(&a2_calst);
	cal_init_struct(&c1_calst);
	cal_init_struct(&c2_calst);	
	//  Gain and Offset Calibration
	glcd_clear_buffer();
	sprintf(display_str, "Processing Data...");
	glcd_tiny_draw_string(0,0,display_str);
	sprintf(display_str, "Calibration:");
	glcd_tiny_draw_string(0,2,display_str);
	sprintf(display_str, "Gain and Offset Cal");
	glcd_tiny_draw_string(0,3,display_str);
	glcd_write();	
	
	//  Perform gain and offset calibration per ellipsoid fit
	sprintf(display_str, "Accelerometer 1      ");
	glcd_tiny_draw_string(0,4,display_str);
	glcd_write();	
	cal_gain_off(a1raw, &a1_calst);// Gain and Offset Calibration, Accelerometer 1
	sprintf(display_str, "Accelerometer 2      ");
	glcd_tiny_draw_string(0,4,display_str);
	glcd_write();
	cal_gain_off(a2raw, &a2_calst);// Gain and Offset Calibration, Accelerometer 2
	sprintf(display_str, "Compass 1            ");
	glcd_tiny_draw_string(0,4,display_str);
	glcd_write();
	cal_gain_off(c1raw, &c1_calst);// Gain and Offset Calibration, Compass 1
	sprintf(display_str, "Compass 2            ");
	glcd_tiny_draw_string(0,4,display_str);
	glcd_write();
	cal_gain_off(c2raw, &c2_calst);	// Gain and Offset Calibration, Compass 2
	//  Apply gain and offset calibration
	for (i=0;i<n_points;i++){
		cal_apply_cal(a1raw[i], a1cal[i], &a1_calst);
		cal_apply_cal(a2raw[i], a2cal[i], &a2_calst);
		cal_apply_cal(c1raw[i], c1cal[i], &c1_calst);
		cal_apply_cal(c2raw[i], c2cal[i], &c2_calst);
	}	
	
	//  Perform Axis Misalignment Calibration
	glcd_clear_buffer();
	sprintf(display_str, "Processing Data...");
	glcd_tiny_draw_string(0,0,display_str);
	sprintf(display_str, "Calibration:");
	glcd_tiny_draw_string(0,2,display_str);
	sprintf(display_str, "Axis Misalignments:");
	sprintf(display_str, "Accelerometer 1      ");
	glcd_tiny_draw_string(0,4,display_str);	glcd_write();
	cal_axis_misalignments(a1cal, &a1_calst); // Sensor axis misalignments, Accelerometer 1
	sprintf(display_str, "Accelerometer 2      ");
	glcd_tiny_draw_string(0,4,display_str);	glcd_write();
	cal_axis_misalignments(a2cal, &a2_calst);// Sensor axis misalignments, Accelerometer 2
	sprintf(display_str, "Compass 1            ");
	glcd_tiny_draw_string(0,4,display_str);	glcd_write();
	cal_axis_misalignments(c1cal, &c1_calst);// Sensor axis misalignments, Compass 1
	sprintf(display_str, "Compass 2            ");
	glcd_tiny_draw_string(0,4,display_str);	glcd_write();
	cal_axis_misalignments(c2cal, &c2_calst);// Sensor axis misalignments, Compass 2
	//  Apply gain and offset calibration
	for (i=0;i<n_points;i++){
		cal_apply_cal(a1raw[i], a1cal[i], &a1_calst);
		cal_apply_cal(a2raw[i], a2cal[i], &a2_calst);
		cal_apply_cal(c1raw[i], c1cal[i], &c1_calst);
		cal_apply_cal(c2raw[i], c2cal[i], &c2_calst);
	}
	

	//  Sensor package to laser axis alignment about Y&Z axis
	glcd_clear_buffer();
	sprintf(display_str, "Processing Data...");
	glcd_tiny_draw_string(0,0,display_str);
	sprintf(display_str, "Calibration:");
	glcd_tiny_draw_string(0,2,display_str);
	sprintf(display_str, "Misalignment Cal, YZ");
	glcd_tiny_draw_string(0,3,display_str);
	glcd_write();		
	for (j=0;j<3;j++){
		sprintf(display_str, "Iteration: %d of 3    ", j+1);
		glcd_tiny_draw_string(0,4,display_str);
		glcd_write();
		cal_angleYZ(a1cal, &a1_calst);//  Sensor Package to laser Y&Z axis alignment, Accelerometer 1
		cal_angleYZ(a2cal, &a2_calst);//  Sensor Package to laser Y&Z axis alignment, Accelerometer 2
		cal_angleYZ(c1cal, &c1_calst);//  Sensor Package to laser Y&Z axis alignment, Compass 1
		cal_angleYZ(c2cal, &c2_calst);//  Sensor Package to laser Y&Z axis alignment, Compass 2
	}	
	//  Apply gain, offset, and angle calibration
	for (i=0;i<n_points;i++){
		cal_apply_cal(a1raw[i], a1cal[i], &a1_calst);
		cal_apply_cal(a2raw[i], a2cal[i], &a2_calst);
		cal_apply_cal(c1raw[i], c1cal[i], &c1_calst);
		cal_apply_cal(c2raw[i], c2cal[i], &c2_calst);
	}	
	
	
	//  Perform X angle sensor-package to laser misalignment calibration
	//  Only calibrate the 2nd sensor
	glcd_clear_buffer();
	sprintf(display_str, "Processing Data...");
	glcd_tiny_draw_string(0,0,display_str);
	sprintf(display_str, "Calibration:");
	glcd_tiny_draw_string(0,2,display_str);
	sprintf(display_str, "Misalignment Cal, X");
	glcd_tiny_draw_string(0,3,display_str);
	glcd_write();
	cal_angleX(a1cal, a2cal, &a2_calst);//  Sensor package to laser X-axis alignment, Accelerometer 2
	cal_angleX(c1cal, c2cal, &c2_calst);//  Sensor package to laser X-axis alignment, Compass 2
		//  Apply gain, offset, and angle calibration
	for (i=0;i<n_points;i++){
		cal_apply_cal(a1raw[i], a1cal[i], &a1_calst);
		cal_apply_cal(a2raw[i], a2cal[i], &a2_calst);
		cal_apply_cal(c1raw[i], c1cal[i], &c1_calst);
		cal_apply_cal(c2raw[i], c2cal[i], &c2_calst);
	}
	
	
	//  Evaluate performance of calibration
	glcd_clear_buffer();
	sprintf(display_str, "Processing Data...");
	glcd_tiny_draw_string(0,0,display_str);
	sprintf(display_str, "Evaluating Results   ");
	glcd_tiny_draw_string(0,2,display_str);
	glcd_write();
	cal_evaluate();	
	//  Write report
	sprintf(display_str, "Writing Report       ");
	glcd_tiny_draw_string(0,2,display_str);
	glcd_write();	
	cal_write_report();
	//  Save data to EEPROM
	save_calibration();
	// Calibration Complete
	sprintf(display_str, "Calibration Complete!");
	glcd_tiny_draw_string(0,2,display_str);
	glcd_write();
	delay_s(3);
	
	// Re-Enable Watchdog Timer
	wdt_enable();
	
	current_input = input_state_complete;
}


void fn_acc_comp_calibration(void){
	struct MEASUREMENT temp_meas;
	
	//uint32_t timer_count;
	uint8_t disp_groups;
	// disp_groups is actual number of groups complete including last one not processed
	if (buf_points>= group_size){disp_groups = n_groups+1;}
	else{disp_groups = n_groups;}
	
	
	if (state_change){
		cal_disp_message();
		//  Set up initial settings
		n_groups = 0;
		n_points = 0;
		ind_stack = 0;
		ind_buf = 0;
		buf_points = 0;
		laser_triggered =  false;
		last_input = input_none;
		
	}
	
	// Button Handler
	switch(last_input){
		case input_button1:
			//  Calibration Done button
			if (disp_groups>=min_groups){//  Requires min_groups to complete
				//  Turn off Rangefinder
				rangefinder_on_off(false);
				//  Add latest datapoint
				cal_add_datapoint(&temp_meas, true);
				current_input = input_state_complete;
			}
			break;
		case input_buttonE:
			//  Set laser and then take measurement
			if (!laser_triggered){
				// Turn on rangefinder module and laser
				rangefinder_on_off(true);
				laser_on_off(true);
			}else{
				//  Take measurement and process data
				full_measurement(&temp_meas, false);
				//  Turn laser module off
				rangefinder_on_off(false);
				cal_add_datapoint(&temp_meas, false);
			}
			break;
		case input_button4:
			// Handled by state machine
			//  Exit from routine
			break;
	}
	last_input = input_none;	
	
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	
	// Display Header
	sprintf(display_str, "Calibration Mode");
	glcd_tiny_draw_string(0,0,display_str);
	
	// Display Current Data
	sprintf(display_str, "Current Group: %d", (n_groups+1));
	glcd_tiny_draw_string(0,2,display_str);
	sprintf(display_str, "Status: %d of 4", buf_points );
	glcd_tiny_draw_string(0,3,display_str);
	sprintf(display_str, "Complete Groups: %d", disp_groups );
	glcd_tiny_draw_string(0,5,display_str);
	
	
	// Display Soft Keys
	//if (n_groups >= 2){
	if (disp_groups >= min_groups){
		sprintf(display_str, "Done");
		glcd_tiny_draw_string(100,0,display_str);
	}
	sprintf(display_str, "Abort");
	glcd_tiny_draw_string(97,7,display_str);
	

	
	glcd_write();
	
	
}

void fn_loop_test(void){
	struct MEASUREMENT temp_meas;
	//uint32_t timer_count;
	uint8_t i, k;
	
	if (state_change){
		cal_disp_message();
		//  Set up initial settings
		n_points = 0;
		loop_distance = 0;
		loop_horizontal = 0;
		loop_vertical = 0;
		loop_azimuth = 0;
		for (i=0;i<nbuf;i++){
			azimuth[i]=0;
			inclination[i]=0;
			roll[i] = 0;
		}
		laser_triggered =  false;
		last_input = input_none;
	}
	
	// Button Handler
	switch(last_input){
		case input_button1:
			//  Calibration Done button
			ioport_set_pin_level(laser_reset, false);
			current_input = input_state_complete;
			
			break;
		case input_buttonE:
			//  Set laser and then take measurement
			if (!laser_triggered){
				// Turn on rangefinder module and laser
				rangefinder_on_off(true);
				laser_on_off(true);
			}else{
				//  Take measurement and process data
				full_measurement(&temp_meas, true);
				//  Turn laser module off
				rangefinder_on_off(false);
				//  Process datapoint
				cal_loop_test(&temp_meas);
			}
			break;
		case input_button4:
		// Handled by state machine
		//  Exit from routine
		break;
	}
	last_input = input_none;
	
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	
	// Display Header
	sprintf(display_str, "Loop Test:");
	glcd_tiny_draw_string(0,0,display_str);
	
	sprintf(display_str, "Segments: %d", n_points);
	glcd_tiny_draw_string(0,2,display_str);
	sprintf(display_str, "Loop length: %.1f", loop_distance);
	if (options.current_unit_dist == feet){
		strcat(display_str,"ft");
	}else{
		strcat(display_str,"m");
	} 
	glcd_tiny_draw_string(0,3,display_str);
	
	sprintf(display_str, "Displacement from P1:");
	glcd_tiny_draw_string(0,4,display_str);
	sprintf(display_str, "Horizontal: %.1f", loop_horizontal);
	if (options.current_unit_dist == feet){
		strcat(display_str,"ft");
	}else{
		strcat(display_str,"m");
	}
	glcd_tiny_draw_string(0,5,display_str);
	sprintf(display_str, "Vertical: %.1f", loop_vertical);
	if (options.current_unit_dist == feet){
		strcat(display_str,"ft");
		}else{
		strcat(display_str,"m");
	}
	glcd_tiny_draw_string(0,6,display_str);
	sprintf(display_str, "Azimuth: %.1f", loop_azimuth);
	glcd_tiny_draw_string(0,7,display_str);
	
	
	// Display Soft Keys
	sprintf(display_str, "Done");
	glcd_tiny_draw_string(100,0,display_str);
	sprintf(display_str, "Abort");
	glcd_tiny_draw_string(97,7,display_str);
	
	glcd_write();
}

void fn_disp_loop_report(void){
		char unit_str[4];
		//float total_error;
		
		if (options.current_unit_dist == feet){
			strcpy(unit_str,"ft");
		}else{
			strcpy(unit_str,"m");
		}
		
		
		glcd_clear_buffer();
		sprintf(display_str, "Loop Test Report:");
		glcd_tiny_draw_string(0,0,display_str);
		
		sprintf(display_str,"Segments: %d",n_points);
		glcd_tiny_draw_string(0,1,display_str);
		
		sprintf(display_str,"Total Length: %.1f %s", loop_distance, unit_str);
		glcd_tiny_draw_string(0,2,display_str);
		
		sprintf(display_str,"Horz Err: %.3f %s", loop_horizontal, unit_str);
		glcd_tiny_draw_string(0,3,display_str);
		
		sprintf(display_str,"Vert Err: %.3f %s", loop_vertical, unit_str);
		glcd_tiny_draw_string(0,4,display_str);
		
		sprintf(display_str,"  Azim Err: %.1f deg", loop_azimuth);
		glcd_tiny_draw_string(0,5,display_str);
		
		sprintf(display_str,"Loop Err: %.3f%% ", loop_error);
		glcd_tiny_draw_string(0,5,display_str);
		
		
		
		glcd_write();
	
}


void fn_dist_calibration(void){
	struct MEASUREMENT temp_meas;
	
	//uint32_t timer_count;
	uint8_t k;
	
	if (state_change){
		cal_disp_message();
		//  Set up initial settings
		buf_points = 0;
		ind_buf = 0;// Circular buffer
		for (k=0;k<shot_size;k++){
			dist_raw_buf[k] = 0;
		}
		laser_triggered =  false;
		last_input = input_none;
	}
	
	// Button Handler
	switch(last_input){
		case input_button1:
			//  Calibration Done button
			if (buf_points >= shot_size){
				//  Turn off rangefinder
				rangefinder_on_off(false);
				// Process Calibration data
				cal_dist_process();
				//  Save data to EEPROM
				save_calibration();
				current_input = input_state_complete;
			}
			break;
		case input_buttonE:
			//  Set laser and then take measurement
			if (!laser_triggered){
				// Turn on rangefinder module and laser
				rangefinder_on_off(true);
				laser_on_off(true);
			}else{
				//  Take measurement and process data
				full_measurement(&temp_meas, false);
				//  Turn laser module off
				rangefinder_on_off(true);
			}
			break;
		case input_button4:
			// Handled by state machine
			//  Exit from routine
			break;
	}
	last_input = input_none;
	
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	
	// Display Header
	sprintf(display_str, "Calibration Mode");
	glcd_tiny_draw_string(0,0,display_str);
	if (options.current_unit_dist == feet){
		sprintf(display_str, " Target %.1f feet",dist_cal_setpoint_ft);
	}else{
		sprintf(display_str, " Target %.1f meters.",dist_cal_setpoint_mt);
	}
	glcd_tiny_draw_string(0,1,display_str);
	
	// Display Current Data
	for (k=0;k<4;k++){
		sprintf(display_str,"M%d: %2.4f ",k,dist_disp_buf[k]);
		if (options.current_unit_dist == feet){
			strcat(display_str,"ft");
		}else{
			strcat(display_str,"m");
		}
		glcd_tiny_draw_string(0,k+2,display_str);
	}
	sprintf(display_str, "Offset: %f", temp_dist_offset);
	glcd_tiny_draw_string(0,6,display_str);

	// Display Soft Keys
	//if (n_groups >= 2){
	if (buf_points >= shot_size){
		sprintf(display_str, "Calibration      Done");
		glcd_tiny_draw_string(0,0,display_str);
	}
	sprintf(display_str, "Abort");
	glcd_tiny_draw_string(97,7,display_str);
		
	glcd_write();
}


void cal_disp_message(void){
	wdt_disable();
	if (current_state==st_acc_comp_calibration){
		glcd_tiny_set_font(Font5x7,5,7,32,127);
		glcd_clear_buffer();
	
		sprintf(display_str, "Azm/Inc Calibration:");
		glcd_tiny_draw_string(0,0,display_str);
		sprintf(display_str, "Take Uni-Directional Groups of 4 Shots    while rotating       instrument. Only last4 shots of each groupwill be saved");
		glcd_tiny_draw_string(0,1,display_str);		
	}else if(current_state == st_dist_calibration){
		glcd_tiny_set_font(Font5x7,5,7,32,127);
		glcd_clear_buffer();
		sprintf(display_str, "Distance Calibration:");
		glcd_tiny_draw_string(0,0,display_str);
		sprintf(display_str, "Place a target at");
		glcd_tiny_draw_string(0,1,display_str);
		if (options.current_unit_dist == feet){
			sprintf(display_str, "  %.1f feet.",dist_cal_setpoint_ft);
		}else{
			sprintf(display_str, "  %.1f meters.",dist_cal_setpoint_mt);
		}
		glcd_tiny_draw_string(0,2,display_str);
		sprintf(display_str, "Take min. 4 shots in");
		glcd_tiny_draw_string(0,3,display_str);
		sprintf(display_str, "Multiple Orientations");
		glcd_tiny_draw_string(0,4,display_str);
		sprintf(display_str, "Only last 4 used.");
		glcd_tiny_draw_string(0,5,display_str);
		
		
		
	}else if(current_state == st_loop_test){
		glcd_tiny_set_font(Font5x7,5,7,32,127);
		glcd_clear_buffer();
		sprintf(display_str, "Loop Test:");
		glcd_tiny_draw_string(0,0,display_str);
		sprintf(display_str, "Take a series of");
		glcd_tiny_draw_string(0,1,display_str);
		sprintf(display_str, "measurements ending");
		glcd_tiny_draw_string(0,2,display_str);
		sprintf(display_str, "back at the first");
		glcd_tiny_draw_string(0,3,display_str);
		sprintf(display_str, "point.  Press 'Done'");
		glcd_tiny_draw_string(0,4,display_str);
		sprintf(display_str, "when complete.");
		glcd_tiny_draw_string(0,5,display_str);
	}
	sprintf(display_str, "Press any button...");
	glcd_tiny_draw_string(10,7,display_str);
	glcd_write();
	while((current_input == input_none) || (current_input == input_1sec));//hold here until an input
	current_input = input_none;
	wdt_enable();
}

void  fn_disp_cal_report(void){
	#define maxPages 3
	static uint8_t pageView;
	if (state_change){
		pageView = 1;
	}
	
	// Button Handler
	switch(last_input){
		case input_button2:
			if (pageView>1){pageView--;}
			break;	
		case input_button3:
			if (pageView<maxPages){pageView++;}
			break;
			//  ButtonE and Button4 exit, handled by state machine
	}
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	sprintf(display_str, "Calibration Report:");
	glcd_tiny_draw_string(0,0,display_str);
	
	switch(pageView){
		///////////////////////// AZM and INC Report
		case 1:
			//// Page 1			
			sprintf(display_str, "Azimuth & Inclination");
			glcd_tiny_draw_string(0,1,display_str);
			sprintf(display_str,"20%02x.%02x.%02x@%02x:%02x:%02x",
				cal_report_azm_inc.time_struct.year, cal_report_azm_inc.time_struct.month, cal_report_azm_inc.time_struct.date,
				cal_report_azm_inc.time_struct.hours, cal_report_azm_inc.time_struct.minutes, cal_report_azm_inc.time_struct.seconds);
			glcd_tiny_draw_string(0,2,display_str);
			sprintf(display_str,"4-Point Groups: %d", cal_report_azm_inc.groups);
			glcd_tiny_draw_string(0,3,display_str);
			sprintf(display_str,"Azm Stdev: %.3f", cal_report_azm_inc.azm_angle_err);
			glcd_tiny_draw_string(0,5,display_str);
			glcd_draw_circle(98, 41, 1, BLACK);// Draw degree symbol
			sprintf(display_str,"Inc Stdev: %.3f", cal_report_azm_inc.inc_angle_err);
			glcd_tiny_draw_string(0,6,display_str);
			glcd_draw_circle(98, 49, 1, BLACK);// Draw degree symbol
			break;
		case 2:
			//// Page 2
			sprintf(display_str, "Azimuth & Inclination");
			glcd_tiny_draw_string(0,1,display_str);
			// Sensor Disparity
			sprintf(display_str,"Sensor Delta X,Y,Z%%");
			glcd_tiny_draw_string(0,2,display_str);
			sprintf(display_str,"A:%.3f,%.3f,%.3f",
				cal_report_azm_inc.disp_stdev_acc[0]*100, cal_report_azm_inc.disp_stdev_acc[1]*100, cal_report_azm_inc.disp_stdev_acc[2]*100);
			glcd_tiny_draw_string(0,3,display_str);
			sprintf(display_str,"C:%.3f,%.3f,%.3f",
				cal_report_azm_inc.disp_stdev_comp[0]*100, cal_report_azm_inc.disp_stdev_comp[1]*100, cal_report_azm_inc.disp_stdev_comp[2]*100);
			glcd_tiny_draw_string(0,4,display_str);
			//  Magnitude Error			
			sprintf(display_str,"Magnitude Error %%");
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str,"A1:%.3f A2:%.3f", cal_report_azm_inc.mag_stdev_a1*100, cal_report_azm_inc.mag_stdev_a2*100);
			glcd_tiny_draw_string(0,6,display_str);
			sprintf(display_str,"C1:%.3f C2:%.3f", cal_report_azm_inc.mag_stdev_c1*100, cal_report_azm_inc.mag_stdev_c2*100);
			glcd_tiny_draw_string(0,7,display_str);
		break;
		//////////////////////// Distance Report
		case 3:
			sprintf(display_str, "Distance");
			glcd_tiny_draw_string(0,1,display_str);
			sprintf(display_str,"20%02x.%02x.%02x@%02x:%02x:%02x",
				cal_report_dist.time_struct.year, cal_report_dist.time_struct.month, cal_report_dist.time_struct.date,
				cal_report_dist.time_struct.hours, cal_report_dist.time_struct.minutes, cal_report_dist.time_struct.seconds);
			glcd_tiny_draw_string(0,2,display_str);
			sprintf(display_str,"Rangefinder Offset:");
			glcd_tiny_draw_string(0,4,display_str);
			sprintf(display_str,"  %.4f meters", dist_calst.dist_offset);
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str,"  %.4f feet", dist_calst.dist_offset*mt2ft);
			glcd_tiny_draw_string(0,6,display_str);
			
		break;
	}
	
	// Display soft keys
	if (pageView>1){draw_arrows(2);}//  Draw up arrow at button 2
	if (pageView<maxPages){draw_arrows(3);}//  Draw up arrow at button 3
	sprintf(display_str, "Exit");
	glcd_tiny_draw_string(102,7,display_str);
	glcd_write();
}



FRESULT save_measurement(struct MEASUREMENT *meas_inst){
	uint32_t btw;
	//uint32_t bw;
	//uint32_t *pbw;
	UINT bw;
	UINT *pbw;
	char write_string_temp[511];
	char write_string_full[511];
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	
	pbw = &bw;
	
	// Get current time
	//get_time(); // Already performed during measurement
	
	
	//  Set up SD card
	config_spi(SD_card);		
	spi_select_slave(&spi_main, &slave_SD, true);	

	diskio_status = disk_status(0);
	
	if(diskio_status){
		//Possibly card not initialized
		configure_SD();
		diskio_status = disk_status(0);
		if(diskio_status){
			fdebug1 = FR_NOT_READY;
			SD_status = fdebug1;
			config_spi(LCD);
			return fdebug1;	
		}
		
	}
	
	
	//  Format data for text data file
	sprintf(filename, "20%02x%02x%02x_datafile.csv", current_time.year, current_time.month, current_time.date);
		
	fdebug1 = f_open(&file1, filename, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	
	if (fdebug1 == FR_NO_FILE){
		// File does not exist, create new file with header
		fdebug2 = f_open(&file1, filename, FA_CREATE_NEW | FA_READ | FA_WRITE);
		
		if(fdebug2!=FR_OK){
			SD_status = fdebug2;
			config_spi(LCD);
			return fdebug2;
		}

		if (options.current_unit_dist == feet){	
			sprintf(write_string_temp, "Time-Stamp, Index, Distance (meters), Azimuth (degrees), Inclination (degrees), Temperature (Celsius),  Error Log\r\n");
			fdebug2 = f_write(&file1, write_string_temp, strlen(write_string_temp), pbw);
		}else{
			sprintf(write_string_temp, "Time-Stamp, Index, Distance (feet), Azimuth (degrees), Inclination (degrees), Temperature (Fahrenheit), Error Log\r\n");
			fdebug2 = f_write(&file1, write_string_temp, strlen(write_string_temp), pbw);
		}
		
		
	}else if(fdebug1 != FR_OK){
		SD_status = fdebug1;
		config_spi(LCD);
		return fdebug1;		
	}
	
	// Format string for timestamps	
	sprintf(write_string_full,"20%02x.%02x.%02x@%02x:%02x:%02x,",
		current_time.year, current_time.month, current_time.date,
		current_time.hours, current_time.minutes, current_time.seconds);
	// Format string for data
	sprintf(write_string_temp," %d, %.3f, %.3f, %.3f,",
		meas_inst->index_ref, meas_inst->distance, meas_inst->azimuth, meas_inst->inclination);
	strcat(write_string_full, write_string_temp);
	//  Format string for temperature
	if (options.current_unit_dist == feet){
		sprintf(write_string_temp," %.3f,", current_time.temperatureF);
	}else{
		sprintf(write_string_temp," %.3f,", current_time.temperatureC);
	} 
	strcat(write_string_full, write_string_temp);
	//Format data for error_log
	//sprintf(write_string_temp,"Laser: %02x",meas_inst->laser_error_code);
	strcat(write_string_full, write_string_temp);
	// Enter line return
	strcat(write_string_full, "\r\n");
	
	// Append data file
	fdebug2 = f_lseek(&file1, f_size(&file1));
	fdebug3 = f_write(&file1, write_string_full, strlen(write_string_full), pbw);
	f_close(&file1);
	
	
	spi_select_slave(&spi_main, &slave_SD, false);
	config_spi(LCD);

	return fdebug3;
}


FRESULT configure_SD(void){
	FRESULT fdebug1;
	config_spi(SD_card);
	spi_select_slave(&spi_main, &slave_SD, true);
	spi_clear();
	sd_mmc_init();

	fdebug1 = f_mount(&FatFS, "", 1);
	spi_select_slave(&spi_main, &slave_SD, false);
	config_spi(LCD);
	
	return fdebug1;
}


void fn_measure(void){
	// increment data buffer index
	data_buf_ind = data_buf_ind+1;
	if (data_buf_ind >= buf_length){data_buf_ind = 0;}
	// Increment reference counter
	data_ref = data_ref+1;
	if (data_ref>= 999){data_ref = 1;}
	data_buf[data_buf_ind].index_ref = data_ref;
	//  Take measurement	
	full_measurement(&data_buf[data_buf_ind], true);
	//  Save data to SD card
	save_measurement(&data_buf[data_buf_ind]);
	//  Turn laser module off
	rangefinder_on_off(false);
	//  Complete measurement function
	current_input = input_state_complete;
}

void fn_aim(void){
	//uint32_t timer_count;
	uint16_t temp_index;
	
	if (state_change) {
		rangefinder_on_off(true);
		laser_on_off(true);
	}
	
	
	temp_index = data_buf_ind+1;
	if(temp_index>=buf_length){temp_index = 0;}
	
	
	quick_measurement(&data_buf[temp_index]);
	
	
	print_data_screen();

}


void fn_error_info(void){
	uint8_t i, nErr;
	static uint8_t shot_list[buf_length+1];
	static uint8_t shot_list_ind;
	static uint8_t nshots;
	uint8_t temp_buf_ind;

	
	if (state_change){ // Perform first time entering function
		// Build list of indexes of bad shots
		shot_list_ind = 0;
		nshots = 0;
		temp_buf_ind = data_buf_ind;
		for (i=0;i<buf_length;i++){
			if (data_buf[temp_buf_ind].num_errors>0){
				//  Add shot to list
				shot_list[shot_list_ind] = temp_buf_ind;
				shot_list_ind++;
				nshots++;
			}
			
			if (temp_buf_ind == 0){ temp_buf_ind = buf_length-1;}//  Buffer wrap-around
			else {temp_buf_ind--;}
		}
		
		shot_list_ind = 0;

		last_input = input_none;
	}
	// Button Handler
	switch(last_input){
		case input_button2:
		if(shot_list_ind>0){
			shot_list_ind--;
		}
		break;
		case input_button3:
		if(shot_list_ind<nshots){
			shot_list_ind++;
		}
		break;
		case input_button4:
		//  Exit back to main screen, handled in state machine
		break;
	}
	
	
	// Display
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"Error Information:");
	glcd_tiny_draw_string(0,0,display_str);
	// Display soft keys
	sprintf(display_str, "Back");
	glcd_tiny_draw_string(90,7,display_str);
	if(shot_list_ind>0){draw_arrows(2);}
	if(shot_list_ind<nshots){draw_arrows(3);}
	
	if(nshots<= shot_list_ind){//  display null message
		sprintf(display_str,"No Additional Errors");
		glcd_tiny_draw_string(8,1,display_str);
		sprintf(display_str,"to Report in Last");
		glcd_tiny_draw_string(8,2,display_str);
		sprintf(display_str,"%d Measurements", buf_length);
		glcd_tiny_draw_string(8,3,display_str);
		}else{
		temp_buf_ind = shot_list[shot_list_ind];
		sprintf(display_str,"Measurement %d", data_buf[temp_buf_ind].index_ref);
		glcd_tiny_draw_string(0,1,display_str);
		for (i=0;i<min(5, data_buf[temp_buf_ind].num_errors); i++){
			gen_err_message(display_str, &data_buf[temp_buf_ind], i);
			glcd_tiny_draw_string(0,i+2,display_str);
		}
		
	}
	
	glcd_write();
}


void fn_menu1(void){
	
	if (state_change){
		cur_Y=1;
		cur_Y_low=1;
		cur_Y_high=6;
		last_input = input_none;
	}
	
	
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	
	//  Button Handler
	switch(last_input){
		case input_button2:
			if(cur_Y > cur_Y_low){--cur_Y; }
			break;
		case input_button3:
			if(cur_Y < cur_Y_high){++cur_Y; }
			break;
		case input_button1:
			if(cur_Y == 1){ // Options
				current_input = input_set_units;
			} else if (cur_Y==2){// Error Info
				current_input = input_error_info;
			} else if (cur_Y==3){ // Calibration
				current_input = input_cal_menu;
			} else if (cur_Y==4){ // Set Clock
				current_input = input_set_clock;
			} else if (cur_Y==5){ //  Bluetooth
				current_input = input_set_bluetooth;
			} else if (cur_Y==6){ // Debug
				current_input = input_menu_debug;
			}
			break;
	}
	
	//print soft key text
	draw_arrows(2);//  Draw up arrow at button 2
	draw_arrows(3);//  Draw down arrow at button 3
	sprintf(display_str, "Menu:");
	glcd_tiny_draw_string(0,0,display_str);
	sprintf(display_str, "Enter");
	glcd_tiny_draw_string(96,0,display_str);
	sprintf(display_str, "Back");
	glcd_tiny_draw_string(100,7,display_str);
	
	sprintf(display_str, "Options");
	glcd_tiny_draw_string(10,1,display_str);
	
	sprintf(display_str, "Error Info");
	glcd_tiny_draw_string(10,2,display_str);
	
	sprintf(display_str, "Calibration");
	glcd_tiny_draw_string(10,3,display_str);
	
	sprintf(display_str, "Set Clock");
	glcd_tiny_draw_string(10,4,display_str);
	
	sprintf(display_str, "Bluetooth");
	glcd_tiny_draw_string(10,5,display_str);
	
	sprintf(display_str, "Debug Menu");
	glcd_tiny_draw_string(10,6,display_str);
	
	
	sprintf(display_str, ">");
	glcd_tiny_draw_string(3, cur_Y,display_str);
	
	glcd_write();
	
}


void fn_menu_debug(void){
	//  Set initial conditions
	if (state_change) {
		cur_Y = 2;
		cur_Y_low = 2;
		cur_Y_high = 4;
	}
	
	// Button Handler
	switch(last_input){
		case input_button2:
			if(cur_Y > cur_Y_low){--cur_Y; }
			break;
		case input_button3:
			if(cur_Y < cur_Y_high){++cur_Y; }
			break;
		case input_button1:
			if (cur_Y==2){
				//  Raw Data Debug
				current_input = input_debug_rawData;
			}else if(cur_Y == 3){
				//  Backlight Debug
				current_input = input_debug_backlight;
			} else if(cur_Y == 4){
				//  Charger Debug
				current_input = input_debug_charger;
			}
		
	}
	
	// Display
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"Debug Menu:");
	glcd_tiny_draw_string(0,0,display_str);
	
	//Display Options
	sprintf(display_str, "Sensor Raw Data");
	glcd_tiny_draw_string(20, 2, display_str);
	sprintf(display_str, "Backlight Manual");
	glcd_tiny_draw_string(20, 3, display_str);
	sprintf(display_str, "Charger Info");
	glcd_tiny_draw_string(20, 4, display_str);
	
	// Display soft keys
	draw_arrows(2);//  Draw up arrow at button 2
	draw_arrows(3);//  Draw down arrow at button 3
	sprintf(display_str, "Enter");
	glcd_tiny_draw_string(96,0,display_str);
	sprintf(display_str, "Back");
	glcd_tiny_draw_string(103,7,display_str);
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(10, cur_Y,display_str);
	
	glcd_write();
	
	
}



void fn_menu_cal(void){
	//  Set initial conditions
	if (state_change) {
		cur_Y = 2;
		cur_Y_low = 2;
		cur_Y_high = 5;
	}
	
	// Button Handler
	switch(last_input){
		case input_button2:
			if(cur_Y > cur_Y_low){--cur_Y; }
			break;
		case input_button3:
			if(cur_Y < cur_Y_high){++cur_Y; }
			break;
		case input_button1:
			if (cur_Y==2){
				//  Display Report
				current_input = input_disp_cal_report;
			}else if(cur_Y == 3){
				//  Distance Calibration
				current_input = input_dist_calibration;
				
			} else if (cur_Y==4){
				//  Accelerometer and Compass Calibration
				current_input = input_acc_comp_calibration;
			} else if (cur_Y==5){
				current_input = input_loop_test;
			}
	}
	
	// Display
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
		//  Display Title
	sprintf(display_str,"Calibration:");
	glcd_tiny_draw_string(0,0,display_str);
	
	//Display Options
	sprintf(display_str, "Display Report");
	glcd_tiny_draw_string(20, 2, display_str);
	sprintf(display_str, "Cal Distance");
	glcd_tiny_draw_string(20, 3, display_str);
	sprintf(display_str, "Cal AZM & INCL");
	glcd_tiny_draw_string(20, 4, display_str);
	sprintf(display_str,"Loop Test");
	glcd_tiny_draw_string(20, 5, display_str);
	
	// Display soft keys
	draw_arrows(2);//  Draw up arrow at button 2
	draw_arrows(3);//  Draw down arrow at button 3
	sprintf(display_str, "Enter");
	glcd_tiny_draw_string(96,0,display_str);
	sprintf(display_str, "Back");
	glcd_tiny_draw_string(103,7,display_str);
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(10, cur_Y,display_str);
	
	glcd_write();
	
}

void fn_debug_backlight(void){
	static char colorRef;
	static struct BACKLIGHTCOLOR *colorPtr;
	//  Set initial conditions
	if (state_change) {
		cur_Y = 2;
		cur_Y_low = 2;
		cur_Y_high = 5;
		options.backlight_setting.colorRef = 0;
		backlightOn();
		colorPtr = backlightCustomAdjust(0, 0);
	}	
	
	switch(cur_Y){
		case 2:
			colorRef = 'r';
			break;
		case 3:
			colorRef = 'g';
			break;
		case 4:
			colorRef = 'b';
			break;
		case 5:
			colorRef = 'L';
			break;
		default:
			colorRef = 'L';
			break;
			
	}	
	
	// Button Handler
	switch(last_input){
		case input_button1:
			if(cur_Y > cur_Y_low){--cur_Y; }
			break;
		case input_button4:
			if(cur_Y < cur_Y_high){++cur_Y; }
			break;
		case input_button2:
			backlightCustomAdjust(colorRef, 1);
			break;
		case input_button3:
			backlightCustomAdjust(colorRef, -1);
			break;
	}
	

	
	// Display
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"Backlight Debug:");
	glcd_tiny_draw_string(0,0,display_str);
	
	//Display Options
	sprintf(display_str, "Red:   %d", colorPtr->red);
	glcd_tiny_draw_string(20, 2, display_str);
	sprintf(display_str, "Green: %d", colorPtr->green);
	glcd_tiny_draw_string(20, 3, display_str);
	sprintf(display_str, "Blue:  %d", colorPtr->blue);
	glcd_tiny_draw_string(20, 4, display_str);
	sprintf(display_str, "Light: %d", options.backlight_setting.brightness);
	glcd_tiny_draw_string(20, 5, display_str);
	
	// Display soft keys
	draw_arrows(2);//  Draw up arrow at button 2
	draw_arrows(3);//  Draw down arrow at button 3
	sprintf(display_str, "Up");
	glcd_tiny_draw_string(96,0,display_str);
	sprintf(display_str, "Down");
	glcd_tiny_draw_string(103,7,display_str);
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(10, cur_Y,display_str);
	
	glcd_write();
	
	
}



void fn_set_options(void){
	//  Set initial conditions
	if (state_change) {
		cur_Y = 1;
		cur_Y_low = 1;
		cur_Y_high = 7;
	}
	
	// Button Handler
	switch(last_input){
		case input_button2:
			if(cur_Y > cur_Y_low){--cur_Y; }
			break;
		case input_button3:
			if(cur_Y < cur_Y_high){++cur_Y; }
			break;
		case input_button1:
			switch (cur_Y){
				case 1:
					//  Distance Units
					if (options.current_unit_dist == feet){ options.current_unit_dist = meters;}
					else{options.current_unit_dist = feet;}
					save_user_settings();
					break;
				case 2:
					//  Distance Units
					if (options.current_unit_temp == celsius){ options.current_unit_temp = fahrenheit;}
					else{options.current_unit_temp = celsius;}
					save_user_settings();
					break;
				case 3:
					// Shot Delay
					options.shot_delay = options.shot_delay+1;
					if (options.shot_delay>shot_delay_max){options.shot_delay = 0;}
					save_user_settings();
					break;
				case 4:
					// Charge Current
					if (options.chargeCurrent == 500){ options.chargeCurrent = 100;}
					else{options.chargeCurrent = 500;}
					setup_charger();
					save_user_settings();
					break;
				case 5:
					// Charge Current
					adjustErrorSensitivity();
					save_user_settings();
					break;	
				case 6:
					// Backlight Color
					backlightColorToggle();					
					save_user_settings();
					break;
				case 7:
					// Backlight Color
					backlightLevelToggle();
					save_user_settings();
					break;
			}
			

	}
	
	// Display
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	
	//Display Options
	if (options.current_unit_dist==feet){
		sprintf(display_str, "Distance:  Feet");
	}else{
		sprintf(display_str, "Distance:  Meters");
	}
	glcd_tiny_draw_string(5, 1, display_str);
	if (options.current_unit_temp==fahrenheit){
		sprintf(display_str, "Temp:  Fahrenheit");
		}else{
		sprintf(display_str, "Temp:  Celsius");
	}
	glcd_tiny_draw_string(5, 2, display_str);
	sprintf(display_str,"Shot Delay: %d sec",options.shot_delay);
	glcd_tiny_draw_string(5, 3, display_str);
	sprintf(display_str,"Charge Curr: %dmA",options.chargeCurrent);
	glcd_tiny_draw_string(5, 4, display_str);
	sprintf(display_str,"Err Sens: %0.2f deg", options.errorSensitivity);
	glcd_tiny_draw_string(5, 5, display_str);
	sprintf(display_str,"BL Color: %s", backlightGetCurrentColor());
	glcd_tiny_draw_string(5, 6, display_str);
	sprintf(display_str,"BL Level: %d", options.backlight_setting.brightness);
	glcd_tiny_draw_string(5, 7, display_str);
	
	//  Display Title
	sprintf(display_str,"Options:");
	glcd_tiny_draw_string(0,0,display_str);
	
	// Display soft keys
	draw_arrows(2);//  Draw up arrow at button 2
	draw_arrows(3);//  Draw down arrow at button 3
	sprintf(display_str, "Adjust");
	glcd_tiny_draw_string(90,0,display_str);
	sprintf(display_str, "Back");
	glcd_tiny_draw_string(104,7,display_str);
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(0, cur_Y,display_str);
		
	glcd_write();

}



void fn_set_bluetooth(void){
	char str_on[] = "On";
	char str_off[] = "Off";
	char *str_ptr;
	bool current_state;
	static bool USART_enabled;
	//debug
	bool debug;
	debug = ioport_get_pin_level(BLE_ota);
	
	if (state_change) {
		cur_Y=2;
		cur_Y_low=2;
		cur_Y_high=5;
		last_input = input_none;
		
	}
	
	switch(last_input){
		case input_button2:
			if(cur_Y > cur_Y_low){--cur_Y; }
			break;
		case input_button3:
			if(cur_Y < cur_Y_high){++cur_Y; }
			break;
		case input_button1:
			if(cur_Y == 2){
				current_state = ioport_get_pin_level(BLE_autorun);
				ioport_set_pin_level(BLE_autorun, !current_state);
			} else if (cur_Y==3){
				current_state = ioport_get_pin_level(BLE_reset);
				ioport_set_pin_level(BLE_reset, !current_state);
			} else if (cur_Y==4){
				current_state = ioport_get_pin_level(BLE_ota);
				ioport_set_pin_level(BLE_ota, !current_state);		
			} else if (cur_Y==5){
				if (USART_BLE_enabled){
					USART_BLE_enabled = false;
					usart_disable(&usart_BLE);
					ioport_set_pin_dir(MCU_RTS1, IOPORT_DIR_OUTPUT);
					ioport_set_pin_level(MCU_RTS1, false);
					ioport_set_pin_dir(MCU_CTS1, IOPORT_DIR_INPUT);
					ioport_reset_pin_mode(MCU_TX1);
					ioport_reset_pin_mode(MCU_RX1);
					ioport_set_pin_dir(MCU_TX1, IOPORT_DIR_INPUT);
					ioport_set_pin_dir(MCU_RX1, IOPORT_DIR_INPUT);
				}else{
					USART_BLE_enabled = true;
					configure_usart();
				}
				
			}
	}


	
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	
	//Display Options
	sprintf(display_str,"AutoRun On/Off");
	glcd_tiny_draw_string(25, 2, display_str);
	sprintf(display_str,"Reset On/Off");
	glcd_tiny_draw_string(25, 3, display_str);
	sprintf(display_str,"OTA On/Off");
	glcd_tiny_draw_string(25, 4, display_str);
	sprintf(display_str,"MC UART On/Off");
	glcd_tiny_draw_string(25, 5, display_str);
	
	// Display soft keys
	sprintf(display_str,"Bluetooth:     Adjust");
	glcd_tiny_draw_string(0,0,display_str);
	sprintf(display_str, "Back");
	glcd_tiny_draw_string(96,7,display_str);
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(18, cur_Y,display_str);
	
	debug = ioport_get_pin_level(BLE_ota);			
	//Display Status
	if (ioport_get_pin_level(BLE_autorun)){ str_ptr = str_off;}
	else{ str_ptr = str_on;}
	glcd_tiny_draw_string(0, 2,str_ptr);
	if (ioport_get_pin_level(BLE_reset)){ str_ptr = str_off;}
	else{str_ptr = str_on;}
	glcd_tiny_draw_string(0, 3,str_ptr);
	if (ioport_get_pin_level(BLE_ota)){ str_ptr = str_on;}
	else{str_ptr = str_off;}
	glcd_tiny_draw_string(0, 4,str_ptr);
	if (USART_BLE_enabled){ str_ptr = str_on;}
	else{str_ptr = str_off;}
	glcd_tiny_draw_string(0, 5,str_ptr);
	

	
	glcd_write();
	
}


void fn_set_clock(void){
	
	// Used for setting clock
	typedef struct {
		uint8_t x_pos; //cursor x position
		uint8_t y_pos;//cursor y position
		uint8_t min;  //unit max
		uint8_t max;  //unit min
		uint8_t *data; //unit location
	} CLOCK_SETTING;
	
	CLOCK_SETTING clock_table[] = {
		{10,	3,	0,	0x24, &temp_time.hours},
		{40,	3,	0,	0x59, &temp_time.minutes},
		{70,	3,	0,	0x59, &temp_time.seconds},
		{10,	6,	1,	0x31, &temp_time.date},
		{40,	6,	1,	0x12, &temp_time.month},
		{76,	6,	0,	0x99, &temp_time.year}
	};
	
	if (state_change) {
		cur_X = 0;
		cur_X_low = 0;
		cur_X_high = 5;
		get_time();
		memcpy(&temp_time,&current_time,sizeof(current_time));	
	}
	
	
	
	
	switch(last_input){
		case input_button2:
			if(*clock_table[cur_X].data < clock_table[cur_X].max){
				++ *clock_table[cur_X].data;
				bcd_adj(clock_table[cur_X].data);
				}
			break;
		case input_button3:
			if(*clock_table[cur_X].data > clock_table[cur_X].min){
				-- *clock_table[cur_X].data;
				bcd_adj(clock_table[cur_X].data);
				}
			break;
		case input_button1:
			if(cur_X >= cur_X_high){
				set_time();
				current_input = input_state_complete;
				}
			else{++cur_X;}
			break;
	}
	
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	
	sprintf(display_str,"Set Clock:");
	glcd_tiny_draw_string(0,0,display_str);
	
	sprintf(display_str,"Hour Min  Sec");
	glcd_tiny_draw_string(10,1,display_str);
	sprintf(display_str,"%02x   %02x   %02x",
	temp_time.hours, temp_time.minutes, temp_time.seconds);
	glcd_tiny_draw_string(10,2,display_str);
	
	sprintf(display_str,"Date Month Year");
	glcd_tiny_draw_string(10,4,display_str);
	sprintf(display_str,"%02x   %02x    20%02x",
	temp_time.date, temp_time.month, temp_time.year);
	glcd_tiny_draw_string(10,5,display_str);
	
	// Display soft keys
	sprintf(display_str, "+");
	glcd_tiny_draw_string(121,3,display_str);
	sprintf(display_str, "-");
	glcd_tiny_draw_string(121,5,display_str);
	sprintf(display_str, "Next");
	glcd_tiny_draw_string(103,0,display_str);
	sprintf(display_str, "Cancel");
	glcd_tiny_draw_string(92,7,display_str);
	
	//Display Pointer
	sprintf(display_str, "^");
	glcd_tiny_draw_string(clock_table[cur_X].x_pos,clock_table[cur_X].y_pos,display_str);
	
	glcd_write();
		
}




void fn_main_display(void){
	
	print_data_screen();
	
	
	//Handle Button Inputs
	if(last_input==input_button2){
		if(options.backlight_setting.brightness<3){
			backlightPlus();
			save_user_settings();
		}
	}else if(last_input==input_button3){
		if(options.backlight_setting.brightness>0){
			backlightMinus();
			save_user_settings();
		}
	}
	
}

void print_data_screen(void){
	static bool flipper;
	uint8_t batt_charge_status;
	get_time();
	batt_charge_status = getChargerStatus();
	
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	
	if (options.current_unit_temp == fahrenheit){
		sprintf(display_str,"T:%4.1fF", current_time.temperatureF);
	}else{
		sprintf(display_str,"T:%4.1fC", current_time.temperatureC);
	}
	
	glcd_tiny_draw_string(86,7,display_str);
	
	sprintf(display_str,"%02x:%02x:%02x", current_time.hours, current_time.minutes, current_time.seconds);
	glcd_tiny_draw_string(0,7,display_str);


	
	//  Draw Charge Status	
	if (batt_charge_status){
		if (flipper){
			flipper = false;
		}else{
			flipper = true;
		}
		//  Draw lines
		glcd_draw_line(49, 64, 49, 54, BLACK);
		glcd_draw_line(49, 54, 83, 54, BLACK);
		glcd_draw_line(83, 64, 83, 54, BLACK);
		
			
	}else{
		flipper = true;
	}
	if (flipper){
		sprintf(display_str,"B:%02d%%", getBatteryLevel());		
	}else{
		sprintf(display_str,"B:%02d", getBatteryLevel());
	}
	glcd_tiny_draw_string(51,7,display_str);
	
	#define x1 0  //  Starting X position for Ref data
	#define x2 28 //  Starting X position for Distance data
	#define x3 63 //  Starting X position for Azimuth data
	#define x4 98 //  Starting X position for Inclination data
	#define y1 0  //  Starting Y position for header
	#define y2 10 //  Starting Y position for data
	
	#define num_lines 5
	//Print Data Headers
	sprintf(display_str,"REF");
	glcd_draw_string_xy(x1,y1, display_str);
	
	sprintf(display_str,"DIST");
	glcd_draw_string_xy(x2,y1,display_str);
	
	sprintf(display_str,"AZM");
	glcd_draw_string_xy(x3, y1, display_str);
	glcd_draw_circle(x3+21, y1+2, 2, BLACK);

	sprintf(display_str,"INCL");
	glcd_draw_string_xy(x4, y1, display_str);
	glcd_draw_circle(x4+26, y1+2, 2, BLACK);

	//Print Grid Lines
	glcd_draw_line(0, y1+8, 128, y1+8, BLACK);
	glcd_draw_line(0, y2+8, 128, y2+8, BLACK);
	glcd_draw_line(x2-2, 0, x2-2, 53, BLACK);
	glcd_draw_line(x3-2, 0, x3-2, 53, BLACK);
	glcd_draw_line(x4-2, 0, x4-2, 53, BLACK);
	
	//Print Data
	uint16_t i;
	uint16_t y_temp;
	int16_t temp_index;
	int16_t temp_ref;
	for (i=0;i<num_lines;i++){
		temp_index=data_buf_ind-i;
		temp_ref=data_ref-i;
		if(current_state==st_aim){//bump everything down to display active reading
			temp_index=temp_index+1;
			temp_ref=temp_ref+1;
		}
		//Set index for buffer wrap-around
		if (temp_index<0){
			temp_index = buf_length+temp_index;
			}else if(temp_index>=buf_length){
			temp_index = temp_index-buf_length;
		}
		//print lines
		if ((temp_ref)>0){
			//Adjust line spacing for grid lines
			if(i<2){y_temp=y2+10*i;}
			else {y_temp=y2+9*i;	}
			if((current_state==st_main_display)||(i>0)){//do not print reference and distance for active reading
				sprintf(display_str, "%d", data_buf[temp_index].index_ref);//reference
				glcd_draw_string_xy(x1, y_temp, display_str);
				sprintf(display_str, "%.1f", data_buf[temp_index].distance);//distance
				glcd_draw_string_xy(x2, y_temp, display_str);
				
				//  Add Error message if necessary
				if (data_buf[temp_index].num_errors!=0){
					glcd_draw_string_xy(x1+18, y_temp, "E");				
				}
				
			}

			sprintf(display_str, "%.1f", data_buf[temp_index].azimuth);//Azimuth
			glcd_draw_string_xy(x3, y_temp, display_str);
			sprintf(display_str, "%.1f", data_buf[temp_index].inclination);//Inclination
			glcd_draw_string_xy(x4, y_temp, display_str);
		}

	}
	//debug//////////////
	//sprintf(display_str,"%d", debug1);
	//glcd_tiny_draw_string(0,0,display_str);
	//sprintf(display_str,"%d", debug2);
	//glcd_tiny_draw_string(0,1,display_str);
	
	////////////////////////

	
	glcd_write();
}

void draw_arrows(uint8_t button){
	//draw menu up/down arrows
	switch(button){
		case 2: //  Display arrow at button 2
			glcd_draw_line(116, 26, 120, 22, BLACK);
			glcd_draw_line(120, 22, 124, 26, BLACK);
			break;
		case 3: // Display arrow at button 3
			glcd_draw_line(116, 40, 120, 44, BLACK);
			glcd_draw_line(120, 44, 124, 40, BLACK);
			break;
	}
	
}



void fn_aim_abort(void){
	rangefinder_on_off(false);
	ioport_set_pin_level(laser_reset, false);

	current_input = input_state_complete;
	
}



void config_pins_powerup(void){
	ioport_init();
	//Input Buttons
	ioport_set_pin_dir(button1, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(button1, IOPORT_MODE_PULLUP);
	ioport_set_pin_dir(button2, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(button2, IOPORT_MODE_PULLUP);
	ioport_set_pin_dir(button3, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(button3, IOPORT_MODE_PULLUP);
	ioport_set_pin_dir(button4, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(button4, IOPORT_MODE_PULLUP);
	ioport_set_pin_dir(buttonE, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(buttonE, IOPORT_MODE_PULLUP);
	//LCD setup
	ioport_set_pin_dir(LCD_SPI_SS_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LCD_SPI_SS_PIN, true);
	ioport_set_pin_dir(LCD_SPI_DC_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LCD_SPI_DC_PIN, false);
	ioport_set_pin_dir(LCD_SPI_RST_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LCD_SPI_RST_PIN, false);
	//power-supply 2 voltage enable
	ioport_set_pin_dir(V2_enable, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(V2_enable, true);
	//SPI pin
	ioport_set_pin_dir(lcd_SS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(lcd_SS, true);
	ioport_set_pin_dir(acc1_SS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(acc1_SS, true);
	ioport_set_pin_dir(acc2_SS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(acc2_SS, true);
	ioport_set_pin_dir(mag1_SS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(mag1_SS, true);
	ioport_set_pin_dir(mag2_SS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(mag2_SS, true);
	ioport_set_pin_dir(SD_CS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SD_CS, true);
	ioport_set_pin_dir(BLE_SS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BLE_SS, true);
	//UART Pins
	//ioport_set_pin_dir(MCU_RTS1, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(MCU_RTS1, false);
	//ioport_set_pin_dir(MCU_CTS1, IOPORT_DIR_INPUT);
	//BLE pins
	ioport_set_pin_dir(BLE_ota, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BLE_ota, false);// low to disable programming over BLE
	ioport_set_pin_dir(BLE_autorun, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BLE_autorun, true);//low for autorun enabled, high for development mode
	ioport_set_pin_dir(BLE_reset, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BLE_reset, false); //low, hold in reset
	//miscellaneous
	ioport_set_pin_dir(laser_reset, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(laser_reset, false);
	ioport_set_pin_dir(SD_CS, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SD_CS, IOPORT_MODE_PULLUP);
}




void config_pins_powerdown(void){
	//spi pins
	ioport_reset_pin_mode(mosi);
	ioport_reset_pin_mode(miso);
	ioport_reset_pin_mode(sclk);
	ioport_set_pin_dir(mosi, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(miso, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(sclk, IOPORT_DIR_INPUT);
	//i2c pins
	ioport_reset_pin_mode(SDA);
	ioport_reset_pin_mode(SCL);
	ioport_set_pin_dir(SDA, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SCL, IOPORT_DIR_INPUT);
	// UART pins
	ioport_reset_pin_mode(MCU_TX1);
	ioport_reset_pin_mode(MCU_TX2);
	ioport_reset_pin_mode(MCU_RX1);
	ioport_reset_pin_mode(MCU_RX2);
	ioport_set_pin_dir(MCU_TX1, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(MCU_TX2, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(MCU_RX1, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(MCU_RX2, IOPORT_DIR_INPUT);
	//set all pins used on V2 devices to high impedance
	ioport_set_pin_dir(lcd_SS , IOPORT_DIR_INPUT);
	ioport_set_pin_dir(acc1_SS, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(acc2_SS , IOPORT_DIR_INPUT);
	ioport_set_pin_dir(mag1_SS , IOPORT_DIR_INPUT);
	ioport_set_pin_dir(mag2_SS , IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SD_CS , IOPORT_DIR_INPUT);
	ioport_set_pin_dir(laser_reset , IOPORT_DIR_INPUT);
	ioport_set_pin_dir(LCD_SPI_SS_PIN , IOPORT_DIR_INPUT);
	ioport_set_pin_dir(LCD_SPI_DC_PIN , IOPORT_DIR_INPUT);
	ioport_set_pin_dir(LCD_SPI_RST_PIN , IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BLE_autorun, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BLE_ota, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BLE_SS, IOPORT_DIR_INPUT);
	ioport_set_pin_level(BLE_reset, false);
	ioport_set_pin_level(BLE_SS, true);
}




void configure_extint_channel(void)
{
	struct extint_chan_conf config_extint_chan;

	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING;
	config_extint_chan.filter_input_signal  = true;
	config_extint_chan.enable_async_edge_detection = true;
	// button 4
	config_extint_chan.gpio_pin           = PIN_PA07A_EIC_EXTINT7;
	config_extint_chan.gpio_pin_mux       = MUX_PA07A_EIC_EXTINT7;
	extint_chan_set_config(7, &config_extint_chan);
	// button 3
	config_extint_chan.gpio_pin           = PIN_PA06A_EIC_EXTINT6;
	config_extint_chan.gpio_pin_mux       = MUX_PA06A_EIC_EXTINT6;
	extint_chan_set_config(6, &config_extint_chan);
	// button 2
	config_extint_chan.gpio_pin           = PIN_PA04A_EIC_EXTINT4;
	config_extint_chan.gpio_pin_mux       = MUX_PA04A_EIC_EXTINT4;
	extint_chan_set_config(4, &config_extint_chan);
	// button 1
	config_extint_chan.gpio_pin           = PIN_PB09A_EIC_EXTINT9;
	config_extint_chan.gpio_pin_mux       = MUX_PB09A_EIC_EXTINT9;
	extint_chan_set_config(9, &config_extint_chan);
	
	// button Ext
	config_extint_chan.detection_criteria = EXTINT_DETECT_BOTH;
	config_extint_chan.gpio_pin           = PIN_PA05A_EIC_EXTINT5;
	config_extint_chan.gpio_pin_mux       = MUX_PA05A_EIC_EXTINT5;
	extint_chan_set_config(5, &config_extint_chan);
	
}

void configure_extint_callbacks(void)
{
	// Button 4
	extint_register_callback(extint_routine, 7,	EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(7,EXTINT_CALLBACK_TYPE_DETECT);
	// Button 3
	extint_register_callback(extint_routine, 6,	EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(6,EXTINT_CALLBACK_TYPE_DETECT);
	// Button 2
	extint_register_callback(extint_routine, 4,	EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(4,EXTINT_CALLBACK_TYPE_DETECT);
	// Button 1
	extint_register_callback(extint_routine, 9,	EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(9,EXTINT_CALLBACK_TYPE_DETECT);
	
	// Button External
	extint_register_callback(extint_routine, 5,	EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(5,EXTINT_CALLBACK_TYPE_DETECT);
}

void extint_routine(void)
{
	switch (extint_get_current_channel()){
		case 5:
			externalButtonRoutine(!ioport_get_pin_level(buttonE));
			break;
		case 7:
			current_input = input_button4;
			break;
		case 6:
			current_input = input_button3;
			break;
		case 4:
			current_input = input_button2;
			break;			
		case 9:
			current_input = input_button1;
			break;
		
	}
		
}

void externalButtonRoutine(bool buttonOn){
	// Button External
	// Special Routines for External Button
	// If held down for less than X seconds, provides normal input upon release
	// If held down for more than X seconds, a separate interrupt routine provides powerdown input
	// When in powerdown state, 3 quick clicks through a separate interrupt routine provides powerup input
	
	
	if (current_state == st_powerdown){
		//  wakup on 3 quick clicks
		if (buttonOn){//if external button is pressed
			quick3_timer(true);
			click_counter = click_counter+1;//  click_counter reset by timer interrupt routine if timer expires.
		}
		if (click_counter>=3){
			current_input = input_wakeup;
		}
		return;	
	}
	
	
	
	if (buttonOn){
		//  Trigger on if button is pressed
		if(!buttonE_triggered){
			buttonE_triggered=true;
			//trigger timer
			tc_set_count_value(&timer1, 0);
			tc_start_counter(&timer1);
		}
		return;
		
	}else{
		//  Releaed in a short amount of time, normal input
		buttonE_triggered=false;
		tc_stop_counter(&timer1);
		current_input = input_buttonE;
		return;	
		
	}
	
		
}



void fn_powerdown(void){
	enum sleepmgr_mode sleep_mode;
	if (state_change){
		// Disable watchdog timer
		wdt_disable();
		//  Switch-Over to low power internal clock
		clock_32k_source(clock_int);
		ext_osc_onoff(false);
		//  Put hardware in low-power state
		disable_comms();
		config_pins_powerdown();		
		ioport_set_pin_level(V2_enable, false);//disable V2 power supply		
		configure_timers(st_powerdown);//Disable TC	
		
	};	
		
	sleepmgr_lock_mode(SLEEPMGR_STANDBY);
	sleep_mode = sleepmgr_get_sleep_mode();
	sleepmgr_sleep(SLEEPMGR_STANDBY);
	
}

void fn_powerup(void){
	config_pins_powerup();
	delay_ms(100);
	
	setup_spi();
	configure_i2c_master();
	configure_usart();
	configure_usart_callbacks();
	
	glcd_init();
	backlightOn();
	configure_extint_channel();
	configure_extint_callbacks();
	setup_accel(&slave_acc1);
	setup_accel(&slave_acc2);
	setup_mag(&slave_mag1);
	setup_mag(&slave_mag2);
	
	
	
	system_interrupt_enable_global();

	config_spi(LCD);
	
	delay_ms(50);	
	ext_osc_onoff(true);
	delay_ms(50);	
	setup_XOSC32k();
	clock_32k_source(clock_ext);
	
	configure_timers(st_powerup);
	
	configure_SD();
	
	ioport_reset_pin_mode(BLE_ota);//  Needed to reset pin mode; set in some previous initialization
	ioport_set_pin_dir(BLE_ota, IOPORT_DIR_OUTPUT);//  Needed to reset pin mode; set in some previous initialization
	ioport_set_pin_level(BLE_ota, false);//  Needed to reset pin mode; set in some previous initialization
	
	wdt_enable();
	
	buttonE_triggered=false;
	current_input = input_state_complete;
	
}


void setup_XOSC32k(void){
	struct system_clock_source_xosc32k_config xosc32k_conf;
	system_clock_source_xosc32k_get_config_defaults(&xosc32k_conf);

	xosc32k_conf.frequency           = 32768UL;
	xosc32k_conf.external_clock      = CONF_CLOCK_XOSC32K_EXTERNAL_CRYSTAL;
	xosc32k_conf.startup_time        = CONF_CLOCK_XOSC32K_STARTUP_TIME;
	xosc32k_conf.enable_1khz_output  = CONF_CLOCK_XOSC32K_ENABLE_1KHZ_OUPUT;
	xosc32k_conf.enable_32khz_output = CONF_CLOCK_XOSC32K_ENABLE_32KHZ_OUTPUT;
	xosc32k_conf.on_demand           = false;
	xosc32k_conf.run_in_standby      = CONF_CLOCK_XOSC32K_RUN_IN_STANDBY;

	system_clock_source_xosc32k_set_config(&xosc32k_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_XOSC32K);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_XOSC32K));
	if (CONF_CLOCK_XOSC32K_ON_DEMAND) {
		OSC32KCTRL->XOSC32K.bit.ONDEMAND = 1;
	}
	
	
	
}


void clock_32k_source(enum clock_type ext_int){
	struct system_gclk_gen_config gclock_gen_conf;
	
	system_gclk_gen_get_config_defaults(&gclock_gen_conf);
	gclock_gen_conf.run_in_standby = true;
	if(ext_int == clock_ext){
		gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_XOSC32K;
	}else{
		gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_ULP32K;
	}	
	system_gclk_gen_set_config(GCLK_GENERATOR_2, &gclock_gen_conf);
    system_gclk_gen_enable(GCLK_GENERATOR_2);
		
}


void clock_16M_source(enum clock_type high_low){
	struct system_gclk_gen_config gclock_gen_conf;
	
	system_gclk_gen_get_config_defaults(&gclock_gen_conf);
	gclock_gen_conf.run_in_standby = true;
	if(high_low == clock_high){
		gclock_gen_conf.source_clock =SYSTEM_CLOCK_SOURCE_OSC16M;
	}else{
		gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_ULP32K;
	}
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclock_gen_conf);
    system_gclk_gen_enable(GCLK_GENERATOR_0);
	
}

void getDefaultOptions(struct OPTIONS *optionptr){
	
	optionptr->shot_delay = 0;//seconds
	optionptr->current_unit_temp = celsius;
	optionptr->current_unit_dist = meters;
	optionptr->chargeCurrent = 100;//mA
	optionptr->errorSensitivity = 1;
	//optionptr->backlight_setting.blue = 30;
	//optionptr->backlight_setting.green = 30;
	//optionptr->backlight_setting.red = 22;
	optionptr->backlight_setting.colorRef = 1;//white
	optionptr->backlight_setting.brightness = 3;
	optionptr->backlight_setting.maxColor = 30;
	optionptr->backlight_setting.maxBrightness = 5;
	
	optionptr->Settings_Initialized_Key = 0xC9;//  Indicator that settings have been initialized

	
}

