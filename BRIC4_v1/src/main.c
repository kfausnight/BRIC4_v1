
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

#include <asf.h>
#include <main.h>
//#include "conf_usb.h"
//

// Debug
volatile uint32_t debug1, debug2, debug3, debug4;
uint32_t debug_ct1, debug_ct2;
#define DEBUG_DISPLAY false

//  State Machine Info
volatile enum INPUT current_input, last_input; // Current and Last Inputs
volatile enum STATE current_state; //  Current State
volatile bool state_change = true;
//  Status Variables

volatile bool isCharging = false;//  Variable to track when plugged in and charging
volatile bool buttonE_triggered = false; // Variable to track when external button is triggered
//  Options structure
struct OPTIONS options;
//  Time structures
struct TIME current_time, temp_time;
//SD card functions
FATFS FatFS;         /* Work area (file system object) for logical drives */
//FIL file1, file2, file_cal_report, file_cal_raw;      /* file objects */
FRESULT SD_status;
char filename[100];
char write_str1[400];
char write_str2[400];
// Buffer to hold characters for display
char display_str[200];

//  UART Buffers
volatile uint8_t rxBufferLaser[UART_BUFFER_LENGTH];
volatile uint8_t rxBufferLaserIndex;
volatile char rxBufferBle[UART_BUFFER_LENGTH];
volatile uint8_t rxBufferBleIndex;
volatile char debugBuff[DEBUG_BUFFER_LENGTH];
volatile uint32_t debugBuffIndex = 0;
char *debugBuffPtr;

//  Calibration Data Buffers
// Azm and Inc Calibration
uint32_t nGroups;//Counter for current completed groups
uint32_t nPoints;// Counter for completed # of points
float a1Raw[NBUFF][3], a2Raw[NBUFF][3], m1Raw[NBUFF][3], m2Raw[NBUFF][3];
float a1Cal[NBUFF][3], a2Cal[NBUFF][3], m1Cal[NBUFF][3], m2Cal[NBUFF][3];
float azimuth[NBUFF], inclination[NBUFF], roll[NBUFF];
// Distance Calibration
float dist_raw_buf[SHOT_SIZE];
float dist_disp_buf[SHOT_SIZE];
float temp_dist_offset;
// Loop Test
float loop_distance, loop_horizontal, loop_vertical, loop_azimuth, loop_error;
//   Calibration data structures
struct INST_CAL a1_calst, a2_calst, m1_calst, m2_calst, dist_calst;
struct CAL_REPORT cal_report;

// Measurement Data Buffers
struct MEASUREMENT data_buf[NBUFF_MEAS];
uint8_t data_buf_ind = 0;
uint32_t data_ref = 0;

//  Interrupt Management
void configure_extint_channel(void);
void configure_extint_callbacks(void);
void extint_routine(void);



//  USB funcations
bool my_flag_autorize_msc_transfert = false;
bool usb_transaction_requested = false;
//bool my_callback_msc_enable(void);
//void my_callback_msc_disable(void);
//void msc_notify_trans(void);


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
	{st_powerdown,		input_usb_transaction,	st_powerup},
		

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
		
	{st_menu_cal,		input_button4,				st_menu1},
	{st_menu_cal,		input_buttonE,				st_main_display},
	{st_menu_cal,		input_dist_calibration,		st_dist_calibration},
	{st_menu_cal,		input_inc_azm_full_calibration,	st_inc_azm_full_calibration},
	{st_menu_cal,		input_azm_quick_calibration,	st_azm_quick_calibration},
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
	{st_menu_debug,		input_reprocess_inc_azm_cal,			st_process_inc_azm_full_cal},
	{st_menu_debug,		input_reprocess_azm_quick_cal,			st_process_azm_quick_cal},
	
	{st_azm_quick_calibration,		input_button4,				st_aim_abort},
	{st_azm_quick_calibration,		input_state_complete,		st_process_azm_quick_cal},
		
	{st_inc_azm_full_calibration,		input_button4,				st_aim_abort},
	{st_inc_azm_full_calibration,		input_state_complete,		st_process_inc_azm_full_cal},	
		
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
		
	{st_process_inc_azm_full_cal,	input_state_complete,	st_disp_cal_report},
	{st_process_azm_quick_cal,		input_state_complete,	st_disp_cal_report},
		
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
	{st_inc_azm_full_calibration,		fn_inc_azm_full_calibration},
	{st_dist_calibration,			fn_dist_calibration},
	{st_azm_quick_calibration,		fn_azm_quick_calibration},
	{st_process_inc_azm_full_cal,		fn_process_inc_azm_full_cal},
	{st_process_azm_quick_cal,		fn_process_azm_quick_cal},
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
	uint8_t iter; // Iteration counter
	
	uint32_t sizeDebug1, sizeDebug2; 
	

	system_init();
	delay_init();	
	delay_ms(500);	
	
	
	fn_powerup();
	
	configure_timer_counter();
	
	//EEPROM_test();
	

	setChargeCurrent(options.chargeCurrent);
	setup_batt();
	sleepmgr_init();
	load_user_settings();
	load_calibration();
	
	options.SerialNumber = getSN();
	
	//BLE Setup
	BLE_init();
	
	current_state = st_main_display;
	current_input = input_1sec;
	
	////////////////////////////////debug
	// Disable watchdog timer
	//wdt_disable();
	////////////////////////
	
	
	while(1){

		while ((current_input==input_none)){
			//  Handle any USB transactions
			if (usb_transaction_requested){
				spi_setBaud(baudRateMax);
				while(udi_msc_process_trans());
				
				usb_transaction_requested = false;		
				spi_setBaud(baudRateMin);		
			}
			//  Handle any BLE messages that have arrived
			BLE_handleMessage();
			//  Return to Sleep State
			if (current_state==st_powerdown){
				sleepmgr_sleep(SLEEPMGR_STANDBY);
			}else{
				sleepmgr_sleep(SLEEPMGR_IDLE);
			}
			
		}
		
		debug2++;
		wdt_reset_count();
		
		//  Determine if idle powerdown will be performed		
		idle_timeout();//Will produce input = input_powerdown if idle
		
		//  Laser Timeout
		laser_timeout();
		
		
		
		//  Find state 
		//  State a function of current input		
		state_change = false;
		for (iter=0; iter<(sizeof(state_nextstate)/sizeof(STATE_NEXTSTATE));iter++){
			if((current_state==state_nextstate[iter].current) && (current_input==state_nextstate[iter].input)){
				if(current_state!=state_nextstate[iter].next){
					current_state = state_nextstate[iter].next;
					state_change = true;
				}
				break;
			}
		}
		last_input = current_input;
		current_input = input_none;
		
		//  Find and run function for current state
		for(iter=0;iter<(sizeof(state_functions)/sizeof(STATE_FUNCTIONS));iter++){
			if(current_state==state_functions[iter].current){
				state_functions[iter].Function();				
				break;
			}
		}

	}//End of main program while loop
}//end of main

void fn_debug_charger(void){
	uint8_t addressList[] = {
		0x00, 
		0x02,
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
	
	sprintf(display_str, "X: %+0.3f %+0.3f", meas_debug.a1Raw[0], meas_debug.a2Raw[0]);
	glcd_tiny_draw_string(0,1,display_str);
	sprintf(display_str, "Y: %+0.3f %+0.3f", meas_debug.a1Raw[1], meas_debug.a2Raw[1]);
	glcd_tiny_draw_string(0,2,display_str);
	sprintf(display_str, "Z: %+0.3f %+0.3f", meas_debug.a1Raw[2], meas_debug.a2Raw[2]);
	glcd_tiny_draw_string(0,3,display_str);
	
	sprintf(display_str, "Comp 1    Comp 2  ");
	glcd_tiny_draw_string(0,4,display_str);
	
	sprintf(display_str, "X: %+0.3f %+0.3f", meas_debug.m1Raw[0], meas_debug.m2Raw[0]);
	glcd_tiny_draw_string(0,5,display_str);
	sprintf(display_str, "Y: %+0.3f %+0.3f", meas_debug.m1Raw[1], meas_debug.m2Raw[1]);
	glcd_tiny_draw_string(0,6,display_str);
	sprintf(display_str, "Z: %+0.3f %+0.3f Exit", meas_debug.m1Raw[2], meas_debug.m2Raw[2]);
	glcd_tiny_draw_string(0,7,display_str);
	glcd_write();	
}



void fn_azm_quick_calibration(void){
	#define xMin	5
	#define yMin	5
	#define nBands	9
	#define numPoints	96
	uint8_t segBand[] = {3, 9, 9, 18, 18, 18, 9, 9, 3};// 96 total units
	#define bandWidth	6
	#define yHeight	54
	
	
	float m1Buf[NBUFFQAZM][3];
	float m2Buf[NBUFFQAZM][3];
	bool isStable1, isStable2;
	
	struct MEASUREMENT tempM;
	float xPos, yPos;
	bool tracker[NBUFF];
	float debug;

	static uint32_t measCounterCurrent = 0;
	static uint32_t measCounterLast = 0;
	float angX, temp;
	
	
	uint8_t k, band, seg, nSeg, trackerInd, segHeight;
	uint8_t xSeg1, xSeg2, ySeg1, ySeg2;
	
	if (state_change){
		cal_disp_message();
		//  Set up initial settings
		for (k=0;k<NBUFF;k++){
			tracker[k] = false;
		}
		//  Set up initial settings
		cal_init();
		last_input = input_none;
	}
	
	
	measCounterCurrent = 0;
	while(current_input==input_none){
		
		
		//  Take measurement for plotting purposes	
		quick_measurement(&tempM);	
		measCounterCurrent++;//  Debug; checking measurement rate
		
		//  Normalize Roll Angle based on accelerometer
		//  Offset roll angle 180 to center with display up
		tempM.roll = tempM.roll+180;
		if (tempM.roll>360){tempM.roll = tempM.roll-360;}
		yPos = yMin+((tempM.roll)/360)*(yHeight); 
		//  XPos is based on magnetometer angle from X axis
		calc_theta_XY( tempM.m1Raw , &temp, &angX);
		xPos = xMin+((angX+90)/180)*(nBands*bandWidth);
		
		glcd_clear_buffer();
		trackerInd = 0;
		nPoints = 0;
		for (band=0;band<nBands;band++){
			nSeg = segBand[band];
			segHeight = yHeight/nSeg;
			xSeg1 = xMin+bandWidth*band;
			xSeg2 = xSeg1+bandWidth;
			for (seg=0;seg<nSeg;seg++){
				ySeg1 = yMin+segHeight*seg;
				ySeg2 = ySeg1+segHeight;
				
				if(tracker[trackerInd]){
					//  Draw Segment
					nPoints++;
					glcd_fill_rect(xSeg1, ySeg1, bandWidth, segHeight,BLACK);
				}else{
					//  Possibly Add Point
					if((xPos>=xSeg1)&(xPos<xSeg2)&(yPos>=ySeg1) &(yPos<ySeg2)){
						//  Missing point for this location
						//  Add a point if stable
						backlightOff();
						for(k=0;k<NBUFFQAZM;k++){
							//  Take a series of measurements
							read_mag_double(m1Buf[k], m2Buf[k]);
						}
						backlightOn(&options.backlight_setting);
						if (cal_azm_quick_add_point(m1Buf, m2Buf, trackerInd)){
							//  Point was stable; add to buffer
							tracker[trackerInd] = true;
						}
					}//End:    Check to see if point slot is empty
					
				}
				trackerInd++;
			}//  Loop for each row
			
		}//  Loop for each column
		
		//  Draw Roll / Y Indicator
 		glcd_draw_line(0, yPos,64, yPos, BLACK);
		//  Draw Attitude / X Indicator
		glcd_draw_line(xPos, 0,xPos, 64, BLACK);

		//  Draw box around area
		glcd_draw_rect(xMin-1, yMin-1, nBands*bandWidth+1, yHeight+1,BLACK);
		
		// Display Header
		sprintf(display_str, "AZM Cal:");
		glcd_tiny_draw_string(65,0,display_str);	
		sprintf(display_str, "Abort");
		glcd_tiny_draw_string(97,7,display_str);
		sprintf(display_str, "Rate: %d", measCounterLast);
		glcd_tiny_draw_string(70,1,display_str);
		sprintf(display_str, "Status:");
		glcd_tiny_draw_string(70,2,display_str);
		sprintf(display_str, "%d / %d", nPoints, trackerInd);
		glcd_tiny_draw_string(70,3,display_str);
		
		glcd_write();
		
		
		if (nPoints>=trackerInd){
			//  Save all Calibration Raw Data
			cal_done(azm_quick);
			// Signal completion
			current_input = input_state_complete;
		}
	}
	measCounterLast = measCounterCurrent;
	

}

void fn_process_azm_quick_cal(void){

	EEPROM_loadCalRawData(azm_quick);
	load_cal_report();
	nGroups = cal_report.groups;
	nPoints = cal_report.points;
	
	//  Display Message
	glcd_clear_buffer();
	sprintf(display_str, "Processing Data...");
	glcd_tiny_draw_string(0,1,display_str);
	glcd_write();
	cal_azm_quick_process();
	wdt_reset_count();
	sprintf(display_str, "Calibration Complete!");
	glcd_tiny_draw_string(0,3,display_str);
	glcd_write();
	delay_s(3);
	// Signal completion
	current_input = input_state_complete;
	
}


void fn_inc_azm_full_calibration(void){
	struct MEASUREMENT temp_meas;
	uint32_t i, k;
	
	if (state_change){
		cal_disp_message();
		//  Set up initial settings
		cal_init();
		last_input = input_none;
		
	}
	
	// Button Handler
	switch(last_input){
		case input_button1:
			//  Calibration Done button
			if (nGroups>=MIN_GROUPS){//  Requires min_groups to complete
				//  Turn off Rangefinder
				rangefinder_on_off(false);
				//  Save all Calibration Raw Data
				
				cal_done(inc_azm_full);
				
				// Signal completion
				current_input = input_state_complete;
			}
			break;
		case input_button3:
			cal_resetGroup();
			break;
		case input_buttonE:
			//  Set laser and then take measurement
			if (!isLaserOn()){
				// Turn on rangefinder module and laser
				rangefinder_on_off(true);
				laser_on_off(true);
			}else{
				//  Take measurement and process data
				full_measurement(&temp_meas, options.shot_delay);
				//  Turn laser module off
				rangefinder_on_off(false);
				cal_add_datapoint(&temp_meas);
			}
			break;
		case input_button4:
			// Handled by state machine
			//  Exit from routine
			break;
		default:
			break;
	}
	last_input = input_none;	
	
	
	
	#define boxMinX	2
	#define boxAminY	16
	#define boxMminY	40
	#define boxWidth	79
	#define boxHeight	8

	glcd_clear_buffer();
	// Display Header
	sprintf(display_str, "Calibration Mode");
	glcd_tiny_draw_string(0,0,display_str);


	
	//  Draw boxes
	//  Titles
	sprintf(display_str, "Accelerometer");
	glcd_tiny_draw_string(0,1,display_str);
	sprintf(display_str, "Magnetometer");
	glcd_tiny_draw_string(0,4,display_str);
	//  Accelerometer
	//  Add 1 pixel margin so points at edge are visible
	glcd_draw_rect(boxMinX-1, boxAminY, boxWidth+2, boxHeight, BLACK);
	//  Magnetometer
	glcd_draw_rect(boxMinX-1, boxMminY, boxWidth+2, boxHeight, BLACK);
	
	//  Draw Points
	uint8_t ind;
	float posX_A, posY_A, posX_M, posY_M;	
	for (i=0;i<nGroups;i++){
		//  One mark for each group
		ind = i*GROUP_SIZE;
		//  Accemerometer Point
		posX_A = getDispX(a1Raw[ind],boxMinX, boxWidth, false);
		glcd_draw_line(posX_A ,boxAminY, posX_A, boxAminY+boxHeight-1, BLACK);
		//  Magnetometer Point
		posX_M = getDispX(m1Raw[ind],boxMinX, boxWidth, true);
		glcd_draw_line(posX_M ,boxMminY, posX_M, boxMminY+boxHeight-1, BLACK);
	}
	
	//  Display Current Group Information
	#define statBarMinY	15
	#define statBarMinX	88
	#define statBarSpace	10
	#define circleRadius	5
	uint8_t xCir, yCir;
	for (i=0;i<GROUP_SIZE;i++){
		yCir = statBarMinY + i*statBarSpace;
		if (i<cal_getGroupPoints()){
			glcd_fill_circle(statBarMinX, yCir, circleRadius, BLACK);
		}else{
			glcd_draw_circle(statBarMinX, yCir, circleRadius, BLACK);
		}

	}
	sprintf(display_str, "G%d", cal_getCurrentGroup());
	glcd_draw_string_xy(statBarMinX-9,statBarMinY+10*GROUP_SIZE-3,display_str);
	
	// Display Current Data
	sprintf(display_str,"%d of min %d", nGroups, MIN_GROUPS);
	glcd_tiny_draw_string(0,7,display_str);
	
	// Display Soft Keys
	if (nGroups >= MIN_GROUPS){
		drawSoftKeys("Done",""," ","Abort");
	}else{
		drawSoftKeys(" "," ","","Abort");
	}
	sprintf(display_str," G%d", cal_getCurrentGroup());
	draw2LineSoftKey("Reset",display_str,3);
	glcd_write();
	
	
	static uint8_t lastPosA =0;
	static uint8_t lastPosM = 0;

	while(current_input==input_none){	
		quick_measurement(&temp_meas);
		posX_A = getDispX(temp_meas.a1Raw,boxMinX, boxWidth, false);
		posX_M = getDispX(temp_meas.m1Raw,boxMinX, boxWidth, true);
		posX_A = posX_A-2; // To account for width of character
		posX_M = posX_M-2; // To account for width of character
		//  Erase last indicator
		glcd_tiny_draw_char(lastPosA, 3, ' ');
		glcd_tiny_draw_char(lastPosM, 6, ' ');
		//  Draw new indicator
		glcd_tiny_draw_char(posX_A, 3, '^');
		glcd_tiny_draw_char(posX_M, 6, '^');
		//  Write to LCD			
		glcd_write();
		//  Save position to overwrite next cycle
		lastPosA = posX_A;
		lastPosM = posX_M;
	}

	
	
}


void fn_process_inc_azm_full_cal(void){
	
	int32_t i, j, k, g, iter;
	bool tempGroupRemove[NGROUP];
	bool permGroupRemove[NGROUP];
	float incErrArray[NGROUP];
	float azmErrArray[NGROUP];
	
	// Disable Watchdog Timer
	wdt_disable();
	
	//wdt_reset_count();
	
	//  Load cal report to get groups
	//  Note cal_report.groupsAll is never overwritten after data creation
	load_cal_report();
	
	//  Clear out arrays
	for (k=0;k<NGROUP;k++){
		permGroupRemove[k] = false;// Track permanently eliminated group
		tempGroupRemove[k] = false;// track temporarily eliminated loop group
		incErrArray[k] = 0;
		azmErrArray[k] = 0;
	}
	
	glcd_clear_buffer();
	for (iter=0;iter<MAX_BAD_GROUPS;iter++){
		
		for (g=cal_report.groupsAll;g>=0;g--){	
			//  Start at last group and end at 0
			//  Group 0 removed no groups
			//  End at Group 0 so that if no bad groups are detected
			//    this will be the last set of data in memory
			sprintf(display_str, "Processing Cal Data:");
			glcd_tiny_draw_string(0,0,display_str);
			sprintf(display_str, "Iteration %d of %d     ", g, cal_report.groupsAll);
			glcd_tiny_draw_string(0,2,display_str);
			glcd_write();
		
			EEPROM_loadCalRawData(inc_azm_full);
		
			for (k=1;k<=cal_report.groupsAll;k++){
				if (permGroupRemove[k]||(k==g)){
					//Remove group of current loop
					//Also remove group if already in permanent removal list
					//Group 0 removes no groups
					tempGroupRemove[k] = true;
				}else{
					tempGroupRemove[k] = false;
				}
			}
			nGroups = cal_removeGroup(tempGroupRemove, cal_report.groupsAll);
			nPoints = nGroups*GROUP_SIZE;
			
			//  Process Calibration
			uint8_t iterations;
			if (g==0){
				iterations = 3;
			}else{
				iterations = 1;
			}
			cal_full_inc_azm_process(iterations);
			
			//  Add Cal results to array
			incErrArray[g] = cal_report.inc_angle_err;
			azmErrArray[g] = cal_report.azm_angle_err;
								
		}//  End Loop:  Cycle through all groups
		
		uint32_t badGroup = 0;
		float badGroupDelta = 0;
		cal_findBadGroup(incErrArray, azmErrArray, &badGroup, &badGroupDelta);
		
		
		if (badGroupDelta>BAD_GROUP_THRESHOLD){
			permGroupRemove[badGroup] = true;
			sprintf(display_str, "Detected Bad Group");
			glcd_tiny_draw_string(0,4,display_str);
			sprintf(display_str, "                     ");// Clear line
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str, "GRP %d Error %0.3fdeg", badGroup, badGroupDelta);
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str, "Group Removed, ");
			glcd_tiny_draw_string(0,6,display_str);
			sprintf(display_str, "Restarting Analysis");
			glcd_tiny_draw_string(0,7,display_str);
			glcd_write();
		}else{
			break;
		}
		
		
	}//  End Loop Error Optimization loop
	
	//  Write report
	SD_save_raw_data(inc_azm_full);
	SD_write_report();
	SD_add_cal_history(inc_azm_full);
	//  Save data to EEPROM
	save_calibration();
	
	// Calibration Complete
	//wdt_reset_count();
	glcd_clear_buffer();
	sprintf(display_str, "Calibration Complete!");
	glcd_tiny_draw_string(0,2,display_str);
	glcd_write();
	delay_s(3);
	
	// Re-Enable Watchdog Timer
	wdt_enable();
	
	current_input = input_state_complete;
}


void fn_loop_test(void){
	struct MEASUREMENT temp_meas;
	//uint32_t timer_count;
	uint8_t i;
	
	if (state_change){
		cal_disp_message();
		//  Set up initial settings
		cal_init();
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
			if (!isLaserOn()){
				// Turn on rangefinder module and laser
				rangefinder_on_off(true);
				laser_on_off(true);
			}else{
				//  Take measurement and process data
				full_measurement(&temp_meas, options.shot_delay);
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
		default:
			break;
	}
	last_input = input_none;
	

	glcd_clear_buffer();
	
	// Display Header
	sprintf(display_str, "Loop Test:");
	glcd_tiny_draw_string(0,0,display_str);
	
	sprintf(display_str, "Segments: %d", nPoints);
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




void fn_dist_calibration(void){
	struct MEASUREMENT temp_meas;
	
	//uint32_t timer_count;
	uint8_t k;
	
	if (state_change){
		cal_disp_message();
		//  Set up initial settings
		cal_init();
		last_input = input_none;
	}
	
	// Button Handler
	switch(last_input){
		case input_button1:
			//  Calibration Done button
			if (cal_getGroupPoints() >= SHOT_SIZE){
				//  Turn off rangefinder
				rangefinder_on_off(false);
				current_input = input_state_complete;
			}
			break;
		case input_buttonE:
			//  Set laser and then take measurement
			if (!isLaserOn()){
				// Turn on rangefinder module and laser
				rangefinder_on_off(true);
				laser_on_off(true);
			}else{
				//  Take measurement and process data
				full_measurement(&temp_meas, options.shot_delay);
				//  Turn laser module off
				rangefinder_on_off(true);
			}
			break;
		case input_button4:
			// Handled by state machine
			//  Exit from routine
			break;
		default:
			break;
	}
	last_input = input_none;
	

	glcd_clear_buffer();
	
	// Display Header
	sprintf(display_str, "Calibration Mode");
	glcd_tiny_draw_string(0,0,display_str);
	if (options.current_unit_dist == feet){
		sprintf(display_str, " Target %.1f feet",DIST_CAL_SETPOINT_FT);
	}else{
		sprintf(display_str, " Target %.1f meters.",DIST_CAL_SETPOINT_MT);
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
	if (cal_getGroupPoints() >= SHOT_SIZE){
		sprintf(display_str, "Calibration      Done");
		glcd_tiny_draw_string(0,0,display_str);
	}
	sprintf(display_str, "Abort");
	glcd_tiny_draw_string(97,7,display_str);
		
	glcd_write();
}


void cal_disp_message(void){
	wdt_disable();
	
	glcd_clear_buffer();
	
	switch(current_state){
		case st_inc_azm_full_calibration:
			sprintf(display_str, "Azm/Inc Calibration:");
			glcd_tiny_draw_string(0,0,display_str);
			sprintf(display_str, "Take Uni-Directional Groups of 4 Shots    while rotating       instrument. Only last4 shots of each groupwill be saved");
			glcd_tiny_draw_string(0,1,display_str);
			break;
		case st_dist_calibration:
			sprintf(display_str, "Distance Calibration:");
			glcd_tiny_draw_string(0,0,display_str);
			sprintf(display_str, "Place a target at");
			glcd_tiny_draw_string(0,1,display_str);
			if (options.current_unit_dist == feet){
				sprintf(display_str, "  %.1f feet.", DIST_CAL_SETPOINT_FT);
			}else{
				sprintf(display_str, "  %.1f meters.", DIST_CAL_SETPOINT_MT);
			}
			glcd_tiny_draw_string(0,2,display_str);
			sprintf(display_str, "Take min. 4 shots in");
			glcd_tiny_draw_string(0,3,display_str);
			sprintf(display_str, "Multiple Orientations");
			glcd_tiny_draw_string(0,4,display_str);
			sprintf(display_str, "Only last 4 used.");
			glcd_tiny_draw_string(0,5,display_str);
			break;
		case st_loop_test:
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
			break;
		case st_azm_quick_calibration:
			sprintf(display_str, "Azm Quick Calibration");
			glcd_tiny_draw_string(0,0,display_str);
			sprintf(display_str, "Rotate device slowly ");
			glcd_tiny_draw_string(0,2,display_str);
			sprintf(display_str, "to cover all points  ");
			glcd_tiny_draw_string(0,3,display_str);
			sprintf(display_str, "in grid.  ");
			glcd_tiny_draw_string(0,4,display_str);
			break;
		default:
			sprintf(display_str, "Hello World");
			glcd_tiny_draw_string(0,2,display_str);
			break;
	}
	
	

	sprintf(display_str, "Press any button...");
	glcd_tiny_draw_string(10,7,display_str);
	glcd_write();
	while((current_input == input_none) || (current_input == input_1sec));//hold here until an input
	current_input = input_none;
	wdt_enable();
}

void  fn_disp_cal_report(void){
	#define maxPages 4
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
		default:
			break;
	}

	glcd_clear_buffer();
	sprintf(display_str, "Calibration Report:");
	glcd_tiny_draw_string(0,0,display_str);
	
	switch(pageView){
		///////////////////////// AZM and INC Report
		case 1:
			//// Page 1			
			sprintf(display_str, "Inclination & Azimuth");
			glcd_tiny_draw_string(0,1,display_str);
			sprintf(display_str,"20%02x.%02x.%02x@%02x:%02x:%02x",
				cal_report.time_inc_azm.year, cal_report.time_inc_azm.month, cal_report.time_inc_azm.date,
				cal_report.time_inc_azm.hours, cal_report.time_inc_azm.minutes, cal_report.time_inc_azm.seconds);
			glcd_tiny_draw_string(0,2,display_str);
			sprintf(display_str,"4-Point Groups: %d", cal_report.groups);
			glcd_tiny_draw_string(0,3,display_str);
			sprintf(display_str,"Azm Stdev: %.3f", cal_report.azm_angle_err);
			glcd_tiny_draw_string(0,5,display_str);
			glcd_draw_circle(98, 41, 1, BLACK);// Draw degree symbol
			sprintf(display_str,"Inc Stdev: %.3f", cal_report.inc_angle_err);
			glcd_tiny_draw_string(0,6,display_str);
			glcd_draw_circle(98, 49, 1, BLACK);// Draw degree symbol
			if (options.current_unit_temp==fahrenheit){
				sprintf(display_str,"Temp: %0.1f F", cal_report.time_inc_azm.temperatureF);
			}else{
				sprintf(display_str,"Temp: %0.1f C", cal_report.time_inc_azm.temperatureC);
			}
			glcd_tiny_draw_string(0,7,display_str);
			break;
		case 2:
			//// Page 2
			sprintf(display_str, "Inclination:");
			glcd_tiny_draw_string(0,1,display_str);
			sprintf(display_str,"20%02x.%02x.%02x@%02x:%02x:%02x",
			cal_report.time_inc_azm.year, cal_report.time_inc_azm.month, cal_report.time_inc_azm.date,
			cal_report.time_inc_azm.hours, cal_report.time_inc_azm.minutes, cal_report.time_inc_azm.seconds);
			glcd_tiny_draw_string(0,2,display_str);
			// Sensor Disparity
			sprintf(display_str,"A1-A2 Delta X,Y,Z %%");
			glcd_tiny_draw_string(0,3,display_str);
			sprintf(display_str,"%.3f, %.3f, %.3f",
				cal_report.disp_stdev_acc[0]*100, cal_report.disp_stdev_acc[1]*100, cal_report.disp_stdev_acc[2]*100);
			glcd_tiny_draw_string(0,4,display_str);
			//  Magnitude Error			
			sprintf(display_str,"Magnitude Error %%");
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str,"A1:%.3f A2:%.3f", cal_report.mag_stdev_a1*100, cal_report.mag_stdev_a2*100);
			glcd_tiny_draw_string(0,6,display_str);
			break;
		case 3:
			//// Page 3
			sprintf(display_str, "Azimuth");
			glcd_tiny_draw_string(0,1,display_str);
			sprintf(display_str,"20%02x.%02x.%02x@%02x:%02x:%02x",
			cal_report.time_quick_azm.year, cal_report.time_quick_azm.month, cal_report.time_quick_azm.date,
			cal_report.time_quick_azm.hours, cal_report.time_quick_azm.minutes, cal_report.time_quick_azm.seconds);
			glcd_tiny_draw_string(0,2,display_str);
			// Sensor Disparity
			sprintf(display_str,"M1-M2 Delta X,Y,Z %%");
			glcd_tiny_draw_string(0,3,display_str);
			sprintf(display_str,"%.3f, %.3f, %.3f",
			cal_report.disp_stdev_comp[0]*100, cal_report.disp_stdev_comp[1]*100, cal_report.disp_stdev_comp[2]*100);
			glcd_tiny_draw_string(0,4,display_str);
			//  Magnitude Error
			sprintf(display_str,"Magnitude Error %%");
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str,"M1:%.3f M2:%.3f", cal_report.mag_stdev_m1*100, cal_report.mag_stdev_m2*100);
			glcd_tiny_draw_string(0,6,display_str);
			break;
		//////////////////////// Distance Report
		case 4:
			sprintf(display_str, "Distance");
			glcd_tiny_draw_string(0,1,display_str);
			sprintf(display_str,"20%02x.%02x.%02x@%02x:%02x:%02x",
				cal_report.time_rangeFinder.year, cal_report.time_rangeFinder.month, cal_report.time_rangeFinder.date,
				cal_report.time_rangeFinder.hours, cal_report.time_rangeFinder.minutes, cal_report.time_rangeFinder.seconds);
			glcd_tiny_draw_string(0,2,display_str);
			sprintf(display_str,"Rangefinder Offset:");
			glcd_tiny_draw_string(0,4,display_str);
			sprintf(display_str,"  %.4f meters", dist_calst.dist_offset);
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str,"  %.4f feet", dist_calst.dist_offset*MT2FT);
			glcd_tiny_draw_string(0,6,display_str);
			
		break;
	}
	
	// Display soft keys
	switch (pageView){
		case 1:
			drawSoftKeys("","",">","Exit");
			break;
		case maxPages:
			drawSoftKeys("","<","","Exit");
			break;
		default:
			drawSoftKeys("","<",">","Exit");
			
	}
		
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
	
	sprintf(display_str,"Segments: %d",nPoints);
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



void fn_measure(void){
	// increment data buffer index
	data_buf_ind = data_buf_ind+1;
	if (data_buf_ind >= NBUFF_MEAS){data_buf_ind = 0;}
	// Increment reference counter
	data_ref = data_ref+1;
	if (data_ref>= 999){data_ref = 1;}
	data_buf[data_buf_ind].index_ref = data_ref;
	//  Take measurement	
	full_measurement(&data_buf[data_buf_ind], options.shot_delay);
	//  Save data to SD card
	save_measurement(&data_buf[data_buf_ind]);
	//  Turn laser module off
	rangefinder_on_off(false);
	//  Send measurement over bluetooth
	BLE_sendMeas(&data_buf[data_buf_ind]);
	
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
	if(temp_index>=NBUFF_MEAS){temp_index = 0;}
	
	
	quick_measurement(&data_buf[temp_index]);
	
	
	print_data_screen();

}


void fn_error_info(void){
	uint8_t i;
	static uint8_t shot_list[NBUFF_MEAS+1];
	static uint8_t shot_list_ind;
	static uint8_t nshots;
	uint8_t temp_buf_ind;

	
	if (state_change){ // Perform first time entering function
		// Build list of indexes of bad shots
		shot_list_ind = 0;
		nshots = 0;
		temp_buf_ind = data_buf_ind;
		for (i=0;i<NBUFF_MEAS;i++){
			if (data_buf[temp_buf_ind].num_errors>0){
				//  Add shot to list
				shot_list[shot_list_ind] = temp_buf_ind;
				shot_list_ind++;
				nshots++;
			}
			
			if (temp_buf_ind == 0){ temp_buf_ind = NBUFF_MEAS-1;}//  Buffer wrap-around
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
		default:
			break;
	}
	
	
	// Display
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"Error Information:");
	glcd_tiny_draw_string(0,0,display_str);
	// Display soft keys
	if(shot_list_ind==0){
		drawSoftKeys("","",">","Back");
	}else if (shot_list_ind>=nshots){
		drawSoftKeys("","<","","Back");
	}else{
		drawSoftKeys("","<",">","Back");
	}
	
	if(nshots<= shot_list_ind){//  display null message
		sprintf(display_str,"No Additional Errors");
		glcd_tiny_draw_string(8,1,display_str);
		sprintf(display_str,"to Report in Last");
		glcd_tiny_draw_string(8,2,display_str);
		sprintf(display_str,"%d Measurements", NBUFF_MEAS);
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
		default:
			break;
	}
	
	//  Title
	sprintf(display_str, "Menu:");
	glcd_tiny_draw_string(0,0,display_str);
	//print soft key text
	drawSoftKeys("Enter","<",">","Back");
	
	//  Write menu entries
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
	
	
	//debug//////////////
	#if DEBUG_DISPLAY==true
	debug1++;
	sprintf(display_str,"debug1:%d", debug1);
	glcd_tiny_draw_string(0,3,display_str);
	sprintf(display_str,"debug2:%d", debug2);
	glcd_tiny_draw_string(0,4,display_str);
	sprintf(display_str,"debug3:%d", debug3);
	glcd_tiny_draw_string(0,5,display_str);
	sprintf(display_str,"debug4:%d", debug4);
	glcd_tiny_draw_string(0,6,display_str);
	#endif
	////////////////////////
	
	
	
	glcd_write();
	
	
	
	
	
}


void fn_menu_debug(void){
	//  Set initial conditions
	if (state_change) {
		cur_Y = 2;
		cur_Y_low = 2;
		cur_Y_high = 6;
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
			}else if(cur_Y == 5){
				//  Charger Debug
				current_input = input_reprocess_inc_azm_cal;
			}else if(cur_Y == 6){
				//  Charger Debug
				current_input = input_reprocess_azm_quick_cal;
			}
		default:
			break;
	}
	
	// Display
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"Debug Menu:");
	glcd_tiny_draw_string(0,0,display_str);
	
	//Display Options
	sprintf(display_str, "Sensor Raw Data");
	glcd_tiny_draw_string(5, 2, display_str);
	sprintf(display_str, "Backlight Manual");
	glcd_tiny_draw_string(5, 3, display_str);
	sprintf(display_str, "Charger Info");
	glcd_tiny_draw_string(5, 4, display_str);
	sprintf(display_str, "Reprocess Full Cal");
	glcd_tiny_draw_string(5, 5, display_str);
	sprintf(display_str, "Reprocess AZM Cal");
	glcd_tiny_draw_string(5, 6, display_str);
	
	// Display soft keys
	drawSoftKeys("Enter","<",">","Back");
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(0, cur_Y,display_str);
	
	glcd_write();
	
	
}



void fn_menu_cal(void){
	//  Set initial conditions
	if (state_change) {
		cur_Y = 1;
		cur_Y_low = 1;
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
			switch (cur_Y){
				case 1:
					//  Display Report
					current_input = input_disp_cal_report;
					break;
				case 2:
					// Perform Loop Test
					current_input = input_loop_test;
					break;
				case 3:
					//  Perform Compass Quick Cal
					current_input = input_azm_quick_calibration;
					break;
				case 4:
					//  Accelerometer and Compass Calibration
					current_input = input_inc_azm_full_calibration;
					break;
				case 5:
					//  Distance Calibration
					current_input = input_dist_calibration;
					break;
					
				
				
				
			}
			if (cur_Y==2){
				
			}else if(cur_Y == 3){
				
				
			} else if (cur_Y==4){
				
			} else if (cur_Y==5){
				
			}
		default:
			break;
	}
	
	// Display
	glcd_clear_buffer();
		//  Display Title
	sprintf(display_str,"Calibration:");
	glcd_tiny_draw_string(0,0,display_str);
	
	//Display Options
	sprintf(display_str, "Display Report");
	glcd_tiny_draw_string(5, 1, display_str);
	sprintf(display_str, "Loop Test");
	glcd_tiny_draw_string(5, 2, display_str);
	sprintf(display_str, "CAL: Quick AZM");
	glcd_tiny_draw_string(5, 3, display_str);
	sprintf(display_str,"CAL: Full INC&AZM");
	glcd_tiny_draw_string(5, 4, display_str);
	sprintf(display_str,"CAL: Range-finder");
	glcd_tiny_draw_string(5, 5, display_str);
	
	// Display soft keys
	drawSoftKeys("Enter","<",">","Back");

	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(0, cur_Y,display_str);
	
	glcd_write();
	
}

void fn_debug_backlight(void){
	static char colorChar;
	static struct BACKLIGHT_COLOR *colorPtr;
	//  Set initial conditions
	if (state_change) {
		cur_Y = 2;
		cur_Y_low = 2;
		cur_Y_high = 4;
		options.backlight_setting.colorRef = 0;//  0 is custom Color
		backlightOn(&options.backlight_setting);
		//colorPtr = backlightCustomAdjust(0, 0);
	}	
	
	switch(cur_Y){
		case 2:
			colorChar = 'r';
			break;
		case 3:
			colorChar = 'g';
			break;
		case 4:
			colorChar = 'b';
			break;

		default:
			colorChar = 'r';
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
			colorPtr = backlightCustomAdjust(colorChar, 1);
			break;
		case input_button3:
			colorPtr = backlightCustomAdjust(colorChar, -1);
			break;
		default:
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
	
	// Display soft keys
	drawSoftKeys("","Up","Down","");
	
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
					if (options.shot_delay>SHOT_DELAY_MAX){options.shot_delay = 0;}
					save_user_settings();
					break;
				case 4:
					// Charge Current
					if (options.chargeCurrent == 500){ options.chargeCurrent = 100;}
					else{options.chargeCurrent = 500;}
					setChargeCurrent(options.chargeCurrent);
					save_user_settings();
					break;
				case 5:
					// Charge Current
					adjustErrorSensitivity();
					save_user_settings();
					break;	
				case 6:
					// Backlight Color
					backlightColorToggle(&options.backlight_setting);					
					save_user_settings();
					break;
				case 7:
					// Backlight Color
					backlightLevelToggle(&options.backlight_setting);
					save_user_settings();
					break;
				default:
					break;
			}
		default:
		break;	

	}
	
	// Display
	glcd_clear_buffer();
	
	//Display Options
	if (options.current_unit_dist==feet){
		sprintf(display_str, "Dist: Feet");
	}else{
		sprintf(display_str, "Dist: Meters");
	}
	glcd_tiny_draw_string(5, 1, display_str);
	if (options.current_unit_temp==fahrenheit){
		sprintf(display_str, "Temp: Fahrenheit");
		}else{
		sprintf(display_str, "Temp: Celsius");
	}
	glcd_tiny_draw_string(5, 2, display_str);
	sprintf(display_str,"Shot Delay: %d sec",options.shot_delay);
	glcd_tiny_draw_string(5, 3, display_str);
	sprintf(display_str,"Charge Curr: %dmA",options.chargeCurrent);
	glcd_tiny_draw_string(5, 4, display_str);
	sprintf(display_str,"Err Sens: %0.2f deg", options.errorSensitivity);
	glcd_tiny_draw_string(5, 5, display_str);
	sprintf(display_str,"BL Color: %s", backlightGetCurrentColor(&options.backlight_setting));
	glcd_tiny_draw_string(5, 6, display_str);
	sprintf(display_str,"BL Level: %d", options.backlight_setting.brightness);
	glcd_tiny_draw_string(5, 7, display_str);
	
	//  Display Title
	sprintf(display_str,"Options:");
	glcd_tiny_draw_string(0,0,display_str);
	
	// Display soft keys
	drawSoftKeys("Adjust","<",">","Back");
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(0, cur_Y,display_str);
		
	
	//debug//////////////
	#if DEBUG_DISPLAY==true
	debug1++;
	sprintf(display_str,"debug1:%d", debug1);
	glcd_tiny_draw_string(0,3,display_str);
	sprintf(display_str,"debug2:%d", debug2);
	glcd_tiny_draw_string(0,4,display_str);
	sprintf(display_str,"debug3:%d", debug3);
	glcd_tiny_draw_string(0,5,display_str);
	sprintf(display_str,"debug4:%d", debug4);
	glcd_tiny_draw_string(0,6,display_str);
	#endif
	////////////////////////	
		
		
	glcd_write();


	
}



void fn_set_bluetooth(void){
	char str_on[] = "On";
	char str_off[] = "Off";
	char *str_ptr;
	bool pinState;
	
	if (state_change) {
		cur_Y=2;
		cur_Y_low=2;
		cur_Y_high=6;
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
				pinState = ioport_get_pin_level(BLE_autorun);
				ioport_set_pin_level(BLE_autorun, !pinState);
			} else if (cur_Y==3){
				pinState = ioport_get_pin_level(BLE_reset);
				ioport_set_pin_level(BLE_reset, !pinState);
			} else if (cur_Y==4){
				pinState = ioport_get_pin_level(BLE_ota);
				ioport_set_pin_level(BLE_ota, !pinState);		
			} else if (cur_Y==5){
				if (isBleCommEnabled()){
					usart_disable(&usart_BLE);
					BLE_usart_isolate();
				}else{
					configure_usart_BLE();
				}
				
			}else if (cur_Y==6){
				pinState = ioport_get_pin_level(BLE_COMMAND_MODE);
				ioport_set_pin_level(BLE_COMMAND_MODE, !pinState);
			}
		default:
			break;
	}


	
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"Bluetooth:");
	glcd_tiny_draw_string(0,0,display_str);
	
	//Display Options
	sprintf(display_str,"AutoRun On/Off");
	glcd_tiny_draw_string(25, 2, display_str);
	sprintf(display_str,"Reset On/Off");
	glcd_tiny_draw_string(25, 3, display_str);
	sprintf(display_str,"OTA On/Off");
	glcd_tiny_draw_string(25, 4, display_str);
	sprintf(display_str,"MC UART On/Off");
	glcd_tiny_draw_string(25, 5, display_str);
	sprintf(display_str,"CMD MODE");
	glcd_tiny_draw_string(25, 6, display_str);
	
	// Display soft keys
	drawSoftKeys("Adjust","<",">","Back");
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(18, cur_Y,display_str);
			
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
	if (isBleCommEnabled()){ str_ptr = str_on;}
	else{str_ptr = str_off;}
	glcd_tiny_draw_string(0, 5,str_ptr);
	if (ioport_get_pin_level(BLE_COMMAND_MODE)){ str_ptr = str_on;}
	else{str_ptr = str_off;}
	glcd_tiny_draw_string(0, 6,str_ptr);
	

	
	glcd_write();
	
}


void fn_set_clock(void){
	uint8_t i, unitMax, unitMin;
	uint8_t *unitPtr;
	// Used for setting clock
	typedef struct {
		uint8_t y_pos;//cursor y position
		uint8_t min;  //unit max
		uint8_t max;  //unit min
		uint8_t *ptr; //unit location
	} CLOCK_SETTING;
	
	CLOCK_SETTING clock_table[] = {
		{1,	0,	99, &temp_time.year},
		{2,	1,	12, &temp_time.month},
		{3,	1,	31, &temp_time.date},
		{4,	0,	24, &temp_time.hours},
		{5,	0,	59, &temp_time.minutes},
		{6,	0,	59, &temp_time.seconds}
	};
	
	
	if (state_change) {
		cur_Y = 1;
		cur_Y_low = 1;
		cur_Y_high = 6;
		get_time();
		memcpy(&temp_time,&current_time,sizeof(current_time));	
	}
	
	for (i=0;i<6;i++){
		if (cur_Y==clock_table[i].y_pos){
			unitMax = clock_table[i].max;
			unitMin = clock_table[i].min;
			unitPtr = clock_table[i].ptr;
			break;
		}
	}
	
	
	
	switch(last_input){
		case input_button2:
			*unitPtr = incBcdData(*unitPtr, 1, unitMin, unitMax);
			break;
		case input_button3:
			*unitPtr = incBcdData(*unitPtr, -1, unitMin, unitMax);
			break;
		case input_button1:
			if(cur_Y >= cur_Y_high){
				set_time();
				current_input = input_state_complete;
				}
			else{++cur_Y;}
			break;
		default:
			break;
			//  input_button4 aborts t
	}
	
	glcd_clear_buffer();
	
	sprintf(display_str,"Set Clock:");
	glcd_tiny_draw_string(0,0,display_str);
	sprintf(display_str,"Year:   20%02x", temp_time.year);
	glcd_tiny_draw_string(10,1,display_str);
	sprintf(display_str,"Month:  %02x", temp_time.month);
	glcd_tiny_draw_string(10,2,display_str);
	sprintf(display_str,"Date:   %02x", temp_time.date);
	glcd_tiny_draw_string(10,3,display_str);
	sprintf(display_str,"Hour:   %02x", temp_time.hours);
	glcd_tiny_draw_string(10,4,display_str);
	sprintf(display_str,"Minute: %02x", temp_time.minutes);
	glcd_tiny_draw_string(10,5,display_str);
	sprintf(display_str,"Second: %02x", temp_time.seconds);
	glcd_tiny_draw_string(10,6,display_str);
	
	// Display soft keys
	drawSoftKeys("Next","+","-","Cancel");
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(1, cur_Y,display_str);
	
	glcd_write();
		
}




void fn_main_display(void){
	
	print_data_screen();
	
	
	//Handle Button Inputs
	if(last_input==input_button2){
		backlightPlus(&options.backlight_setting);
		save_user_settings();
	}else if(last_input==input_button3){
		backlightMinus(&options.backlight_setting);
			save_user_settings();
	}
	
}

void print_data_screen(void){
	static bool flipper;
	
	get_time();
	isCharging = getChargerStatus();
	
	glcd_clear_buffer();
	
	if (options.current_unit_temp == fahrenheit){
		sprintf(display_str,"T:%4.1fF", current_time.temperatureF);
	}else{
		sprintf(display_str,"T:%0.1fC", current_time.temperatureC);
	}
	
	glcd_tiny_draw_string(86,7,display_str);
	
	sprintf(display_str,"%02x:%02x:%02x", current_time.hours, current_time.minutes, current_time.seconds);
	glcd_tiny_draw_string(0,7,display_str);


	
	//  Draw Charge Status	
	if (isCharging){
		flipper = !flipper;
		//if (flipper){
		//	flipper = false;
		//}else{
		//	flipper = true;
		//}
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
			temp_index = NBUFF_MEAS+temp_index;
			}else if(temp_index>=NBUFF_MEAS){
			temp_index = temp_index-NBUFF_MEAS;
		}
		//print lines
		if ((temp_ref)>0){
			//Adjust line spacing for grid lines
			if(i<2){y_temp=y2+10*i;}
			else {y_temp=y2+9*i;	}
			if((current_state==st_main_display)||(i>0)){//do not print reference and distance for active reading
				sprintf(display_str, "%d", data_buf[temp_index].index_ref);//reference
				glcd_draw_string_xy(x1, y_temp, display_str);
				sprintf(display_str, "%.1f", data_buf[temp_index].distCal);//distance
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
	#if DEBUG_DISPLAY==true
	debug1++;
	sprintf(display_str,"debug1:%d", debug1);
	glcd_tiny_draw_string(0,3,display_str);
	sprintf(display_str,"debug2:%d", debug2);
	glcd_tiny_draw_string(0,4,display_str);
	sprintf(display_str,"debug3:%d", debug3);
	glcd_tiny_draw_string(0,5,display_str);
	sprintf(display_str,"debug4:%d", debug4);
	glcd_tiny_draw_string(0,6,display_str);
	#endif
	////////////////////////

	
	glcd_write();
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
	//UART Pins
	//ioport_set_pin_dir(MCU_RTS1, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(MCU_RTS1, false);
	//ioport_set_pin_dir(MCU_CTS1, IOPORT_DIR_INPUT);
	//BLE pins
	//ioport_set_pin_dir(BLE_ota, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(BLE_ota, false);// low to disable programming over BLE
	//ioport_set_pin_dir(BLE_autorun, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(BLE_autorun, true);//low for autorun enabled, high for development mode
	//ioport_set_pin_dir(BLE_reset, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(BLE_reset, false); //low, hold in reset
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
	//ioport_set_pin_dir(MCU_TX1, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(MCU_TX1, false);
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
	//ioport_set_pin_dir(BLE_autorun, IOPORT_DIR_INPUT);
	//ioport_set_pin_dir(BLE_ota, IOPORT_DIR_INPUT);
	//ioport_set_pin_dir(BLE_SS, IOPORT_DIR_INPUT);
	//ioport_set_pin_level(BLE_reset, false);
	ioport_set_pin_level(BLE_COMMAND_MODE, false);
}




void configure_extint_channel(void)
{
	struct extint_chan_conf config_extint_chan;

	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING;
	config_extint_chan.filter_input_signal  = false;
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
	static uint32_t last_time_ms;
	uint32_t current_time_ms;
	enum INPUT tempInput;
	
	//cpu_irq_disable();
	
	
	current_time_ms = getCurrentMs(); 
	
	

	switch (extint_get_current_channel()){
		case 5:
			tempInput = externalButtonRoutine(!ioport_get_pin_level(buttonE), current_time_ms);
			break;
		case 7:
			tempInput = input_button4;
			break;
		case 6:
			tempInput = input_button3;
			break;
		case 4:
			tempInput = input_button2;
			break;
		case 9:
			tempInput = input_button1;
			break;
		default:
			tempInput = input_none;
			break;
	}// End switch for each input type
	
	
	
	//  Debounce Function
	if(tempInput != input_none){
		if((current_time_ms-last_time_ms)>DEBOUNCE_MS){
			last_time_ms = current_time_ms;	
			current_input = tempInput;
		}
	}
	//cpu_irq_enable();
	
	
}

enum INPUT externalButtonRoutine(bool buttonOn, uint32_t current_time_ms){
	// Button External
	// Special Routines for External Button
	// If held down for less than X seconds, provides normal input upon release
	// If held down for more than X seconds, a separate interrupt routine provides powerdown input
	// When in powerdown state, 3 quick clicks through a separate interrupt routine provides powerup input
	static uint32_t last_time_ms;
	
	static uint8_t click_counter=0;
	
	
	switch (current_state){
		case st_powerup:
			//  Ignore external button inputs during powerup
			return input_none;
			break;
		case st_powerdown:
			//  Special Routine to wake up from sleep
			
			//  Debounce function
			//  Lower debounce time than normal
			if ((current_time_ms-last_time_ms)<DEBOUNCE_MS_QUICK3){
				return input_none;
			}
			
			//  Find out if click counter will be incremented
			if( (current_time_ms-last_time_ms)<QUICK3_MS){
				//  Clicked within necessary time interval, add to click counter.
				click_counter++;
			}else{
				//  Not fast enough, record as first click
				click_counter = 1;
			}
			last_time_ms = current_time_ms;
			
			
			//  See if enough clicks have occurred
			if (click_counter>=3){
				click_counter = 0;
				return input_wakeup;
				
			}else{
				return input_none;
			}			
			break;
		default:
			if (buttonOn){
				//  Trigger on if button is pressed
				if(!buttonE_triggered){
					buttonE_triggered=true;
					//trigger timer
					timerStartExt();
				}
				return input_none;
			}else{
				//  Releaed in a short amount of time, normal input
				buttonE_triggered=false;
				timerStopExt();
				return input_buttonE;
			}

	}//  End Switch Statement

	
		
}



void fn_powerdown(void){

	if (state_change){
		// Disable watchdog timer
		wdt_disable();
		//  Switch-Over to low power internal clock
		ext_osc_onoff(false);
		//  Put hardware in low-power state
		disable_comms();
		config_pins_powerdown();		
		ioport_set_pin_level(V2_enable, false);//disable V2 power supply		
		//configure_timers(st_powerdown);//Disable TC	
		udc_stop();// disable USB
		powerdown_timer_1s();
		powerdown_timer_ExtLong();
		
		//  Setup GCLK 0 for low power 32 khz
		mainClockPowerdown();
		
	};	
	
	
	sleepmgr_sleep(SLEEPMGR_STANDBY);

	
}

void fn_powerup(void){
	config_pins_powerup();
	delay_ms(100);
	
	//  Setup GCLK 0 for 48MHZ
	mainClockPowerup();
	//  Various setups
	enable_comms();
	
	
	load_user_settings();//  Needed for backlight setting
	backlightOn(&options.backlight_setting);
	configure_extint_channel();
	configure_extint_callbacks();
	setup_accel(&slave_acc1);
	setup_accel(&slave_acc2);
	setup_mag(&slave_mag1);
	setup_mag(&slave_mag2);
	rangefinder_on_off(false);	
	
	//  Turn on external clock and take it as a source
	ext_osc_onoff(true);
	delay_ms(10);	

	//  Initialize LCD controller
	delay_ms(500);  //  Delay requested by LCD controller
	glcd_init();
	glcd_tiny_set_font(Font5x7,5,7,32,127);//  All font in "tiny" mode
	
	//  Setup background timers
	wdt_enable();
	configure_timer_1s();
	configure_timer_ExtLong();
	
	//  Bluetooth Init
	//BLE_init();
	
	
	//  Startup USB mass storage
	system_interrupt_enable_global();	
	irq_initialize_vectors();
	cpu_irq_enable();
	configure_SD();
	udc_start();
	
	
	
	//  Set initial conditions for state machine
	buttonE_triggered=false;//  In case button was pressed again during powerup
	current_input = input_state_complete;
	
	
	
}



void getDefaultOptions(struct OPTIONS *optionptr){
	
	optionptr->shot_delay = 0;//seconds
	optionptr->current_unit_temp = celsius;
	optionptr->current_unit_dist = meters;
	optionptr->chargeCurrent = 100;//mA
	optionptr->errorSensitivity = 1;
	optionptr->backlight_setting.colorRef = 1;//white
	optionptr->backlight_setting.brightness = 3;
	optionptr->SerialNumber = 0;
	optionptr->Settings_Initialized_Key = 0xC3;//  Indicator that settings have been initialized

	
}




void msc_notify_trans(void){
	
	//current_input=input_usb_transaction;
	usb_transaction_requested = true;
}


bool my_callback_msc_enable(void)
{
	my_flag_autorize_msc_transfert = true;
	return true;
}
void my_callback_msc_disable(void)
{
	my_flag_autorize_msc_transfert = false;
}





