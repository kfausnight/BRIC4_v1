
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
volatile enum STATE current_state, last_state; //  Current State
volatile bool state_change;
//  Status Variables
volatile bool isCharging = false;//  Variable to track when plugged in and charging
volatile bool SD_WriteLockout = false;  //  Locks out internal write processes when USB connected to computer
volatile bool buttonE_triggered = false; // Variable to track when external button is triggered
char BleClientMAC[20];//  Connected Device (client) MAC Address
char BleDeviceMAC[20]; //  This device (BRIC4) MAC address
char BleDeviceName[20]; // BRIC4 device name (e.g. "BRIC4_0039")
char BleCommandQueue[20];//  Queued BLE command
//  Options structure
struct OPTIONS options;
//  Measurement Tracker
struct BLE_SYNC_TRACKER bleSyncTracker;
//  Time structures
struct TIME current_time;
//  Temperature
float currentTempC;
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
volatile char laserDebugBuff[DEBUG_BUFFER_LENGTH];
volatile uint32_t laserDebugBuffIndex = 0;
char *laserDebugBuffPtr;
volatile char debugBuff[DEBUG_BUFFER_LENGTH];
volatile uint32_t debugBuffIndex = 0;
char *debugBuffPtr;
char *bleBuffPtr;

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

// Measurement Display Data Buffers
struct MEASUREMENT measBuf[N_MEASBUF];
uint32_t measBufInd = 0;
uint32_t refIndex = 1;

//  USB funcations
bool usb_transaction_requested = false;

// Menu cursors
uint8_t curY, curY_off, curY_N;






typedef void (*stateFunction)(void);

typedef struct  
{
	stateFunction Function;
	enum STATE	genGOTO[N_GENERIC_INPUTS];
} STATE_INFO;

const STATE_INFO state_info[]= {
	[st_main_display].Function = fn_main_display,
	[st_main_display].genGOTO = {
		[input_button1] = st_menu1,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu1,
		[input_buttonE] = st_NULL,
		[input_pwrDown] = st_powerdown,
	},
	
	[st_scan].Function = fn_scan,
	[st_scan].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_NULL,
		[input_buttonE] = st_NULL,
		[input_pwrDown] = st_NULL,
	},
	
	
	[st_powerup].Function = fn_powerup,
	[st_powerup].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_NULL,
		[input_buttonE] = st_NULL,
		[input_pwrDown] = st_NULL,
	},
	
	[st_powerdown].Function = fn_powerdown,
	[st_powerdown].genGOTO = {
		[input_button1] = st_powerup,
		[input_button2] = st_powerup,
		[input_button3] = st_powerup,
		[input_button4] = st_powerup,
		[input_buttonE] = st_powerup,
		[input_pwrDown] = st_NULL,
	},
	
	
	[st_menu1].Function = fn_menu1,
	[st_menu1].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_main_display,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_powerdown,
	},
	
	
	[st_set_clock].Function = fn_set_clock,
	[st_set_clock].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu1,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_powerdown,
	},
	
	
	[st_menu_BLE].Function = fn_menu_BLE,
	[st_menu_BLE].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu1,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_NULL,
	},
	
	
	[st_debug_BLE].Function = fn_debug_BLE,
	[st_debug_BLE].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu_BLE,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_NULL,
	},
	
	
	[st_set_options].Function = fn_set_options,
	[st_set_options].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu1,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_powerdown,
	},
	
	
	[st_menu_cal].Function = fn_menu_cal,
	[st_menu_cal].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu1,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_powerdown,
	},
	
	
	[st_menu_debug].Function = fn_menu_debug,
	[st_menu_debug].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu1,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_powerdown,
	},
	
	[st_inc_azm_full_calibration].Function = fn_inc_azm_full_calibration,
	[st_inc_azm_full_calibration].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_main_display,
		[input_buttonE] = st_NULL,
		[input_pwrDown] = st_NULL,
	},
	
	[st_dist_calibration].Function = fn_dist_calibration,
	[st_dist_calibration].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_main_display,
		[input_buttonE] = st_NULL,
		[input_pwrDown] = st_NULL,
	},
	
	[st_azm_quick_calibration].Function = fn_azm_quick_calibration,
	[st_azm_quick_calibration].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_main_display,
		[input_buttonE] = st_NULL,
		[input_pwrDown] = st_NULL,
	},
	
	[st_process_inc_azm_full_cal].Function = fn_process_inc_azm_full_cal,
	[st_process_inc_azm_full_cal].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_NULL,
		[input_buttonE] = st_NULL,
		[input_pwrDown] = st_NULL,
	},
	
	[st_process_azm_quick_cal].Function = fn_process_azm_quick_cal,
	[st_process_azm_quick_cal].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_NULL,
		[input_buttonE] = st_NULL,
		[input_pwrDown] = st_NULL,
	},
	
	[st_disp_cal_report].Function = fn_disp_cal_report,
	[st_disp_cal_report].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_main_display,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_powerdown,
	},
	
	[st_loop_test].Function = fn_loop_test,
	[st_loop_test].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_main_display,
		[input_buttonE] = st_NULL,
		[input_pwrDown] = st_NULL,
	},
	
	[st_disp_loop_report].Function = fn_disp_loop_report,
	[st_disp_loop_report].genGOTO = {
		[input_button1] = st_main_display,
		[input_button2] = st_main_display,
		[input_button3] = st_main_display,
		[input_button4] = st_main_display,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_powerdown,
	},
	
	[st_debug_rawData].Function = fn_debug_rawData,
	[st_debug_rawData].genGOTO = {
		[input_button1] = st_menu1,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu1,
		[input_buttonE] = st_NULL,
		[input_pwrDown] = st_powerdown,
	},
	
	[st_debug_backlight].Function = fn_debug_backlight,
	[st_debug_backlight].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_NULL,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_powerdown,
	},
	
	[st_debug_charger].Function = fn_debug_charger,
	[st_debug_charger].genGOTO = {
		[input_button1] = st_menu1,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu_debug,
		[input_buttonE] = st_menu1,
		[input_pwrDown] = st_powerdown,
	},
	
	[st_firmware].Function = fn_firmware,
	[st_firmware].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu_debug,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_powerdown,
	},
	
	[st_error_info].Function = fn_error_info,
	[st_error_info].genGOTO = {
		[input_button1] = st_NULL,
		[input_button2] = st_NULL,
		[input_button3] = st_NULL,
		[input_button4] = st_menu1,
		[input_buttonE] = st_main_display,
		[input_pwrDown] = st_powerdown,
	},
	
	
};





int main (void)

  {
	system_init();	
	delay_init();	
	delay_ms(500);	
	
	
	configure_timer_counter();
	fn_powerup();
	
	delay_s(1);
	
	//EEPROM_test();
	load_user_settings();
	load_calibration();
	
	//  Battery Setup
	setupCharger();	
	setup_batt();
	
	sleepmgr_init();
	
	
	getSN();
	
	//BLE Setup
	BLE_init();
	
	//  Load Sync Tracker and any measurements on SD card
	load_sync_tracker();
	loadMeasBuffer();
	//  Initialize states and inputs
	current_state = st_main_display;
	last_state = st_NULL;
	current_input = input_1sec;
	last_input = input_none;
	
	while(1){

		do {
			//  Idle Loop
			//  Take care of any background processes and go back to sleep
			
			//  Break if a state change
			if (last_state!=current_state){break;}
			
				
			//  Handle any USB transactions
			UsbHandleTransactions();
			
			// Upload any outstanding measurements over BLE
			SyncDataBLE();
						
			//  Return to Sleep State
			if(current_input==input_none){
				if (current_state==st_powerdown){
					sleepmgr_sleep(SLEEPMGR_STANDBY);
				}else{
					sleepmgr_sleep(SLEEPMGR_IDLE);
				}
			}			
			debug1++;
		}while ((current_input==input_none)||
			(current_input == input_BLE_message)||
			(current_input == input_usb_transaction));
		
		debug2++;
		
		//  Pet Watchdog
		if(current_state!=st_powerdown){
			wdt_reset_count();
		}
		
		
		//  Handle any remote BLE commands
		if(current_input == input_BLE_command){
			BLE_remoteCommand(BleCommandQueue);
		}
		
		//  Get BLE Connection Status and client MAC address
		BLE_get_client_MAC();
		
		//  Determine if idle powerdown will be performed		
		idle_timeout();//Will produce input = input_powerdown if idle
		
		//  Laser Timeout
		laser_timeout();
		
		//  Battery Level Update
		BleUpdateBattLevel(); //  Update battery level characteristic if it changes
		if(!isCharging){SD_WriteLockout = false;}
				
		//  Determine if current input brings new state
		if (current_input<N_GENERIC_INPUTS){
			enum STATE st_temp;
			st_temp = state_info[current_state].genGOTO[current_input];
			if (st_temp!=st_NULL){
				current_state = st_temp;
			}
		}
		
		//  Catch jut in case state set to st_NULL
		if (current_state == st_NULL){current_state = last_state;}
		
		//  Flag a new change of state
		state_change = !(last_state==current_state);
		last_state = current_state;
		
		//  Reset inputs prior to performing state functions
		//  Reset last input; helps avoid buggy menus
		if (state_change){
			last_input = input_none;
		}else{
			last_input = current_input;
		}		
		current_input = input_none;
		
		//  Turn off laser in case it is accidentally left on from previous state
		if(state_change && isLaserOn()){
			rangefinder_on_off(false);
		}
		
		//  Perform function associated with current state		
		state_info[current_state].Function();
		
		/*
		if (current_state!= st_powerdown){
			sprintf(display_str,"                ");
			glcd_tiny_draw_string(0,5,display_str);
			sprintf(display_str,"%d, %d", debug1, debug2);
			glcd_tiny_draw_string(0,5,display_str);
			glcd_write();
			debug1 = 0;
		}
		*/
		
	}//End of main program while loop
}//end of main



void fn_scan(void){
	struct MEASUREMENT_FULL tempMeas;
	int dataRate;
	
	if (state_change){
		
		//  Display
		glcd_clear_buffer();
		sprintf(display_str,"Scan Mode:");
		glcd_tiny_draw_string(0,0,display_str);
		
		// Connected to
		glcd_tiny_draw_string(0, 1,"Streaming Data ");
		glcd_tiny_draw_string(0, 2,"over BLE");
		glcd_tiny_draw_string(0, 4,"Connected to:");
		glcd_tiny_draw_string(10, 5,BleClientMAC);
		
		glcd_write();
		
		//  Start Measurements
		laser_start_continuous();
		rxBufferLaserClear();
		
		
	}
	dataRate = 0;
	while (current_input==input_none){
		//   Read data
		full_measurement(&tempMeas, 0, measScan);
		//scan_measurement(&tempMeas);
		//  Send measurement over bluetooth
		if (tempMeas.distRaw!=0){
			//  Only send if distance data was valid (not 0)
			//  Send over BLE, save to SD, and increment reference
			//processMeasurement(&tempMeas);

			dataRate++;
		}
		
	}
	
	sprintf(display_str,"Meas Rate: %d hz   ", dataRate);
	glcd_tiny_draw_string(0, 7,display_str);
	glcd_write();
	
	if((current_input!=input_none)&&
	   (current_input!=input_1sec)&&
	   (current_input!=input_BLE_message)){
		//  Close up and return
		rangefinder_on_off(false);
		rxBufferBleClear();
		current_state = st_main_display;
		current_input= input_none;
		
	}
	
}


void fn_debug_charger(void){
	uint8_t addressList[] = {
		0x02,
		0x04,
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
	
	struct MEASUREMENT_FULL meas_debug;
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
	
	struct MEASUREMENT_FULL tempM;
	float xPos, yPos;
	static bool tracker[NBUFF];

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
		//if (nPoints>=10){
			//  Save all Calibration Raw Data
			cal_done(azm_quick);
			// Signal completion
			current_state = st_process_azm_quick_cal;
			break;
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
	current_state = st_disp_cal_report;
	
}


void fn_inc_azm_full_calibration(void){
	struct MEASUREMENT_FULL temp_meas;
	uint32_t i;
	
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
				current_state = st_process_inc_azm_full_cal;
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
				full_measurement(&temp_meas, options.shot_delay, measCal);
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
	float posX_A, posX_M;	
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
	uint8_t yCir;
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
	
	uint32_t  k, iter;
	int32_t g;
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
	for (k=0;k<MAX_BAD_GROUPS;k++){
		cal_report.groupRemoved[k] = 0;
		cal_report.groupRemovedSource[k] = 0;
		cal_report.groupRemovedImprovement[k] = 0;
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
		
		//  Evaluate calibration, find bad groups
		uint8_t badGroupSource;
		uint32_t badGroup = 0;
		float badGroupDelta = 0;
		badGroupSource = cal_findBadGroup(incErrArray, azmErrArray, &badGroup, &badGroupDelta);
		
		
		if (badGroupDelta>BAD_GROUP_THRESHOLD){
			//  Remove Group
			permGroupRemove[badGroup] = true;
			//  Log removal
			cal_report.groupRemoved[iter] = badGroup;
			cal_report.groupRemovedSource[iter] = badGroupSource;
			cal_report.groupRemovedImprovement[iter] = badGroupDelta;
			//  Print Status
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
	glcd_clear_buffer();
	sprintf(display_str, "Calibration Complete!");
	glcd_tiny_draw_string(0,2,display_str);
	glcd_write();
	delay_s(3);
	
	// Re-Enable Watchdog Timer
	wdt_enable();
	
	current_state = st_disp_cal_report;
}


void fn_loop_test(void){
	struct MEASUREMENT_FULL temp_meas;
	//uint32_t timer_count;
	
	if (state_change){
		cal_disp_message();
		//  Set up initial settings
		cal_init();
	}
	
	// Button Handler
	switch(last_input){
		case input_button1:
			//  Calibration Done button
			ioport_set_pin_level(laser_reset, false);
			current_state = st_disp_loop_report;
			break;
		case input_buttonE:
			//  Set laser and then take measurement
			if (!isLaserOn()){
				// Turn on rangefinder module and laser
				rangefinder_on_off(true);
				laser_on_off(true);
			}else{
				//  Take measurement and process data
				full_measurement(&temp_meas, options.shot_delay, measCal);
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
	struct MEASUREMENT_FULL temp_meas;
	
	//uint32_t timer_count;
	uint8_t k;
	
	if (state_change){
		cal_disp_message();
		//  Set up initial settings
		cal_init();
	}
	
	// Button Handler
	switch(last_input){
		case input_button1:
			//  Calibration Done button
			if (cal_getGroupPoints() >= SHOT_SIZE){
				//  Turn off rangefinder
				rangefinder_on_off(false);
				cal_done(rangeFinder);
				current_state = st_disp_cal_report;
			}
			break;
		case input_buttonE:
			//  Set laser and then take measurement
			if (!isLaserOn()){
				// Turn on rangefinder module and laser
				rangefinder_on_off(true);
				laser_on_off(true);
			}else{
				//  Take measurement
				full_measurement(&temp_meas, options.shot_delay, measCal);
				//  Turn laser module off
				rangefinder_on_off(true);
				//  Process point
				cal_add_dist(&temp_meas);
			}
			break;
		case input_button4:
			// Handled by state machine
			//  Exit from routine
			break;
		default:
			break;
	}
	

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
	
	//  Hold for any button
	wdt_disable();
	sprintf(display_str, "Press any button...");
	glcd_tiny_draw_string(10,7,display_str);
	glcd_write();
	current_input = input_none;
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
	
	//  Print Calibration data to glcd buffer
	disp_report(pageView);
	
	
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


void fn_error_info(void){
	uint32_t i;
	static uint8_t shot_list[N_MEASBUF+1];
	static uint8_t shot_list_ind;
	static uint8_t nshots;
	uint32_t temp_buf_ind;

	
	if (state_change){ // Perform first time entering function
		// Build list of indexes of bad shots
		shot_list_ind = 0;
		nshots = 0;
		temp_buf_ind = measBufInd;
		for (i=0;i<N_MEASBUF;i++){
			circBuffDec(&temp_buf_ind, N_MEASBUF);//  Backup to last reading
			if (measBuf[temp_buf_ind].errCode[0]>0){
				//  Add shot to list
				shot_list[shot_list_ind] = temp_buf_ind;
				shot_list_ind++;
				nshots++;
			}

		}
		
		shot_list_ind = 0;
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
		sprintf(display_str,"%d Measurements", N_MEASBUF);
		glcd_tiny_draw_string(8,3,display_str);
	}else{
		temp_buf_ind = shot_list[shot_list_ind];
		sprintf(display_str,"Measurement %d", measBuf[temp_buf_ind].refIndex);
		glcd_tiny_draw_string(0,1,display_str);
		for (i=0;i<MAX_ERRORS; i++){
			gen_err_message(display_str, &measBuf[temp_buf_ind], i);
			glcd_tiny_draw_string(0,i+2,display_str);
		}
		
	}
	
	glcd_write();
}


void fn_menu1(void){
	uint8_t i;
	const enum STATE gotoList[] = {st_set_options, 
		st_error_info, 
		st_menu_cal, 
		st_set_clock, 
		st_menu_BLE, 
		st_menu_debug};
	const char * labels[] = {
		"Options",
		"Error Info",
		"Calibration",
		"Set Clock",
		"Bluetooth",
		"Debug Menu",
	};
		
	if (state_change){
		curY = 0;
		curY_N = 6;
		curY_off = 1;
	}
	
	//  Button Handler
	curY = getCursor(last_input, curY, curY_N);
	if (last_input==input_button1){
		current_state = gotoList[curY];
	}
	
	
	glcd_clear_buffer();	
	//  Title
	sprintf(display_str, "Main Menu:");
	glcd_tiny_draw_string(0, 0,display_str);
	//print soft key text
	drawSoftKeys("Enter","<",">","Back");
	//print cursor
	sprintf(display_str, ">");
	glcd_tiny_draw_string(0, curY+curY_off,display_str);
	//  Draw Options
	for (i=0;i<curY_N;i++){
		glcd_tiny_draw_string(5,i+curY_off,labels[i]);
	}
	glcd_write();
	
	
	
	
	
}


void fn_menu_debug(void){
	uint8_t i;
	const enum STATE gotoList[] = {st_debug_rawData,
		 st_debug_backlight, 
		 st_debug_charger, 
		 st_process_inc_azm_full_cal,
		 st_process_azm_quick_cal,
		 st_firmware
		 };
		 
	const char * labels[] = {
		"Sensor Raw Data",
		"Backlight Manual",
		"Charger Info",
		"Reprocess Full Cal",
		"Reprocess AZM Cal",
		"Firmware"
	};
		 
	//  Set initial conditions	 
	if (state_change) {
		curY = 0;
		curY_N = 6;
		curY_off = 1;
	}
	
	// Button Handler
	curY = getCursor(last_input, curY, curY_N);
	if (last_input==input_button1){
		current_state = gotoList[curY];
	}
		
	// Display
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"Debug Menu:");
	glcd_tiny_draw_string(0,0,display_str);
	// Display soft keys
	drawSoftKeys("Enter","<",">","Back");
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(0, curY+curY_off,display_str);
	// Draw menu options
	for(i=0;i<curY_N;i++){
		glcd_tiny_draw_string(5, i+curY_off, labels[i]);
	}
	glcd_write();
	
	
}



void fn_menu_cal(void){
	uint8_t i;
	const enum STATE gotoList[] = {st_disp_cal_report,
		st_loop_test,
		st_azm_quick_calibration,
		st_inc_azm_full_calibration,
		st_dist_calibration};
	const char * labels[] = {
		"Display Report",
		"Loop Test",
		"CAL: Quick AZM",
		"CAL: Full INC&AZM",
		"CAL: Range-finder",
	};
		
	//  Set initial conditions
	if (state_change) {
		curY = 0;
		curY_N = 5;
		curY_off = 1;
	}
	
	// Button Handler
	curY = getCursor(last_input, curY, curY_N);
	if (last_input==input_button1){
		current_state = gotoList[curY];
	}
	
	// Display
	glcd_clear_buffer();
		//  Display Title
	sprintf(display_str,"Calibration:");
	glcd_tiny_draw_string(0,0,display_str);
	// Display soft keys
	drawSoftKeys("Enter","<",">","Back");
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(0, curY+curY_off,display_str);
	//Display Options
	for (i=0;i<curY_N;i++){	
		glcd_tiny_draw_string(5, i+curY_off, labels[i]);
	}

	glcd_write();
	
}

void fn_debug_backlight(void){
	static char colorChar;
	static struct BACKLIGHT_COLOR *colorPtr;
	//  Set initial conditions
	if (state_change) {
		curY = 0;
		curY_N = 3;
		curY_off = 2;
		options.backlight_setting.colorRef = 0;//  0 is custom Color
		backlightOn(&options.backlight_setting);
		//colorPtr = backlightCustomAdjust(0, 0);
	}	
	
	switch(curY){
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
	curY = getCursor(last_input, curY, curY_N);
	switch(last_input){
		case input_button1:
			colorPtr = backlightCustomAdjust(colorChar, 1);
			break;
		case input_button4:
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
	glcd_tiny_draw_string(5, 2, display_str);
	sprintf(display_str, "Green: %d", colorPtr->green);
	glcd_tiny_draw_string(5, 3, display_str);
	sprintf(display_str, "Blue:  %d", colorPtr->blue);
	glcd_tiny_draw_string(5, 4, display_str);
	
	// Display soft keys
	drawSoftKeys("","Up","Down","");
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(0, curY_off,display_str);
	
	glcd_write();
	
	
}



void fn_set_options(void){
	//  Set initial conditions
	if (state_change) {
		curY = 0;
		curY_N = 7;
		curY_off = 1;
	}
	
	// Button Handler
	curY = getCursor(last_input, curY, curY_N);
	if (last_input==input_button1){
		switch (curY){
			case 0:
			//  Distance Units
			if (options.current_unit_dist == feet){ options.current_unit_dist = meters;}
			else{options.current_unit_dist = feet;}
			save_user_settings();
			break;
			case 1:
			//  Distance Units
			if (options.current_unit_temp == celsius){ options.current_unit_temp = fahrenheit;}
			else{options.current_unit_temp = celsius;}
			save_user_settings();
			break;
			case 2:
			// Shot Delay
			options.shot_delay = options.shot_delay+1;
			if (options.shot_delay>SHOT_DELAY_MAX){options.shot_delay = 0;}
			save_user_settings();
			break;
			case 3:
			// Charge Current
			if (options.chargeCurrent == 500){ options.chargeCurrent = 100;}
			else{options.chargeCurrent = 500;}
			setChargeCurrent(options.chargeCurrent);
			save_user_settings();
			break;
			case 4:
			// Charge Current
			adjustErrorSensitivity();
			save_user_settings();
			break;
			case 5:
			// Backlight Color
			backlightColorToggle(&options.backlight_setting);
			save_user_settings();
			break;
			case 6:
			// Backlight Color
			backlightLevelToggle(&options.backlight_setting);
			save_user_settings();
			break;
			default:
			break;
		}
		
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
	glcd_tiny_draw_string(0, curY+curY_off,display_str);
		

	glcd_write();


	
}

void fn_debug_BLE(void){
	bool pinState, doAction;
	bool debugPinState1,debugPinState2, debugPinState3;
	uint8_t i;
	char addStr[10];
	
	if (state_change) {
		curY=0;
		curY_N=3;
		curY_off=1;
	}
	
	
	curY = getCursor(last_input, curY, curY_N);
	
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"BLE Advanced:");
	glcd_tiny_draw_string(0,0,display_str);
	//Cycle through all entries
	for (i=0;i<curY_N;i++){
		if ((curY==i) && (last_input==input_button1)){
			doAction = true;
		}else{
			doAction = false;
		}
		switch (i){
			//  BLE Reset
			case 0:
				strcpy(display_str,"RST to AT Mode ");
				if (doAction){
					//  Reset to AT Mode
					BLE_reset_to_AT_mode();
					strcpy(addStr,"RST");
				}else{
					strcpy(addStr,"");
				}
				break;
			//  BLE Module Mode
			case 1:
				strcpy(display_str,"RST to Run Mode");
				if (doAction){
					BLE_init();
					strcpy(addStr,"RST");
				}else{
					strcpy(addStr,"");
				}
				break;
			//  BLE Communication Mode
			case 2:
				strcpy(display_str,"Curr Comm: ");

				if (isBleCommEnabled()){
					strcpy(addStr,"CPU");
				}else{
					strcpy(addStr,"TERM");
				}
				break;
		}//  End switch for each level
		//  Write entry
		strcat(display_str, addStr);
		glcd_tiny_draw_string(5, i+curY_off,display_str);
	}
	
	// Display soft keys
	drawSoftKeys("Adjust","<",">","Back");
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(0, curY+curY_off,display_str);
	
	
	
	//print BLE buffer
	glcd_draw_rect(0, 32, 104, 32,BLACK);
	print_Buff_to_Box(debugBuff, debugBuffIndex, 3, 33, 19, 3);
	
	glcd_write();
	
}

void fn_menu_BLE(void){
	char addStr[10];
	bool doAction;
	uint32_t i;
	
	if (state_change) {
		curY=0;
		curY_N=2;
		curY_off=6;
	}
	
	curY = getCursor(last_input, curY, curY_N);
		
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"Bluetooth:");
	glcd_tiny_draw_string(0,0,display_str);
	
	for(i=0;i<curY_N;i++){
		if((last_input==input_button1)&&(curY==i)){
			doAction=true;
			}
		else{
			doAction = false;
			}
		
		switch (i){
			//  BLE reset
			case 0:
			sprintf(display_str,"Reset BLE");
			if(doAction){
				BLE_init();
				sprintf(addStr," RST...");
			}else{
				sprintf(addStr,"");
			}			
			break;
			
			
			case 1:
			sprintf(display_str,"Advanced Menu");
			if(doAction){
				current_state=st_debug_BLE;
			}
			break;	
			
		}//  Loop for each entry
		//  Write entry
		strcat(display_str, addStr);
		glcd_tiny_draw_string(5, i+curY_off,display_str);
		
	}

	// Display soft keys
	drawSoftKeys("Adjust","<",">","Back");
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(0, curY+curY_off,display_str);
	
	//  Display device name
	BLE_get_device_name();
	sprintf(display_str,"Name:%s", BleDeviceName);		
	glcd_tiny_draw_string(0, 1,display_str);
	// Display device address
	BLE_get_device_MAC();
	glcd_tiny_draw_string(0, 2,"MAC Address:");
	glcd_tiny_draw_string(10, 3,BleDeviceMAC);
	// Connected to
	glcd_tiny_draw_string(0, 4,"Connected To:");
	glcd_tiny_draw_string(10, 5,BleClientMAC);
	
	glcd_write();
	
}


void fn_set_clock(void){
	uint8_t adjustment;
	static struct TIME tempTime;	
	
	if (state_change) {
		curY = 0;
		curY_N = 6;
		curY_off = 1;
		get_time();
		memcpy(&tempTime,&current_time,sizeof(current_time));	
		//  Set seconds to zero
		tempTime.seconds = 0;
	}

	// Button Handler
	adjustment = 0;
	switch(last_input){
		case input_button2:
			adjustment = 1;
			break;
		case input_button3:
			adjustment = -1;
			break;
		case input_button1:
			if((curY+1) >= curY_N){
				//  Done, set time and exit
				set_time(&tempTime);
				current_state = st_menu1;
			}else{
				curY++;
			}
			break;
		default:
			break;
		//  input_button4 aborts
	}
	
	//  Time Adjustment
	switch(curY){
		case 0:
			tempTime.year = incDecData(tempTime.year, adjustment, 2000, 2099);
			break;
		case 1:
			tempTime.month = incDecData(tempTime.month, adjustment, 1, 12);
			break;
		case 2:
			tempTime.day = incDecData(tempTime.day, adjustment, 1, 31);
			break;
		case 3:
			tempTime.hours = incDecData(tempTime.hours, adjustment, 0, 23);
			break;
		case 4:
			tempTime.minutes = incDecData(tempTime.minutes, adjustment, 0, 59);
			break;
		case 5:
			tempTime.seconds = incDecData(tempTime.seconds, adjustment, 0, 59);
			break;

	}
	
	
	glcd_clear_buffer();
	
	sprintf(display_str,"Set Clock:");
	glcd_tiny_draw_string(0,0,display_str);
	sprintf(display_str,"Year:   %04d", tempTime.year);
	glcd_tiny_draw_string(10,1,display_str);
	sprintf(display_str,"Month:  %02d", tempTime.month);
	glcd_tiny_draw_string(10,2,display_str);
	sprintf(display_str,"Date:   %02d", tempTime.day);
	glcd_tiny_draw_string(10,3,display_str);
	sprintf(display_str,"Hour:   %02d", tempTime.hours);
	glcd_tiny_draw_string(10,4,display_str);
	sprintf(display_str,"Minute: %02d", tempTime.minutes);
	glcd_tiny_draw_string(10,5,display_str);
	sprintf(display_str,"Second: %02d", tempTime.seconds);
	glcd_tiny_draw_string(10,6,display_str);
	
	// Display soft keys
	drawSoftKeys("Next","+","-","Cancel");
	
	//Display Pointer
	sprintf(display_str, ">");
	glcd_tiny_draw_string(1, curY+curY_off,display_str);
	
	glcd_write();
		
}




void fn_main_display(void){

	struct MEASUREMENT_FULL measTemp;
	
	//Handle Button Inputs
	
	switch (last_input){
		case input_button2:
			backlightPlus(&options.backlight_setting);
			save_user_settings();
			break;
		case input_button3:
			backlightMinus(&options.backlight_setting);
			save_user_settings();
			break;
		case input_buttonE:
			if (isLaserOn()){
				//  Laser already on, take measurement
				full_measurement(&measTemp, options.shot_delay, measRegular);
				rangefinder_on_off(false);
				//processMeasurement(&measTemp);	
			}else{
				//  Turn on laser for measuring
				rangefinder_on_off(true);
				laser_on_off(true);
				
			}		
			break;
		default:
			break;
	}
	
	if (isLaserOn()){
		//  If laser is on, take a measurement for display
		struct MEASUREMENT_FULL tempMeas;
		quick_measurement(&tempMeas);
		//memcpy(&measBuf[measBufInd],&tempMeas,sizeof(measBuf[measBufInd]));
		//copyMeasurement(&measBuf[measBufInd], &tempMeas);
		
	}
	
	print_data_screen();
	
	
	
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
	//  Buzzer
	ioport_set_pin_dir(BuzzerPin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BuzzerPin, false);
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
	ioport_reset_pin_mode(MCU_RTS1);
	ioport_reset_pin_mode(MCU_CTS1);
	ioport_set_pin_dir(MCU_TX1, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(MCU_TX2, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(MCU_RX1, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(MCU_RX2, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(MCU_RTS1, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(MCU_CTS1, IOPORT_DIR_INPUT);
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

}







void fn_powerdown(void){

	if (state_change){
		//  Save into EEPROM
		save_sync_tracker();
		
		//  Turn off BLE advertising
		BLE_advert_OnOff(false);
		// Disable watchdog timer
		wdt_disable();
		//  Play powerdown song
		buzzOn(tone4, 150);
		buzzOn(tone2, 150);
		buzzOn(tone1, 150);			
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
	//  Setup GCLK 0 for 48MHZ
	mainClockPowerup();
	
	config_pins_powerup();
	//delay_ms(10);
	//  Play powerup song
	buzzOn(tone1, 150);
	buzzOn(tone2, 150);
	buzzOn(tone4, 150);
	
	//  Various setups
	enable_comms();	
	load_user_settings();//  Needed for backlight setting
	backlightOn(&options.backlight_setting);
	
	
	//configure_extint_channel();
	//configure_extint_callbacks();
	setup_accel(&slave_acc1);
	setup_accel(&slave_acc2);
	setup_mag(&slave_mag1);
	setup_mag(&slave_mag2);
	rangefinder_on_off(false);	
	
	//  Turn on external clock and take it as a source
	ext_osc_onoff(true);
	//delay_ms(10);	

	//  Initialize LCD controller
	//delay_ms(500);  //  Delay requested by LCD controller
	glcd_init();
	glcd_tiny_set_font(Font5x7,5,7,32,127);//  All font in "tiny" mode
	
	//  Setup background timers
	wdt_enable();
	configure_timer_1s();
	configure_timer_ExtLong();
	
	//  Try turning on bluetooth advertising
	BLE_advert_OnOff(true);
	
	
	//  Startup USB mass storage
	system_interrupt_enable_global();	
	irq_initialize_vectors();
	cpu_irq_enable();
	configure_SD();
	udc_start();
	
	//  Setup Buttons
	config_buttons();
	
	//  Initialize bluetooth measurement sync tracker
	load_sync_tracker();
	
	//  Set initial conditions for state machine
	buttonE_triggered=false;//  In case button was pressed again during powerup
	current_state = st_main_display;
	current_input = input_none;
	last_input = input_none;
	
	
	
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

	
}







void processMeasurement(struct MEASUREMENT_FULL *measFullPtr){
	struct MEASUREMENT *measPtr;
	struct MEASUREMENT measTemp;
	
	//  Display Buffer Operations
	switch (measFullPtr->meas_type){
		case measRegular:
			//  Point to Display Buffer
			measPtr = &measBuf[measBufInd];
			// Increment Counter
			circBuffInc(&measBufInd, N_MEASBUF);	
			break;
		case measScan:
			//  Point to temporary location
			measPtr = &measTemp;
			break;
		case measQuick:
			//  Point to Display Buffer
			measPtr = &measBuf[measBufInd];
			//  Do not increment
			break;	
		default:
			return;
	}
	//  Copy Data Over
	memcpy(measPtr,measFullPtr,sizeof(*measPtr));
	
	//  SD Card Storage
	//  Also updates BLE measurement tracker
	switch (measFullPtr->meas_type){
		case measRegular:
			save_measurement(measPtr);
			break;
		case measScan:
			save_measurement(measPtr);
			break;
		case measQuick:
			//  Do Nothing
			break;
		default:
			return;
	}
	
		
}

void fn_firmware(void){
	
	if (state_change) {
		
	}
	
	
	
	glcd_clear_buffer();
	//  Display Title
	sprintf(display_str,"Firmware:");
	glcd_tiny_draw_string(0,0,display_str);
	
	if(last_input==input_button1){
		NVIC_SystemReset();

	}
	
	// Display soft keys
	drawSoftKeys("Bootloader","","","Back");
	
	
	sprintf(display_str,"Firmware Ver: %0.2f", SOFTWARE_VERSION);
	glcd_tiny_draw_string(0, 3,display_str);
	
	sprintf(display_str,"Hardware Ver: %s", HARDWARE_VERSION);
	glcd_tiny_draw_string(0, 4,display_str);
	
	strcpy(display_str,"Press and hold");
	glcd_tiny_draw_string(0, 5,display_str);
	strcpy(display_str,"\"Bootloader\" for");
	glcd_tiny_draw_string(0, 6,display_str);
	strcpy(display_str,"USB Bootloader");
	glcd_tiny_draw_string(0, 7,display_str);

	
	glcd_write();
	
	
}


void msc_notify_trans(void){
	
	//current_input=input_usb_transaction;
	usb_transaction_requested = true;
}


bool my_callback_msc_enable(void)
{
	SD_WriteLockout = true;
	return true;
}
void my_callback_msc_disable(void)
{
	SD_WriteLockout = true;
}

void UsbHandleTransactions(void){
	
	if (current_state==st_powerdown){
		return;
	}
	if (usb_transaction_requested){
		spi_setBaud(baudRateMax);
		while(udi_msc_process_trans());
			
		usb_transaction_requested = false;		
		spi_setBaud(baudRateMin);		
	}
	if(current_input== input_usb_transaction){
		current_input = input_none;
	}
	
}
			
