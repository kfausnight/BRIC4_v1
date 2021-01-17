/*
 * BLE_func.c
 *
 * Created: 9/6/2020 1:59:29 PM
 *  Author: Kris Fausnight
 */ 

#include <BLE_func.h>



///////////////////////////////////////////////////////////////////////////////////////////////////////////
//  CPU to BLE communication commands
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Advertisement Commands
#define cmdAdvertInterval	1// ["nnnn"]- Set advertising interval in milliseconds
#define cmdAdvertTimeout	2// ["nnnn"]- Set advertising timeout in milliseconds
#define cmdAdvertFilter		3// ["n"] - Set advertising filter policy 0..3 (see ADV_FILTERPOLICY_xx)
#define cmdMacAddress		4// ["nnnnnnnnnnnn"] - Set mac address - 12 hex digits
#define cmdSetAdvert		5// ["n"]  Start or Stop Adverts
//  If n=0 then stop adverts otherwise start adverts if
//  n=1 then ADV_IND
//  n=2 then ADV_DIRECT_IND
//  n=3 then ADV_SCAN_IND
//  n=4 then ADV_NONCONN_IND

//  Battery Service
#define cmdSetBattery		10 // ["nn"] Set Battery Level
//    Battery level % in hex, maximum of 100
//    "25" is battery level of 37%
#define cmdGetBattery		11 // []  Request Current Battery Level

//  Device Information Service
//  Set Variables
#define cmdSetDeviceName	20 // ["n...n"]  Set Device Name String
#define cmdSetSn		21 // ["nnnn"]  Set Device Serial Number as string
#define cmdSetMfg		22 // ["n...n"]  Set Manufacturer Name string
#define cmdSetModel			23 // ["n...n"]  Set Model Name String
#define cmdSetHardwareVer	24 // ["n...n"]  Set Hardware Version String
#define cmdSetFirmwareVer	25 // ["n...n"]  Set Firmware Version String
#define cmdSetSoftwareVer	26 // ["n...n"]  Set Software Version String
//  Get Variables
#define cmdGetDeviceName	27 // []  Get Device Name String
#define cmdGetSn		28 // []  Get Device Serial Number as string
#define cmdGetMfg		29 // []  Get Manufacturer Name string
#define cmdGetModel			30 // []  Get Model Name String
#define cmdGetHardwareVer	31 // []  Get Hardware Version String
#define cmdGetFirmwareVer	32 // []  Get Firmware Version String
#define cmdGetSoftwareVer	33 // []  Get Software Version String

// Measurement Service
//  Set Variables
#define cmdSetMeasurement	40 //["n....n"] Measurement string to be posted directly to attribute, max 20 bytes
#define cmdSetMeasurementMetadata	41
#define cmdSetMeasurementErrors	42
#define cmdSetMeasurementAll	43
#define cmdSetDateLast		44 //["nnnnnnnnnn"] POSIX time, last measurement indicated
#define cmdSetRefLast		45 //["nnnnnnnnnn"] Reference #, last measurement indicated


//  Device Control Service
#define cmdDeviceControl	60 //["n....n"] String command for device

//  Miscellaneous
#define cmdStartup			80 // []  Command to start advertising and connections
#define cmdQuit				81 // []  Command to quit application to AT mode
#define cmdGetMacAdd		82 // []  Command to send mac address
#define cmdGetConnectedDev	83 // []  Command to send connected device address
#define cmdDeviceConnected	84 // []  Command to alert device has connected
#define cmdDeviceDisconnected	85 // [] Command to alert device has been disconnected

#define cmdOK				254 // [...]  Indicates success
#define cmdError			255 // [...]  Indicates error
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


volatile bool isBleConnectedBool = false;


void BLE_handleMessage(void){
	char rcvCmd; 
	uint8_t rcvLength;
	uint8_t debugB;
	enum status_code commStatus;
	
	//Check to see if device is in powerdown mode
	//if (current_state== st_powerdown){
	//	return;
	//}
	
	//  First check that there is anything to be done
	//if(!isBleReceiveComplete()){
	//	return;
	//}

	
	//  Read back any messages
	//  Also clears buffer
	commStatus = BLE_read_message(&rcvCmd, write_str1, &rcvLength, sizeof(write_str1));
	if (commStatus!= STATUS_OK){
		return;
	}
	
	//  Add idle_timeout reset so that this counts as an input
	idle_timeout();
	
	//tempInput = input_none;//  Default no input
	
	switch (rcvCmd){
		case (cmdDeviceConnected):
			isBleConnectedBool = true;
			break;
		case (cmdDeviceDisconnected):
			isBleConnectedBool = false;
			break;
		case (cmdSetMeasurement):
			BLE_update_tracker(write_str1);
			break;
		case (cmdSetMeasurementMetadata	):
			debugB = 1;
			break;
		case (cmdSetMeasurementErrors):
			debugB = 2;
			break;
		case (cmdSetDateLast):
			BLE_adjLastDate_tracker(write_str1);
			break;
		case (cmdDeviceControl):
			//BLE_remoteCommand(write_str1);
			strncpy(BleCommandQueue,write_str1, sizeof(BleCommandQueue));
			current_input = input_BLE_command;			
			break;
		default:
			break;
	}
		
		
	//current_input = tempInput;
	
	
}



void  BLE_remoteCommand(char *commandStr){
	
	if (current_state==st_powerdown){
		return;
	}
	
	
	if (current_state == st_scan){
		//  If in scan mode, any command exits
		//  Turn on laser for measuring
		rangefinder_on_off(false);
		current_state = st_main_display;
	}else if(strstr(commandStr,"scan")){
		current_state = st_scan;
	}else if (strstr(commandStr,"laser")){
		if(isLaserOn()){
			// Turn off laser
			rangefinder_on_off(false);
			
		}else{
			//  Turn on laser for measuring
			rangefinder_on_off(true);
			laser_on_off(true);
		}
	}else if (strstr(commandStr,"power off")){
		current_input = input_pwrDown;
	}else if(strstr(commandStr,"shot")){
		if(isLaserOn()){
			//  Laser already on, take measurement
			struct MEASUREMENT_FULL measTemp;
			full_measurement(&measTemp, 0, measRegular);
			rangefinder_on_off(false);
		}else{
			//  Turn on laser for measuring
			rangefinder_on_off(true);
			laser_on_off(true);
		}
		
		
	}

	
};

void BLE_MAC_format(char macAdd[], char messageStr[]){
	//  Always 16 Hex characters MAC add 
	//  prepended with 2 unnecessary characters
	//  Ignore these characters
	uint8_t i;
	
	for(i=0;i<16;i++){
		macAdd[i] = messageStr[i+2];
	}
	macAdd[16] = 0;//  Terminate with Null character
	
	
	
}

void BLE_get_device_MAC(void){
	char rcvCmd;
	uint8_t rcvLength;
	char temp;
	enum status_code commStatus;
	
	if (current_state==st_powerdown){
		return;
	}
	
	commStatus = BLE_send_parse_CMD(cmdGetMacAdd, &temp, 0,
		&rcvCmd, write_str1 , &rcvLength, sizeof(write_str1));
	
	//  Format returned string into MAC address
	BLE_MAC_format(BleDeviceMAC, write_str1);

}

void BLE_get_client_MAC(){
	char rcvCmd, temp;
	uint8_t rcvLength;

	enum status_code commStatus;
	
	if (current_state==st_powerdown){
		return;
	}
	
	
	commStatus = BLE_send_parse_CMD(cmdGetConnectedDev, &temp, 0,
		&rcvCmd, write_str1, &rcvLength, sizeof(write_str1));
	if(rcvLength==0){
		strcpy(BleClientMAC,"Not Connected");
		isBleConnectedBool = false;
	}else{
		//  Format returned string into MAC address
		BLE_MAC_format(BleClientMAC, write_str1);
		isBleConnectedBool = true;
	}
	
}
bool isBleConnected(void){
	return isBleConnectedBool;
}

void BLE_get_device_name(void){
	char rcvCmd;
	uint8_t rcvLength;
	enum status_code commStatus;
	
	commStatus = BLE_send_parse_CMD(cmdGetDeviceName, write_str1, 0,
		&rcvCmd, BleDeviceName, &rcvLength, sizeof(BleDeviceName) );
		
	BleDeviceName[rcvLength] = 00;// Terminate with null character
}

void BLE_advert_OnOff(bool onOff){
	enum status_code commStatus;
	char rcvCmd;
	uint8_t rcvLength;

	if (onOff){
		write_str1[0] = '1';
	}else{
		write_str1[0] = '0';

	}
	
	commStatus = BLE_send_parse_CMD(cmdSetAdvert, write_str1, 1,
		&rcvCmd, write_str2, &rcvLength, sizeof(write_str2));

}


void BLE_reset_to_AT_mode(void){
	
	//  Hold in reset
	ioport_set_pin_level(BLE_nRESET_pin, false);
	//  Turn off Autorun
	ioport_set_pin_level(BLE_nAUTORUN_pin, true);//low for autorun enabled, high for development mode
	//  Disable OTA
	ioport_set_pin_level(BLE_OTA_sendMode_pin, false);
	//  Isolate UART for debug/programming interface
	BLE_usart_isolate();
	
	// Turn on Module
	delay_ms(100);//  Ensure held in reset for min 100 ms
	ioport_set_pin_level(BLE_nRESET_pin, true);
	
}

void BLE_error(void){
	
	
	BLE_reset_to_AT_mode();
	
	
	wdt_disable();
	
	glcd_clear_buffer();
	
	sprintf(display_str, "BLE Error");
	glcd_tiny_draw_string(0,1,display_str);
	sprintf(display_str, "Setting BLE in");
	glcd_tiny_draw_string(0,2,display_str);
	sprintf(display_str, "AT Mode for program");
	glcd_tiny_draw_string(0,3,display_str);
	sprintf(display_str, "Reset after program");
	glcd_tiny_draw_string(0,5,display_str);
	
	

	//hold here until an input
	sprintf(display_str, "Press any button...");
	glcd_tiny_draw_string(10,7,display_str);
	glcd_write();
	while((current_input == input_none) || (current_input == input_1sec));
	
	//Resume normal operation
	current_input = input_none;
	wdt_enable();
	
}


void BLE_init(void){
	enum status_code commStatus;
	uint8_t  rcvLength;
	char rcvCmd;
	
	//  Configure Reset pin; low holds in reset
	ioport_set_pin_dir(BLE_nRESET_pin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BLE_nRESET_pin, false);
	//  Configure Autorun Pin
	//  Low enables auto-run of downloaded applications.
	ioport_set_pin_dir(BLE_nAUTORUN_pin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BLE_nAUTORUN_pin, false);
	//  Configure OTA Enable / Send-Mode
	//  High enables OTA downloads (Only in AT mode)
	//  Dual purpose send-mode indicator to BL652
	ioport_set_pin_dir(BLE_OTA_sendMode_pin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BLE_OTA_sendMode_pin, false);//  Start low; no data
	//  Configure Receive-mode pin
	ioport_set_pin_dir(BLE_rcvMode_pin, IOPORT_DIR_INPUT);
	
	//  Initialize communications	
	if (!isBleCommEnabled()){
		configure_usart_BLE();
	}
	//  Ensure minimum of 100 ms lowbefore release of reset
	delay_ms(100);
	// Turn on Module, release reset pin
	ioport_set_pin_level(BLE_nRESET_pin, true);
	delay_ms(500);

		
	//  Assign Device Name
	sprintf(write_str1,"BRIC4_%04d", options.SerialNumber);
	commStatus = BLE_send_parse_CMD(cmdSetDeviceName, write_str1, strlen(write_str1),
			&rcvCmd, write_str2, &rcvLength, sizeof(write_str2));
	if ((commStatus!= STATUS_OK)||(rcvCmd!= cmdOK)){
		BLE_error();			
		return;
	}
	//  Assign Device SN
	sprintf(write_str1,"%04d", options.SerialNumber);
	commStatus = BLE_send_parse_CMD(cmdSetSn, write_str1, strlen(write_str1),
			&rcvCmd, write_str2, &rcvLength, sizeof(write_str2));
	if ((commStatus!= STATUS_OK)||(rcvCmd!= cmdOK)){
		BLE_error();
		return;
	}
	
	//  Assign Device SW version
	sprintf(write_str1,"%0.2f", SOFTWARE_VERSION);
	commStatus = BLE_send_parse_CMD(cmdSetSoftwareVer, write_str1, strlen(write_str1),
	&rcvCmd, write_str2, &rcvLength, sizeof(write_str2));
	if ((commStatus!= STATUS_OK)||(rcvCmd!= cmdOK)){
		BLE_error();
		return;
	}
	//  Assign Device Hardware version
	sprintf(write_str1,"%s", HARDWARE_VERSION);
	commStatus = BLE_send_parse_CMD(cmdSetHardwareVer, write_str1, strlen(write_str1),
	&rcvCmd, write_str2, &rcvLength, sizeof(write_str2));
	if ((commStatus!= STATUS_OK)||(rcvCmd!= cmdOK)){
		BLE_error();
		return;
	}
	
	
	//  Start Application
	commStatus = BLE_send_parse_CMD(cmdStartup, write_str1, 0,
			&rcvCmd, write_str2, &rcvLength, sizeof(write_str2));
	if ((commStatus!= STATUS_OK)||(rcvCmd!= cmdOK)){
		BLE_error();
		return;
	}
	
	//  Start Advertising
	BLE_advert_OnOff(true);
	

	
}

uint8_t BLE_getBatteryLevel(void){
	uint8_t bleErr;
	char rcvCmd;
	uint8_t rcvLength;
	uint8_t battLevel;
	
	write_str1[0] = cmdGetBattery;
	write_str1[1] = 0x00;
	bleErr = BLE_send_parse_CMD(cmdGetBattery, write_str1, 0,
		&rcvCmd, write_str2, &rcvLength, sizeof(write_str2));
	if (bleErr){
		//BLE_error();
		return 0;
	}
	//  Data returned in hex format, 0-100
	//  for example, "64" is 100
	battLevel = write_str2[0];
	
	return battLevel;
	
}

void BLE_setBatteryLevel(uint8_t battLevel){
	uint8_t bleErr, rcvLength;
	char rcvCmd;
	
	write_str1[0] = battLevel;	
	bleErr = BLE_send_parse_CMD(cmdSetBattery, write_str1, 1,
		&rcvCmd, write_str2, &rcvLength, sizeof(write_str2));
	if (bleErr){
		//BLE_error();
		return;
	}
}


void BLE_sendMeas(struct MEASUREMENT *meas){
	char rcvCmd;
	uint8_t bleErr, rcvLength;
	
	if (!isBleConnected()){
		return;
	}
	/*
	//  Send Error Data
	memcpy(write_str1, &meas->errCode[0], 1);
	memcpy(write_str1+1, &meas->measurement_error_data1[0], 4);
	memcpy(write_str1+5, &meas->measurement_error_data2[0], 4);
	memcpy(write_str1+9, &meas->errCode[2], 1);
	memcpy(write_str1+10, &meas->measurement_error_data1[2], 4);
	memcpy(write_str1+14, &meas->measurement_error_data2[2], 4);
	bleErr = BLE_send_parse_CMD(cmdSetMeasurementErrors, write_str1, 18,
		&rcvCmd, write_str2, &rcvLength);	
	
	//  Send MetaData
	memcpy(write_str1, &meas->refIndex, 4);
	memcpy(write_str1+4, &meas->dip, 4);
	memcpy(write_str1+8, &meas->roll, 4);
	memcpy(write_str1+12, &meas->temperatureC, 4);
	memcpy(write_str1+16, &meas->samples, 2);
	memcpy(write_str1+18, &meas->meas_type, 1);
	bleErr = BLE_send_parse_CMD(cmdSetMeasurementMetadata, write_str1, 19,
		&rcvCmd, write_str2, &rcvLength);
	
	//  Send Primary Measurement
	memcpy(write_str1, &meas->measTime, 8);
	memcpy(write_str1+8, &meas->distMeters, 4);
	memcpy(write_str1+12, &meas->azimuth, 4);
	memcpy(write_str1+16, &meas->inclination, 4);
	bleErr = BLE_send_parse_CMD(cmdSetMeasurement, write_str1, 20,
		&rcvCmd, write_str2, &rcvLength);
	*/
	
	//  Primary
	memcpy(write_str1, &meas->measTime, 8);
	memcpy(write_str1+8, &meas->distMeters, 4);
	memcpy(write_str1+12, &meas->azimuth, 4);
	memcpy(write_str1+16, &meas->inclination, 4);
	//  Send MetaData
	memcpy(write_str1+20, &meas->refIndex, 4);
	memcpy(write_str1+24, &meas->dip, 4);
	memcpy(write_str1+28, &meas->roll, 4);
	memcpy(write_str1+32, &meas->temperatureC, 4);
	memcpy(write_str1+36, &meas->samples, 2);
	memcpy(write_str1+38, &meas->meas_type, 1);
	//  Send Error Data
	memcpy(write_str1+40, &meas->errCode[0], 1);
	memcpy(write_str1+41, &meas->measurement_error_data1[0], 4);
	memcpy(write_str1+45, &meas->measurement_error_data2[0], 4);
	memcpy(write_str1+49, &meas->errCode[2], 1);
	memcpy(write_str1+50, &meas->measurement_error_data1[2], 4);
	memcpy(write_str1+54, &meas->measurement_error_data2[2], 4);
	bleErr = BLE_send_parse_CMD(cmdSetMeasurementAll, write_str1, 60,
	&rcvCmd, write_str2, &rcvLength, sizeof(write_str2));
	
	

}




