/*
 * BLE_func.c
 *
 * Created: 9/6/2020 1:59:29 PM
 *  Author: Kris Fausnight
 */ 

#include <BLE_func.h>


void BLE_handleMessage(void){
	uint8_t debug;
	//  First check that there is anything to be done
	if(!isBleReceiveComplete()){
		return;
	}
	enum status_code writeStatus;
	
	//  
	if (strstr(rxBufferBle,"info")){
		sprintf(write_str1,"BRIC4\n SN: %04d\n Software version %f\n", options.SerialNumber, SOFTWARE_VERSION);
		writeBle(write_str1, strlen(write_str1));
		/*
		while(!isBleTransmitComplete());
		
		while(!isBleReceiveComplete());
		
		rxBufferBleClear();
		sprintf(write_str1,"RESUME\r\n");		
		writeBle(write_str1, strlen(write_str1));
		while(!isBleTransmitComplete());
		
		while(!isBleReceiveComplete());
		debug = 1;
		*/
		
	}else{
		return;
	}
	
	rxBufferBleClear();
	
	
}

void ble_error(void){
	
	
	ioport_set_pin_level(BLE_reset, false);
	//  Turn on Autorun
	ioport_set_pin_level(BLE_autorun, true);//low for autorun enabled, high for development mode
	//  Turn on Command MOde
	ioport_set_pin_level(BLE_COMMAND_MODE, true);//  Start in Command Mode
	//  Diable OTA programming
	ioport_set_pin_level(BLE_ota, false);// low to disable programming over BLE
	//  Isolate UART for debug/programming interface
	BLE_usart_isolate();
	
	// Turn on Module
	delay_ms(100);
	ioport_set_pin_level(BLE_reset, true);
	
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
	uint32_t ms_counter;
	char *debugPtr;
	debugPtr = &debugBuff[0];
	ioport_set_pin_dir(BLE_ota, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(BLE_autorun, IOPORT_DIR_OUTPUT);	
	ioport_set_pin_dir(BLE_reset, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(BLE_COMMAND_MODE, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BLE_ota, false);// low to disable programming over BLE
	
	if (!isBleCommEnabled()){
		configure_usart_BLE();
	}
	
	
	//  Hold in reset
	ioport_set_pin_level(BLE_reset, false);
	//  Turn on Autorun
	ioport_set_pin_level(BLE_autorun, false);//low for autorun enabled, high for development mode
	//  Turn on Command MOde
	ioport_set_pin_level(BLE_COMMAND_MODE, true);//  Start in Command Mode
	delay_ms(100);			
	// Turn on Module
	ioport_set_pin_level(BLE_reset, true);
	delay_ms(500);
	//  Assign Device Name
	sprintf(write_str1,"cfg$ wr  1 BRIC4_%04d\r\n", options.SerialNumber);
	rxBufferBleClear();
	writeBle(write_str1, strlen(write_str1));
	while(!isBleTransmitComplete());	
	ms_counter = 0;
	while(!strstr(rxBufferBle,"OK")){
		delay_ms(100);
		ms_counter += 100;
		if(ms_counter>2000){
			ble_error();			
			return;
		}
	}
	while(!isBleReceiveComplete());	
	rxBufferBleClear();
	//  Start Bluetooth Connection
	sprintf(write_str1,"connect\r\n");
	rxBufferBleClear();
	writeBle(write_str1, strlen(write_str1));
	while(!isBleTransmitComplete());	
	while(!isBleReceiveComplete());	
	rxBufferBleClear();
	//  Turn off Command Mode
	ioport_set_pin_level(BLE_COMMAND_MODE, false);//  Start in Command Mode
	
	
	
}


void BLE_sendMeas(struct MEASUREMENT *meas){
	enum status_code writeStatus;
	
	sprintf(write_str1,"Posix:%d, Ref:%d, Dist:%0.2f, Azm: %0.3f, Inc: %0.3f, Dec: %0.3f\n",
		meas->posix_time, meas->index_ref, meas->distCal, meas->azimuth, meas->inclination,	meas->declination);
		
	
	writeStatus = writeBle(write_str1, strlen(write_str1));
		
	while(!isBleTransmitComplete());
	
	
	//clear_rx_buffer();
	
	
	
}
