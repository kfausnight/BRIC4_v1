/*
 * EEPROM.c
 *
 * Created: 2/2/2019 1:53:07 PM
 *  For MICROCHIP EEPROM 24LC32A
 *  Author: Kris Fausnight
 */ 


#include <EEPROM.h>


//  EEPROM I2C address
#define EEPROM_add		0x57  //0b1010111RW
#define PAGE_SIZE 128 // per datasheet for 24LC512


//  EEPROM Memory Map 
//  Maximum 0xFA00
#define add_measTracker	0x0000
#define add_options		0x0100
#define add_a1_calst	0x0300
#define add_a2_calst	0x0400
#define add_m1_calst	0x0500
#define add_m2_calst	0x0600
#define add_dist_calst	0x0700
#define add_cal_report	0x0800
#define add_calRawData_full	0x1000
#define add_calRawData_qazm	0x2500

//  EEPROM Initialization Keys
#define KEY_SETTINGS_INITIALIZED	0xC3
#define KEY_MEASUREMENT_TRACKER		0xA1

uint16_t EEPROM_get_address(enum CALTYPE caltype){
	uint16_t tempAddr;
	switch (caltype){
		case inc_azm_full:
			tempAddr = add_calRawData_full;
			break;
		case azm_quick:
			tempAddr = add_calRawData_qazm;
			break;
		default:
			tempAddr = 0;
	}
	return tempAddr;
}


void EEPROM_saveCalRawData(enum CALTYPE caltype){
	uint32_t addOff;
	uint16_t tempAddr = EEPROM_get_address(caltype);
		
	EEPROM_write(tempAddr ,a1Raw, sizeof(a1Raw));
	addOff = sizeof(a1Raw);
	EEPROM_write(tempAddr + addOff ,a2Raw, sizeof(a2Raw));
	addOff = addOff+sizeof(a2Raw);
	EEPROM_write(tempAddr + addOff,m1Raw, sizeof(m1Raw));
	addOff = addOff+sizeof(m1Raw);
	EEPROM_write(tempAddr+addOff ,m2Raw, sizeof(m2Raw));
}

void EEPROM_loadCalRawData(enum CALTYPE caltype){
	uint32_t addOff;
	uint16_t tempAddr = EEPROM_get_address(caltype);
		
	EEPROM_read(tempAddr ,a1Raw, sizeof(a1Raw));
	addOff = sizeof(a1Raw);
	EEPROM_read(tempAddr + addOff ,a2Raw, sizeof(a2Raw));
	addOff = addOff+sizeof(a2Raw);
	EEPROM_read(tempAddr + addOff,m1Raw, sizeof(m1Raw));
	addOff = addOff+sizeof(m1Raw);
	EEPROM_read(tempAddr+addOff ,m2Raw, sizeof(m2Raw));
}


void load_sync_tracker(void){
	//  Read options structure from EEPROM
	EEPROM_read(add_measTracker, &bleSyncTracker, sizeof(bleSyncTracker));
	
	if (bleSyncTracker.keyMeasTrackerInitialized != KEY_MEASUREMENT_TRACKER){
		// Settings in EEPROM not initialized or are out of date
		//  Reload default settings into 
		initSyncTracker();
		bleSyncTracker.keyMeasTrackerInitialized = KEY_MEASUREMENT_TRACKER;
		
		//  Save into EEPROM
		save_sync_tracker();
	}
	
}

void save_sync_tracker(void){

	// Save User Options
	EEPROM_write(add_measTracker, &bleSyncTracker, sizeof(bleSyncTracker));
	
}

void load_user_settings(void){
	
	//  Read options structure from EEPROM
	EEPROM_read(add_options, &options, sizeof(options));
	
	if (options.keySettingsInitialized != KEY_SETTINGS_INITIALIZED){
		// Settings in EEPROM not initialized or are out of date
		//  Reload default settings into 
		getDefaultOptions(&options);
		options.keySettingsInitialized = KEY_SETTINGS_INITIALIZED;
		//  Save into EEPROM
		save_user_settings();
	}
	
	
}

void save_user_settings(void){

	// Save User Options
	EEPROM_write(add_options, &options, sizeof(options));
	
}


void load_calibration(void){
	
	//  Create example calibration structure to find initialization key
	struct INST_CAL tempCal;	
	cal_init_struct(&tempCal);
	//  Read All  calibration structure back	
	EEPROM_read(add_a1_calst, &a1_calst, sizeof(a1_calst)); 
	EEPROM_read(add_a2_calst, &a2_calst, sizeof(a1_calst));
	EEPROM_read(add_m1_calst, &m1_calst, sizeof(a1_calst));
	EEPROM_read(add_m2_calst, &m2_calst, sizeof(a1_calst));
	EEPROM_read(add_dist_calst, &dist_calst, sizeof(a1_calst));
	EEPROM_read(add_cal_report, &cal_report, sizeof(cal_report));
	
	// assume first struct is representative of remainder
	if(tempCal.Cal_Initialized_Key != a1_calst.Cal_Initialized_Key){
		//  EEPROM data has not been initialized or is out of date
		cal_init_struct(&a1_calst);
		cal_init_struct(&a2_calst);
		cal_init_struct(&m1_calst);
		cal_init_struct(&m2_calst);
		cal_init_struct(&dist_calst);
		save_calibration();		
	}
		
	
	
}

void save_calibration(void){

	uint8_t bytes_calst, bytes_report;
	
	bytes_report = sizeof(cal_report);
	bytes_calst = sizeof(a1_calst);
	
	// Save calibration structures
	EEPROM_write(add_a1_calst, &a1_calst, bytes_calst);
	EEPROM_write(add_a2_calst, &a2_calst, bytes_calst);
	EEPROM_write(add_m1_calst, &m1_calst, bytes_calst);
	EEPROM_write(add_m2_calst, &m2_calst, bytes_calst);
	EEPROM_write(add_dist_calst, &dist_calst, bytes_calst);
		
	// Save Calibration Report
	EEPROM_write(add_cal_report, &cal_report, bytes_report);
	
	
}
void save_cal_report(void){
	EEPROM_write(add_cal_report, &cal_report, sizeof(cal_report));
}
void load_cal_report(void){
	EEPROM_read(add_cal_report, &cal_report, sizeof(cal_report));
}




void EEPROM_read(uint16_t data_address, char data_buf[], uint32_t bytes_to_read){
	struct i2c_master_packet packet = {
		.address     = EEPROM_add,
		.data        = data_buf,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};
	
	
	//  Write 16bit read address
	//  Set Up packet
	uint8_t add_buf[2];
	add_buf[0] = data_address>>8;//high byte
	add_buf[1] = data_address & 0xff;//low byte
	packet.data = add_buf;
	packet.data_length=2;
	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) !=STATUS_OK) {
	}
	//  Send read request to eeprom chip
	packet.data = data_buf;
	packet.data_length=bytes_to_read;
	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) !=STATUS_OK) {
		//if (timeout++ == limit) {   break;   }
	}
	
	
	
	
}

void EEPROM_write(uint16_t address_init, char data_buf[], uint32_t bytes_to_write){
	uint32_t bytes_written;
	uint16_t bytes_possible;
	uint16_t bytes_packet;
	uint16_t address_packet;
	uint32_t bytes_remainder;
	uint16_t  i;

	uint8_t send_buf[PAGE_SIZE+2];
	
	//  Packet template
	struct i2c_master_packet packet;
	packet.data = send_buf;
	packet.ten_bit_address = false;
	packet.high_speed = false;
	packet.hs_master_code = 0x0;
	packet.address = EEPROM_add;


	//  Initialize variables
	bytes_written = 0;
	address_packet = address_init;
	bytes_remainder = bytes_to_write;
	
	//  Send packets until everything is written
	while(bytes_written<bytes_to_write){
		//  Can only send in 32-byte page increments 
		//  Cannot pass page boundaries
		bytes_possible = PAGE_SIZE-(address_packet & 0x7F);
		//  Determine number of bytes to send in packet
		if (bytes_possible<bytes_remainder){
			bytes_packet = bytes_possible;
		}else{
			bytes_packet = bytes_remainder;
		}
		//  Set up Packet
		send_buf[0] = address_packet>>8;
		send_buf[1] = address_packet & 0x00FF;
		packet.data_length = bytes_packet+2; //2 for data address
		for (i=0;i<bytes_packet;i++){
			send_buf[i+2] = data_buf[bytes_written+i];
		}
		
		// Send Packet
		while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) !=STATUS_OK) {
		}
		//  Increment Variables
		bytes_written = bytes_written+bytes_packet;
		bytes_remainder = bytes_to_write-bytes_written;
		address_packet = address_packet+bytes_packet;
		
	}
	
}




