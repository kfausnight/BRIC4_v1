/*
 * EEPROM.c
 *
 * Created: 2/2/2019 1:53:07 PM
 *  Author: Kris Fausnight
 */ 

#include <stdint.h>
#include <EEPROM.h>
#include <comms/comms.h>

#include <calibration.h>
#include <sensors.h>
#include <main.h>

//  EEPROM I2C address
#define EEPROM_add		0x57  //0b1010111RW


//  EEPROM Memory Map and Key
#define add_a1_calst	0x0300
#define add_a2_calst	0x0400
#define add_c1_calst	0x0500
#define add_c2_calst	0x0600
#define add_dist_calst	0x0700
#define add_cal_report_azm_inc	0x0800
#define add_cal_report_dist		0x0900
#define add_options				0x0102

extern struct INST_CAL a1_calst, a2_calst, c1_calst, c2_calst, dist_calst;
extern struct CAL_REPORT cal_report_azm_inc, cal_report_dist;
extern struct OPTIONS options;
extern struct BACKLIGHT_SETTING backlight_setting;




void load_user_settings(void){
	
	//  Load example options structure to find initialized key
	struct OPTIONS tempOptions;
	getDefaultOptions(&tempOptions);
	
	//  Read options structure from EEPROM
	EEPROM_read(add_options, &options, sizeof(options));
	
	if (options.Settings_Initialized_Key != tempOptions.Settings_Initialized_Key){
		// Settings in EEPROM not initialized or are out of date
		//  Reload default settings into 
		getDefaultOptions(&options);
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
	EEPROM_read(add_c1_calst, &c1_calst, sizeof(a1_calst));
	EEPROM_read(add_c2_calst, &c2_calst, sizeof(a1_calst));
	EEPROM_read(add_dist_calst, &dist_calst, sizeof(a1_calst));
	EEPROM_read(add_cal_report_azm_inc, &cal_report_azm_inc, sizeof(cal_report_azm_inc));
	EEPROM_read(add_cal_report_dist, &cal_report_dist, sizeof(cal_report_azm_inc));
	
	// assume first struct is representative of remainder
	if(tempCal.Cal_Initialized_Key != a1_calst.Cal_Initialized_Key){
		//  EEPROM data has not been initialized or is out of date
		cal_init_struct(&a1_calst);
		cal_init_struct(&a2_calst);
		cal_init_struct(&c1_calst);
		cal_init_struct(&c2_calst);
		cal_init_struct(&dist_calst);
		save_calibration();		
	}
		
	
	
}

void save_calibration(void){

	uint8_t bytes_calst, bytes_report;
	
	bytes_report = sizeof(cal_report_azm_inc);
	bytes_calst = sizeof(a1_calst);
	
	// Save calibration structures
	EEPROM_write(add_a1_calst, &a1_calst, bytes_calst);
	EEPROM_write(add_a2_calst, &a2_calst, bytes_calst);
	EEPROM_write(add_c1_calst, &c1_calst, bytes_calst);
	EEPROM_write(add_c2_calst, &c2_calst, bytes_calst);
	EEPROM_write(add_dist_calst, &dist_calst, bytes_calst);
		
	// Save Calibration Report
	EEPROM_write(add_cal_report_azm_inc, &cal_report_azm_inc, bytes_report);
	EEPROM_write(add_cal_report_dist, &cal_report_dist, bytes_report);
	
	
}





void EEPROM_read(uint16_t data_address, uint8_t data_buf[], uint8_t bytes_to_read){
	uint16_t limit=20;
	uint16_t timeout;
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
	timeout=0;
	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) !=STATUS_OK) {
		if (timeout++ == limit) {   break;   }
	}
	//  Send read request to eeprom chip
	packet.data = data_buf;
	packet.data_length=bytes_to_read;
	timeout=0;
	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) !=STATUS_OK) {
		if (timeout++ == limit) {   break;   }
	}
	
	
	
	
}

void EEPROM_write(uint16_t address_init, uint8_t data_buf[], uint8_t bytes_to_write){
	#define page_size 32 // per datasheet for 24LC32A
	uint8_t packets;
	uint8_t k,p;
	uint8_t send_buf[page_size+2];
	uint8_t bytes_packet;
	uint16_t address_packet;
	struct i2c_master_packet packet;
	
	uint16_t limit=200;
	uint16_t timeout;
	//  Packet template
	packet.data = send_buf;
	packet.ten_bit_address = false;
	packet.high_speed = false;
	packet.hs_master_code = 0x0;
	packet.address = EEPROM_add;
	//  Determine number of packets
	packets = floor(bytes_to_write/page_size)+1;

	//  Iterate through packets
	for (p=0;p<packets;p++){
		//  Determine bytes to write in packet
		if (bytes_to_write>page_size){
			bytes_packet = page_size;
			bytes_to_write = bytes_to_write-page_size;
			}else{
			bytes_packet = bytes_to_write;
		}
		packet.data_length = bytes_packet + 2;// Include 16-bit address
		//  Set address
		address_packet = address_init + p*page_size;
		send_buf[0] = address_packet>>8;//high byte
		send_buf[1] = address_packet & 0x00FF;// low byte
		//  Copy data to buffer
		for (k=0;k<bytes_packet;k++){
			send_buf[k+2] = data_buf[p*page_size+k];
		}
		// Send Packet
		timeout=0;
		while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) !=STATUS_OK) {
			if (timeout++ == limit) {   break;   }
		}
		
	}
	
}
