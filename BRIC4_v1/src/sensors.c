/*
 * sensors.c
 *
 * Created: 1/23/2019 7:19:25 PM
 *  Author: Kris Fausnight
 */ 
#include <sensors.h>


// Accelerometer Constants
#define a_coarse_gain 5000
uint8_t read_x[]=		{0x04,0x00,0x00,0xF7};
uint8_t read_y[]=		{0x08,0x00,0x00,0xFD};
uint8_t read_z[]=		{0x0C,0x00,0x00,0xFB};
uint8_t read_status[]=	{0x18,0x00,0x00,0xE5};
uint8_t read_temp[]=	{0x14,0x00,0x00,0xEF};
uint8_t	set_mode4[]=	{0xB4,0x00,0x03,0x38};
uint8_t	sw_reset[]=		{0xB4,0x00,0x20,0x98};
uint8_t read_whoami[]=	{0x40,0x00,0x00,0x91};

// Compass Constants
//#define c_coarse_gain 7000
//uint8_t cycle_count1=0x01; //cycle count for setup;
#define c_coarse_gain 10000
//uint8_t cycle_count1=0x01; //cycle count for setup;
//uint8_t cycle_count2=0x00; //0x0190 = 400d
uint8_t cycle_count1=0x02; //cycle count for setup;
uint8_t cycle_count2=0x00; //0x0190 = 400d

//  Laser commands
#define laser_reset IOPORT_CREATE_PIN(IOPORT_PORTA, 2)
char cmd_laser_on[5]=		{0xAA, 0x00, 0x42, 0x42, 0xA8};
char cmd_laser_off[5]=		{0xAA, 0x00, 0x43, 0x43, 0xA8};
char cmd_laser_single[5]=	{0xAA, 0x01, 0x44, 0x45, 0xA8};
char cmd_laser_cont[5]=		{0xAA, 0x01, 0x45, 0x46, 0xA8};
char cmd_beep_on[6]=			{0xAA, 0x00, 0x47, 0x01, 0x48, 0xA8};
char cmd_beep_off[6]=		{0xAA, 0x00, 0x47, 0x00, 0x48, 0xA8};

//  Status
volatile bool laserStatus = false; //  Variable to track when laser is turned on


bool isLaserOn(void){
	return laserStatus;
}

void laser_start_continuous(void){
	rangefinder_on_off(false);
	delay_ms(100);
	rangefinder_on_off(true);
	rxBufferLaserClear();
	writeLaser(cmd_beep_off, sizeof(cmd_laser_single));
	delay_ms(100);
	rxBufferLaserClear();
	writeLaser(cmd_laser_cont, sizeof(cmd_laser_single));
	
	
	
}

void quick_measurement(struct MEASUREMENT_FULL *meas_inst){
	
	read_accel(&slave_acc1, meas_inst->a1Raw);
	read_accel(&slave_acc2, meas_inst->a2Raw);
	read_mag_double(meas_inst->m1Raw, meas_inst->m2Raw);

	
	//  Calibrate Results
	cal_apply_cal(meas_inst->a1Raw, meas_inst->a1Cal, &a1_calst);
	cal_apply_cal(meas_inst->a2Raw, meas_inst->a2Cal, &a2_calst);
	cal_apply_cal(meas_inst->m1Raw, meas_inst->m1Cal, &m1_calst);
	cal_apply_cal(meas_inst->m2Raw, meas_inst->m2Cal, &m2_calst);

	//  Calculate instrument inclination, roll, azimuth, dip
	calc_orientation(meas_inst);
	
	meas_inst->refIndex = 0;
	meas_inst->posix_time = 0;
	meas_inst->errCode[0] = 0;
	
	meas_inst->meas_type = measQuick;
	
	//  Process Measurement
	processMeasurement(meas_inst);
}




void full_measurement(struct MEASUREMENT_FULL *meas_inst, uint8_t shot_delay, enum MEAS_TYPE measType){
	uint8_t i, temp;
	uint32_t refMs;
	float a1temp[3], a2temp[3], m1temp[3], m2temp[3];
	
	
	// Initialize structure
	for (i=0;i<3;i++){
		meas_inst->a1Raw[i] = 0;
		meas_inst->a2Raw[i] = 0;
		meas_inst->m1Raw[i] = 0;
		meas_inst->m2Raw[i] = 0;
	}
	for(i=0;i<MAX_ERRORS;i++){
		meas_inst->errCode[i]=0;
	}
	
	meas_inst->samples = 0;
	
	
	switch (measType){
		case measScan:
			//  Laser already triggered to continuous
			//  Nothing to do
			break;
		default:
			//  Turn off backlight
			backlightOff();
			//Delay
			laser_delay(shot_delay);//  Beep then delay for 1 second
			//Initiate Laser Measurement	
			rxBufferLaserClear();
			writeLaser(cmd_laser_single, sizeof(cmd_laser_single));
	}
	
	refMs = getCurrentMs(); //  Background clock running at 1000hz
	while(1){
		//Take measurements while laser is responding
		read_accel(&slave_acc1,a1temp);
		read_accel(&slave_acc2, a2temp);
		read_mag_double(m1temp, m2temp);
		for (i=0;i<3;i++){
			meas_inst->a1Raw[i] += a1temp[i];
			meas_inst->a2Raw[i] += a2temp[i];
			meas_inst->m1Raw[i] += m1temp[i];
			meas_inst->m2Raw[i] += m2temp[i];
		}
		meas_inst->samples += 1;
		
		meas_inst->readTimeMs = getCurrentMs()-refMs;
		if (meas_inst->readTimeMs > MEASUREMENT_TIMEOUT){
			//usart_abort_job(&usart_laser, USART_TRANSCEIVER_RX);
			break;
		}
		if (isLaserReceiveComplete()){
			temp = laserMessageType() ;
			if((temp==CONT_MEASUREMENT)||(temp== SINGLE_MEASUREMENT)){
				break;
			}else{
				rxBufferLaserClear();
			}
		}
	}
	
	// Parse Laser rangefinder data and populate measurement structure
	laser_parse_buffer(meas_inst);
	
	//  Calibrate Distance
	// Note:  Laser rangefinder results always in meters
	//  distance offset is in meters
	meas_inst->distMeters = meas_inst->distRaw+dist_calst.dist_offset;

	// Divide measurements by samples for average.
	for (i=0;i<3;i++){
		meas_inst->a1Raw[i] =meas_inst->a1Raw[i] / meas_inst->samples;
		meas_inst->a2Raw[i] =meas_inst->a2Raw[i] / meas_inst->samples;
		meas_inst->m1Raw[i] =meas_inst->m1Raw[i] / meas_inst->samples;
		meas_inst->m2Raw[i] =meas_inst->m2Raw[i] / meas_inst->samples;
	}
	//  Calibrate Results
	cal_apply_cal(meas_inst->a1Raw, meas_inst->a1Cal, &a1_calst);
	cal_apply_cal(meas_inst->a2Raw, meas_inst->a2Cal, &a2_calst);
	cal_apply_cal(meas_inst->m1Raw, meas_inst->m1Cal, &m1_calst);
	cal_apply_cal(meas_inst->m2Raw, meas_inst->m2Cal, &m2_calst);

	// Calculate inclination and compass readings
	calc_orientation(meas_inst);
	
	// Perform Error Checking
	error_check(meas_inst);
	
	// Add Time-Stamp
	get_time();
	memcpy(&meas_inst->measTime, &current_time,sizeof(current_time));
	meas_inst->posix_time = gen_posix_time(&current_time);// Save POSIX time
	
	//  Add Temperature
	meas_inst->temperatureC = currentTempC;
	
	// Add measurement type
	meas_inst->meas_type = measType;
	
	//  Add Reference Index
	switch (measType){
		case measRegular:
			meas_inst->refIndex = refIndex;
			//  Increment reference Index
			refIndex++;
			if (refIndex>REF_INDEX_MAX){
				refIndex = 1;
			}
			//  Turn backlight back on
			backlightOn(&options.backlight_setting);
			//  Provide feedback Beep
			if (meas_inst->errCode[0]==0){
				buzzOn(tone3,200);
			}else{
				buzzOn(tone1,200);
			}
			
			break;
		case measCal:
			meas_inst->refIndex = 0;
			//  Turn backlight back on
			backlightOn(&options.backlight_setting);
			buzzOn(tone3,200);
			break;
		case measScan:
			//  Add current reference but don't increment
			meas_inst->refIndex = refIndex;
			//  Provide feedback Beep
			if (meas_inst->errCode[0]==0){
				buzzOn(tone3,50);
			}else{
				buzzOn(tone1,50);
			}
			
			break;
		default:
			break;
	}
	
	
	//  Process Measurement
	//  Display Buffer, SD Card, and Bluetooth Handling
	processMeasurement(meas_inst);


}


void laser_delay(uint8_t shot_delay){
	uint8_t delay_count = 0;
	for (delay_count=0;delay_count<shot_delay;delay_count ++){
		buzzOn(tone2, 200);
		delay_ms(800);
	}
	

}



void laser_parse_buffer(struct MEASUREMENT_FULL *meas_inst){
	uint32_t mult, temp1, temp_err;
	uint8_t mask = 0x0F;
	uint8_t i;
	uint8_t AA_index;
	
	
	AA_index=0;
	for (i=0; i<sizeof(rxBufferLaser); i++){
		if(rxBufferLaser[i]==0xAA){
			AA_index=i;
			break;
		}
	}
	
	//parse data
	if(meas_inst->readTimeMs > MEASUREMENT_TIMEOUT){//timeout error
		increment_error_count(meas_inst, laser_response_timeout, MEASUREMENT_TIMEOUT, 0);
		meas_inst->distRaw = 0;
	}else if(i==sizeof(rxBufferLaser)){//No 0xAA initiate message, pattern error
		increment_error_count(meas_inst, laser_pattern_error, 0, 0);
		meas_inst->distRaw = 0;
	}else if(rxBufferLaser[AA_index+2]!=0x44){
		increment_error_count(meas_inst, laser_wrong_message, rxBufferLaser[AA_index+2], 0);
		meas_inst->distRaw = 0;	
	}else if (rxBufferLaser[AA_index+3]=='E'){//rangefinder generated error
		enum MEAS_ERROR_TYPE tempErrCode;
		float data1 = 0;
		temp_err = 0;
		mult=100;
		for(i=6; i<9; i++){
			temp1 = rxBufferLaser[AA_index+i] & mask;
			temp_err = temp_err + temp1*mult;
			mult=mult/10;
		}
		switch (temp_err){
			case 204:
				tempErrCode = laser_calc_err;
				break;
			case 255:
				tempErrCode = laser_weak_signal;
				break;
			case 256:
				tempErrCode = laser_strong_signal;
				break;
			default:
				tempErrCode = laser_unknown;
				data1 = temp_err;
				
		}
		increment_error_count(meas_inst, tempErrCode,data1, 0);
		meas_inst->distRaw = 0;
	}else{
		//  No error, proceed with parsing string into distance measurement
		mult = 100000;
		temp1=0;
		meas_inst->distRaw = 0;
		for(i=3; i<9; i++){
			temp1=rxBufferLaser[AA_index+i] & mask;
			meas_inst->distRaw=meas_inst->distRaw + temp1*mult;
			mult=mult/10;
		}
		meas_inst->distRaw=meas_inst->distRaw/1000;
	}
	rxBufferLaserClear();
}



void rangefinder_on_off(bool on_off){
	if (on_off){
		ioport_set_pin_level(laser_reset, true);
		delay_ms(100);
	}else{
		ioport_set_pin_level(laser_reset, false);
	}

	laserStatus = false;

}


void laser_on_off(bool on_off){
	//write_complete = false;
	if(on_off){
		writeLaser(cmd_laser_on, sizeof(cmd_laser_on));
		//while(!isLaserTransmitComplete());
		laserStatus = true;
		buzzOn(tone2, 100);
	}else{
		writeLaser(cmd_laser_off, sizeof(cmd_laser_off));
		//usart_write_buffer_job(&usart_laser, cmd_laser_off, 5);
		//while(!isLaserTransmitComplete());
		laserStatus = false;
	}
	
}




void read_accel(struct spi_slave_inst *const sensor, float vector[3]){
	#define accelMessLength	4
	static uint16_t length = accelMessLength;
	uint8_t read_buffer[accelMessLength];
	float tempV[3];
	uint8_t i;
	
	//select acc1 chip
	// Assumes SPI already setup for sensors
	spi_select_slave(&spi_main, sensor, true);


	//Send Read X command
	spi_transceive_buffer_wait(&spi_main, read_x, read_buffer, length);
	spi_select_slave(&spi_main, sensor, false);
	//Send Read Y command, Read X
	spi_select_slave(&spi_main, sensor, true);
	spi_transceive_buffer_wait(&spi_main, read_y, read_buffer, length);
	spi_select_slave(&spi_main, sensor, false);
	//Parse X data
	tempV[0]=parse_acc_data(read_buffer);
	//Send Read Z command, Read Y
	spi_select_slave(&spi_main, sensor, true);
	spi_transceive_buffer_wait(&spi_main, read_z, read_buffer, length);
	spi_select_slave(&spi_main, sensor, false);
	//Parse Y data
	tempV[1]=parse_acc_data(read_buffer);
	//Send read status command (not used), read Z
	spi_select_slave(&spi_main, sensor, true);
	spi_transceive_buffer_wait(&spi_main, read_status, read_buffer, length);
	spi_select_slave(&spi_main, sensor, false);
	//Parse Z data
	tempV[2]=parse_acc_data(read_buffer);
	
	for (i=0;i<3;i++){
		tempV[i] = tempV[i]/a_coarse_gain;
	}
	
	//Correct for sensor orientation
	vector[0] = tempV[1];
	vector[1] = -1*tempV[0];
	vector[2] = -1*tempV[2];
	//temp=vector[1];
	//vector[1]=vector[0];
	//vector[0]=-1*temp;
	
	
	
	
}




float parse_acc_data(uint8_t buffer[4]){
	float result=0;
	int16_t var16;
	var16= buffer[1];
	var16=var16<<8;
	var16=var16+buffer[2];
	result=var16;
	return result;
}


void setup_accel(struct spi_slave_inst *const sensor){
	uint8_t read_buffer[4];
	
	//select acc1 chip
	//config_spi(sensors);
	
	spi_select_slave(&spi_main, sensor, true);
	//sw reset
	spi_transceive_buffer_wait(&spi_main, sw_reset, read_buffer, 4);
	//toggle CS line
	spi_select_slave(&spi_main, sensor, false);
	delay_ms(10);
	spi_select_slave(&spi_main, sensor, true);
	//set mode
	spi_transceive_buffer_wait(&spi_main, set_mode4, read_buffer, 4);
	//toggle CS line
	spi_select_slave(&spi_main, sensor, false);
	//config_spi(LCD);
	
}



uint8_t read_mag_double( float mag1[3],float mag2[3]){
	uint8_t write_buffer[10];
	uint8_t read_buffer[10];
	uint8_t counter1;
	uint8_t i;
	bool data_ready;
	
	float *vecPtr[2];
	struct spi_slave_inst *slavePtr[2];
	vecPtr[0] = mag1;
	vecPtr[1] = mag2;
	slavePtr[0] = &slave_mag1;
	slavePtr[1] = &slave_mag2;
	
	// Poll XYZ command to address 0x00
	write_buffer[0]=0x00; //poll register
	write_buffer[1]=0x70; //set to poll X,Y,Z	
	for (i=0;i<2;i++){
		// Select Magnetometer 
		spi_select_slave(&spi_main, slavePtr[i], true);
		delay_us(1);
		//Send Send Poll XYZ command to 0x00
		spi_write_buffer_wait(&spi_main, write_buffer, 2);
		spi_select_slave(&spi_main, slavePtr[i], false);
		
	}
	//  Wait for Data to be ready
	counter1 = 0x00;
	write_buffer[0]=0xB4;
	//  First poll mag1
	for (i=0;i<2;i++){
		
		data_ready = false;
		while(!data_ready){
			delay_us(20);
			spi_select_slave(&spi_main, slavePtr[i], true);
			delay_us(1);
			spi_transceive_buffer_wait(&spi_main, write_buffer, read_buffer, 2);
			spi_select_slave(&spi_main, slavePtr[i], false);
			
			//  Check if data is ready
			if (read_buffer[1] & 0x80){
				data_ready = true;
			}
			
			//  Check for time-out
			counter1++;
			if (counter1>=0xFF){
				break;
			}
		}//  Loop checking for data ready
	}//  Loop for each instrument
	
	//  Read Back Data
	write_buffer[0] = 0xA4;	
	for (i=0;i<2; i++){
		spi_select_slave(&spi_main, slavePtr[i], true);
		delay_us(1);		
		spi_transceive_buffer_wait(&spi_main, write_buffer, read_buffer, 10);
		spi_select_slave(&spi_main, slavePtr[i], false);
		
		parse_mag_arr(&read_buffer[1], vecPtr[i]);
	
	}
	debug_ct1 = counter1;
	return counter1;
	
}

void  parse_mag_arr(uint8_t array[9], float data[3]){
	uint8_t i;
	int32_t temp;
	
	for (i=0;i<3;i++){
		temp = 0x00;
		if(array[i*3] & 0x80){//negative number
			temp=0xff;
			temp=temp<<8;
		}
		temp=temp+array[i*3];
		temp=temp<<8;
		temp=temp+array[i*3+1];
		temp=temp<<8;
		temp=temp+array[i*3+2];
		data[i] = temp;
		if (i==2){
			//  Z axis inverted
			data[i] = -1*data[i];
		}
		
		//  Apply coarse gain		
		data[i] = data[i]/c_coarse_gain;
		
	}
}



void setup_mag(struct spi_slave_inst *const sensor){
	uint8_t write_buffer[7];
	//select sensor
	//config_spi(sensors);
	spi_select_slave(&spi_main, sensor, true);

	//Set cycle count registers
	write_buffer[0]=0x04;//location of first write count register
	write_buffer[1]=cycle_count1;
	write_buffer[2]=cycle_count2;
	write_buffer[3]=cycle_count1;
	write_buffer[4]=cycle_count2;
	write_buffer[5]=cycle_count1;
	write_buffer[6]=cycle_count2;
	spi_write_buffer_wait(&spi_main, write_buffer, 7);
	spi_select_slave(&spi_main, sensor, false);
	//Turn off continuous read mode
	delay_ms(10);
	write_buffer[0]=0x01;//location of CRM register
	write_buffer[1]=0x00;
	spi_select_slave(&spi_main, sensor, true);
	spi_write_buffer_wait(&spi_main, write_buffer, 2);
	spi_select_slave(&spi_main, sensor, false);
	//Set BIST register
	delay_us(1);
	write_buffer[0]=0x33;//location of BIST register
	write_buffer[1]=0x00;	//default value
	spi_select_slave(&spi_main, sensor, true);
	spi_write_buffer_wait(&spi_main, write_buffer, 2);
	spi_select_slave(&spi_main, sensor, false);
	//SET HSHAKe register
	delay_us(1);
	write_buffer[0]=0x35;//location of HSHAKE register
	write_buffer[1]=0x1B;//default value
	spi_select_slave(&spi_main, sensor, true);
	spi_write_buffer_wait(&spi_main, write_buffer, 2);
	spi_select_slave(&spi_main, sensor, false);
	//config_spi(LCD);
}









