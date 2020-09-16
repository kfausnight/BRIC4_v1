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
uint8_t cmd_laser_on[5]=		{0xAA, 0x00, 0x42, 0x42, 0xA8};
uint8_t cmd_laser_off[5]=		{0xAA, 0x00, 0x43, 0x43, 0xA8};
uint8_t cmd_laser_single[5]=	{0xAA, 0x00, 0x44, 0x44, 0xA8};
uint8_t cmd_beep_on[6]=			{0xAA, 0x00, 0x47, 0x01, 0x48, 0xA8};
uint8_t cmd_beep_off[6]=		{0xAA, 0x00, 0x47, 0x00, 0x48, 0xA8};

//  Status
volatile bool laserStatus = false; //  Variable to track when laser is turned on


bool isLaserOn(void){
	return laserStatus;
}

float calc_magnitude(float xyz[3]){
	float magnitude;
	magnitude = sqrt(pow(xyz[0],2)+pow(xyz[1],2)+pow(xyz[2],2));
	return magnitude;	
}

void quick_measurement(struct MEASUREMENT *meas_inst){
	
	read_accel(&slave_acc1, meas_inst->a1Raw);
	read_accel(&slave_acc2, meas_inst->a2Raw);
	read_mag_double(meas_inst->m1Raw, meas_inst->m2Raw);

	
	//  Calibrate Results
	cal_apply_cal(meas_inst->a1Raw, meas_inst->a1Cal, &a1_calst);
	cal_apply_cal(meas_inst->a2Raw, meas_inst->a2Cal, &a2_calst);
	cal_apply_cal(meas_inst->m1Raw, meas_inst->m1Cal, &m1_calst);
	cal_apply_cal(meas_inst->m2Raw, meas_inst->m2Cal, &m2_calst);

	
		

	//  Calculate instrument inclination, roll, azimuth, declination
	calc_orientation(meas_inst);
		
}






void full_measurement(struct MEASUREMENT *meas_inst, uint8_t shot_delay){
	uint8_t i;
	uint32_t refMs;
	enum LASER_MESSAGE_TYPE debugLM;
	float a1temp[3], a2temp[3], m1temp[3], m2temp[3];
	
	//Delay
	laser_delay(shot_delay);//  Beep then delay for 1 second
	laser_beep();
	delay_ms(100);//  Avoids cutting off the beep too quickly with another command
	
	//  Turn off backlight
	backlightOff();
	
	//  Configure SPI to talk to sensors
	//config_spi(sensors);
		
	// Initialize structure
	for (i=0;i<3;i++){
		meas_inst->a1Raw[i] = 0;
		meas_inst->a2Raw[i] = 0;
		meas_inst->m1Raw[i] = 0;
		meas_inst->m2Raw[i] = 0;
	}
	meas_inst->num_errors = 0;
	meas_inst->samples = 0;
	
	//Initiate Laser Measurement	
	rxBufferLaserClear();
	writeLaser(cmd_laser_single, sizeof(cmd_laser_single));
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
			if(laserMessageType()==SINGLE_MEASUREMENT){
				break;
			}
		}
	}
	
	// Parse Laser rangefinder data and populate measurement structure
	laser_parse_buffer(meas_inst);
	
	//  Calibrate Distance
	// Note:  Laser rangefinder results always in meters
	//  distance offset is in meters
	meas_inst->distCal = meas_inst->distRaw+dist_calst.dist_offset;
	if (options.current_unit_dist == feet){
		meas_inst->distRaw = meas_inst->distRaw * MT2FT;//convert from meters to feet
		meas_inst->distCal = meas_inst->distCal * MT2FT;//convert from meters to feet
		meas_inst->distance_units = feet;
	}else{
		meas_inst->distance_units = meters;
	}
	
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
	meas_inst->posix_time = gen_posix_time(&current_time);// Save POSIX time
	
	//  Add Temperature
	if (options.current_unit_temp == celsius){
		meas_inst->temperature = current_time.temperatureC;
		meas_inst->temp_units = celsius;
	}else{
		meas_inst->temperature = current_time.temperatureF;
		meas_inst->temp_units = fahrenheit;
	}	
	
	//  Turn backlight back on
	backlightOn(&options.backlight_setting);

}


void laser_delay(uint8_t shot_delay){
	uint8_t delay_count = 0;
	for (delay_count=0;delay_count<shot_delay;delay_count ++){
		laser_beep();
		delay_ms(900);
	}
	

}

void laser_beep(void){
	
	rxBufferLaserClear();
	writeLaser(cmd_beep_on, sizeof(cmd_beep_on));
	//  Wait for laser to finish sending beep response .
	while(!isLaserReceiveComplete());
}




void laser_parse_buffer(struct MEASUREMENT *meas_inst){
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
	if(i==sizeof(rxBufferLaser)){//No 0xAA initiate message, pattern error
		meas_inst->measurement_error[meas_inst->num_errors] = laser_pattern_error; 		
		meas_inst->measurement_error_data1[meas_inst->num_errors] = 0;
		increment_error_count(meas_inst);
		meas_inst->distRaw = 0;
		return;
	}else if(meas_inst->readTimeMs > MEASUREMENT_TIMEOUT){//timeout error
		meas_inst->measurement_error[meas_inst->num_errors] = laser_response_timeout;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = meas_inst->samples;
		increment_error_count(meas_inst);
		meas_inst->distRaw = 0;
		return;
	}else if(rxBufferLaser[AA_index+2]!=0x44){
		meas_inst->measurement_error[meas_inst->num_errors] = laser_wrong_message;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = rxBufferLaser[AA_index+2];
		increment_error_count(meas_inst);
		meas_inst->distRaw = 0;
		return;		
	}else if (rxBufferLaser[AA_index+3]=='E'){//rangefinder generated error
		temp_err = 0;
		mult=100;
		for(i=6; i<9; i++){
			temp1 = rxBufferLaser[AA_index+i] & mask;
			temp_err = temp_err + temp1*mult;
			mult=mult/10;
		}
		switch (temp_err){
			case 204:
				meas_inst->measurement_error[meas_inst->num_errors] = laser_calc_err;
			break;
			case 255:
				meas_inst->measurement_error[meas_inst->num_errors] = laser_weak_signal;
			break;
			case 256:
				meas_inst->measurement_error[meas_inst->num_errors] = laser_strong_signal;
			break;
			default:
				meas_inst->measurement_error[meas_inst->num_errors] = laser_unknown;
		}
		meas_inst->measurement_error_data1[meas_inst->num_errors] = 0;
		increment_error_count(meas_inst);
		meas_inst->distRaw = 0;
		return;
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
}


void beep_on_off(bool on_off){
	//write_complete = false;
	if(on_off){
		// beep on
		writeLaser(cmd_beep_on, sizeof(cmd_beep_on));

		//while(!write_complete);
	}else{
		// beep off
		writeLaser(cmd_beep_off, sizeof(cmd_beep_off));
		//while(!write_complete);
	}
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
		while(!isLaserTransmitComplete());
		laserStatus = true;
		//laser_timeout_timer(true);
	}else{
		writeLaser(cmd_laser_off, sizeof(cmd_laser_off));
		//usart_write_buffer_job(&usart_laser, cmd_laser_off, 5);
		while(!isLaserTransmitComplete());
		laserStatus = false;
		//laser_timeout_timer(false);
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




void adjustErrorSensitivity(void){
	options.errorSensitivity = options.errorSensitivity+STEP_ERROR_SENSITIVITY;
	
	if (options.errorSensitivity>MAX_ERROR_SENSITIVITY){
		options.errorSensitivity = STEP_ERROR_SENSITIVITY;
		
	}
	
}

bool increment_error_count(struct MEASUREMENT *meas_inst)
{
	uint32_t array_max;
	bool error_incremented;
	array_max = sizeof(meas_inst->measurement_error)/sizeof(meas_inst->measurement_error[0]);
	if 	((meas_inst->num_errors+1)<array_max){
		meas_inst->num_errors++;
		error_incremented = true;
	}else{
		error_incremented = false;
	}
	return error_incremented;
}

void error_check(struct MEASUREMENT *meas_inst){
	float magA1, magA2, magM1, magM2, delta;
	float accel_err_limit, comp_err_limit;
	float azm_arr[4]; float inc_arr[4];
	float angMax, angMin;
	float foo1, foo2;
	uint8_t i;
	#define errlim_mag 200 // number of stdev's
	#define errlim_disp 200 // number of stdev's
	
	accel_err_limit = errlim_mag*max(cal_report.mag_stdev_a1, cal_report.mag_stdev_a2);
	comp_err_limit  = errlim_mag*max(cal_report.mag_stdev_m1, cal_report.mag_stdev_m2);
	
	accel_err_limit = 0.5;
	comp_err_limit  = 0.5;
	
	magA1 = calc_magnitude(meas_inst->a1Cal);
	magA2 = calc_magnitude(meas_inst->a2Cal);
	magM1 = calc_magnitude(meas_inst->m1Cal);
	magM2 = calc_magnitude(meas_inst->m2Cal);
		
	//  Magnitude Check accelerometer 1
	//mag = calc_magnitude(meas_inst->a1xyz);
	delta = fabs(magA1-1);
	if (fabs(magA1-1)>accel_err_limit)
	{
		meas_inst->measurement_error[meas_inst->num_errors] = accel1_mag_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = magA1;
		increment_error_count(meas_inst);
	}
	//  Magnitude Check accelerometer 2
	//mag = calc_magnitude(meas_inst->a2xyz);
	delta = fabs(magA2-1);
	if (delta>accel_err_limit)
	{
		meas_inst->measurement_error[meas_inst->num_errors] = accel2_mag_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = magA2;
		increment_error_count(meas_inst);
	}
	//  Magnitude Check Compass 1
	//mag = calc_magnitude(meas_inst->m1xyz);
	delta = fabs(magM1-1);
 	if (delta>comp_err_limit)
	{
		meas_inst->measurement_error[meas_inst->num_errors] = comp1_mag_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = magM1;
		increment_error_count(meas_inst);

	}
	//  Magnitude Check Compass 2
	//mag = calc_magnitude(meas_inst->m2xyz);
	delta = fabs(magM2-1);
	if (delta>comp_err_limit)
	{
		meas_inst->measurement_error[meas_inst->num_errors] = comp2_mag_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = magM2;
		increment_error_count(meas_inst);

	}
	
	accel_err_limit = errlim_disp*max(cal_report.mag_stdev_a1, cal_report.mag_stdev_a2);
	comp_err_limit  = errlim_disp*max(cal_report.mag_stdev_m1, cal_report.mag_stdev_m2);
	
	
	
	// Axis check, Accelerometer
	for (i=0;i<3;i++){
		//  Cycle through all 3 axis
		delta = fabs((meas_inst->a1Cal[i]/magA1) - (meas_inst->a2Cal[i]/magA2));
		accel_err_limit = errlim_disp*cal_report.disp_stdev_acc[i];
		accel_err_limit = 0.5;
		if (delta>accel_err_limit){
			meas_inst->measurement_error[meas_inst->num_errors] = accel_disp_err;
			meas_inst->measurement_error_data1[meas_inst->num_errors] = delta;
			meas_inst->measurement_error_data2[meas_inst->num_errors] = i+1;
			increment_error_count(meas_inst);
		}
		
	}
	
	// Axis check, Compass
	for (i=0;i<3;i++){
		delta = fabs((meas_inst->m1Cal[i]/magM1) - (meas_inst->m2Cal[i]/magM2));
		comp_err_limit = errlim_mag*cal_report.disp_stdev_comp[i];
		comp_err_limit  = 0.5;
		if (delta>comp_err_limit){
			meas_inst->measurement_error[meas_inst->num_errors] = comp_disp_err;
			meas_inst->measurement_error_data1[meas_inst->num_errors] = delta;
			meas_inst->measurement_error_data2[meas_inst->num_errors] = i+1;
			increment_error_count(meas_inst);
		}
		
	}
	
	
	
	//  Check Angle Disparity
	calc_azm_inc_roll_dec(meas_inst->a1Cal, meas_inst->m1Cal, &azm_arr[0], &inc_arr[0], &foo1, &foo2);
	calc_azm_inc_roll_dec(meas_inst->a2Cal, meas_inst->m1Cal, &azm_arr[1], &inc_arr[1], &foo1, &foo2);
	calc_azm_inc_roll_dec(meas_inst->a1Cal, meas_inst->m2Cal, &azm_arr[2], &inc_arr[2], &foo1, &foo2);
	calc_azm_inc_roll_dec(meas_inst->a2Cal, meas_inst->m2Cal, &azm_arr[3], &inc_arr[3], &foo1, &foo2);
	//  Check Inclinometer
	angMin = inc_arr[0]; angMax = inc_arr[0];
	for (i=1;i<4;i++){
		angMin = min(angMin, inc_arr[i]);
		angMax = max(angMax, inc_arr[i]);
	}	
	delta = angMax-angMin;
	if (delta>options.errorSensitivity){
		meas_inst->measurement_error[meas_inst->num_errors] = inc_ang_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = delta;
		increment_error_count(meas_inst);
	}
	// Check Compass
	//  Check for possible angle wrap-around
	bool wrapFlag = false;
	for (i=0;i<4;i++){
		if (azm_arr[i]<90){ wrapFlag = true;}
	}
	if (wrapFlag){
		for (i=0;i<4;i++){
			if (azm_arr[i]>270){ azm_arr[i]= azm_arr[i]-360;}
		}
	}	
	angMin = azm_arr[0]; angMax = azm_arr[0]; 
	for (i=1;i<4;i++){
		angMin = min(angMin, azm_arr[i]);
		angMax = max(angMax, azm_arr[i]);
	}
	delta = (angMax-angMin)*cos(meas_inst->inclination*DEG2RAD); //  Adjust for high angle shots
	if (delta>options.errorSensitivity){
		meas_inst->measurement_error[meas_inst->num_errors] = azm_ang_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = delta;
		increment_error_count(meas_inst);
	}
	
	
	
	
}



void gen_err_message(char *err_str, struct MEASUREMENT *meas_inst, uint8_t errN){
	float data1, data2;
	uint8_t axis;
	data1 = meas_inst->measurement_error_data1[errN];
	data2 = meas_inst->measurement_error_data2[errN];
	axis = data2;
	
	switch(meas_inst->measurement_error[errN]){
		case accel1_mag_err:
			if (data1>1){sprintf(err_str,"Acc1 High: %0.4f", data1);}
			else{sprintf(err_str,"Acc1 Low: %0.4f", data1);}
			break;
		case accel2_mag_err:
			if (data1>1){sprintf(err_str,"Acc2 High: %0.4f", data1);}
			else{sprintf(err_str,"Acc2 Low: %0.4f", data1);}
			break;
		case comp1_mag_err:
			if (data1>1){sprintf(err_str,"Comp1 High: %0.4f", data1);}
			else{sprintf(err_str,"Comp1 Low: %0.4f", data1);}
			break;
		case comp2_mag_err:
			if (data1>1){sprintf(err_str,"Comp2 High: %0.4f", data1);}
			else{sprintf(err_str,"Comp2 Low: %0.4f", data1);}
			break;
		case accel_disp_err:
			sprintf(err_str,"Acc delta ax%d %0.3f%%", axis, 100*data1);
			break;
		case comp_disp_err:
			sprintf(err_str,"Mag delta ax%d %0.3f%%", axis, 100*data1);
			break;
		case inc_ang_err:
			sprintf(err_str,"Inc Delta: %0.3fdeg", data1);
			break;
		case azm_ang_err:
			sprintf(err_str,"Azm Delta: %0.3fdeg", data1);
			break;
		case laser_calc_err:
			sprintf(err_str,"laser calc error");
			break;
		case laser_weak_signal:
			sprintf(err_str,"laser weak signal");
			break;
		case laser_strong_signal:
			sprintf(err_str,"laser strong signal");
			break;
		case laser_response_timeout:
			sprintf(err_str,"laser comm timeout");
			break;
		case laser_unknown:
			sprintf(err_str,"laser error, unknown");
			break;
		case laser_wrong_message:
			sprintf(err_str,"laser wrong message");
			break;
		default:
			sprintf(err_str,"unknown error %d",meas_inst->measurement_error[errN]);	
	};
	
	
	
	
}

