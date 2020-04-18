/*
 * sensors.c
 *
 * Created: 1/23/2019 7:19:25 PM
 *  Author: Kris Fausnight
 */ 
#include <sensors.h>
#include <backlight.h>


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
#define c_coarse_gain 7000
uint8_t cycle_count1=0x01; //cycle count for setup;
uint8_t cycle_count2=0x90; //0x0190 = 400d

//  Laser commands
#define laser_reset IOPORT_CREATE_PIN(IOPORT_PORTA, 2)
uint8_t cmd_laser_on[5]=		{0xAA, 0x00, 0x42, 0x42, 0xA8};
uint8_t cmd_laser_off[5]=		{0xAA, 0x00, 0x43, 0x43, 0xA8};
uint8_t cmd_laser_single[5]=	{0xAA, 0x00, 0x44, 0x44, 0xA8};
uint8_t cmd_beep_on[6]=			{0xAA, 0x00, 0x47, 0x01, 0x48, 0xA8};
uint8_t cmd_beep_off[6]=		{0xAA, 0x00, 0x47, 0x00, 0x48, 0xA8};




// Miscellaneous
extern float rad2deg, deg2rad, mt2ft;//  Conversion factor meters to feet, degrees to radians
extern struct INST_CAL a1_calst, a2_calst, c1_calst, c2_calst, dist_calst;
extern volatile bool laser_triggered;
#define max_samples 200 //  Maximum samples to collect before error
extern struct OPTIONS options;

//  Error Checking
extern struct CAL_REPORT cal_report_azm_inc;
#define errorSensitivityAdjustmentIncrement 0.25
#define errorSensitivityAdjustmentMin		 0.5
#define errorSensitivityAdjustmentMax		3

void adjustErrorSensitivity(void){
	options.errorSensitivity = options.errorSensitivity+errorSensitivityAdjustmentIncrement;
	
	if (options.errorSensitivity>errorSensitivityAdjustmentMax){
		options.errorSensitivity = errorSensitivityAdjustmentMin;
		
	}
	
	
	
}


void error_check(struct MEASUREMENT *meas_inst){
	float maga1, maga2, magc1, magc2, delta;
	float accel_err_limit, comp_err_limit;
	float azm_arr[4]; float inc_arr[4];
	float angMax, angMin;
	float foo1, foo2;
	uint8_t i;
	#define errlim_mag 200 // number of stdev's
	#define errlim_disp 200 // number of stdev's
	
	accel_err_limit = errlim_mag*max(cal_report_azm_inc.mag_stdev_a1, cal_report_azm_inc.mag_stdev_a2);
	comp_err_limit  = errlim_mag*max(cal_report_azm_inc.mag_stdev_c1, cal_report_azm_inc.mag_stdev_c2);
	
	accel_err_limit = 0.5;
	comp_err_limit  = 0.5;
	
	maga1 = calc_magnitude(meas_inst->a1xyz);
	maga2 = calc_magnitude(meas_inst->a2xyz);
	magc1 = calc_magnitude(meas_inst->c1xyz);
	magc2 = calc_magnitude(meas_inst->c2xyz);
		
	//  Magnitude Check accelerometer 1
	//mag = calc_magnitude(meas_inst->a1xyz);
	delta = fabs(maga1-1);
	if (fabs(maga1-1)>accel_err_limit)
	{
		meas_inst->measurement_error[meas_inst->num_errors] = accel1_mag_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = maga1;
		increment_error_count(meas_inst);
	}
	//  Magnitude Check accelerometer 2
	//mag = calc_magnitude(meas_inst->a2xyz);
	delta = fabs(maga2-1);
	if (delta>accel_err_limit)
	{
		meas_inst->measurement_error[meas_inst->num_errors] = accel2_mag_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = maga2;
		increment_error_count(meas_inst);
	}
	//  Magnitude Check Compass 1
	//mag = calc_magnitude(meas_inst->c1xyz);
	delta = fabs(magc1-1);
 	if (delta>comp_err_limit)
	{
		meas_inst->measurement_error[meas_inst->num_errors] = comp1_mag_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = magc1;
		increment_error_count(meas_inst);

	}
	//  Magnitude Check Compass 2
	//mag = calc_magnitude(meas_inst->c2xyz);
	delta = fabs(magc2-1);
	if (delta>comp_err_limit)
	{
		meas_inst->measurement_error[meas_inst->num_errors] = comp2_mag_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = magc2;
		increment_error_count(meas_inst);

	}
	
	accel_err_limit = errlim_disp*max(cal_report_azm_inc.mag_stdev_a1, cal_report_azm_inc.mag_stdev_a2);
	comp_err_limit  = errlim_disp*max(cal_report_azm_inc.mag_stdev_c1, cal_report_azm_inc.mag_stdev_c2);
	
	
	
	// Axis check, Accelerometer
	for (i=0;i<3;i++){
		//  Cycle through all 3 axis
		delta = fabs((meas_inst->a1xyz[i]/maga1) - (meas_inst->a2xyz[i]/maga2));
		accel_err_limit = errlim_disp*cal_report_azm_inc.disp_stdev_acc[i];
		accel_err_limit = 0.5;
		if (delta>accel_err_limit){
			meas_inst->measurement_error[meas_inst->num_errors] = accel_disp_err;
			meas_inst->measurement_error_data1[meas_inst->num_errors] = delta;
			meas_inst->measurement_error_data2[meas_inst->num_errors] = i+1;
			increment_error_count(meas_inst);
		}
		
	}
	
	// Axis check, Accelerometer
	for (i=0;i<3;i++){
		delta = fabs((meas_inst->c1xyz[i]/magc1) - (meas_inst->c2xyz[i]/magc2));
		comp_err_limit = errlim_mag*cal_report_azm_inc.disp_stdev_comp[i];
		comp_err_limit  = 0.5;
		if (delta>comp_err_limit){
			meas_inst->measurement_error[meas_inst->num_errors] = comp_disp_err;
			meas_inst->measurement_error_data1[meas_inst->num_errors] = delta;
			meas_inst->measurement_error_data2[meas_inst->num_errors] = i+1;
			increment_error_count(meas_inst);
		}
		
	}
	
	
	
	//  Check Angle Disparity
	calc_azm_inc_roll_dec(meas_inst->a1xyz, meas_inst->c1xyz, &azm_arr[0], &inc_arr[0], &foo1, &foo2);
	calc_azm_inc_roll_dec(meas_inst->a2xyz, meas_inst->c1xyz, &azm_arr[1], &inc_arr[1], &foo1, &foo2);
	calc_azm_inc_roll_dec(meas_inst->a1xyz, meas_inst->c2xyz, &azm_arr[2], &inc_arr[2], &foo1, &foo2);
	calc_azm_inc_roll_dec(meas_inst->a2xyz, meas_inst->c2xyz, &azm_arr[3], &inc_arr[3], &foo1, &foo2);
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
	delta = (angMax-angMin)*cos(meas_inst->inclination*deg2rad); //  Adjust for high angle shots
	if (delta>options.errorSensitivity){
		meas_inst->measurement_error[meas_inst->num_errors] = azm_ang_err;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = delta;
		increment_error_count(meas_inst);
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

float calc_magnitude(float xyz[3]){
	float magnitude;
	magnitude = sqrt(pow(xyz[0],2)+pow(xyz[1],2)+pow(xyz[2],2));
	return magnitude;	
}

void quick_measurement(struct MEASUREMENT *meas_inst){

	
	config_spi(sensors);
	
	read_accel(&slave_acc1, meas_inst->a1xyz);
	read_accel(&slave_acc2, meas_inst->a2xyz);
	read_mag(&slave_mag1, meas_inst->c1xyz);
	read_mag(&slave_mag2, meas_inst->c2xyz);
	config_spi(LCD);
	
	calc_orientation(meas_inst);
		
}






void full_measurement(struct MEASUREMENT *meas_inst, bool calibrate_data){
	uint8_t i;
	float a1temp[3], a2temp[3], c1temp[3], c2temp[3];
	
	//Delay
	laser_delay(options.shot_delay);//  Also adds beep
	
	//  Turn off backlight
	backlightOff();
	
	//  Configure SPI to talk to sensors
	config_spi(sensors);
	
	
	// Initialize structure
	for (i=0;i<3;i++){
		meas_inst->a1xyz[i] = 0;
		meas_inst->a2xyz[i] = 0;
		meas_inst->c1xyz[i] = 0;
		meas_inst->c2xyz[i] = 0;
	}
	meas_inst->num_errors = 0;
	meas_inst->samples = 0;
	
	//Initiate Laser Measurement	
	clear_rx_buffer();
	reception_complete=false;
	usart_write_buffer_job(&usart_laser, cmd_laser_single, sizeof(cmd_laser_single));
	usart_read_job(&usart_laser, &rx_buffer[rx_buffer_index]);
	do {
		//Take measurements while laser is responding
		read_accel(&slave_acc1,a1temp);
		read_accel(&slave_acc2, a2temp);
		read_mag(&slave_mag1, c1temp);
		read_mag(&slave_mag2, c2temp);
		for (i=0;i<3;i++){
			meas_inst->a1xyz[i] += a1temp[i];
			meas_inst->a2xyz[i] += a2temp[i];
			meas_inst->c1xyz[i] += c1temp[i];
			meas_inst->c2xyz[i] += c2temp[i];
		}
		meas_inst->samples += 1;
		if (meas_inst->samples > max_samples){
			usart_abort_job(&usart_laser, USART_TRANSCEIVER_RX);
			break;
		}
	}while(!reception_complete);
	
	// Parse Laser rangefinder data and populate measurement structure
	laser_parse_buffer(meas_inst);
	if (calibrate_data){
		// Note:  Laser rangefinder results always in meters
		//  distance offset is in meters
		meas_inst->distance = meas_inst->distance+dist_calst.dist_offset;
	}
	if (options.current_unit_dist == feet){
		meas_inst->distance = meas_inst->distance * mt2ft;//convert from meters to feet
		meas_inst->distance_units = feet;
	}else{
		meas_inst->distance_units = meters;
	}
	
	// Divide measurements by samples for average.
	for (i=0;i<3;i++){
		meas_inst->a1xyz[i] =meas_inst->a1xyz[i] / meas_inst->samples;
		meas_inst->a2xyz[i] =meas_inst->a2xyz[i] / meas_inst->samples;
		meas_inst->c1xyz[i] =meas_inst->c1xyz[i] / meas_inst->samples;
		meas_inst->c2xyz[i] =meas_inst->c2xyz[i] / meas_inst->samples;
	}
	//  Calibrate Results
	if (calibrate_data){
		cal_apply_cal(meas_inst->a1xyz, meas_inst->a1xyz, &a1_calst);
		cal_apply_cal(meas_inst->a2xyz, meas_inst->a2xyz, &a2_calst);
		cal_apply_cal(meas_inst->c1xyz, meas_inst->c1xyz, &c1_calst);
		cal_apply_cal(meas_inst->c2xyz, meas_inst->c2xyz, &c2_calst);
		
	}	
	// Calculate inclination and compass readings
	calc_orientation(meas_inst);
	
	// Perform Error Checking
	if (calibrate_data){//  Only perform error checking if data is calibrated
		error_check(meas_inst);
	}
	
	
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
	backlightOn();
	//  Configure SPI to speak to LCD	
	config_spi(LCD);	
}


void laser_delay(uint8_t shot_delay){
	uint8_t delay_count = 0;


	while(delay_count<shot_delay){
		laser_beep();
		delay_ms(800);
		delay_count = delay_count +1;
	}
	laser_beep();


}

void laser_beep(void){
	clear_rx_buffer();
	reception_complete=false;
	usart_write_buffer_job(&usart_laser, cmd_beep_on, sizeof(cmd_beep_on));
	usart_read_job(&usart_laser, &rx_buffer[rx_buffer_index]);
	while(!reception_complete);
	delay_ms(15);

}




void laser_parse_buffer(struct MEASUREMENT *meas_inst){
	uint32_t mult, temp1, temp_err;
	uint8_t mask = 0x0F;
	uint8_t i;
	uint8_t AA_index;
	
	
	AA_index=0xFF;
	for (i=0; i<20; i++){
		if(rx_buffer[i]==0xAA){
			AA_index=i;
			break;
		}
	}
	//parse data
	if(AA_index==0xFF){//No 0xAA initiate message, pattern error
		meas_inst->measurement_error[meas_inst->num_errors] = laser_pattern_error; 		
		meas_inst->measurement_error_data1[meas_inst->num_errors] = 0;
		increment_error_count(meas_inst);
		meas_inst->distance = 0;
	}else if(meas_inst->samples > max_samples){//timeout error
		meas_inst->measurement_error[meas_inst->num_errors] = laser_response_timeout;
		meas_inst->measurement_error_data1[meas_inst->num_errors] = 0;
		increment_error_count(meas_inst);
		meas_inst->distance = 0;
	}else if (rx_buffer[AA_index+3]==0x45){//rangefinder generated error
		temp_err = 0;
		mult=100;
		for(i=6; i<9; i++){
			temp1 = rx_buffer[AA_index+i] & mask;
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
		meas_inst->distance = 0;
	}else{
		//  No error, proceed with parsing string into distance measurement
		mult = 100000;
		temp1=0;
		meas_inst->distance = 0;
		for(i=3; i<9; i++){
			temp1=rx_buffer[AA_index+i] & mask;
			meas_inst->distance=meas_inst->distance + temp1*mult;
			mult=mult/10;
		}
		meas_inst->distance=meas_inst->distance/1000;
	}
}


void beep_on_off(bool on_off){
	write_complete = false;
	if(on_off){
		// beep on
		usart_write_buffer_job(&usart_laser, cmd_beep_on, 6);
		while(!write_complete);
	}else{
		// beep off
		usart_write_buffer_job(&usart_laser, cmd_beep_off, 6);
		while(!write_complete);
	}
}

void rangefinder_on_off(bool on_off){
	if (on_off){
		ioport_set_pin_level(laser_reset, true);
		delay_ms(100);
	}else{
		ioport_set_pin_level(laser_reset, false);
	}

	laser_triggered = false;

}


void laser_on_off(bool on_off){
	write_complete = false;
	if(on_off){
		usart_write_buffer_job(&usart_laser, cmd_laser_on, 5);
		while(!write_complete);
		laser_triggered = true;
		laser_timeout_timer(true);
	}else{
		usart_write_buffer_job(&usart_laser, cmd_laser_off, 5);
		while(!write_complete);
		laser_triggered = false;
		laser_timeout_timer(false);
	}
	
}




void read_accel(struct spi_slave_inst *const sensor, float vector[3]){
	uint8_t read_buffer[4];
	uint8_t i;
	float temp;
	//select acc1 chip
	// Assumes SPI already setup for sensors
	spi_select_slave(&spi_main, sensor, true);
	//clear out receive buffer
	spi_clear();
	//Send Read X command
	spi_transceive_buffer_wait(&spi_main, read_x, read_buffer, 4);
	spi_select_slave(&spi_main, sensor, false);
	//Send Read Y command, Read X
	spi_select_slave(&spi_main, sensor, true);
	spi_transceive_buffer_wait(&spi_main, read_y, read_buffer, 4);
	spi_select_slave(&spi_main, sensor, false);
	//Parse X data
	vector[0]=parse_acc_data(read_buffer);
	//Send Read Z command, Read Y
	spi_select_slave(&spi_main, sensor, true);
	spi_transceive_buffer_wait(&spi_main, read_z, read_buffer, 4);
	spi_select_slave(&spi_main, sensor, false);
	//Parse Y data
	vector[1]=parse_acc_data(read_buffer);
	//Send read status command (not used), read Z
	spi_select_slave(&spi_main, sensor, true);
	spi_transceive_buffer_wait(&spi_main, read_status, read_buffer, 4);
	spi_select_slave(&spi_main, sensor, false);
	//Parse Z data
	vector[2]=parse_acc_data(read_buffer);
	//Correct for sensor orientation
	temp=vector[1];
	vector[1]=vector[0];
	vector[0]=-1*temp;
	
	for (i=0;i<3;i++){
		vector[i] = vector[i]/a_coarse_gain;
	}
	
	
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
	config_spi(sensors);
	
	spi_select_slave(&spi_main, sensor, true);
	spi_clear();
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
	config_spi(LCD);
	
}




uint8_t read_mag(struct spi_slave_inst *const sensor, float vector[3]){
	uint8_t write_buffer[2];
	uint8_t read_buffer[9];
	uint8_t data_ready;
	uint8_t counter1;
	uint8_t i;
	//select sensor
	// Assumes SPI already set up for sensors
	spi_select_slave(&spi_main, sensor, true);
	spi_clear();
	//Send Send Poll command to 0x00
	write_buffer[0]=0x00; //poll register
	write_buffer[1]=0x70; //set to poll X,Y,Z
	spi_write_buffer_wait(&spi_main, write_buffer, 2);
	spi_select_slave(&spi_main, sensor, false);
	delay_us(1);
	data_ready=0x00;
	counter1 = 0x00;
	while(!data_ready){
		spi_select_slave(&spi_main, sensor, true);
		write_buffer[0]=0xB4;
		write_buffer[1]=0xFF;
		spi_transceive_buffer_wait(&spi_main, write_buffer, read_buffer, 2);
		data_ready=read_buffer[1];
		data_ready=data_ready & 0x80;
		counter1=counter1+1;
		spi_select_slave(&spi_main, sensor, false);
		delay_us(1);
		if(counter1==0xFF){break;}
	}
	spi_select_slave(&spi_main, sensor, true);
	write_buffer[0]=0xA4;
	spi_write_buffer_wait(&spi_main, write_buffer, 1);
	spi_read_buffer_wait(&spi_main, read_buffer, 9, 0xFF);
	spi_select_slave(&spi_main, sensor, false);
	delay_us(1);
	
	vector[0]=parse_mag_data(&read_buffer[0]);
	vector[1]=parse_mag_data(&read_buffer[3]);
	vector[2]=-1* parse_mag_data(&read_buffer[6]);//Z axis inverted
	
	for (i=0;i<3;i++){
		vector[i] = vector[i]/c_coarse_gain;
	}
	
	return counter1;
}

float parse_mag_data(uint8_t data[3]){
	float result;
	int32_t temp=0x00000000;
	if(data[0] & 0x80){//negative number
		temp=0xff;
		temp=temp<<8;
	}
	temp=temp+data[0];
	temp=temp<<8;
	temp=temp+data[1];
	temp=temp<<8;
	temp=temp+data[2];
	result=temp;
	return result;
}

void setup_mag(struct spi_slave_inst *const sensor){
	uint8_t write_buffer[7];
	//select sensor
	config_spi(sensors);
	spi_select_slave(&spi_main, sensor, true);
	spi_clear();
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
	config_spi(LCD);
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
			sprintf(err_str,"Acc delta ax%d: %0.3f%%", axis, 100*data1);
			break;
		case comp_disp_err:
			sprintf(err_str,"Cmp delta ax%d: %0.3f%%", axis, 100*data1);
			break;
		case inc_ang_err:
			sprintf(err_str,"Inc Delta: %0.3f deg", data1);
			break;
		case azm_ang_err:
			sprintf(err_str,"Azm Delta: %0.3f deg", data1);
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
		default:
			sprintf(err_str,"unrecognized error");	
	};
	
	
	
	
}

