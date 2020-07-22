/*
 * sensors.h
 *
 * Created: 1/23/2019 7:16:41 PM
 *  Author: Kris Fausnight
 */ 




#ifndef SENSORS_H_
#define SENSORS_H_

#include <main.h>
#include <asf.h>
#include <comms/comms.h>
#include <calibration.h>
#include <clockSetup.h>
#include <timers.h>
#include <math.h>

enum measurement_error_type{
	no_error =			0,
	accel1_mag_err =	1,
	accel2_mag_err =	2,
	comp1_mag_err =		3,
	comp2_mag_err =		4,
	accel_disp_err =	5,
	comp_disp_err =		6,
	laser_calc_err=			7,
	laser_weak_signal	=	8,
	laser_strong_signal	=	9,
	laser_pattern_error	=	10,
	laser_response_timeout= 11,
	laser_unknown=			12,
	laser_wrong_message =	13,
	inc_ang_err=			14,
	azm_ang_err=			15,
	make8bit_measurement_error_type		= 0xff
};

#define max_errors 8
struct MEASUREMENT{
	uint32_t index_ref;//
	uint32_t posix_time;//  POSIX time, number of seconds since Jan 1 1970
	float temperature; // Temperature in celsius
	float azimuth, inclination, roll, declination, distance;// Processed readings
	float a1xyz[3];// raw data, accelerometer 1
	float a2xyz[3];// raw data, accelerometer 2
	float c1xyz[3];// raw data, compass 1
	float c2xyz[3];// raw data, compass 2	
	uint32_t samples;
	unit_type distance_units;
	unit_type temp_units;
	//  Time to take measurement
	uint32_t readTimeMs;	
	//  Measurement Error Data
	uint32_t num_errors;
	enum measurement_error_type measurement_error[max_errors];
	float measurement_error_data1[max_errors];
	float measurement_error_data2[max_errors];
	
};



// miscellaneous
void quick_measurement( struct MEASUREMENT *);
void full_measurement( struct MEASUREMENT *, bool);
void error_check(struct MEASUREMENT *);
void adjustErrorSensitivity(void);
bool increment_error_count(struct MEASUREMENT *);
float calc_magnitude(float *);
void gen_err_message(char *, struct MEASUREMENT *, uint8_t);

//Accelerometer functions
void setup_accel(struct spi_slave_inst *const);
void read_accel(struct spi_slave_inst *const, float *);
float parse_acc_data(uint8_t *);


//Compass functions
uint8_t read_mag(struct spi_slave_inst *const, float *);
void setup_mag(struct spi_slave_inst *const);
float parse_mag_data(uint8_t *);


//Laser Distance Functions
void laser_on_off(bool);
void beep_on_off(bool);
void rangefinder_on_off(bool );
void laser_parse_buffer(struct MEASUREMENT *meas_inst);
void laser_delay(uint8_t);
void laser_beep(void);


#endif /* SENSORS_H_ */