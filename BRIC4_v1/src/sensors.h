/*
 * sensors.h
 *
 * Created: 1/23/2019 7:16:41 PM
 *  Author: Kris Fausnight
 */ 




#ifndef SENSORS_H_
#define SENSORS_H_

#include <asf.h>
#include <math.h>
#include <mathBRIC.h>
#include <comms/comms.h>


// miscellaneous
void quick_measurement( struct MEASUREMENT *);
void full_measurement( struct MEASUREMENT *, uint8_t);
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
//uint8_t read_mag(struct spi_slave_inst *const, float *);
uint8_t read_mag_double( float *,float *);
void setup_mag(struct spi_slave_inst *const);
//float parse_mag_data(uint8_t *);
void parse_mag_arr(uint8_t *, float *);


//Laser Distance Functions
void laser_on_off(bool);
void beep_on_off(bool);
void rangefinder_on_off(bool );
void laser_parse_buffer(struct MEASUREMENT *meas_inst);
void laser_delay(uint8_t);
void laser_beep(void);
bool isLaserOn(void);


#include <main.h>


#endif /* SENSORS_H_ */