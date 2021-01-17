/*
 * mathBRIC.h
 *
 * Created: 8/29/2020 5:32:36 PM
 *  Author: Kris Fausnight
 */ 


#ifndef MATHBRIC_H_
#define MATHBRIC_H_

#include <asf.h>
#include <stdint.h>
#include <math.h>
#include <calibration.h>


// Math functions
void circBuffInc(uint32_t *, uint32_t );
void circBuffDec(uint32_t *, uint32_t );
void inverse(float [][6], float [][6], uint8_t);
void transpose(float [][6], float [][6], float [][6], uint8_t);
float determinant(float [][6], uint8_t);
void calc_theta_XY(float [3], float *, float *);
void calc_azm_inc_roll_dec(float [3], float [3], float *, float *, float *, float *);
void rotvec_theta_ZY(float [3], float [3], float, float);
void rotvec_theta_XY(float [3], float [3], float, float);
float stdev(float [], uint32_t);
float meanArr(float [], uint32_t);
float calc_magnitude(float *);
float calc_mag_stdev(float [][3]);
float calc_disp_stdev(float [][3], float [][3], uint8_t);
void mat_mult_33_31(float [3][3], float [3], float [3]);
void calc_orientation(struct MEASUREMENT_FULL *);
float celsius2fahrenheit(float );
void copyMeasurement(struct MEASUREMENT *, struct MEASUREMENT_FULL * );


#endif /* MATHBRIC_H_ */