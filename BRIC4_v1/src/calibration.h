/*
 * calibration.h
 *
 * Created: 1/21/2019 5:14:23 PM
 *  Author: Kris Fausnight
 */ 



#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <stdint.h>
#include <sensors.h>
#include <math.h>
#include "FatFS_R13C\ff.h"
#include "FatFS_R13C\diskio.h"
#include <extern_clock\extern_clock.h>

#define nbuf		80
#define group_size	4// For Azm/Inc calibration
#define shot_size   4// For Distance calibration


struct INST_CAL{
	uint8_t Cal_Initialized_Key;//  Key to keep track of whether structure has been initialized
	float offset[3];
	float gain[3];
	float axmYX;
	float axmZY;
	float axmZX;
	float thetaX;
	float thetaY;
	float thetaZ;
	float RotM[3][3];
	float dist_offset;
	float angle_stdev;
	
} ;

struct CAL_REPORT{
	float software_version;
	uint32_t groups;
	uint32_t points;
	uint32_t timestamp;// posix time
	float inc_angle_err, azm_angle_err;
	float mag_stdev_a1, mag_stdev_a2, mag_stdev_c1, mag_stdev_c2;
	float disp_stdev_acc[3];
	float disp_stdev_comp[3];
	struct Time time_struct;	
	};

enum CAL_TYPE{gain_offset, RotM, all};



// Calibration Functions
void cal_process_calibration(void);
void cal_init_struct(struct INST_CAL *);
void cal_gain_off(float [][3], struct INST_CAL *);
void cal_angleYZ(float[][3], struct INST_CAL *);
void cal_angleX(float[][3], float[][3], struct INST_CAL *);
void cal_apply_cal(float [3], float [3], struct INST_CAL *);
void cal_add_datapoint(struct MEASUREMENT *, bool);
void cal_evaluate(void);
FRESULT cal_write_report(void);
void cal_add_dist(struct MEASUREMENT *);
void cal_dist_process(void);
void cal_loop_test(struct MEASUREMENT *);
void cal_axis_misalignments(float [][3], struct INST_CAL *);
void gen_RotM(struct INST_CAL *);

// Math functions
void inverse(float [][6], float [][6], uint8_t);
void transpose(float [][6], float [][6], float [][6], uint8_t);
float determinant(float [][6], uint8_t);
void calc_theta_XY(float [3], float *, float *);
void calc_azm_inc_roll_dec(float [3], float [3], float *, float *, float *, float *);
void rotvec_theta_ZY(float [3], float [3], float *, float *);
void rotvec_theta_XY(float [3], float [3], float *, float *);
float stdev(float [], uint32_t);
float calc_mag_stdev(float [][3]);
float calc_disp_stdev(float [][3], float [][3], uint8_t);
void mat_mult_33_31(float [3][3], float [3], float [3]);
void calc_orientation(struct MEASUREMENT *);





#endif /* CALIBRATION_H_ */