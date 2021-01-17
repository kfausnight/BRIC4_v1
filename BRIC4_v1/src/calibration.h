/*
 * calibration.h
 *
 * Created: 1/21/2019 5:14:23 PM
 *  Author: Kris Fausnight
 */ 



#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <timers.h>
#include <sensors.h>

#include <stdint.h>
#include <math.h>



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
	
} ;

struct CAL_REPORT{
	float software_version;
	uint32_t groups;
	uint32_t points;
	uint32_t groupsAll;
	uint32_t pointsAll;
	//  Group auto-removal
	uint8_t groupRemoved[MAX_BAD_GROUPS];
	uint8_t groupRemovedSource[MAX_BAD_GROUPS];
	float groupRemovedImprovement[MAX_BAD_GROUPS];
	float inc_angle_err, azm_angle_err;
	float mag_stdev_a1, mag_stdev_a2, mag_stdev_m1, mag_stdev_m2;
	float disp_stdev_acc[3];
	float disp_stdev_comp[3];
	struct TIME time_inc_azm;	
	struct TIME time_quick_azm;	
	struct TIME time_rangeFinder;	
	float tempC_inc_azm;
	float tempC_quick_azm;
	float tempC_rangeFinder;
	
};



// Calibration Functions
uint8_t cal_getGroupPoints(void);
uint8_t cal_getCurrentGroup(void);
void cal_init(void);
void cal_resetGroup(void);
uint32_t cal_removeGroup(bool *, uint32_t);
uint8_t cal_findBadGroup(float [], float [], uint32_t *, float *);
bool cal_checkStability(float [][3], float [3]);
bool cal_azm_quick_add_point(float [][3], float [][3], uint32_t);
void cal_init_struct(struct INST_CAL *);
void cal_gain_off(float [][3], struct INST_CAL *);
void cal_angleYZ(float[][3], struct INST_CAL *);
void cal_angleX(float[][3], float[][3], struct INST_CAL *);
void cal_apply_cal(float [3], float [3], struct INST_CAL *);
void cal_apply_cal_all(void);
void cal_add_datapoint(struct MEASUREMENT_FULL *);
void cal_inc_azm_eval(void);
void cal_full_inc_azm_process(uint8_t);
void cal_azm_quick_process(void);
void cal_done(enum CALTYPE);
void cal_add_dist(struct MEASUREMENT_FULL *);
void cal_loop_test(struct MEASUREMENT_FULL *);
void cal_axis_misalignments(float [][3], struct INST_CAL *);

void gen_RotM(struct INST_CAL *);





#endif /* CALIBRATION_H_ */