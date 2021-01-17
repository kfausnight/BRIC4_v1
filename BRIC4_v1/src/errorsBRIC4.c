/*
 * errorsBRIC4.c
 *
 * Created: 10/11/2020 4:50:52 PM
 *  Author: Kris Fausnight
 */ 


#include <errorsBRIC4.h>


void gen_err_message(char *err_str, struct MEASUREMENT *meas_inst, uint8_t errN){

	float data1, data2;
	uint8_t axis;
	data1 = meas_inst->measurement_error_data1[errN];
	data2 = meas_inst->measurement_error_data2[errN];
	axis = data2;
	
	if (meas_inst->errCode[errN] == 0){
		*err_str = 0x00;
		return;
	}
	
	switch(meas_inst->errCode[errN]){
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
			sprintf(err_str,"laser error unknown, %f",data1);
			break;
		case laser_wrong_message:
			sprintf(err_str,"laser wrong message, %f",data1);
			break;
		default:
			sprintf(err_str,"Unknown error %d",meas_inst->errCode[errN]);
	};
	
	
	
	
}



void error_check(struct MEASUREMENT_FULL *meas_inst){
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
		increment_error_count(meas_inst,accel1_mag_err, magA1,0);
	}
	//  Magnitude Check accelerometer 2
	//mag = calc_magnitude(meas_inst->a2xyz);
	delta = fabs(magA2-1);
	if (delta>accel_err_limit)
	{
		increment_error_count(meas_inst, accel2_mag_err, magA2, 0 );
	}
	//  Magnitude Check Compass 1
	//mag = calc_magnitude(meas_inst->m1xyz);
	delta = fabs(magM1-1);
	if (delta>comp_err_limit)
	{
		increment_error_count(meas_inst, comp1_mag_err, magM1,0);
	}
	//  Magnitude Check Compass 2
	//mag = calc_magnitude(meas_inst->m2xyz);
	delta = fabs(magM2-1);
	if (delta>comp_err_limit)
	{
		increment_error_count(meas_inst, comp2_mag_err, magM2, 0);

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
			increment_error_count(meas_inst, accel_disp_err, delta, i+1);
		}
		
	}
	
	// Axis check, Compass
	for (i=0;i<3;i++){
		delta = fabs((meas_inst->m1Cal[i]/magM1) - (meas_inst->m2Cal[i]/magM2));
		comp_err_limit = errlim_mag*cal_report.disp_stdev_comp[i];
		comp_err_limit  = 0.5;
		if (delta>comp_err_limit){
			increment_error_count(meas_inst, comp_disp_err, delta, i+1);
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
		increment_error_count(meas_inst, inc_ang_err, delta, 0);
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
		increment_error_count(meas_inst, azm_ang_err, delta, 0);
	}
	
	
	
	
}



bool increment_error_count(struct MEASUREMENT_FULL *meas_inst, enum MEAS_ERROR_TYPE errCode, float data1, float data2)
{
	bool error_incremented = false;
	uint8_t i;
	
	for (i=0;i<MAX_ERRORS;i++){
		if(meas_inst->errCode[i]==0){
			meas_inst->errCode[i] = errCode;
			meas_inst->measurement_error_data1[i] = data1;
			meas_inst->measurement_error_data2[i] = data2;
			error_incremented = true;
			break;
		}
	}
	
	return error_incremented;
}



void adjustErrorSensitivity(void){
	options.errorSensitivity = options.errorSensitivity+STEP_ERROR_SENSITIVITY;
	
	if (options.errorSensitivity>MAX_ERROR_SENSITIVITY){
		options.errorSensitivity = STEP_ERROR_SENSITIVITY;
		
	}
	
}

