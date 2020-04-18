/*
 * calibration.c
 *
 * Created: 1/21/2019 5:16:02 PM
 *  Author: Kris Fausnight
 */ 
#include <calibration.h>


#define nbuf		80
#define group_size	4// For Azm/Inc calibration
#define shot_size   4// For Distance calibration

#define delta_min	10



//  External Global Data
// Azm/Inc Calibration
extern uint32_t n_groups;
extern uint32_t n_points;
extern float a1raw[nbuf][3], a2raw[nbuf][3], c1raw[nbuf][3], c2raw[nbuf][3];
extern float a1cal[nbuf][3], a2cal[nbuf][3], c1cal[nbuf][3], c2cal[nbuf][3];
extern uint8_t ind_buf, ind_stack, buf_points;
extern struct INST_CAL a1_calst, a2_calst, c1_calst, c2_calst, dist_calst;
extern struct CAL_REPORT cal_report_azm_inc, cal_report_dist;
extern float azimuth[nbuf], inclination[nbuf], roll[nbuf];
extern float software_version;
extern float rad2deg, deg2rad, mt2ft;
//  Distance calibration
extern float dist_raw_buf[shot_size];
extern float dist_disp_buf[shot_size];
extern float temp_dist_offset;
extern float dist_cal_setpoint_ft;//  Distance to set marker during distance calibration, feet
extern float dist_cal_setpoint_mt;

// Loop Test
extern float loop_horizontal, loop_vertical, loop_azimuth, loop_distance, loop_error;

// SD card stuff
extern FRESULT configure_SD(void);
extern FRESULT SD_status;
extern FIL file_cal_report, file_cal_raw;

extern struct OPTIONS options;

void cal_loop_test(struct MEASUREMENT *meas_inst){
	float N1, E1, D1;
	float dN1, dE1, dD1;
	float delta_horizontal;
	
	
	// increment distance and point counter
	n_points = n_points+1;
	loop_distance = loop_distance + meas_inst->distance;
	// find current position
	N1 = loop_horizontal*cos(deg2rad*loop_azimuth);
	E1 = loop_horizontal*sin(deg2rad*loop_azimuth);
	D1 = loop_vertical;
	// find difference in position
	delta_horizontal = meas_inst->distance*cos(deg2rad*meas_inst->inclination);
	dN1 = delta_horizontal*cos(deg2rad*meas_inst->azimuth);
	dE1 = delta_horizontal*sin(deg2rad*meas_inst->azimuth);
	dD1 = meas_inst->distance*sin(deg2rad*meas_inst->inclination);
	//  add  new offsets
	N1 = N1 + dN1;
	E1 = E1 + dE1;
	D1 = D1 + dD1;
	//  Calculate new horizontal, vertical, azimuth
	loop_horizontal = sqrt(pow(N1,2)+pow(E1,2));
	loop_vertical = D1;
	loop_azimuth = rad2deg*atan2(E1, N1);
	
	loop_error = sqrt(pow(loop_horizontal,2) + pow(loop_vertical,2))/loop_distance;
	
}

void cal_dist_process(void){
	
	
	//  Fill Out data for report structure
	get_time();//  Get current time
	cal_report_dist.timestamp = gen_posix_time(&current_time);
	cal_report_dist.software_version = software_version;
	cal_report_dist.points = buf_points;
	memcpy(&cal_report_dist.time_struct,&current_time,sizeof(current_time));
	
	dist_calst.dist_offset = temp_dist_offset;
}

void cal_add_dist(struct MEASUREMENT *meas_inst){
	float avg_raw;
	uint8_t k;
	
	dist_raw_buf[ind_buf] = meas_inst->distance;
	ind_buf = ind_buf+1;
	if (ind_buf>=shot_size){ind_buf = 0;}
	
	avg_raw = 0;
	for (k=0;k<shot_size;k++){
		avg_raw = avg_raw + dist_raw_buf[k];
	}
	avg_raw = avg_raw/shot_size;
	
	// temp_dist_offset always in meters
	if (options.current_unit_dist == feet){
		// If measurements are in feet, find offset in meters
		temp_dist_offset = (dist_cal_setpoint_ft - avg_raw)/mt2ft;
	}else{
		//  Measurement already in meters
		temp_dist_offset = dist_cal_setpoint_mt-avg_raw;
	}
	
	
	//  Fill in values to display
	for (k=0;k<shot_size;k++){
		if (options.current_unit_dist == feet){
			dist_disp_buf[k] = dist_raw_buf[k]+temp_dist_offset*mt2ft;
		}else{
			dist_disp_buf[k] = dist_raw_buf[k]+temp_dist_offset;
		}
		
	}
	
	buf_points = buf_points+1;
	
	
	
}





FRESULT cal_write_report(void){
	uint32_t i, j, k;
	//uint32_t bw;
	//uint32_t *pbw;
	UINT	bw;
	UINT	*pbw;
	char file_name[250];
	char write_str1[600];
	char write_str2[600];
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	pbw = &bw;
	struct INST_CAL *pcal_struct;
	float raw_data_entry[12];
	uint32_t group_current;
	
	struct MEASUREMENT temp_meas;
	float azm_raw, inc_raw, roll_raw;
	float azm_cal, inc_cal, roll_cal;
	
	//  Set up SD card
	config_spi(SD_card);
	spi_select_slave(&spi_main, &slave_SD, true);

	diskio_status = disk_status(0);

	if(diskio_status){
		//Possibly card not initialized
		configure_SD();
		diskio_status = disk_status(0);
		if(diskio_status){
			fdebug1 = FR_NOT_READY;
			SD_status = fdebug1;
			config_spi(LCD);
			return fdebug1;
		}
	
	}	
	
	
	
	sprintf(file_name, "20%02x%02x%02x_%02x%02x%02x_calibration_report.txt", current_time.year, current_time.month, current_time.date,
							current_time.hours, current_time.minutes, current_time.seconds);
	fdebug2 = f_open(&file_cal_report, file_name, FA_CREATE_NEW | FA_READ | FA_WRITE);						
	if(fdebug2!=FR_OK){
		SD_status = fdebug2;
		config_spi(LCD);
		return fdebug2;
	}
	
	
	//  Write Header
	sprintf(write_str1,"Calibration Report\r\n\r\nDate and Time:\r\nYYYY.MM.DD@HH:mm:ss\r\n20%02x.%02x.%02x@%02x:%02x:%02x\r\n\r\n",
				cal_report_azm_inc.time_struct.year, cal_report_azm_inc.time_struct.month, cal_report_azm_inc.time_struct.date,
				cal_report_azm_inc.time_struct.hours, cal_report_azm_inc.time_struct.minutes, cal_report_azm_inc.time_struct.seconds);
	fdebug3 = f_write(&file_cal_report, write_str1, strlen(write_str1), pbw);
	
	//  Write Version
	sprintf(write_str1,"Software Version: %1.1f\r\n\r\n", cal_report_azm_inc.software_version);
	fdebug3 = f_write(&file_cal_report, write_str1, strlen(write_str1), pbw);
	
	//  Write Metrics
	// groups and points
	sprintf(write_str1,"Measurements: %d\r\n4-Point Groups: %d\r\n\r\n", 
				cal_report_azm_inc.points, cal_report_azm_inc.groups);
	fdebug3 = f_write(&file_cal_report, write_str1, strlen(write_str1), pbw);
	// angle error
	sprintf(write_str1,"Angle Error Standard Deviation:\r\n  Azimuth: %3.6f degrees\r\n  Inclination: %3.6f degrees\r\n",
				cal_report_azm_inc.azm_angle_err, cal_report_azm_inc.inc_angle_err);				
	fdebug3 = f_write(&file_cal_report, write_str1, strlen(write_str1), pbw);
	// Magnitude Error Standard Deviation
	sprintf(write_str1,"Magnitude Error Standard Deviation:\r\n  Accelerometer 1: %3.6f %%\r\n  Accelerometer 2: %3.6f %%\r\n  Compass 1: %3.6f %%\r\n  Compass 2: %3.6f %%\r\n",
				cal_report_azm_inc.mag_stdev_a1*100, cal_report_azm_inc.mag_stdev_a2*100,cal_report_azm_inc.mag_stdev_c1*100, cal_report_azm_inc.mag_stdev_c2*100);
	// Axis Disparity
	fdebug3 = f_write(&file_cal_report, write_str1, strlen(write_str1), pbw);
	sprintf(write_str1,"Axis Disparity:\r\n  Accelerometer: X-%3.6f%% Y-%3.6f%% Z-%3.6f%%\r\n",
		cal_report_azm_inc.disp_stdev_acc[0]*100, cal_report_azm_inc.disp_stdev_acc[1]*100,cal_report_azm_inc.disp_stdev_acc[2]*100);
	fdebug3 = f_write(&file_cal_report, write_str1, strlen(write_str1), pbw);	
	sprintf(write_str1,"  Compass: X-%3.6f%% Y-%3.6f%% Z-%3.6f%%\r\n\r\n\r\n",
		cal_report_azm_inc.disp_stdev_comp[0]*100, cal_report_azm_inc.disp_stdev_comp[1]*100,cal_report_azm_inc.disp_stdev_comp[2]*100);
	fdebug3 = f_write(&file_cal_report, write_str1, strlen(write_str1), pbw);

	
	//  Write Calibration Values
	for (i=0;i<4;i++){
		switch(i){
			case 0:
				pcal_struct = &a1_calst;
				sprintf(write_str1,"Inclinometer 1 Calibration:\r\n");
				break;
			case 1:
				pcal_struct = &a2_calst;
				sprintf(write_str1,"Inclinometer 2 Calibration:\r\n");
				break;
			case 2:
				pcal_struct = &c1_calst;
				sprintf(write_str1,"Compass 1 Calibration:\r\n");
				break;
			case 3:
				pcal_struct = &c2_calst;
				sprintf(write_str1,"Compass 2 Calibration:\r\n");
				break;
		}
		sprintf(write_str2,"  Gain X: %.6f\r\n  Gain Y: %.6f\r\n  GainZ: %.6f\r\n", pcal_struct->gain[0], pcal_struct->gain[1], pcal_struct->gain[2]);
		strcat(write_str1, write_str2);
		sprintf(write_str2,"  Offset X: %.6f\r\n  Offset Y: %.6f\r\n  Offset Z: %.6f\r\n", pcal_struct->offset[0], pcal_struct->offset[1], pcal_struct->offset[2]);
		strcat(write_str1, write_str2);
		sprintf(write_str2,"  Axis Misalignment, Y-X: %.6f deg\r\n  Axis Misalignment, Z-X: %.6f deg\r\n  Axis Misalignment, Z-Y: %.6f deg\r\n", 
				rad2deg*pcal_struct->axmYX, rad2deg*pcal_struct->axmZX, rad2deg*pcal_struct->axmZY);
		strcat(write_str1, write_str2);
		
		
		
		sprintf(write_str2,"  Package Misalignment About X: %.3f deg\r\n  Package Misalignment About Y: %.3f deg\r\n  Package Misalignment About Z: %.3f deg\r\n\r\n", 
			pcal_struct->thetaX, pcal_struct->thetaY, pcal_struct->thetaZ);
		strcat(write_str1, write_str2);
		//fdebug2 = f_lseek(&file_cal, f_size(&file_cal));
		fdebug3 = f_write(&file_cal_report, write_str1, strlen(write_str1), pbw);
	}
	
	
	// Write Compass/Incl/Roll Data
	sprintf(write_str1,"\r\nCalibration Measurements:\r\n");
	sprintf(write_str2,"                Uncalibrated                     Calibrated\r\n");
	strcat(write_str1, write_str2);
	sprintf(write_str2,"Group, Point,   Azimuth,  Inclination, Roll,     Azimuth,  Inclination, Roll\r\n");
	strcat(write_str1, write_str2);
	fdebug3 = f_write(&file_cal_report, write_str1, strlen(write_str1), pbw);
	for (i=0;i<n_points;i++){
		//  Fist Measure uncalibrated raw data
		for (j=0;j<3;j++){
			temp_meas.a1xyz[j] = a1raw[i][j];
			temp_meas.a2xyz[j] = a2raw[i][j];
			temp_meas.c1xyz[j] = c1raw[i][j];
			temp_meas.c2xyz[j] = c2raw[i][j];
		}
		calc_orientation(&temp_meas);
		azm_raw = temp_meas.azimuth;
		inc_raw = temp_meas.inclination;
		roll_raw = temp_meas.roll;
		//  Next Measure calibrated data
		for (j=0;j<3;j++){
			temp_meas.a1xyz[j] = a1cal[i][j];
			temp_meas.a2xyz[j] = a2cal[i][j];
			temp_meas.c1xyz[j] = c1cal[i][j];
			temp_meas.c2xyz[j] = c2cal[i][j];
		}
		calc_orientation(&temp_meas);
		azm_cal = temp_meas.azimuth;
		inc_cal = temp_meas.inclination;
		roll_cal = temp_meas.roll;
		group_current = floor(i/group_size)+1;
		sprintf(write_str1,"%-2d,    %-2d,      %03.2f,   %03.2f,      %03.2f,   %03.2f,   %03.2f,      %03.2f\r\n",
			group_current, i+1,  azm_raw, inc_raw, roll_raw, azm_cal, inc_cal, roll_cal);
			
		fdebug3 = f_write(&file_cal_report, write_str1, strlen(write_str1), pbw);
		
	}
	
	
	f_close(&file_cal_report);
	
	
	
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//write raw data file
	sprintf(file_name, "20%02x%02x%02x_%02x%02x%02x_calibration_raw_data.bin", current_time.year, current_time.month, current_time.date,
	current_time.hours, current_time.minutes, current_time.seconds);
	
	fdebug2 = f_open(&file_cal_raw, file_name, FA_CREATE_NEW | FA_READ | FA_WRITE);
	if(fdebug2 != FR_OK){
		// Some other SD card error
		SD_status = fdebug2;
		config_spi(LCD);
		return fdebug2;
	}
	
	
	// Write File
	for (i=0;i<n_points;i++){
		for (k=0;k<3;k++){
			raw_data_entry[k] = a1raw[i][k];
			raw_data_entry[k+3] = a2raw[i][k];
			raw_data_entry[k+6] = c1raw[i][k];
			raw_data_entry[k+9] = c2raw[i][k];
			
		}
		//fdebug2 = f_lseek(&file_cal, f_size(&file_cal));
		fdebug3 = f_write(&file_cal_raw, raw_data_entry, sizeof(raw_data_entry), pbw);
	}
	
	//fdebug3 = f_write(&file2, write_string_full, strlen(write_string_full), bw);
	f_close(&file_cal_raw);
	
	
	return fdebug3;
	
}





void cal_evaluate(void){
	uint32_t i, p, g, k, ind1;
	struct MEASUREMENT temp_meas;
	uint8_t wrap_around;
	float inc_group[4];
	float azm_group[4];
	float inc_avg, azm_avg;
	float inc_err_array[nbuf];
	float azm_err_array[nbuf];
	
	//  Fill Out data for report structure
	get_time();//  Get current time
	cal_report_azm_inc.timestamp = gen_posix_time(&current_time);
	cal_report_azm_inc.software_version = software_version;
	cal_report_azm_inc.points = n_points;
	cal_report_azm_inc.groups = n_groups;
	memcpy(&cal_report_azm_inc.time_struct,&current_time,sizeof(current_time));

	//  Calculate azimuth, incination, and roll for all data
	for (p=0;p<n_points;p++){
		for (i=0;i<3;i++){
			temp_meas.a1xyz[i] = a1cal[p][i];
			temp_meas.a2xyz[i] = a2cal[p][i];
			temp_meas.c1xyz[i] = c1cal[p][i];
			temp_meas.c2xyz[i] = c2cal[p][i];
		}
		calc_orientation(&temp_meas);
		azimuth[p] = temp_meas.azimuth;
		inclination[p] = temp_meas.inclination;
		roll[p] = temp_meas.roll;
	}
	
	//  evaluate metrics of calibration
	for (g=0;g<n_groups;g++){
		
		//  Create group of shots
		wrap_around = 0;
		for (k=0;k<group_size;k++){
			ind1 = g*group_size+k;
			inc_group[k] = inclination[ind1];
			azm_group[k] = azimuth[ind1];
			if (azm_group[k]>340){
				wrap_around = 1;//  Possible wrap-around on azimuth
			}
		}
		
		//  Adjust for angle wrap-around
		if (wrap_around){
			for (k=0;k<group_size;k++){
				if (azm_group[k]<20){
					azm_group[k] = azm_group[k]+360;
				}
			}
		}
		
		//  Find group average
		inc_avg = 0;
		azm_avg = 0;
		for (k=0;k<group_size;k++){
			inc_avg = inc_avg + inc_group[k];
			azm_avg = azm_avg + azm_group[k];
		}
		inc_avg = inc_avg/group_size;
		azm_avg = azm_avg/group_size;
		
		//  Calculate errors
		for (k=0;k<group_size;k++){
			ind1 = g*group_size+k;
			inc_err_array[ind1] = inc_group[k]-inc_avg;
			azm_err_array[ind1] = (azm_group[k]-azm_avg)*cos(inc_avg*deg2rad);
		}
		
		
		
	}
	
	//  Calculate standard deviation of angle errors
	cal_report_azm_inc.inc_angle_err = stdev(inc_err_array, n_points);	 
	cal_report_azm_inc.azm_angle_err = stdev(azm_err_array, n_points);
	
	//  Evaluate magnitude error Standard Deviation
	cal_report_azm_inc.mag_stdev_a1 = calc_mag_stdev(a1cal);
	cal_report_azm_inc.mag_stdev_a2 = calc_mag_stdev(a2cal);
	cal_report_azm_inc.mag_stdev_c1 = calc_mag_stdev(c1cal);
	cal_report_azm_inc.mag_stdev_c2 = calc_mag_stdev(c2cal);
	
	//  Evaluate Axis Disparity Standard Deviation
	for (i=0;i<3;i++){
		cal_report_azm_inc.disp_stdev_acc[i]  = calc_disp_stdev(a1cal, a2cal, i);
		cal_report_azm_inc.disp_stdev_comp[i] = calc_disp_stdev(c1cal, c2cal, i);
	}
		
	
}

void cal_add_datapoint(struct MEASUREMENT *meas_inst, bool last_shot){
	static float a1buf[group_size][3], a2buf[group_size][3];
	static float c1buf[group_size][3], c2buf[group_size][3];
	static float aX_ang_ref, cX_ang_ref;
	//static uint8_t ind_stack, ind_buf, buf_points;
	float aX_ang, cX_ang, foo;
	float aDelta, cDelta;
	float aXYZ[3], cXYZ[3];
	uint8_t j, k;
	
	
	//  If this is the first point, initialize reference angles
	if ((ind_stack==0) && (ind_buf==0) && (buf_points == 0)){
		aX_ang_ref = 200;//  Ensures delta will fail on first iteration
		cX_ang_ref = 200;//  Ensures delta will fail on first iteration
	}

	
	//  Filter and sort into groups
	for (j=0;j<3;j++){
		aXYZ[j] = 0.5*(meas_inst->a1xyz[j]+meas_inst->a2xyz[j]);
		cXYZ[j] = 0.5*(meas_inst->c1xyz[j]+meas_inst->c2xyz[j]);
	}
	calc_theta_XY(aXYZ, &foo, &aX_ang);
	calc_theta_XY(cXYZ, &foo, &cX_ang);
	aDelta = fabs(aX_ang - aX_ang_ref);
	cDelta = fabs(cX_ang - cX_ang_ref);
	if ((aDelta>delta_min)||(cDelta>delta_min)||last_shot){
		// New Orientation Encountered
		if ((buf_points>= group_size)&&(ind_stack<(nbuf-buf_points))){//  Prevent buffer overflow
			//  Enough points for new group, add to stack
			n_groups = n_groups+1;
			n_points = n_points+group_size;
			for (k=0; k<group_size;k++){
				for (j=0;j<3;j++){
					a1raw[ind_stack][j] = a1buf[k][j];
					a2raw[ind_stack][j] = a2buf[k][j];
					c1raw[ind_stack][j] = c1buf[k][j];
					c2raw[ind_stack][j] = c2buf[k][j];
				}
				ind_stack = ind_stack+1;
			}
		}
		ind_buf = 0;
		buf_points = 0;
		aX_ang_ref = aX_ang;
		cX_ang_ref = cX_ang;
	}
	for (j=0;j<3;j++){
		a1buf[ind_buf][j] = meas_inst->a1xyz[j];
		a2buf[ind_buf][j] = meas_inst->a2xyz[j];
		c1buf[ind_buf][j] = meas_inst->c1xyz[j];
		c2buf[ind_buf][j] = meas_inst->c2xyz[j];
	}
	buf_points = buf_points+1;
	ind_buf = ind_buf+1;
	if (ind_buf>=group_size){
		ind_buf = 0;
	}
}


void calc_orientation(struct MEASUREMENT *meas_inst){
	uint8_t i;
	float aXYZ[3], cXYZ[3];
	float thetaX, thetaY, thetaZ;
	
	//  Take average reading from both sensors for each axis
	for (i=0;i<3;i++){
		aXYZ[i] = 0.5*(meas_inst->a1xyz[i]+meas_inst->a2xyz[i]);
		cXYZ[i] = 0.5*(meas_inst->c1xyz[i]+meas_inst->c2xyz[i]);
	}
	
	//  Calculate Aximuth, Inclination, and Roll
	calc_azm_inc_roll_dec(aXYZ, cXYZ,
		&meas_inst->azimuth,		&meas_inst->inclination,
		&meas_inst->roll,		&meas_inst->declination);
	
	
}

void rotvec_theta_ZY(float XYZ[3], float rotXYZ[3], float *thetaZ, float *thetaY){
	float rotM[3][3];
	float rthetaZ;
	float rthetaY;
	
	rthetaZ = *thetaZ*deg2rad;
	rthetaY = *thetaY*deg2rad;
	
	//  Rotation Matrix about Z Axis
	rotM[0][0] = cos(rthetaZ);
	rotM[0][1] = -1*sin(rthetaZ);
	rotM[0][2] = 0;
	rotM[1][0] = sin(rthetaZ);
	rotM[1][1] = cos(rthetaZ);
	rotM[1][2] = 0;
	rotM[2][0] = 0;
	rotM[2][1] = 0;
	rotM[2][2] = 1;
	mat_mult_33_31(rotM, XYZ, rotXYZ);
	// Rotation Matrix about Y Axis
	rotM[0][0] = cos(rthetaY);
	rotM[0][1] = 0;
	rotM[0][2] = sin(rthetaY);
	rotM[1][0] = 0;
	rotM[1][1] = 1;
	rotM[1][2] = 0;
	rotM[2][0] = -1*sin(rthetaY);
	rotM[2][1] = 0;
	rotM[2][2] = cos(rthetaY);
	mat_mult_33_31(rotM, rotXYZ, rotXYZ);
	
	
}

void rotvec_theta_XY(float XYZ[3], float rotXYZ[3], float *thetaX, float *thetaY){
	float rotM[3][3];
	float rthetaX;
	float rthetaY;
	
	rthetaX = *thetaX*deg2rad;
	rthetaY = *thetaY*deg2rad;
	
	// Product of two rotation matrixes, R(thetaX)*R(thetaY)
	//  Rotate around X axis
	rotM[0][0] = 1;
	rotM[0][1] = 0;
	rotM[0][2] = 0;
	rotM[1][0] = 0;
	rotM[1][1] = cos(rthetaX);
	rotM[1][2] = -1*sin(rthetaX);
	rotM[2][0] = 0;
	rotM[2][1] = sin(rthetaX);
	rotM[2][2] = cos(rthetaX);
	mat_mult_33_31(rotM, XYZ, rotXYZ);
	
	//  Rotate about Y axis
	rotM[0][0] = cos(rthetaY);
	rotM[0][1] = 0;
	rotM[0][2] = sin(rthetaY);
	rotM[1][0] = 0;
	rotM[1][1] = 1;
	rotM[1][2] = 0;
	rotM[2][0] = -1*sin(rthetaY);
	rotM[2][1] = 0;
	rotM[2][2] = cos(rthetaY);
	mat_mult_33_31(rotM, rotXYZ, rotXYZ);


}


void mat_mult_33_31(float mat33[3][3], float mat3[3], float ret3[3]){
	uint8_t i, j;
	float temp[3];
	
	//  Multiply 3x3 mat33 matrix by 3x1 mat3 matrix
	for(i=0;i<3;i++){
		temp[i] = 0;
		for (j=0;j<3;j++){
			temp[i] = temp[i]+mat33[i][j]*mat3[j];
		}

	}
	//  Copy temp matrix back into ret3 matrix
	for(i=0;i<3;i++){
		ret3[i] = temp[i];
	}
	
}

void calc_azm_inc_roll_dec(float aXYZ[3], float cXYZ[3], float *azimuthP, float *inclinationP, float *rollP, float *declinationP){
	
	float crotXYZ[3];
	float thetaX, thetaY, crxy;
	
	//  Calculate Inclination and Roll
	calc_theta_XY( aXYZ , &thetaX, &thetaY);
	*inclinationP = -1*thetaY;
	*rollP = thetaX;
	
	//  Calculate Azimuth
	rotvec_theta_XY(cXYZ, crotXYZ, &thetaX, &thetaY);
	*azimuthP = rad2deg*atan2(crotXYZ[1], crotXYZ[0]);
	if ((*azimuthP)<0){
		//  Result is -180 to +180; Compass must be 0-360
		*azimuthP = *azimuthP+360;
	}
	
	//  Calculate declination
	crxy= sqrt(pow(crotXYZ[0],2)+pow(crotXYZ[1],2));
	*declinationP = rad2deg*atan2(crotXYZ[2], crxy);
	
		
}


void calc_theta_XY(float XYZ[3], float *thetaX, float *thetaY){
	float ryz;
	ryz = sqrt(pow(XYZ[1],2) + pow(XYZ[2],2));
	
	*thetaX = rad2deg*atan2(XYZ[1], XYZ[2]) + 180;
	*thetaY = rad2deg*atan2(XYZ[0], ryz);
	
}


float stdev(float data[nbuf], uint32_t n_meas){
	uint8_t i;
	double mean, sumsq;
	
	mean = 0;
	for (i=0;i<n_meas;i++){
		mean = mean+data[i];
	}
	mean = mean/n_meas;
	
	sumsq = 0;
	for (i=0;i<n_meas;i++){
		sumsq = sumsq+pow((data[i]-mean),2);
	}
	sumsq = sumsq/(n_meas-1);
	sumsq = sqrt(sumsq);
	
	return sumsq;
}

float calc_mag_stdev(float XYZ[nbuf][3]){
	float err_mag[nbuf];
	uint32_t p;
	float temp1;
	
	for (p=0;p<n_points;p++){
		temp1 = sqrt(pow(XYZ[p][0],2)+pow(XYZ[p][1],2)+pow(XYZ[p][2],2));
		err_mag[p] = temp1-1;
	}
	temp1 = stdev(err_mag, n_points);
	
	return temp1;
}

float calc_disp_stdev(float XYZ1[nbuf][3], float XYZ2[nbuf][3], uint8_t axis){
	float err_disp[nbuf];
	uint32_t p;
	for (p=0;p<n_points;p++){
		err_disp[p] =  XYZ1[p][axis]-XYZ2[p][axis];
		
	}
	
	return stdev(err_disp, n_points);
	
}

void cal_axis_misalignments(float XYZ[nbuf][3], struct INST_CAL *cal_struct){
	float D[nbuf][4];
	float V[nbuf];
	uint32_t i, j, k;
	float DtD [6][6];
	float DtV[4];
	float Res[4];
	// D*Res = V;
	//  D'D*Res = D'*V;
	// inv(D'D)*D'D*Res = Res = inv(D'D)*(D'*V);
	
	
	// Create D
	for (k=0;k<n_points;k++){
		D[k][0] = 1;
		D[k][1] = -2*XYZ[k][0]*XYZ[k][1];
		D[k][2] = -2*XYZ[k][1]*XYZ[k][2];
		D[k][3] = -2*XYZ[k][0]*XYZ[k][2];
		
		V[k] = pow(XYZ[k][0],2)+pow(XYZ[k][1],2)+pow(XYZ[k][2],2);
	}
	//printf("\n\nD Matrix:\n");
	//print_matrix(D, n_points, 4);
	//D'*D
	for (i=0;i<4;i++){
		for (j=0;j<4;j++){
			DtD[i][j] = 0;
			for (k=0;k<n_points;k++){
				DtD[i][j] = DtD[i][j] + D[k][i]*D[k][j];
				//if ((i==0)&&(j==1)){
				//printf("\n DtD[0,1] = %f D[k,0] = %F, D[k,1] = %f",
				//        DtD[i][j], D[k][i], D[k][j]);
				//}
			}
		}
	}
	//printf("\n\nDtD Matrix:\n");
	//print_matrix(DtD, 6, 6);
	
	// Dt*V
	for (i=0;i<4;i++){
		DtV[i] = 0;
		for (k=0;k<n_points;k++){
			DtV[i] = DtV[i]+D[k][i]*V[k];
		}
	}
	//printf("\n\nDtV Matrix:\n");
	//print_matrix(DtV, 4, 1);
	
	//  Inv(D'D)
	inverse(DtD, DtD, 4);
	//printf("\n\ninverse DtD Matrix:\n");
	//print_matrix(DtD, 6, 6);
	//  Find result
	for (i=0;i<4;i++){
		Res[i] = 0;
		for (j=0;j<4;j++){
			Res[i] = Res[i]+DtD[i][j]*DtV[j];
			
		}
	}
	
	
	cal_struct->axmYX = Res[1];
	cal_struct->axmZY = Res[2];
	cal_struct->axmZX = Res[3];
	//printf("\n\naxmYX:%f, axmZY: %f, axmZX: %f\n",
	//   cal_struct->axmYX, cal_struct->axmZY, cal_struct->axmZX);
	//print_matrix(DtD, 6, 6);
}




void cal_angleX(float XYZ1[][3], float XYZ2[][3], struct INST_CAL *cal_struct){
	float D[nbuf];
	float V[nbuf];
	float DtD, DtV, Res;
	uint32_t i;
	
	// D = Y2-Z2
	// V = Y2-Y1 + Z2-Z1;
	for (i=0;i<n_points;i++){
		D[i] = XYZ2[i][1]-XYZ2[i][2];
		V[i] = XYZ2[i][1]-XYZ1[i][1]+XYZ2[i][2]-XYZ1[i][2];
	}
	
	// DtD = D'*D;
	DtD = 0;
	for (i=0;i<n_points;i++){
		DtD = DtD + pow(D[i],2);
	}
	// DtV = D'*V;
	DtV = 0;
	for (i=0;i<n_points;i++){
		DtV = DtV + D[i]*V[i];
	}
	
	// Res = (1/DtD)*DtV;
	Res = (1/DtD)*DtV;
	
	cal_struct->thetaX = -1*rad2deg*Res;

	// Create rotation matrix
	gen_RotM(cal_struct);
	
}




void cal_angleYZ(float XYZ[][3], struct INST_CAL *cal_struct){
	//  This code assumes input data has been calibrated for gain and offset
	//  But no rotation calibration has been applied
	float roll_ang[nbuf];
	float x_ang_shift[nbuf];
	uint8_t ind, i, j, k, np;
	float x_ang_comp, group_avg;
	float B1_init, B2_init;

	//  Variables for line fitting
	float detX;
	float X[nbuf][2];
	float tempX1[2][2], tempX2[2][2], tempX3[2], B[2];
	
	//  Initialize A and B for group average
	B1_init = cal_struct->thetaY;
	B2_init = -1*cal_struct->thetaZ;
	
	// Create shifted x_ang array
	for (k=0;k< n_groups;k++){
		group_avg = 0;
		for (j=0;j<group_size;j++){
			ind = k*group_size + j;
			calc_theta_XY(&XYZ[ind][0], &roll_ang[ind], &x_ang_shift[ind]);
			x_ang_comp  = x_ang_shift[ind]
			- B1_init*cos(deg2rad*roll_ang[ind])
			- B2_init*sin(deg2rad*roll_ang[ind]);
			group_avg = group_avg+x_ang_comp;
		}
		group_avg = group_avg/group_size;
		for (j=0;j<group_size;j++){
			ind = k*group_size + j;
			x_ang_shift[ind] = x_ang_shift[ind]-group_avg;
			
		}
		
	}

	
	//  X = [cosd(roll_fit), sind(roll_fit)]
	for (np=0;np<n_points;np++){
		X[np][0] = cos(deg2rad*roll_ang[np]);
		X[np][1] = sin(deg2rad*roll_ang[np]);
	}
	
	// tempX1 = X'*X;
	for (i=0;i<2;i++){
		for (j=0;j<2;j++){
			tempX1[i][j]=0;
			for (np=0;np<n_points;np++){
				tempX1[i][j] = tempX1[i][j]+X[np][j]*X[np][i];
			}
		}
	}

	//  tempX2 = inverse(tempX1) = inv(X'*X);
	detX = tempX1[0][0]*tempX1[1][1]-(tempX1[0][1]*tempX1[1][0]);
	tempX2[0][0] = tempX1[1][1]/detX;
	tempX2[1][1] = tempX1[0][0]/detX;
	tempX2[0][1] = -1*tempX1[1][0]/detX;
	tempX2[1][0] = -1*tempX1[0][1]/detX;


	// tempX3 = X'*Y;
	for (i=0;i<2;i++){
		tempX3[i] = 0;
		for (np=0;np<n_points;np++){
			tempX3[i] = tempX3[i] + X[np][i]*x_ang_shift[np];
		}
	}
	
	// B = inv(X'*X)*X'*Y = tempX2*tempX3
	for (i=0;i<2;i++){
		B[i] = 0;
		for (j=0;j<2;j++){
			B[i] = B[i]+tempX2[i][j]*tempX3[j];
		}
		
	}
	
	
	// Update calibration structure
	cal_struct->thetaY = B[0];
	cal_struct->thetaZ = -1*B[1];
	
	// Create rotation matrix
	gen_RotM(cal_struct);

	
	// Find standard deviation of result
	B1_init = cal_struct->thetaY;
	B2_init = -1*cal_struct->thetaZ;
	
	for (np=0;np<n_points;np++){
		x_ang_shift[np] = x_ang_shift[np]
		- B1_init*cos(deg2rad*roll_ang[np])
		- B2_init*sin(deg2rad*roll_ang[np]);
	}
	cal_struct->angle_stdev = stdev(x_ang_shift,n_points);
	
}




void cal_apply_cal(float uncalXYZ[3], float calXYZ[3], struct INST_CAL *cal_struct){
	uint8_t i;
	
	//  Apply gain and offset calibration
	for (i=0;i<3;i++){
		//  Subtract Offset
		calXYZ[i] = uncalXYZ[i]-cal_struct->offset[i];
		//  Divide by Gain
		calXYZ[i] = calXYZ[i]/cal_struct->gain[i];
	}
	//  Apply axis misalignments
	calXYZ[1] = calXYZ[1]+calXYZ[0]*cal_struct->axmYX;
	calXYZ[2] = calXYZ[2]+calXYZ[1]*cal_struct->axmZY+ calXYZ[0]*cal_struct->axmZX;


	//  Apply rotation calibration

	//Rotate vector by rotation matrix
	mat_mult_33_31(cal_struct->RotM, calXYZ, calXYZ);
	
}

void cal_init_struct(struct INST_CAL *cal_struct){
	uint8_t i, j;
	
	for (i=0;i<3;i++){
		cal_struct->gain[i] = 1;
		cal_struct->offset[i] = 0;
		
		for (j=0;j<3;j++){
			cal_struct->RotM[i][j] = 0;//Clear row
		}
		cal_struct->RotM[i][i] = 1;// Create identity matrix
		
	}
	cal_struct->axmYX = 0;
	cal_struct->axmZY = 0;
	cal_struct->axmZX = 0;
	
	cal_struct->thetaX = 0;
	cal_struct->thetaY = 0;
	cal_struct->thetaZ = 0;
	cal_struct->angle_stdev = 1;
	
	cal_struct->dist_offset = 0;
	
	cal_struct->Cal_Initialized_Key = 0xB1;//  Indicator that structure has been initialized
	
}

void gen_RotM(struct INST_CAL *cal_struct){
	float tX, tY, tZ;
	
	tX = cal_struct->thetaX*deg2rad;
	tY = cal_struct->thetaY*deg2rad;
	tZ = cal_struct->thetaZ*deg2rad;
	
	cal_struct->RotM[0][0] = cos(tY)*cos(tZ);
	cal_struct->RotM[0][1] = -sin(tZ);
	cal_struct->RotM[0][2] = sin(tY);
	cal_struct->RotM[1][0] = (cos(tX)*sin(tZ))+(sin(tX)*sin(tY));
	cal_struct->RotM[1][1] = cos(tX)*cos(tZ);
	cal_struct->RotM[1][2] = -sin(tX)*cos(tY);
	cal_struct->RotM[2][0]  = (sin(tX)*sin(tZ))-(cos(tX)*sin(tY));
	cal_struct->RotM[2][1] = sin(tX)*cos(tZ);
	cal_struct->RotM[2][2] = cos(tX)*cos(tY);
	
}





void cal_gain_off(float XYZ[][3], struct INST_CAL *cal_struct){
	float D[nbuf][6];
	float temp1[6][6], temp2[6];
	float v[6];
	float g;
	uint8_t i,j,k;
	

	
	
	for (i=0;i<n_points;i++){
		D[i][0] = XYZ[i][0]*XYZ[i][0];
		D[i][1] = XYZ[i][1]*XYZ[i][1];
		D[i][2] = XYZ[i][2]*XYZ[i][2];
		D[i][3] = 2*XYZ[i][0];
		D[i][4] = 2*XYZ[i][1];
		D[i][5] = 2*XYZ[i][2];
		
	}
	
	for (i=0;i<6;i++){
		for (j=0;j<6;j++){
			temp1[i][j] = 0;
			for (k=0;k<n_points;k++){
				temp1[i][j] = temp1[i][j] + D[k][i]*D[k][j];
				
				
			}
		}
	}
	
	//printf("\n\nTemp Matrix:\n");
	//print_matrix(temp1, 6, 6);
	

	inverse(temp1, temp1, 6);
	
	//printf("\n\nInverse of Temp Matrix:\n");
	//print_matrix(temp1,6, 6);
	
	for (i=0;i<6;i++){
		temp2[i] = 0;
		for (k = 0;k<n_points;k++){
			temp2[i] = temp2[i]+D[k][i];
			
		}
		
	}
	

	for (i=0;i<6;i++){
		v[i] = 0;
		for (j=0;j<6;j++){
			v[i] = v[i]+temp1[i][j]*temp2[j];
			
		}
		
	}
	

	for (i=0;i<3;i++){
		cal_struct->offset[i] = -1*(v[i+3]/v[i]);
	}
	
	
	g=1 + pow(v[3],2)/v[0]+pow(v[4],2)/v[1]+pow(v[5],2)/v[2];
	
	
	for (i=0;i<3;i++){
		cal_struct->gain[i] = sqrt(g/v[i]);
		
	}
	

}



void inverse(float source[6][6], float dest[6][6], uint8_t f)
{
	float b[6][6], fac[6][6];
	uint8_t p, q, m, n, i, j;
	//f = 6;
	for (q = 0;q < f; q++)
	{
		for (p = 0;p < f; p++)
		{
			m = 0;
			n = 0;
			for (i = 0;i < f; i++)
			{
				for (j = 0;j < f; j++)
				{
					if (i != q && j != p)
					{
						b[m][n] = source[i][j];
						if (n < (f - 2))
						n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			fac[q][p] = pow(-1, q + p) * determinant(b, f - 1);
		}
	}
	transpose(source, dest, fac, f);
}
/*Finding transpose of matrix*/
void transpose(float source[6][6], float dest[6][6], float fac[6][6], uint8_t r)
{
	uint8_t i, j;
	//r = 6;
	float b[6][6], d;
	
	for (i = 0;i < r; i++)
	{
		for (j = 0;j < r; j++)
		{
			b[i][j] = fac[j][i];
		}
	}

	
	d = determinant(source, r);
	for (i = 0;i < r; i++)
	{
		for (j = 0;j < r; j++)
		{
			dest[i][j] = b[i][j] / d;
		}
	}
	
}


float determinant(float a[6][6], uint8_t k)
{
	float s = 1, det = 0, b[6][6];
	uint8_t i, j, m, n, c;
	if (k == 1)
	{
		return (a[0][0]);
	}
	else
	{
		det = 0;
		for (c = 0; c < k; c++)
		{
			m = 0;
			n = 0;
			for (i = 0;i < k; i++)
			{
				for (j = 0 ;j < k; j++)
				{
					b[i][j] = 0;
					if (i != 0 && j != c)
					{
						b[m][n] = a[i][j];
						if (n < (k - 2))
						n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			det = det + s * (a[0][c] * determinant(b, k - 1));
			s = -1 * s;
		}
	}
	
	return (det);
}





