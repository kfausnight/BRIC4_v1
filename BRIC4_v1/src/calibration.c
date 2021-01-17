/*
 * calibration.c
 *
 * Created: 1/21/2019 5:16:02 PM
 *  Author: Kris Fausnight
 */ 
#include <calibration.h>

uint8_t ind_buf, groupPoints, currentGroup;

uint8_t cal_getGroupPoints(void){
	return groupPoints;
	
}

uint8_t cal_getCurrentGroup(void){
	return currentGroup;
}



void cal_resetGroup(void){
	
	ind_buf = 0;
	groupPoints = 0;
	nGroups = currentGroup-1;
}

void removeGroup(float XYZ[NBUFF][3], uint32_t group){
	uint32_t indStart, ind1, ind2;
	uint32_t k;
	
	indStart = (group-1)*GROUP_SIZE;

	for (ind1=indStart;ind1<(NBUFF-GROUP_SIZE);ind1++){
		ind2 = ind1+GROUP_SIZE;
		for (k=0;k<3;k++){
			XYZ[ind1][k] = XYZ[ind2][k];
		}
	}
}

uint32_t cal_removeGroup(bool *logRemove, uint32_t groupsStart){
	uint32_t g;
	uint32_t groupsEnd;
	
	groupsEnd = groupsStart;
	for (g=groupsStart;g>0;g--){
		if(logRemove[g]){
			removeGroup(a1Raw, g);
			removeGroup(a2Raw, g);
			removeGroup(m1Raw, g);
			removeGroup(m2Raw, g);			
			groupsEnd--;
		}
	}
	
	return groupsEnd;
	
	
}

uint8_t cal_findBadGroup(float errArr1[], float errArr2[], uint32_t *badGroup, float *badGroupDelta){
	//  Find worst group in both arrays
	//  Assumes index 0 is baseline
	//  Return source of worst group, Arr1 or Arr2
	float tempWorst;
	float dTemp1, dTemp2;	
	uint32_t grp;
	
	uint8_t temp_errSource = 0;
	uint8_t errSource = 0;
	
	for (grp=1;grp<=cal_report.groupsAll;grp++){
		dTemp1 = errArr1[0]-errArr1[grp];
		dTemp2 = errArr2[0]-errArr2[grp];
		if (dTemp1>dTemp2){
			tempWorst = dTemp1;
			temp_errSource = 1;
		}else{
			tempWorst = dTemp2;
			temp_errSource = 2;
		}
		if (tempWorst>*badGroupDelta){
			*badGroupDelta = tempWorst;
			*badGroup = grp;
			errSource = temp_errSource;
		}

	}
	return errSource;


}

void cal_init(void){
	uint32_t k;
	
	
	nGroups = 0;
	currentGroup = 1;
	nPoints = 0;
	ind_buf = 0;
	groupPoints = 0;
	
	for (k=0;k<SHOT_SIZE;k++){
		dist_raw_buf[k] = 0;
		dist_disp_buf[k] = 0;
	}
	
	loop_distance = 0;
	loop_horizontal = 0;
	loop_vertical = 0;
	loop_azimuth = 0;
	for (k=0;k<NBUFF;k++){
		azimuth[k]=0;
		inclination[k]=0;
		roll[k] = 0;
	}
}


void cal_done(enum CALTYPE caltype){
	
	switch (caltype){
		case inc_azm_full:
			EEPROM_saveCalRawData(inc_azm_full);
			//  Save initial cal report needed for processing
			cal_report.points = nGroups*GROUP_SIZE;
			cal_report.pointsAll = nGroups*GROUP_SIZE;;
			cal_report.groups = nGroups;
			cal_report.groupsAll = nGroups;
			get_time();//  Get current time and temp
			memcpy(&cal_report.time_inc_azm,&current_time,sizeof(current_time));
			memcpy(&cal_report.time_quick_azm,&current_time,sizeof(current_time));
			cal_report.tempC_inc_azm = currentTempC;
			save_cal_report();
			break;
		case azm_quick:
			EEPROM_saveCalRawData(azm_quick);
			//  Save initial cal report needed for processing
			cal_report.points = nPoints;
			cal_report.pointsAll = nPoints;
			get_time();//  Get current time
			memcpy(&cal_report.time_quick_azm,&current_time,sizeof(current_time));
			cal_report.tempC_quick_azm = currentTempC;
			save_cal_report();
			break;
		case rangeFinder:
			// Process Calibration data
			dist_calst.dist_offset = temp_dist_offset;
			
			//  Fill Out data for report structure
			get_time();//  Get current time
			memcpy(&cal_report.time_rangeFinder,&current_time,sizeof(current_time));
			cal_report.tempC_rangeFinder = currentTempC;
			//  Save data to EEPROM
			save_calibration();
			//  Add cal history entry
			SD_add_cal_history(rangeFinder);
			break;
		
		
	}
	
	
	
};

void cal_apply_cal_all(void){
	uint32_t i;
	//  Apply gain and offset calibration
	for (i=0;i<nPoints;i++){
		cal_apply_cal(a1Raw[i], a1Cal[i], &a1_calst);
		cal_apply_cal(a2Raw[i], a2Cal[i], &a2_calst);
		cal_apply_cal(m1Raw[i], m1Cal[i], &m1_calst);
		cal_apply_cal(m2Raw[i], m2Cal[i], &m2_calst);
	}
}

void cal_full_inc_azm_process(uint8_t nLoops){
	uint32_t loop;
	
	// Start with empty cal structures
	cal_init_struct(&a1_calst);
	cal_init_struct(&a2_calst);
	cal_init_struct(&m1_calst);
	cal_init_struct(&m2_calst);
	
	
	//  Individual Sensor Point Cloud Calibration
	for (loop=0;loop<nLoops;loop++){
		
		//  Apply current calibration
		cal_apply_cal_all();
		
		//  Gain and Offset Calibration
		cal_gain_off(a1Cal, &a1_calst);
		cal_gain_off(a2Cal, &a2_calst);
		cal_gain_off(m1Cal, &m1_calst);
		cal_gain_off(m2Cal, &m2_calst);
		
		//  Apply current calibration
		cal_apply_cal_all();
	
		//  Perform Axis Misalignment Calibration
		cal_axis_misalignments(a1Cal, &a1_calst); // Sensor axis misalignments, Accelerometer 1
		cal_axis_misalignments(a2Cal, &a2_calst);// Sensor axis misalignments, Accelerometer 2
		cal_axis_misalignments(m1Cal, &m1_calst);// Sensor axis misalignments, Compass 1
		cal_axis_misalignments(m2Cal, &m2_calst);// Sensor axis misalignments, Compass 2	
	}
	
	
	//  Sensor Package Alignment to Instrument	
	for (loop=0;loop<nLoops;loop++){
		//  Apply current calibration
		cal_apply_cal_all();
		
		//  Sensor package to laser axis alignment about Y&Z axis
		cal_angleYZ(a1Cal, &a1_calst);//  Sensor Package to laser Y&Z axis alignment, Accelerometer 1
		cal_angleYZ(a2Cal, &a2_calst);//  Sensor Package to laser Y&Z axis alignment, Accelerometer 2
		cal_angleYZ(m1Cal, &m1_calst);//  Sensor Package to laser Y&Z axis alignment, Compass 1
		cal_angleYZ(m2Cal, &m2_calst);//  Sensor Package to laser Y&Z axis alignment, Compass 2
		
		//  Apply current calibration
		cal_apply_cal_all();
	
		//  Perform X angle sensor-package to laser misalignment calibration
		//  Only calibrate the 2nd sensor
		cal_angleX(a1Cal, a2Cal, &a2_calst);//  Sensor package to laser X-axis alignment, Accelerometer 2
		cal_angleX(m1Cal, m2Cal, &m2_calst);//  Sensor package to laser X-axis alignment, Compass 2
		
	}
		
	//  Apply current calibration one last time
	cal_apply_cal_all();

	//  Evaluate performance of calibration
	cal_inc_azm_eval();
	
}

bool cal_checkStability(float m1Buf[][3], float arrMeans[3]){
	float temp1[NBUFFQAZM];
	float stdev3[3];
	bool isStable = true;
	uint32_t i, k;
	for (k=0;k<3;k++){
		for (i=0;i<NBUFFQAZM;i++){
			temp1[i] = m1Buf[i][k];		
		}
		stdev3[k] = stdev(temp1, NBUFFQAZM);
		arrMeans[k] = meanArr(temp1, NBUFFQAZM);
		
		isStable = isStable & (stdev3[k]<QAZM_STDEV_MIN);
	}
		
	return isStable;
}

bool cal_azm_quick_add_point(float m1Buf[][3], float m2Buf[][3], uint32_t index){
	bool isStableBoth, isStable1, isStable2;;
	float meanArr1[3], meanArr2[3];
	uint32_t i;
	
	isStable1 = cal_checkStability(m1Buf,meanArr1);
	isStable2 = cal_checkStability(m2Buf,meanArr2);
	
	
	if ( isStable1 && isStable2){
		isStableBoth = true;
		for (i=0;i<3;i++){
			m1Raw[index][i] = meanArr1[i];
			m2Raw[index][i] = meanArr2[i];
		}
		
	}else{
		isStableBoth = false;
	}
	
	
	return isStableBoth;
	
}

void cal_azm_quick_process(void){
	uint32_t i, k;
	struct INST_CAL temp_cal;
	float (*rawPtr)[3];
	float (*calPtr)[3];
	struct INST_CAL *calStructPtr;
	
	
	for (k=0;k<2;k++){
		switch (k){
			case 0:
				rawPtr = m1Raw;
				calPtr = m1Cal;
				calStructPtr = &m1_calst;
				break;
			case 1:
				rawPtr = m2Raw;
				calPtr = m2Cal;
				calStructPtr = &m2_calst;
				break;
	
		}

		//  Intialize blank cal structure
		cal_init_struct(&temp_cal);
		
		//  Gain/Offset Calibration
		cal_gain_off(rawPtr, &temp_cal);
		
		//  Apply Calibration
		for (i=0;i<nPoints;i++){
			cal_apply_cal(rawPtr[i], calPtr[i], &temp_cal);
		}
		
		//  Axis Misalignments Calibration
		cal_axis_misalignments(calPtr, &temp_cal);
				
		//  Overwrite New Measured Calibration Coefficients
		calStructPtr->axmYX = temp_cal.axmYX;
		calStructPtr->axmZX = temp_cal.axmZX;
		calStructPtr->axmZY = temp_cal.axmZY;
		for (i=0;i<3;i++){
			calStructPtr->gain[i] = temp_cal.gain[i];
			calStructPtr->offset[i] = temp_cal.offset[i];

		}
		
		//  Apply Full calibration with old & new coefficients
		for (i=0;i<nPoints;i++){
			cal_apply_cal(rawPtr[i], calPtr[i], calStructPtr);
		}
		
	}
	

	
	//  Evaluate Results
	get_time();//  Get current time
	cal_report.software_version = SOFTWARE_VERSION;
	memcpy(&cal_report.time_quick_azm,&current_time,sizeof(current_time));
	//  Magnitude standard deviation
	cal_report.mag_stdev_m1 = calc_mag_stdev(m1Cal);
	cal_report.mag_stdev_m2 = calc_mag_stdev(m2Cal);
	//  Evaluate Axis Disparity Standard Deviation
	for (i=0;i<3;i++){
		cal_report.disp_stdev_comp[i] = calc_disp_stdev(m1Cal, m2Cal, i);
	}
	
	//  Save Results
	SD_save_raw_data(azm_quick);
	SD_add_cal_history(azm_quick);	
	//  Save data to EEPROM
	save_calibration();

	
}



void cal_loop_test(struct MEASUREMENT_FULL *meas_inst){
	float N1, E1, D1;
	float dN1, dE1, dD1;
	float delta_horizontal;
	
	
	// increment distance and point counter
	nPoints = nPoints+1;
	loop_distance = loop_distance + meas_inst->distMeters;
	// find current position
	N1 = loop_horizontal*cos(DEG2RAD*loop_azimuth);
	E1 = loop_horizontal*sin(DEG2RAD*loop_azimuth);
	D1 = loop_vertical;
	// find difference in position
	delta_horizontal = meas_inst->distMeters*cos(DEG2RAD*meas_inst->inclination);
	dN1 = delta_horizontal*cos(DEG2RAD*meas_inst->azimuth);
	dE1 = delta_horizontal*sin(DEG2RAD*meas_inst->azimuth);
	dD1 = meas_inst->distMeters*sin(DEG2RAD*meas_inst->inclination);
	//  add  new offsets
	N1 = N1 + dN1;
	E1 = E1 + dE1;
	D1 = D1 + dD1;
	//  Calculate new horizontal, vertical, azimuth
	loop_horizontal = sqrt(pow(N1,2)+pow(E1,2));
	loop_vertical = D1;
	loop_azimuth = RAD2DEG*atan2(E1, N1);
	
	loop_error = sqrt(pow(loop_horizontal,2) + pow(loop_vertical,2))/loop_distance;
	
}


void cal_add_dist(struct MEASUREMENT_FULL *meas_inst){
	float avg_raw;
	uint8_t k;
	
	if(ind_buf>=SHOT_SIZE){
		//  Bump everything down 1 slot
		for(k=0;k<(SHOT_SIZE-1);k++){
			dist_raw_buf[k] = dist_raw_buf[k+1];
		}
		ind_buf--;
	}
	dist_raw_buf[ind_buf] = meas_inst->distRaw;
	ind_buf++;
	
	avg_raw = meanArr(dist_raw_buf, ind_buf);
	
	// temp_dist_offset always in meters
	if (options.current_unit_dist == feet){
		// If measurements are in feet, find offset in meters
		temp_dist_offset = (DIST_CAL_SETPOINT_FT - avg_raw)/MT2FT;
	}else{
		//  Measurement already in meters
		temp_dist_offset = DIST_CAL_SETPOINT_MT-avg_raw;
	}
	
	
	//  Fill in values to display
	for (k=0;k<ind_buf;k++){
		if (options.current_unit_dist == feet){
			dist_disp_buf[k] = dist_raw_buf[k]+temp_dist_offset*MT2FT;
		}else{
			dist_disp_buf[k] = dist_raw_buf[k]+temp_dist_offset;
		}
		
	}
	
	groupPoints = groupPoints+1;
	
	
	
}




void cal_inc_azm_eval(void){
	uint32_t i, p, g, k, ind1;
	struct MEASUREMENT_FULL temp_meas;
	uint8_t wrap_around;
	float inc_group[4];
	float azm_group[4];
	float inc_avg, azm_avg;
	float inc_err_array[NBUFF];
	float azm_err_array[NBUFF];
	
	//  Fill Out data for report structure
	get_time();//  Get current time
	cal_report.software_version = SOFTWARE_VERSION;
	cal_report.points = nPoints;
	cal_report.groups = nGroups;
	//  Don't copy in time; this was already done at time of data collection
	//memcpy(&cal_report.time_inc_azm,&current_time,sizeof(current_time));

	//  Calculate azimuth, incination, and roll for all data
	for (p=0;p<nPoints;p++){
		for (i=0;i<3;i++){
			temp_meas.a1Cal[i] = a1Cal[p][i];
			temp_meas.a2Cal[i] = a2Cal[p][i];
			temp_meas.m1Cal[i] = m1Cal[p][i];
			temp_meas.m2Cal[i] = m2Cal[p][i];
		}
		calc_orientation(&temp_meas);
		azimuth[p] = temp_meas.azimuth;
		inclination[p] = temp_meas.inclination;
		roll[p] = temp_meas.roll;
	}
	
	//  evaluate metrics of calibration
	for (g=0;g<nGroups;g++){
		
		//  Create group of shots
		wrap_around = 0;
		for (k=0;k<GROUP_SIZE;k++){
			ind1 = g*GROUP_SIZE+k;
			inc_group[k] = inclination[ind1];
			azm_group[k] = azimuth[ind1];
			if (azm_group[k]>340){
				wrap_around = 1;//  Possible wrap-around on azimuth
			}
		}
		
		//  Adjust for angle wrap-around
		if (wrap_around){
			for (k=0;k<GROUP_SIZE;k++){
				if (azm_group[k]<20){
					azm_group[k] = azm_group[k]+360;
				}
			}
		}
		
		//  Find group average
		inc_avg = meanArr(inc_group, GROUP_SIZE);
		azm_avg = meanArr(azm_group, GROUP_SIZE);
		
		//inc_avg = 0;
		//azm_avg = 0;
		//for (k=0;k<GROUP_SIZE;k++){
		//	inc_avg = inc_avg + inc_group[k];
		//	azm_avg = azm_avg + azm_group[k];
		//}
		//inc_avg = inc_avg/GROUP_SIZE;
		//azm_avg = azm_avg/GROUP_SIZE;
		
		//  Calculate errors
		for (k=0;k<GROUP_SIZE;k++){
			ind1 = g*GROUP_SIZE+k;
			inc_err_array[ind1] = inc_group[k]-inc_avg;
			azm_err_array[ind1] = (azm_group[k]-azm_avg)*cos(inc_avg*DEG2RAD);
		}
		
		
		
	}
	
	//  Calculate standard deviation of angle errors
	cal_report.inc_angle_err = stdev(inc_err_array, nPoints);	 
	cal_report.azm_angle_err = stdev(azm_err_array, nPoints);
	
	//  Evaluate magnitude error Standard Deviation
	cal_report.mag_stdev_a1 = calc_mag_stdev(a1Cal);
	cal_report.mag_stdev_a2 = calc_mag_stdev(a2Cal);
	cal_report.mag_stdev_m1 = calc_mag_stdev(m1Cal);
	cal_report.mag_stdev_m2 = calc_mag_stdev(m2Cal);
	
	//  Evaluate Axis Disparity Standard Deviation
	for (i=0;i<3;i++){
		cal_report.disp_stdev_acc[i]  = calc_disp_stdev(a1Cal, a2Cal, i);
		cal_report.disp_stdev_comp[i] = calc_disp_stdev(m1Cal, m2Cal, i);
	}
		
	
}

void cal_add_datapoint(struct MEASUREMENT_FULL *meas_inst){
	static float aX_ang_ref, mX_ang_ref;
	float aX_ang, mX_ang, foo;
	float aDelta, mDelta;
	float aXYZ[3], mXYZ[3];
	uint8_t j;
	uint32_t ind_stack;
	
	
	//  If this is the first point, initialize reference angles
	if ((nGroups==0) && (ind_buf==0) && (groupPoints == 0)){
		aX_ang_ref = 200;//  Ensures delta will fail on first iteration
		mX_ang_ref = 200;//  Ensures delta will fail on first iteration
	}
	
	
	//  Find X-axis vector angle
	for (j=0;j<3;j++){
		aXYZ[j] = 0.5*(meas_inst->a1Raw[j]+meas_inst->a2Raw[j]);
		mXYZ[j] = 0.5*(meas_inst->m1Raw[j]+meas_inst->m2Raw[j]);
	}
	calc_theta_XY(aXYZ, &foo, &aX_ang);
	calc_theta_XY(mXYZ, &foo, &mX_ang);
	aDelta = fabs(aX_ang - aX_ang_ref);
	mDelta = fabs(mX_ang - mX_ang_ref);
	
	//  Check to see if this measurement is a new group
	if ((aDelta>DELTA_ANG_MIN)||(mDelta>DELTA_ANG_MIN)){
		//  New group encountered
		if (groupPoints>= GROUP_SIZE){
			//  Enough points in last group	
			currentGroup++;
			nPoints =nGroups*GROUP_SIZE;
		}
		groupPoints = 0;
		ind_buf = 0;
		aX_ang_ref = aX_ang;
		mX_ang_ref = mX_ang;
	}
	
	//  Add Point to raw buffer
	ind_stack = (currentGroup-1)*GROUP_SIZE+ind_buf;
	//  Precaution to ensure no overflow
	if (ind_stack>=NBUFF){
		ind_stack = NBUFF-1;
	}
	for (j=0;j<3;j++){
		a1Raw[ind_stack][j] = meas_inst->a1Raw[j];
		a2Raw[ind_stack][j] = meas_inst->a2Raw[j];
		m1Raw[ind_stack][j] = meas_inst->m1Raw[j];
		m2Raw[ind_stack][j] = meas_inst->m2Raw[j];
	}
	groupPoints++;
	ind_buf++;
	if (ind_buf>= GROUP_SIZE){
		//  Buffer filled
		nGroups = currentGroup;
		ind_buf = 0; //  Roll over
	}
	
	
}







void cal_axis_misalignments(float XYZ[NBUFF][3], struct INST_CAL *cal_struct){
	float D[NBUFF][4];
	float V[NBUFF];
	uint32_t i, j, k;
	float DtD [6][6];
	float DtV[4];
	float Res[4];
	// D*Res = V;
	//  D'D*Res = D'*V;
	// inv(D'D)*D'D*Res = Res = inv(D'D)*(D'*V);
	
	
	// Create D
	for (k=0;k<nPoints;k++){
		D[k][0] = 1;
		D[k][1] = -2*XYZ[k][0]*XYZ[k][1];
		D[k][2] = -2*XYZ[k][1]*XYZ[k][2];
		D[k][3] = -2*XYZ[k][0]*XYZ[k][2];
		
		V[k] = pow(XYZ[k][0],2)+pow(XYZ[k][1],2)+pow(XYZ[k][2],2);
	}

	for (i=0;i<4;i++){
		for (j=0;j<4;j++){
			DtD[i][j] = 0;
			for (k=0;k<nPoints;k++){
				DtD[i][j] = DtD[i][j] + D[k][i]*D[k][j];
			}
		}
	}
	
	// Dt*V
	for (i=0;i<4;i++){
		DtV[i] = 0;
		for (k=0;k<nPoints;k++){
			DtV[i] = DtV[i]+D[k][i]*V[k];
		}
	}
	
	//  Inv(D'D)
	inverse(DtD, DtD, 4);

	//  Find result
	for (i=0;i<4;i++){
		Res[i] = 0;
		for (j=0;j<4;j++){
			Res[i] = Res[i]+DtD[i][j]*DtV[j];
			
		}
	}
	
	
	cal_struct->axmYX += Res[1];
	cal_struct->axmZY += Res[2];
	cal_struct->axmZX += Res[3];

}




void cal_angleX(float XYZ1[][3], float XYZ2[][3], struct INST_CAL *cal_struct){
	float D[NBUFF];
	float V[NBUFF];
	float DtD, DtV, Res;
	uint32_t i;
	
	// D = Y2-Z2
	// V = Y2-Y1 + Z2-Z1;
	for (i=0;i<nPoints;i++){
		D[i] = XYZ2[i][1]-XYZ2[i][2];
		V[i] = XYZ2[i][1]-XYZ1[i][1]+XYZ2[i][2]-XYZ1[i][2];
	}
	
	// DtD = D'*D;
	DtD = 0;
	for (i=0;i<nPoints;i++){
		DtD = DtD + pow(D[i],2);
	}
	// DtV = D'*V;
	DtV = 0;
	for (i=0;i<nPoints;i++){
		DtV = DtV + D[i]*V[i];
	}
	
	// Res = (1/DtD)*DtV;
	Res = (1/DtD)*DtV;
	
	cal_struct->thetaX += -1*RAD2DEG*Res;

	// Create rotation matrix
	gen_RotM(cal_struct);
	
}




void cal_angleYZ(float XYZ[][3], struct INST_CAL *cal_struct){
	//  This code assumes input data has been calibrated for gain and offset
	//  But no rotation calibration has been applied
	float roll_ang[NBUFF];
	float x_ang_shift[NBUFF];
	uint8_t ind, i, j, k, np;
	float group_avg;

	//  Variables for line fitting
	float detX;
	float X[NBUFF][2];
	float tempX1[2][2], tempX2[2][2], tempX3[2], B[2];
	
	
	// Create shifted x_ang array
	for (k=0;k< nGroups;k++){
		//  Calculate Vector Angle from X-Axis
		for (j=0;j<GROUP_SIZE;j++){
			ind = k*GROUP_SIZE + j;
			calc_theta_XY(&XYZ[ind][0], &roll_ang[ind], &x_ang_shift[ind]);

		}
		//  Find Group Average X-axis angle
		group_avg = meanArr( &x_ang_shift[k*GROUP_SIZE],GROUP_SIZE);
		//  Subtract group average from x-axis angle
		for (j=0;j<GROUP_SIZE;j++){
			ind = k*GROUP_SIZE + j;
			x_ang_shift[ind] = x_ang_shift[ind]-group_avg;
		}
	}

	
	//  X = [cosd(roll_fit), sind(roll_fit)]
	for (np=0;np<nPoints;np++){
		X[np][0] = cos(DEG2RAD*roll_ang[np]);
		X[np][1] = sin(DEG2RAD*roll_ang[np]);
	}
	
	// tempX1 = X'*X;
	for (i=0;i<2;i++){
		for (j=0;j<2;j++){
			tempX1[i][j]=0;
			for (np=0;np<nPoints;np++){
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
		for (np=0;np<nPoints;np++){
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
	cal_struct->thetaY += B[0];
	cal_struct->thetaZ += -1*B[1];
	
	// Create rotation matrix
	gen_RotM(cal_struct);


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
	//  Rotate vector by rotation matrix
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
	
	cal_struct->dist_offset = 0;
	
	cal_struct->Cal_Initialized_Key = 0xB2;//  Indicator that structure has been initialized
	
}

void gen_RotM(struct INST_CAL *cal_struct){
	float tX, tY, tZ;
	
	tX = cal_struct->thetaX*DEG2RAD;
	tY = cal_struct->thetaY*DEG2RAD;
	tZ = cal_struct->thetaZ*DEG2RAD;
	
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
	float D[NBUFF][6];
	float temp1[6][6], temp2[6];
	float v[6];
	float g;
	uint8_t i,j,k;
	

	
	
	for (i=0;i<nPoints;i++){
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
			for (k=0;k<nPoints;k++){
				temp1[i][j] = temp1[i][j] + D[k][i]*D[k][j];
				
				
			}
		}
	}
	
	inverse(temp1, temp1, 6);
	
	for (i=0;i<6;i++){
		temp2[i] = 0;
		for (k = 0;k<nPoints;k++){
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
		cal_struct->offset[i] += -1*(v[i+3]/v[i]);
	}
	
	
	g=1 + pow(v[3],2)/v[0]+pow(v[4],2)/v[1]+pow(v[5],2)/v[2];
	
	
	for (i=0;i<3;i++){
		cal_struct->gain[i] *= sqrt(g/v[i]);
		
	}
	

}



