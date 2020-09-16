/*
 * SDcardBRIC.c
 *
 * Created: 8/30/2020 10:23:02 AM
 *  Author: Kris Fausnight
 */ 
#include <SDcardBRIC.h>




FRESULT SD_add_cal_history(enum CALTYPE calType){
	FIL file1;
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	UINT bytesWritten;
	char str_temp[10];
	uint32_t i, j;
	struct INST_CAL *calStPtr;
	struct TIME *timePtr;
	
	//  Set up SD card
	//spi_select_slave(&spi_main, &slave_SD, true);

	diskio_status = disk_status(0);

	if(diskio_status){
		//Possibly card not initialized
		configure_SD();
		diskio_status = disk_status(0);
		if(diskio_status){
			fdebug1 = FR_NOT_READY;
			SD_status = fdebug1;
			//config_spi(LCD);
			return fdebug1;
		}
		
	}
	
	sprintf(filename, "Calibration_History_%04d.csv", options.SerialNumber);
	fdebug1 = f_open(&file1,filename, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	if ((fdebug1!=FR_OK) && (fdebug1!=FR_NO_FILE)){
		//  Something failed, exit function
		return fdebug1;
	}
	if (fdebug1 == FR_NO_FILE){
		// File does not exist, create new file with header
		fdebug1 = f_open(&file1, filename, FA_CREATE_NEW | FA_READ | FA_WRITE);
		
		if(fdebug1!=FR_OK){
			SD_status = fdebug1;
			return fdebug1;
		}

		sprintf(write_str1,"YYYY.MM.DD, HH:mm:ss,Calibration Type,SN,Software Version,Temp C,Temp F, ");
		f_write(&file1, write_str1, strlen(write_str1), &bytesWritten);
		
		sprintf(write_str1,"Rangefinder Offset (Mt), Rangefinder Offset (ft), Groups, Points,Inc Angle Error stdev (deg),Azm Angle Error stdev (deg),Acc1 magnitude stdev (%%),Acc2 magnitude stdev (%%),Mag1 magnitude stdev (%%),Mag2 magnitude stdev (%%),");
		f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
		
		sprintf(write_str1,"Acc X disparity (%%),Acc Y disparity (%%),Acc Z disparity (%%),Mag X disparity (%%),Mag Y disparity (%%),Mag Z disparity (%%),");
		f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
		
		for (i=0;i<4;i++){
			switch (i){
				case 0:
				strcpy(str_temp,"Acc1");
				break;
				case 1:
				strcpy(str_temp,"Acc2");
				break;
				case 2:
				strcpy(str_temp,"Mag1");
				break;
				case 3:
				strcpy(str_temp,"Mag2");
				break;
			}
			
			sprintf(write_str1,"%s X Offset,%s Y Offset,%s Z Offset,%s X Gain,%s Y Gain,%s Z Gain,",str_temp,str_temp,str_temp,str_temp,str_temp,str_temp);
			f_write(&file1, write_str1, strlen(write_str1), &bytesWritten);
			
			sprintf(write_str1,"%s YX Misalignment (deg),%s ZY Misalignment (deg),%s ZX Misalignment (deg),",str_temp,str_temp,str_temp);
			f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
			
			sprintf(write_str1,"%s thetaX Misalignment (deg),%s thetaY Misalignment (deg),%s thetaZ Misalignment (deg),",str_temp,str_temp,str_temp);
			f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
		}

		
		
		}else if(fdebug1 != FR_OK){
		SD_status = fdebug1;
		return fdebug1;
	}
	
	// Append data file
	fdebug2 = f_lseek(&file1, f_size(&file1));
	
	//  Add entry
	switch (calType){
		case inc_azm_full:
		strcpy(write_str2,"Inclination & Azimuth Full Calibration");
		timePtr = &cal_report.time_inc_azm;
		break;
		case azm_quick:
		strcpy(write_str2,"Azimuth Quick Calibration");
		timePtr = &cal_report.time_quick_azm;
		break;
		case rangeFinder:
		strcpy(write_str2,"Rangefinder Calibration");
		timePtr = &cal_report.time_rangeFinder;
		break;
	}
	sprintf(write_str1,"\r\n20%02x.%02x.%02x,%02x:%02x:%02x,%s,%04d,%0.1f,%0.1f,%0.1f,",
	timePtr->year, timePtr->month, timePtr->date,timePtr->hours, timePtr->minutes,timePtr->seconds, write_str2, options.SerialNumber, SOFTWARE_VERSION,
	timePtr->temperatureC,timePtr->temperatureF);
	f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	
	sprintf(write_str1,"%0.6f,%0.6f,%d,%d,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,",
	dist_calst.dist_offset, dist_calst.dist_offset*MT2FT, cal_report.groups, cal_report.points,
	cal_report.inc_angle_err, cal_report.azm_angle_err,cal_report.mag_stdev_a1*100, cal_report.mag_stdev_a2*100, cal_report.mag_stdev_m1*100, cal_report.mag_stdev_m2*100);
	f_write(&file1, write_str1, strlen(write_str1), &bytesWritten);
	
	sprintf(write_str1,"%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,",
	cal_report.disp_stdev_acc[0]*100, cal_report.disp_stdev_acc[1]*100, cal_report.disp_stdev_acc[2]*100,
	cal_report.disp_stdev_comp[0]*100, cal_report.disp_stdev_comp[1]*100, cal_report.disp_stdev_comp[2]*100);
	f_write(&file1, write_str1, strlen(write_str1), &bytesWritten);
	for (i=0;i<4;i++){
		switch (i){
			case 0:
			calStPtr = &a1_calst;
			break;
			case 1:
			calStPtr = &a2_calst;
			break;
			case 2:
			calStPtr = &m1_calst;
			break;
			case 3:
			calStPtr = &m2_calst;
			break;
			
		}
		sprintf(write_str1,"%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,",
		calStPtr->offset[0],calStPtr->offset[1], calStPtr->offset[2],
		calStPtr->gain[0],calStPtr->gain[1], calStPtr->gain[2]);
		f_write(&file1, write_str1, strlen(write_str1), &bytesWritten);
		
		sprintf(write_str1,"%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,",
		RAD2DEG*calStPtr->axmYX,RAD2DEG*calStPtr->axmZY, RAD2DEG*calStPtr->axmZX,
		calStPtr->thetaX,calStPtr->thetaY, calStPtr->thetaZ);
		f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
		
	}
	f_close(&file1);
	
	return fdebug1;
};


FRESULT SD_write_report(void){
	uint32_t i, j, k;
	//uint32_t bw;
	//uint32_t *pbw;
	FIL file1;
	UINT	bytesWritten;
	char file_name[100];
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	struct INST_CAL *pcal_struct;
	
	uint32_t group_current;
	float temperature;
	
	struct MEASUREMENT temp_meas;
	float azm_raw, inc_raw, roll_raw;
	float azm_cal, inc_cal, roll_cal;
	
	//  Set up SD card
	diskio_status = disk_status(0);

	if(diskio_status){
		//Possibly card not initialized
		configure_SD();
		diskio_status = disk_status(0);
		if(diskio_status){
			fdebug1 = FR_NOT_READY;
			SD_status = fdebug1;
			return fdebug1;
		}
		
	}
	
	
	
	sprintf(file_name, "20%02x%02x%02x_%02x%02x%02x_SN%04d_calibration_report.txt",
	current_time.year, current_time.month, current_time.date,
	current_time.hours, current_time.minutes, current_time.seconds,
	options.SerialNumber);
	fdebug2 = f_open(&file1, file_name, FA_CREATE_NEW | FA_READ | FA_WRITE);
	if(fdebug2!=FR_OK){
		SD_status = fdebug2;
		return fdebug2;
	}
	
	
	//  Write Header
	sprintf(write_str1,"Calibration Report\r\n\r\nDate and Time:\r\nYYYY.MM.DD@HH:mm:ss\r\n20%02x.%02x.%02x@%02x:%02x:%02x\r\n\r\n",
	cal_report.time_inc_azm.year, cal_report.time_inc_azm.month, cal_report.time_inc_azm.date,
	cal_report.time_inc_azm.hours, cal_report.time_inc_azm.minutes, cal_report.time_inc_azm.seconds);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	
	//  Write Version and SN
	sprintf(write_str1,"SN: %04d\r\nSoftware Version: %1.1f\r\n", options.SerialNumber, cal_report.software_version);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	
	//  Write Temperature
	sprintf(write_str1,"Temperature: %0.1f farenheit, %0.1f celsius\r\n\r\n", cal_report.time_inc_azm.temperatureF, cal_report.time_inc_azm.temperatureC);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1), &bytesWritten);
	
	//  Write Metrics
	// groups and points
	sprintf(write_str1,"Measurements: %d\r\n4-Point Groups: %d\r\n\r\n",
	cal_report.points, cal_report.groups);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	// angle error
	sprintf(write_str1,"Angle Error Standard Deviation:\r\n  Azimuth: %3.6f degrees\r\n  Inclination: %3.6f degrees\r\n",
	cal_report.azm_angle_err, cal_report.inc_angle_err);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	// Magnitude Error Standard Deviation
	sprintf(write_str1,"Magnitude Error Standard Deviation:\r\n  Accelerometer 1: %3.6f %%\r\n  Accelerometer 2: %3.6f %%\r\n  Compass 1: %3.6f %%\r\n  Compass 2: %3.6f %%\r\n",
	cal_report.mag_stdev_a1*100, cal_report.mag_stdev_a2*100,cal_report.mag_stdev_m1*100, cal_report.mag_stdev_m2*100);
	// Axis Disparity
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	sprintf(write_str1,"Axis Disparity:\r\n  Accelerometer: X-%3.6f%% Y-%3.6f%% Z-%3.6f%%\r\n",
	cal_report.disp_stdev_acc[0]*100, cal_report.disp_stdev_acc[1]*100,cal_report.disp_stdev_acc[2]*100);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	sprintf(write_str1,"  Compass: X-%3.6f%% Y-%3.6f%% Z-%3.6f%%\r\n\r\n\r\n",
	cal_report.disp_stdev_comp[0]*100, cal_report.disp_stdev_comp[1]*100,cal_report.disp_stdev_comp[2]*100);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);

	
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
			pcal_struct = &m1_calst;
			sprintf(write_str1,"Compass 1 Calibration:\r\n");
			break;
			case 3:
			pcal_struct = &m2_calst;
			sprintf(write_str1,"Compass 2 Calibration:\r\n");
			break;
		}
		sprintf(write_str2,"  Gain X: %.6f\r\n  Gain Y: %.6f\r\n  GainZ: %.6f\r\n", pcal_struct->gain[0], pcal_struct->gain[1], pcal_struct->gain[2]);
		strcat(write_str1, write_str2);
		sprintf(write_str2,"  Offset X: %.6f\r\n  Offset Y: %.6f\r\n  Offset Z: %.6f\r\n", pcal_struct->offset[0], pcal_struct->offset[1], pcal_struct->offset[2]);
		strcat(write_str1, write_str2);
		sprintf(write_str2,"  Axis Misalignment, Y-X: %.6f deg\r\n  Axis Misalignment, Z-X: %.6f deg\r\n  Axis Misalignment, Z-Y: %.6f deg\r\n",
		RAD2DEG*pcal_struct->axmYX, RAD2DEG*pcal_struct->axmZX, RAD2DEG*pcal_struct->axmZY);
		strcat(write_str1, write_str2);
		
		
		
		sprintf(write_str2,"  Package Misalignment About X: %.3f deg\r\n  Package Misalignment About Y: %.3f deg\r\n  Package Misalignment About Z: %.3f deg\r\n\r\n",
		pcal_struct->thetaX, pcal_struct->thetaY, pcal_struct->thetaZ);
		strcat(write_str1, write_str2);
		//fdebug2 = f_lseek(&file_cal, f_size(&file_cal));
		fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	}
	
	
	// Write Compass/Incl/Roll Data
	sprintf(write_str1,"\r\nCalibration Measurements:\r\n");
	sprintf(write_str2,"                Uncalibrated                     Calibrated\r\n");
	strcat(write_str1, write_str2);
	sprintf(write_str2,"Group, Point,   Azimuth,  Inclination, Roll,     Azimuth,  Inclination, Roll\r\n");
	strcat(write_str1, write_str2);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	for (i=0;i<nPoints;i++){
		//  Fist Measure uncalibrated raw data
		for (j=0;j<3;j++){
			//  Get uncalibrated data by falsely putting raw data into cal data location
			temp_meas.a1Cal[j] = a1Raw[i][j];
			temp_meas.a2Cal[j] = a2Raw[i][j];
			temp_meas.m1Cal[j] = m1Raw[i][j];
			temp_meas.m2Cal[j] = m2Raw[i][j];
		}
		calc_orientation(&temp_meas);
		azm_raw = temp_meas.azimuth;
		inc_raw = temp_meas.inclination;
		roll_raw = temp_meas.roll;
		//  Next Measure calibrated data
		for (j=0;j<3;j++){
			temp_meas.a1Cal[j] = a1Cal[i][j];
			temp_meas.a2Cal[j] = a2Cal[i][j];
			temp_meas.m1Cal[j] = m1Cal[i][j];
			temp_meas.m2Cal[j] = m2Cal[i][j];
		}
		calc_orientation(&temp_meas);
		azm_cal = temp_meas.azimuth;
		inc_cal = temp_meas.inclination;
		roll_cal = temp_meas.roll;
		group_current = floor(i/GROUP_SIZE)+1;
		sprintf(write_str1,"%-2d,    %-2d,      %03.2f,   %03.2f,      %03.2f,   %03.2f,   %03.2f,      %03.2f\r\n",
		group_current, i+1,  azm_raw, inc_raw, roll_raw, azm_cal, inc_cal, roll_cal);
		
		fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
		
	}
	
	
	f_close(&file1);
	
	
}






FRESULT SD_save_raw_data(enum CALTYPE calType ){
	uint32_t i, j, k;
	float raw_data_entry[12];
	FIL file1;
	UINT	 bytesWritten;
	char file_name[100];
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//write raw data file
	switch (calType){
		case azm_quick:
		sprintf(write_str1,"azmQuickRawData");
		break;
		case inc_azm_full:
		sprintf(write_str1,"_fullCalRawData");
		break;
	}
	sprintf(file_name, "20%02x%02x%02x_%02x%02x%02x_SN%04d_%s.bin",
	current_time.year, current_time.month, current_time.date,
	current_time.hours, current_time.minutes, current_time.seconds,
	options.SerialNumber, write_str1);
	
	fdebug2 = f_open(&file1, file_name, FA_CREATE_NEW | FA_READ | FA_WRITE);
	if(fdebug2 != FR_OK){
		// Some other SD card error
		SD_status = fdebug2;
		//config_spi(LCD);
		return fdebug2;
	}
	
	
	// Write File
	for (i=0;i<nPoints;i++){
		for (k=0;k<3;k++){
			raw_data_entry[k] = a1Raw[i][k];
			raw_data_entry[k+3] = a2Raw[i][k];
			raw_data_entry[k+6] = m1Raw[i][k];
			raw_data_entry[k+9] = m2Raw[i][k];
			
		}
		//fdebug2 = f_lseek(&file_cal, f_size(&file_cal));
		fdebug3 = f_write(&file1, raw_data_entry, sizeof(raw_data_entry),  &bytesWritten);
	}
	
	f_close(&file1);
	
	
	return fdebug3;
	
	
}


FRESULT save_measurement(struct MEASUREMENT *meas_inst){
	FIL file1;
	UINT  bytesWritten;
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	
	
	//  Format data for text data file
	sprintf(filename, "20%02x%02x%02x_SN%04d_datafile.csv", current_time.year, current_time.month, current_time.date, options.SerialNumber);
	
	fdebug1 = f_open(&file1, filename, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	
	if ((fdebug1!=FR_OK) && (fdebug1!=FR_NO_FILE)){
		//  Something failed, exit function
		return fdebug1;
	}
	
	if (fdebug1 == FR_NO_FILE){
		// File does not exist, create new file with header
		fdebug2 = f_open(&file1, filename, FA_CREATE_NEW | FA_READ | FA_WRITE);
		
		if(fdebug2!=FR_OK){
			SD_status = fdebug2;
			return fdebug2;
		}

		if (options.current_unit_dist == feet){
			sprintf(write_str2, "Time-Stamp, Index, Distance (meters), Azimuth (degrees), Inclination (degrees), Temperature (Celsius),  Error Log\r\n");
			fdebug2 = f_write(&file1, write_str2, strlen(write_str2),  &bytesWritten);
			}else{
			sprintf(write_str2, "Time-Stamp, Index, Distance (feet), Azimuth (degrees), Inclination (degrees), Temperature (Fahrenheit), Error Log\r\n");
			fdebug2 = f_write(&file1, write_str2, strlen(write_str2),  &bytesWritten);
		}
		
		
		}else if(fdebug1 != FR_OK){
		SD_status = fdebug1;
		return fdebug1;
	}
	
	// Format string for timestamps
	sprintf(write_str1,"20%02x.%02x.%02x@%02x:%02x:%02x,",
	current_time.year, current_time.month, current_time.date,
	current_time.hours, current_time.minutes, current_time.seconds);
	// Format string for data
	sprintf(write_str2," %d, %.3f, %.3f, %.3f,",
	meas_inst->index_ref, meas_inst->distCal, meas_inst->azimuth, meas_inst->inclination);
	strcat(write_str1, write_str2);
	//  Format string for temperature
	if (options.current_unit_dist == feet){
		sprintf(write_str2," %.3f,", current_time.temperatureF);
		}else{
		sprintf(write_str2," %.3f,", current_time.temperatureC);
	}
	strcat(write_str1, write_str2);
	//Format data for error_log
	//sprintf(write_str2,"Laser: %02x",meas_inst->laser_error_code);
	strcat(write_str1, write_str2);
	// Enter line return
	strcat(write_str1, "\r\n");
	
	// Append data file
	fdebug2 = f_lseek(&file1, f_size(&file1));
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	f_close(&file1);

	return fdebug3;
}


FRESULT configure_SD(void){
	FRESULT fdebug1;
	//spi_select_slave(&spi_main, &slave_SD, true);
	sd_mmc_init();
	
	disk_status(0);

	fdebug1 = f_mount(0, &FatFS);
	//spi_select_slave(&spi_main, &slave_SD, false);
	
	return fdebug1;
}



uint32_t getSN(void){
	char snStr[4];
	uint8_t i;
	
	FIL file1;
	UINT	bytesWritten;
	FRESULT fdebug1;
	
	fdebug1 = f_open(&file1,"SN.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	if (fdebug1!=FR_OK){
		//  Something failed, exit function
		return options.SerialNumber;
	}
	
	f_read(&file1, snStr, 4, &bytesWritten);
	
	uint32_t mult = 1000;
	uint8_t temp;
	uint32_t SN = 0;
	for (i=0;i<4;i++){
		temp = snStr[i] & 0x0f;
		SN = SN + (temp*mult);
		mult = mult/10;
		
	}
	f_close(&file1);
	
	return SN;


}

