/*
 * SDcardBRIC.c
 *
 * Created: 8/30/2020 10:23:02 AM
 *  Author: Kris Fausnight
 */ 
#include <SDcardBRIC.h>

#define FILENAME_RAW_DATA_BUFFER "rawDataBuffer.bin"


FRESULT SD_add_cal_history(enum CALTYPE calType){
	FIL file1;
	FRESULT fdebug1, fdebug2;
	DSTATUS diskio_status;
	UINT bytesWritten;
	char str_temp[10];
	uint32_t i;
	struct INST_CAL *calStPtr;
	struct TIME *timePtr;
	float calTempC;
	
	//  Exit if USB is attached
	if (SD_WriteLockout){
		return FR_DENIED;
	}
	
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
	
	//  Change current directory 
	fdebug1 = SD_change_directory(folderCalibration);
	if(fdebug1!=FR_OK){
		return fdebug1;
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

		sprintf(write_str1,"YYYY.MM.DD@HH:mm:ss,Calibration Type,SN,Software Version,Temp C, ");
		f_write(&file1, write_str1, strlen(write_str1), &bytesWritten);
		
		sprintf(write_str1,"Rangefinder Offset (Mt), Groups, Points,Inc Angle Error stdev (deg),Azm Angle Error stdev (deg),Acc1 magnitude stdev (%%),Acc2 magnitude stdev (%%),Mag1 magnitude stdev (%%),Mag2 magnitude stdev (%%),");
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
		calTempC = cal_report.tempC_inc_azm;
		break;
		case azm_quick:
		strcpy(write_str2,"Azimuth Quick Calibration");
		timePtr = &cal_report.time_quick_azm;
		calTempC = cal_report.tempC_quick_azm;
		break;
		case rangeFinder:
		strcpy(write_str2,"Rangefinder Calibration");
		timePtr = &cal_report.time_rangeFinder;
		calTempC = cal_report.tempC_rangeFinder;
		break;
	}
	genTimestampString(display_str, timePtr, 1);
	sprintf(write_str1,"\r\n%s,%s,%04d,%0.1f,%0.1f,",
		display_str, write_str2, options.SerialNumber, SOFTWARE_VERSION,	calTempC);
	f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	
	sprintf(write_str1,"%0.6f,%d,%d,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,",
	dist_calst.dist_offset, cal_report.groups, cal_report.points,
	cal_report.inc_angle_err, cal_report.azm_angle_err,
	cal_report.mag_stdev_a1*100, cal_report.mag_stdev_a2*100, 
	cal_report.mag_stdev_m1*100, cal_report.mag_stdev_m2*100);
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
	uint32_t i, j;
	//uint32_t bw;
	//uint32_t *pbw;
	FIL file1;
	UINT	bytesWritten;
	char file_name[100];
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	struct INST_CAL *pcal_struct;
	
	uint32_t group_current;
	
	struct MEASUREMENT_FULL temp_meas;
	float azm_raw, inc_raw, roll_raw;
	float azm_cal, inc_cal, roll_cal;
	
	
	//  Exit if USB is attached
	if (SD_WriteLockout){
		return FR_DENIED;
	}
	
	
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
	//  Change current directory 
	fdebug1 = SD_change_directory(folderCalibration);
	if(fdebug1!=FR_OK){
		return fdebug1;
	}
	
	genTimestampString(write_str2, &current_time, 2);
	sprintf(file_name, "Calibration_Report_%s_SN%04d.txt",
		write_str2,	options.SerialNumber);
	fdebug2 = f_open(&file1, file_name, FA_CREATE_NEW | FA_READ | FA_WRITE);
	if(fdebug2!=FR_OK){
		SD_status = fdebug2;
		return fdebug2;
	}
	
	
	//  Write Header
	genTimestampString(write_str2, &current_time, 1);
	sprintf(write_str1,"Calibration Report\r\n\r\nDate and Time:\r\nYYYY.MM.DD@HH:mm:ss\r\n%s\r\n\r\n",write_str2);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	
	//  Write Version and SN
	sprintf(write_str1,"SN: %04d\r\nSoftware Version: %1.1f\r\n", options.SerialNumber, cal_report.software_version);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	
	//  Write Temperature
	sprintf(write_str1,"Temperature: %0.2f celsius, %0.2f fahrenheit\r\n\r\n", 
			cal_report.tempC_inc_azm, celsius2fahrenheit(cal_report.tempC_inc_azm));
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
	sprintf(write_str1,"Axis Disparity:\r\n  Accelerometer: X: %3.6f%% Y: %3.6f%% Z: %3.6f%%\r\n",
	cal_report.disp_stdev_acc[0]*100, cal_report.disp_stdev_acc[1]*100,cal_report.disp_stdev_acc[2]*100);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	sprintf(write_str1,"  Compass: X: %3.6f%% Y: %3.6f%% Z: %3.6f%%\r\n\r\n\r\n",
	cal_report.disp_stdev_comp[0]*100, cal_report.disp_stdev_comp[1]*100,cal_report.disp_stdev_comp[2]*100);
	fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);


	//  Write group removal info
	for(i=0;i<MAX_BAD_GROUPS;i++){
		if(cal_report.groupRemoved[i]==0){	break;	}
		
		if(cal_report.groupRemovedSource[i]==1){
			strcpy(write_str2,"Inclination");
		}else{
			strcpy(write_str2,"Azimuth");
		}
		sprintf(write_str1,"Removed Group %d for %f deg improvement in %s\r\n",
				cal_report.groupRemoved[i], cal_report.groupRemovedImprovement[i], write_str2);
		fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
	}
	strcpy(write_str1,"\r\n\r\n");
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
		sprintf(write_str1,"%02d,    %02d,      %06.2f,   %+06.2f,      %06.2f,   %06.2f,   %+06.2f,      %06.2f\r\n",
		group_current, i+1,  azm_raw, inc_raw, roll_raw, azm_cal, inc_cal, roll_cal);
		
		fdebug3 = f_write(&file1, write_str1, strlen(write_str1),  &bytesWritten);
		
	}
	
	
	f_close(&file1);
	
	return fdebug1;
}






FRESULT SD_save_raw_data(enum CALTYPE calType ){
	uint32_t i, k;
	float raw_data_entry[12];
	FIL file1;
	UINT	 bytesWritten;
	char file_name[100];
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	
	
	//  Exit if USB is attached
	if (SD_WriteLockout){
		return FR_DENIED;
	}
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//  Change current directory 
	fdebug1 = SD_change_directory(folderCalibration);
	if(fdebug1!=FR_OK){
		return fdebug1;
	}
	
	
	//write raw data file
	switch (calType){
		case azm_quick:
			strcpy(write_str1,"azmQuick");
			break;
		case inc_azm_full:
			strcpy(write_str1,"fullCal");
			break;
	}
	genTimestampString(write_str2, &current_time, 2);
	sprintf(file_name, "raw_%s_SN%04d_%s.bin",
		write_str2,	options.SerialNumber, write_str1);
	
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
	FIL tempFile;
	UINT  bytesWritten;
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	
	
	//  Exit if USB is attached
	if (SD_WriteLockout){
		return FR_DENIED;
	}
	
	//**********************************************************************************
	//*************  Write ASCII .csv File
	//**********************************************************************************
	
	//  Change current directory to "/data"
	//  Change current directory 
	fdebug1 = SD_change_directory(folderData);
	if(fdebug1!=FR_OK){
		return fdebug1;
	}

	//  Format data for text data file
	genTimestampString(write_str1, &current_time,  3);
	sprintf(filename, "data_%s_SN%04d.csv", write_str1, options.SerialNumber);
	
	fdebug1 = f_open(&tempFile, filename, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	
	if ((fdebug1!=FR_OK) && (fdebug1!=FR_NO_FILE)){
		//  Something failed, exit function
		return fdebug1;
	}
	
	if (fdebug1 == FR_NO_FILE){
		// File does not exist, create new file with header
		fdebug2 = f_open(&tempFile, filename, FA_CREATE_NEW | FA_READ | FA_WRITE);
		
		if(fdebug2!=FR_OK){
			SD_status = fdebug2;
			return fdebug2;
		}
		//   Write file header
		sprintf(write_str2, "Time-Stamp, POSIX Time, Index, Distance (meters), Azimuth (deg), Inclination (deg), Dip (deg), Roll (deg), Temperature (Celsius),  Measurement Type, Error Log\r\n");
			fdebug2 = f_write(&tempFile, write_str2, strlen(write_str2),  &bytesWritten);
		
	}else if(fdebug1 != FR_OK){
		SD_status = fdebug1;
		return fdebug1;
	}
	
	// Write string for timestamps
	genTimestampString(display_str, &current_time, 1);
	//  Write string for error log
	gen_err_message(write_str2, meas_inst, 0);
	//  Write string for measurement type
	char measTypeStr[10];
	switch (meas_inst->meas_type){
		case measScan:
			strcpy(measTypeStr,"Scan");
			break;
		case measRegular:
			strcpy(measTypeStr,"Regular");
			break;
		default:
			strcpy(measTypeStr,"Unknown");
			break;
	}
	// Format string for data entry
	sprintf(write_str1,"%s,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%s\r\n",
		display_str, meas_inst->posix_time, meas_inst->refIndex, 
		meas_inst->distMeters, 
		meas_inst->azimuth, meas_inst->inclination, meas_inst->dip, meas_inst->roll, 
		meas_inst->temperatureC,measTypeStr, write_str2);
	
	// Append data file
	fdebug2 = f_lseek(&tempFile, f_size(&tempFile));
	fdebug3 = f_write(&tempFile, write_str1, strlen(write_str1),  &bytesWritten);
	f_close(&tempFile);
	
	
	
	
	//**********************************************************************************
	//*************  Write Binary File to Data Buffer
	//**********************************************************************************
	FIL bufferFile;
	
	fdebug1 = f_open(&bufferFile, FILENAME_RAW_DATA_BUFFER, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	
	if (fdebug1 == FR_NO_FILE){
		//  File does not exist yet
		//  Create the file and initialize the sync tracker
		initSyncTracker();
	}else if (fdebug1!=FR_OK){
		//  Something failed, exit function
		return fdebug1;
	}
	
	
	// Append data file
	fdebug2 = f_lseek(&bufferFile, f_size(&bufferFile));
	fdebug3 = f_write(&bufferFile, meas_inst, sizeof(*meas_inst),  &bytesWritten);
	f_close(&bufferFile);
	
	//  Update BLE Sync Tracker
	incrementSyncTracker(meas_inst);
		
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



void getSN(void){
	char snStr[4];
	uint8_t i;
	
	FIL file1;
	UINT	bytesWritten;
	FRESULT fdebug1;
	
	//  Ensure back in main directory
	fdebug1 = f_chdir("0:/");
	//  Look for file
	fdebug1 = f_open(&file1,"SN.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	if (fdebug1!=FR_OK){
		//  Something failed, exit function
		//  Leave SN as is
		return ;
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
	
	if (SN!=options.SerialNumber){
		options.SerialNumber = SN;
		save_user_settings();
	}
	
	
	return;


}





FRESULT SD_change_directory(enum FOLDER_TYPE tempFolder){
	FRESULT fdebug1;
	char folderName[20];
	
	//  Get directory name
	switch (tempFolder){
		case folderData:
			strcpy(folderName,"data");
			break;
		case folderRaw:
			strcpy(folderName,"raw");
			break;
		case folderCalibration:
			strcpy(folderName,"calibration");
			break;
	}
	
	//  Get current directory
	fdebug1 = f_getcwd(write_str1, sizeof(write_str1));
	if(fdebug1!=FR_OK){
		return fdebug1;
	}
	
	//  Check to see if it's the correct directory
	sprintf(write_str2,"0:/%s",folderName);
	if (strncmp(write_str1,write_str2,20)!=0){
		//  Wrong directory
		//  Try to change it
		fdebug1 = f_chdir(write_str2);
		if (fdebug1==FR_OK){
			return fdebug1;
		}else if(fdebug1==FR_NO_PATH){
			//  Make directory and change to it
			fdebug1 = f_chdir("0:/");
			fdebug1 = f_mkdir(folderName);
			fdebug1 = f_chdir(write_str2);
		}else{
			return fdebug1;
		}
		
		
	}
	return fdebug1;

}


void loadMeasBuffer(void){
	FIL bufferFile;
	UINT  bytesWritten;
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	uint32_t i;
	
	//  Open the file
	fdebug1 = SD_change_directory(folderData);
	fdebug1 = f_open(&bufferFile, FILENAME_RAW_DATA_BUFFER, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	if (fdebug1!=FR_OK){
		//  Something failed, exit function
		//  File won't exist initially
		
		return;
	}
	
	unsigned long fileSize, fileIndex;
	uint32_t nMeasSD, measSize, nMeasCopy;
	measSize = sizeof(measBuf[0]);
	fileSize = f_size(&bufferFile);
	// Find number of measurements stored:
	nMeasSD = fileSize/measSize;
	nMeasCopy = min(nMeasSD, N_MEASBUF);
	if(nMeasSD==0){
		f_close(&bufferFile);
		return;
	}
	
	
	//  Read Measurement
	for (measBufInd=0;measBufInd<nMeasCopy;measBufInd++){
		fileIndex = fileSize-((nMeasCopy-measBufInd)*measSize);
		fdebug1 = f_lseek(&bufferFile, fileIndex);
		fdebug2 = f_read(&bufferFile, &measBuf[measBufInd],measSize, &bytesWritten);
		refIndex = measBuf[measBufInd].refIndex;
	}
	
	f_close(&bufferFile);
	
	
}



/////////////////////////////////////////////////////////////////////////////////////////
//  BLE Measurement Synchronization
/////////////////////////////////////////////////////////////////////////////////////////

void incrementSyncTracker(struct MEASUREMENT *meas_inst){
	//  Add another measurement to stack
	bleSyncTracker.measStackEnd++;
	bleSyncTracker.refIndexEnd = meas_inst->refIndex;
	memcpy(&bleSyncTracker.timeEnd,&meas_inst->measTime,sizeof(&bleSyncTracker.timeEnd));
	
}


void SyncDataBLE(void){
	FIL bufferFile;
	UINT  bytesWritten;
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	struct MEASUREMENT tempMeas;
	static uint32_t lastMeasAttempted;
	static uint32_t lastMS;
	
	//Check to see if device is in powerdown mode
	if (current_state== st_powerdown){
		return;
	}
	
	//  Check to see if device is connected
	if (!isBleConnected()){
		return;
	}
	
	//  Check to see if any measurements need to be sent
	if ((bleSyncTracker.measStackEnd<=bleSyncTracker.measStackSync)||
				(bleSyncTracker.measStackEnd == 0)){
		//  Ensure last stack index never exceed current stack index
		bleSyncTracker.measStackSync=bleSyncTracker.measStackEnd;
		return;
	}
	
	//  Repeated Write Attempt Control
	//  Check to see if enough time has passed before trying again to send
	if (lastMeasAttempted == bleSyncTracker.measStackSync){
		//  Still on the same measurement
		//  Check time
		if ((getCurrentMs()-lastMS)<2000){
			return;
		}
	}
	
	//  Open the file
	fdebug1 = SD_change_directory(folderData);
	fdebug1 = f_open(&bufferFile, FILENAME_RAW_DATA_BUFFER, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	if (fdebug1!=FR_OK){
		//  Something failed, exit function
		//  File won't exist initially
		return;
	}
	
	
	//  Get size of file
	unsigned long fileSize;
	uint32_t measSize;
	fileSize = f_size(&bufferFile);
	measSize = sizeof(tempMeas);
	
	// Quality check, file size must be greater than # of measurements to be sent
	unsigned long expFileSize;
	expFileSize = measSize*bleSyncTracker.measStackEnd;
	if (expFileSize<fileSize){
		//  Bump up measurement stack to match file size
		//  Rounds down in case not an even integer # of measurements
		bleSyncTracker.measStackEnd = fileSize/measSize;
	}else if (expFileSize>fileSize){
		//  Not enough raw binary data, need to reduce stack
		//   Need to exit in case end exceeds sync
		//  Rounds down in case not an even integer # of measurements
		bleSyncTracker.measStackEnd = fileSize/measSize;
		f_close(&bufferFile);
		return;
	}
	
	//  Set to location of last data
	unsigned long fileLoc;
	fileLoc = fileSize-((bleSyncTracker.measStackEnd-bleSyncTracker.measStackSync)*measSize);
	fdebug2 = f_lseek(&bufferFile, fileLoc);
	
	//  Read Measurement
	f_read(&bufferFile, &tempMeas,measSize, &bytesWritten);
	f_close(&bufferFile);
	
	//  Send over Bluetooth
	BLE_sendMeas(&tempMeas);
	
	//  Repeated Write Attempt Control
	lastMeasAttempted = bleSyncTracker.measStackSync;
	lastMS = getCurrentMs();

	
	
}



void initSyncTracker(void){
	FIL tempFile;
	UINT  bytesWritten;
	FRESULT fdebug1, fdebug2, fdebug3;
	DSTATUS diskio_status;
	struct MEASUREMENT tempMeas;
	
	fdebug1 = SD_change_directory(folderData);
	fdebug1 = f_open(&tempFile, FILENAME_RAW_DATA_BUFFER, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	
	if ((fdebug1!=FR_OK) && (fdebug1!=FR_NO_FILE)){
		//  Something failed, exit function
		return;
	}
	
	
	//  Get size of file
	unsigned long fileSize;
	uint32_t measSize;
	fileSize = f_size(&tempFile);
	measSize = sizeof(tempMeas);
	
	// Read Last Measurement
	if (fileSize>=measSize){
		//  Set Current (top) measurement
		fdebug2 = f_lseek(&tempFile, (fileSize-measSize));
		f_read(&tempFile, &tempMeas,measSize, &bytesWritten);
		bleSyncTracker.measStackEnd = fileSize/measSize;
		bleSyncTracker.refIndexSync = tempMeas.refIndex;
		memcpy(&bleSyncTracker.timeEnd, &tempMeas.measTime,sizeof(bleSyncTracker.timeEnd));
		
	}else{
		//  File size too small, less than 1 meas size
		//  Reset everything
		resetSyncTrackerEnd();
		resetSyncTrackerSync();
	}
	
	f_close(&tempFile);
	
	//  Reset last synced
	resetSyncTrackerSync();
	
	
}



void BLE_update_tracker(char *strMeas){
	bleSyncTracker.measStackSync++;
	memcpy(&bleSyncTracker.timeSync, strMeas, 8);
	
}

void BLE_adjLastDate_tracker(char *strDate){
	FIL bufferFile;
	UINT  bytesWritten;
	FRESULT fdebug1, fdebug2, fdebug3;
	uint32_t posixTimeAdj, posixTimeTemp;

	struct TIME timeAdj;
	struct MEASUREMENT tempMeas;
	
	//  Copy over date string into time structure
	memcpy(&timeAdj, strDate, sizeof(timeAdj));
	
	//  Special condition check
	if(timeAdj.year==0000){
		//  Special Condition
		//  Reset everything
		initSyncTracker();
		return;
	}
	
	//  Convert time to Posix time to make comparisons easier
	// Quality check in case empty fields are entered
	time_quality_check(&timeAdj);
	posixTimeAdj = gen_posix_time(&timeAdj);
	
	//  Open file and begin rolling back	
	fdebug1 = SD_change_directory(folderData);
	fdebug1 = f_open(&bufferFile, FILENAME_RAW_DATA_BUFFER, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	if (fdebug1!=FR_OK){
		//  Something failed, exit function
		//  File won't exist initially
		return;
	}
	
	
	//  Get file size
	signed long fileSize;
	uint32_t measSize, nMeas, i;
	measSize = sizeof(tempMeas);
	fileSize = f_size(&bufferFile);
	nMeas = fileSize/measSize;
	
	//  Count back until you find an earlier date
	for (i=0;i<nMeas;i++){
		//  Read back measurement, starting from end
		fdebug1 = f_lseek(&bufferFile, fileSize-((i+1)*measSize));
		fdebug2 = f_read(&bufferFile, &tempMeas,measSize, &bytesWritten);
		//  Convert to posix time for comparison
		posixTimeTemp = gen_posix_time(&tempMeas.measTime);
		
		if (posixTimeTemp<posixTimeAdj){
			break;
		}
		
	}
	//  Done reading file, close
	f_close(&bufferFile);
	
	
	//  Update Sync Tracker
	if(i==nMeas){
		//  Reached end of stack, re-initialize sync counter
		initSyncTracker();
	}else{
		bleSyncTracker.measStackSync = nMeas-i;
		bleSyncTracker.refIndexSync = tempMeas.refIndex;
		memcpy(&bleSyncTracker.timeSync, &tempMeas.measTime, sizeof(timeAdj));
			
	}
	
	
	
	
	
	

	
	
}







void resetSyncTrackerEnd(void){
	//  Reset everything
	bleSyncTracker.measStackEnd = 0;
	bleSyncTracker.refIndexEnd = 0;
	memset(&bleSyncTracker.timeEnd,0,sizeof(bleSyncTracker.timeEnd));
	
	
}

void resetSyncTrackerSync(void){
	//  Reset everything
	bleSyncTracker.measStackSync = 0;
	bleSyncTracker.refIndexSync = 0;
	memset(&bleSyncTracker.timeSync,0,sizeof(bleSyncTracker.timeSync));
	
	
}
