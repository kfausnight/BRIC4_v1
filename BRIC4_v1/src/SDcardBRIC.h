/*
 * SDcardBRIC.h
 *
 * Created: 8/30/2020 10:23:40 AM
 *  Author: Kris Fausnight
 */ 


#ifndef SDCARDBRIC_H_
#define SDCARDBRIC_H_


#include <stdint.h>
#include <calibration.h>

#include <stdint.h>
#include <math.h>
#include <mathBRIC.h>

enum FOLDER_TYPE{	folderData, folderRaw, folderCalibration};


FRESULT SD_add_cal_history(enum CALTYPE);
FRESULT SD_write_report(void);
FRESULT SD_save_raw_data(enum CALTYPE);
FRESULT configure_SD(void);
FRESULT save_measurement(struct MEASUREMENT *);
void getSN(void);
FRESULT SD_change_directory(enum FOLDER_TYPE);

void loadMeasBuffer(void);

//  BLE Synchronization Tracker
void initSyncTracker(void);
void SyncDataBLE(void);
void BLE_update_tracker(char *);
void BLE_adjLastDate_tracker(char *);
void resetSyncTrackerEnd(void);
void resetSyncTrackerSync(void);
void incrementSyncTracker(struct MEASUREMENT *);


#endif /* SDCARDBRIC_H_ */