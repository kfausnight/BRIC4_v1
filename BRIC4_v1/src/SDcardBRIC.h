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

FRESULT SD_add_cal_history(enum CALTYPE);
FRESULT SD_write_report(void);
FRESULT SD_save_raw_data(enum CALTYPE);
FRESULT configure_SD(void);
FRESULT save_measurement(struct MEASUREMENT *meas_inst);
uint32_t getSN(void);



#endif /* SDCARDBRIC_H_ */