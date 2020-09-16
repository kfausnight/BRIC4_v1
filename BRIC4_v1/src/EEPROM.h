/*
 * EEPROM.h
 *
 * Created: 2/2/2019 1:52:10 PM
 *  Author: Kris Fausnight
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

#include <main.h>


void EEPROM_read(uint16_t , uint8_t *, uint32_t);

void EEPROM_write(uint16_t , uint8_t *, uint32_t);

void load_user_settings(void);

void save_user_settings(void);

void load_calibration(void);

void save_calibration(void);

void EEPROM_saveCalRawData(enum CALTYPE);

void EEPROM_loadCalRawData(enum CALTYPE);

void save_cal_report(void);

void load_cal_report(void);

void EEPROM_test(void);



#endif /* EEPROM_H_ */