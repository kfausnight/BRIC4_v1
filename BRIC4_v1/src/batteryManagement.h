/*
 * batteryManagement.h
 *
 * Created: 4/14/2020 6:51:57 PM
 *  Author: Kris Fausnight
 */ 


#ifndef BATTERYMANAGEMENT_H_
#define BATTERYMANAGEMENT_H_


#include <asf.h>


//Battery Fuel Gauge
void setup_batt(void);
void config_batt(void);
uint16_t getBatteryLevel(void);
void BleUpdateBattLevel(void);


//Battery Charger
void setupCharger(void);
void setChargeCurrent(uint32_t);
uint8_t getChargerStatus(void);
uint8_t getChargerRegister(uint8_t);

//  Miscellaneous
void bin2str(uint8_t, char *);


//#include <main.h>





#endif /* BATTERYMANAGEMENT_H_ */