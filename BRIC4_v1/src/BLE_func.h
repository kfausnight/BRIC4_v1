/*
 * BLE_func.h
 *
 * Created: 9/6/2020 1:59:48 PM
 *  Author: Kris Fausnight
 */ 


#ifndef BLE_FUNC_H_
#define BLE_FUNC_H_

#include <asf.h>

void BLE_init(void);

void BLE_handleMessage(void);
void BLE_remoteCommand(char *);
void BLE_reset_to_AT_mode(void);
void BLE_error(void);
void BLE_get_device_name(void);
void BLE_MAC_format(char*, char *);
void BLE_get_device_MAC(void);
void BLE_get_client_MAC(void);
bool isBleConnected(void);
void BLE_advert_OnOff(bool);

//  Measurement Service
void BLE_sendMeas(struct MEASUREMENT *);

//  Battery Service
uint8_t BLE_getBatteryLevel(void);
void BLE_setBatteryLevel(uint8_t );

#endif /* BLE_FUNC_H_ */