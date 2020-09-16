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
void BLE_sendMeas(struct MEASUREMENT *);
void BLE_handleMessage(void);
void ble_error(void);


#endif /* BLE_FUNC_H_ */