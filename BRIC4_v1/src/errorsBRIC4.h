/*
 * errorsBRIC4.h
 *
 * Created: 10/11/2020 4:51:18 PM
 *  Author: Kris Fausnight
 */ 


#ifndef ERRORSBRIC4_H_
#define ERRORSBRIC4_H_

#include <asf.h>


void error_check(struct MEASUREMENT_FULL *);
void gen_err_message(char *, struct MEASUREMENT *, uint8_t);
bool increment_error_count(struct MEASUREMENT_FULL *, enum MEAS_ERROR_TYPE , float, float);
void adjustErrorSensitivity(void);

#endif /* ERRORSBRIC4_H_ */