/*
 * dispFunctions.h
 *
 * Created: 8/22/2020 5:56:37 PM
 *  Author: Kris Fausnight
 */ 


#ifndef DISPFUNCTIONS_H_
#define DISPFUNCTIONS_H_

#include <asf.h>


float getDispX(float [3], uint8_t , uint8_t, bool);
void drawSoftKeys(char *, char *, char *, char *);
void draw2LineSoftKey(char *, char *, uint8_t);

#endif /* DISPFUNCTIONS_H_ */