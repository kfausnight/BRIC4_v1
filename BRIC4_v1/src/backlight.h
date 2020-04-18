/*
 * backlight.h
 *
 * Created: 4/11/2020 3:54:27 PM
 *  Author: Kris Fausnight
 */ 


#ifndef BACKLIGHT_H_
#define BACKLIGHT_H_


#include <main.h>
#include <asf.h>
#include <comms/comms.h>

struct BACKLIGHTCOLOR
{
	char *colorStringPtr;
	uint8_t red;
	uint8_t blue;
	uint8_t green;	
};



void backlightOff(void);
void backlightOn(void);
void backlight_level(struct BACKLIGHT_SETTING *);
char* backlightGetCurrentColor(void);
void backlightColorToggle(void);
void backlightMinus(void);
void backlightPlus(void);
void backlightLevelToggle(void);
struct BACKLIGHTCOLOR* backlightCustomAdjust(char , int8_t );

#endif /* BACKLIGHT_H_ */