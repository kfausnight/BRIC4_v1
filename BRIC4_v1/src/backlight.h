/*
 * backlight.h
 *
 * Created: 4/11/2020 3:54:27 PM
 *  Author: Kris Fausnight
 */ 


#ifndef BACKLIGHT_H_
#define BACKLIGHT_H_



#include <asf.h>
#include <main.h>

#define COLOR_MAX	30  //  maximum color index
#define BRIGHT_MAX	5   //  maximum brightness index



void backlightOff(void);
void backlightOn(struct BACKLIGHT_SETTING *);
void backlight_level(struct BACKLIGHT_SETTING *);
char* backlightGetCurrentColor(struct BACKLIGHT_SETTING *);
void backlightColorToggle(struct BACKLIGHT_SETTING *);
void backlightMinus(struct BACKLIGHT_SETTING *);
void backlightPlus(struct BACKLIGHT_SETTING *);
void backlightLevelToggle(struct BACKLIGHT_SETTING *);
struct BACKLIGHT_COLOR * backlightCustomAdjust(char , int8_t );


#include <main.h>

#endif /* BACKLIGHT_H_ */