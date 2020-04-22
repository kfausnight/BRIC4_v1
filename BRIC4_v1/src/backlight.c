/*
 * backlight.c
 *
 * Created: 4/11/2020 3:55:09 PM
 *  Author: Kris Fausnight
 */ 


#include <backlight.h>


extern struct OPTIONS options;


char colorStrWhite[]	= "White ";
char colorStrRed[]	= "Red   ";
char colorStrBlue[]	= "Blue  ";
char colorStrGreen[]	= "Green ";
char colorStrPurple[] = "Purple";
char colorStrCyan[]	= "Cyan  ";
char colorStrCustom[] = "custom";

struct BACKLIGHTCOLOR colorCustom	= {.colorStringPtr = colorStrCustom,	.red = 15, .green = 15, .blue = 15};
struct BACKLIGHTCOLOR colorWhite	= {.colorStringPtr = colorStrWhite,		.red = 22, .green = 30, .blue = 30};
struct BACKLIGHTCOLOR colorRed		= {.colorStringPtr = colorStrRed,		.red = 30, .green =  0, .blue = 0};
struct BACKLIGHTCOLOR colorBlue		= {.colorStringPtr = colorStrBlue,		.red = 0,  .green =  8, .blue = 30};
struct BACKLIGHTCOLOR colorGreen	= {.colorStringPtr = colorStrGreen,		.red = 0,  .green = 30, .blue = 0};
struct BACKLIGHTCOLOR colorPurple	= {.colorStringPtr = colorStrPurple,	.red = 30, .green =  4, .blue = 30};
struct BACKLIGHTCOLOR colorCyan		= {.colorStringPtr = colorStrCyan,		.red = 0,  .green = 30, .blue = 24};


struct BACKLIGHTCOLOR *colorOptions[] = {&colorCustom, &colorWhite, &colorRed, &colorBlue, &colorGreen, &colorPurple, &colorCyan, };

//Backlight Settings
#define LED_MAX 225 //93%; limited by driver current limit per datasheet



struct BACKLIGHTCOLOR * backlightCustomAdjust(char refColor, int8_t adjustment){
	uint8_t * colorPtr;
	uint8_t maxRef;
	
	switch (refColor){
		case 'r':
			colorPtr = &colorCustom.red;
			maxRef = options.backlight_setting.maxColor;
			break;
		case 'b':
			colorPtr = &colorCustom.blue;
			maxRef = options.backlight_setting.maxColor;
			break;
		case 'g':
			colorPtr = &colorCustom.green;
			maxRef = options.backlight_setting.maxColor;
			break;	
		case 'L':
			colorPtr = &options.backlight_setting.brightness;
			maxRef = options.backlight_setting.maxBrightness;
			break;
		default:
			colorPtr = &options.backlight_setting.brightness;
			maxRef = options.backlight_setting.maxBrightness;
	}
	
	if (adjustment>0){
		if (*colorPtr<maxRef){
			*colorPtr = *colorPtr+adjustment;
		}
	}else if (adjustment<0){
		if (*colorPtr>0){
			*colorPtr = *colorPtr+adjustment;
		}
	}
	
	backlightOn();
	
	return &colorCustom;
	
}


void backlightColorToggle(void){
	options.backlight_setting.colorRef++;
	if (options.backlight_setting.colorRef>=(sizeof(colorOptions)/sizeof(&colorCustom))){
		options.backlight_setting.colorRef = 1;
	}
	backlightOn();
	
}

void backlightPlus(void){
	if (options.backlight_setting.brightness<options.backlight_setting.maxBrightness){
		options.backlight_setting.brightness = options.backlight_setting.brightness+1;
	}	
	backlightOn();
}

void backlightMinus(void){	
	if (options.backlight_setting.brightness>0){
		options.backlight_setting.brightness = options.backlight_setting.brightness-1;
	}
	backlightOn();
	
}

void backlightLevelToggle(void){
	options.backlight_setting.brightness = options.backlight_setting.brightness+1;
	
	if (options.backlight_setting.brightness>options.backlight_setting.maxBrightness){
		options.backlight_setting.brightness = 0;
	}
	backlightOn();
	
}

char* backlightGetCurrentColor(void){
	return colorOptions[options.backlight_setting.colorRef]->colorStringPtr;	
}

void backlightOn(void){
	backlight_level(&options.backlight_setting);	
}


void backlight_level(struct BACKLIGHT_SETTING *blset){
	uint8_t u8blue, u8green, u8red;
	float fred, fblue, fgreen;
	float scale;
	
	fred = colorOptions[blset->colorRef]->red;
	fgreen = colorOptions[blset->colorRef]->green;
	fblue = colorOptions[blset->colorRef]->blue;
	
	
	
	scale = 1/(fred+fgreen+fblue);
	scale = scale*blset->brightness/blset->maxBrightness;
	fred = scale*fred;
	fblue = scale*fblue;
	fgreen = scale*fgreen;
	
	u8red = fred*LED_MAX;
	u8blue = fblue*LED_MAX;
	u8green = fgreen*LED_MAX;
	
	// Precaution to not put too much current through PWM driver
	if (u8red>LED_MAX){u8red = LED_MAX;}
	if (u8blue>LED_MAX){u8blue = LED_MAX;}
	if (u8blue>LED_MAX){u8blue = LED_MAX;}
	
	uint16_t mes_len = 10;
	uint8_t temp_buf[mes_len];
	temp_buf[0]= 0x80;  //control register 0b10000000
	temp_buf[1]= 0x80; //Mode1: 0b10000000
	temp_buf[2]= 0x08; //Mode2: 0b00001000
	temp_buf[3]= u8blue; //PWM0 - blue
	temp_buf[4]= u8green; //PWM1 - green
	temp_buf[5]= u8red; //PWM2 - red
	temp_buf[6]= 0x00; //PWM3
	temp_buf[7]= 0x00; //GRPPWM
	temp_buf[8]= 0x00; //FRPFREQ
	temp_buf[9]= 0x2A; //LEDOUT: 0b00101010
	
	i2c_read_write(writep, led_add,temp_buf, mes_len);
	

}


void backlightOff(void){
		
	uint16_t mes_len = 10;
	uint8_t temp_buf[mes_len];
	temp_buf[0]= 0x80;  //control register 0b10000000
	temp_buf[1]= 0x80; //Mode1: 0b10000000
	temp_buf[2]= 0x08; //Mode2: 0b00001000
	temp_buf[3]= 0x00; //PWM0 - blue
	temp_buf[4]= 0x00; //PWM1 - green
	temp_buf[5]= 0x00; //PWM2 - red
	temp_buf[6]= 0x00; //PWM3
	temp_buf[7]= 0x00; //GRPPWM
	temp_buf[8]= 0x00; //FRPFREQ
	temp_buf[9]= 0x2A; //LEDOUT: 0b00101010
	
	i2c_read_write(writep, led_add,temp_buf, mes_len);
	
}
