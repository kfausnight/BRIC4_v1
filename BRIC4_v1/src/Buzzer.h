/*
 * Buzzer.h
 *
 * Created: 10/4/2020 6:46:41 PM
 *  Author: Kris Fausnight
 */ 


#ifndef BUZZER_H_
#define BUZZER_H_

#include <asf.h>
#include <stdint.h>

enum TONE{
	tone4 = 4698,//6270,//4741,//3951,
	tone3 = 4500, 
	tone2 = 3951,//3986,//3322,
	tone1 = 3135//3164,//2637
	
	//tone4 = 5500,
	//tone3 = 4500, 
	//tone2 = 4000,
	//tone1 = 3500
	};




void buzzOn(enum TONE, uint32_t);




#endif /* BUZZER_H_ */