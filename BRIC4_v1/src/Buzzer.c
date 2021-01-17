/*
 * Buzzer.c
 *
 * Created: 10/4/2020 6:45:16 PM
 *  Author: Kris Fausnight
 */ 

#include <Buzzer.h>





void buzzOn(enum TONE tone, uint32_t buzzMs){


	
	
	uint32_t period;// = 220;//us
	//uint32_t period = 150;//us
	uint32_t maxCycles;// = 4000;
	
	//  For some reason, clock runs half expected speed
	period = (1000000)/tone;
	maxCycles = buzzMs*1000/(2*period);
	
	uint32_t cycles;
	for (cycles=0;cycles<maxCycles;cycles++){
		ioport_set_pin_level(BuzzerPin,true);
		delay_cycles_us(period/2);
		ioport_set_pin_level(BuzzerPin,false);
		delay_cycles_us(period/2);
		
	}
	
	
};
