/*
 * clockSetup.h
 *
 * Created: 6/10/2018 3:35:54 PM
 *  Author: Kris Fausnight
 */ 

#ifndef CLOCKSETUP_H_
#define CLOCKSETUP_H_


#include <main.h>



//Clock functions
enum clock_type {clock_ext, clock_int, clock_low, clock_high} ;


//  Internal clock Functions
//void setup_XOSC32k(void);
void clock_32k_source(enum clock_type);
void mainClockPowerup(void);
void mainClockPowerdown(void);
void DFLL_Enable(void);


// External Clock Functions
void get_time(void);
void set_time(void);
void ext_osc_onoff(bool);
uint32_t bcd2int(uint8_t);
uint8_t int2bcd(uint8_t);
uint32_t gen_posix_time(struct TIME *);
uint8_t incBcdData(uint8_t, int8_t, uint8_t, uint8_t);

//#include <main.h>




#endif /* CLOCKSETUP_H_ */