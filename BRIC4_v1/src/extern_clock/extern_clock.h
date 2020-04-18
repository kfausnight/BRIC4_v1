/*
 * extern_clock.h
 *
 * Created: 6/10/2018 3:35:54 PM
 *  Author: Kris Fausnight
 */ 

#ifndef EXTERN_CLOCK_H_
#define EXTERN_CLOCK_H_


#include <asf.h>


struct Time {
	uint8_t	seconds;
	uint8_t	minutes;
	uint8_t	hours;
	uint8_t day; //day of the week
	uint8_t	date; //day of the month
	uint8_t	month;
	uint8_t year; // Last two digits only, i.e. "18" for 2018
	uint8_t control;
	uint8_t control_status;
	float temperatureC;
	float temperatureF;
};
struct Time current_time;
struct Time temp_time;




// Clock/Calendar setup
void get_time(void);
void set_time(void);
void bcd_adj( uint8_t * );
void ext_osc_onoff(bool);
uint32_t bcd2dec(uint8_t);
uint32_t gen_posix_time(struct Time *);







#endif /* EXTERN_CLOCK_H_ */