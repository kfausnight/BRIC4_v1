/*
 * timers.h
 *
 * Created: 9/29/2019 6:27:27 PM
 *  Author: Kris Fausnight
 */ 


#ifndef TIMERS_H_
#define TIMERS_H_

#include <asf.h>
#include <main.h>






//WDT Functions
void wdt_enable(void);
void wdt_disable(void);
//  TC Functions
void timer_callback_longbutton(void);
void timer_callback_laser_timeout(void);
void timer_callback_1s(void );
void configure_timer_1s(void);
void configure_timer_counter(void);
void configure_timer_ExtLong(void);

uint32_t getCurrentMs(void);

void idle_timeout(void);
void laser_timeout(void);

void timerStartExt(void);
void timerStopExt(void);

void powerdown_timer_1s(void);
void powerdown_timer_ExtLong(void);




#endif /* TIMERS_H_ */