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
#include <sensors.h>
#include <clockSetup.h>
#include <clock.h>


//  RTC Functions
void setup_rtc(void);
void rtc_overflow_callback(void);
void configure_rtc_callbacks(void);
//WDT Functions
void wdt_enable(void);
void wdt_disable(void);
//  TC Functions
void configure_timers(enum STATE);
void timer_callback_longbutton(struct tc_module *const);
void timer_callback_laser_timeout(struct tc_module *const);
void timer_callback_1s(struct tc_module *const );
void configure_timer_1s(void);
void configure_timer_laser_timeout(void);
void configure_timer_longbutton(void);
void laser_timeout_timer(bool);
void idle_timeout(void);


struct tc_module timer1;//External button long press timer
struct tc_module timer2; //1 second refresh timer
struct tc_module timer3; //aim timeout for laser
struct rtc_module rtc1;



#endif /* TIMERS_H_ */