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

//extern enum state_type;
//extern enum input_type;

void configure_timers(enum STATE);



void timer_callback_longbutton(struct tc_module *const);
void timer_callback_laser_timeout(struct tc_module *const);
void timer_callback_1s(struct tc_module *const );
void timer_callback_quick3(struct tc_module *const );
void configure_timer_1s(void);
void configure_timer_laser_timeout(void);
void configure_timer_longbutton(void);
void configure_timer_quick3(void);
void laser_timeout_timer(bool);
void quick3_timer(bool);
void idle_timeout(void);
void wdt_enable(void);
void wdt_disable(void);
struct tc_module timer1;//External button long press timer
struct tc_module timer2; //1 second refresh timer
struct tc_module timer3; //aim timeout for laser




#endif /* TIMERS_H_ */