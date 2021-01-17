/*
 * buttons.h
 *
 * Created: 10/17/2020 5:51:33 PM
 *  Author: Kris Fausnight
 */ 


#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <asf.h>
#include <main.h>
#include <extint.h>

//  Interrupt Management
void config_buttons(void);
//void configure_extint_channel(void);
//void configure_extint_callbacks(void);
void extint_routine(void);
bool externalButtonRoutine(bool);


#endif /* BUTTONS_H_ */