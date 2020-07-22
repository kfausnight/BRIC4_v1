/*
 * clockSetup.h
 *
 * Created: 6/10/2018 3:35:54 PM
 *  Author: Kris Fausnight
 */ 

#ifndef CLOCKSETUP_H_
#define CLOCKSETUP_H_


#include <asf.h>

#define GCLK_FOR_32khz				GCLK_GENERATOR_2
#define GCLK_FOR_SPI				GCLK_GENERATOR_0
#define GCLK_FOR_USART_LASER		GCLK_GENERATOR_0
#define GCLK_FOR_USART_BLE			GCLK_GENERATOR_0
#define GCLK_FOR_I2C				GCLK_GENERATOR_0
#define GCLK_FOR_TIMER1S			GCLK_GENERATOR_2
#define GCLK_FOR_LASER_TIMEOUT		GCLK_GENERATOR_2
#define GCLK_FOR_LONGBUTTON			GCLK_GENERATOR_2
#define GCLK_FOR_QUICK3				GCLK_GENERATOR_2

//  Note:  External Interrupts Clocked on ULP32K in "conf_extint.h":  #define EXTINT_CLOCK_SELECTION   EXTINT_CLK_ULP32K

//Clock functions
enum clock_type {clock_ext, clock_int, clock_low, clock_high} ;


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



//  Internal clock Functions
void setup_XOSC32k(void);
void clock_32k_source(enum clock_type);
void clock_gclk0_source(enum clock_type);



// External Clock Functions
void get_time(void);
void set_time(void);
//void bcd_adj( uint8_t * );
void ext_osc_onoff(bool);
uint32_t bcd2int(uint8_t);
uint8_t int2bcd(uint8_t);
uint32_t gen_posix_time(struct Time *);
uint8_t incBcdData(uint8_t, int8_t, uint8_t, uint8_t);







#endif /* CLOCKSETUP_H_ */