/*
 * timers.c
 *
 * Created: 9/29/2019 6:25:35 PM
 *  Author: Kris Fausnight
 */ 
#include <timers.h>
#include <main.h>

extern volatile enum INPUT current_input, last_input;
extern volatile uint8_t click_counter;
extern volatile uint8_t debug1;

struct wdt_conf wdt_configuration;

bool timer1_on_off = false;
bool timer2_on_off = false;
bool timer3_on_off = false;


#define quick_click_time 16// 32k, div1024 prescaler;  32 counts/second
#define laser_timeout  900// 32k, div1024 prescaler;  32 counts/second
#define off_hold_time 96  // 32k, div1024 prescaler;  32 counts/second
#define idle_seconds_max 60 // seconds



void wdt_enable(void){
	wdt_get_config_defaults(&wdt_configuration);
	// Uses internal 1khz clock
	wdt_configuration.timeout_period = WDT_PERIOD_16384CLK;
	wdt_set_config(&wdt_configuration);
	
	
}

void wdt_disable(void){
	wdt_get_config_defaults(&wdt_configuration);
	//wdt_configuration.clock_source = GCLK_GENERATOR_2;// Uses internal 1khz clock
	wdt_configuration.enable = false;
	wdt_set_config(&wdt_configuration);
	
	
}


void idle_timeout(void){
	static uint32_t idle_seconds = 0;
	
	if (current_input == input_1sec){
		idle_seconds++;
	}else{
		idle_seconds = 0;
	}
	
	if (idle_seconds>idle_seconds_max){
		current_input = input_powerdown;
		idle_seconds = 0;
	}
	
	
	
	
};

void configure_timers(enum STATE timer_state){
	
	switch (timer_state)
	{
		case st_powerup:
			//********Powerup****************************************
			//  configure timers
			configure_timer_1s();
			configure_timer_laser_timeout();
			configure_timer_longbutton();			
			break;
		case st_powerdown:
			//********Powerdown****************************
			//  disable all timers
			tc_disable(&timer1);
			timer1_on_off = false;
			tc_disable(&timer2);
			timer2_on_off = false;
			tc_disable(&timer3);
			timer3_on_off = false;
			//  enable timer to detect 3 quick external button presses
			configure_timer_quick3();
		

			break;
		default:
			break;
	}


}

void configure_timer_1s(void){
	//  Timer setup for 1 second refresh
	struct tc_config config_tc;
	
	//  Configure Timer
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size    = TC_COUNTER_SIZE_8BIT;
	config_tc.clock_prescaler =     TC_CLOCK_PRESCALER_DIV1024 ;//  TC_CLOCK_PRESCALER_DIV1024;
	config_tc.clock_source = GCLK_GENERATOR_2;
	config_tc.count_direction =  TC_COUNT_DIRECTION_UP;// TC_COUNT_DIRECTION_DOWN;
	config_tc.reload_action =  TC_RELOAD_ACTION_PRESC;
	config_tc.counter_8_bit.period = 0x1F;//0x1F = 31d, 1 second exactly on counter
	if(timer2_on_off){
		tc_disable(&timer2);
		timer2_on_off = false;
	}
	tc_init(&timer2, TC1, &config_tc);
	tc_enable(&timer2);
	timer2_on_off = true;
	tc_start_counter(&timer2);

	//  Configure Callbacks
	//1 second refresh
	tc_register_callback(
	&timer2,
	timer_callback_1s,
	TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&timer2, TC_CALLBACK_OVERFLOW);

}


void configure_timer_laser_timeout(void){
	//  Timer setup for laser time-out
	struct tc_config config_tc;

	//Timer for laser timeout
	tc_get_config_defaults(&config_tc);
	//config_tc.counter_size    = TC_COUNTER_SIZE_8BIT;
	config_tc.counter_size    = TC_COUNTER_SIZE_16BIT;
	config_tc.clock_prescaler =     TC_CLOCK_PRESCALER_DIV1024 ;//  TC_CLOCK_PRESCALER_DIV1024;
	config_tc.clock_source = GCLK_GENERATOR_2;
	config_tc.count_direction =  TC_COUNT_DIRECTION_DOWN;
	config_tc.reload_action =  TC_RELOAD_ACTION_PRESC;
	//config_tc.counter_16_bit.compare_capture_channel = 5;
	//config_tc.counter_8_bit.period = 100;
	if(timer3_on_off){
		tc_disable(&timer3);
		timer3_on_off = false;
	}
	tc_init(&timer3, TC4, &config_tc);
	tc_enable(&timer3);
	timer3_on_off = true;
	tc_stop_counter(&timer3);
	tc_set_count_value(&timer3, laser_timeout);

	//  Configure callbacks
	//laser Timeout
	tc_register_callback(
	&timer3,
	timer_callback_laser_timeout,
	TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&timer3, TC_CALLBACK_OVERFLOW);

}


void configure_timer_longbutton(void){
	//  Timer Setup for external button long-press
	struct tc_config config_tc;

	//Timer for long button press
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size    = TC_COUNTER_SIZE_8BIT;
	config_tc.clock_prescaler =    TC_CLOCK_PRESCALER_DIV1024;//  TC_CLOCK_PRESCALER_DIV1024;
	config_tc.clock_source = GCLK_GENERATOR_2;
	config_tc.count_direction = TC_COUNT_DIRECTION_UP;
	config_tc.reload_action = TC_RELOAD_ACTION_PRESC;
	//config_tc.counter_8_bit.value=0;
	config_tc.counter_8_bit.period = off_hold_time;
	if(timer1_on_off){
		tc_disable(&timer1);
		timer1_on_off = false;
	}
	tc_init(&timer1, TC0, &config_tc);
	tc_enable(&timer1);
	timer1_on_off = true;
	tc_stop_counter(&timer1);//enable starts timer, stop immediately
	tc_set_count_value(&timer1, 0);

	// Configure callback
	//Long button press
	tc_register_callback(
	&timer1,
	timer_callback_longbutton,
	TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&timer1, TC_CALLBACK_OVERFLOW);



}

//
void configure_timer_quick3(void)
{
	//  Timer setup for external button 3-quick presses to be used during power down
	//  Re-uses 1 second refresh timer
	struct tc_config config_tc;

	//  Configure Timer
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size    = TC_COUNTER_SIZE_8BIT;
	config_tc.clock_prescaler =     TC_CLOCK_PRESCALER_DIV1024 ;//  TC_CLOCK_PRESCALER_DIV1024;
	config_tc.clock_source = GCLK_GENERATOR_2;
	config_tc.count_direction =  TC_COUNT_DIRECTION_UP;// TC_COUNT_DIRECTION_DOWN;
	config_tc.reload_action =  TC_RELOAD_ACTION_PRESC;
	config_tc.counter_8_bit.period = quick_click_time;
	if(timer2_on_off){
		tc_disable(&timer2);
		timer2_on_off = false;
	}
	tc_init(&timer2, TC1, &config_tc);
	tc_enable(&timer2);
	timer2_on_off = true;
	tc_stop_counter(&timer2);
	tc_set_count_value(&timer2, 0);

	//  Configure Callbacks
	//1 second refresh
	tc_register_callback(
	&timer2,
	timer_callback_quick3,
	TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&timer2, TC_CALLBACK_OVERFLOW);

	click_counter = 0;

}


void timer_callback_1s(struct tc_module *const module_inst)
{

	
	current_input=input_1sec;
	//debug1++;

}

void timer_callback_laser_timeout(struct tc_module *const module_inst)
{

	tc_stop_counter(&timer3);
	tc_set_count_value(&timer3, laser_timeout);
	rangefinder_on_off(false);
	current_input = input_laser_timeout;
}




void timer_callback_longbutton(struct tc_module *const module_inst)
{
	current_input=input_powerdown;
	tc_stop_counter(&timer1);
	tc_set_count_value(&timer1, 0);
}



void timer_callback_quick3(struct tc_module *const module_inst)
{
	tc_stop_counter(&timer2);
	tc_set_count_value(&timer2, 0);
	click_counter = 0;

}



//*******************************
void laser_timeout_timer(bool on_off)
{
	if (on_off){
		tc_set_count_value(&timer3, laser_timeout);
		tc_start_counter(&timer3);
		
	}else{
		tc_stop_counter(&timer3);
		tc_set_count_value(&timer3, laser_timeout);
	
	}


}


void quick3_timer(bool on_off)
{
	if (on_off){
		tc_set_count_value(&timer2, 0);
		tc_start_counter(&timer2);
		
	}else{
		tc_stop_counter(&timer2);
		tc_set_count_value(&timer2, 0);
		
	}

}