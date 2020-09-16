/*
 * timers.c
 *
 * Created: 9/29/2019 6:25:35 PM
 *  Author: Kris Fausnight
 */ 
#include <timers.h>


//  Counter/Timer Structures
struct tc_module timerExtLong;//External button long press timer
struct tc_module timer1Sec; //1 second refresh timer
struct tc_module timerCounter; //aim timeout for laser



// *************  Watchdog Timer WDT***********************************

void wdt_enable(void){
	struct wdt_conf wdt_configuration;
	wdt_get_config_defaults(&wdt_configuration);
	// Uses internal 1khz clock
	wdt_configuration.timeout_period = WDT_PERIOD_16384CLK;
	wdt_set_config(&wdt_configuration);
	
	
}

void wdt_disable(void){
	struct wdt_conf wdt_configuration;
	wdt_get_config_defaults(&wdt_configuration);
	//wdt_configuration.clock_source = GCLK_GENERATOR_2;// Uses internal 1khz clock
	wdt_configuration.enable = false;
	wdt_set_config(&wdt_configuration);
	
	
}


//****************  Timers ****************************************

void idle_timeout(void){
	static uint32_t idle_seconds = 0;
	
	if ((current_input == input_1sec) && (!isCharging)){
		idle_seconds++;
	}else{
		idle_seconds = 0;
	}
	
	if (idle_seconds>IDLE_MAX_S){
		current_input = input_powerdown;
		idle_seconds = 0;
	}
	
	
	
	
}

void laser_timeout(void){
	static uint8_t laserSecondsOn = 0;
	
	if (isLaserOn() && (current_input == input_1sec))
	{
		laserSecondsOn++;
		if (laserSecondsOn>LASER_TIMEOUT_S){
			rangefinder_on_off(false);
			current_input = input_laser_timeout;
		}
		
	}else{
		laserSecondsOn = 0;
	}
}




void configure_timer_1s(void){
	//  Timer setup for 1 second refresh
	struct tc_config config_tc;
	
	//  Configure Timer
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size    = TC_COUNTER_SIZE_8BIT;
	config_tc.clock_prescaler =     TC_CLOCK_PRESCALER_DIV1024 ;//  TC_CLOCK_PRESCALER_DIV1024;
	config_tc.clock_source = GCLK_FOR_TIMERS;
	config_tc.count_direction =  TC_COUNT_DIRECTION_UP;// TC_COUNT_DIRECTION_DOWN;
	config_tc.reload_action =  TC_RELOAD_ACTION_PRESC;
	config_tc.counter_8_bit.period = 0x1F;//0x1F = 31d, 1 second exactly on counter
	
	if (timer1Sec.hw->COUNT8.CTRLA.reg & TC_CTRLA_ENABLE){
		tc_disable(&timer1Sec);
	}
	
	tc_init(&timer1Sec, TC1, &config_tc);
	tc_enable(&timer1Sec);
	//timer2_on_off = true;
	tc_start_counter(&timer1Sec);

	//  Configure Callbacks
	//1 second refresh
	tc_register_callback(
	&timer1Sec,
	timer_callback_1s,
	TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&timer1Sec, TC_CALLBACK_OVERFLOW);

}


void configure_timer_counter(void){
	//  Timer setup for laser time-out
	struct tc_config config_tc;

	//Timer for laser timeout
	tc_get_config_defaults(&config_tc);
	//config_tc.counter_size    = TC_COUNTER_SIZE_8BIT;
	config_tc.counter_size    =		TC_COUNTER_SIZE_16BIT;
	config_tc.clock_prescaler =     TC_CLOCK_PRESCALER_DIV1024 ;//  TC_CLOCK_PRESCALER_DIV1024;
	config_tc.clock_source =		GCLK_FOR_TIMERS;
	config_tc.count_direction =		TC_COUNT_DIRECTION_UP;
	config_tc.reload_action =		TC_RELOAD_ACTION_GCLK;
	//config_tc.counter_16_bit.compare_capture_channel = 5;
	//config_tc.counter_8_bit.period = 100;
	
	if (timerCounter.hw->COUNT8.CTRLA.reg & TC_CTRLA_ENABLE){
		tc_disable(&timerCounter);
	}
	
	tc_init(&timerCounter, TC4, &config_tc);
	tc_enable(&timerCounter);
	tc_set_count_value(&timerCounter, 0);	
	tc_start_counter(&timerCounter);
	

}


void configure_timer_ExtLong(void){
	//  Timer Setup for external button long-press
	struct tc_config config_tc;
	uint32_t temp;

	//Timer for long button press
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size    = TC_COUNTER_SIZE_8BIT;
	config_tc.clock_prescaler =    TC_CLOCK_PRESCALER_DIV1024;
	config_tc.clock_source = GCLK_FOR_TIMERS;
	config_tc.count_direction = TC_COUNT_DIRECTION_UP;
	config_tc.reload_action = TC_RELOAD_ACTION_PRESC;
	temp = OFF_HOLD_MS>>5; //  divide by 32; X ms * (1/1000 sec/ms)*(32 counts/sec); 
	config_tc.counter_8_bit.period = temp;
	
	if (timerExtLong.hw->COUNT8.CTRLA.reg & TC_CTRLA_ENABLE){
		tc_disable(&timerExtLong);
	}
	
	tc_init(&timerExtLong, TC0, &config_tc);
	tc_enable(&timerExtLong);
	//timer1_on_off = true;
	tc_stop_counter(&timerExtLong);//enable starts timer, stop immediately
	tc_set_count_value(&timerExtLong, 0);

	// Configure callback
	//Long button press
	tc_register_callback(
		&timerExtLong,
		timer_callback_longbutton,
		TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&timerExtLong, TC_CALLBACK_OVERFLOW);



}

//


void timer_callback_1s(void)
{

	
	current_input=input_1sec;


}




void timer_callback_longbutton(void)
{
	
	tc_stop_counter(&timerExtLong);
	tc_set_count_value(&timerExtLong, 0);
	if(!ioport_get_pin_level(buttonE)){
		//  Double-check that button is still pressed
		current_input=input_powerdown;
	}
	
}





uint32_t getCurrentMs(void){
	//  32khz clock, 1024  prescaler.
	//  (32khz clock/sec) * (1 count/1024 clock) * (1 sec / 1000 ms) = .032 count/ms
	//  Multiply by 32 to return ms, equivalent to <<5.
	return tc_get_count_value(&timerCounter)<<5; 
}

void timerStartExt(void){
	tc_set_count_value(&timerExtLong, 0);
	tc_start_counter(&timerExtLong);
}
void timerStopExt(void){
	tc_stop_counter(&timerExtLong);
}


void powerdown_timer_1s(void){
	tc_disable(&timer1Sec);
}

void powerdown_timer_ExtLong(void){
	tc_disable(&timerExtLong);
	
	
}


