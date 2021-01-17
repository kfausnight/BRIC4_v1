/*
 * timers.c
 *
 * Created: 9/29/2019 6:25:35 PM
 *  Author: Kris Fausnight
 */ 
#include <timers.h>

//  RTC structure
//struct rtc_module rtc_instance;
//  Counter/Timer Structures
struct tc_module timerExtLong;//External button long press timer
struct tc_module timer1Sec; //1 second refresh timer
struct tc_module timerCounter; // timer for ms background counter



volatile uint32_t accumulatorMs;
volatile uint32_t tick1sMs;

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
//**************** RTC  ******************************************
//  RTC will work intermittently on only some units
//  Abandoned RTC due to intermittent issues after days of futile debugging
/*
void configure_rtc_count(void)
{
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);
	config_rtc_count.prescaler           = RTC_COUNT_PRESCALER_DIV_32;
	config_rtc_count.mode                = RTC_COUNT_MODE_32BIT;//RTC_COUNT_MODE_16BIT;
	#ifdef FEATURE_RTC_CONTINUOUSLY_UPDATED
	config_rtc_count.continuously_update = true;
	#endif
	
	if (rtc_instance.hw->MODE0.CTRLA.reg & RTC_MODE0_CTRLA_ENABLE){
		rtc_count_disable(&rtc_instance);
	}
	
	//if (timer1Sec.hw->COUNT8.CTRLA.reg & TC_CTRLA_ENABLE){
	////	tc_disable(&timer1Sec);
	//}
	
	// Set period 
	//rtc_count_set_period(&rtc_instance, 1000);
	
	rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
	//  Disable all interrupts
	rtc_instance.hw->MODE0.INTENSET.reg = 0x0000;
	rtc_instance.hw->MODE0.INTENCLR.reg = 0x0000;
	rtc_count_enable(&rtc_instance);
	
	rtc_count_set_count(&rtc_instance, 0);
	
	// rtc_count_register_callback(
	 //&rtc_instance, rtc_overflow_callback, RTC_COUNT_CALLBACK_OVERFLOW);
	// rtc_count_enable_callback(&rtc_instance, RTC_COUNT_CALLBACK_OVERFLOW);
	 
	 
	 
}
void rtc_overflow_callback(void)
{
	// Do something on RTC overflow here 
	//port_pin_toggle_output_level(LED_0_PIN);
}

*/

//****************  Timers ****************************************

void idle_timeout(void){
	static uint32_t idle_seconds = 0;
	
	if ((current_input == input_1sec) && (!isCharging)&&(!SD_WriteLockout)){
		idle_seconds++;
	}else{
		idle_seconds = 0;
	}
	
	if (idle_seconds>IDLE_MAX_S){
		current_input = input_pwrDown;
		idle_seconds = 0;
	}
	
	
}

void laser_timeout(void){
	static uint8_t laserSecondsOn = 0;
	
	if (current_state==st_powerdown){
		return;
	}
	
	if (isLaserOn() && (current_input == input_1sec))
	{
		laserSecondsOn++;
		if (laserSecondsOn>LASER_TIMEOUT_S){
			rangefinder_on_off(false);
			last_input = input_laser_timeout;
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
	tc_register_callback(&timer1Sec,timer_callback_1s,TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&timer1Sec, TC_CALLBACK_OVERFLOW);

}



void configure_timer_counter(void){
	//  Timer setup for background timer
	struct tc_config config_tc;

	//Timer for laser timeout
	tc_get_config_defaults(&config_tc);
	//config_tc.counter_size    = TC_COUNTER_SIZE_8BIT;
	config_tc.counter_size    =		TC_COUNTER_SIZE_16BIT;
	config_tc.clock_prescaler =     TC_CLOCK_PRESCALER_DIV16 ;//  TC_CLOCK_PRESCALER_DIV1024;
	config_tc.clock_source =		GCLK_FOR_TIMERS;
	config_tc.count_direction =		TC_COUNT_DIRECTION_UP;
	config_tc.reload_action =		TC_RELOAD_ACTION_GCLK;
	config_tc.run_in_standby = true; //  Run during sleep mode
	config_tc.on_demand = true;
	
	if (timerCounter.hw->COUNT8.CTRLA.reg & TC_CTRLA_ENABLE){
		tc_disable(&timerCounter);
	}
	
	tc_init(&timerCounter, TC4, &config_tc);
	tc_enable(&timerCounter);
	tc_set_count_value(&timerCounter, 0);	
	tc_start_counter(&timerCounter);
	
	accumulatorMs = 0;  //  reset ms accumulator
	
	//  Configure Callbacks
	tc_register_callback(
	&timerCounter,
	timer_callback_backgroundCounter,
	TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&timerCounter, TC_CALLBACK_OVERFLOW);

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
	tick1sMs = getCurrentMs();
	
	current_input=input_1sec;

}

void timer_callback_backgroundCounter(void){
	
	accumulatorMs = accumulatorMs+32000;
	
}


void timer_callback_longbutton(void)
{
	
	tc_stop_counter(&timerExtLong);
	tc_set_count_value(&timerExtLong, 0);
	if(!ioport_get_pin_level(buttonE)){
		//  Double-check that button is still pressed
		current_input=input_pwrDown;
	}
	
}


uint8_t getCentiSeconds(void){
	uint32_t deltaMS;
	uint8_t centiSec;
	
	deltaMS = getCurrentMs();
	deltaMS = deltaMS-tick1sMs;
	deltaMS = deltaMS/10;
	centiSec = deltaMS;
	
	return centiSec;
	
	
}


uint32_t getCurrentMs(void){
	//  32khz clock, 16  prescaler.
	//  (32,768 clock/sec) * (1 count/16 clock) * (1 sec / 1000 ms) = 2.048 count/ms
	//  Divide by 2 to return ms, equivalent to >>1.
	
	uint32_t currentMs;
	currentMs = tc_get_count_value(&timerCounter)>>1;
	currentMs = currentMs+accumulatorMs;
	
	return currentMs;

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


