/*
 * clockSetup.c
 *
 * Created: 6/10/2018 4:40:50 PM
 *  Author: Kris Fausnight
 */ 
#include <clockSetup.h>
#include <clock.h>
#include <conf_clocks.h>
#include <system.h>

#define dfllctrEnable	0x0002

//  Accumulated days per month, needed for POSIX time conversion
//  Days in each month:           [31, 28, 31, 30,  31,  30,  31,  31,  30,  31,  30,  31];
uint32_t days_per_month_acc[12] = {0,  31, 59, 90,  120, 151, 181, 212, 243, 273, 304, 334};


bool isExtClockEnabled = false;


void clock_32k_source(enum clock_type ext_int){
	struct system_gclk_gen_config gclock_gen_conf;
	
	system_gclk_gen_get_config_defaults(&gclock_gen_conf);
	gclock_gen_conf.run_in_standby = true;
	if(ext_int == clock_ext){
		gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_XOSC32K;
	}else{
		gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_ULP32K;
	}
	system_gclk_gen_set_config(GCLK_FOR_32khz, &gclock_gen_conf);
	system_gclk_gen_enable(GCLK_FOR_32khz);
	
}


void mainClockPowerdown(void){
	struct system_gclk_gen_config gclk_conf; 
	
	//  Set GCLK 0 to use ULP32k
	system_gclk_gen_get_config_defaults(&gclk_conf);                  \
	gclk_conf.source_clock    = SYSTEM_CLOCK_SOURCE_ULP32K;   \
	gclk_conf.division_factor = 1;      \
	gclk_conf.run_in_standby  = true; \
	gclk_conf.output_enable   = false;  \
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);       \
	system_gclk_gen_enable(GCLK_GENERATOR_0);   
	
	//  Turn off DFLL	
	OSCCTRL->DFLLCTRL.reg = OSCCTRL->DFLLCTRL.reg & (!dfllctrEnable);
	
	//  Turn off 16MHZ oscillator
	//  Should already be turned off
	OSCCTRL->OSC16MCTRL.reg = OSCCTRL->OSC16MCTRL.reg &(!0x02);
	
	//  Set performance level
	system_switch_performance_level(SYSTEM_PERFORMANCE_LEVEL_0);
	
	//  Re-initialize delay with new main clock
	delay_init();
}


void mainClockPowerup(void){
	  
	
	// If DFLL is disabled, enable it
	if (!(OSCCTRL->DFLLCTRL.reg & dfllctrEnable)){
		DFLL_Enable();
		
	}
	
	//  Set GCLK 0 to DFLL
	struct system_gclk_gen_config gclk_conf;   
	system_gclk_gen_get_config_defaults(&gclk_conf);                  \
	gclk_conf.source_clock    = CONF_CLOCK_GCLK_0_CLOCK_SOURCE;   \
	gclk_conf.division_factor = CONF_CLOCK_GCLK_0_PRESCALER;      \
	gclk_conf.run_in_standby  = CONF_CLOCK_GCLK_0_RUN_IN_STANDBY; \
	gclk_conf.output_enable   = CONF_CLOCK_GCLK_0_OUTPUT_ENABLE;  \
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);       \
	system_gclk_gen_enable(GCLK_GENERATOR_0);                       \
	
	//  Re-initialize delay with new main clock	
	delay_init();
	
	
}


void DFLL_Enable(void){
	
	//if ((OSCCTRL->DFLLCTRL.bit.ENABLE==1)){
		system_switch_performance_level(SYSTEM_PERFORMANCE_LEVEL_2);
	
		struct system_clock_source_dfll_config dfll_conf;
		system_clock_source_dfll_get_config_defaults(&dfll_conf);
	
		dfll_conf.loop_mode      = CONF_CLOCK_DFLL_LOOP_MODE;
		dfll_conf.on_demand      = false;
		dfll_conf.run_in_stanby  = CONF_CLOCK_DFLL_RUN_IN_STANDBY;

		// Using DFLL48M COARSE CAL value from NVM Software Calibration Area Mapping 
		// in DFLL.COARSE helps to output a frequency close to 48 MHz.
	#define NVM_DFLL_COARSE_POS    26 // DFLL48M Coarse calibration value bit position.
	#define NVM_DFLL_COARSE_SIZE   6  // DFLL48M Coarse calibration value bit size.

		uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP5)
				+ (NVM_DFLL_COARSE_POS / 32))
				>> (NVM_DFLL_COARSE_POS % 32))
				& ((1 << NVM_DFLL_COARSE_SIZE) - 1);
		// In some revision chip, the Calibration value is not correct 
		if (coarse == 0x3f) {
			coarse = 0x1f;
		}

		dfll_conf.coarse_value = coarse;

		if (CONF_CLOCK_DFLL_LOOP_MODE == SYSTEM_CLOCK_DFLL_LOOP_MODE_OPEN) {
			dfll_conf.fine_value   = CONF_CLOCK_DFLL_FINE_VALUE;
		}

	#  if CONF_CLOCK_DFLL_QUICK_LOCK == true
		dfll_conf.quick_lock = SYSTEM_CLOCK_DFLL_QUICK_LOCK_ENABLE;
	#  else
		dfll_conf.quick_lock = SYSTEM_CLOCK_DFLL_QUICK_LOCK_DISABLE;
	#  endif

	#  if CONF_CLOCK_DFLL_TRACK_AFTER_FINE_LOCK == true
		dfll_conf.stable_tracking = SYSTEM_CLOCK_DFLL_STABLE_TRACKING_TRACK_AFTER_LOCK;
#  else
		dfll_conf.stable_tracking = SYSTEM_CLOCK_DFLL_STABLE_TRACKING_FIX_AFTER_LOCK;
	#  endif

	#  if CONF_CLOCK_DFLL_KEEP_LOCK_ON_WAKEUP == true
		dfll_conf.wakeup_lock = SYSTEM_CLOCK_DFLL_WAKEUP_LOCK_KEEP;
	#  else
		dfll_conf.wakeup_lock = SYSTEM_CLOCK_DFLL_WAKEUP_LOCK_LOSE;
	#  endif

	#  if CONF_CLOCK_DFLL_ENABLE_CHILL_CYCLE == true
		dfll_conf.chill_cycle = SYSTEM_CLOCK_DFLL_CHILL_CYCLE_ENABLE;
	#  else
		dfll_conf.chill_cycle = SYSTEM_CLOCK_DFLL_CHILL_CYCLE_DISABLE;
	#  endif
		
		dfll_conf.multiply_factor = CONF_CLOCK_DFLL_MULTIPLY_FACTOR;

		dfll_conf.coarse_max_step = CONF_CLOCK_DFLL_MAX_COARSE_STEP_SIZE;
		dfll_conf.fine_max_step   = CONF_CLOCK_DFLL_MAX_FINE_STEP_SIZE;


		system_clock_source_dfll_set_config(&dfll_conf);
	
	
		//  Set up clock channel for closed loop
		struct system_gclk_chan_config dfll_gclk_chan_conf;
		system_gclk_chan_get_config_defaults(&dfll_gclk_chan_conf);
		dfll_gclk_chan_conf.source_generator = CONF_CLOCK_DFLL_SOURCE_GCLK_GENERATOR;
		system_gclk_chan_set_config(OSCCTRL_GCLK_ID_DFLL48, &dfll_gclk_chan_conf);
		system_gclk_chan_enable(OSCCTRL_GCLK_ID_DFLL48);
	
		//  Turn on closed loop mode	
		system_clock_source_enable(SYSTEM_CLOCK_SOURCE_DFLL);
		while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_DFLL));
		if (CONF_CLOCK_DFLL_ON_DEMAND) {
			OSCCTRL->DFLLCTRL.bit.ONDEMAND = 1;
		}
	
}




//  External Clock Functions *************************************************************

void get_time(void){
	uint8_t temp_buf[20];
	int16_t temp_var16;
	temp_buf[0]=0x00;
	i2c_read_write(readp, rtc_add, temp_buf, 19);
	//Parse Data
	current_time.seconds=	bcd2int(temp_buf[0x01]);
	current_time.minutes=	bcd2int(temp_buf[0x02]);
	current_time.hours=		bcd2int(temp_buf[0x03]);
	current_time.day=		bcd2int(temp_buf[0x05]);
	current_time.month=		bcd2int(temp_buf[0x06] & 0x7F);//mask out first "century" bit
	current_time.year=		bcd2int(temp_buf[0x07])+2000;
	
	current_time.centiseconds = getCentiSeconds();
	
	//  Also get the temperature from the RTC
	temp_var16=temp_buf[0x12];
	temp_var16=temp_var16<<8;
	temp_var16=temp_var16+temp_buf[0x13];
	
	currentTempC=temp_var16;
	currentTempC=currentTempC/256;
	
}

void set_time(struct TIME *temp_time){
	uint8_t temp_buf[20];

	temp_buf[0]=0x00;

	//Parse Data
	temp_buf[0x01]= int2bcd(temp_time->seconds);
	temp_buf[0x02]= int2bcd(temp_time->minutes);
	temp_buf[0x03]= int2bcd(temp_time->hours);
	temp_buf[0x04]= 0x00;
	temp_buf[0x05]= int2bcd(temp_time->day);
	temp_buf[0x06]= int2bcd(temp_time->month);
	temp_buf[0x07]= int2bcd(temp_time->year-2000);
	i2c_read_write(writep, rtc_add, temp_buf, 8);
	
}


void ext_osc_onoff(bool onoff){
	
	///////////////////////////////////////////////////////////////////////////
	//  Switch over 32k source to internal clock if turning off external oscillator
	if (!onoff){
		clock_32k_source(clock_int);
	}	
	
	
	///////////////////////////////////////////////////////////////////
	//  Turn on external oscillator (RTC chip)
	uint8_t temp_buf[3];

	temp_buf[0] = 0x0E;
	//Register 0x0Eh 
	//0b00000100 = 0x04
	temp_buf[1] = 0x04;
	//Register 0x0Fh
	//0b00001000 = 0x08; 32kHz osc on
	//0b00000000 = 0x00; 32kHz osc off
	if (onoff){
		temp_buf[2] = 0x08;
		isExtClockEnabled = true;
	}else{
		temp_buf[2] = 0x00;
		isExtClockEnabled = false;
	}	
	i2c_read_write(writep, rtc_add, temp_buf, 3);
	delay_ms(10);	
	
	/////////////////////////////////////////////////////////
	//  Configure external oscillator clock source and turn it on
	if (onoff){
		struct system_clock_source_xosc32k_config xosc32k_conf;
		system_clock_source_xosc32k_get_config_defaults(&xosc32k_conf);

		xosc32k_conf.frequency           = 32768UL;
		xosc32k_conf.external_clock      = SYSTEM_CLOCK_EXTERNAL_CLOCK;//CONF_CLOCK_XOSC32K_EXTERNAL_CRYSTAL;
		xosc32k_conf.startup_time        = SYSTEM_XOSC32K_STARTUP_2048;//CONF_CLOCK_XOSC32K_STARTUP_TIME;
		xosc32k_conf.enable_1khz_output  = true;//CONF_CLOCK_XOSC32K_ENABLE_1KHZ_OUPUT;
		xosc32k_conf.enable_32khz_output = true;//CONF_CLOCK_XOSC32K_ENABLE_32KHZ_OUTPUT;
		xosc32k_conf.on_demand           = false;
		xosc32k_conf.run_in_standby      = true;//CONF_CLOCK_XOSC32K_RUN_IN_STANDBY;

		system_clock_source_xosc32k_set_config(&xosc32k_conf);
		system_clock_source_enable(SYSTEM_CLOCK_SOURCE_XOSC32K);
		while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_XOSC32K));
		OSC32KCTRL->XOSC32K.bit.ONDEMAND = 1;
		
	}else{
		system_clock_source_disable(SYSTEM_CLOCK_SOURCE_XOSC32K);
	}
	
	
	///////////////////////////////////////////////////////////
	//  Setup GCLK for external clock source
	if (onoff){
		clock_32k_source(clock_ext);
	}
	

}



bool time_quality_check(struct TIME *time_inst){
	bool isValid = true;
	
	if(time_inst->year<1900){
		time_inst->year = 1900;
		isValid = false;
	}
	
	if(time_inst->month<1){
		time_inst->month = 1;
		isValid = false;
	}else if(time_inst->month>12){
		time_inst->month = 12;
		isValid = false;
	}
	
	if(time_inst->day<1){
		time_inst->day = 1;
		isValid = false;
	}else if(time_inst->day>31){
		time_inst->day = 31;
		isValid = false;
	}
	
	if(time_inst->hours>24){
		time_inst->hours = 24;
		isValid = false;
	}
	
	return isValid;
	
}

uint32_t gen_posix_time(struct TIME *time_inst){
	//https://stackoverflow.com/questions/21975472/how-to-calculate-epoch-day
	uint32_t posix_time;
	uint32_t tm_sec, tm_min, tm_hour, tm_yday, tm_year, tm_month;
	
	tm_sec = time_inst->seconds;
	tm_min = time_inst->minutes;
	tm_hour = time_inst->hours;
	tm_year = time_inst->year-1900;//  Years since 1900 per algorithm
	tm_yday = time_inst->day;
	tm_month = time_inst->month;
	/*
	tm_sec = bcd2int(time_inst->seconds);
	tm_min = bcd2int(time_inst->minutes);
	tm_hour = bcd2int(time_inst->hours);
	tm_year = bcd2int(time_inst->year)-1900;//  Years since 1900 per algorithm
	tm_yday = bcd2int(time_inst->date);
	tm_month = bcd2int(time_inst->month);
	*/
	
	//  Find days since year roll-over
	// On "days_per_month_acc" index, subtract 1 to account for 0-base index.
	// Subtract 1 from "tm_yday" to account for referencing.  i.e. January 3, 12 PM is 2.5 days into year
	// Subtract 1 to account for 0-base index of month_index
	tm_yday = tm_yday + days_per_month_acc[(tm_month-1)] -1;
	
	
	posix_time = tm_sec+tm_min*60+tm_hour*3600+tm_yday*86400 + (tm_year-70)*31536000;
	//  Add adjustments for leap-years
	posix_time += ((tm_year-69)/4)*86400 -((tm_year-1)/100)*86400 + ((tm_year+299)/400)*86400;
	
	return posix_time;
	
	
	
	
}




uint8_t int2bcd(uint8_t binaryInput){
	uint8_t bcdResult = 0;
	uint8_t shift = 0;
	while (binaryInput > 0) {
		bcdResult |= (binaryInput % 10) << (shift++ << 2);
		binaryInput /= 10;
	}
	return bcdResult;
}


uint32_t bcd2int(uint8_t bcd_var){
	uint8_t dec;
	dec = bcd_var & 0xF0;
	dec = dec>>4;
	dec = dec*10;
	dec = dec + (bcd_var & 0x0F);
	
	return dec;
}

