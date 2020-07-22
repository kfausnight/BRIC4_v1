/*
 * clockSetup.c
 *
 * Created: 6/10/2018 4:40:50 PM
 *  Author: Kris Fausnight
 */ 
#include <clockSetup.h>
#include <comms\comms.h>

//  Accumulated days per month, needed for POSIX time conversion
//  Days in each month:           [31, 28, 31, 30,  31,  30,  31,  31,  30,  31,  30,  31];
uint32_t days_per_month_acc[12] = {0,  31, 59, 90,  120, 151, 181, 212, 243, 273, 304, 334};


bool isExtClockEnabled = false;


void setup_XOSC32k(void){
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
	//system_clock_source_disable(SYSTEM_CLOCK_SOURCE_OSC32K);

}


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


void clock_gclk0_source(enum clock_type high_low){
	struct system_gclk_gen_config gclock_gen_conf;
	
	system_gclk_gen_get_config_defaults(&gclock_gen_conf);
	gclock_gen_conf.run_in_standby = true;
	if(high_low == clock_high){
		gclock_gen_conf.source_clock =SYSTEM_CLOCK_SOURCE_OSC16M;
		}else{
		gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_ULP32K;
	}
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclock_gen_conf);
	system_gclk_gen_enable(GCLK_GENERATOR_0);
	
}










//  External Clock Functions *************************************************************






/*
void bcd_adj(uint8_t *x){
	if ((*x & 0x0F)==0x0A){
		*x = *x & 0xF0;
		*x = *x + 0x10;
	} else if ((*x & 0x0F)==0x0F){
		*x = *x & 0xF0;
		*x = *x + 0x09;
	}
}
*/


void get_time(void){
	uint8_t temp_buf[20];
	int16_t temp_var16;
	temp_buf[0]=0x00;
	i2c_read_write(readp, rtc_add, temp_buf, 19);
	//Parse Data
	current_time.seconds=	temp_buf[0x01];
	current_time.minutes=	temp_buf[0x02];
	current_time.hours=		temp_buf[0x03];
	current_time.date=		temp_buf[0x05] ;
	current_time.month=		temp_buf[0x06] & 0x7F;//mask out first "century" bit
	current_time.year=		temp_buf[0x07];
	current_time.control=	temp_buf[0x0F];
	current_time.control_status=	temp_buf[0x10];
	temp_var16=temp_buf[0x12];
	temp_var16=temp_var16<<8;
	temp_var16=temp_var16+temp_buf[0x13];
	current_time.temperatureC=temp_var16;
	current_time.temperatureC=current_time.temperatureC/256;
	current_time.temperatureF=current_time.temperatureC*1.8+32;
	
}

void set_time(void){
	uint8_t temp_buf[20];

	temp_buf[0]=0x00;

	//Parse Data
	temp_buf[0x01]=temp_time.seconds;
	temp_buf[0x02]=temp_time.minutes;
	temp_buf[0x03]=temp_time.hours;
	temp_buf[0x04]=temp_time.day;
	temp_buf[0x05]=temp_time.date;
	temp_buf[0x06]=temp_time.month;
	temp_buf[0x07]=temp_time.year;
	i2c_read_write(writep, rtc_add, temp_buf, 8);
	
}


void ext_osc_onoff(bool onoff){
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
	
	//****************debug
	temp_buf[0]=0x0E;
	i2c_read_write(readp, rtc_add, temp_buf, 3);
	//Parse Data

	//*******************debug
}



uint32_t gen_posix_time(struct Time *time_inst){
	//https://stackoverflow.com/questions/21975472/how-to-calculate-epoch-day
	uint32_t posix_time;
	uint32_t tm_sec, tm_min, tm_hour, tm_yday, tm_year, tm_month;
	
	tm_sec = bcd2int(time_inst->seconds);
	tm_min = bcd2int(time_inst->minutes);
	tm_hour = bcd2int(time_inst->hours);
	tm_year = bcd2int(time_inst->year)+100;//  Years since 1900, time_inst->year is years since 2000
	tm_yday = bcd2int(time_inst->date);
	tm_month = bcd2int(time_inst->month);
	
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

uint8_t incBcdData(uint8_t bcdData, int8_t increment, uint8_t dataMin, uint8_t dataMax){
	
	uint8_t intData;
	intData = bcd2int(bcdData);
	
	if (increment<0){
		if (intData>dataMin){
			intData = intData-1;
		}else{
			intData = dataMax;
		}
	}else{
		if (intData<dataMax){
			intData = intData+1;
		}else{
			intData = dataMin;
		}
	}
	return int2bcd(intData);
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

