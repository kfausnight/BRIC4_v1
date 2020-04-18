/*
 * extern_clock.c
 *
 * Created: 6/10/2018 4:40:50 PM
 *  Author: Kris Fausnight
 */ 
#include <extern_clock\extern_clock.h>
#include <comms\comms.h>

//  Accumulated days per month, needed for POSIX time conversion
//  Days in each month:           [31, 28, 31, 30,  31,  30,  31,  31,  30,  31,  30,  31];
uint32_t days_per_month_acc[12] = {0,  31, 59, 90,  120, 151, 181, 212, 243, 273, 304, 334};



void bcd_adj(uint8_t *x){
	if ((*x & 0x0F)==0x0A){
		*x = *x & 0xF0;
		*x = *x + 0x10;
		} else if ((*x & 0x0F)==0x0F){
		*x = *x & 0xF0;
		*x = *x + 0x09;
	}
}


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
	}else{
		temp_buf[2] = 0x00;
	}
	
	i2c_read_write(writep, rtc_add, temp_buf, 3);
	
	
}



uint32_t gen_posix_time(struct Time *time_inst){
	//https://stackoverflow.com/questions/21975472/how-to-calculate-epoch-day
	uint32_t posix_time;
	uint32_t tm_sec, tm_min, tm_hour, tm_yday, tm_year, tm_month;
	
	tm_sec = bcd2dec(time_inst->seconds);
	tm_min = bcd2dec(time_inst->minutes);
	tm_hour = bcd2dec(time_inst->hours);
	tm_year = bcd2dec(time_inst->year)+100;//  Years since 1900, time_inst->year is years since 2000
	tm_yday = bcd2dec(time_inst->date);
	tm_month = bcd2dec(time_inst->month);
	
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





uint32_t bcd2dec(uint8_t bcd_var){
	uint8_t dec;
	dec = bcd_var & 0xF0;
	dec = dec>>4;
	dec = dec*10;
	dec = dec + (bcd_var & 0x0F);
	
	return dec;
}

