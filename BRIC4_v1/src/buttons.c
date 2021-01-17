/*
 * buttons.c
 *
 * Created: 10/17/2020 5:50:30 PM
 *  Author: Kris Fausnight
 */ 

#include <buttons.h>




void config_buttons(void){
	struct extint_chan_conf config_extint_chan;

	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_LOW;
	config_extint_chan.filter_input_signal  = true;
	//config_extint_chan.enable_async_edge_detection = true;
	// button 4
	config_extint_chan.gpio_pin           = PIN_PA07A_EIC_EXTINT7;
	config_extint_chan.gpio_pin_mux       = MUX_PA07A_EIC_EXTINT7;
	extint_chan_set_config(7, &config_extint_chan);
	extint_register_callback(extint_routine, 7,	EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(7,EXTINT_CALLBACK_TYPE_DETECT);
	// button 3
	config_extint_chan.gpio_pin           = PIN_PA06A_EIC_EXTINT6;
	config_extint_chan.gpio_pin_mux       = MUX_PA06A_EIC_EXTINT6;
	extint_chan_set_config(6, &config_extint_chan);
	extint_register_callback(extint_routine, 6,	EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(6,EXTINT_CALLBACK_TYPE_DETECT);
	// button 2
	config_extint_chan.gpio_pin           = PIN_PA04A_EIC_EXTINT4;
	config_extint_chan.gpio_pin_mux       = MUX_PA04A_EIC_EXTINT4;
	extint_chan_set_config(4, &config_extint_chan);
	extint_register_callback(extint_routine, 4,	EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(4,EXTINT_CALLBACK_TYPE_DETECT);
	// button 1
	config_extint_chan.gpio_pin           = PIN_PB09A_EIC_EXTINT9;
	config_extint_chan.gpio_pin_mux       = MUX_PB09A_EIC_EXTINT9;
	extint_chan_set_config(9, &config_extint_chan);
	extint_register_callback(extint_routine, 9,	EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(9,EXTINT_CALLBACK_TYPE_DETECT);
	
	// button Ext
	//config_extint_chan.detection_criteria = EXTINT_DETECT_BOTH;
	config_extint_chan.gpio_pin           = PIN_PA05A_EIC_EXTINT5;
	config_extint_chan.gpio_pin_mux       = MUX_PA05A_EIC_EXTINT5;
	extint_chan_set_config(5, &config_extint_chan);
	extint_register_callback(extint_routine, 5,	EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(5,EXTINT_CALLBACK_TYPE_DETECT);
	
	
	
	
	
	
	
}

bool extIntToggle(const uint8_t channel){
	/* Get a pointer to the module hardware instance */
	#define CONFIG_SENSE_MASK 0x00000007
	//const uint32_t EIC_ENABLE_MASK =  0x00000002;
	Eic *const EIC_module = _extint_get_eic_from_channel(channel);
	bool pinLevel;
	uint32_t config_pos;
	uint32_t currentConfig, newConfig;
	enum extint_detect currentDetection, newDetection;
	
	//  Disable for modification
	EIC_module->CTRLA.reg &= ~EIC_CTRLA_ENABLE;
	while (extint_is_syncing()) {
		/* Wait for all hardware modules to complete synchronization */
	}
	
	extint_chan_clear_detected(channel);
	//  Start position of this channel's config
	config_pos = (4 * (channel % 8));
	//  Current configuration and detection
	currentConfig = EIC_module->CONFIG[channel / 8].reg;

	currentDetection = (currentConfig>>config_pos) & CONFIG_SENSE_MASK;

	//  Find current condition
	if(currentDetection==EXTINT_DETECT_LOW){
		newDetection = EXTINT_DETECT_HIGH;
		pinLevel = false;
	}else{
		newDetection = EXTINT_DETECT_LOW;
		pinLevel = true;
	}
	
	//  Set new configuration
	newConfig = currentConfig & (~(CONFIG_SENSE_MASK<<config_pos));
	newConfig = newConfig | (newDetection<<config_pos);
	
	EIC_module->CONFIG[channel / 8].reg = newConfig;

	//  Re-Enable
	EIC_module->CTRLA.reg |= EIC_CTRLA_ENABLE;
	while (extint_is_syncing()) {
		/* Wait for all hardware modules to complete synchronization */
	}
	
	extint_chan_clear_detected(channel);
	
	return pinLevel;
}

bool button_Debounce(void){
	static uint32_t last_ms = 0;
	uint32_t current_ms;
	
	current_ms = getCurrentMs();
	if((current_ms-last_ms)>DEBOUNCE_MS){
		last_ms = current_ms;
		return true;
	}else{
		return false;
	}
	
}

void extint_routine(void)
{
	uint8_t current_channel;
	
	current_channel = extint_get_current_channel();
	
	//return;
	//tempInput = input_none;
	switch (current_channel){
		case 5:
			if(externalButtonRoutine(!extIntToggle(current_channel))){
				if(button_Debounce()){
					current_input = input_buttonE;
				}
			}
			break;
		case 7:
			if(!extIntToggle(current_channel)){
				if(button_Debounce()){
					current_input = input_button4;
				}
			}
			break;
		case 6:
			if(!extIntToggle(current_channel)){
				if(button_Debounce()){
					current_input = input_button3;
				}
			}
			break;
		case 4:
			if(!extIntToggle(current_channel)){
				if(button_Debounce()){
					current_input = input_button2;
				}
			}
			break;
		case 9:
			if(!extIntToggle(current_channel)){
				if(button_Debounce()){
					current_input = input_button1;
				}
			}
			break;
		default:
			
			break;
	}// End switch for each input type
	
	extint_chan_clear_detected(current_channel);
	//cpu_irq_enable();
	
	
}


bool externalButtonRoutine(bool buttonPressed){
	// Button External
	// Special Routines for External Button
	// If held down for less than X seconds, provides normal input upon release
	// If held down for more than X seconds, a separate interrupt routine provides powerdown input
	// When in powerdown state, 3 quick clicks provides powerup input
	static uint32_t last_time_ms;
	uint32_t current_time_ms;
	
	static uint8_t click_counter=0;
	current_time_ms = getCurrentMs();
	
	switch (current_state){
		case st_powerup:
			//  Ignore external button inputs during powerup
			return false;
			break;
		case st_powerdown:
			//  Special Routine to wake up from sleep
			if (buttonPressed){
				//  Button Pressed
				
				
				//  Find out if click counter will be incremented
				if( (current_time_ms-last_time_ms)<QUICK3_MS){
					//  Clicked within necessary time interval, add to click counter.
					click_counter++;
				}else{
					//  Not fast enough, record as first click
					click_counter = 1;
				}
				last_time_ms = current_time_ms;
				
				
				//  See if enough clicks have occurred
				if (click_counter>=3){
					click_counter = 0;
					return true;	
				}else{
					return false;
				}

			}
			break;
		default:
			if (buttonPressed){
				//  Button Pressed
				
				//  Trigger on if button is pressed
				if(!buttonE_triggered){
					buttonE_triggered=true;
						//trigger timer
					timerStartExt();
				}
				return false;
			}else{
				//  Released in a short amount of time, normal input
				buttonE_triggered=false;
				timerStopExt();
				return true;
			}

	}//  End Switch Statement

	return false;
	
}



