/*
 * comms.c
 *
 * Created: 6/10/2018 4:56:22 PM
 *  Author: Kris Fausnight
 */ 


#include <comms\comms.h>

volatile bool LaserTransmitComplete = false;
volatile bool LaserReceiveComplete = false;
volatile bool BleTransmitComplete = false;
volatile bool BleReceiveComplete = false;
volatile enum LASER_MESSAGE_TYPE laserCurrentMessage;
volatile char bleRcvByte;



//Disable
void disable_comms(void){
	i2c_master_disable(&i2c_master_instance);
	spi_disable(&spi_main);
	usart_disable(&usart_laser);
	usart_disable(&usart_BLE);
}

void enable_comms(void){
	setup_spi();
	configure_i2c_master();
	configure_usart_Laser();
	configure_usart_BLE();
}

//SPI
//***************************************
void spi_setBaud(uint32_t baudRate){
	uint16_t baud = 0;
	
	//  Disable module to make change
	spi_disable(&spi_main);
	
	
	/* Find frequency of the internal SERCOMi_GCLK_ID_CORE */
	uint32_t sercom_index = _sercom_get_sercom_inst_index(spi_main.hw);
	uint32_t gclk_index   = sercom_index + SERCOM0_GCLK_ID_CORE;
	uint32_t internal_clock = system_gclk_chan_get_hz(gclk_index);

	/* Get baud value, based on baudrate and the internal clock frequency */
	enum status_code error_code = _sercom_get_sync_baud_val(
		baudRate,
		internal_clock, &baud);
	spi_main.hw->SPI.BAUD.reg = (uint8_t)baud;
	
	// Re-Enable
	spi_enable(&spi_main);
}


void setup_spi(void){
	//  Maximum Speed LCD:  2.5 MHz
	//uint8_t *reg_ptr;
	
	struct spi_slave_inst_config slave_dev_config;
	
	// Set buffer-overflow status to display immediately so it can be cleared:
	//reg_ptr=0x42001001;
	//*reg_ptr=0x01;
	// Configure and initialize software device instance of peripheral slave
	spi_slave_inst_get_config_defaults(&slave_dev_config);
	slave_dev_config.ss_pin = lcd_SS;
	spi_attach_slave(&slave_lcd, &slave_dev_config);
	slave_dev_config.ss_pin = acc1_SS;
	spi_attach_slave(&slave_acc1, &slave_dev_config);
	slave_dev_config.ss_pin = acc2_SS;
	spi_attach_slave(&slave_acc2, &slave_dev_config);
	slave_dev_config.ss_pin = mag1_SS;
	spi_attach_slave(&slave_mag1, &slave_dev_config);
	slave_dev_config.ss_pin = mag2_SS;
	spi_attach_slave(&slave_mag2, &slave_dev_config);
	slave_dev_config.ss_pin = SD_CS;
	spi_attach_slave(&slave_SD, &slave_dev_config);
	// Configure, initialize and enable SERCOM SPI module
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.generator_source = GCLK_FOR_SPI;
	//config_spi_master.transfer_mode = SPI_TRANSFER_MODE_3;//initialize with LCD mode
	config_spi_master.transfer_mode = SPI_TRANSFER_MODE_0;//initialize with LCD mode
	config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_E;
	config_spi_master.pinmux_pad0 = PINMUX_PB08D_SERCOM4_PAD0;
	config_spi_master.pinmux_pad2 = PINMUX_PB10D_SERCOM4_PAD2;
	config_spi_master.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;
	config_spi_master.character_size = SPI_CHARACTER_SIZE_8BIT ;
	config_spi_master.mode_specific.master.baudrate = baudRateMin;
	spi_init(&spi_main, SERCOM4, &config_spi_master);
	spi_enable(&spi_main);

	
}




//USART
//******************************************

void configure_usart_Laser(void){
	struct usart_config config_usart;
	enum status_code usart_status;
	
	//  Laser UART setup SERCOM1
	usart_get_config_defaults(&config_usart);
	config_usart.generator_source = GCLK_FOR_USART_LASER;
	config_usart.baudrate    = 9600;
	config_usart.mux_setting = USART_RX_1_TX_0_XCK_1;
	config_usart.pinmux_pad0 = PINMUX_PA16C_SERCOM1_PAD0;
	config_usart.pinmux_pad1 = PINMUX_PA17C_SERCOM1_PAD1;
	config_usart.pinmux_pad2 = PINMUX_UNUSED;
	config_usart.pinmux_pad3 = PINMUX_UNUSED;
	do {
		usart_status = usart_init(&usart_laser,	SERCOM1, &config_usart) ;
	}while((usart_status != STATUS_OK) && (usart_status != STATUS_ERR_DENIED) );
	usart_enable(&usart_laser);
	
	
	//  Setup Callbacks
	usart_register_callback(&usart_laser,writeLaserCallback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_laser,readLaserCallback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_laser, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_laser, USART_CALLBACK_BUFFER_RECEIVED);
	
	//  Laser, Initiate background read to buffer
	rxBufferLaserIndex = 0;
	usart_read_job(&usart_laser, rxBufferLaser); // 
}

void configure_usart_BLE(void){
	struct usart_config config_usart;
	enum status_code usart_status;
	
	
	// BLE UART setup SERCOM0
	usart_get_config_defaults(&config_usart);
	config_usart.generator_source = GCLK_FOR_USART_BLE;
	config_usart.baudrate    = 115200;
	config_usart.mux_setting = USART_RX_1_TX_0_RTS_2_CTS_3;
	config_usart.pinmux_pad0 = PINMUX_PA08C_SERCOM0_PAD0;
	config_usart.pinmux_pad1 = PINMUX_PA09C_SERCOM0_PAD1;
	config_usart.pinmux_pad2 = PINMUX_PA10C_SERCOM0_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PA11C_SERCOM0_PAD3;
	do {
		usart_status = usart_init(&usart_BLE,	SERCOM0, &config_usart) ;
	}while((usart_status != STATUS_OK) && (usart_status != STATUS_ERR_DENIED) );
	usart_enable(&usart_BLE);
	
	//  Setup Callbacks
	usart_register_callback(&usart_BLE,writeBleCallback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_BLE,readBleCallback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_BLE, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_BLE, USART_CALLBACK_BUFFER_RECEIVED);
	
	//  Initiate background read to buffer
	rxBufferBleClear();
	usart_read_job(&usart_BLE, &bleRcvByte);
}

void BLE_usart_isolate(void){
	//  Isolate MCU from BLE over USART
	//  Allows an external interface to communicate with BLE
	usart_disable(&usart_BLE);
	ioport_set_pin_dir(MCU_RTS1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(MCU_RTS1, false);
	ioport_set_pin_dir(MCU_CTS1, IOPORT_DIR_INPUT);
	ioport_reset_pin_mode(MCU_TX1);
	ioport_reset_pin_mode(MCU_RX1);
	ioport_set_pin_dir(MCU_TX1, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(MCU_RX1, IOPORT_DIR_INPUT);
	
}



void readLaserCallback(struct usart_module *const usart_module)
{

	if(rxBufferLaser[rxBufferLaserIndex]==0xA8){
		//  End of message key received (per laser comm protocol)
		LaserReceiveComplete=true;		
		rxBufferLaserIndex = 0;
		laserCurrentMessage = laserMessageType();
		
	}else if(rxBufferLaser[rxBufferLaserIndex]==0xAA){
		//  Start of message key received
		LaserReceiveComplete=false;
		rxBufferLaser[0]=0xAA;  //  Ensure message starts at beginning of buffer
		rxBufferLaserIndex = 1;	// Next byte to be placed at 1	
		laserCurrentMessage = NONE;
	}else{
		//  Not Start or end of message, continue filling buffer
		rxBufferLaserIndex++;
		if (rxBufferLaserIndex>=sizeof(rxBufferLaser)){
			rxBufferLaserIndex = 0;
		}
		
	}
	//  Prepare to take another byte
	usart_read_job(&usart_laser, &rxBufferLaser[rxBufferLaserIndex]);

}

void writeLaserCallback(struct usart_module *const usart_module)
{
	LaserTransmitComplete = true;
}


void readBleCallback(struct usart_module *const usart_module)
{	
	//DEBUG///////////////////////////////////////////////////
	debugBuffPtr = &debugBuff[0];
	debugBuff[debugBuffIndex] = bleRcvByte;
	debugBuffIndex++;
	
	if (debugBuffIndex>=sizeof(debugBuff)){debugBuffIndex = 0;}
	//DEBUG///////////////////////////////////////////////////
	
	if((bleRcvByte==NL)||(bleRcvByte==CR)||(bleRcvByte==00)){ 
		// "\n" or "CR" or null character received
		//  End of message key received (per BLE comm protocol)
		//  Ignore line return if it's the first character
		if(rxBufferBleIndex!=0){
			//  Replace line return with string Null
			BleReceiveComplete=true;		
			rxBufferBle[rxBufferBleIndex]=0x00;  //  Put in null character at end	
		}
		
	}else{
		// Regular character received
		if (BleReceiveComplete){
			rxBufferLaserClear();	
		}
		rxBufferBle[rxBufferBleIndex] = bleRcvByte;
		rxBufferBleIndex++;		
		
		//  Catch to prevent buffer overflow
		if (rxBufferBleIndex>=sizeof(rxBufferBle)){
			rxBufferBleIndex = 0;
		}
		
	}
	//  Prepare to take another byte
	usart_read_job(&usart_BLE, &bleRcvByte);

}

void writeBleCallback(struct usart_module *const usart_module)
{
	BleTransmitComplete = true;
}

enum status_code writeLaser(uint8_t *tx_data, uint16_t length){
	enum status_code writeStatus;
	//clear_rx_buffer();
	LaserTransmitComplete=false;
	writeStatus = usart_write_buffer_job(&usart_laser, tx_data, length);
	return writeStatus;
}

enum status_code writeBle(uint8_t *tx_data, uint16_t length){
	enum status_code writeStatus;
	BleTransmitComplete=false;
	writeStatus = usart_write_buffer_job(&usart_BLE, tx_data, length);
	return writeStatus;
}	
	

bool isLaserTransmitComplete(void){
	return LaserTransmitComplete;
}
bool isLaserReceiveComplete(void){
	return LaserReceiveComplete;
}

bool isBleTransmitComplete(void){
	return BleTransmitComplete;
}
bool isBleReceiveComplete(void){
	return BleReceiveComplete;
}

void rxBufferLaserClear(void){
	//uint8_t i;
	//for (i=0;i<sizeof(rxBufferLaser);i++){
	//	rxBufferLaser[i] = 0;
	//}
	laserCurrentMessage = NONE;
	LaserReceiveComplete=false;
	rxBufferLaserIndex = 0;
}

void rxBufferBleClear(void){
	uint32_t i;
	for (i=0;i<UART_BUFFER_LENGTH;i++){
		rxBufferBle[i] = 0;
	}
	
	rxBufferBleIndex = 0;
	BleReceiveComplete = false;
}

//  Determine the type of message currently in the buffer
enum LASER_MESSAGE_TYPE laserMessageType(void){
	uint8_t i;
	enum LASER_MESSAGE_TYPE messType = 0;
	for(i=0;i<sizeof(rxBufferLaser);i++){
		if(rxBufferLaser[i]==0xAA){
			messType =  rxBufferLaser[i+2];
			break;
		}
		
	}
	return messType;
}


bool isBleCommEnabled(void){
	return (usart_BLE.hw->USART.CTRLA.reg & SERCOM_USART_CTRLA_ENABLE);
}


//I2C
//************************************************
void configure_i2c_master(void)
{
	/* Initialize config structure and software module. */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	/* Change buffer timeout to something longer. */
	config_i2c_master.generator_source = GCLK_FOR_I2C;
	config_i2c_master.buffer_timeout = 10000;
	config_i2c_master.pinmux_pad0    = PINMUX_PA12C_SERCOM2_PAD0;
	config_i2c_master.pinmux_pad1    = PINMUX_PA13C_SERCOM2_PAD1;
	i2c_master_init(&i2c_master_instance, SERCOM2, &config_i2c_master);
	i2c_master_enable(&i2c_master_instance);
}

void i2c_read_write(enum read_write mode, uint8_t device, uint8_t *buf, uint8_t length){
	//first character in buffer is read/write register address
	uint16_t limit=100;
	uint16_t timeout;
	struct i2c_master_packet packet = {
		.address     = device,
		.data        = buf,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};
	
	if (mode==readp){
		timeout=0;
		packet.data_length=1;
		while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) !=STATUS_OK) {
			if (timeout++ == limit) {   break;   }
		}
		timeout=0;
		packet.data=buf+1;
		packet.data_length=length;
		while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) !=STATUS_OK) {
			if (timeout++ == limit) {   break;   }
		}
		
	} else{
		timeout=0;
		packet.data_length=length;
		while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) !=STATUS_OK) {
			if (timeout++ == limit) {   break;   }
		}
	}
	
}


void max17055_reg_read_write(enum read_write mode, uint8_t address, uint16_t *data){
	//MAX17055 is the battery fuel gauge
	//MAX17055 is 16-bit register read/writes with LSB first
	uint16_t temp16;
	uint8_t temp_buf[3];
	
	temp_buf[0] = address;
	if (mode==readp){
		i2c_read_write(readp, batt_add, temp_buf, 2);
		temp16=temp_buf[2];
		temp16=temp16<<8;
		temp16=temp16+temp_buf[1];
		*data=temp16;
	}else{
		temp16=*data;
		temp_buf[1]=temp16 & 0x00FF;
		temp16=temp16>>8;
		temp_buf[2]=temp16 & 0x00FF;
		i2c_read_write(writep, batt_add, temp_buf, 2);
	}
}

void adp5062_reg_read_write(enum read_write mode, uint8_t address, uint8_t *data){
	// ADP5062 is the charger chip
	// Write and read 8-bit registers
	uint8_t temp_buf[2];
	
	temp_buf[0] = address;
	if (mode==readp){
		i2c_read_write(readp, charger_add, temp_buf, 1);
		*data=temp_buf[1];
	}else{
		
		temp_buf[1]=data;
		i2c_read_write(writep, charger_add, temp_buf, 2);
	}
}

