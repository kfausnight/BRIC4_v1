/*
 * comms.c
 *
 * Created: 6/10/2018 4:56:22 PM
 *  Author: Kris Fausnight
 */ 


#include <comms\comms.h>

extern bool USART_BLE_enabled;
volatile bool LaserTransmitComplete;
volatile bool LaserReceiveComplete;
volatile enum LASER_MESSAGE_TYPE laserCurrentMessage;


//Disable
void disable_comms(void){
	i2c_master_disable(&i2c_master_instance);
	spi_disable(&spi_main);
	usart_disable(&usart_laser);
	usart_disable(&usart_BLE);
	USART_BLE_enabled = false;
}



//SPI
//***************************************

void setup_spi(void){
	uint8_t *reg_ptr;
	
	struct spi_slave_inst_config slave_dev_config;
	
	// Set buffer-overflow status to display immediately so it can be cleared:
	reg_ptr=0x42001001;
	*reg_ptr=0x01;
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
	config_spi_master.transfer_mode = SPI_TRANSFER_MODE_3;//initialize with LCD mode
	config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_E;
	config_spi_master.pinmux_pad0 = PINMUX_PB08D_SERCOM4_PAD0;
	config_spi_master.pinmux_pad2 = PINMUX_PB10D_SERCOM4_PAD2;
	config_spi_master.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;
	config_spi_master.character_size = SPI_CHARACTER_SIZE_8BIT ;
	spi_init(&spi_main, SERCOM4, &config_spi_master);
	spi_enable(&spi_main);
	//ioport_set_pin_mode(miso, IOPORT_MODE_PULLUP);
	//ioport_set_pin_mode(sclk, IOPORT_MODE_PULLUP);
	
}






void config_spi(enum spi_device SPI_DEVICE){
	uint8_t *ptr_POL;
	//uint8_t *ptr_CTRLA,

	//ptr_CTRLA=0x42001000;//  CTRLA byte 1 register
	ptr_POL  =0x42001003;//  SPI polarity register
	spi_disable(&spi_main);
	switch (SPI_DEVICE)
	{
		case LCD:
			*ptr_POL = 0b00110000;
			break;
		case sensors:
			*ptr_POL = 0b00000000;
			break;
		case SD_card:
			*ptr_POL = 0b00000000;
			break;
		default:
			*ptr_POL = 0b00000000;
	}

	spi_enable(&spi_main);

}




void spi_clear(void){
	
	uint8_t *reg_ptr;
	uint8_t read_buffer[4];
	//clear out receive buffer
	while(spi_is_ready_to_read(&spi_main)){	spi_read(&spi_main, read_buffer);	}
	//Clear overflow error bit:
	reg_ptr=0x4200101A;
	*reg_ptr=0x01;
}



//USART
//******************************************

void configure_usart(void){
	struct usart_config config_usart;
	enum status_code usart_status;
	
	//  Laser UART setup SERCOM1
	usart_get_config_defaults(&config_usart);
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
	
	// BLE UART setup SERCOM0
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 115200;
	config_usart.mux_setting = USART_RX_1_TX_0_RTS_2_CTS_3;
	config_usart.pinmux_pad0 = PINMUX_PA08C_SERCOM0_PAD0;
	config_usart.pinmux_pad1 = PINMUX_PA09C_SERCOM0_PAD1;
	config_usart.pinmux_pad2 = PINMUX_PA10C_SERCOM0_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PA11C_SERCOM0_PAD3;
	do {
		usart_status = usart_init(&usart_laser,	SERCOM1, &config_usart) ;
	}while((usart_status != STATUS_OK) && (usart_status != STATUS_ERR_DENIED) );
	usart_enable(&usart_BLE);
	USART_BLE_enabled = true;
	
	//  Setup Callbacks
	configure_usart_callbacks();
	
	//  Initiate background read to buffer
	rxBufferLaserIndex = 0;
	debugBufferIndex = 0;
	usart_read_job(&usart_laser, rxBufferLaser); // 
}

void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_laser,
	writeLaserCallback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_laser,
	readLaserCallback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_laser, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_laser, USART_CALLBACK_BUFFER_RECEIVED);
}




void readLaserCallback(struct usart_module *const usart_module)
{
	//***************debug*******************
	debugBuffer[debugBufferIndex] = rxBufferLaser[rxBufferLaserIndex];
	debugBufferIndex++;
	if(debugBufferIndex>=sizeof(debugBuffer)){debugBufferIndex=0;}
	//**************debug********************
	if(rxBufferLaser[rxBufferLaserIndex]==0xA8){
		//  End of message key received (per laser comm protocol)
		LaserReceiveComplete=true;		
		rxBufferLaserIndex = 0;
		laserCurrentMessage = laserMessageType();;
		
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


enum status_code writeLaser(uint8_t *tx_data, uint16_t length){
	enum status_code writeStatus;
	//clear_rx_buffer();
	LaserTransmitComplete=false;
	writeStatus = usart_write_buffer_job(&usart_laser, tx_data, length);
	return writeStatus;
}
	
	
enum status_code readLaser(){
	enum status_code readStatus;
	//clear_rx_buffer();
	//LaserReceiveComplete = false;
	//Length of returned message is unknown
	//usart_abort_job(&usart_laser, USART_TRANSCEIVER_RX);
	//while(laserReadStatus());
	//usart_abort_job(&usart_laser, USART_TRANSCEIVER_RX);
	//clear_rx_buffer();
	LaserReceiveComplete = false;
	//rxBufferLaserIndex = 0;
	//readStatus = usart_read_job(&usart_laser, rxBufferLaser); // 
	//usart_read_job(&usart_laser, &rx_buffer[rx_buffer_index]);
	return readStatus;
};

bool isLaserTransmitComplete(void){
	return LaserTransmitComplete;
}
bool isLaserReceiveComplete(void){
	return LaserReceiveComplete;
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

//  Determine the type of message currently in the buffer
enum LASER_MESSAGE_TYPE laserMessageType(void){
	uint8_t i;
	for(i=0;i<sizeof(rxBufferLaser);i++){
		if(rxBufferLaser[i]==0xAA){
			return rxBufferLaser[i+2];
		}
		
	}
};



//I2C
//************************************************
void configure_i2c_master(void)
{
	/* Initialize config structure and software module. */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	/* Change buffer timeout to something longer. */
	config_i2c_master.buffer_timeout = 10000;
	//#if SAMR30
	config_i2c_master.pinmux_pad0    = PINMUX_PA12C_SERCOM2_PAD0;
	config_i2c_master.pinmux_pad1    = PINMUX_PA13C_SERCOM2_PAD1;
	//#endif
	/* Initialize and enable device with config. */
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

