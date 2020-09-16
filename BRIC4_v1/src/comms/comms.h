/*
 * comms.h
 *
 * Created: 6/10/2018 4:56:09 PM
 *  Author: Kris Fausnight
 */ 

#ifndef COMMS_H_
#define COMMS_H_

#include <asf.h>
#include <clockSetup.h>


#define lcd_SS	IOPORT_CREATE_PIN(IOPORT_PORTA, 23)//CS6
#define acc1_SS	IOPORT_CREATE_PIN(IOPORT_PORTA, 18)//CS1
#define acc2_SS	IOPORT_CREATE_PIN(IOPORT_PORTA, 19)//CS2
#define mag1_SS IOPORT_CREATE_PIN(IOPORT_PORTA, 21)//CS4
#define mag2_SS IOPORT_CREATE_PIN(IOPORT_PORTA, 20)//CS3
#define SD_CS	IOPORT_CREATE_PIN(IOPORT_PORTA, 15)//CS7

#define SDA		IOPORT_CREATE_PIN(IOPORT_PORTA, 12)
#define SCL		IOPORT_CREATE_PIN(IOPORT_PORTA, 13)
#define MCU_TX1	IOPORT_CREATE_PIN(IOPORT_PORTA, 8)//bluetooth module TX
#define MCU_RX1	IOPORT_CREATE_PIN(IOPORT_PORTA, 9)//bluetooth module RX
#define MCU_RTS1		IOPORT_CREATE_PIN(IOPORT_PORTA, 10)//bluetooth module RTS
#define MCU_CTS1		IOPORT_CREATE_PIN(IOPORT_PORTA, 11)//bluetooth module CTS
#define MCU_TX2	IOPORT_CREATE_PIN(IOPORT_PORTA, 16)
#define MCU_RX2	IOPORT_CREATE_PIN(IOPORT_PORTA, 17)
#define mosi	IOPORT_CREATE_PIN(IOPORT_PORTB, 10)//SPI MOSI
#define miso	IOPORT_CREATE_PIN(IOPORT_PORTB, 8)//SPI MISO
#define sclk	IOPORT_CREATE_PIN(IOPORT_PORTB, 11)//SPI SCLK

//  SPI Max Baud Rates
#define baudMaxBL652	 4000000
#define baudMaxSD		10000000
#define baudMaxDisp		 2500000
#define baudMaxAcc		 2500000
#define baudMaxComp		 1000000
#define baudRateMin		baudMaxComp
#define baudRateMax		baudMaxSD


enum read_write {readp, writep};
enum spi_device	{LCD, sensors, SD_card};
//Disable comms
void disable_comms(void);
void enable_comms(void);


// SPI setup
void spi_setBaud(uint32_t );
void setup_spi(void);
//void config_spi(enum spi_device);
struct spi_module spi_main; //Master software module
struct spi_config config_spi_master;//  Master configuration
struct spi_slave_inst	slave_lcd;
struct spi_slave_inst	slave_acc1;
struct spi_slave_inst	slave_acc2;
struct spi_slave_inst	slave_mag1;
struct spi_slave_inst	slave_mag2;
struct spi_slave_inst	slave_SD;

//void configure_spi_master_callbacks(void);
//static void callback_spi_master( struct spi_module *const );
//bool isSpiTransrevComplete(void);
//void resetSpiTransrevComplete(void);

//USART
enum LASER_MESSAGE_TYPE{
	NONE = 0x00,
	SW_VERSION = 0x01,
	DEVICE_TYPE = 0x02,
	DEVICE_STATUS = 0x08,
	READ_SLAVE_ADDRESS = 0x04,
	SET_SLAVE_ADDRESS = 0x41,
	LASER_ON = 0x42,
	LASER_OFF = 0x43,
	SINGLE_MEASUREMENT = 0x44,
	CONT_MEASUREMENT = 0x45,
	STOP_MEASUREMENT = 0x46,
	BEEP_ON_OFF = 0x47,
};


enum LASER_MESSAGE_TYPE laserMessageType(void);
struct usart_module usart_laser;
struct usart_module usart_BLE;
enum status_code writeLaser(uint8_t *tx_data,uint16_t length);
enum status_code writeBle(uint8_t *tx_data,uint16_t length);
void readLaserCallback(struct usart_module *const usart_module);
void writeLaserCallback(struct usart_module *const usart_module);
void readBleCallback(struct usart_module *const usart_module);
void writeBleCallback(struct usart_module *const usart_module);
void configure_usart_Laser(void);
void configure_usart_BLE(void);
void BLE_usart_isolate(void);
bool isBleCommEnabled(void);
bool isLaserTransmitComplete(void);
bool isLaserReceiveComplete(void);
bool isBleTransmitComplete(void);
bool isBleReceiveComplete(void);
void rxBufferLaserClear(void);
void rxBufferBleClear(void);


// I2C setup
//I2C addresses   note:  must be 7-bit; does not include R/W bit
#define led_add		0x62  //0b1100010RW
#define batt_add	0x36  //0b0110110RW
//#define EEPROM_add	0x53  //0b1010011RW
#define rtc_add 0x68  //0b1101000RW
#define charger_add 0x14 //0b00001 0100 read 0x28, write 0x29


struct i2c_master_module i2c_master_instance;
void configure_i2c_master(void);
void i2c_read_write(enum read_write, uint8_t, uint8_t *, uint8_t);
void max17055_reg_read_write(enum read_write, uint8_t, uint16_t *);
void adp5062_reg_read_write(enum read_write, uint8_t , uint8_t *);




//#include <main.h>



#endif /* COMMS_H_ */