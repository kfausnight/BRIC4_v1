/*
 * comms.h
 *
 * Created: 6/10/2018 4:56:09 PM
 *  Author: Kris Fausnight
 */ 

#ifndef COMMS_H_
#define COMMS_H_

#include <asf.h>

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




enum read_write {readp, writep};
enum spi_device	{LCD, sensors, SD_card};
//Disable comms
void disable_comms(void);


// SPI setup
void setup_spi(void);
void config_spi(enum spi_device);
struct spi_module spi_main; //Master software module
struct spi_config config_spi_master;//  Master configuration
struct spi_slave_inst	slave_lcd;
struct spi_slave_inst	slave_acc1;
struct spi_slave_inst	slave_acc2;
struct spi_slave_inst	slave_mag1;
struct spi_slave_inst	slave_mag2;
struct spi_slave_inst	slave_SD;
void spi_clear(void);

//USART
#define rx_buffer_length	20
volatile uint8_t rx_buffer[rx_buffer_length];
volatile uint8_t rx_buffer_index;
volatile bool reception_complete;
volatile bool write_complete;
struct usart_module usart_laser;
struct usart_module usart_BLE;
void clear_rx_buffer(void);
void usart_read_callback(struct usart_module *const usart_module);
void usart_write_callback(struct usart_module *const usart_module);
void configure_usart(void);
void configure_usart_callbacks(void);


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








#endif /* COMMS_H_ */