/**
	\file STM32F4.h
	\author Andy Gock (modified for STM32F4xx by Moreto)
	\brief Functions specific to STM32 F4 ARM Cortex-M4 devices.
 */

/*
	Copyright (c) 2012, Andy Gock

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
		* Redistributions of source code must retain the above copyright
		  notice, this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.
		* Neither the name of Andy Gock nor the
		  names of its contributors may be used to endorse or promote products
		  derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL ANDY GOCK BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



/* Includes from CMSIS and Peripheral Library */
#include <asf.h>

/* Includes from GLCD */
#include "glcd.h"

extern struct spi_module spi_main;

//void delay_ms(uint32_t ms);

//#define BACKLIGHT_INVERT	// Uncomment if LED backlight turn on with low value

void glcd_init(void)
{


	ioport_set_pin_dir(CONTROLLER_SPI_SS_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(CONTROLLER_SPI_DC_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(CONTROLLER_SPI_RST_PIN, IOPORT_DIR_OUTPUT);

	
	GLCD_DESELECT();
	

	glcd_select_screen((uint8_t *)&glcd_buffer,&glcd_bbox);

	glcd_reset();
	glcd_reset();
	glcd_ST7565R_init();


}



void glcd_spi_write(uint8_t c)
{
	//uint8_t temp;

	GLCD_SELECT();
	/*!< Loop while DR register in not empty */
	//while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
	while(spi_is_ready_to_write(&spi_main)==false);
	
	
	//SPI_I2S_SendData(SPIx, (uint16_t) c);
	spi_write(&spi_main, c);
	
	/* Wait until entire byte has been read (which we discard anyway) */
	//while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) != RESET);
	while(spi_is_write_complete(&spi_main)==false);

	//temp = SPI_I2S_ReceiveData(SPIx);

	GLCD_DESELECT();
}

void glcd_reset(void)
{
	/* Toggle RST low to reset. Minimum pulse 100ns on data sheet. */
	GLCD_SELECT();
	GLCD_RESET_LOW();


	delay_ms(GLCD_RESET_TIME);
	//DelayTask(GLCD_RESET_TIME);
	GLCD_RESET_HIGH();
	GLCD_DESELECT();
}



