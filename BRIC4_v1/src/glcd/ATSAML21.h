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




#ifndef ATSAML21_H_
#define ATSAML21_H_

#include <asf.h>
#include <comms\comms.h>


#define CONTROLLER_SPI_SS_PIN   IOPORT_CREATE_PIN(IOPORT_PORTA, 23)
#define CONTROLLER_SPI_DC_PIN   IOPORT_CREATE_PIN(IOPORT_PORTA, 27)
#define CONTROLLER_SPI_RST_PIN  IOPORT_CREATE_PIN(IOPORT_PORTB, 23)


#define GLCD_SELECT()		ioport_set_pin_level(CONTROLLER_SPI_SS_PIN, 0)
#define GLCD_DESELECT()		ioport_set_pin_level(CONTROLLER_SPI_SS_PIN, 1)
#define GLCD_A0_LOW()		ioport_set_pin_level(CONTROLLER_SPI_DC_PIN, 0)
#define GLCD_A0_HIGH()		ioport_set_pin_level(CONTROLLER_SPI_DC_PIN, 1)
#define GLCD_RESET_LOW()	ioport_set_pin_level(CONTROLLER_SPI_RST_PIN, 0)
#define GLCD_RESET_HIGH()	ioport_set_pin_level(CONTROLLER_SPI_RST_PIN, 1)


#endif


#ifndef GLCD_DEVICES_H_
#define GLCD_DEVICES_H_



/** \addtogroup Devices Devices
 *  Functions specific to certain devices (microcontrollers)
 *  \{
 */

/**
 * Initialise the LCD. This function is platform and controller specific.
 */
void glcd_init(void);


void glcd_spi_write(uint8_t c);

/**
 *  Reset the LCD.
 *  \note Not all LCD controllers support reset.
 */
void glcd_reset(void);

/** @}*/

#endif /* GLCD_DEVICES_H_ */
