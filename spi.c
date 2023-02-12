/*
 * spi.c
 *
 *  Created on: 23.08.2018
 *      Author: andre
 */

#include "../../includes.h"
#include "spi.h"

#ifdef SPI_ENABLE

// receive callback
static void (*callback_rx) (const uint8_t rx);
static uint8_t spi_is_free = TRUE;

static struct
{
	uint8_t ndx;
	uint8_t volatile * done;
	const uint8_t * data;
	uint16_t size;
	uint16_t repeat;
} fast_transfer;

// Pin	|  	Master	|	Slave		// pin name AVR
//-----------------------------
// MOSI |	out		|	in			// PB2
// MISO |	in		| 	out			// PB3
// SCK  |	out		|	in			// PB1
// ~SS  | 	out		|	in			// PB0

int8_t spi_init_master (const e_spi_mode mode, const e_spi_prescaler pre)
{
	// set pin configuration:
	DDRB |= (1 << PB2) | (1 << PB1);

	// clear the "power reduction" bit
	PRR0 &= ~(1 << PRSPI);

	// set the clock speed, set master
	SPCR = (1 << MSTR) | (pre & 0x03);

	// double the spi speed?
	if (pre & (1 << 2))
		SPSR |= (1 << SPI2X);
	else
		SPSR &= ~(1 << SPI2X);

	// set clock polarity and phase
	SPCR |= (mode << 2);

	// enable SPI, enable interrupt
	SPCR |= (1 << SPE) | (1 << SPIE);

	return NOERR;
}

uint8_t spi_is_ready (void)
{
	return spi_is_free;
}

int8_t spi_interrupt_rx (const uint8_t rx_on_off, void (*callback) (const uint8_t rx))
{
	if (rx_on_off == OFF)
	{
		callback_rx = NULL;
		return NOERR;
	}
	else if (callback != NULL)
	{
		callback_rx = callback;
		return NOERR;
	}

	return CONFIG;	// can not enable NULL callback
}

int8_t spi_send_byte (const uint8_t byte)
{
	// check if the previous byte was send already
	if (spi_is_ready())
	{
		spi_is_free = FALSE;
		SPDR = byte;
		return NOERR;
	}

	return BUSY;
}

static void stream_data (const uint8_t rx)
{
	if (++fast_transfer.ndx >= fast_transfer.size)
	{
		fast_transfer.ndx = 0;
		if (fast_transfer.repeat)
			fast_transfer.repeat--;
		else
		{
			spi_interrupt_rx(OFF, NULL);	// done
			*fast_transfer.done = TRUE;
			return;
		}
	}

	spi_is_free = FALSE;
	SPDR = fast_transfer.data[fast_transfer.ndx];
}

int8_t spi_fast_transfer (uint8_t volatile * done, const uint8_t * data, const uint16_t size, const uint16_t repeat)
{
	if (callback_rx != NULL)
		return CONFIG;

	// save information
	fast_transfer.ndx  = 0;
	fast_transfer.data = data;
	fast_transfer.done = done;
	fast_transfer.size = size;
	fast_transfer.repeat = repeat;

	// wait for spi
	while (spi_is_free != TRUE);

	// set callback
	spi_interrupt_rx(ON, &stream_data);

	// start transfer
	spi_send_byte(data[0]);

	return NOERR;
}

//!! check if the master bit (MSTR) is still set on receive!
// if another device pulled the µCs ~SS pin low it becomes a slave automatically!
ISR (SPI_STC_vect)
{
	uint8_t rx_data = SPDR;
	if (callback_rx != NULL)
		callback_rx (rx_data);

	spi_is_free = TRUE;
}
#endif
