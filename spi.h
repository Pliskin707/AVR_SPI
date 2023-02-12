/*
 * spi.h
 *
 *  Created on: 23.08.2018
 *      Author: andre
 */

#ifndef SPI_H_
#define SPI_H_
#include "../../includes.h"
#ifdef SPI_ENABLE

/**
 * Notes:
 * - the Slave Select (~SS) line must be activated (pulled low) by software
 * - for now only master is supported
 * - IMPORTANT! if the µC is master and the ~SS pin is configured as input, the µC becomes a Slave if the input is low!
 * 		-> use as output (right LED on arduino) to avoid this
 */

typedef enum {
	e_spi_mode_sample_rising_setup_falling = 0,
	e_spi_mode_setup_rising_sample_falling = 1,
	e_spi_mode_sample_falling_setup_rising = 2,
	e_spi_mode_setup_falling_sample_rising = 3
} e_spi_mode;

typedef enum {
	e_spi_pre_4			= 0,
	e_spi_pre_16		= 1,
	e_spi_pre_64		= 2,
	e_spi_pre_128		= 3,
	e_spi_pre_MASTER_2	= 4,	// only supported in master mode
	e_spi_pre_8			= 5,
	e_spi_pre_32		= 6,
	e_spi_pre_64_dbl	= 7,	// doubled spi speed (with lower PLL speed)
} e_spi_prescaler;

int8_t spi_init_master (const e_spi_mode mode, const e_spi_prescaler pre);
int8_t spi_interrupt_rx (const uint8_t rx_on_off, void (*callback) (const uint8_t rx));
uint8_t spi_is_ready (void);
int8_t spi_send_byte (const uint8_t byte);
int8_t spi_fast_transfer (uint8_t volatile * done, const uint8_t * data, const uint16_t size, const uint16_t repeat);

#endif /* SPI_H_ */
#endif
