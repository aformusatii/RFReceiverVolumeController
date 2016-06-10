/********************************************************************************
Includes
********************************************************************************/
#include "atmega328.h"

/********************************************************************************
	Global Variables
********************************************************************************/
volatile uint64_t startTime = 0;

/* ======================================================= */
// Set up a memory regions to access GPIO
void setup_io()
{
	_out(SPI_CSN, DDRD); // CSN
    _out(SPI_CE, DDRD); // CE
	_out(DDB5, DDRB); // SCK
	 _in(DDB4, DDRB); // MISO
	_out(DDB3, DDRB); // MOSI
	// SS pin of AVR must be configured as output.
	// If it is input and it happens to be ever low, the AVR SPI hardware will drop out of master mode into slave mode,
	// and it will not resume until you put it back into master mode. So basically you cannot use AVR SS pin as input when you want to be the SPI master.
	_out(DDB2, DDRB); // SS
} // setup_io

/* ======================================================= */
// Set up SPI interface
void setup_spi()
{
	/* Enable SPI, Master, set clock rate fck/2 */
	SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0);
	SPSR |= (1<<SPI2X);
} // setup_spi

/* ======================================================= */
void setCSN(uint8_t value)
{
	if (value) {
		_on(SPI_CSN, PORTD);
	} else {
		_off(SPI_CSN, PORTD);
	}
}

/* ======================================================= */
void setCE(uint8_t value)
{
	if (value) {
		_on(SPI_CE, PORTD);
	} else {
		_off(SPI_CE, PORTD);
	}
}

/* ======================================================= */
// SPI transfer
uint8_t transfer_spi(uint8_t tx_)
{
	/* Start transmission */
	SPDR = tx_;

	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));

	/* Return data register */
	return SPDR;
} // transfer_spi
