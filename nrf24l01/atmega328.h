/********************************************************************************
Includes
********************************************************************************/
#include <stdio.h>
#include <util/delay.h>

#include "../common/util.h"

/********************************************************************************
Macros and Defines
********************************************************************************/
#define SPI_CSN PD6
#define SPI_CE  PD5

/* =========== SPI and GPIO function ============ */
void setup_io();
void setup_spi();
void setCSN(uint8_t value);
void setCE(uint8_t value);
uint8_t transfer_spi(uint8_t tx_);
