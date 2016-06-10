/********************************************************************************
	Includes
********************************************************************************/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>

#include "../nrf24l01/RF24.h"
#include "../common/util.h"
#include "../nrf24l01/atmega328.h"

extern "C" {
#include "../atmega328/usart.h"
}

/********************************************************************************
	Macros and Defines
********************************************************************************/
#define DEBUG_ARRAY_SIZE 1
#define SKIP_REPEATED_IR 3

#define VP0 0
#define VP1 1

#define VC0 4
#define VC1 5

/********************************************************************************
	Function Prototypes
********************************************************************************/
void initGPIO();
void send_spi(uint16_t data);
void sendVolume();

/********************************************************************************
	Global Variables
********************************************************************************/
RF24 radio;
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
volatile uint8_t volume = 0;

/********************************************************************************
	Interrupt Service
********************************************************************************/
ISR(USART_RX_vect)
{
	handle_usart_interrupt();
}

ISR(INT0_vect)
{
    bool tx_ok, tx_fail, rx_ok;
    radio.whatHappened(tx_ok, tx_fail, rx_ok);

    if (rx_ok) {
        uint8_t data[50];
        uint8_t len = radio.getDynamicPayloadSize();
        radio.read(data, len);

		if (data[0] == 100) {
			if (volume != 255) {
				volume+=5;
			}
		} else if (data[0] == 101) {
			if (volume != 0) {
				volume-=5;
			}
		} else if (data[0] == 102) {
			volume = data[1];
		}

		sendVolume();

        radio.flush_rx();
    }
}

/********************************************************************************
	Main
********************************************************************************/
int main(void) {
    // initialize usart module
	usart_init();

    // Init GPIO
    initGPIO();

    // enable interrupts
    sei();

    // set default volume
    sendVolume();

	// Console friendly output
    printf(CONSOLE_PREFIX);

    radio.begin();
    radio.setRetries(15, 15);
    radio.setPayloadSize(8);
    radio.setPALevel(RF24_PA_MAX);
    radio.setChannel(116);

    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);

    radio.startListening();

    radio.printDetails();

	// main loop
    while (1) {
    	// main usart loop for console
    	usart_check_loop();
    }
}

/********************************************************************************
	Functions
********************************************************************************/
void initGPIO() {
    _in(DDD2, DDRD); // INT0 input

    // GPIO Interrupt INT0
    // The falling edge of INT0 generates an interrupt request.
    EICRA = (0<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00);
    // Enable INT0
    EIMSK = (1<<INT0);

    _out(DDC0, DDRC); // SI
    _out(DDC1, DDRC); // SCK
    _out(DDC2, DDRC); // CSN

    _on(PC2, PORTC); // Default disable CSN
    _off(PC1, PORTC); // SCK default 0
}

void handle_usart_cmd(char *cmd, char *args) {
	if (strcmp(cmd, "test") == 0) {
		printf("\n TEST [%s]", args);
	}

	if (strcmp(cmd, "send1") == 0) {
		transfer_spi(100);
		transfer_spi(101);
	}

	if (strcmp(cmd, "send2") == 0) {
		uint8_t spi_cmd = (0<<VC1)|(1<<VC0)|(1<<VP1)|(1<<VP0);
		uint8_t spi_data = atoi(args);
		uint16_t spi_packet = (((uint16_t) spi_cmd) << 8) | (uint16_t) spi_data;
		send_spi(spi_packet);
	}
}

void send_spi(uint16_t data) {
	_off(PC2, PORTC); // Enable CSN

	uint8_t i = 15;
	do {
		_off(PC1, PORTC); // 0 -> SCK

		_delay_loop_1(10);

		if (GET_REG1_FLAG(data, i)) {
			_on(PC0, PORTC); // 1 -> SI
		} else {
			_off(PC0, PORTC); // 0 -> SI
		}

		_delay_loop_1(10);

		_on(PC1, PORTC); // 1 -> SCK
		_delay_loop_1(40);
	} while (i--);

	_off(PC1, PORTC); // 0 -> SCK
	_delay_loop_1(40);

	_off(PC0, PORTC); // 0 -> SI
	_on(PC2, PORTC); // Disable CSN
	_delay_loop_1(5);
}

void sendVolume() {
	//printf("\nsend  volume %d", volume);

	uint8_t spi_cmd = (0<<VC1)|(1<<VC0)|(1<<VP1)|(1<<VP0);
	uint8_t spi_data = volume;
	uint16_t spi_packet = (((uint16_t) spi_cmd) << 8) | (uint16_t) spi_data;
	send_spi(spi_packet);
}
