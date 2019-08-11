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
#include <avr/eeprom.h>

#include "../nrf24l01/RF24.h"
#include "../common/util.h"
#include "../nrf24l01/atmega328.h"
#include "../atmega328/mtimer.h"

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

#define PT2257_ADDR  0b10001000
#define START 		 0x08
#define MT_SLA_ACK	 0x18   //slave ACK has been received
#define MT_DATA_ACK	 0x28   //master ACK has been received
#define VOLUME_MAX   79

// #define CONTROLLER_CHANNEL 116 // Controller 1 (PC)
#define CONTROLLER_CHANNEL 124 // Controller 2 (Bathroom)

/********************************************************************************
	Function Prototypes
********************************************************************************/
void initGPIO();
void send_spi(uint16_t data);
void sendVolume();
void initTWI();
void initLowVolume();

/********************************************************************************
	Global Variables
********************************************************************************/
RF24 radio;
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
volatile uint8_t volume = VOLUME_MAX;
bool volChanged = false;
volatile uint64_t saveVolJob1Cicles = 0;

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

        //printf("\nRX %d", data[0]);

        if ((len > 3) && (data[0] == 110) && (data[1] == 120) && (data[2] == 130)) {

    		if (data[3] == 100) {
    			if (volume > 1) {
    				volume -= 2;
    			}
    		} else if (data[3] == 101) {
    			if (volume < VOLUME_MAX) {
    				volume += 2;
    			}
    		} else if (data[3] == 102) {
    			volume = data[4];
    		}

    		if (volume > VOLUME_MAX) {
    			volume = VOLUME_MAX;
    		}

    		volChanged = true;

        }

        radio.flush_rx();
    }
}

ISR(TIMER1_OVF_vect)
{
	incrementOvf();
}

/********************************************************************************
	Main
********************************************************************************/
int main(void) {

    // initialize usart module
	usart_init();

    // Init GPIO
    initGPIO();

    initTWI();

    initTimer();

    // enable interrupts
    sei();

    _delay_ms(2000);

	// Console friendly output
    printf(CONSOLE_PREFIX);

    // Set low volume for 10 seconds
    initLowVolume();

    radio.begin();
    radio.setRetries(15, 15);
    radio.setPayloadSize(8);
    radio.setPALevel(RF24_PA_MAX);
	radio.setChannel(CONTROLLER_CHANNEL);

    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);

    radio.startListening();

    radio.printDetails();

    // Read saved volume value from EEPROM
    volume = eeprom_read_byte((uint8_t *) 0);

    // set default volume
    sendVolume();

	// main loop
    while (1) {
    	// main usart loop for console
    	usart_check_loop();

    	if (volChanged) {
    		sendVolume();
    		volChanged = false;
    		saveVolJob1Cicles = convertSecondsToCicles(2); // save volume each 2 seconds
    		_on(PB0, PORTB);
    	}

    	uint64_t currentTimeCicles = getCurrentTimeCicles();

    	/**
    	 * --------> Job 1 (Save volume to EEPROM)
    	 */
    	if ((saveVolJob1Cicles != 0) && (currentTimeCicles >= saveVolJob1Cicles)) {
    		_off(PB0, PORTB);
    		saveVolJob1Cicles = 0;
    		eeprom_write_byte((uint8_t *) 0, volume);
    	}
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

    _out(DDB0, DDRB); // LED1
    _off(PB0, PORTB); // LED1 default 0
}

void initTWI() {
    //set SCL to ?kHz
    TWSR = (1<<TWPS1)|(0<<TWPS0); // Prescaler Value = 16
    TWBR = 0x02;

    //enable TWI
    TWCR = (1<<TWEN);
}

void sendTWI() {

    uint8_t b = volume/10 & 0b0000111;  //get the most significant digit (eg. 79 gets 7) and limit the most significant digit to 3 bit (7)
    uint8_t a = volume%10;  //get the least significant digit (eg. 79 gets 9)

	// Send START condition
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	// Wait for TWINT Flag set. This indicates that the START condition has been transmitted
	while ((TWCR & (1<<TWINT)) == 0);


	if ((TWSR & 0xF8) != START) {
		printf("\nFailed START");
	 	return;
	}

	//Load SLA_W into TWDR Register. Clear TWINT bit in TWCR to start transmission of address
	TWDR = PT2257_ADDR;
	TWCR = (1<<TWINT)|(1<<TWEN);

	// Wait for TWINT Flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
	while ((TWCR & (1<<TWINT)) == 0);

	if ((TWSR & 0xF8) != MT_SLA_ACK) {
		printf("\nFailed MT_SLA_ACK");
	 	return;
	}

	// Load DATA1 into TWDR Register. Clear TWINT bit in TWCR to start transmission of data
	TWDR = 0b11100000 | b;
	TWCR = (1<<TWINT)|(1<<TWEN);

	// Wait for TWINT Flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received.
	while ((TWCR & (1<<TWINT)) == 0);

	if ((TWSR & 0xF8) != MT_DATA_ACK) {
		printf("\nFailed MT_DATA_ACK 1");
	 	return;
	}

	// Load DATA2 into TWDR Register. Clear TWINT bit in TWCR to start transmission of data
	TWDR = 0b11010000 | a;
	TWCR = (1<<TWINT)|(1<<TWEN);

	// Wait for TWINT Flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received.
	while ((TWCR & (1<<TWINT)) == 0);

	if ((TWSR & 0xF8) != MT_DATA_ACK) {
		printf("\nFailed MT_DATA_ACK 2");
	 	return;
	}

	// Transmit STOP condition
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void handle_usart_cmd(char *cmd, char *args) {
	if (strcmp(cmd, "test") == 0) {
		printf("\n TEST [%s]", args);
	}

	if (strcmp(cmd, "send1") == 0) {
		printf("\nsendTWI");
		sendTWI();
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
	sendTWI();
}

void initLowVolume() {
    volume = 70;

    // set min volume
    sendVolume();

    for (int i = 0; i < 10; i++) {
    	_delay_ms(1000);
    }
}
