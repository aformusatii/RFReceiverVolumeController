/********************************************************************************
Includes
********************************************************************************/
#include <avr/io.h>
#include <stdio.h>
#include "../common/util.h"

/********************************************************************************
	Macros and Defines
********************************************************************************/
#define _to_uint64(x,y) ((uint64_t) x << 16) | y

/********************************************************************************
Function Prototypes
********************************************************************************/
void initTimer();
void incrementOvf();
uint64_t convertSecondsToCicles(uint16_t value);
uint64_t getCurrentTimeCicles();
uint32_t getElapsedMilliseconds(uint64_t startTime);
