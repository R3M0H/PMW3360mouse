/* Mouse wheel encoder handler
 * https://github.com/R3M0H/
 *
 * Based on https://github.com/brianlow/Rotary
 * 	Copyright 2011 Ben Buxton. Licenced under the GNU GPL Version 3.
 */

#include "Rotary.h"
#include <avr/io.h>


/*
 * The below state table has, for each state (row), the new state
 * to set based on the next encoder output. From left to right in,
 * the table, the encoder outputs are 00, 01, 10, 11, and the value
 * in that position is the new state to set.
 */


// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Counter-clockwise step.
#define DIR_CCW 0x20
// Enable this to emit codes twice per step.
#define HALF_STEP
// Enable weak pullups
#define ENABLE_PULLUPS



#define R_START 0x0

#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5
static const uint8_t ttable[6][4] = {
	// R_START (00)
	{R_START_M,				R_CW_BEGIN,		R_CCW_BEGIN,	R_START},
	// R_CCW_BEGIN
	{R_START_M | DIR_CCW,	R_START,		R_CCW_BEGIN,	R_START},
	// R_CW_BEGIN
	{R_START_M | DIR_CW,	R_CW_BEGIN,		R_START,		R_START},
	// R_START_M (11)
	{R_START_M,				R_CCW_BEGIN_M,	R_CW_BEGIN_M,	R_START},
	// R_CW_BEGIN_M
	{R_START_M,				R_START_M,		R_CW_BEGIN_M,	R_START | DIR_CW},
	// R_CCW_BEGIN_M
	{R_START_M,				R_CCW_BEGIN_M,	R_START_M,		R_START | DIR_CCW},
};
#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

static const uint8_t ttable[7][4] = {
	// R_START
	{R_START,		R_CW_BEGIN,		R_CCW_BEGIN,	R_START},
	// R_CW_FINAL
	{R_CW_NEXT,		R_START,		R_CW_FINAL,		R_START | DIR_CW},
	// R_CW_BEGIN
	{R_CW_NEXT,		R_CW_BEGIN,		R_START,		R_START},
	// R_CW_NEXT
	{R_CW_NEXT,		R_CW_BEGIN,		R_CW_FINAL,		R_START},
	// R_CCW_BEGIN
	{R_CCW_NEXT,	R_START,		R_CCW_BEGIN,	R_START},
	// R_CCW_FINAL
	{R_CCW_NEXT,	R_CCW_FINAL,	R_START,		R_START | DIR_CCW},
	// R_CCW_NEXT
	{R_CCW_NEXT,	R_CCW_FINAL,	R_CCW_BEGIN,	R_START},
};
#endif

#define WHEEL_DDR_A DDRB
#define WHEEL_PORT_A PORTB
#define WHEEL_PIN_A PINB
#define WHEEL_A 6
#define WHEEL_DDR_B DDRF
#define WHEEL_PORT_B PORTF
#define WHEEL_PIN_B PINF
#define WHEEL_B 7

static uint8_t state;

void rotaryInit(void) {
	// Set pins to input.
	WHEEL_DDR_A &= ~(1 << WHEEL_A);	
	WHEEL_DDR_B &= ~(1 << WHEEL_B);	

#ifdef ENABLE_PULLUPS
	WHEEL_PORT_A |= (1 << WHEEL_A);	
	WHEEL_PORT_B |= (1 << WHEEL_B);	
#endif
	// Initialise state.
	state = R_START;
}

int8_t rotaryProcess(void) {
	// Grab state of input pins.
	uint8_t wheel_a;
	uint8_t wheel_b;
	wheel_a = (WHEEL_PIN_A & (1 << WHEEL_A)) ? 0x01 : 0x00;
	wheel_b = (WHEEL_PIN_B & (1 << WHEEL_B)) ? 0x02 : 0x00;

	// Determine new state from the pins and state table.
	state = ttable[state & 0xf][wheel_a + wheel_b];

	// Return the event.
	if((state & 0x30) == DIR_CW){
		return 1;
	}else if((state & 0x30) == DIR_CCW){
		return -1;
	}else{
		return 0;
	}
}
