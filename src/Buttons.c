/* Mouse buttons SR latch debounce
 * https://github.com/R3M0H/
 */

#include "Buttons.h"
#include <avr/io.h>

// SR latch debounce
//
// Left click
// SET: PD4
// RESET: PD7
//
// Right click
// SET: PC7
// RESET: PC6
//
// Middle click
// SET: PD3
// RESET: PD2

#define LEFT_DDR DDRD
#define LEFT_PORT PORTD
#define LEFT_PIN PIND
#define LEFT_SET 4
#define LEFT_RESET 7

#define RIGHT_DDR DDRC
#define RIGHT_PORT PORTC
#define RIGHT_PIN PINC
#define RIGHT_SET 7
#define RIGHT_RESET 6

#define MIDDLE_DDR DDRD
#define MIDDLE_PORT PORTD
#define MIDDLE_PIN PIND
#define MIDDLE_SET 3
#define MIDDLE_RESET 2

void buttonsInit(void){
	// all buttons as input
	LEFT_DDR &= ~(1 << LEFT_SET);	
	LEFT_DDR &= ~(1 << LEFT_RESET);	
	RIGHT_DDR &= ~(1 << RIGHT_SET);	
	RIGHT_DDR &= ~(1 << RIGHT_RESET);	
	MIDDLE_DDR &= ~(1 << MIDDLE_SET);	
	MIDDLE_DDR &= ~(1 << MIDDLE_RESET);	

	// enable pullups
	LEFT_PORT |= (1 << LEFT_SET);	
	LEFT_PORT |= (1 << LEFT_RESET);	
	RIGHT_PORT |= (1 << RIGHT_SET);	
	RIGHT_PORT |= (1 << RIGHT_RESET);	
	MIDDLE_PORT |= (1 << MIDDLE_SET);	
	MIDDLE_PORT |= (1 << MIDDLE_RESET);	
}

uint8_t buttonsCheck(){
	static uint8_t left = 0;
	static uint8_t right = 0;
	static uint8_t middle = 0;

	uint8_t aux_set;
	uint8_t aux_reset;

	// left
	aux_set = 	LEFT_PIN & (1 << LEFT_SET);	
	aux_reset = LEFT_PIN & (1 << LEFT_RESET);	
	if(!aux_set && aux_reset){
		left = 1;
	}
	else if(aux_set && !aux_reset){
		left = 0;
	}
	// right
	aux_set = 	RIGHT_PIN & (1 << RIGHT_SET);	
	aux_reset = RIGHT_PIN & (1 << RIGHT_RESET);	
	if(!aux_set && aux_reset){
		right = 1;
	}
	else if(aux_set && !aux_reset){
		right = 0;
	}
	// middle
	aux_set = 	MIDDLE_PIN & (1 << MIDDLE_SET);	
	aux_reset = MIDDLE_PIN & (1 << MIDDLE_RESET);	
	if(!aux_set && aux_reset){
		middle = 1;
	}
	else if(aux_set && !aux_reset){
		middle = 0;
	}

	return (middle << 2) + (right << 1) + (left << 0);
}
