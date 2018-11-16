/* PMW3360 sensor library 
 * https://github.com/R3M0H/
 */

#pragma once
#include <stdint.h>

struct motion_burstDATA_t{
	uint8_t motion;
	uint8_t observation;
	uint8_t delta_x_l;
	uint8_t delta_x_h;
	uint8_t delta_y_l;
	uint8_t delta_y_h;
	uint8_t squal;
	uint8_t raw_data_sum;
	uint8_t maximum_raw_data;
	uint8_t minimum_raw_data;
	uint8_t shutter_upper;
	uint8_t shutter_lower;
};

/*	Defines how many bytes will be read
	in the Burst Mode operation */
#define MOTION_READ_BYTES 6
extern uint8_t motion_read[MOTION_READ_BYTES];

//Functions
void spiInit(void);
void PMW3360powerUp(void);
void PMW3360motionBurst(void);
void PMW3360shutdown(void);
uint8_t PMW3360status(void);
