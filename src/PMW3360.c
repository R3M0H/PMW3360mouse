/* PMW3360 sensor library 
 * https://github.com/R3M0H/
 *
 * Some stuff from https://github.com/mrjohnk/PMW3360DM-T2QU
 */

#include "PMW3360.h"
#include "PMW3360DM_srom_0x04.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

// Registers
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

// spi stuff
#define NCS_PIN 0
#define PORT_SPI PORTB
#define ncs_low() (PORT_SPI &= ~(1<<NCS_PIN))
#define ncs_high() (PORT_SPI |= (1<<NCS_PIN))

// delays
#define delay_us(t) __builtin_avr_delay_cycles((t) * (F_CPU/1000000))
#define delay_ms(t) __builtin_avr_delay_cycles((t) * (F_CPU/1000))


// Global Variables
uint8_t motion_read[MOTION_READ_BYTES];


// Private Variables
static uint8_t _init_complete=0;
static uint8_t _last_was_motion_burst_read=0;


// Private Functions 
static uint8_t spiXfer(uint8_t txData);
static uint8_t readRegister(uint8_t reg_addr);
static void writeRegister(uint8_t reg_addr, uint8_t data);
static uint8_t sromDownload(void);
static void defaultConfig(void);

static uint8_t spiXfer(uint8_t txData){
	// Load data into the buffer
    SPDR = txData;
 	
	//Wait until transmission complete
	while(!(SPSR & (1<<SPIF)));
	
	// Return received data
	return(SPDR);
}

static uint8_t readRegister(uint8_t reg_addr){
	ncs_low();
	
	// First byte is the address of the register with MSB = 0
	spiXfer(reg_addr & 0x7f );
	delay_us(160); // tSRAD
	// Second byte is the data
	uint8_t data = spiXfer(0);
	
	delay_us(1); // tSCLK-NCS for read operation is 120ns
	ncs_high();
	delay_us(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS
	
	_last_was_motion_burst_read = 0;

	return data;
}

static void writeRegister(uint8_t reg_addr, uint8_t data){
	ncs_low();
	
	// First byte is the address of the register with MSB = 1
	spiXfer(reg_addr | 0x80 );
	// Second byte is the data
	spiXfer(data);
	
	delay_us(35); // tSCLK-NCS for write operation
	ncs_high();
	delay_us(120); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 

	_last_was_motion_burst_read = 0;
}

static uint8_t sromDownload(void){
	//Write 0 to Rest_En bit of Config2 register to disable Rest mode.
	readRegister(Config2);
	writeRegister(Config2, 0x00 & 0xdf);
	
	//Write 0x1d in SROM_Enable register for initializing
	writeRegister(SROM_Enable, 0x1d); 
	
	//Wait for 10 ms
	delay_ms(10);

	//Write 0x18 to SROM_Enable register again to start SROM Download
	writeRegister(SROM_Enable, 0x18); 
	
	//Write the SROM file into SROM_Load_Burst register
	//first data must start with SROM_Load_Burst address.
	ncs_low();
	spiXfer(SROM_Load_Burst | 0x80);
	delay_us(15);
	
	// upload the firmware
	for(int i = 0; i < firmware_length; i++){ 
		spiXfer(pgm_read_byte(firmware_data + i));
		delay_us(15);
	}
	ncs_high();
	
	//Soonest to read SROM_ID
	delay_us(200);
	
	_last_was_motion_burst_read = 0;

	//Read the SROM_ID register to verify the ID before any other register reads or writes.
	return(readRegister(SROM_ID));
}

static void defaultConfig(void){
	//Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
	writeRegister(Config2, 0x00);
	// set CPI resolution
	writeRegister(Config1, 0x08); //900 CPI
}


// Global Functions

/** Configures the board SPI interface. */
void spiInit(void){
	// SS	PB0
	// SCLK	PB1
	// MOSI	PB2
	// MISO	PB3
	DDRB |= (1<<2) | (1<<1) | (1<<0);	// MOSI, SCLK, SS as outputs
	DDRB |= (1<<0); PORTB |= (1<<0); 	// set the hardware SS pin to low to enable SPI

	//SPI enable, master, mode 3
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA);	
}

void PMW3360powerUp(void){
	if(_init_complete) return;

	delay_ms(100);

	// Reset SPI port
	ncs_high();
	delay_us(1);
	ncs_low();
	delay_us(1);
	ncs_high();
	delay_us(1);
	
	// Write 0x5A to Power_Up_Reset register
	writeRegister(Power_Up_Reset, 0x5a);

	// Wait for at least 50ms.
	delay_ms(50);

	// Read from registers 0x02 to 0x06
	readRegister(Motion);
	readRegister(Delta_X_L);
	readRegister(Delta_X_H);
	readRegister(Delta_Y_L);
	readRegister(Delta_Y_H);

	// Perform SROM download
	uint8_t status = sromDownload();

	// Load configuration for other registers
	defaultConfig();

	delay_ms(10);

	if(status == srom_version) _init_complete = 1;	
	else _init_complete = 0;
}

void PMW3360motionBurst(void){
	if(!_init_complete) return;

	if(!_last_was_motion_burst_read){
		writeRegister(Motion_Burst, 0x69); // write any value to motion burst register
		_last_was_motion_burst_read = 1;
	}

	ncs_low(); 						// lower ncs
	spiXfer(Motion_Burst & 0x7f); 	// send motion burst address
	delay_us(35);					// wait for tsrad_motbr

	// read continuouly up to 12 bytes
	for(uint8_t i=0; i<MOTION_READ_BYTES; i++){
		motion_read[i] = spiXfer(0);
	}

	ncs_high();		// pull ncs high
	delay_us(1);	// wait for at least tbexit
}

void PMW3360shutdown(void){
	if(!_init_complete) return;

	// Write 0xB6 to set the chip to shutdown mode.
	writeRegister(Shutdown, 0xB6);

	_last_was_motion_burst_read = 0;
	_init_complete = 0;
}

uint8_t PMW3360status(void){
	return _init_complete;
}
