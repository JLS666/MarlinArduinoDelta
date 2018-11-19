
#ifndef TMC2130_KD_H
#define TMC2130_KD_H
	
#include <SPI.h>

// pin defines
#define TMC2130_CS_PIN    49
#define TMC2130_EN_PIN    29
//#define TMC2130_STEP_PIN  30
//#define TMC2130_DIR_PIN   31

// for writing register of tmc
void tmc2130_write(byte pRegisterAdress, byte pBit24_31, byte pBit16_23, byte pBit8_15, byte pBit0_7);

// for init of tmc
void tmc2130_init();
	
// for reading data from tmc
void tmc2130_read(byte pRegisterAdress);

// for get data of tmc and serial print of SG_Result
void tmc2130_print_SG_RESULT();

// print data of received register
// use this after tmc2130_read();
void tmc2130_print_data();


/*
//****************************************************
// Use this only for testing on standalone hardware!
// This functions working only on Microchip/Atmel/AVR
// based Arduino (Mega/Uno/Nano)
//****************************************************
// timer interrupt for generating step signal
void init_interrupt();

// ISR for toggle step pin of tmc
// works only with function "init interrupt()"

ISR(TIMER1_COMPA_vect);
*/
			
#endif
