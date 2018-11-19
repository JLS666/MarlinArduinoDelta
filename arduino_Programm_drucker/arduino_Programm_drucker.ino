#include "tmc2130_kd.h"

void setup() 
{

	// for generating step signal for testing,
	// see tmc2130_kd.h for more details
//  init_interrupt();

	// for debugging, adjusting of tmc 
//  Serial.begin(250000);
  
	// start spi
	 SPI.begin();
  
	// init tmc2130
	tmc2130_init();
  
	// for debugging, adjusting of tmc  
//	tmc2130_read(0x6D);
//  tmc2130_print_data();
 
}

void loop() 
{

}
