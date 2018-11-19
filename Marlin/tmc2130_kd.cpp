#include "tmc2130_kd.h"

// tmc defines
#define STALLGUARD2_THRESHOLD_VALUE 0 // -64..63 
#define STALLGUARD2_FILTER_ENABLE 0   // disabel

// variable buffer to read results of driver	
byte tmc2130_read_data[6];

// for writing register of tmc2130
void tmc2130_write(byte pRegisterAdress, byte pBit24_31, byte pBit16_23, byte pBit8_15, byte pBit0_7)
{
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));	// spi begin
    digitalWrite(TMC2130_CS_PIN, LOW);  		// select chip
    SPI.transfer(pRegisterAdress | 0x80);		// write adress + write
    SPI.transfer(pBit24_31);			// transfer data
    SPI.transfer(pBit16_23);			// transfer data
    SPI.transfer(pBit8_15);				// transfer data
    SPI.transfer(pBit0_7);				// transfer data
    digitalWrite(TMC2130_CS_PIN, HIGH);	// deselct chip
    SPI.endTransaction();				// spi end
}

// for init of tmc
void tmc2130_init()
{
	pinMode(TMC2130_CS_PIN, OUTPUT);	// spi cs pin -> output
	#ifdef TMC2130_EN_PIN
 	  pinMode(TMC2130_EN_PIN, OUTPUT);	// tmc enabel -> output
  #endif
  #ifdef TMC2130_STEP_PIN
	  pinMode(TMC2130_STEP_PIN, OUTPUT);	// tmc step pin -> output
  #endif
  #ifdef TMC2130_EN_PIN
	  digitalWrite(TMC2130_EN_PIN, HIGH);	// disable tmc
  #endif
	digitalWrite(TMC2130_CS_PIN, HIGH); // deselct chip
	delay(1);	// wait for next action (until tmc is ready)

	/*    
	// Initialization Example from Trinamic TMC2130 Datasheet
	
	// SPI send: 0xEC000100C3; // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
	tmc2130_write(0x6C, 0x00, 0x01, 0x00, 0xC3);
	
	// SPI send: 0x9000061F0A; // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
	tmc2130_write(0x10, 0x00, 0x06, 0x1F, 0x0A);
	
	// SPI send: 0x910000000A; // TPOWERDOWN=10: Delay before power down in stand still
	tmc2130_write(0x11, 0x00, 0x00, 0x00, 0x0A);
	
	// SPI send: 0x8000000004; // EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF)
	tmc2130_write(0x00, 0x00, 0x00, 0x00, 0x04);
	
	// SPI send: 0x93000001F4; // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
	tmc2130_write(0x13, 0x00, 0x00, 0x01, 0xF4);
		
	// SPI send: 0xF0000401C8; // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1
		tmc2130_write(0x70, 0x00, 0x04, 0x01, 0xC8);
	*/    
	
	// initializing of tmc
	// SPI send: 0xEC000100C3; // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
	tmc2130_write(0x6C, 0x18, 0x01, 0x00, 0xC3);
	
	// SPI send: 0x6D  // COOLCONF: 
	tmc2130_write(0x6D, STALLGUARD2_FILTER_ENABLE, STALLGUARD2_THRESHOLD_VALUE, 0x00, 0x00);
	
	// SPI send: 0x9000061F0A; // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
	//tmc2130_write(0x10, 0x00, 0x06, 0x1F, 0x0A);
	tmc2130_write(0x10, 0x00, 0x06, 0x1A, 0x0A); //IRUN=16
	
	// SPI send: 0x910000000A; // TPOWERDOWN=10: Delay before power down in stand still
	tmc2130_write(0x11, 0x00, 0x00, 0x00, 0x0A);
	
	// SPI send: 0x8000000004; // EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF)
	//tmc2130_write(0x00, 0x00, 0x00, 0x00, 0x04);
	tmc2130_write(0x00, 0x00, 0x00, 0x00, 0x00);  
	
	// SPI send: 0x93000001F4; // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
	tmc2130_write(0x13, 0x00, 0x00, 0x01, 0xF4);
	
	// SPI send: 0xF0000401C8; // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1
	tmc2130_write(0x70, 0x00, 0x04, 0x01, 0xC8);
	
	digitalWrite(TMC2130_EN_PIN, LOW); 	// set enable pin of tmc
}
	
// for reading data from tmc
void tmc2130_read(byte pRegisterAdress)
{
	tmc2130_read_data[5]= pRegisterAdress;	// register of the data from tmc
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // spi begin
	digitalWrite(TMC2130_CS_PIN, LOW);  // select chip
	// first write register adress and read spi status
	tmc2130_read_data[0] = SPI.transfer(pRegisterAdress);	 

	// then write dummys and read data bytes of register
	for(int i = 1; i < 5;i++)
	{
		tmc2130_read_data[i] = SPI.transfer(0x00);
	}

	digitalWrite(TMC2130_CS_PIN, HIGH);	// deselect chip
	SPI.endTransaction();				// spi end
}

// for get data of tmc and serial print of SG_Result
void tmc2130_print_SG_RESULT()
{
	tmc2130_read(0x6F);				// read register with sg_result
	Serial.print("SPI_status: 0b");	// print spi status of tmc
	Serial.print(tmc2130_read_data[0], BIN); // print spi status of tmc	 
	int SG_RESULT = 0;				// tmp variable for SG_Result
	SG_RESULT = (tmc2130_read_data[3] & 0b00000011);	// msb of SG_Result (bit 8-9)
	SG_RESULT = (SG_RESULT<<8);				// shift to correct Position
	SG_RESULT = (SG_RESULT | tmc2130_read_data[3]);	// get other bits (bit 0-7)
	Serial.print(" SG_RESULT: ");	// print SG_Result
	Serial.println(SG_RESULT);		// print SG_Result
     
}

// print data of received register
// use this after tmc2130_read();
void tmc2130_print_data()
{
	Serial.print("SPI_status: 0b");
	Serial.print(tmc2130_read_data[0], BIN);
	Serial.print(" RegisterAdress: 0x");
	Serial.print(tmc2130_read_data[5], HEX);  
	Serial.print("  24_31: 0x");
	Serial.print(tmc2130_read_data[1], HEX);
	Serial.print("  16_23: 0x");
	Serial.print(tmc2130_read_data[2], HEX); 
	Serial.print("  8_15: 0x");
	Serial.print(tmc2130_read_data[3], HEX);
	Serial.print("  0_7: 0x");
	Serial.println(tmc2130_read_data[4], HEX);

}


/*
//****************************************************
// Use this only for testing on standalone hardware!
// This functions working only on Microchip/Atmel/AVR
// based Arduino (Mega/Uno/Nano)
//****************************************************
// timer interrupt for generating step signal
void init_interrupt()
{
	// Set stepper interrupt
	cli();		//stop interrupts
	TCCR1A = 0;	// set entire TCCR1A register to 0
	TCCR1B = 0;	// same for TCCR1B
	TCNT1  = 0;	//initialize counter value to 0
	OCR1A = 3000;// = (16*10^6) / (1*1024) - 1 (must be <65536)
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS11 bits for 8 prescaler
	TCCR1B |= (1 << CS11);// | (1 << CS10);  
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);
	sei();//allow interrupts
}	

// ISR for toggle step pin of tmc
// works only with function "init interrupt()"

ISR(TIMER1_COMPA_vect)
{
    digitalWrite(TMC2130_STEP_PIN, LOW);
    digitalWrite(TMC2130_STEP_PIN, HIGH); 
}
*/
