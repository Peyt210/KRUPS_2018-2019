/*
KRUPS 2018-2019 Software
Sponsored by Dr. Alexandre Martin, University of Kentucky
Authors: Jordan Johnston, 
*/

// All the libraries used for entire project
#include <Arduino.h>
#include <SPI.h>


// Constants
#define power_on	0	// GPIO pin used for switching on the battery

void setup(){
  pinMode(power_on, OUTPUT);
  digitalWrite(power_on, HIGH);		// Set to output and write high to turn on battery
  
  
  
}

void loop(){
	
}
