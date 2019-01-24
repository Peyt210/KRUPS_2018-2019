#ifndef KRUPS_ALT_RADIO_H
#define KRUPS_ALT_RADIO_H

/*
 * KRUPS_Alt_Radio.h
 * Header file containing data and functions to be included in the KRUPS capsule
 * software and ground station software. Contains functions for both testing and
 * actual operation of the Texas Instruments CC1120 Evaluation modules for both
 * transmitting (capsule) and receiving (ground station).
 * 
 * Authors: James Michael Miller, Philip Lam
 * Organization: Kentucky Re-Entry Universal 
 * Payload System (KRUPS), University of Kentucky
 */

#include <SPI.h>

// Constants
#define fSCLK		10000000;	// 10MHz clock for SPI transactions

// SPI Settings - Send data with a specified SCLK freq., most sig. bit first and
// mode 0 (deals with clock position/polarity during transfer)
SPISettings commSettings(fSCLK, MSBFIRST, SPI_MODE0);

// ----------------------------------------
//        Pin Dedication
// ----------------------------------------
// Pin dedication for Chip Select (CS) pins. CS pins preferred but regular GPIO pins
// can be substituted. SPI pins can be reassigned with SPI.setMOSI(), SPI.setMISO etc.
// Teensy 3.5 pins default to:
//  Pin 13 (SCLK)
// 	Pin 11 (MOSI)
//  Pin 12 (MISO)
const int CSPin = 20;
// ---------End of Pin Dedication----------

// ----------------------------------------
//         Initialization Functions
//   (Used in both testing and operation)
// ----------------------------------------
void altRadioTXInit(){
	// Initialize CS pin as output
	pinMode(CSPin, OUTPUT);
	
}

void altRadioRXInit(){
	// Initialize CS pin as output
	pinMode(CSPin, OUTPUT);
	
}
// ---------End of Init Functions----------

#endif
