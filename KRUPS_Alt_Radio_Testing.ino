/*
 * Code for testing SPI reads/writes and basic communication
 * between two CC1120 Evaluation modules.
 * 
 * Author: James Michael Miller, Philip Lam
 * Organization: Kentucky Re-Entry Universal 
 * Payload System (KRUPS), University of Kentucky
 */

#include <Arduino.h>
#include <SPI.h>

// Send data with a specified SCLK freq., most sig. bit first and
// mode 0 (deals with clock position/polarity during transfer)
SPISettings commSettings(fSCLK, MSBFIRST, SPI_MODE0);

void setup(){
  // Initialize CS pins as outputs
  pinMode(transmitCSPin, OUTPUT);
  pinMode(receiveCSPin, OUTPUT);
  // Begin SPI
  SPI.begin();
  // Load registers of transmitting module with
  // appropriate values
  
  // Load registers of receiving modules with
  // appropriate values
  
}

void loop(){
  
}
