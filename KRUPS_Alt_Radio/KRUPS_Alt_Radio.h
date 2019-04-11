#ifndef KRUPS_ALT_RADIO_H
#define KRUPS_ALT_RADIO_H

/*
 * KRUPS_Alt_Radio.h
 * Header file containing data and functions to be included in the KRUPS capsule
 * software and ground station software. Contains functions for both testing and
 * actual operation of the Texas Instruments CC1120 Evaluation modules for both
 * transmitting (capsule) and receiving (ground station).
 * 
 * Authors: James Miller, Philip Lam, Peyton Moore
 * Organization: Kentucky Re-Entry Universal 
 * Payload System (KRUPS), University of Kentucky
 */

#include <SPI.h>

// Constants
#define fSCLK             6000000	// 6MHz clock for SPI transactions

// Command Strobe Register Addresses
#define SRES    0x30  // Touching this address will reset CC1120 chip
#define SRX     0x34  // Touching this address will send CC1120 into receive mode
#define STX     0x35  // Touching this address will send CC1120 into transmit mode
#define SIDLE   0x36  // Touching this address will send CC1120 back into idle
                      // state. Registers are configured in IDLE state.
#define SFRX    0x3A  // Touching this address will flush the receive FIFO buffer.
                      // Only use in IDLE or RX_FIFO_ERROR states
#define SFTX    0x3B  // Touching this address will flush the transmit FIFO buffer.
                      // Only use in IDLE or TX_FIFO_ERROR states
#define SNOP    0x3D  // No Operation. This address can be touched in order to receive a CC1120
                      // status byte without changing anything

// Register data
extern uint16_t CC1120FreqBlast[][2];
extern const int CC1120_FB_NUM_REGS;
extern uint16_t CC1120PacketTx[][2];
extern const int CC1120_PTx_NUM_REGS;
//extern uint16_t CC1120PacketRx[][2];
//extern const int CC1120_PRx_NUM_REGS;

// SPI Settings - Send data with a specified SCLK freq., most sig. bit first and
// mode 0 (deals with clock position/polarity during transfer)
SPISettings commSettings(fSCLK, MSBFIRST, SPI_MODE0);

// Pin dedication for Chip Select (CS) pin. Dedicated CS pin preferred but regular GPIO pins
// can be substituted. SPI pins can be reassigned with SPI.setMOSI(), SPI.setMISO etc.
// Teensy 3.5 pins default to:
//  Pin 13 (SCLK), Pin 11 (MOSI), Pin 12 (MISO)
const uint8_t CS_Radio = 15;
//const int GPIO_0 = 27;  // OPTIONAL
//const int GPIO_2 = 28;  // OPTIONAL

// ----------------------------------------
// CC1120 Radio Class
// ----------------------------------------
class altRadio{
  private:
    byte cmdByte;
    void regWrite(uint16_t address, uint16_t data);
    void extRegWrite(uint16_t address, uint16_t data);
    byte regRead(uint16_t address);
    byte extRegRead(uint16_t address);
    byte cmdStrobe(uint8_t cmdStrobeAddr);
  public:
    altRadio();
    void begin();
    void initFreqBlast();
    void initPacketTx();
    void transmitData(uint8_t data[128], uint8_t data_size);
    void beginFreqBlast();
    void endFreqBlast();
};
// ----------------------------------------

// Constructor for alternate radio object
altRadio::altRadio(){}

// Begin function for alternate radio object
void altRadio::begin(){
  // Set CS pin to high (idle), start SPI
  pinMode(CS_Radio, OUTPUT);
  digitalWrite(CS_Radio, HIGH);
  SPI.begin();
  delay(1); // Gives the CC1120 oscillator time to settle
}

// Initialization function for the frequency blast test.
// Writes appropriate registers using the CC1120FreqBlast.c
void altRadio::initFreqBlast(){
  cmdStrobe(SRES);  // Reset CC1120 to ensure all registers are defaulted
  cmdStrobe(SIDLE);
  for (int i = 0; i < CC1120_FB_NUM_REGS; i++){
    if ((CC1120FreqBlast[i][0] >> 8) == 0x2F){ // Check to see if in extended register space
      extRegWrite(CC1120FreqBlast[i][0], CC1120FreqBlast[i][1]);
    }
    else{
      regWrite(CC1120FreqBlast[i][0], CC1120FreqBlast[i][1]);
    }
  }
}

// Initialization function for packet transmitting
// Writes appropriate registers for typical Tx operation
void altRadio::initPacketTx(){
  cmdStrobe(SRES);  // Reset CC1120 to ensure all registers are defaulted
  cmdStrobe(SIDLE);
  for (int i = 0; i < CC1120_PTx_NUM_REGS; i++){
    if ((CC1120PacketTx[i][0] >> 8) == 0x2F){ // Check to see if in extended register space
      extRegWrite(CC1120PacketTx[i][0], CC1120PacketTx[i][1]);
    }
    else{
      regWrite(CC1120PacketTx[i][0], CC1120PacketTx[i][1]);
    }
  }
}

// Function that allows the user to transmit data.
void altRadio::transmitData(uint8_t data[127], uint8_t data_size){
  byte temp = cmdStrobe(SNOP);
  
  if ((temp & 0x70) == 0x70){  // If CC1120 is in Tx error state, flush Tx FIFO
    cmdStrobe(SFTX);
  }

  byte cmdByte = 0x7F; // Command byte to notify radio of impending Tx FIFO burst write

  SPI.beginTransaction(commSettings);
  digitalWrite(CS_Radio, LOW);
  while(digitalRead(12) == HIGH){}  // Wait for MISO to go low
  temp = SPI.transfer(cmdByte); // Send cmd byte with write bit and address
  SPI.transfer(data_size);
  for (int i = 0; i < data_size; i++){
    SPI.transfer(data[i]);
  }
  digitalWrite(CS_Radio, HIGH);
  SPI.endTransaction();

  cmdStrobe(STX);  // Send the data that was just loaded into the FIFO buffer
}

// After frequency blast is initialized, sends a command to CC1120
// to begin a continuous transmission for testing
void altRadio::beginFreqBlast(){
  cmdStrobe(STX); 
}

void altRadio::endFreqBlast(){
  cmdStrobe(SIDLE);
}

// Writes a single register on the CC1120 board
void altRadio::regWrite(uint16_t address, uint16_t data){
  cmdByte = (address & 0x3F);  // Put address in bottom 6 bits and set top two bits to 00
  byte temp;

  SPI.beginTransaction(commSettings);
  digitalWrite(CS_Radio, LOW);
  while(digitalRead(12) == HIGH){}  // Wait for MISO to go low
  temp = SPI.transfer(cmdByte); // Send cmd byte with write bit and address
  SPI.transfer((data & 0xFF)); // Send bottom 8 bits of data
  digitalWrite(CS_Radio, HIGH);
  SPI.endTransaction();
}

// Writes a single register in the extended register space on the CC1120 board
void altRadio::extRegWrite(uint16_t address, uint16_t data){
  cmdByte = 0x2F;  // First byte must contain write bit and extended address
  byte addr = (address & 0xFF);  // Grab bottom byte for regular address
  byte temp;

  SPI.beginTransaction(commSettings);
  digitalWrite(CS_Radio, LOW);
  while(digitalRead(12) == HIGH){}  // Wait for MISO to go low
  temp = SPI.transfer(cmdByte); // Send cmd byte with extended address, receive status (optional)
  SPI.transfer(addr); // Send regular address
  temp = SPI.transfer((data & 0xFF)); // Send bottom 8 bits of data
  SPI.endTransaction();
}

// Reads a single register on the CC1120 board
// Implemented for debugging
byte altRadio::regRead(uint16_t address){
  cmdByte = ((address & 0x3F) | 0x80);  // Put address in bottom 6 bits and set top two bits to 10
  byte temp, data;
  
  SPI.beginTransaction(commSettings);
  digitalWrite(CS_Radio, LOW);
  while(digitalRead(12) == HIGH){}  // Wait for MISO to go low
  temp = SPI.transfer(cmdByte); // Send cmd byte and receive status byte (optional)
  data = SPI.transfer(0);       // Receive data
  digitalWrite(CS_Radio, HIGH);
  SPI.endTransaction();
  
  return data;
}

// Reads a single register in the extended register space on the CC1120 board
// Implemented for debugging
byte altRadio::extRegRead(uint16_t address){
  cmdByte = 0xAF;  // First byte must contain read bit and extended address
  byte addr = (address & 0xFF);  // Grab bottom byte for regular address
  byte temp, data;

  SPI.beginTransaction(commSettings);
  digitalWrite(CS_Radio, LOW);
  while(digitalRead(12) == HIGH){}  // Wait for MISO to go low
  temp = SPI.transfer(cmdByte); // Send cmd byte with extended address, receive status (optional)
  SPI.transfer(addr); // Send regular address
  data = SPI.transfer(0); // Get data
  digitalWrite(CS_Radio, HIGH);
  SPI.endTransaction();
  
  return data;
}

// Touches a register in the CC1120 which changes the state of the CC1120
// (Unless the register is the NOP register)
byte altRadio::cmdStrobe(uint8_t cmdStrobeAddr){
  byte temp = 0x00;
  if (cmdStrobeAddr == SRES){
    SPI.beginTransaction(commSettings);
    digitalWrite(CS_Radio, LOW);
    while(digitalRead(12) == HIGH){}  // Wait for MISO to go low
    SPI.transfer(cmdStrobeAddr);
    while(digitalRead(12) == HIGH){}  // Wait for MISO to go low
    digitalWrite(CS_Radio, HIGH);
    SPI.endTransaction();
  }
  else{
    SPI.beginTransaction(commSettings);
    digitalWrite(CS_Radio, LOW);
    while(digitalRead(12) == HIGH){}  // Wait for MISO to go low
    temp = SPI.transfer(cmdStrobeAddr); // Touch register and get status byte
    digitalWrite(CS_Radio, HIGH);
    SPI.endTransaction();
  }

  return temp;
}

#endif
