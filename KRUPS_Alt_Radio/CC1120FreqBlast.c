// Rf settings for CC1120 Frequency Blast test
// Created using RF Studio
// (Only affects specific registers. Rest are default values.)

#include <avr/pgmspace.h>

const uint16_t CC1120FreqBlast[][2] PROGMEM = {
// Addr,   Value
  {0x0000, 0xB0},  // IOCFG3               GPIO3 IO Pin Configuration
  {0x0001, 0x08},  // IOCFG2               GPIO2 IO Pin Configuration
  {0x0002, 0xB0},  // IOCFG1               GPIO1 IO Pin Configuration
  {0x0003, 0x09},  // IOCFG0               GPIO0 IO Pin Configuration
  {0x0008, 0x0B},  // SYNC_CFG1            Sync Word Detection Configuration Reg. 1
  {0x000C, 0x1C},  // DCFILT_CFG           Digital DC Removal Configuration
  {0x000D, 0x00},  // PREAMBLE_CFG1        Preamble Length Configuration Reg. 1
  {0x0010, 0xC6},  // IQIC                 Digital Image Channel Compensation Configuration
  {0x0011, 0x08},  // CHAN_BW              Channel Filter Configuration
  {0x0012, 0x06},  // MDMCFG1              General Modem Parameter Configuration Reg. 1
  {0x0013, 0x05},  // MDMCFG0              General Modem Parameter Configuration Reg. 0
  {0x0017, 0x20},  // AGC_REF              AGC Reference Level Configuration
  {0x0018, 0x19},  // AGC_CS_THR           Carrier Sense Threshold Configuration
  {0x001C, 0xA9},  // AGC_CFG1             Automatic Gain Control Configuration Reg. 1
  {0x001E, 0x00},  // FIFO_CFG             FIFO Configuration
  {0x0021, 0x12},  // FS_CFG               Frequency Synthesizer Configuration
  {0x0026, 0x05},  // PKT_CFG2             Packet Configuration Reg. 2
  {0x0027, 0x00},  // PKT_CFG1             Packet Configuration Reg. 1
  {0x0028, 0x20},  // PKT_CFG0             Packet Configuration Reg. 0
  {0x002B, 0x44},  // PA_CFG2              Power Amplifier Configuration Reg. 2
  {0x2F00, 0x00},  // IF_MIX_CFG           IF Mix Configuration
  {0x2F01, 0x22},  // FREQOFF_CFG          Frequency Offset Correction Configuration
  {0x2F0C, 0x72},  // FREQ2                Frequency Configuration [23:16]
  {0x2F0D, 0x60},  // FREQ1                Frequency Configuration [15:8]
  {0x2F12, 0x00},  // FS_DIG1              Frequency Synthesizer Digital Reg. 1
  {0x2F13, 0x5F},  // FS_DIG0              Frequency Synthesizer Digital Reg. 0
  {0x2F16, 0x40},  // FS_CAL1              Frequency Synthesizer Calibration Reg. 1
  {0x2F17, 0x0E},  // FS_CAL0              Frequency Synthesizer Calibration Reg. 0
  {0x2F18, 0x27},  // FS_CHP               Frequency Synthesizer Charge Pump Configuration
  {0x2F19, 0x03},  // FS_DIVTWO            Frequency Synthesizer Divide by 2
  {0x2F1B, 0x33},  // FS_DSM0              FS Digital Synthesizer Module Configuration Reg. 0
  {0x2F1D, 0x17},  // FS_DVC0              Frequency Synthesizer Divider Chain Configuration ..
  {0x2F1F, 0x50},  // FS_PFD               Frequency Synthesizer Phase Frequency Detector Con..
  {0x2F20, 0x6E},  // FS_PRE               Frequency Synthesizer Prescaler Configuration
  {0x2F21, 0x14},  // FS_REG_DIV_CML       Frequency Synthesizer Divider Regulator Configurat..
  {0x2F22, 0xAC},  // FS_SPARE             Frequency Synthesizer Spare
  {0x2F23, 0x13},  // FS_VCO4              FS Voltage Controlled Oscillator Configuration Reg..
  {0x2F25, 0x64},  // FS_VCO2              FS Voltage Controlled Oscillator Configuration Reg..
  {0x2F26, 0xAC},  // FS_VCO1              FS Voltage Controlled Oscillator Configuration Reg..
  {0x2F27, 0xB4},  // FS_VCO0              FS Voltage Controlled Oscillator Configuration Reg..
  {0x2F32, 0x0E},  // XOSC5                Crystal Oscillator Configuration Reg. 5
  {0x2F36, 0x03},  // XOSC1                Crystal Oscillator Configuration Reg. 1
  {0x2F64, 0x01},  // WOR_TIME1            eWOR Timer Counter Value MSB
  {0x2F65, 0x46},  // WOR_TIME0            eWOR Timer Counter Value LSB
  {0x2F66, 0x01},  // WOR_CAPTURE1         eWOR Timer Capture Value MSB
  {0x2F67, 0x46},  // WOR_CAPTURE0         eWOR Timer Capture Value LSB
  {0x2F69, 0xE9},  // DCFILTOFFSET_I1      DC Filter Offset I MSB
  {0x2F6A, 0x36},  // DCFILTOFFSET_I0      DC Filter Offset I LSB
  {0x2F6B, 0x0F},  // DCFILTOFFSET_Q1      DC Filter Offset Q MSB
  {0x2F6C, 0x54},  // DCFILTOFFSET_Q0      DC Filter Offset Q LSB
  {0x2F6D, 0x0B},  // IQIE_I1              IQ Imbalance Value I MSB
  {0x2F6E, 0xAF},  // IQIE_I0              IQ Imbalance Value I LSB
  {0x2F6F, 0xF9},  // IQIE_Q1              IQ Imbalance Value Q MSB
  {0x2F70, 0xF3},  // IQIE_Q0              IQ Imbalance Value Q LSB
  {0x2F71, 0xF3},  // RSSI1                Received Signal Strength Indicator Reg. 1
  {0x2F72, 0x43},  // RSSI0                Received Signal Strength Indicator Reg.0
  {0x2F74, 0x16},  // LQI_VAL              Link Quality Indicator Value
  {0x2F75, 0xFA},  // PQT_SYNC_ERR         Preamble and Sync Word Error
  {0x2F76, 0x8F},  // DEM_STATUS           Demodulator Status
  {0x2F78, 0x08},  // FREQOFF_EST0         Frequency Offset Estimate LSB
  {0x2F79, 0x27},  // AGC_GAIN3            Automatic Gain Control Reg. 3
  {0x2F83, 0x89},  // MAGN0                Signal Magnitude after CORDIC [7:0]
  {0x2F85, 0xDD},  // ANG0                 Signal Angular after CORDIC [7:0]
  {0x2F88, 0x20},  // CHFILT_I0            Channel Filter Data Real Part [7:0]
  {0x2F8B, 0x5F},  // CHFILT_Q0            Channel Filter Data Imaginary Part [7:0]
  {0x2F8F, 0x48},  // PARTNUMBER           Part Number
  {0x2F90, 0x23},  // PARTVERSION          Part Revision
  {0x2F91, 0x18},  // SERIAL_STATUS        Serial Status
  {0x2F92, 0x91}   // MODEM_STATUS1
};

const int CC1120_FB_NUM_REGS = (sizeof(CC1120FreqBlast)/sizeof(CC1120FreqBlast[0]));
