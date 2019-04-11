// Rf settings for CC1120 Packet Transmission
// Created using RF Studio
// (Only affects specific registers. Rest are default values.)

// Note: PA_CFG2 set to 0x44 to transmit at 1W with amplifier

#include <avr/pgmspace.h>

const uint16_t CC1120PacketTx[][2] PROGMEM = {
// Addr,   Value
   {0x0000, 0xB0},  // IOCFG3              GPIO3 IO Pin Configuration
   {0x0001, 0x06},  // IOCFG2              GPIO2 IO Pin Configuration
   {0x0002, 0xB0},  // IOCFG1              GPIO1 IO Pin Configuration
   {0x0003, 0x40},  // IOCFG0              GPIO0 IO Pin Configuration
   {0x0008, 0x0B},  // SYNC_CFG1           Sync Word Detection Configuration Reg. 1
   {0x000C, 0x1C},  // DCFILT_CFG          Digital DC Removal Configuration
   {0x000D, 0x18},  // PREAMBLE_CFG1       Preamble Length Configuration Reg. 1
   {0x0010, 0xC6},  // IQIC                Digital Image Channel Compensation Configuration
   {0x0011, 0x08},  // CHAN_BW             Channel Filter Configuration
   {0x0013, 0x05},  // MDMCFG0             General Modem Parameter Configuration Reg. 0
   {0x0017, 0x20},  // AGC_REF             AGC Reference Level Configuration
   {0x0018, 0x19},  // AGC_CS_THR          Carrier Sense Threshold Configuration
   {0x001C, 0xA9},  // AGC_CFG1            Automatic Gain Control Configuration Reg. 1
   {0x001D, 0xCF},  // AGC_CFG0            Automatic Gain Control Configuration Reg. 0
   {0x001E, 0x00},  // FIFO_CFG            FIFO Configuration
   {0x0021, 0x12},  // FS_CFG              Frequency Synthesizer Configuration
   {0x0028, 0x20},  // PKT_CFG0            Packet Configuration Reg. 0
   {0x002E, 0xFF},  // PKT_LEN             Packet Length Configuration
   {0x2F00, 0x00},  // IF_MIX_CFG          IF Mix Configuration
   {0x2F01, 0x22},  // FREQOFF_CFG         Frequency Offset Correction Configuration
   {0x2F0C, 0x72},  // FREQ2               Frequency Configuration [23:16]
   {0x2F0D, 0x60},  // FREQ1               Frequency Configuration [15:8]
   {0x2F12, 0x00},  // FS_DIG1             Frequency Synthesizer Digital Reg. 1
   {0x2F13, 0x5F},  // FS_DIG0             Frequency Synthesizer Digital Reg. 0
   {0x2F16, 0x40},  // FS_CAL1             Frequency Synthesizer Calibration Reg. 1
   {0x2F17, 0x0E},  // FS_CAL0             Frequency Synthesizer Calibration Reg. 0
   {0x2F19, 0x03},  // FS_DIVTWO           Frequency Synthesizer Divide by 2
   {0x2F1B, 0x33},  // FS_DSM0             FS Digital Synthesizer Module Configuration Reg. 0
   {0x2F1D, 0x17},  // FS_DVC0             Frequency Synthesizer Divider Chain Configuration ..
   {0x2F1F, 0x50},  // FS_PFD              Frequency Synthesizer Phase Frequency Detector Con..
   {0x2F20, 0x6E},  // FS_PRE              Frequency Synthesizer Prescaler Configuration
   {0x2F21, 0x14},  // FS_REG_DIV_CML      Frequency Synthesizer Divider Regulator Configurat..
   {0x2F22, 0xAC},  // FS_SPARE            Frequency Synthesizer Spare
   {0x2F27, 0xB4},  // FS_VCO0             FS Voltage Controlled Oscillator Configuration Reg..
   {0x2F32, 0x0E},  // XOSC5               Crystal Oscillator Configuration Reg. 5
   {0x2F36, 0x03},  // XOSC1               Crystal Oscillator Configuration Reg. 1
   {0x2F8F, 0x48},  // PARTNUMBER          Part Number
   {0x2F90, 0x23},  // PARTVERSION         Part Revision
   {0x2F92, 0x10}  // MODEM_STATUS1       Modem Status Reg. 1
};

const int CC1120_PTx_NUM_REGS = (sizeof(CC1120PacketTx)/sizeof(CC1120PacketTx[0]));
