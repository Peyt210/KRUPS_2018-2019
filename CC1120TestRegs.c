// Rf settings for CC1120 Test
// Created using RF Studio
// (Only affects specific registers. Rest are default values.)

#include <avr/pgmspace.h>

const uint32_t cc1120Regs[][] PROGMEM = {
//   Addr,   Value
    {0x0000, 0xB0},  // IOCFG3              GPIO3 IO Pin Configuration
    {0x0001, 0x06},  // IOCFG2              GPIO2 IO Pin Configuration
    {0x0002, 0xB0},  // IOCFG1              GPIO1 IO Pin Configuration
    {0x0003, 0x40},  // IOCFG0              GPIO0 IO Pin Configuration
    {0x0004, 0x55},  // SYNC3               Sync Word Configuration [31:24]
    {0x0005, 0x55},  // SYNC2               Sync Word Configuration [23:16]
    {0x0006, 0x7A},  // SYNC1               Sync Word Configuration [15:8]
    {0x0007, 0x0E},  // SYNC0               Sync Word Configuration [7:0]
    {0x0008, 0x0B},  // SYNC_CFG1           Sync Word Detection Configuration Reg. 1
    {0x0009, 0x0B},  // SYNC_CFG0           Sync Word Length Configuration Reg. 0
    {0x000A, 0x99},  // DEVIATION_M         Frequency Deviation Configuration
    {0x000B, 0x05},  // MODCFG_DEV_E        Modulation Format and Frequency Deviation Configur..
    {0x000C, 0x15},  // DCFILT_CFG          Digital DC Removal Configuration
    {0x000D, 0x18},  // PREAMBLE_CFG1       Preamble Length Configuration Reg. 1
    {0x000F, 0x3A},  // FREQ_IF_CFG         RX Mixer Frequency Configuration
    {0x0010, 0x00},  // IQIC                Digital Image Channel Compensation Configuration
    {0x0011, 0x02},  // CHAN_BW             Channel Filter Configuration
    {0x0013, 0x05},  // MDMCFG0             General Modem Parameter Configuration Reg. 0
    {0x0014, 0x99},  // SYMBOL_RATE2        Symbol Rate Configuration Exponent and Mantissa [1..
    {0x0015, 0x99},  // SYMBOL_RATE1        Symbol Rate Configuration Mantissa [15:8]
    {0x0016, 0x99},  // SYMBOL_RATE0        Symbol Rate Configuration Mantissa [7:0]
    {0x0017, 0x3C},  // AGC_REF             AGC Reference Level Configuration
    {0x0018, 0xEF},  // AGC_CS_THR          Carrier Sense Threshold Configuration
    {0x001C, 0xA9},  // AGC_CFG1            Automatic Gain Control Configuration Reg. 1
    {0x001D, 0xC0},  // AGC_CFG0            Automatic Gain Control Configuration Reg. 0
    {0x001E, 0x00},  // FIFO_CFG            FIFO Configuration
    {0x0021, 0x12},  // FS_CFG              Frequency Synthesizer Configuration
    {0x0028, 0x20},  // PKT_CFG0            Packet Configuration Reg. 0
    {0x002B, 0x4F},  // PA_CFG2             Power Amplifier Configuration Reg. 2
    {0x002D, 0x79},  // PA_CFG0             Power Amplifier Configuration Reg. 0
    {0x002E, 0xFF},  // PKT_LEN             Packet Length Configuration
    {0x2F00, 0x00},  // IF_MIX_CFG          IF Mix Configuration
    {0x2F02, 0x0A},  // TOC_CFG             Timing Offset Correction Configuration
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
    {0x2F92, 0x10},  // MODEM_STATUS1       Modem Status Reg. 1
};