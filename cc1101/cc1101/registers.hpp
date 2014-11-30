// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_CC1101_REGISTERS_HPP
#define XPCC_CC1101_REGISTERS_HPP

#include "bit_constants.hpp"

namespace xpcc
{
namespace radio
{

/*
 * CC1101 Registers
 *
 * This class contains all register definitions for the CC1101
 * low power sub 1 GHz RF transceiver from Texas Instruments.
 * http://www.ti.com/product/cc1101
 *
 * Please note: a lot of the CC11xx radios from TI have similar registers,
 * thus this class could be used to address e.g. the CC115L Transmitter
 * as well.
 */
class
CC1101Registers
{
public:

	enum class
	ConfigurationRegister : uint8_t
	{
		IOCFG2   = 0x00,	///< GDO2 output pin configuration
		IOCFG1   = 0x01,	///< GDO1 output pin configuration
		IOCFG0   = 0x02,	///< GDO0 output pin configuration
		FIFOTHR  = 0x03,	///< Rx FIFO and TX FIFO thresholds
		SYNC1    = 0x04,	///< sync word, high byte
		SYNC0    = 0x05,	///< sync word, low byte
		PKTLEN   = 0x06,	///< packet length
		PKTCTRL1 = 0x07,	///< packet automation control
		PKTCTRL0 = 0x08,	///< packet automation control
		ADDR     = 0x09,	///< device address
		CHANNR   = 0x0a,	///< channel number
		FSCTRL1  = 0x0b,	///< frequency synthesizer control
		FSCTRL0  = 0x0c,	///< frequency synthesizer control
		FREQ2    = 0x0d,	///< frequency control word, high byte
		FREQ1    = 0x0e,	///< frequency control word, middle byte
		FREQ0    = 0x0f,	///< frequency control word, low byte
		MDMCFG4  = 0x10,	///< modem configuration
		MDMCFG3  = 0x11,	///< modem configuration
		MDMCFG2  = 0x12,	///< modem configuration
		MDMCFG1  = 0x13,	///< modem configuration
		MDMCFG0  = 0x14,	///< modem configuration
		DEVIATN  = 0x15,	///< modem deviation settings
		MCSM2    = 0x16,	///< main radio control state machine configuration
		MCSM1    = 0x17,	///< main radio control state machine configuration
		MCSM0    = 0x18,	///< main radio control state machine configuration
		FOCCFG   = 0x19,	///< frequency offset compensation configuration
		BSCFG    = 0x1a,	///< bit synchronization configuration
		AGCTRL2  = 0x1b,	///< AGC control
		AGCTRL1  = 0x1c,	///< AGC control
		AGCTRL0  = 0x1d,	///< AGC control
		WOREVT1  = 0x1e,	///< high byte event 0 timeout
		WOREVT0  = 0x1f,	///< low byte event 0 timeout
		WORCTRL  = 0x20,	///< wake on radio control
		FREND1   = 0x21,	///< front end RX configuration
		FREND0   = 0x22,	///< front end TX configuration
		FSCAL3   = 0x23,	///< frequency synthesizer calibration
		FSCAL2   = 0x24,	///< frequency synthesizer calibration
		FSCAL1   = 0x25,	///< frequency synthesizer calibration
		FSCAL0   = 0x26,	///< frequency synthesizer calibration
		RCCTRL1  = 0x27,	///< RC oscillator configuration
		RCCTRL0  = 0x28,	///< RC oscillator configuration
		FSTEST   = 0x29,	///< frequency synthesizer calibration control
		PTEST    = 0x2a,	///< production test
		AGCTEST  = 0x2b,	///< AGC test
		TEST2    = 0x2c,	///< various test settings
		TEST1    = 0x2d,	///< various test settings
		TEST0    = 0x2e,	///< various test settings
	};


	enum class
	StatusRegister : uint8_t
	{
		PARTNUM        = 0x30,	///< part number for cc1101
		VERSION        = 0x31,	///< current version number
		FREQEST        = 0x32,	///< frequency offset estimate
		LQI            = 0x33,	///< demodulator estimate for link quality
		RSSI           = 0x34,	///< received signal strength indication
		MARCSTATE      = 0x35,	///< control state machine
		WORTIME1       = 0x36,	///< high byte of WOR timer
		WORTIME0       = 0x37,	///< low byte of WOR timer
		PKTSTATUS      = 0x38,	///< current GDOx status and packet status
		VCO_VC_DAC     = 0x39,	///< current settings from PLL calibration module
		TXBYTES        = 0x3a,	///< underflow and number of bytes in the TX FIFO
		RXBYTES        = 0x3b,	///< overflow and number of bytes in the RX FIFO
		RCCTRL1_STATUS = 0x3c,	///< last RC oscillator calibration result
		RCCTRL0_STATUS = 0x3d,	///< last RC oscillator calibration result
	};

	enum class
	IOCFG2 : uint8_t
	{
		INV   = Bit6,	///< invert output
		CFG   = Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
		CFG_5 = Bit5,
		CFG_4 = Bit4,
		CFG_3 = Bit3,
		CFG_2 = Bit2,
		CFG_1 = Bit1,
		CFG_0 = Bit0,
	};

	enum class
	IOCFG1 : uint8_t
	{
		INV   = Bit6,	///< invert output
		CFG   = Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
		CFG_5 = Bit5,
		CFG_4 = Bit4,
		CFG_3 = Bit3,
		CFG_2 = Bit2,
		CFG_1 = Bit1,
		CFG_0 = Bit0,
	};

	enum class
	IOCFG0 : uint8_t
	{
		/// enable analog temperature sensor, set all other bits to zero when using it
		TEMP_SENSOR_ENABLE = Bit7,
		INV   = Bit6,	///< invert output
		CFG   = Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
		CFG_5 = Bit5,
		CFG_4 = Bit4,
		CFG_3 = Bit3,
		CFG_2 = Bit2,
		CFG_1 = Bit1,
		CFG_0 = Bit0,
	};

	enum class
	FIFOTHR : uint8_t
	{
		ADC_RETENTION = Bit6,
		/// rx attenuation
		CLOSE_IN_RX   = Bit5 | Bit4,
		CLOSE_IN_RX_1 = Bit5,	///< rx attenuation
		CLOSE_IN_RX_0 = Bit4,	///< rx attenuation
		/// TX and RX FIFO threshold
		FIFO_THR      = Bit3 | Bit2 | Bit1 | Bit0,
		FIFO_THR_3    = Bit3,	///< TX and RX FIFO threshold
		FIFO_THR_2    = Bit2,	///< TX and RX FIFO threshold
		FIFO_THR_1    = Bit1,	///< TX and RX FIFO threshold
		FIFO_THR_0    = Bit0,	///< TX and RX FIFO threshold
	};

	enum class
	PKTCTRL1 : uint8_t
	{
		/// preamble quality estimator threshold
		PQT           = Bit7 | Bit6 | Bit5,
		PQT_2         = Bit7,
		PQT_1         = Bit6,
		PQT_0         = Bit5,
		/// enable automatic flush of RX FIFO when CRC is not OK
		CRC_AUTOFLUSH = Bit3,
		/// when enabled, two status bytes will be appended to the payload of the packet
		APPEND_STATUS = Bit2,
		/// controls address check configuration of received packages
		ADR_CHK       = Bit1 | Bit0,
		ADR_CHK_1     = Bit1,
		ADR_CHK_0     = Bit0,
	};

	enum class
	PKTCTRL0 : uint8_t
	{
		WHITE_DATA      = Bit6,	///< turn data whitening on / off
		/// format of RX and TX Data
		PKT_FORMAT      = Bit5 | Bit4,
		PKT_FORMAT_1    = Bit5,
		PKT_FORMAT_0    = Bit4,
		CRC_EN          = Bit2,	///< enable/disable CRC calculation/check
		/// packet length
		LENGTH_CONFIG   = Bit1 | Bit0,
		LENGTH_CONFIG_1 = Bit1,
		LENGTH_CONFIG_0 = Bit0,
	};

	enum class
	FSCTRL1 : uint8_t
	{
		/// the desired IF frequency to employ in RX
		FREQ_IF = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
	};

	enum class
	MDMCFG4 : uint8_t
	{
		/// ??
		CHANBW_E = Bit7 | Bit6,
		/// the decimation ratio for the delta-sigma ADC input stream and this the channel bandwidth
		CHANBW_M = Bit5 | Bit4,
		/// the exponent of the user specific symbol rate
		DRATE_E  = Bit3, Bit2, Bit1, Bit0,
	};

	enum class
	MDMCFG2 : uint8_t
	{
		/// disable digital DC blocking filter before demodulator by setting to 1
		DEM_DCFILT_OFF = Bit7,
		/// the modulation format of the radio signal
		MOD_FORMAT     = Bit6 | Bit5 | Bit4,
		MOD_FORMAT_2   = Bit6,
		MOD_FORMAT_1   = Bit5,
		MOD_FORMAT_0   = Bit4,
		/// enable Manchester encoding/decoding
		MANCHESTER_EN  = Bit3,
		/// combined sync-word qualifier mode
		SYNC_MODE      = Bit2 | Bit1 | Bit0,
		SYNC_MODE_2    = Bit2,
		SYNC_MODE_1    = Bit1,
		SYNC_MODE_0    = Bit0,
	};

	enum class
	MDMCFG1 : uint8_t
	{
		/// enable forward error correction
		FEC_EN       = Bit7,
		/// minimum number of preamble bytes to be transmitted
		NUM_PREAMBLE = Bit6 | Bit5 | Bit4,
		/// 2 bit exponent of channel spacing
		CHANSPC_EN   = Bit1 | Bit0,
	};


	enum class
	DEVIATN : uint8_t
	{
		/// deviation exponent
		DEVIATION_E = Bit6 | Bit5 | Bit4,
		/// nominal frequency deviation (effects depend on RX/TX and modulation)
		DEVIATION_M = Bit2 | Bit1 | Bit0,
	};

	enum class
	MCSM2 : uint8_t
	{
		/// direct RX termination based on RSSI measurements (carrier sense)
		RX_TIME_RSSI = Bit4,
		/// specifies what happens when RX_TIMER expires
		RX_TIME_QUAL = Bit3,
		/// timeout for sync word search in RX for both WOR mode and normal RX operation
		RX_TIME      = Bit2 | Bit1 | Bit0,
		RX_TIME_2    = Bit2,
		RX_TIME_1    = Bit1,
		RX_TIME_0    = Bit0,
	};

	enum class
	MCSM1 : uint8_t
	{
		/// selects CCA_MODE; reflected in CCA signal
		CCA_MODE   = Bit5 | Bit4,
		CCA_MODE_1 = Bit5,
		CCA_MODE_0 = Bit4,
		/// select what should happen when a packet has been received
		RXOFF_MODE   = Bit3 | Bit2,
		RXOFF_MODE_1 = Bit3,
		RXOFF_MODE_0 = Bit2,
		/// select what should happen when a packet has been sent
		TXOFF_MODE   = Bit1 | Bit0,
		TXOFF_MODE_1 = Bit1,
		TXOFF_MODE_0 = Bit0,
	};

	enum class
	MCSM0 : uint8_t
	{
		/// automatically calibrate when going to RX or TX, or back to IDLE
		FS_AUTOCALL   = Bit5 | Bit4,
		FS_AUTOCALL_1 = Bit5,
		FS_AUTOCALL_0 = Bit4,
		/// programs the number of times the six-bit ripple counter must expire 
		PO_TIMEOUT    = Bit3 | Bit2,
		PO_TIMEOUT_1  = Bit3,
		PO_TIMEOUT_0  = Bit2,
		/// enable the pin radio control option
		PIN_CTRL_EN   = Bit1,
		/// force the XOSC to stay on in the SLEEP state
		XOSC_FORCE_ON = Bit0,
	};

	enum class
	FOCCFG : uint8_t
	{
		/// if set the demodulator freezes the frequency offset compensation
		FOC_BS_CS_GATE = Bit5,
		/// the frequency compensation loop gain to be used before a sync word is detected
		FOC_PRE_K      = Bit4 | Bit3,
		FOC_PRE_K_1    = Bit4,
		FOC_PRE_K_0    = Bit3,
		/// the frequency compensation loop gain to be used after a sync word is detected
		FOC_POST_K     = Bit2,
		/// the saturation point for the frequency offset compensation algorithm
		FOC_LIMIT      = Bit1 | Bit0,
		FOC_LIMIT_1    = Bit1,
		FOC_LIMIT_0    = Bit0,
	};

	enum class
	BSCFG : uint8_t
	{
		/// the clock recovery feedback loop integrral gain, used before a sync word is detected
		BS_PRE_K    = Bit7 | Bit6,
		BS_PRE_K_1  = Bit7,
		BS_PRE_K_0  = Bit6,
		/// the clock recovery feedback loop proportional gain, used before a sync word is detected
		BS_PRE_KP   = Bit5 | Bit4,
		BS_PRE_KP_1 = Bit5,
		BS_PRE_KP_0 = Bit4,
		/// the clock recovery feedback loop integrral gain, used after a sync word is detected
		BS_POST_KI = Bit3,
		/// the clock recovery feedback loop proportional gain, used after a sync word is detected
		BS_POST_KP = Bit2,
		/// the saturation point for the data rate offset compensation algorithm
		BS_LIMIT   = Bit1 | Bit0,
		BS_LIMIT_1 = Bit1,
		BS_LIMIT_0 = Bit0,
	};

	enum class
	AGCTRL2 : uint8_t
	{
		/// reduces the maximuym allowable DVGA gain
		MAX_DVA_GAIN   = Bit7 | Bit6,
		MAX_DVA_GAIN_1 = Bit7,
		MAX_DVA_GAIN_0 = Bit6,
		/// set the macimum allowable LNA + LNA 2 gain relative to the maximum possible gain
		MAX_LNA_GAIN   = Bit5 | Bit4 | Bit3,
		MAX_LNA_GAIN_2 = Bit5,
		MAX_LNA_GAIN_1 = Bit4,
		MAX_LNA_GAIN_0 = Bit3,
		/// these bits set the target value for the averaged amplitude from the digital channel filter
		MAGN_TARGET    = Bit2 | Bit1 | Bit0,
		MAGN_TARGET_2  = Bit2,
		MAGN_TARGET_1  = Bit1,
		MAGN_TARGET_0  = Bit0,
	};

	enum class
	AGCTRL1 : uint8_t
	{
		/// selects between different strategies for LNA and LNA 2 gain
		AGC_LNA_PRIORITY        = Bit6,
		/// sets the relative change threshold for asserting carrier sense
		CARRIER_SENSE_REL_THR   = Bit5 | Bit4,
		CARRIER_SENSE_REL_THR_1 = Bit5,
		CARRIER_SENSE_REL_THR_0 = Bit4,
		/// sets the absolute RSSI threshold for asserting carier sense
		CARRIER_SENSE_ABS_THR   = Bit3 | Bit2 | Bit1 | Bit0,
		CARRIER_SENSE_ABS_THR_3 = Bit3,
		CARRIER_SENSE_ABS_THR_2 = Bit2,
		CARRIER_SENSE_ABS_THR_1 = Bit1,
		CARRIER_SENSE_ABS_THR_0 = Bit0,
	};

	enum class
	AGCTRL0 : uint8_t
	{
		/// sets the level of hysteresis on the magnitude deviation
		HYST_LEVEL      = Bit7 | Bit6,
		HYST_LEVEL_1    = Bit7,
		HYST_LEVEL_0    = Bit6,
		/// sets the number of channel filter samples
		WAIT_TIME       = Bit5 | Bit4,
		WAIT_TIME_1     = Bit5,
		WAIT_TIME_0     = Bit4,
		/// controls when the AGC gain should be frozen
		AGC_FREEZE      = Bit3 | Bit2,
		AGC_FREEZE_1    = Bit3,
		AGC_FREEZE_0    = Bit2,
		/// channel filter samples / decision boundary
		FILTER_LENGTH   = Bit1 | Bit0,
		FILTER_LENGTH_1 = Bit1,
		FILTER_LENGTH_0 = Bit0,
	};

	enum class
	WORCTRL : uint8_t
	{
		/// power down signal to RC oscialltor
		RC_PD     = Bit7,
		/// timeout setting from register block
		EVENT     = Bit6 | Bit5 | Bit4,
		/// enables or disables the RC oscialltor calibration
		RC_CAL    = Bit3,
		/// controls the event 0 resolution as well as maximum timeout
		WOR_RES   = Bit1 | Bit0,
	};

	enum class
	FREND1 : uint8_t
	{
		/// adjusts front end LNA PTAT current output
		LNA_CURRENT            = Bit7 | Bit6,
		LNA_CURRENT_1          = Bit7,
		LNA_CURRENT_0          = Bit6,
		/// adjusts front end PTAT outputs
		LNA2MIX_CURRENT        = Bit5 | Bit4,
		LNA2MIX_CURRENT_1      = Bit5,
		LNA2MIX_CURRENT_0      = Bit4,
		/// adjusts current in RX LO buffer
		LODIV_BUF_CURRENT_RX   = Bit3 | Bit2,
		LODIV_BUF_CURRENT_RX_1 = Bit3,
		LODIV_BUF_CURRENT_RX_0 = Bit2,
		/// adjusts current in mixer
		MIX_CURRENT            = Bit1 | Bit0,
		MIX_CURRENT_1          = Bit1,
		MIX_CURRENT_0          = Bit0,
	};

	enum class
	FREND2 : uint8_t
	{
		/// adjusts current TX LO buffer
		LODIV_BUF_CURRENT_TX   = Bit5 | Bit4,
		LODIV_BUF_CURRENT_TX_1 = Bit5,
		LODIV_BUF_CURRENT_TX_0 = Bit4,
		/// selects PA power setting (index to the PATABLE)
		PA_POWER               = Bit2 | Bit1 | Bit0,
		PA_POWER_2             = Bit2,
		PA_POWER_1             = Bit1,
		PA_POWER_0             = Bit0,
	};

	enum class
	FSCAL3 : uint8_t
	{
		FSCAL3_7          = Bit7,
		FSCAL3_6          = Bit6,
		// disable charge pump calibration stage when 0
		CHP_CURR_CAL_EN   = Bit5 | Bit4,
		FSCAL3_3          = Bit3,
		FSCAL3_2          = Bit2,
		FSCAL3_1          = Bit1,
		FSCAL3_0          = Bit0,
	};

	enum class
	FSCAL2 : uint8_t
	{
		/// chose high (1) or low (0) VCO
		VCO_CORE_H_EN = Bit5,
		/// frequency synthesizer calibration results
		FSCAL2        = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
	};

	enum class
	TEST0 : uint8_t
	{
		// Bit7:Bit2 magic values from TI GUI
		/// enable VCO selection calibration stage
		VCO_SEL_CAL_EN = Bit1,
		// Bit0 magic value from TI GUI
	};

	enum class
	MARCSTATE : uint8_t
	{
		/// main radio control FSM state
		MARC_STATE   = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
		MARC_STATE_4 = Bit4,
		MARC_STATE_3 = Bit3,
		MARC_STATE_2 = Bit2,
		MARC_STATE_1 = Bit1,
		MARC_STATE_0 = Bit0,
	};

	enum class
	PKTSTATUS : uint8_t
	{
		CRC_OK      = Bit7,	///< the last CRC comparison matched
		CS          = Bit6,	///< carrier sense
		PQT_REACHED = Bit5,	///< preamble quality reached
		CCA         = Bit4,	///< channel is clear
		SFD         = Bit3,	///< sync word found
		GDO2        = Bit2,	///< current GDO2 value
		GDO0        = Bit0,	///< current GDO0 value
	};

	enum class
	TXBYTES : uint8_t
	{
		TXFIFO_UNDERFLOW = Bit7,
		NUM_TXBYTES      = Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
	};

	enum class
	RXBYTES : uint8_t
	{
		RXFIFO_OVERFLOW = Bit7,
		NUM_RXBYTES     = Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
	};

};	// class CC1101Registers
}	// namespace radio
}	// namespace cc1101


#endif	// XPCC_CC1101_REGISTERS_HPP