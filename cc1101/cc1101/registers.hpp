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
#include <type_traits>

namespace xpcc
{
namespace radio
{

/**
 * CC1101 Registers
 *
 * This class contains all register definitions for the CC1101
 * low power sub 1 GHz RF transceiver from Texas Instruments.
 * http://www.ti.com/product/cc1101
 *
 * Please note: a lot of the CC11xx radios from TI have similar registers,
 * thus this class could be used to address e.g. the CC115L Transmitter
 * as well.
 *
 * @ingroup radio
 * @author  eKiwi <electron.kiwi@gmail.com>
 */
class
CC1101Registers
{
public:

	enum class
	TransferMode : uint8_t
	{
		WriteSingleByte = 0x00,
		WriteBurst      = 0x40,
		ReadSingleByte  = 0x80,
		ReadBurst       = 0xc0,
	};

	enum class
	Command : uint8_t
	{
		/// reset chip
		SRES    = 0x30,
		/// enable and calibrate frequency synthesizer
		SFSTXON = 0x31,
		/// turn off crystal oscillator
		SXOFF   = 0x32,
		/// calibrate frequency synthesizer and turn it off
		SCAL    = 0x33,
		/// enable RX
		SRX     = 0x34,
		/// in IDLE state: enable TX
		STX     = 0x35,
		/// exit RX/TX turn off frequency synthesizer and exit wake-on-radio mode if applicable
		SIDLE   = 0x36,
		/// start automatic RX polling sequence (wake-on-radio)
		SWOR    = 0x38,
		/// enter power down mode when CSn goes high
		SPWD    = 0x39,
		/// flush the RX FIFO buffer
		SFRX    = 0x3a,
		/// flush the TX FIFO buffer
		SFTX    = 0x3b,
		/// reset real time clock to Event1 value
		SWORRST = 0x3c,
		/// no operation, may be used to access to the chip status byte
		SNOP    = 0x3d,
	};

	enum class
	Register : uint8_t
	{
		// Configuration Registers
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
		// Status Register, read only
		PARTNUM        = 0x30 | 0xc0,	///< part number for cc1101
		VERSION        = 0x31 | 0xc0,	///< current version number
		FREQEST        = 0x32 | 0xc0,	///< frequency offset estimate
		LQI            = 0x33 | 0xc0,	///< demodulator estimate for link quality
		RSSI           = 0x34 | 0xc0,	///< received signal strength indication
		MARCSTATE      = 0x35 | 0xc0,	///< control state machine
		WORTIME1       = 0x36 | 0xc0,	///< high byte of WOR timer
		WORTIME0       = 0x37 | 0xc0,	///< low byte of WOR timer
		PKTSTATUS      = 0x38 | 0xc0,	///< current GDOx status and packet status
		VCO_VC_DAC     = 0x39 | 0xc0,	///< current settings from PLL calibration module
		TXBYTES        = 0x3a | 0xc0,	///< underflow and number of bytes in the TX FIFO
		RXBYTES        = 0x3b | 0xc0,	///< overflow and number of bytes in the RX FIFO
		RCCTRL1_STATUS = 0x3c | 0xc0,	///< last RC oscillator calibration result
		RCCTRL0_STATUS = 0x3d | 0xc0,	///< last RC oscillator calibration result
	};

	/// Indicates whether the ouput should be inverted or not.
	///
	/// Part of the IOCFG[0:2] registers (datasheet page 71)
	enum class
	GdoInverted : uint8_t
	{
		No  = 0,
		Yes = Bit6,
	};

	/// GDOx Signal Selection
	///
	/// See CC1101 datasheet page 62, table 41 for more information
	/// Part of the IOCFG[0:2] registers (datasheet page 71)
	enum class
	GdoSignalSelection : uint8_t
	{
		/// Indicates whether Rx Fifo is filled at/above (`1`)
		/// or below (`0`) Fifo threshold.
		RxFifoThreshold     = 0x00,
		/// Set when Rx Fifo is filled at/above threshold or end of packet,
		/// reset when Rx Fifo is empty.
		RxFifoNotEmpty      = 0x01,
		/// Indicates whether Tx Fifo is filled at/above (`1`)
		/// or below (`0`) Fifo threshold.
		TxFifoThreshold     = 0x02,
		/// Set when Tx Fifo is full, reset when Tx Fifo is full below
		/// threshold.
		TxFifoFull          = 0x03,
		/// Set when Rx Fifo has overflown, reset when the Fifo has been flushed.
		RxFifoOverflow      = 0x04,
		/// Set when Rx Fifo has overflown, reset when the Fifo has been flushed.
		TxFifoUnderflow     = 0x05,
		/// Set when sync word has been sent / received, reset at the end of packet.
		SyncWord            = 0x06,
		/// Set when a packet with correct CRC has been received,
		/// reset when byte is read from Rx Fifo.
		PacketReceived      = 0x07,
		/// Set when preamble quality has been reached, reset when reentering
		/// Rx state or when the pramble quality goes below the threshold.
		PreambleQuality     = 0x08,
		/// Clear channel assessment. Set when RSSI level is below threshold.
		ClearChannel        = 0x09,
		/// Pll is in lock if the output is set.
		PllLock             = 0x0a,
		/// Serial clock, synchronous to the data in synchronous serial mode.
		SerialClock         = 0x0b,
		/// Serial synchronous data output.
		SynchronousDataOut  = 0x0c,
		/// Serial data output used in asynchronous serial mode.
		AsynchronousDataOut = 0x0d,
		/// Set if RSSI level is above threshold, reset when entering Idle mode.
		CarrierSense        = 0x0e,
		/// Set if the last CRC matched, reset when entering/restarting Rx mode.
		CrcOk               = 0x0f,
		/// Can be used together with RxSymbolTick for alternative serial Rx output.
		RxHardData1         = 0x16,
		/// Can be used together with RxSymbolTick for alternative serial Rx output.
		RxHardData0         = 0x17,
		/// Will have the same signal in Sleep and Tx states.
		/// Do not use to control and external PA.
		PaPd                = 0x1b,
		/// Will have the same signal in Sleep and Tx states.
		/// Do not use to control and external LNA.
		LnaPd               = 0x1c,
		/// Can be used together with RxHardData for alternative serial Rx output.
		RxSymbolTick        = 0x1d,
		/// WorEvent0 output
		WorEvent0           = 0x24,
		/// WorEvent1 output
		WorEvent1           = 0x25,
		/// Clk256 output
		Clk256              = 0x26,
		/// Clk32k output
		Clk32k              = 0x27,
		/// ChipReady output
		ChipReady           = 0x29,
		/// XOscialltorStable output
		XOscialltorStable   = 0x2b,
		/// High Impedance
		HighImpedance       = 0x2e,
		/// Can be used to control an external LNA/PA or Rx/Tx switch.
		Hardware0           = 0x2f,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver1       = 0x30,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver3Over2  = 0x31,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver2       = 0x32,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver3       = 0x33,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver4       = 0x34,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver6       = 0x35,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver8       = 0x36,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver12      = 0x37,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver16      = 0x38,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver24      = 0x39,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver32      = 0x3a,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver48      = 0x3b,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver64      = 0x3c,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver96      = 0x3d,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver128     = 0x3e,
		/// XOscillatorClock output. Can only be used on one pin!
		XOscillatorClockOver192     = 0x3f,
	};

	/// Adc Retention setting in the Fifo threshold register
	///
	/// It is not fully explained what this acutally does, but the
	/// datasheet says to set this to 1 if you want to have a Rx filter
	/// bandwidth below 325kHz at time of wake-up.
	/// Part of the 0x03 FIFOTHR register (datasheet page 72)
	enum class
	AdcRetention : uint8_t
	{
		RxFilterAbove325kHz = 0,
		RxFilterBelow325kHz = Bit6,
	};

	/// Attenuation at the receiver.
	///
	/// This can be used for close in reception (see DN010).
	/// Part of the 0x03 FIFOTHR register (datasheet page 72)
	enum class
	RxAttenuation : uint8_t
	{
		dB0  = 0,
		dB6  = Bit4,
		dB12 = Bit5,
		dB18 = Bit5 | Bit4,
	};

	/// Tx and Rx Fifo threshold, exceeded when the number of bytes is equal or higher.
	///
	/// Part of the 0x03 FIFOTHR register (datasheet page 72)
	enum class
	FifoThreshold : uint8_t
	{
		Tx61Rx4  = 0x00,
		Tx57Rx8  = 0x01,
		Tx53Rx12 = 0x02,
		Tx49Rx16 = 0x03,
		Tx45Rx20 = 0x04,
		Tx41Rx24 = 0x05,
		Tx37Rx28 = 0x06,
		Tx33Rx32 = 0x07,
		Tx29Rx36 = 0x08,
		Tx25Rx40 = 0x09,
		Tx21Rx44 = 0x0a,
		Tx17Rx48 = 0x0b,
		Tx13Rx52 = 0x0c,
		Tx9Rx56  = 0x0d,
		Tx5Rx60  = 0x0e,
		Tx1Rx64  = 0x0f,
	};

	/// Enable automatic flush of Rx Fifo when CRC is not OK.
	///
	/// This requires that only one packet is in the Rx Fifo and that the
	/// packet lentgh is limited to the Rx Fifo size.
	/// Part of the 0x07 PKTCTRL1 register (datasheet page 73)
	enum class
	CrcAutoFlush : uint8_t
	{
		Disabled = 0,
		Enabled  = Bit3,
	};

	/// Append status bytes containing RSSI, LQI and CRC OK to payload
	///
	/// Part of the 0x07 PKTCTRL1 register (datasheet page 73)
	enum class
	AppendStatus : uint8_t
	{
		Disabled = 0,
		Enabled  = Bit2,
	};

	/// Address Check
	///
	/// Part of the 0x07 PKTCTRL1 register (datasheet page 73)
	enum class
	AddressCheck : uint8_t
	{
		None             = 0,			///< No address check.
		NoBradcast       = Bit0,		///< Only check for this radio's address.
		Broadcast00      = Bit1,		///< Also listen to broadcast address `0x00`.
		Broadcast00AndFF = Bit1 | Bit0,	///< Also listen to broadcast address `0x00` and `0xff`.
	};

	/// Turn data whitening on/off.
	///
	/// For more information on data whitening, consult the datasheet on page 37.
	/// Part of the 0x08 PKTCTRL0 register (datasheet page 74
	enum class
	DataWhitening : uint8_t
	{
		Disabled = 0,
		Enabled  = Bit6,
	};

	/// Rx/Tx Packet Format
	///
	/// Part of the 0x08 PKTCTRL0 register (datasheet page 74
	enum class
	PacketFormat : uint8_t
	{
		Normal             = 0,				///< use Fifos for Rx and Tx
		SynchronousSerial  = Bit4,			///< data in on Gdo0, data out on Gdox
		RandomTx           = Bit5,			///< random data using PN9 generator
		AsynchronousSerial = Bit5 | Bit4,	///< data in on Gdo0, data out on Gdox
	};

	/// Crc Calculation
	///
	/// Part of the 0x08 PKTCTRL0 register (datasheet page 74
	enum class
	CrcCalculation : uint8_t
	{
		Disabled = 0,
		Enabled  = Bit2,
	};

	/// Packet Length Config
	///
	/// Part of the 0x08 PKTCTRL0 register (datasheet page 74
	enum class
	PacketLengthConfig : uint8_t
	{
		Fixed    = 0,		///< length configured in PKTLEN register
		Variable = Bit0,	///< packet length configure by the first byte after sync word
		Infinite = Bit1,	///< infinite packet length mode
	};

	/// Sets the decimation ration for the delta-sigma ADC input stream and thus the channel bandwidth.
	///
	/// Part of the 0x10 MDMCFG4 register (datasheet page 76)
	enum class
	ChannelBandwidth : uint8_t
	{
		XOscOver32  = (0x00 << 4),	///< `8*(4+0) * 2^0`
		XOscOver40  = (0x01 << 4),	///< `8*(4+1) * 2^0`
		XOscOver48  = (0x02 << 4),	///< `8*(4+2) * 2^0`
		XOscOver56  = (0x03 << 4),	///< `8*(4+3) * 2^0`
		XOscOver64  = (0x04 << 4),	///< `8*(4+0) * 2^1`
		XOscOver80  = (0x05 << 4),	///< `8*(4+1) * 2^1`
		XOscOver96  = (0x06 << 4),	///< `8*(4+2) * 2^1`
		XOscOver112 = (0x07 << 4),	///< `8*(4+3) * 2^1`
		XOscOver128 = (0x08 << 4),	///< `8*(4+0) * 2^2`
		XOscOver160 = (0x09 << 4),	///< `8*(4+1) * 2^2`
		XOscOver192 = (0x0a << 4),	///< `8*(4+2) * 2^2`
		XOscOver224 = (0x0b << 4),	///< `8*(4+3) * 2^2`
		XOscOver256 = (0x0c << 4),	///< `8*(4+0) * 2^3`
		XOscOver320 = (0x0d << 4),	///< `8*(4+1) * 2^3`
		XOscOver384 = (0x0e << 4),	///< `8*(4+2) * 2^3`
		XOscOver448 = (0x0f << 4),	///< `8*(4+3) * 2^3`
	};

	/// Digital Dc Blocking Filter
	///
	/// Disabling the digital DC blocking filter before the demodulator
	/// decreases sensitivity but improves power usage.
	/// @warn The recommended IF frequency changes when the DC blocking is disabled.
	/// @warn Should only be disabled for data rates below 250 kBaud.
	/// Part of the 0x12 MDMCFG2 register (datasheet page 77)
	enum class
	DigitalDcBlockingFilter : uint8_t
	{
		Enabled  = 0,
		Disabled = Bit7,
	};

	/// Modulation Format
	///
	/// Part of the 0x12 MDMCFG2 register (datasheet page 77)
	enum class
	ModulationFormat : uint8_t
	{
		Fsk2   = 0,						///< Frequency Shift Keying 1 bit/symbol
		GFsk   = Bit4,					///< Gaussian Frequency Shift Keying
		AskOok = Bit5 | Bit4,			///< Amplitude Shift Keying or On Off Keying
		Fsk4   = Bit6,					///< Frequency Shift Keying 2 bit/symbol
		Msk    = Bit6 | Bit5 | Bit4,	///< Minimum Shift Keying
	};

	/// Manchester Encoding
	///
	/// Part of the 0x12 MDMCFG2 register (datasheet page 77)
	enum class
	ManchesterEncoding : uint8_t
	{
		Disabled = 0,
		Enabled  = Bit3,
	};

	/// SyncMode
	///
	/// Part of the 0x12 MDMCFG2 register (datasheet page 77)
	enum class
	SyncMode : uint8_t
	{
		NoPreambleSync                        = 0,
		/// 15 sync word bits need to match
		SyncWord15OutOf16Bits                 = Bit0,
		/// 16 sync word bits need to match
		SyncWord16OutOf16Bits                 = Bit1,
		/// 30 sync word bits need to match
		SyncWord30OutOf32Bits                 = Bit1 | Bit0,
		/// carrier sense needs to be above threshold
		NoPreambleSyncCarrierSense            = NoPreambleSync | Bit2,
		/// 15 sync word bits need to match, carrier sense needs to be above threshold
		SyncWord15OutOf16BitsSyncCarrierSense = SyncWord15OutOf16Bits | Bit2,
		/// 16 sync word bits need to match, carrier sense needs to be above threshold
		SyncWord16OutOf16BitsSyncCarrierSense = SyncWord16OutOf16Bits | Bit2,
		/// 30 sync word bits need to match, carrier sense needs to be above threshold
		SyncWord30OutOf32BitsSyncCarrierSense = SyncWord30OutOf32Bits | Bit2,
	};

	/// Forward Error Correction
	///
	/// Part of the 0x13 MDMCFG1 register (datasheet page 78)
	enum class
	ForwardErrorCorrection : uint8_t
	{
		Disabled = 0,
		Enabled  = Bit7,
	};

	/// Minimum number of preamble bytes to be transmitted
	///
	/// Part of the 0x13 MDMCFG1 register (datasheet page 78)
	enum class
	PreambleLength : uint8_t
	{
		Bytes2  = 0,
		Bytes3  = Bit4,
		Bytes4  = Bit5,
		Bytes6  = Bit5 | Bit4,
		Bytes8  = Bit6,
		Bytes12 = Bit6 | Bit4,
		Bytes16 = Bit6 | Bit5,
		Bytes24 = Bit6 | Bit5 | Bit4,
	};

	/// Direct RX termination based on RSSI measurement (carrier sense)
	///
	/// Part of the 0x16 MCSM2 register (datasheet page 80)
	enum class
	RxTimeRssi : uint8_t
	{
		Disabled = 0,
		Enabled  = Bit4,
	};

	/// When RX_TIME timer expires and this is enabled PQI is checked
	///
	/// Part of the 0x16 MCSM2 register (datasheet page 80)
	enum class
	RxTimeQual : uint8_t
	{
		Disabled = 0,
		Enabled  = Bit3,
	};

	/// Slelects the CCA Mode
	///
	/// Part of the 0x17 MCSM1 register (datasheet page 81)
	enum class
	CcaMode : uint8_t
	{
		Always                = 0,
		IfRssiBelowThreshold  = Bit4,
		UnlessReceivingPacket = Bit5,
		IfRssiBelowThresholdUnlessReceivingPacket = Bit5 | Bit4,
	};

	/// Slelects what should happen when a packet has been received
	///
	/// Part of the 0x17 MCSM1 register (datasheet page 81)
	enum class
	RxOffMode : uint8_t
	{
		Idle     = 0,
		FsTxOn   = Bit2,
		Tx       = Bit3,
		StayInRx = Bit3 | Bit2,
	};

	/// Slelects what should happen when a packet has been received
	///
	/// Part of the 0x17 MCSM1 register (datasheet page 81)
	enum class
	TxOffMode : uint8_t
	{
		Idle     = 0,
		FsTxOn   = Bit0,
		StayInTx = Bit1,
		Rx       = Bit1 | Bit0,
	};

	/// Automatically callibrate when going to Rx or Tx or back to Idle
	///
	/// Part of the 0x18 MCSM0 register (datasheet page 82)
	enum class
	FsAutoCallibration : uint8_t
	{
		Never           = 0,
		IdleToRxOrTx    = Bit4,
		RxOrTxToIdle    = Bit5,
		/// every 4th time when going from Rx or Tx to Idle automatically
		RxOrTxToIdle4th = Bit4 | Bit5,
	};

	/// Sets timeout between XSOC stabilization and CHP_RDYn going low
	///
	/// Programs the number of times the six-bit ripple counter must
	/// expire.
	/// For robust operation `Count64` or `Count256` when XOSC id off
	/// during power-down is recommended.
	/// Part of the 0x18 MCSM0 register (datasheet page 82)
	enum class
	PowerTimeout : uint8_t
	{
		Count1   = 0,			///< aprrox. 2.3-2.4 us
		Count16  = Bit2,		///< approx. 37 - 39 us
		Count64  = Bit3,		///< approx. 149 - 155 us
		Count256 = Bit3 | Bit2,	///< approx. 597 - 620 us
	};

	/// Pin radio control option.
	///
	/// Part of the 0x18 MCSM0 register (datasheet page 82)
	enum class
	PinControl : uint8_t
	{
		Disabled = 0,
		Enabled  = Bit1,
	};

	/// Forces XOSC to stay on in Sleep mode.
	///
	/// Part of the 0x18 MCSM0 register (datasheet page 82)
	enum class
	ForceXOscOnDuringSleep : uint8_t
	{
		Disabled = 0,
		Enabled  = Bit0,
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
}	// namespace xpcc

#endif	// XPCC_CC1101_REGISTERS_HPP
