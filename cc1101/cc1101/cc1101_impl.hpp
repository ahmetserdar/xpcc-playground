// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_CC1101_HPP
	#error	"Don't include this file directly, use 'cc1101.hpp' instead!"
#endif

#include <xpcc/architecture/driver/delay.hpp>

#if 1
	#include <xpcc/debug/logger.hpp>
	#undef	XPCC_LOG_LEVEL
	#define	XPCC_LOG_LEVEL xpcc::log::DEBUG
#else
	#undef	XPCC_LOG_LEVEL
	#define	XPCC_LOG_LEVEL xpcc::log::ERROR
#endif

template<class T>
constexpr T
max(T a, T b)
{
	return (a > b)? a : b;
}

template<class T>
constexpr T
min(T a, T b)
{
	return (a < b)? a : b;
}

namespace xpcc
{
namespace radio
{

template<typename Configuration>
xpcc::co::Result<CC1101Base::InitializeError>
CC1101<Configuration>::initialize()
{
	CO_BEGIN();

	// 1.) reset as described in the datasheet on page 51 (19.1.2 Manual Reset)
	// FIXME: the delayMicorseconds is very ugly, but how can we fix this?
	Cs::reset();
	::xpcc::delayMicroseconds(15);
	Cs::set();
	::xpcc::delayMicroseconds(25 + 5);	// "at LEAST 40 us"
	Cs::reset();
	CO_WAIT_UNTIL(!Miso::read());		// wait for Miso to go low
	CO_CALL(Spi::transfer(static_cast<uint8_t>(Command::SRES)));
	CO_WAIT_UNTIL(!Miso::read());		// wait for Miso to go low again
	Cs::set();

	// 2.) poll PARTNUM which should be zero
	if(CO_CALL(readRegister(Register::PARTNUM)) != 0) {
		CO_RETURN(InitializeError::PartnumNotZero);
	}

	// 3.) poll VERSION which is expected to be 20 (but could be different)
	if(CO_CALL(readRegister(Register::VERSION)) != 20) {
		CO_RETURN(InitializeError::VersionIsNot20AsExpected);
	}

	CO_END_RETURN(InitializeError::None);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureGdo(
	Gdo gdo, GdoSignalSelection sel, GdoInverted inverted)
{
	uint8_t d = static_cast<uint8_t>(inverted) | static_cast<uint8_t>(sel);
	return writeRegister(static_cast<Register>(gdo), d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureFifoThreshold(
	FifoThreshold threshold, AdcRetention retention, RxAttenuation attenuation)
{
	uint8_t d = static_cast<uint8_t>(threshold) |
	            static_cast<uint8_t>(retention) |
	            static_cast<uint8_t>(attenuation);
	return writeRegister(Register::FIFOTHR, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureSyncWord(uint16_t sync)
{
	CO_BEGIN();
	CO_CALL(writeRegister(Register::SYNC1, static_cast<uint8_t>(sync >> 8)));
	CO_CALL(writeRegister(Register::SYNC0, static_cast<uint8_t>(sync & 0xff)));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configurePacketLength(uint8_t length)
{
	return writeRegister(Register::PKTLEN, length);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configurePacketAutomationControl1(
	uint8_t preambleQualityThreshold, CrcAutoFlush auto_flush,
	AppendStatus append_status, AddressCheck address_check)
{
	uint8_t d = ((preambleQualityThreshold & 0x7) << 5) |
	            static_cast<uint8_t>(auto_flush) |
	            static_cast<uint8_t>(append_status) |
	            static_cast<uint8_t>(address_check);
	return writeRegister(Register::PKTCTRL1, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configurePacketAutomationControl0(
	DataWhitening data_whitening, PacketFormat packet_format,
	CrcCalculation crc, PacketLengthConfig packet_length_config)
{
	uint8_t d = static_cast<uint8_t>(data_whitening) |
	            static_cast<uint8_t>(packet_format) |
	            static_cast<uint8_t>(crc) |
	            static_cast<uint8_t>(packet_length_config);
	return writeRegister(Register::PKTCTRL0, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureAddress(uint8_t address)
{
	return writeRegister(Register::ADDR, address);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureChannelNumber(uint8_t channel)
{
	return writeRegister(Register::CHANNR, channel);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureIfFrequency(uint8_t if_freq)
{
	return writeRegister(Register::FSCTRL1, (if_freq & 0x1f));
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureFrequencyOffset(int8_t freq_off)
{
	return writeRegister(Register::FSCTRL0, freq_off);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureBaseFrequency(uint32_t base_freq)
{
	static uint32_t v;
	CO_BEGIN();
	v = (base_freq & 0x3fffff);	// TODO: actually calculate value from frequency
	CO_CALL(writeRegister(Register::FREQ2, static_cast<uint8_t>((v >> 16) & 0x3f)));
	CO_CALL(writeRegister(Register::FREQ1, static_cast<uint8_t>((v >>  8) & 0xff)));
	CO_CALL(writeRegister(Register::FREQ0, static_cast<uint8_t>((v >>  0) & 0xff)));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureDataRateAndBandwidth(
	uint8_t data_rate_m, uint8_t data_rate_e, ChannelBandwidth bw)
{
	CO_BEGIN();
	CO_CALL(writeRegister(Register::MDMCFG4, static_cast<uint8_t>(bw) | (data_rate_e & 0x0f)));
	CO_CALL(writeRegister(Register::MDMCFG3, data_rate_m));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureModem2(
	ModulationFormat mod_format, SyncMode sync_mode,
	ManchesterEncoding manchester, DigitalDcBlockingFilter dc_block)
{
	uint8_t d = static_cast<uint8_t>(mod_format) |
	            static_cast<uint8_t>(sync_mode) |
	            static_cast<uint8_t>(manchester) |
	            static_cast<uint8_t>(dc_block);
	return writeRegister(Register::MDMCFG2, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureModem1(
	uint8_t channel_spacing_m, uint8_t channel_spacing_e,
	PreambleLength preamble_length, ForwardErrorCorrection fec)
{
	CO_BEGIN();
	CO_CALL(writeRegister(Register::MDMCFG1,
		static_cast<uint8_t>(fec) |
		static_cast<uint8_t>(preamble_length) |
		(channel_spacing_e & 0x03)));
	CO_CALL(writeRegister(Register::MDMCFG0, channel_spacing_m));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureDeviation(
	uint8_t deviation_m, uint8_t deviation_e)
{
	uint8_t d = ((deviation_e & 0x07) << 4) | (deviation_m & 0x07);
	return writeRegister(Register::DEVIATN, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureMainRadioFsm2(
	uint8_t rx_time, RxTimeRssi rx_time_rssi, RxTimeQual rx_time_qual)
{
	uint8_t d = (rx_time & 0x07) |
	            static_cast<uint8_t>(rx_time_rssi) |
	            static_cast<uint8_t>(rx_time_qual);
	return writeRegister(Register::MCSM2, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureMainRadioFsm1(
	CcaMode cca, RxOffMode rx_off_mode, TxOffMode tx_off_mode)
{
	uint8_t d = static_cast<uint8_t>(cca)         |
	            static_cast<uint8_t>(rx_off_mode) |
	            static_cast<uint8_t>(tx_off_mode);
	return writeRegister(Register::MCSM1, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureMainRadioFsm0(
	FsAutoCallibration auto_cal, PowerTimeout power_timeout,
	PinControl pin_control, ForceXOscOnDuringSleep force_xosc_on)
{
	uint8_t d = static_cast<uint8_t>(auto_cal)      |
	            static_cast<uint8_t>(power_timeout) |
	            static_cast<uint8_t>(pin_control)   |
	            static_cast<uint8_t>(force_xosc_on);
	return writeRegister(Register::MCSM0, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureFrequencyOffsetCompensation(
	FocBsCsGate bs_cs_gate, FocPreK pre_k, FocPostK post_kl, FocLimit limit)
{
	uint8_t d = static_cast<uint8_t>(bs_cs_gate) |
	            static_cast<uint8_t>(pre_k)      |
	            static_cast<uint8_t>(post_kl)    |
	            static_cast<uint8_t>(limit);
	return writeRegister(Register::FOCCFG, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureBitSynchronization(
	BitSynchronizationPreKI  pre_ki,  BitSynchronizationPreKP  pre_kp,
	BitSynchronizationPostKI post_ki, BitSynchronizationPostKP post_kp,
	BitSynchronizationLimit limit)
{
	uint8_t d = static_cast<uint8_t>(pre_ki)  |
	            static_cast<uint8_t>(pre_kp)  |
	            static_cast<uint8_t>(post_ki) |
	            static_cast<uint8_t>(post_kp) |
	            static_cast<uint8_t>(limit);
	return writeRegister(Register::BSCFG, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureAgc(
	MaxDVgaGain max_dvga_gain, MaxLnaGain max_lna_gain,
	MagnTarget magn_target, AgcLnaPriority lna_priority,
	CarrierSenseRelativeThreshold carrier_sense_relative_threshold,
	int8_t carrier_sense_absolute_threshold, HysteresisLevel hyst_level,
	WaitTime wait_time, AgcFreeze agc_freeze, FilterLength filter_length)
{
	static uint32_t v;
	CO_BEGIN();
	v = static_cast<uint8_t>(max_dvga_gain) |
	    static_cast<uint8_t>(max_lna_gain)  |
	    static_cast<uint8_t>(magn_target);
	CO_CALL(writeRegister(Register::AGCTRL2, v));
	v =  static_cast<uint8_t>(lna_priority) |
	     static_cast<uint8_t>(carrier_sense_relative_threshold) |
	    (static_cast<uint8_t>(carrier_sense_absolute_threshold) & 0x0f);
	CO_CALL(writeRegister(Register::AGCTRL1, v));
	v = static_cast<uint8_t>(hyst_level) |
	    static_cast<uint8_t>(wait_time)  |
	    static_cast<uint8_t>(agc_freeze) |
	    static_cast<uint8_t>(filter_length);
	CO_CALL(writeRegister(Register::AGCTRL0, v));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureEvent0Timeout(uint16_t timeout_value)
{
	CO_BEGIN();
	CO_CALL(writeRegister(Register::WOREVT1, static_cast<uint8_t>(timeout_value >> 8)));
	CO_CALL(writeRegister(Register::WOREVT0, static_cast<uint8_t>(timeout_value & 0xff)));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureWakeOnRadio(
	WorResolution wor_resolution, PowerDownSignalToRcOscialltor power_down,
	Event1Timeout event1_timeout, RcOscillatorCalibration rc_callibration)
{
	uint8_t d = static_cast<uint8_t>(power_down) |
	            static_cast<uint8_t>(event1_timeout) |
	            static_cast<uint8_t>(rc_callibration) |
	            static_cast<uint8_t>(wor_resolution);
	return writeRegister(Register::WORCTRL, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureRxFrontEnd(
	uint8_t lna_current, uint8_t lna2mix_current,
	uint8_t lodiv_buf_current_rx, uint8_t mix_current)
{
	uint8_t d = ((lna_current          & 0x03) << 6) |
	            ((lna2mix_current      & 0x03) << 4) |
	            ((lodiv_buf_current_rx & 0x03) << 2) |
	            ((mix_current          & 0x03) << 0);
	return writeRegister(Register::FREND1, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureTxFrontEnd(
	uint8_t pa_power_index, uint8_t lodiv_buf_current)
{
	uint8_t d = ((lodiv_buf_current & 0x03) << 4) |
	            ((pa_power_index    & 0x07) << 0);
	return writeRegister(Register::FREND0, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureFrequencySynthesizerCalibration321(
	uint8_t fs_cal_3, uint8_t fs_cal_2, uint8_t fs_cal_1)
{
	CO_BEGIN();
	CO_CALL(writeRegister(Register::FSCAL3, fs_cal_3));
	CO_CALL(writeRegister(Register::FSCAL2, fs_cal_2));
	CO_CALL(writeRegister(Register::FSCAL1, fs_cal_1));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureFrequencySynthesizerCalibration0(
	uint8_t fs_cal_0)
{
	return writeRegister(Register::FSCAL0, fs_cal_0);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureRcOscillator(
	uint8_t high_byte, uint8_t low_byte)
{
	CO_BEGIN();
	CO_CALL(writeRegister(Register::RCCTRL1, high_byte));
	CO_CALL(writeRegister(Register::RCCTRL0, low_byte));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureTest(
	VcoSelectionCalibration vco_cal, uint8_t ptest, uint8_t agc_test,
	uint8_t test2, uint8_t test1, uint8_t test0)
{
	CO_BEGIN();
	CO_CALL(writeRegister(Register::PTEST,   ptest));
	CO_CALL(writeRegister(Register::AGCTEST, agc_test));
	CO_CALL(writeRegister(Register::TEST2,   test2));
	CO_CALL(writeRegister(Register::TEST1,   test1));
	CO_CALL(writeRegister(Register::TEST0, (test0 & 0xfd) | static_cast<uint8_t>(vco_cal)));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::sendData(const uint8_t *buffer, const size_t len)
{
	CO_BEGIN();
	// Go Into Rx State
	// CO_CALL(writeCommand(Command::SRX));
	// TODO: maybe check MARCSTATE
	xpcc::delayMicroseconds(500);	// TODO: is this necessary
	// set data length
	CO_CALL(writeRegister(Register::TXFIFO, min(static_cast<size_t>(61), len)));
	// write data to tx buffer
	CO_CALL(writeRegister(Register::TXFIFO, buffer, min(static_cast<size_t>(61), len)));
	// Go Into Tx State
	CO_CALL(writeCommand(Command::STX));
	// Wait for Gdo0 to go high (i.e. until sync word is transmitted)
	CO_WAIT_UNTIL(Gdo0::read());
	// Wait for Gdo0 to go low  (i.e. until the end of the packet)
	CO_WAIT_WHILE(Gdo0::read());
	// Go to IDLE state
	CO_CALL(writeCommand(Command::SIDLE));
	// Flush Tx Fifo
	CO_CALL(writeCommand(Command::SFTX));
	CO_END();
}

//-----------------------------------------------------------------------------
template<typename Configuration>
xpcc::co::Result<uint8_t>
CC1101<Configuration>::readRegister(const CC1101Base::Register reg)
{
	static uint8_t value;
	CO_BEGIN();
	Cs::reset();
	// wait for Miso to go low
	// FIXME: probably miso is always low if we are not in sleep mode
	//        thus maybe it might be more performant to busy wait here
	//        since in most cases waiting won't be necessary at all.
	CO_WAIT_UNTIL(!Miso::read());
	CO_CALL(Spi::transfer(
		static_cast<uint8_t>(reg) |
		static_cast<uint8_t>(TransferMode::ReadSingleByte)));
	value = CO_CALL(Spi::transfer(0x00));
	Cs::set();
	CO_END_RETURN(value);
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::readRegister(const CC1101Base::Register reg, uint8_t* values, const size_t length)
{
	CO_BEGIN();

	// FIXME: implement!
	CO_END();
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::writeRegister(const CC1101Base::Register reg, const uint8_t value)
{
	CO_BEGIN();
	Cs::reset();
	// wait for Miso to go low
	// FIXME: see `readRegister`
	CO_WAIT_UNTIL(!Miso::read());
	CO_CALL(Spi::transfer(
		static_cast<uint8_t>(reg) |
		static_cast<uint8_t>(TransferMode::WriteSingleByte)));
	CO_CALL(Spi::transfer(value));
	Cs::set();
	CO_END();
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::writeRegister(const CC1101Base::Register reg, const uint8_t* values, const size_t length)
{
	static size_t ii;
	CO_BEGIN();
	Cs::reset();
	// wait for Miso to go low
	// FIXME: see `readRegister`
	CO_WAIT_UNTIL(!Miso::read());
	CO_CALL(Spi::transfer(
		static_cast<uint8_t>(reg) |
		static_cast<uint8_t>(TransferMode::WriteBurst)));
	for(ii = 0; ii < length; ++ii) {
		CO_CALL(Spi::transfer(values[ii]));
	}
	Cs::set();
	CO_END();
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::writeCommand(const CC1101Base::Command command)
{
	CO_BEGIN();
	Cs::reset();
	// wait for Miso to go low
	// FIXME: see `readRegister`
	CO_WAIT_UNTIL(!Miso::read());
	CO_CALL(Spi::transfer(static_cast<uint8_t>(command)));
	Cs::set();
	CO_END();
}


}	// namespace radio
}	// namespace cc1101
