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

namespace xpcc
{
namespace radio
{

template<typename Configuration>
xpcc::co::Result<CC1101Base::InitializeError>
CC1101<Configuration>::initialize(void *ctx)
{
	CO_BEGIN(ctx);

	// 1.) reset as described in the datasheet on page 51 (19.1.2 Manual Reset)
	// FIXME: the delayMicorseconds is very ugly, but how can we fix this?
	Cs::reset();
	::xpcc::delayMicroseconds(15);
	Cs::set();
	::xpcc::delayMicroseconds(25 + 5);	// "at LEAST 40 us"
	Cs::reset();
	CO_WAIT_UNTIL(!Miso::read());		// wait for Miso to go low
	CO_CALL(Spi::writeRead(static_cast<uint8_t>(Command::SRES)));
	CO_WAIT_UNTIL(!Miso::read());		// wait for Miso to go low again
	Cs::set();

	// 2.) poll PARTNUM which should be zero
	if(CO_CALL(readRegister(ctx, Register::PARTNUM)) != 0) {
		CO_RETURN(InitializeError::PartnumNotZero);
	}

	// 3.) poll VERSION which is expected to be 20 (but could be different)
	if(CO_CALL(readRegister(ctx, Register::VERSION)) != 20) {
		CO_RETURN(InitializeError::VersionIsNot20AsExpected);
	}

	CO_END_RETURN(InitializeError::None);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureGdo(void *ctx,
	Gdo gdo, GdoSignalSelection sel, GdoInverted inverted)
{
	uint8_t d = static_cast<uint8_t>(inverted) | static_cast<uint8_t>(sel);
	return writeRegister(ctx, static_cast<Register>(gdo), d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureFifoThreshold(void *ctx,
	FifoThreshold threshold, AdcRetention retention, RxAttenuation attenuation)
{
	uint8_t d = static_cast<uint8_t>(threshold) |
	            static_cast<uint8_t>(retention) |
	            static_cast<uint8_t>(attenuation);
	return writeRegister(ctx, Register::FIFOTHR, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureSyncWord(void *ctx, uint16_t sync)
{
	CO_BEGIN(ctx);
	CO_CALL(writeRegister(ctx, Register::SYNC1, static_cast<uint8_t>(sync >> 8)));
	CO_CALL(writeRegister(ctx, Register::SYNC0, static_cast<uint8_t>(sync & 0xff)));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configurePacketLength(void *ctx, uint8_t length)
{
	return writeRegister(ctx, Register::PKTLEN, length);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configurePacketAutomationControl1(void *ctx,
	uint8_t preambleQualityThreshold, CrcAutoFlush auto_flush,
	AppendStatus append_status, AddressCheck address_check)
{
	uint8_t d = ((preambleQualityThreshold & 0x7) << 5) |
	            static_cast<uint8_t>(auto_flush) |
	            static_cast<uint8_t>(append_status) |
	            static_cast<uint8_t>(address_check);
	return writeRegister(ctx, Register::PKTCTRL1, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configurePacketAutomationControl0(void *ctx,
	DataWhitening data_whitening, PacketFormat packet_format,
	CrcCalculation crc, PacketLengthConfig packet_length_config)
{
	uint8_t d = static_cast<uint8_t>(data_whitening) |
	            static_cast<uint8_t>(packet_format) |
	            static_cast<uint8_t>(crc) |
	            static_cast<uint8_t>(packet_length_config);
	return writeRegister(ctx, Register::PKTCTRL0, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureAddress(void *ctx, uint8_t address)
{
	return writeRegister(ctx, Register::ADDR, address);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureChannelNumber(void *ctx, uint8_t channel)
{
	return writeRegister(ctx, Register::CHANNR, channel);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureIfFrequency(void *ctx, uint8_t if_freq)
{
	return writeRegister(ctx, Register::FSCTRL1, (if_freq & 0x1f));
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureFrequencyOffset(void *ctx, int8_t freq_off)
{
	return writeRegister(ctx, Register::FSCTRL0, freq_off);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureBaseFrequency(void *ctx, uint32_t base_freq)
{
	static uint32_t v;
	CO_BEGIN(ctx);
	v = (base_freq & 0x3fffff);	// TODO: actually calculate value from frequency
	CO_CALL(writeRegister(ctx, Register::FREQ2, static_cast<uint8_t>((v >> 16) & 0x3f)));
	CO_CALL(writeRegister(ctx, Register::FREQ1, static_cast<uint8_t>((v >>  8) & 0xff)));
	CO_CALL(writeRegister(ctx, Register::FREQ0, static_cast<uint8_t>((v >>  0) & 0xff)));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureDataRateAndBandwidth(void *ctx,
	uint8_t data_rate_m, uint8_t data_rate_e, ChannelBandwidth bw)
{
	CO_BEGIN(ctx);
	CO_CALL(writeRegister(ctx, Register::MDMCFG4, static_cast<uint8_t>(bw) | (data_rate_e & 0x0f)));
	CO_CALL(writeRegister(ctx, Register::MDMCFG3, data_rate_m));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureModem2(void *ctx,
	ModulationFormat mod_format, SyncMode sync_mode,
	ManchesterEncoding manchester, DigitalDcBlockingFilter dc_block)
{
	uint8_t d = static_cast<uint8_t>(mod_format) |
	            static_cast<uint8_t>(sync_mode) |
	            static_cast<uint8_t>(manchester) |
	            static_cast<uint8_t>(dc_block);
	return writeRegister(ctx, Register::MDMCFG2, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureModem1(void *ctx,
	uint8_t channel_spacing_m, uint8_t channel_spacing_e,
	PreambleLength preamble_length, ForwardErrorCorrection fec)
{
	CO_BEGIN(ctx);
	CO_CALL(writeRegister(ctx, Register::MDMCFG1,
		static_cast<uint8_t>(fec) |
		static_cast<uint8_t>(preamble_length) |
		(channel_spacing_e & 0x03)));
	CO_CALL(writeRegister(ctx, Register::MDMCFG0, channel_spacing_m));
	CO_END();
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureDeviation(void *ctx,
	uint8_t deviation_m, uint8_t deviation_e)
{
	uint8_t d = ((deviation_e & 0x07) << 4) | (deviation_m & 0x07);
	return writeRegister(ctx, Register::DEVIATN, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureMainRadioFsm2(void *ctx,
	uint8_t rx_time, RxTimeRssi rx_time_rssi, RxTimeQual rx_time_qual)
{
	uint8_t d = (rx_time & 0x07) |
	            static_cast<uint8_t>(rx_time_rssi) |
	            static_cast<uint8_t>(rx_time_qual);
	return writeRegister(ctx, Register::MCSM2, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureMainRadioFsm1(void *ctx,
	CcaMode cca, RxOffMode rx_off_mode, TxOffMode tx_off_mode)
{
	uint8_t d = static_cast<uint8_t>(cca) |
	            static_cast<uint8_t>(rx_off_mode) |
	            static_cast<uint8_t>(tx_off_mode);
	return writeRegister(ctx, Register::MCSM1, d);
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureMainRadioFsm0(void *ctx,
	FsAutoCallibration auto_cal, PowerTimeout power_timeout,
	PinControl pin_control, ForceXOscOnDuringSleep force_xosc_on)
{
	uint8_t d = static_cast<uint8_t>(auto_cal) |
	            static_cast<uint8_t>(power_timeout) |
	            static_cast<uint8_t>(pin_control) |
	            static_cast<uint8_t>(force_xosc_on);
	return writeRegister(ctx, Register::MCSM0, d);
}

//-----------------------------------------------------------------------------
template<typename Configuration>
xpcc::co::Result<uint8_t>
CC1101<Configuration>::readRegister(void *ctx, CC1101Base::Register reg)
{
	static uint8_t value;
	CO_BEGIN(ctx);
	Cs::reset();
	// wait for Miso to go low
	// FIXME: probably miso is always low if we are not in sleep mode
	//        thus maybe it might be more performant to busy wait here
	//        since in most cases waiting won't be necessary at all.
	CO_WAIT_UNTIL(!Miso::read());
	CO_CALL(Spi::writeRead(
		static_cast<uint8_t>(reg) |
		static_cast<uint8_t>(TransferMode::ReadSingleByte)));
	value = CO_CALL(Spi::writeRead(0x00));
	Cs::set();
	CO_END_RETURN(value);
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::readRegister(void *ctx, CC1101Base::Register reg, uint8_t* values, size_t length)
{
	CO_BEGIN(ctx);

	// FIXME: implement!
	CO_END();
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::writeRegister(void *ctx, CC1101Base::Register reg, uint8_t value)
{
	CO_BEGIN(ctx);
	Cs::reset();
	// wait for Miso to go low
	// FIXME: see `readRegister`
	CO_WAIT_UNTIL(!Miso::read());
	CO_CALL(Spi::writeRead(
		static_cast<uint8_t>(reg) |
		static_cast<uint8_t>(TransferMode::WriteSingleByte)));
	CO_CALL(Spi::writeRead(value));
	Cs::set();
	CO_END();
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::writeRegister(void *ctx, CC1101Base::Register reg, uint8_t* values, size_t length)
{
	CO_BEGIN(ctx);

	// FIXME: implement!
	CO_END();
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::writeCommand(void *ctx, CC1101Base::Command command)
{
	CO_BEGIN(ctx);
	Cs::reset();
	// wait for Miso to go low
	// FIXME: see `readRegister`
	CO_WAIT_UNTIL(!Miso::read());
	CO_CALL(Spi::writeRead(static_cast<uint8_t>(command)));
	Cs::set();
	CO_END();
}


}	// namespace radio
}	// namespace cc1101
