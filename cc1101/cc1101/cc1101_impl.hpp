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
	return writeRegister(ctx,
		static_cast<Register>(gdo),
		static_cast<uint8_t>(inverted) | static_cast<uint8_t>(sel));
}

template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::configureFifoThreshold(void *ctx,
	FifoThreshold threshold, AdcRetention retention, RxAttenuation attenuation)
{
	return writeRegister(ctx, Register::FIFOTHR,
		static_cast<uint8_t>(threshold) |
		static_cast<uint8_t>(retention) |
		static_cast<uint8_t>(attenuation));
}

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
