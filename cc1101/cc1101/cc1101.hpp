// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_CC1101_HPP
#define XPCC_CC1101_HPP

#include <xpcc/processing/coroutine.hpp>
#include <cstddef>

#include "registers.hpp"

namespace xpcc
{
namespace radio
{

/// Small base class for some common enum classes that do not depend on template parameters.
class
CC1101Base : public CC1101Registers
{
public:
	/// Returned by `initialize` to indicate error conditions.
	enum class
	InitializeError
	{
		None,
		InvalidSomething,
	};

	/// Translates the error code to its string representation.
	static const char*
	enumToString(InitializeError e);
};


/**
 * Hardware abstraction layer for the CC1101 chip
 *
 * http://www.ti.com/product/cc1101
 *
 * Please note: a lot of the CC11xx radios from TI have similar registers,
 * thus this class could be used to address e.g. the CC115L Transmitter
 * as well.
 *
 * ### Example Configuration
 *
 *    struct CC1101Config {
 *        typedef stm32::SpiSimpleMaster3 SpiMaster;
 *        typedef stm32::GpioOutputA15    Cs;
 *        typedef stm32::GpioInputB4      Miso;
 *        typedef stm32::GpioD6           Gdo0;
 *        typedef stm32::GpioD4           Gdo2;
 *    };
 *
 *
 * @ingroup radio
 * @author  eKiwi <electron.kiwi@gmail.com>
 */
template<typename Configuration>
class
CC1101 : protected xpcc::co::NestedCoroutine<1>, public CC1101Base
{
public:
	///
	xpcc::co::Result<InitializeError>
	initialize(void *ctx);

	/// Reads and returns value of register.
	xpcc::co::Result<uint8_t>
	readRegister(void *ctx, Register reg);

	/// Reads values from `length` registers starting at the `reg`
	///
	/// The output will be written to memory starting at `values`.
	/// Make sure that `length` bytes of memory are available.
	xpcc::co::Result<void>
	readRegister(void *ctx, Register reg, uint8_t* values, size_t length);

	/// Writes `value` to `reg`.
	xpcc::co::Result<void>
	writeRegister(void *ctx, Register reg, uint8_t value);

	/// Writes `length` values to registers starting at `reg`.
	xpcc::co::Result<void>
	writeRegister(void *ctx, Register reg, uint8_t* values, size_t length);

	/// Sends command to the CC1101 chip.
	xpcc::co::Result<void>
	writeCommand(void *ctx, Command command);

private:
	/// Spi Master interface the cc1101 chip is connected to
	/// remember to connect pins before using the CC1101 class
	typedef typename Configuration::SpiMaster Spi;
	/// Chips Select Gpio, configured as output outside of CC1101 class
	typedef typename Configuration::Cs   Cs;
	/// Miso Gpio, connected to Spi outside of CC1101 class
	typedef typename Configuration::Miso Miso;
	typedef typename Configuration::Gdo0 Gdo0;
	typedef typename Configuration::Gdo2 Gdo2;

};	// class CC1101Registers
}	// namespace radio
}	// namespace cc1101

#include "cc1101_impl.hpp"

#endif	// XPCC_CC1101_HPP
