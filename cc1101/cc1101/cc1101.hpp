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
		PartnumNotZero,
		VersionIsNot20AsExpected,
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
 *        typedef stm32::GpioInputD6      Gdo0;
 *        typedef stm32::GpioInputD4      Gdo2;
 *    };
 *
 *
 * @ingroup radio
 * @author  eKiwi <electron.kiwi@gmail.com>
 */
template<typename Configuration>
class
CC1101 : protected xpcc::co::NestedCoroutine<2>, public CC1101Base
{
public:
	/// Identifies the Gdo pin by it's register address.
	enum class
	Gdo : uint8_t
	{
		Gdo2 = Register::IOCFG2,
		Gdo1 = Register::IOCFG1,
		Gdo0 = Register::IOCFG0,
	};

	/// Indicates whether the ouput should be inverted or not.
	enum class
	GdoInverted : uint8_t
	{
		No  = 0,
		Yes = IOCFG2::INV,
	};

	/// GDOx Signal Selection
	///
	/// See CC1101 datasheet page 62, table 41 for more information
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


public:
	/// Resets the cc1101 chip and checks PARTNUM and VERSION registers for expected values.
	xpcc::co::Result<InitializeError>
	initialize(void *ctx);

	/// Configures one of the three available output pins by writing to the
	/// corresponding register.
	///
	/// Please not the Gdo1 is the MISO pin and can only be used when Cs ist
	/// set. If you have any other devices connected to the SPI Bus, it must
	/// be configured in High Impedance mode in order not to caus any shorts.
	///
	/// Currently there is no support for enabling the temperature sensor
	/// on Gdo0 nor for controlling the driver strength of Gdo1. If needed
	/// this could be added to this method.
	///
	/// For more information on the Gdo pins see CC1101 datasheet page 61.
	xpcc::co::Result<void>
	configureGdo(void *ctx, Gdo gdo, GdoInverted inverted, GdoSignalSelection sel);

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
