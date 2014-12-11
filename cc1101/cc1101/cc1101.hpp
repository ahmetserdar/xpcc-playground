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

	/// Identifies the Gdo pin by it's register address.
	enum class
	Gdo : uint8_t
	{
		Gdo2 = Register::IOCFG2,
		Gdo1 = Register::IOCFG1,
		Gdo0 = Register::IOCFG0,
	};
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
 * ~~~~~~~~~~~~~{.cpp}
 * struct CC1101Config {
 *     typedef stm32::SpiSimpleMaster3 SpiMaster;
 *     typedef stm32::GpioOutputA15    Cs;
 *     typedef stm32::GpioInputB4      Miso;
 *     typedef stm32::GpioInputD6      Gdo0;
 *     typedef stm32::GpioInputD4      Gdo2;
 * };
 * ~~~~~~~~~~~~~
 *
 * @ingroup radio
 * @author  eKiwi <electron.kiwi@gmail.com>
 */
template<typename Configuration>
class
CC1101 : protected xpcc::co::NestedCoroutine<2>, public CC1101Base
{
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
	inline xpcc::co::Result<void>
	configureGdo(void *ctx, Gdo gdo, GdoSignalSelection sel,
		GdoInverted inverted = GdoInverted::No);

	/// Writes configuration to the Rx/Tx Fifo threshold register
	inline xpcc::co::Result<void>
	configureFifoThreshold(void *ctx, FifoThreshold threshold,
		AdcRetention retention,
		RxAttenuation attenuation = RxAttenuation::dB0);

	/// Configures the 16-bit sync word.
	inline xpcc::co::Result<void>
	configureSyncWord(void *ctx, uint16_t sync);

	/// Configures the packet length.
	///
	/// If variable packet length mode is used, this specifies the maximum
	/// packet length allowed. `length` must be differnt from 0.
	inline xpcc::co::Result<void>
	configurePacketLength(void *ctx, uint8_t length);

	/// Configures the PKTCTRL1 register
	///
	/// @param preambleQualityThreshold
	///        The preamble quality estimate increases an internal counter by
	///        one each time a bit that is different from the previous bit is
	///        received and decreases the counter by 8 each time the same bit
	///        is received. A threshold of 4*`preambleQualityThreshold` is
	///        used to gate sync word detection. When `preambleQualityThreshold`
	///        is `0`, a sync word is always accepted.
	inline xpcc::co::Result<void>
	configurePacketAutomationControl1(void *ctx,
		uint8_t preambleQualityThreshold,
		CrcAutoFlush auto_flush,
		AppendStatus append_status,
		AddressCheck address_check);

	/// Configures the PKTCTRL0 register
	inline xpcc::co::Result<void>
	configurePacketAutomationControl0(void *ctx,
		DataWhitening data_whitening,
		PacketFormat packet_format,
		CrcCalculation crc,
		PacketLengthConfig packet_length_config);

	/// Configures the 8 bit device address
	///
	/// 0x00 and 0xff can be used for broadcasts, see #AddressCheck.
	inline xpcc::co::Result<void>
	configureAddress(void *ctx, uint8_t address);

	/// Configures the channel number
	///
	/// @param channel will be multiplied by the channel spacing and added to
	///        the base frequency.
	inline xpcc::co::Result<void>
	configureChannelNumber(void *ctx, uint8_t channel);

	/// Configures the IF frequency used in Rx mode.
	///
	/// TODO: find out what this exactly does.
	inline xpcc::co::Result<void>
	configureIfFrequency(void *ctx, uint8_t if_freq);

	/// Configures the frequency offset
	///
	/// TODO: find out what this exactly does.
	inline xpcc::co::Result<void>
	configureFrequencyOffset(void *ctx, int8_t freq_off);

	/// Configures the base frequency
	///
	/// TODO: see how we can make this a little bit more readable
	inline xpcc::co::Result<void>
	configureBaseFrequency(void *ctx, uint32_t base_freq);

	/// Configures the data rate and the bandwidth
	///
	/// TODO: see how we can make this a little bit more readable
	inline xpcc::co::Result<void>
	configureDataRateAndBandwidth(void *ctx,
		uint8_t data_rate_m, uint8_t data_rate_e, ChannelBandwidth bw);

	/// Configures the modulation format, encoding and sync mode
	inline xpcc::co::Result<void>
	configureModem2(void *ctx,
		ModulationFormat mod_format, SyncMode sync_mode,
		ManchesterEncoding manchester = ManchesterEncoding::Disabled,
		DigitalDcBlockingFilter dc_block = DigitalDcBlockingFilter::Enabled);

	/// Configures the channel spacing, the length of the preamble and the FEC
	inline xpcc::co::Result<void>
	configureModem1(void *ctx,
		uint8_t channel_spacing_m, uint8_t channel_spacing_e,
		PreambleLength preamble_length = PreambleLength::Bytes4,
		ForwardErrorCorrection fec = ForwardErrorCorrection::Disabled);

	/// Configures deviation
	///
	/// TODO: find out what exactly this does
	inline xpcc::co::Result<void>
	configureDeviation(void *ctx, uint8_t deviation_m, uint8_t deviation_e);

	/// Configures part of the Main Radio State Machine
	///
	/// TODO: see how we can make this a little bit more readable
	inline xpcc::co::Result<void>
	configureMainRadioFsm2(void *ctx,
		uint8_t rx_time,
		RxTimeRssi rx_time_rssi = RxTimeRssi::Disabled,
		RxTimeQual rx_time_qual = RxTimeQual::Disabled);

	/// Configures part of the Main Radio State Machine
	///
	/// TODO: see how we can make this a little bit more readable
	inline xpcc::co::Result<void>
	configureMainRadioFsm1(void *ctx,
		CcaMode cca, RxOffMode rx_off_mode, TxOffMode tx_off_mode);

	/// Configures part of the Main Radio State Machine
	///
	/// TODO: see how we can make this a little bit more readable
	inline xpcc::co::Result<void>
	configureMainRadioFsm0(void *ctx,
		FsAutoCallibration auto_cal,
		PowerTimeout power_timeout = PowerTimeout::Count64,
		PinControl pin_control = PinControl::Disabled,
		ForceXOscOnDuringSleep force_xosc_on = ForceXOscOnDuringSleep::Disabled);

	/// Configures the Frequency Offset Compensation Algorithm
	inline xpcc::co::Result<void>
	configureFrequencyOffsetCompensation(void *ctx,
		FocBsCsGate bs_cs_gate,
		FocPreK pre_k,
		FocPostK post_kl = FocPostK::KOver2,
		FocLimit limit = FocLimit::ChannelBandwidthOver4);

	/// Configures the Bit Synchronization Algorithm
	inline xpcc::co::Result<void>
	configureBitSynchronization(void *ctx,
		BitSynchronizationPreKI  pre_ki  = BitSynchronizationPreKI::KI2,
		BitSynchronizationPreKP  pre_kp  = BitSynchronizationPreKP::KP3,
		BitSynchronizationPostKI post_ki = BitSynchronizationPostKI::KIOver2,
		BitSynchronizationPostKP post_kp = BitSynchronizationPostKP::KPOver2,
		BitSynchronizationLimit  limit   = BitSynchronizationLimit::NoDataRateOffsetCompensation);

	/// Configures the Adjustable Gain Control Algorithm
	inline xpcc::co::Result<void>
	configureAgc(void *ctx,
		MaxDVgaGain max_dvga_gain,
		MaxLnaGain  max_lna_gain = MaxLnaGain::Max,
		MagnTarget  magn_target  = MagnTarget::dB33,
		AgcLnaPriority lna_priority = AgcLnaPriority::DecreaseLnaGainFirst,
		CarrierSenseRelativeThreshold
			carrier_sense_relative_threshold = CarrierSenseRelativeThreshold::Disabled,
		int8_t carrier_sense_absolute_threshold = 0,
		HysteresisLevel hyst_level = HysteresisLevel::Medium,
		WaitTime wait_time = WaitTime::FilterSamples16,
		AgcFreeze agc_freeze = AgcFreeze::NormalOperation,
		FilterLength filter_length = FilterLength::FilterSamples16);

	/// Configures the Event0 timeout
	///
	/// TODO: see how we can make this a little bit more readable
	inline xpcc::co::Result<void>
	configureEvent0Timeout(void *ctx, uint16_t timeout_value);

	/// Configures the wake on radio
	inline xpcc::co::Result<void>
	configureWakeOnRadio(void *ctx,
		WorResolution wor_resolution,
		PowerDownSignalToRcOscialltor
			power_down = PowerDownSignalToRcOscialltor::Default,
		Event1Timeout event1_timeout = Event1Timeout::TEvent48,
		RcOscillatorCalibration
			rc_callibration = RcOscillatorCalibration::Enabled);

	/// Configures the rx front end
	///
	/// Basically these are all magic values without any real description
	/// in the TI datasheet (page 89).
	inline xpcc::co::Result<void>
	configureRxFrontEnd(void *ctx,
		uint8_t lna_current = 1,
		uint8_t lna2mix_current = 1,
		uint8_t lodiv_buf_current_rx = 1,
		uint8_t mix_current = 2);

	/// Configures the tx front end
	inline xpcc::co::Result<void>
	configureTxFrontEnd(void *ctx,
		uint8_t pa_power_index = 0,		///< points to value in the PATABLE
		uint8_t lodiv_buf_current = 1	///< magic value
		);

	/// Configures the upper bytes of the Frequency Synthesizer Calibration
	///
	/// This was split up into one method for the upper three and
	/// one for the lowest byte, because the upper three bytes
	/// might need to be changed during run time.
	///
	/// Basically these are all magic values without any real description
	/// in the TI datasheet (page 89-90).
	inline xpcc::co::Result<void>
	configureFrequencySynthesizerCalibration321(void *ctx,
		uint8_t fs_cal_3, uint8_t fs_cal_2, uint8_t fs_cal_1);

	/// Configures the lowest byte of the Frequency Synthesizer Calibration
	///
	/// This was split up into one method for the upper three and
	/// one for the lowest byte, because the upper three bytes
	/// might need to be changed during run time.
	///
	/// Basically these are all magic values without any real description
	/// in the TI datasheet (page 89-90).
	inline xpcc::co::Result<void>
	configureFrequencySynthesizerCalibration0(void *ctx, uint8_t fs_cal_0);

	/// Configures the RC Oscillator
	///
	/// Basically these are all magic values without any real description
	/// in the TI datasheet (page 90).
	inline xpcc::co::Result<void>
	configureRcOscillator(void *ctx, uint8_t high_byte = 0x41, uint8_t low_byte = 0x00);

	// FSTEST is "For test only. Do not write to this register."

	/// Configures the Test Registers
	///
	/// Basically these are all magic values without any real description
	/// in the TI datasheet (page 91).
	inline xpcc::co::Result<void>
	configureTest(void *ctx,
		VcoSelectionCalibration vco_cal,
		uint8_t ptest    = 0x7f,
		uint8_t agc_test = 0x3f,
		uint8_t test2    = 0x81,
		uint8_t test1    = 0x35,
		uint8_t test0    = 0x09		///< bit1 will be overwritten by vco_cal
		);

	/// Sends data from a max 61 byte long buffer
	inline xpcc::co::Result<void>
	sendData(void *ctx, uint8_t *buffer, size_t len);

	//-------------------------------------------------------------------------
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
