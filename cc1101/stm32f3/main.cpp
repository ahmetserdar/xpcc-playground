// coding: utf-8
/* Copyright (c) 2014, Electronic Kiwi
* All Rights Reserved.
*
* The file is part of the xpcc-playground and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*/

#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>
#include "../../xpcc/examples/stm32f3_discovery/stm32f3_discovery.hpp"

// Create an IODeviceWrapper around the Uart Peripheral we want to use
xpcc::IODeviceWrapper<Usart2, xpcc::IOBuffer::BlockIfFull> loggerDevice;

// Set all four logger streams to use the UART
xpcc::log::Logger xpcc::log::debug(loggerDevice);
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::warning(loggerDevice);
xpcc::log::Logger xpcc::log::error(loggerDevice);

// Set the log level
#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::DEBUG

#include <xpcc_git_info.hpp>
#include <xpcc_build_info.hpp>


#include <xpcc/processing/timeout.hpp>
#include <xpcc/processing/periodic_timer.hpp>
#include <xpcc/processing/protothread.hpp>
#include "../cc1101/cc1101.hpp"


static uint8_t data[60] = {
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55, 0xff,
	0x00, 0x55,
	0x00, 0x00, 0x00, 0x00	// space for packet count
};

enum class Transmission
{
	Fsk34k,
	GFsk250k,
};

static constexpr Transmission Settings = Transmission::GFsk250k;

class MainThread : public xpcc::pt::Protothread
{
public:
	MainThread() : blinkTimer(500)
	{
	}

	/// Needs to be called as often as possible.
	bool
	run()
	{
		if(blinkTimer.isExpired()) {
			LedSouth::toggle();
		}

		PT_BEGIN();

		// try to initialize the device
		static Radio::InitializeError e;
		e = PT_CALL(radio.initialize());
		if(e != Radio::InitializeError::None) {
			XPCC_LOG_ERROR << XPCC_FILE_INFO;
			XPCC_LOG_ERROR << "Error trying to initialize the cc1101: ";
			XPCC_LOG_ERROR << Radio::enumToString(e) << xpcc::endl;
		} else {
			XPCC_LOG_DEBUG << XPCC_FILE_INFO << "Initialized cc1101." << xpcc::endl;
		}

		PT_CALL(radio.configureGdo(
			Radio::Gdo::Gdo0,
			Radio::GdoSignalSelection::SyncWord));			// 0x06
		PT_CALL(radio.configureFifoThreshold(
			Radio::FifoThreshold::Tx33Rx32,
			Radio::AdcRetention::RxFilterAbove325kHz));		// 0x07
		PT_CALL(radio.configureSyncWord(0xb547));		// 0xb5, 0x47
		PT_CALL(radio.configurePacketLength(0x3d));	// 0x3d
		PT_CALL(radio.configurePacketAutomationControl1(
			0, Radio::CrcAutoFlush::Disabled,
			Radio::AppendStatus::Enabled,
			Radio::AddressCheck::Broadcast00));				// 0x06
		PT_CALL(radio.configurePacketAutomationControl0(
			Radio::DataWhitening::Disabled,
			Radio::PacketFormat::Normal,	// TODO: change to RandomTx
			Radio::CrcCalculation::Enabled,
			Radio::PacketLengthConfig::Variable));			// 0x05
		PT_CALL(radio.configureAddress(0xff));		// 0xff
		PT_CALL(radio.configureChannelNumber(0x00));	// 0x00
		if(Settings == Transmission::Fsk34k) {
			PT_CALL(radio.configureIfFrequency(0x08));	// 0x08
		} else if(Settings == Transmission::GFsk250k) {
			PT_CALL(radio.configureIfFrequency(0x0C));
		}
		PT_CALL(radio.configureFrequencyOffset(0x00));// 0x00
		// 433MHz
		PT_CALL(radio.configureBaseFrequency(0x10a762));// 0x10, 0xa7, 0x62
		if(Settings == Transmission::Fsk34k) {
			PT_CALL(radio.configureDataRateAndBandwidth(
				0x83, 0x0a,
				Radio::ChannelBandwidth::XOscOver256));			// 0xca, 0x83
		} else if(Settings == Transmission::GFsk250k) {
			PT_CALL(radio.configureDataRateAndBandwidth(
				0x3b, 0x0d,
				Radio::ChannelBandwidth::XOscOver48));
		}
		PT_CALL(radio.configureModem2(
			Radio::ModulationFormat::GFsk,
			Radio::SyncMode::SyncWord30OutOf32Bits,
			Radio::ManchesterEncoding::Disabled,
			Radio::DigitalDcBlockingFilter::Disabled));		// 0x93
		PT_CALL(radio.configureModem1(0xf8, 0x02));	// 0x22, 0xf8
		if(Settings == Transmission::Fsk34k) {
			PT_CALL(radio.configureDeviation(5, 3));		// 0x35
		} else if(Settings == Transmission::GFsk250k) {
			PT_CALL(radio.configureDeviation(2, 6));
		}
		PT_CALL(radio.configureMainRadioFsm2(7));		// 0x07
		PT_CALL(radio.configureMainRadioFsm1(
			Radio::CcaMode::UnlessReceivingPacket,
			Radio::RxOffMode::Idle,
			Radio::TxOffMode::Idle));							// 0x20
		PT_CALL(radio.configureMainRadioFsm0(
			Radio::FsAutoCallibration::IdleToRxOrTx));			// 0x18
		PT_CALL(radio.configureFrequencyOffsetCompensation(
			Radio::FocBsCsGate::Disabled, Radio::FocPreK::K3));	// 0x16
		PT_CALL(radio.configureBitSynchronization());		// 0x6c
		PT_CALL(radio.configureAgc(
			Radio::MaxDVgaGain::SecondHighestSetting));		// 0x43, 0x40, 0x91
		PT_CALL(radio.configureEvent0Timeout(0x876b));// 0x87, 0x6b
		PT_CALL(radio.configureWakeOnRadio(
			Radio::WorResolution::Period2ToThe15));			// 0xfb
		PT_CALL(radio.configureRxFrontEnd());			// 0x56
		PT_CALL(radio.configureTxFrontEnd());			// 0x10
		PT_CALL(radio.configureFrequencySynthesizerCalibration321(
			0xe9, 0x2a, 0x00));								// 0xe9, 0x2a, 0x00
		PT_CALL(radio.configureFrequencySynthesizerCalibration0(
			0x1f));											// 0x1f
		PT_CALL(radio.configureRcOscillator());			// 0x41, 0x00
		PT_CALL(radio.configureTest(
			Radio::VcoSelectionCalibration::Disabled));		// 0x7f, 0x3f, 0x81, 0x35, 0x09

			XPCC_LOG_DEBUG << XPCC_FILE_INFO << "Configured cc1101." << xpcc::endl;

		// main loop
		while(true){
			// send some data
			PT_CALL(radio.sendData(data, DataSize));
			XPCC_LOG_INFO << ".";
			// update packet count
			packet_count++;
			data[DataSize-4] = (packet_count >> 8 * 3) & 0xff;
			data[DataSize-3] = (packet_count >> 8 * 2) & 0xff;
			data[DataSize-2] = (packet_count >> 8 * 1) & 0xff;
			data[DataSize-1] = (packet_count >> 8 * 0) & 0xff;
			// timeout
			timer.restart(1);
			PT_WAIT_UNTIL(timer.isExpired());
		}

		PT_END();
	}

public:
	struct CC1101Config {
		typedef SpiMaster3    SpiMaster;
		typedef GpioOutputA15 Cs;
		typedef GpioInputB4   Miso;
		typedef GpioInputD6   Gdo0;
		typedef GpioInputD4   Gdo2;
	};

private:
	typedef xpcc::radio::CC1101<CC1101Config> Radio;
	xpcc::Timeout<> timer;
	xpcc::PeriodicTimer<> blinkTimer;
	Radio radio;
	uint32_t packet_count;
	static constexpr size_t DataSize = sizeof(data);
};

MainThread mainThread;


// ----------------------------------------------------------------------------
MAIN_FUNCTION
{
	defaultSystemClock::enable();
	xpcc::cortex::SysTickTimer::enable();

	LedNorth::setOutput(xpcc::Gpio::Low);
	LedSouth::setOutput(xpcc::Gpio::Low);

	// Initialize Usart
	GpioOutputA2::connect(Usart2::Tx);
	GpioInputA3::connect(Usart2::Rx);
	Usart2::initialize<defaultSystemClock, 115200>(10);

	// Print project information
	XPCC_LOG_INFO << "[log-start] " XPCC_BUILD_PROJECT_NAME << xpcc::endl;
	XPCC_LOG_INFO << "[build] " __DATE__            " @ " __TIME__           << xpcc::endl;
	XPCC_LOG_INFO << "[build] " XPCC_BUILD_USER     " @ " XPCC_BUILD_MACHINE << xpcc::endl;
	XPCC_LOG_INFO << "[build] " XPCC_BUILD_COMPILER " @ " XPCC_BUILD_OS      << xpcc::endl;
	XPCC_LOG_INFO << "[git] " XPCC_GIT_SHA_ABBR " "  XPCC_GIT_SUBJECT          << xpcc::endl;
	XPCC_LOG_INFO << "[git] " XPCC_GIT_AUTHOR   " <" XPCC_GIT_AUTHOR_EMAIL ">" << xpcc::endl;


	// Initialize Spi
	MainThread::CC1101Config::Cs::setOutput(xpcc::Gpio::High);
	MainThread::CC1101Config::Miso::connect(SpiMaster3::Miso);
	GpioOutputB5::connect(SpiMaster3::Mosi);
	GpioOutputB3::connect(SpiMaster3::Sck);
	SpiMaster3::initialize<defaultSystemClock, 1125 * kHz1>();
	//SpiMaster3::initialize<defaultSystemClock, 140625>();

	// Initialize Gdos
	MainThread::CC1101Config::Gdo0::setInput();
	MainThread::CC1101Config::Gdo2::setInput();

	while (1)
	{
		mainThread.run();
	}

	return 0;
}
