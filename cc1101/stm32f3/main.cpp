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
xpcc::IODeviceWrapper< Usart2 > loggerDevice;

// Set all four logger streams to use the UART
xpcc::log::Logger xpcc::log::debug(loggerDevice);
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::warning(loggerDevice);
xpcc::log::Logger xpcc::log::error(loggerDevice);

// Set the log level
#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::DEBUG

#include <xpcc_git_info.hpp>
#include <xpcc_project_info.hpp>


#include <xpcc/processing/timeout.hpp>
#include <xpcc/processing/periodic_timer.hpp>
#include <xpcc/processing/protothread.hpp>
#include "../cc1101/cc1101.hpp"


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
		e = PT_CALL(radio.initialize(this));
		if(e != Radio::InitializeError::None) {
			XPCC_LOG_ERROR << XPCC_FILE_INFO;
			XPCC_LOG_ERROR << "Error trying to initialize the cc1101: ";
			XPCC_LOG_ERROR << Radio::enumToString(e) << xpcc::endl;
		} else {
			XPCC_LOG_DEBUG << XPCC_FILE_INFO << "Initialized cc1101." << xpcc::endl;
		}
		// Configure Gdo0 as high impedance
		PT_CALL(radio.configureGdo(this,
			Radio::Gdo::Gdo0,
			Radio::GdoInverted::No,
			Radio::GdoSignalSelection::HighImpedance));

		// main loop
		while(true){
			PT_YIELD();
		}

		PT_END();
	}

public:
	struct CC1101Config {
		typedef SpiSimpleMaster3 SpiMaster;
		typedef GpioOutputA15    Cs;
		typedef GpioInputB4      Miso;
		typedef GpioInputD6      Gdo0;
		typedef GpioInputD4      Gdo2;
	};

private:
	typedef xpcc::radio::CC1101<CC1101Config> Radio;
	xpcc::Timeout<> timer;
	xpcc::PeriodicTimer<> blinkTimer;
	Radio radio;
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
	XPCC_LOG_INFO << "[log-start] " XPCC_PROJECT_NAME ": " __DATE__ "@" __TIME__ << xpcc::endl;
	XPCC_LOG_INFO << "[git] " XPCC_GIT_SHA_ABBR " " XPCC_GIT_SUBJECT << xpcc::endl;
	XPCC_LOG_INFO << "[git] " XPCC_GIT_AUTHOR "<" XPCC_GIT_AUTHOR_EMAIL ">" << xpcc::endl;

	// Initialize Spi
	MainThread::CC1101Config::Cs::setOutput(xpcc::Gpio::High);
	MainThread::CC1101Config::Miso::connect(SpiSimpleMaster3::Miso);
	GpioOutputB5::connect(SpiSimpleMaster3::Mosi);
	GpioOutputB3::connect(SpiSimpleMaster3::Sck);
	SpiSimpleMaster3::initialize<defaultSystemClock, 1125 * kHz1>();
	//SpiSimpleMaster3::initialize<defaultSystemClock, 140625>();

	// Initialize Gdos
	MainThread::CC1101Config::Gdo0::setInput();
	MainThread::CC1101Config::Gdo2::setInput();

	while (1)
	{
		mainThread.run();
	}

	return 0;
}
