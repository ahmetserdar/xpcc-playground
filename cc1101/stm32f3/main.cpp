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



#include "../cc1101/registers.hpp"

// ----------------------------------------------------------------------------
MAIN_FUNCTION
{
	defaultSystemClock::enable();

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

	while (1)
	{
		LedSouth::toggle();
		xpcc::delayMilliseconds(500);
	}

	return 0;
}
