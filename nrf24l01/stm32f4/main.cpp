// coding: utf-8
/* Copyright (c) 2014, Electronic Kiwi, daniel-k
* All Rights Reserved.
*
* The file is part of the xpcc-playground and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*/

#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>
#include <nrf24.hpp>
#include "../../xpcc/examples/stm32f4_discovery/stm32f4_discovery.hpp"



xpcc::IODeviceWrapper< Usart2 > loggerDevice;
xpcc::log::Logger xpcc::log::error(loggerDevice);
xpcc::log::Logger xpcc::log::warning(loggerDevice);
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::debug(loggerDevice);

// Set the log level
#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::DEBUG

typedef LedBlue Led;

static constexpr bool isReceiverNotTransmitter = true;

MAIN_FUNCTION
{
	defaultSystemClock::enable();

	Led::setOutput(xpcc::Gpio::Low);

	// Initialize Usart
	GpioOutputA2::connect(Usart2::Tx);
	GpioInputA3::connect(Usart2::Rx, Gpio::InputType::PullUp);
	Usart2::initialize<defaultSystemClock, 115200>(10);

	XPCC_LOG_INFO << "Nrf24L01+ Test" << xpcc::endl;


	if(isReceiverNotTransmitter) {
	///////////// Receiver /////////////////////////////////////////////////////
		XPCC_LOG_INFO << "acting as receiver" << xpcc::endl;
		while (1)
		{
			Led::toggle();
			xpcc::delay_ms(500);
		}
	} else {
	///////////// Transmitter //////////////////////////////////////////////////
		XPCC_LOG_INFO << "acting as transmitter" << xpcc::endl;
		while (1)
		{
			Led::toggle();
			xpcc::delay_ms(100);
		}
	}



}
