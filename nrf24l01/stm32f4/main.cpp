// coding: utf-8
/* Copyright (c) 2014, Electronic Kiwi, daniel-k
* All Rights Reserved.
*
* The file is part of the xpcc-playground and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*/

#include <xpcc/architecture.hpp>
#include <nrf24.hpp>
#include "../../xpcc/examples/stm32f4_discovery/stm32f4_discovery.hpp"


typedef LedBlue Led;

MAIN_FUNCTION
{
	defaultSystemClock::enable();

	Led::setOutput(xpcc::Gpio::Low);


	while (1)
	{
		Led::toggle();
		xpcc::delay_ms(500);
	}
}
