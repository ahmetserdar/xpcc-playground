// coding: utf-8
/* Copyright (c) 2014, Electronic Kiwi
* All Rights Reserved.
*
* The file is part of the xpcc-playground and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*/

#include <xpcc/architecture.hpp>
#include "../../xpcc/examples/stm32f4_discovery/stm32f4_discovery.hpp"

#include "../functions.hpp"

typedef LedBlue Led;

MAIN_FUNCTION
{
	defaultSystemClock::enable();

	Led::setOutput(xpcc::Gpio::Low);

	// Enable DAC Clock
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	// Reset DAC
	RCC->APB1RSTR |=  RCC_APB1RSTR_DACRST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_DACRST;

	// setAnalog for DAC Output Pin....
	GpioInputA4::connect(Adc1::Channel4);

	// Enable DAC
	DAC->CR |= DAC_CR_EN1 | DAC_CR_BOFF1;

	uint16_t ii = 0;
	while (1)
	{
		DAC->DHR12R1 = SINE[ii];
		++ii;
		if(ii >= SINE_LENGTH) ii = 0;
		Led::toggle();
	}

	return 0;
}
