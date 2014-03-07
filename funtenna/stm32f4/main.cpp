// coding: utf-8
/* Copyright (c) 2014, Electronic Kiwi
* All Rights Reserved.
*
* The file is part of the xpcc-playground and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*/

#include <xpcc/architecture.hpp>
#include "../../xpcc/examples/stm32f4_discovery/stm32f4_discovery.hpp"

static inline void longTone() {
	Timer4::start();
	xpcc::delay_ms(700);
	Timer4::pause();
	xpcc::delay_ms(200);
}

static inline void shortTone() {
	Timer4::start();
	xpcc::delay_ms(300);
	Timer4::pause();
	xpcc::delay_ms(200);
}

// ----------------------------------------------------------------------------
MAIN_FUNCTION
{
	defaultSystemClock::enable();

	LedGreen::connect(Timer4::Channel1);
	LedGreen::configure(Gpio::OutputType::PushPull, Gpio::OutputSpeed::MHz100);

	Timer4::enable();
	Timer4::setMode(Timer4::Mode::UpCounter);

	// 168 MHz / 1 / 4 ~ 42MHz
	Timer4::setPrescaler(1);
	Timer4::setOverflow(1);
	Timer4::configureOutputChannel(1, Timer4::OutputCompareMode::Pwm, 1);
	Timer4::applyAndReset();

	while (1)
	{
		longTone();
		shortTone();
		xpcc::delay_ms(700);
		longTone();
		shortTone();
		shortTone();
		shortTone();
		xpcc::delay_ms(700);
		longTone();
		shortTone();
		longTone();
		shortTone();
		xpcc::delay_ms(700);
		xpcc::delay_ms(2000);
	}

	return 0;
}
