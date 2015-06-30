#include <xpcc/architecture.hpp>
#include "../../../xpcc/examples/stm32f3_discovery/stm32f3_discovery.hpp"



MAIN_FUNCTION
{
	defaultSystemClock::enable();

	// configure Timer1
	Timer1::enable();
	Timer1::setMode(Timer1::Mode::UpCounter);
	Timer1::setPrescaler(defaultSystemClock::Timer1 / MHz1);
	Timer1::setOverflow(20 * 1000);	// 20ms
	// for shows
	GpioOutputA8::connect(Timer1::Channel1);
	Timer1::configureOutputChannel(1, Timer1::OutputCompareMode::Pwm, 1300);
	GpioOutputA9::connect(Timer1::Channel2);
	Timer1::configureOutputChannel(2, Timer1::OutputCompareMode::Pwm, 1300);
	GpioOutputA10::connect(Timer1::Channel3);
	Timer1::configureOutputChannel(3, Timer1::OutputCompareMode::Pwm, 1300);
	// for advanced timers (Timer1 and Timer2) output needs to be enabled explicitly
	Timer1::enableOutput();
	Timer1::applyAndReset();	// this does NOT start the timer
	Timer1::start();			// this does


	while (1) {
		

	}

	return 0;
}
