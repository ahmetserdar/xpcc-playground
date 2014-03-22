#include <xpcc/architecture.hpp>
#include "../../../xpcc/examples/stm32f3_discovery/stm32f3_discovery.hpp"

/* In this project Timer1 is used to first start Timer4 and then,
 * after a predefined time, start Timer2.
 * This makes it possible to start both of them with an arbitrary phase shift.
 * You can use your oscilloscope to observe the result.
 */

// Please note: this is only accurate if the frequency of the timer
// peripheral is a multiple of the desired output frequency.
static constexpr uint32_t Timer4Frequency = 320000;
static constexpr uint32_t Timer2Frequency = 320000;
// Delay between starting the two timers. Resolution when running on the
// "defaultSystemClock": 1/72MHz
// If Timer4Frequency == Timer2Frequency this translates into a constant
// phase shift.
static constexpr uint32_t Delay = 100; // Minimum: about 100 to work reliably


MAIN_FUNCTION
{
	defaultSystemClock::enable();

	// configure Timer4
	Timer4::enable();
	// SlaveModeTrigger: Internal0 is Timer1 (see STM32F3 reference manual)
	Timer4::setMode(Timer4::Mode::UpCounter, Timer4::SlaveMode::Trigger,
		Timer4::SlaveModeTrigger::Internal0);
	Timer4::setOverflow(defaultSystemClock::Timer4 / Timer4Frequency - 1);
	// for shows
	GpioOutputB7::connect(Timer4::Channel2);
	// 50% duty cycle
	Timer4::configureOutputChannel(2, Timer4::OutputCompareMode::Pwm,
		defaultSystemClock::Timer4 / Timer4Frequency / 2);
	Timer4::applyAndReset();	// this does NOT start the timer

	// configure Timer2
	Timer2::enable();
	// SlaveModeTrigger: Internal0 is Timer1 (see STM32F3 reference manual)
	Timer2::setMode(Timer2::Mode::UpCounter, Timer2::SlaveMode::Disabled,
		Timer2::SlaveModeTrigger::Internal0);
	Timer2::setOverflow(defaultSystemClock::Timer2 / Timer2Frequency - 1);
	// for shows
	GpioOutputA1::connect(Timer2::Channel2);
	// 50% duty cycle
	Timer2::configureOutputChannel(2, Timer2::OutputCompareMode::Pwm,
		defaultSystemClock::Timer2 / Timer2Frequency / 2);
	Timer2::applyAndReset();	// this does NOT start the timer

	// configure Timer1
	Timer1::enable();
	// use channel1 compare event as trigger
	Timer1::setMode(Timer1::Mode::UpCounter, Timer1::SlaveMode::Disabled,
		Timer1::SlaveModeTrigger::Internal0,
		Timer1::MasterMode::CompareOc1Ref);
	// set delay, running on the default clock, the resolution is 1/72MHz
	Timer1::setOverflow(Delay - 1);
	// for shows
	GpioOutputA8::connect(Timer1::Channel1);
	Timer1::configureOutputChannel(1, Timer1::OutputCompareMode::Pwm, Delay / 2);
	// for advanced timers (Timer1 and Timer2) output needs to be enabled explicitly
	Timer1::enableOutput();
	Timer1::applyAndReset();	// this does NOT start the timer
	Timer1::start();			// this does

	// now poll Timer4 until it is started
	while(Timer4::getValue() == 0);
	//LedEast::setOutput(xpcc::Gpio::High);
	// then enable trigger for Timer2
	Timer2::setMode(Timer2::Mode::UpCounter, Timer2::SlaveMode::Trigger,
		Timer2::SlaveModeTrigger::Internal0);
	Timer2::applyAndReset();
	//LedEast::setOutput(xpcc::Gpio::Low);
	// now poll Timer2 until it is started
	while(Timer2::getValue() == 0);

	// then Timer1 can be disabled, it is no longer needed
	Timer1::pause();
	Timer1::disable();


	while (1){}

	return 0;
}
