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


// Allocate 64k bytes Memory
// constexpr uint16_t SampleLength = 0xffff / 2;
constexpr uint16_t SampleLength = 100;
uint16_t samples[SampleLength];

constexpr int SampleFrequency = MHz4;


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

	XPCC_LOG_INFO << "#############################################" << xpcc::endl;
	XPCC_LOG_INFO << "# DMA Demo for STM32F4Discoveryboards       #" << xpcc::endl;
	XPCC_LOG_INFO << "# powered by XPCC                           #" << xpcc::endl;
	XPCC_LOG_INFO << "# https://github.com/roboterclubaachen/xpcc #" << xpcc::endl;
	XPCC_LOG_INFO << "#############################################" << xpcc::endl;
	XPCC_LOG_INFO << "# Sampling Freqeuncy: " << kHz(SampleFrequency) << "kHz" << xpcc::endl;

	// Init Gpio Inputs
	GpioInputD0::setInput(Gpio::InputType::Floating);
	GpioInputD1::setInput(Gpio::InputType::Floating);
	GpioInputD2::setInput(Gpio::InputType::Floating);
	GpioInputD3::setInput(Gpio::InputType::Floating);
	GpioInputD4::setInput(Gpio::InputType::Floating);
	GpioInputD5::setInput(Gpio::InputType::Floating);
	GpioInputD6::setInput(Gpio::InputType::Floating);
	GpioInputD7::setInput(Gpio::InputType::Floating);
	GpioInputD8::setInput(Gpio::InputType::Floating);
	GpioInputD9::setInput(Gpio::InputType::Floating);
	GpioInputD10::setInput(Gpio::InputType::Floating);
	GpioInputD11::setInput(Gpio::InputType::Floating);
	GpioInputD12::setInput(Gpio::InputType::Floating);
	GpioInputD13::setInput(Gpio::InputType::Floating);
	GpioInputD14::setInput(Gpio::InputType::Floating);
	GpioInputD15::setInput(Gpio::InputType::Floating);


	for(uint32_t i = 0; i < SampleLength; ++i) {
		samples[i] = 0xffff;
	}

	Dma1::enable();
	Dma1::Stream6::stop();
	Dma1::Stream6::configure(SampleLength, DmaBase::Priority::VeryHigh);
	Dma1::Stream6::setPeripheralSource(const_cast<uint16_t*>(&(GPIOD->IDR)));
	Dma1::Stream6::setMemoryDestination(&samples[0]);

	// Start Timer1
	constexpr uint16_t OVERFLOW = (defaultSystemClock::Timer1 / SampleFrequency) - 1;
	constexpr uint16_t COMPARE  = (OVERFLOW / 2) + 1;	// 50% duty cycle
	Timer1::enable();
	Timer1::setMode(Timer1::Mode::UpCounter);
	Timer1::setPrescaler(1);	// 168MHz
	Timer1::setOverflow(OVERFLOW);
	Timer1::configureOutputChannel(3, Timer1::OutputCompareMode::Pwm, COMPARE);
	GpioOutputA10::configure(Gpio::OutputType::PushPull);
	GpioOutputA10::connect(Timer1::Channel3);
	Timer1::applyAndReset();
	Timer1::start();
	Timer1::enableOutput();

	// Use Timer1 Channel3 as DMA Trigger
	TIM1->DIER |= TIM_DIER_CC3DE;	// CC 3 DMA Request enable


	// Start Timer2 as sample signal
	constexpr uint16_t T2_OVERFLOW = (defaultSystemClock::Timer2 / kHz100 / 4) + 1;
	constexpr uint16_t T2_COMPARE  = (T2_OVERFLOW / 2) + 1;	// 50% duty cycle
	Timer2::enable();
	Timer2::setMode(Timer1::Mode::UpCounter);
	Timer2::setPrescaler(1);
	Timer2::setOverflow(T2_OVERFLOW);
	Timer2::configureOutputChannel(4, Timer2::OutputCompareMode::Pwm, T2_COMPARE);
	GpioOutputB11::configure(Gpio::OutputType::PushPull);
	GpioOutputB11::connect(Timer2::Channel4);
	Timer2::applyAndReset();
	Timer2::start();


	LedNorth::set();
	Dma1::Stream6::start();
	while(!Dma1::Stream6::isFinished());
	LedNorth::reset();

	// Output Sampled Data
	for(int i = 0; i < SampleLength; ++i) {
		uint16_t sample = samples[i];
		for(int bit = 0; bit < 16; ++bit) {
			if(sample & 0x8000) {
				XPCC_LOG_INFO << "1";
			} else {
				XPCC_LOG_INFO << "0";
			}
			sample <<= 1;
		}
		XPCC_LOG_INFO << xpcc::endl;
	}

	while (1)
	{
		LedSouth::toggle();
		xpcc::delayMilliseconds(500);
	}

	return 0;
}
