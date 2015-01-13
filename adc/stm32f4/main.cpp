// coding: utf-8
/* Copyright (c) 2014, Electronic Kiwi
* All Rights Reserved.
*
* The file is part of the xpcc-playground and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*/


#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>

// ----------------------------------------------------------------------------
// Settings
static constexpr uint32_t Channels = 4;
static constexpr uint32_t Samples = 64; // per channel
static constexpr uint32_t SamplingFrequency = 8 * 40 * kHz1;

// ----------------------------------------------------------------------------
// Set the log level
#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::DEBUG

#include "../../xpcc/examples/stm32f4_discovery/stm32f4_discovery.hpp"

typedef GpioInputA7 AdcIn0;
typedef GpioInputB0 AdcIn1;
typedef GpioInputB1 AdcIn2;
typedef GpioInputC0 AdcIn3;

xpcc::IODeviceWrapper<Usart2, xpcc::IOBuffer::BlockIfFull> loggerDevice;
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::debug(loggerDevice);

static constexpr uint32_t BufferLength = Samples * Channels;
uint16_t buffer[Samples][Channels];

// ----------------------------------------------------------------------------
MAIN_FUNCTION
{
	defaultSystemClock::enable();

	// initialize Uart2 for XPCC_LOG_INFO
	GpioOutputA2::connect(Usart2::Tx);
	GpioInputA3::connect(Usart2::Rx);
	Usart2::initialize<defaultSystemClock, 115200>(12);

	// print application information
	XPCC_LOG_INFO << "#############################################" << xpcc::endl;
	XPCC_LOG_INFO << "# ADC Demo for STM32F4Discoveryboards       #" << xpcc::endl;
	XPCC_LOG_INFO << "# powered by XPCC                           #" << xpcc::endl;
	XPCC_LOG_INFO << "# https://github.com/roboterclubaachen/xpcc #" << xpcc::endl;
	XPCC_LOG_INFO << "#############################################" << xpcc::endl;
	XPCC_LOG_INFO << "# Sampling Frequency:  " << kHz(SamplingFrequency) << "kHz" << xpcc::endl;
	XPCC_LOG_INFO << "# Number of Channels:  " << Channels << xpcc::endl;
	XPCC_LOG_INFO << "# Samples per Channel: " << Samples << xpcc::endl;
	XPCC_LOG_INFO << "#############################################" << xpcc::endl;

	// initialize Adc1
	Adc1::initialize<defaultSystemClock, MHz10>();	// FIXME: what ADC clock frequency do we want?
	AdcIn0::connect(Adc1::Channel7);
	AdcIn1::connect(Adc1::Channel8);
	AdcIn2::connect(Adc1::Channel9);
	AdcIn3::connect(Adc1::Channel10);
	Adc1::setChannel(AdcIn0::Adc1Channel, Adc1::SampleTime::Cycles3);
	Adc1::addChannel(AdcIn1::Adc1Channel, Adc1::SampleTime::Cycles3);
	Adc1::addChannel(AdcIn2::Adc1Channel, Adc1::SampleTime::Cycles3);
	Adc1::addChannel(AdcIn3::Adc1Channel, Adc1::SampleTime::Cycles3);

	// enable scan mode
	ADC1->CR1 |= ADC_CR1_SCAN;

	// initialize DMA
	Dma2::enable();
	Dma2::Stream0::stop();
	Dma2::Stream0::configure(DmaBase::Channel::Channel0, BufferLength, DmaBase::Priority::VeryHigh);
	Dma2::Stream0::setPeripheralSource(reinterpret_cast<uint16_t*>(const_cast<uint32_t*>(&(ADC1->DR))));
	Dma2::Stream0::setMemoryDestination(&buffer[0][0]);
	Dma2::Stream0::start();

	// enable DMA (DDS enable DMA for following conversions)
	ADC1->CR2 |= ADC_CR2_DDS | ADC_CR2_DMA;

	// Use Timer 1 CC1 event, falling rising edge as Trigger
	ADC1->CR2 |= (0b0000 << 24) | ADC_CR2_EXTEN_0;

	// Setup Timer 1 to use as trigger
	const uint32_t Overflow = defaultSystemClock::Timer1 / SamplingFrequency;
	Timer1::enable();
	GpioOutputA8::connect(Timer1::Channel1);	// for debugging
	Timer1::setMode(Timer1::Mode::UpCounter);
	Timer1::setPrescaler(1);
	Timer1::setOverflow(Overflow);
	Timer1::configureOutputChannel(1, Timer1::OutputCompareMode::Pwm, Overflow / 2);
	Timer1::applyAndReset();
	Timer1::start();
	Timer1::enableOutput();						// necessary to see signal on PA8

	// Wait for DMA to fill up
	while(!Dma2::Stream0::isFinished());

	// Print results
	for(uint32_t channel = 0; channel < Channels; ++channel){
		XPCC_LOG_INFO << "Channel " << channel;
		XPCC_LOG_INFO <<" -------------------------------------------------" << xpcc::endl;
		for(uint32_t sample = 0; sample < Samples; ++sample) {
			int v = buffer[sample][channel] * 20 / 0xfff;
			for(uint8_t jj = 0; jj < v; ++jj) {
				XPCC_LOG_INFO << "#";
			}
			XPCC_LOG_INFO << xpcc::endl;
		}
	}

	while (1){}

	return 0;
}
