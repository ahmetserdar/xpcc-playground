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
// Set the log level
#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::DEBUG

#include "../../xpcc/examples/stm32f4_discovery/stm32f4_discovery.hpp"

typedef GpioInputA7 AdcIn0;
typedef GpioInputB0 AdcIn1;
typedef GpioInputB1 AdcIn2;
typedef GpioInputC0 AdcIn3;

xpcc::IODeviceWrapper<Usart2> loggerDevice;
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::debug(loggerDevice);

static constexpr uint32_t BufferLength = 1024;
uint16_t buffer[BufferLength];

static  void
printBits(uint32_t v, int8_t max, int8_t min)
{
	for(int8_t ii = max; ii >= min; --ii) {
		if(ii >= 10) {
			XPCC_LOG_DEBUG << "| " << ii;
		} else {
			XPCC_LOG_DEBUG << "|  " << ii;
		}
	}
	XPCC_LOG_DEBUG << "|" << xpcc::endl << "+";
	for(int8_t ii = max; ii >= min; --ii) {
		XPCC_LOG_DEBUG << "---+";
	}
	XPCC_LOG_DEBUG << xpcc::endl;
	for(int8_t ii = max; ii >= min; --ii) {
		bool b = static_cast<bool>(v & (1<<ii));
		XPCC_LOG_DEBUG << "| " << b << " ";
	}
	XPCC_LOG_DEBUG << "|" << xpcc::endl;
}

static void
printRegister(char* name, const volatile long unsigned int reg)
{
	XPCC_LOG_DEBUG << name << xpcc::endl;
	printBits(static_cast<uint32_t>(reg), 31, 16);
	printBits(static_cast<uint32_t>(reg), 15,  0);
}

// ----------------------------------------------------------------------------
MAIN_FUNCTION
{
	defaultSystemClock::enable();


	for(uint32_t ii = 0; ii < BufferLength; ++ii) {
		buffer[ii] = 0xff; // signature bits
	}

	// initialize Uart2 for XPCC_LOG_INFO
	GpioOutputA2::connect(Usart2::Tx);
	GpioInputA3::connect(Usart2::Rx);
	Usart2::initialize<defaultSystemClock, 115200>(12);

	// initialize Adc1
	Adc1::initialize(Adc1::Prescaler::Div8);
	AdcIn0::connect(Adc1::Channel7);
	AdcIn1::connect(Adc1::Channel8);
	AdcIn2::connect(Adc1::Channel9);
	AdcIn3::connect(Adc1::Channel10);
	Adc1::setChannel(AdcIn0::Adc1Channel, Adc1::SampleTime::Cycles15);
	Adc1::addChannel(AdcIn1::Adc1Channel, Adc1::SampleTime::Cycles15);
	Adc1::addChannel(AdcIn2::Adc1Channel, Adc1::SampleTime::Cycles15);
	Adc1::addChannel(AdcIn3::Adc1Channel, Adc1::SampleTime::Cycles15);

	// enable scan mode
	ADC1->CR1 |= ADC_CR1_SCAN;


	// initialize DMA
	Dma2::enable();
	Dma2::Stream0::stop();
	Dma2::Stream0::configure(DmaBase::Channel::Channel0, 4, DmaBase::Priority::VeryHigh);
	Dma2::Stream0::setPeripheralSource(reinterpret_cast<uint16_t*>(const_cast<uint32_t*>(&(ADC1->DR))));
	Dma2::Stream0::setMemoryDestination(&buffer[0]);
	Dma2::Stream0::start();

	// enable DMA (DDS enable DMA for following conversions)
	ADC1->CR2 |= ADC_CR2_DDS | ADC_CR2_DMA; // ADC_CR2_DDS | 

	//printRegister("ADC1->SQR1", ADC1->SQR1);
	//printRegister("ADC1->SQR3", ADC1->SQR3);
	//printRegister("ADC1->CR1",  ADC1->CR1);
	//printRegister("ADC1->SMPR2", ADC1->SMPR2);
	//printRegister("ADC1->SMPR1", ADC1->SMPR1);

	// Sample ADC
	Adc1::startConversion();
	xpcc::delay_ms(200);
	Adc1::startConversion();
	xpcc::delay_ms(200);

	for(uint8_t ii = 0; ii < 4; ++ii) {
		XPCC_LOG_INFO << "buffer[" << ii << "]" << xpcc::endl;
		int adcValue = buffer[ii];
		XPCC_LOG_INFO << "adcValue=" << adcValue;
		float voltage = adcValue * 3.3 / 0xfff;
		XPCC_LOG_INFO << " voltage=" << static_cast<int>(voltage*1000) << "mV" << xpcc::endl;
	}

	while (1)
	{

	}

	return 0;
}
