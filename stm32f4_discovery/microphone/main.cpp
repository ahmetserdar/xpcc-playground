// coding: utf-8
/* Copyright (c) 2014, Electronic Kiwi
* All Rights Reserved.
*
* The file is part of the xpcc-playground and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*/

#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>
#include "../../xpcc/examples/stm32f4_discovery/stm32f4_discovery.hpp"

xpcc::IODeviceWrapper<Usart2, xpcc::IOBuffer::BlockIfFull> loggerDevice;
xpcc::log::Logger xpcc::log::debug(loggerDevice);
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::warning(loggerDevice);
xpcc::log::Logger xpcc::log::error(loggerDevice);

// Set the log level
#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::DEBUG

#include <xpcc_git_info.hpp>
#include <xpcc_build_info.hpp>

typedef GpioOutputB10 MP45DT02Clock;
typedef GpioInputC3 MP45DT02Dout;



// 32 bits are sampled with 32 kHz
// =>  second of samples => 32000 * 32
static constexpr size_t NumberOf1BitSamplesPerSecond = 32000 * 32;
// sample for 1/ 50 of a second => ten periods of 500Hz test wave
static constexpr size_t SampleLength = NumberOf1BitSamplesPerSecond / 50;


size_t buffer_index;
uint16_t buffer[SampleLength / 16];
bool sampling_in_progress;

static constexpr uint32_t PllM = Stm32F2F4PllSettings<MHz192, MHz8, MHz168, MHz48>::PllM;
static constexpr uint32_t PllN = Stm32F2F4PllSettings<MHz192, MHz8, MHz168, MHz48>::PllN;
static constexpr uint32_t PllP = Stm32F2F4PllSettings<MHz192, MHz8, MHz168, MHz48>::PllP;
static constexpr uint32_t PllQ = Stm32F2F4PllSettings<MHz192, MHz8, MHz168, MHz48>::PllQ;

///
static inline void
initializeI2s()
{
	// connect pins
	GpioOutputB10::connect(SpiMaster2::Sck);
	// technically used as an input here, but since the peripheral decides
	// on data direction it does not matter
	GpioOutputC3::connect(SpiMaster2::Mosi);

	// TODO: remove when debugging is over
	GpioOutputB12::connect(SpiMaster2::Nss);


	// generate i2s clock with dedicated PLL
	const uint32_t N = 258 / 2;		// this should lead to a 86 MHz
	const uint32_t R =   3;			// I2S clock
	RCC->PLLI2SCFGR = ((N & 0x1ff) << 6) | ((R & 0x07) << 28);
	RCC->CR |= RCC_CR_PLLI2SON;
	while(!(RCC->CR & RCC_CR_PLLI2SRDY))
		;


	// enable clock
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	// reset spi
	RCC->APB1RSTR |=  RCC_APB1RSTR_SPI2RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

	// word width
	uint32_t word_width = 16;
	uint32_t audio_f    = 32000;

	// wanted bitrate: 1.024 MHz => 64 * 16kHz
	// i.e. 64 oversampling

	// configure i2s
	const uint32_t i2s_clk = (((MHz8 / PllM) * N) / R);
	uint32_t div = ((i2s_clk / (2 * word_width) * 10 / audio_f) + 5) / 10;

	// extract odd bit
	uint32_t odd = (div & 1);
	div = div / 2;
	SPI2->I2SPR = (odd << 8) | (0xff & div);
	SPI2->I2SCFGR = SPI_I2SCFGR_I2SMOD   |
	//                SPI_I2SCFGR_I2SE     |	// do not enable just yet
	                SPI_I2SCFGR_I2SCFG_1 | SPI_I2SCFGR_I2SCFG_0 |	// MasterRx
	                SPI_I2SCFGR_I2SSTD_1 |							// LSB justified
	                SPI_I2SCFGR_CKPOL    ;							// CPOL High

	// initialize interrupt
	SpiHal2::enableInterruptVector(true, 1);
}

static inline void
startI2s()
{
	buffer_index = 0;
	//uint16_t temp = SPI2->DR;	// make sure that receive register is empty
	SpiHal2::enableInterrupt(SpiHal2::Interrupt::RxBufferNotEmpty);
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE;
	sampling_in_progress = true;
}


extern "C" void
SPI2_IRQHandler()
{
	if(SpiHal2::getInterruptFlags() & SpiHal2::InterruptFlag::RxBufferNotEmpty) {
		if(buffer_index < (sizeof(buffer)/sizeof(buffer[0]))) {
			buffer[buffer_index] = SPI2->DR;
			++buffer_index;
		} else {
			uint16_t temp = SPI2->DR;	// clear flag
			SpiHal2::disableInterrupt(SpiHal2::Interrupt::RxBufferNotEmpty);
			SPI2->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
			sampling_in_progress = false;
		}
	}
}

static inline void
printBitstream()
{
	for(size_t ii = 0; ii < (sizeof(buffer)/sizeof(buffer[0])); ii++) {
//	for(size_t ii = 0; ii < 8; ii++) {
		uint16_t value = buffer[ii];
		for(unsigned int bb = 0; bb < sizeof(buffer[0])*8; bb++) {
			if(value & (1<<15)) XPCC_LOG_INFO << "1";
			else                XPCC_LOG_INFO << "0";
			value = (value << 1) & (0xffff);
		}
	}
}

MAIN_FUNCTION
{
	defaultSystemClock::enable();

	// Initialize Usart
	GpioOutputA2::connect(Usart2::Tx);
	GpioInputA3::connect(Usart2::Rx);
	Usart2::initialize<defaultSystemClock, 115200>(10);

	// Print project information
	XPCC_LOG_INFO << "[log-start] " XPCC_BUILD_PROJECT_NAME << xpcc::endl;
	XPCC_LOG_INFO << "[build] " __DATE__            " @ " __TIME__           << xpcc::endl;
	XPCC_LOG_INFO << "[build] " XPCC_BUILD_USER     " @ " XPCC_BUILD_MACHINE << xpcc::endl;
	XPCC_LOG_INFO << "[build] " XPCC_BUILD_COMPILER " @ " XPCC_BUILD_OS      << xpcc::endl;
	XPCC_LOG_INFO << "[git] " XPCC_GIT_SHA_ABBR " "  XPCC_GIT_SUBJECT          << xpcc::endl;
	XPCC_LOG_INFO << "[git] " XPCC_GIT_AUTHOR   " <" XPCC_GIT_AUTHOR_EMAIL ">" << xpcc::endl;

	LedBlue::setOutput(xpcc::Gpio::Low);

	initializeI2s();

	XPCC_LOG_DEBUG << "initialized i2s" << xpcc::endl;

	// take some samples before output
//	for(int ii = 0; ii < 4; ii++) {
//		startI2s();
//		while(sampling_in_progress)
//			;
//	}
	startI2s();
	while(sampling_in_progress)
		;
	startI2s();
	while(sampling_in_progress)
		;
	startI2s();
	while(sampling_in_progress)
		;
	startI2s();
	while(sampling_in_progress)
		;
	startI2s();
	while(sampling_in_progress)
		;
	startI2s();
	while(sampling_in_progress)
		;
	startI2s();
	while(sampling_in_progress)
		;
	startI2s();
	while(sampling_in_progress)
		;
	startI2s();
	while(sampling_in_progress)
		;
	startI2s();
	while(sampling_in_progress)
		;


	XPCC_LOG_DEBUG << "sampled i2s" << xpcc::endl;

	XPCC_LOG_DEBUG << xpcc::bin << buffer[0] << xpcc::endl;
	XPCC_LOG_DEBUG << xpcc::bin << buffer[1] << xpcc::endl;
	XPCC_LOG_DEBUG << xpcc::bin << buffer[2] << xpcc::endl;
	XPCC_LOG_DEBUG << xpcc::bin << buffer[3] << xpcc::endl;
	XPCC_LOG_DEBUG << xpcc::bin << buffer[4] << xpcc::endl;
	XPCC_LOG_DEBUG << xpcc::bin << buffer[5] << xpcc::endl;
	XPCC_LOG_DEBUG << xpcc::bin << buffer[6] << xpcc::endl;
	XPCC_LOG_DEBUG << xpcc::bin << buffer[7] << xpcc::endl;

	XPCC_LOG_INFO << xpcc::endl << xpcc::endl;
	printBitstream();

	while (1)
	{
		xpcc::delayMilliseconds(500);
		LedBlue::toggle();
	}

	return 0;
}
