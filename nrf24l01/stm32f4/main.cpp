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

using namespace xpcc::stm32;

typedef GpioOutputD15 LedBlue;		// User LED 6

typedef LedBlue Led;

xpcc::IODeviceWrapper< Usart2 > loggerDevice;
xpcc::log::Logger xpcc::log::error(loggerDevice);
xpcc::log::Logger xpcc::log::warning(loggerDevice);
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::debug(loggerDevice);

// Set the log level
#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::DEBUG



static constexpr bool isReceiverNotTransmitter = false;

typedef SystemClock<Pll<ExternalCrystal<MHz8>, MHz48, MHz48> > defaultSystemClock;

struct
Fake8MHzSystemClock
{
	static constexpr int Usart2 = MHz16;
};


MAIN_FUNCTION
{
	//defaultSystemClock::enable();

	Led::setOutput(xpcc::Gpio::Low);

	// Initialize Usart
	GpioOutputA2::connect(Usart2::Tx);
	GpioInputA3::connect(Usart2::Rx, Gpio::InputType::PullUp);
	Usart2::initialize<Fake8MHzSystemClock, 115200>(10);

	XPCC_LOG_INFO << "Nrf24L01+ Test" << xpcc::endl;


	if(isReceiverNotTransmitter) {
	///////////// Receiver /////////////////////////////////////////////////////
		XPCC_LOG_INFO << "acting as receiver" << xpcc::endl;

		uint8_t data_array[4];
		const uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
		const uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};

		nrf24_init();
		XPCC_LOG_DEBUG << "called nrf24_init()..." << xpcc::endl;

		nrf24_config(2,4);
		XPCC_LOG_DEBUG << "called nrf24_config(2,4)..." << xpcc::endl;

		nrf24_tx_address(tx_address);
		XPCC_LOG_DEBUG << "called nrf24_tx_address({0xD7,0xD7,0xD7,0xD7,0xD7})..." << xpcc::endl;

		nrf24_rx_address(rx_address);
		XPCC_LOG_DEBUG << "called nrf24_rx_address({0xE7,0xE7,0xE7,0xE7,0xE7})..." << xpcc::endl;

		while (1)
		{
			if(nrf24_dataReady())
			{
				nrf24_getData(data_array);
				XPCC_LOG_INFO << "Msg: 0x" << xpcc::hex
					<< data_array[0] << data_array[1] << data_array[2]
					<< data_array[3] << xpcc::endl;
			}
			Led::toggle();
			xpcc::delay_ms(100);
		}

	} else {
	///////////// Transmitter //////////////////////////////////////////////////
		XPCC_LOG_INFO << "acting as transmitter" << xpcc::endl;

		uint8_t temp;
		uint8_t q = 0;
		uint8_t data_array[4];
		uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
		uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};


		/* init hardware pins */
		nrf24_init();
		XPCC_LOG_INFO << "nrf24_init() done" << xpcc::endl;

		/* Channel #2 , payload length: 4 */
		nrf24_config(2,4);
		XPCC_LOG_INFO << "nrf24_config(2,4) done" << xpcc::endl;

		/* Set the device addresses */
		nrf24_tx_address(tx_address);
		XPCC_LOG_INFO << "nrf24_tx_address(tx_address) done" << xpcc::endl;

		nrf24_rx_address(rx_address);
		XPCC_LOG_INFO << "nrf24_rx_address(rx_address) done" << xpcc::endl;

		while(1)
		{
			/* Fill the data buffer */
			data_array[0] = 0x00;
			data_array[1] = 0xAA;
			data_array[2] = 0x55;
			data_array[3] = q++;

			/* Automatically goes to TX mode */
			nrf24_send(data_array);
			XPCC_LOG_INFO << "nrf24_send(data_array) done" << xpcc::endl;

			/* Wait for transmission to end */
			while(nrf24_isSending());
			XPCC_LOG_INFO << "nrf24_isSending() done" << xpcc::endl;

			/* Make analysis on last tranmission attempt */
			temp = nrf24_lastMessageStatus();
			XPCC_LOG_INFO << "nrf24_lastMessageStatus() done" << xpcc::endl;

			if(temp == NRF24_TRANSMISSON_OK)
			{
				XPCC_LOG_INFO << "> Tranmission went OK" << xpcc::endl;

			}
			else if(temp == NRF24_MESSAGE_LOST)
			{
				XPCC_LOG_INFO << "> Message is lost ..." << xpcc::endl;
			}

			/* Retranmission count indicates the tranmission quality */
			temp = nrf24_retransmissionCount();
			XPCC_LOG_INFO << "> Retranmission count: " << temp << xpcc::endl;

			/* Optionally, go back to RX mode ... */
			nrf24_powerUpRx();
			XPCC_LOG_INFO << "nrf24_powerUpRx() done" << xpcc::endl;

			/* Or you might want to power down after TX */
			// nrf24_powerDown();

			/* Wait a little ... */
			_delay_ms(10);
			Led::toggle();
		}
	}

	while (1)
	{
		Led::toggle();
		xpcc::delay_ms(100);
	}


}
