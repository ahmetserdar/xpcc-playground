// coding: utf-8
/* Copyright (c) 2014, Electronic Kiwi
* All Rights Reserved.
*
* The file is part of the xpcc-playground and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*/

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
