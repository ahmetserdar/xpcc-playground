// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_CC1101_PHY_HPP
	#error	"Don't include this file directly, use 'phy.hpp' instead!"
#endif

namespace xpcc
{
namespace radio
{

template<typename Configuration>
xpcc::co::Result<CC1101PhyBase::InitializeError>
CC1101Phy<Configuration>::initialize()
{
	// FIXME: implement!
	return InitializeError::None;
}


template<typename Configuration>
xpcc::co::Result<uint8_t>
CC1101Phy<Configuration>::readRegister(CC1101PhyBase::Register reg)
{
	// FIXME: implement!
	return 0xff;
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101Phy<Configuration>::readRegister(CC1101PhyBase::Register reg, uint8_t* values, size_t length)
{
	// FIXME: implement!
	return;
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101Phy<Configuration>::writeRegister(CC1101PhyBase::Register reg, uint8_t value)
{
	// FIXME: implement!
	return;
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101Phy<Configuration>::writeRegister(CC1101PhyBase::Register reg, uint8_t* values, size_t length)
{
	// FIXME: implement!
	return;
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101Phy<Configuration>::writeCommand(CC1101PhyBase::Command command)
{
	// FIXME: implement!
	return;
}


}	// namespace radio
}	// namespace cc1101
