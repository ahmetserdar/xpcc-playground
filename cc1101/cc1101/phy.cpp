// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#include "phy.hpp"

namespace xpcc
{
namespace radio
{

const char*
CC1101PhyBase::enumToString(CC1101PhyBase::InitializeError e)
{
	switch(e) {
	case InitializeError::None:
		return "None";
	case InitializeError::InvalidSomething:
		return "InvalidSomething";
	default:
		return "Unknown";
	}
}

}	// namespace radio
}	// namespace cc1101
