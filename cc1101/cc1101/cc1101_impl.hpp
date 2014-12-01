// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_CC1101_HPP
	#error	"Don't include this file directly, use 'cc1101.hpp' instead!"
#endif

namespace xpcc
{
namespace radio
{

template<typename Configuration>
xpcc::co::Result<CC1101Base::InitializeError>
CC1101<Configuration>::initialize(void *ctx)
{
	CO_BEGIN(ctx);

	// FIXME: implement!
	CO_END_RETURN(InitializeError::InvalidSomething);
}


template<typename Configuration>
xpcc::co::Result<uint8_t>
CC1101<Configuration>::readRegister(void *ctx, CC1101Base::Register reg)
{
	CO_BEGIN(ctx);

	// FIXME: implement!

	CO_END_RETURN(0xff);
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::readRegister(void *ctx, CC1101Base::Register reg, uint8_t* values, size_t length)
{
	CO_BEGIN(ctx);

	// FIXME: implement!
	CO_END();
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::writeRegister(void *ctx, CC1101Base::Register reg, uint8_t value)
{
	CO_BEGIN(ctx);

	// FIXME: implement!
	CO_END();
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::writeRegister(void *ctx, CC1101Base::Register reg, uint8_t* values, size_t length)
{
	CO_BEGIN(ctx);

	// FIXME: implement!
	CO_END();
}


template<typename Configuration>
xpcc::co::Result<void>
CC1101<Configuration>::writeCommand(void *ctx, CC1101Base::Command command)
{
	CO_BEGIN(ctx);

	// FIXME: implement!
	CO_END();
}


}	// namespace radio
}	// namespace cc1101
