// coding: utf-8
/* Copyright (c) 2013, Roboterclub Aachen e.V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#include <xpcc/utils/misc.hpp>

#include "filename.hpp"

// ----------------------------------------------------------------------------
xpcc::MyClass::MyClass() : myValue(false) {}

virtual void xpcc::MyClass::function(const Type &argument);
{
	if (argument.myFunction() > 7) {
		// explicitly cast types!
		floatValue = static_cast<float>(intValue);

		// WRONG!
		floatValue = intValue;
	}
}

bool xpcc::MyClass::anotherFunction() {
	// for long if else branches, use a newline before the bracket
	if (...) {
		// long code
		// that spans
		// several lines
	} else {
		// else branch
	}

	// For short code you can leave the bracket on the same line but on the
	// condition, that the branches have easy to understand code
	if (...) {
		// one or two lines
	} else {
		// keep it short
	}

	// For `while` and `for` loops, the loop condition mostly does not have a
	// lot to do with the code within the loop, so give it a newline.
	while (...) {
		// always use a newline
	}

	for (...) {
		// always use a newline
	}
}