// coding: utf-8
/* Copyright (c) 2013, Roboterclub Aachen e.V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

// do _not_ use double underscores "__", they are reserved by your **system**!
#ifndef XPCC_{MODULE } _{CLASSNAME } _HPP
#define XPCC_                                                                  \
	{ MODULE }                                                                 \
	_ { CLASSNAME }                                                            \
	_HPP

#include "path_specific_file.hpp"
#include <xpcc/xpcc_file.hpp>

/// global constants used for XPCC internal use are all CAPS
static constexpr uint8_t XPCC_INTERNAL_CONSTANT_IN_ALL_CAPS = 0x04;
/// XPCC defines are also all CAPS
#define XPCC_DEFINE 0x2

// this is almost always the top namespace!
namespace xpcc {
// do not indent namespaces, since it reduces the available horizontal space
// but put a newline here

// by using a struct instead of a namespace
// other classes can inherit from this
// You should use this especially for template classes!
struct Module {
	// put common enums, constexpr here

	/// the answer to life the universe and everything
	static constexpr uint8_t PublicAndScopedConstantInCamelCase = 0x42;

	///@{
	/// This a description of a group of stuff, which will group the following
	/// together in the documentation

	// prefer enum classes over simple enums whenever applicable
	// in this case there are no name collisions of `Stop`

	/// Enum class enums are Capitalized, but **not** in CAPITAL letters!
	enum class
	    // put a newline here
	    Operation : uint8_t {
		    Stop = 0,    ///< Generate a Stop Condition
		    Restart = 1, ///< Generate a Restart
		    Write = 2,   ///< Write data to the slave
		    Read = 3,    ///< Read data from the slave
		};

	/// Further operations after write operation.
	enum class OperationAfterWrite {
		Stop = 0,
		Restart = 1,
		Write = 2,
	};
	///@}
};

/**
 * Put a brief description of this class here ending in a dot or newline.
 *
 * More details about the class here.
 * @see	Reference to other class or similar
 *
 * The author tag should name the person that wrote or added this code.
 * It is then easier to know who to contact.
 *
 * Even when you only adapted this code, put your name there and add this:
 * Adapted from {the code} of {original author}.
 *
 * @author	{Name of author/adapter}
 * @author	{Name of author/contributor}
 * @ingroup	doxygen_group
 */
class ModuleSpecificClass : public ::xpcc::SuperClass, public Module {
	/// one line descriptions should _not_ use /** */
	static void
	    // put a newline here
	    methodNameUsingCamelCase();

	/// efficient inline function
	static inline void methodName1() {
		// remember to put inline functions in the `*.hpp` or better
		// the `*_impl.hpp`
	}

	/**
	 * Short description of functionality
	 *
	 *
	 * @param	arg1
	 *	Say what arg1 is for and use a new line for it
	 * @param	epic
	 *	epic description and don't forget the tab in front
	 * @param	longArgumentName
	 *	long descriptions can be annoying and by putting the description
	 *	in a new line, you gain a lot of space and do not have to cram
	 *	it into the remaining space to the right of the argument name.
	 *
	 * @return	what it returns
	 */
	static bool
	methodName2(uint8_t arg1, EpicEnum epic, uint32_t longArgumentName);
};

// template class
/**
 * Brief description of this class.
 *
 * @tparam	Timer
 *	Describe what Timer is
 *
 * @author	{Name of author}
 * @ingroup	doxygen_group
 */
template <typename Timer> class TemplateClass : public Module {
  public:
	// Do NOT add any enums, structs, constexpr here,
	// that have nothing to do with the dependency on TIMER!
	//
	// Otherwise you MUST provide TIMER whenever you access this enum class:
	// xpcc::TemplateClass<TIMER>::Options::InconvenientEnumElement !
	enum class Options {
		InconvenientEnumElement,
		// options
	};
};

} // namespace xpcc

#endif // XPCC_{MODULE}_{CLASSNAME}_HPP
