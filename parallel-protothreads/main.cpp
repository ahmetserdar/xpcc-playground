#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>

#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::INFO


#include <xpcc/processing/timer/timeout.hpp>
#include <xpcc/processing/protothread.hpp>
#include <cstdlib>	// std::rand

enum class
Color : uint8_t
{
	Black   = 0,
	Red     = 1,
	Green   = 2,
	Yellow  = 3,
	Blue    = 4,
	Magenta = 5,
	Cyan    = 6,
	White  = 7,
};


xpcc::pc::Terminal term;
xpcc::IOStream xout(term);

class
CountThread : public xpcc::pt::Protothread
{
public:
	CountThread(const char* name, Color color)
		: name(name), color(color), number(0) {}

	/// Needs to be called as often as possible.
	bool
	run()
	{
		PT_BEGIN();
		while(true) {
			timeout.restart((std::rand() / (RAND_MAX / 500)) + 250);
			PT_WAIT_UNTIL(timeout.execute());
			xout << "\033[3" << static_cast<uint8_t>(color) << "m"
			     << name << ": " << number
			     << "\033[0m" << xpcc::endl;
			number++;
		}
		PT_END();
	}

private:
	const char* name;
	Color color;
	int number;
	xpcc::Timeout timeout;
};

class
MainThread : public xpcc::pt::Protothread
{
public:
	MainThread()
		: countThread1("CountThread1", Color::Red),
		  countThread2("CountThread2", Color::Blue)
	{
		// Stop the two threads (this is somewhat unintuetive)
		countThread1.stop();
		countThread2.stop();
	}

	/// Needs to be called as often as possible.
	bool
	run()
	{
		// Run subthreads
		// this is awkward
		countThread1.run();
		countThread2.run();

		PT_BEGIN();
		// use constant seed for reproducibility
		std::srand(37894);
		// Start Two Threads
		countThread1.restart();
		countThread2.restart();
		// Wait for the two threads to exit
		PT_WAIT_UNTIL(!countThread1.isRunning() && !countThread1.isRunning());
		PT_END();
	}

private:
	CountThread countThread1;
	CountThread countThread2;
};



int
main()
{
	MainThread m;
	while(m.run());
	return 0;
}
