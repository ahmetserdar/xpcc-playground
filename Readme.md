xpcc-playground
===============

Some random projects, mostly for STM32 microcontrollers, that I wrote using
the [xpcc](https://github.com/roboterclubaachen/xpcc) library.
They aren't exactly examples, but might still be a good starting point
if you want to start your own project using xpcc.

All files are three clause BSD licensed in order to encourage including
them in your own projects. If you want to include them in a GPL project
just write me a message and I will give you permission to relicense.

To try out the projects you need at least a recent g++ for bare metal arm
development. Take the [gcc-arm-embedded](https://launchpad.net/gcc-arm-embedded) from Linaro.
In addition to that you need `scons` as well as the `Jinja2` for `python 2.7`.

`xpcc` is included as a git submodule, so you will need to run:

	git submodule init
	git submodule update

For more help on how to setup your system you can have a look at
[salkinium's guide](https://github.com/salkinium/corona/tree/master/software).
