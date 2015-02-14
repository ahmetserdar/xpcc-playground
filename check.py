#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Check
# =======================
# This script will try to compile all projects in this repository.
#
# @author: eKiwi <electron.kiwi@gmail.com>

# directories to check for SConstruct files
DIRECTORIES = ['adc', 'cc1101', 'dac', 'dma', 'funtenna', 'nrf24l01', 'parallel-protothreads', 'stm32f4_discovery', 'timer']

import os, sys, subprocess

class Checker(object):

	def __init__(self):
		if hasattr(sys.stdout, "isatty") and sys.stdout.isatty() or \
			('TERM' in os.environ and os.environ['TERM']=='ANSI'):
			self.ok   = "\033[0;32m" + "OK"   + "\033[0m"
			self.fail = "\033[0;31m" + "FAIL" + "\033[0m"
		else:
			self.ok   = "OK"
			self.fail = "FAIL"
		self.report = []
		self.everything_ok = True

	def run(self, dirs):
		if not isinstance(dirs, list):
			dirs = [dirs]
		for d in dirs:
			for path, directories, files in os.walk(d):
				if 'SConstruct' in files and 'project.cfg' in files:
					self._check(path)

	def printreport(self):
		print("\nRESULTS:\n")
		print('\n'.join(self.report))
		if self.everything_ok:
			print("\n{0}!".format(self.ok))
			return 0
		else:
			print("\n{0}!".format(self.fail))
			return 1

	def _check(self, path):
		exitStatus = subprocess.call(['scons', '-Q', '-C', path, 'build'])
		if exitStatus == 0:
			self.report.append("check: {0} -> {1}".format(path, self.ok))
		else:
			self.report.append("check: {0} -> {1}".format(path, self.fail))
			self.everything_ok = False


if __name__ == "__main__":
	c = Checker()
	c.run(DIRECTORIES)
	sys.exit(c.printreport())
