#
# Copyright 2011 Free Software Foundation, Inc.
# 
# This file is part of GNU Radio
# 
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

include $(top_srcdir)/Makefile.common

TESTS =
EXTRA_DIST += run_tests.in

if PYTHON
SUBDIRS = utils
TESTS += run_tests

digitaldir = $(grpythondir)/digital

noinst_PYTHON = \
	qa_digital.py			\
	qa_constellation.py		\
	qa_constellation_receiver.py	\
	qa_costas_loop_cc.py

digital_PYTHON = \
	__init__.py	\
	psk.py		\
	dbpsk.py	\
	dqpsk.py	\
	d8psk.py	\
	psk2.py		\
	generic_mod_demod.py	\
	qam.py		\
	bpsk.py		\
	qpsk.py		\
	ofdm.py		\
	pkt.py		\
	modulation_utils2.py

endif