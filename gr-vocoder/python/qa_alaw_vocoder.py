#!/usr/bin/env python
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

from gnuradio import gr, gr_unittest
from vocoder_swig import *

class test_alaw_vocoder (gr_unittest.TestCase):

    def setUp (self):
        self.tb = gr.top_block()
        
    def tearDown (self):
        self.tb = None

    def test001_module_load (self):
        enc = alaw_encode_sb();
        dec = alaw_decode_bs();

if __name__ == '__main__':
    gr_unittest.run(test_alaw_vocoder, "test_alaw_vocoder.xml")
