#!/usr/bin/env python
#
# Copyright 2007,2008 Free Software Foundation, Inc.
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

import math
from numpy import fft
from gnuradio import gr

class ofdm_sync_pn(gr.hier_block2):
    def __init__(self, fft_length, cp_length, kstime, threshold, logging=False):
        """
        OFDM synchronization using PN Correlation:
        T. M. Schmidl and D. C. Cox, "Robust Frequency and Timing
        Synchonization for OFDM," IEEE Trans. Communications, vol. 45,
        no. 12, 1997.
        """
        
	gr.hier_block2.__init__(self, "ofdm_sync_pn",
				gr.io_signature(1, 1, gr.sizeof_gr_complex), # Input signature
                                gr.io_signature2(2, 2, gr.sizeof_float, gr.sizeof_char)) # Output signature

        self.input = gr.add_const_cc(0)

        # PN Sync

        # Create a delay line
        self.delay = gr.delay(gr.sizeof_gr_complex, fft_length/2)

        # Correlation from ML Sync
        self.conjg = gr.conjugate_cc();
        self.corr = gr.multiply_cc();

        # Create a moving sum filter for the corr output
        if 1:
            moving_sum_taps = [1.0 for i in range(fft_length//2)]
            self.moving_sum_filter = gr.fir_filter_ccf(1,moving_sum_taps)
        else:
            moving_sum_taps = [complex(1.0,0.0) for i in range(fft_length//2)]
            self.moving_sum_filter = gr.fft_filter_ccc(1,moving_sum_taps)

        # Create a moving sum filter for the input
        self.inputmag2 = gr.complex_to_mag_squared()
        movingsum2_taps = [1.0 for i in range(fft_length//2)]
	#movingsum2_taps = [0.5 for i in range(fft_length*4)]		#apurv - implementing Veljo's suggestion, when pause b/w packets

        if 1:
            self.inputmovingsum = gr.fir_filter_fff(1,movingsum2_taps)
        else:
            self.inputmovingsum = gr.fft_filter_fff(1,movingsum2_taps)

        self.square = gr.multiply_ff()
        self.normalize = gr.divide_ff()
     
        # Get magnitude (peaks) and angle (phase/freq error)
        self.c2mag = gr.complex_to_mag_squared()
        self.angle = gr.complex_to_arg()

        self.sample_and_hold = gr.sample_and_hold_ff()

        #ML measurements input to sampler block and detect
        self.sub1 = gr.add_const_ff(-1)
        self.pk_detect = gr.peak_detector_fb(0.20, 0.20, 30, 0.001)	#apurv - implementing Veljo's suggestion, when pause b/w packets

        self.connect(self, self.input)
        
        # Calculate the frequency offset from the correlation of the preamble
        self.connect(self.input, self.delay)
        self.connect(self.input, (self.corr,0))
        self.connect(self.delay, self.conjg)
        self.connect(self.conjg, (self.corr,1))


        self.connect(self.corr, self.moving_sum_filter)
        #self.connect(self.moving_sum_filter, self.c2mag)
        self.connect(self.moving_sum_filter, self.angle)
        self.connect(self.angle, (self.sample_and_hold,0))		# apurv--
	#self.connect(self.angle, gr.delay(gr.sizeof_float, offset), (self.sample_and_hold, 0))	#apurv++

	cross_correlate = 1
	if cross_correlate==1:
	   # cross-correlate with the known symbol
	   kstime = [k.conjugate() for k in kstime]
           kstime.reverse()
           self.crosscorr_filter = gr.fir_filter_ccc(1, kstime)

	   # get the magnitude #
	   self.corrmag = gr.complex_to_mag_squared()

	   self.f2b = gr.float_to_char()
	   self.threshold_factor = threshold #0.0012 #0.012   #0.0015
	   self.slice = gr.threshold_ff(self.threshold_factor, self.threshold_factor, 0, fft_length)

	   self.connect(self.input, self.crosscorr_filter, self.corrmag, self.slice, self.f2b)

	   # some debug dump #
	   #self.connect(self.corrmag, gr.file_sink(gr.sizeof_float, "ofdm_corrmag.dat"))
	   #self.connect(self.f2b, gr.file_sink(gr.sizeof_char, "ofdm_f2b.dat"))
	   

	self.connect(self.f2b, (self.sample_and_hold,1))
	
        # Set output signals
        #    Output 0: fine frequency correction value
        #    Output 1: timing signal
        self.connect(self.sample_and_hold, (self,0))
	#self.connect(self.pk_detect, (self,1))									#removed
	self.connect(self.f2b, (self, 1))

        if logging:
            self.connect(self.matched_filter, gr.file_sink(gr.sizeof_float, "ofdm_sync_pn-mf_f.dat"))
            self.connect(self.normalize, gr.file_sink(gr.sizeof_float, "ofdm_sync_pn-theta_f.dat"))
            self.connect(self.angle, gr.file_sink(gr.sizeof_float, "ofdm_sync_pn-epsilon_f.dat"))
            self.connect(self.pk_detect, gr.file_sink(gr.sizeof_char, "ofdm_sync_pn-peaks_b.dat"))
            self.connect(self.sample_and_hold, gr.file_sink(gr.sizeof_float, "ofdm_sync_pn-sample_and_hold_f.dat"))
            self.connect(self.input, gr.file_sink(gr.sizeof_gr_complex, "ofdm_sync_pn-input_c.dat"))

