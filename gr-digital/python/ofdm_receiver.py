#!/usr/bin/env python

#
# Copyright 2006, 2007, 2008 Free Software Foundation, Inc.
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

import digital_swig
from ofdm_sync_pn import ofdm_sync_pn
from ofdm_sync_fixed import ofdm_sync_fixed
from ofdm_sync_pnac import ofdm_sync_pnac
from ofdm_sync_ml import ofdm_sync_ml

class ofdm_receiver(gr.hier_block2):
    """
    Performs receiver synchronization on OFDM symbols.

    The receiver performs channel filtering as well as symbol, frequency, and phase synchronization.
    The synchronization routines are available in three flavors: preamble correlator (Schmidl and Cox),
    modifid preamble correlator with autocorrelation (not yet working), and cyclic prefix correlator
    (Van de Beeks).
    """

    def __init__(self, fft_length, cp_length, occupied_tones, snr, ks, threshold, logging=False):
        """
	Hierarchical block for receiving OFDM symbols.

	The input is the complex modulated signal at baseband.
        Synchronized packets are sent back to the demodulator.

        @param fft_length: total number of subcarriers
        @type  fft_length: int
        @param cp_length: length of cyclic prefix as specified in subcarriers (<= fft_length)
        @type  cp_length: int
        @param occupied_tones: number of subcarriers used for data
        @type  occupied_tones: int
        @param snr: estimated signal to noise ratio used to guide cyclic prefix synchronizer
        @type  snr: float
        @param ks: known symbols used as preambles to each packet
        @type  ks: list of lists
        @param logging: turn file logging on or off
        @type  logging: bool
	"""

	gr.hier_block2.__init__(self, "ofdm_receiver",
				gr.io_signature(1, 1, gr.sizeof_gr_complex), # Input signature
                                #gr.io_signature2(2, 2, gr.sizeof_gr_complex*occupied_tones, gr.sizeof_char)) # Output signature apurv--
				#gr.io_signature3(3, 3, gr.sizeof_gr_complex*occupied_tones, gr.sizeof_char, gr.sizeof_gr_complex*occupied_tones))	# apurv++, goes into frame sink for hestimates
        			gr.io_signature4(4, 4, gr.sizeof_gr_complex*occupied_tones, gr.sizeof_char, gr.sizeof_gr_complex*occupied_tones, gr.sizeof_gr_complex*fft_length))     # apurv++, goes into frame sink for hestimates

        bw = (float(occupied_tones) / float(fft_length)) / 2.0
        tb = bw*0.08
        chan_coeffs = gr.firdes.low_pass (1.0,                     # gain
                                          1.0,                     # sampling rate
                                          bw+tb,                   # midpoint of trans. band
                                          tb,                      # width of trans. band
                                          gr.firdes.WIN_HAMMING)   # filter type
        self.chan_filt = gr.fft_filter_ccc(1, chan_coeffs)
        
        win = [1 for i in range(fft_length)]

        zeros_on_left = int(math.ceil((fft_length - occupied_tones)/2.0))
        ks0 = fft_length*[0,]
        ks0[zeros_on_left : zeros_on_left + occupied_tones] = ks[0]
        
	

        ks0 = fft.ifftshift(ks0)
        ks0time = fft.ifft(ks0)
        # ADD SCALING FACTOR
        ks0time = ks0time.tolist()

        SYNC = "pn"
        if SYNC == "ml":
            nco_sensitivity = -1.0/fft_length                             # correct for fine frequency
            self.ofdm_sync = ofdm_sync_ml(fft_length, cp_length, snr, ks0time, logging)
        elif SYNC == "pn":
            nco_sensitivity = -2.0/fft_length                             # correct for fine frequency
            #self.ofdm_sync = ofdm_sync_pn(fft_length, cp_length, logging)
	    self.ofdm_sync = ofdm_sync_pn(fft_length, cp_length, ks0time, threshold, logging)  # apurv++
        elif SYNC == "pnac":
            nco_sensitivity = -2.0/fft_length                             # correct for fine frequency
            self.ofdm_sync = ofdm_sync_pnac(fft_length, cp_length, ks0time, logging)
        elif SYNC == "fixed":                                             # for testing only; do not user over the air
            self.chan_filt = gr.multiply_const_cc(1.0)                    # remove filter and filter delay for this
            nsymbols = 18                                                 # enter the number of symbols per packet
            freq_offset = 0.0                                             # if you use a frequency offset, enter it here
            nco_sensitivity = -2.0/fft_length                             # correct for fine frequency
            self.ofdm_sync = ofdm_sync_fixed(fft_length, cp_length, nsymbols, freq_offset, logging)

        # Set up blocks

        self.nco = gr.frequency_modulator_fc(nco_sensitivity)         # generate a signal proportional to frequency error of sync block
        self.sigmix = gr.multiply_cc()				     
        self.sampler = digital_swig.ofdm_sampler(fft_length, fft_length+cp_length)
        self.fft_demod = gr.fft_vcc(fft_length, True, win, True)
        self.ofdm_frame_acq = digital_swig.ofdm_frame_acquisition(occupied_tones, fft_length,
                                                        cp_length, ks[0])

	# apurv++ modified to allow collected time domain data to artifically pass through the rx chain #
	block_chan_filt = 2 
	if block_chan_filt == 0:
		self.connect(self, self.chan_filt)
		#self.connect(self.chan_filt, gr.file_sink(gr.sizeof_gr_complex, "dump_chan_filt.dat"))
		self.connect(self.chan_filt, gr.null_sink(gr.sizeof_gr_complex))
		#self.connect(gr.file_source(gr.sizeof_gr_complex, "comb_rainier"), self.ofdm_sync)	# apurv++: new source
		#self.connect(gr.file_source(gr.sizeof_gr_complex, "comb_rainier"), gr.delay(gr.sizeof_gr_complex, (fft_length)), (self.sigmix, 0))
		#self.connect((self.ofdm_sync,0), self.nco, (self.sigmix,1))
		#self.connect(self.sigmix, (self.sampler,0)) 
		#self.connect((self.ofdm_sync,1), gr.delay(gr.sizeof_char, fft_length), (self.sampler, 1))		

		#self.connect(gr.file_source(gr.sizeof_gr_complex, "comb_rainier"), (self.sampler,0))
		#self.connect(gr.file_source(gr.sizeof_char, "comb_rainier_timing_sampler"), (self.sampler, 1))

		self.connect(gr.file_source(gr.sizeof_gr_complex*fft_length, "comb_rainier"), self.fft_demod)
                self.connect(gr.file_source(gr.sizeof_char*fft_length, "comb_rainier_timing_sampler"), (self.ofdm_frame_acq,1))

	
	elif block_chan_filt == 1:
		self.connect(self, self.chan_filt)
		self.connect(self.chan_filt, gr.file_sink(gr.sizeof_gr_complex, "dump_chan_filt.dat"))
		self.connect(gr.file_source(gr.sizeof_gr_complex, "comb_rainier"), self.ofdm_sync)
		self.connect((self.ofdm_sync,0), self.nco, (self.sigmix,1))
		self.connect((self.ofdm_sync,1), gr.delay(gr.sizeof_char, fft_length), (self.sampler, 1))
		self.connect(gr.file_source(gr.sizeof_gr_complex, "comb_rainier"), gr.delay(gr.sizeof_gr_complex, (fft_length)), (self.sigmix, 0))
		self.connect(self.sigmix, (self.sampler,0))
		self.connect((self.ofdm_sync,1), gr.delay(gr.sizeof_char, fft_length), gr.file_sink(gr.sizeof_char, "ofdm_sync_pn-peaks_b.dat"))
		self.connect(self.sigmix, gr.file_sink(gr.sizeof_gr_complex, "ofdm_rx_ff_corrected_data_c.dat"))

	        self.connect((self.sampler,0), self.fft_demod)                # send derotated sampled signal to FFT
		self.connect((self.sampler,1), (self.ofdm_frame_acq,1))       # send timing signal to signal frame start

	        self.connect((self.sampler, 0), gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_receiver-sampler_c.dat"))
        	self.connect((self.sampler, 1), gr.file_sink(gr.sizeof_char*fft_length, "ofdm_timing_sampler_c.dat"))
	
	
	elif block_chan_filt == 2:
		
		 # to replay the input manually, use this #
		#self.connect(self, gr.null_sink(gr.sizeof_gr_complex))
		#self.connect(gr.file_source(gr.sizeof_gr_complex, "input.dat"), self.chan_filt)		

		############# input -> chan_filt ##############
		self.connect(self, self.chan_filt)
		
		use_chan_filt = 1
		
		if use_chan_filt == 1:
 		    ##### chan_filt -> SYNC, chan_filt -> SIGMIX ####
		    self.connect(self.chan_filt, self.ofdm_sync)
		    self.connect(self.chan_filt, gr.delay(gr.sizeof_gr_complex, (fft_length)), (self.sigmix, 0))        # apurv++ follow freq offset
		    #self.connect(self.chan_filt, gr.file_sink(gr.sizeof_gr_complex, "ofdm_receiver-chan_filt_c.dat"))
		else: 
		    #### alternatve: chan_filt-> NULL, file_source -> SYNC, file_source -> SIGMIX ####
		    self.connect(self.chan_filt, gr.null_sink(gr.sizeof_gr_complex))
		    self.connect(gr.file_source(gr.sizeof_gr_complex, "chan_filt.dat"), self.ofdm_sync)
		    self.connect(gr.file_source(gr.sizeof_gr_complex, "chan_filt.dat"), gr.delay(gr.sizeof_gr_complex, (fft_length)), (self.sigmix, 0))
		
		method = -1

	        if method == -1:
		    ################## for offline analysis, dump sampler input till the frame_sink, using io_signature4 #################

                    self.connect((self.ofdm_sync,0), self.nco, (self.sigmix,1))   					# freq offset (0'ed :/)
                    self.connect(self.sigmix, (self.sampler,0))                   					# corrected output (0'ed FF)
                    self.connect((self.ofdm_sync,1), gr.delay(gr.sizeof_char, fft_length), (self.sampler, 1))           # timing signal

		    # route received time domain to sink (all-the-way) for offline analysis #
		    self.connect((self.sampler, 0), (self.ofdm_frame_acq, 2))

		    # some logging #
                    #self.connect(self.sigmix, gr.file_sink(gr.sizeof_gr_complex, "ofdm_rx_ff_corrected_data_c.dat"))
                    #self.connect((self.sigmix, 1), gr.file_sink(gr.sizeof_gr_complex, "ofdm_rx_ff_uncorrected_data_c.dat"))
                    #self.connect((self.sampler, 1), gr.file_sink(gr.sizeof_char*fft_length, "ofdm_sampler_timing.dat"))         #timing
		

		if method == 0:
				# NORMAL functioning #

        	    self.connect((self.ofdm_sync,0), self.nco, (self.sigmix,1))   # use sync freq. offset output to derotate input signal
		    self.connect(self.sigmix, (self.sampler,0))                   # sample off timing signal detected in sync alg
		    self.connect((self.ofdm_sync,1), gr.delay(gr.sizeof_char, fft_length), (self.sampler, 1))		# delay?

		    # some logging #
		    #self.connect(self.sigmix, gr.file_sink(gr.sizeof_gr_complex, "ofdm_rx_ff_corrected_data_c.dat"))
		    #self.connect((self.sigmix, 1), gr.file_sink(gr.sizeof_gr_complex, "ofdm_rx_ff_uncorrected_data_c.dat"))
		    #self.connect(self.nco, gr.file_sink(gr.sizeof_gr_complex, "ofdm_receiver-nco_c.dat"))
		    #self.connect((self.sampler, 2), gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_sampler-nco_c.dat"))	

		    #self.connect((self.sampler, 2), (self.ofdm_frame_acq, 2))	

		elif method == 2:
		    self.connect(self.chan_filt, gr.delay(gr.sizeof_gr_complex, (fft_length)), (self.sampler,0))
		    self.connect((self.ofdm_sync,1), (self.sampler, 1))
		    self.connect((self.ofdm_sync, 0), (self.sampler, 2))		# send fine offset (float) to sampler
		    self.connect((self.sampler, 2), (self.ofdm_frame_acq, 2))		# fwd fine offset (float) to frame_acq
		
		    self.connect((self.sampler, 1), gr.file_sink(gr.sizeof_char*fft_length, "ofdm_sampler_timing.dat"))         #timing	
		    self.connect((self.sampler, 2), gr.file_sink(gr.sizeof_float*fft_length, "ofdm_sampler_fine_offset.dat"))	#fine offset

		elif method == 3:
			# bypass nco, and rather use sampler to correct the offset #
		    #self.connect(self.chan_filt, gr.delay(gr.sizeof_gr_complex, (fft_length)), (self.sampler,0))	# symbols
		    self.connect(self.chan_filt, (self.sampler,0))
		    self.connect((self.ofdm_sync, 0), (self.sampler, 2))						# fine offset
		    self.connect((self.ofdm_sync, 1), (self.sampler, 1))						# timing signal
                    #self.connect((self.ofdm_sync, 1), gr.delay(gr.sizeof_char, fft_length), (self.sampler, 1))           # delay?
	            #self.connect((self.sampler, 2), (self.ofdm_frame_acq, 2))		
		
		    self.connect((self.sampler, 2), gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_sampler-nco_c.dat"))		    

                elif method == 4:
                        # let the nco run, bypass the sigmix and rather use sampler to correct the offset #
                    self.connect(self.chan_filt, gr.delay(gr.sizeof_gr_complex, (fft_length)), (self.sampler,0))       # symbols
                    self.connect((self.ofdm_sync, 0), self.nco, (self.sampler, 2))                                      # fine offset (complex multiplier)
                    self.connect((self.ofdm_sync, 1), gr.delay(gr.sizeof_char, fft_length), (self.sampler, 1))           # delay?
                    self.connect((self.sampler, 2), gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_sampler-nco_c.dat"))		
		#######################################################################

		use_default = 1

                if use_default == 0:		#(set method == 0)
			# hack the inputs to fft_demod and ofdm_frame_acq (timing) #
		    self.connect((self.sampler, 0), gr.null_sink(gr.sizeof_gr_complex*fft_length))
		    self.connect((self.sampler, 1), gr.null_sink(gr.sizeof_char*fft_length))
		
                    self.connect(gr.file_source(gr.sizeof_gr_complex*fft_length, "symbols_src.dat"), self.fft_demod)
		    self.connect(self.fft_demod, gr.file_sink(gr.sizeof_gr_complex*fft_length, "dump_fft_out.dat"))
                    self.connect(gr.file_source(gr.sizeof_char*fft_length, "timing_src.dat"), (self.ofdm_frame_acq,1))
		elif use_default == 1:		#(set method == -1)
			# normal functioning! #
                    self.connect((self.sampler,0), self.fft_demod)                # send derotated sampled signal to FFT
                    self.connect((self.sampler,1), (self.ofdm_frame_acq,1))       # send timing signal to signal frame start
		    #self.connect(self.fft_demod, gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_receiver-fft_out_c.dat"))	    
	
		########################### some logging start ##############################
		#self.connect((self.ofdm_sync,1), gr.delay(gr.sizeof_char, fft_length), gr.file_sink(gr.sizeof_char, "ofdm_sync_pn-peaks_b.dat"))
		#self.connect(self.sigmix, gr.file_sink(gr.sizeof_gr_complex, "ofdm_rx_ff_corrected_data_c.dat"))
	        #self.connect((self.sampler, 0), gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_receiver-sampler_c.dat"))
        	#self.connect((self.sampler, 1), gr.file_sink(gr.sizeof_char*fft_length, "ofdm_timing_sampler_c.dat"))
		############################ some logging end ###############################


	# apurv++ modified for manual check #
	block_fft_demod = 2 
	if block_fft_demod == 0:
	    self.connect(self.fft_demod, (self.ofdm_frame_acq,0)) 
            self.connect((self.ofdm_frame_acq, 0), gr.null_sink(gr.sizeof_gr_complex*occupied_tones))
            self.connect((self.ofdm_frame_acq, 1), gr.null_sink(gr.sizeof_char))
	    self.connect(gr.file_source(gr.sizeof_gr_complex*occupied_tones, "tx_data1"), (self,0))
	    self.connect(gr.file_source(gr.sizeof_char, "timing1"), (self,1))

	elif block_fft_demod == 1:
	    #self.connect(self.fft_demod, gr.file_sink(gr.sizeof_gr_complex*fft_length, "dump_fft_out.dat"))
	    self.connect(self.fft_demod, gr.null_sink(gr.sizeof_gr_complex*fft_length))
	    self.connect(gr.file_source(gr.sizeof_gr_complex*fft_length, "tx_data1"), (self.ofdm_frame_acq,0))
	    self.connect(gr.file_source(gr.sizeof_char*fft_length, "timing1"), (self.ofdm_frame_acq,1))
            self.connect((self.ofdm_frame_acq,0), (self,0))               # finished with fine/coarse freq correction,
            self.connect((self.ofdm_frame_acq,1), (self,1))               # frame and symbol timing, and equalization
	    self.connect((self.ofdm_frame_acq,2), (self,2))
	    self.connect((self.ofdm_frame_acq,3), (self,3))               # ref: method=-1

	elif block_fft_demod == 2:
		# for normal functioning! #
	    self.connect(self.fft_demod, (self.ofdm_frame_acq,0)) 		
            self.connect((self.ofdm_frame_acq,0), (self,0))               # finished with fine/coarse freq correction,
	    self.connect((self.ofdm_frame_acq,1), (self,1))               # frame and symbol timing, and equalization
	    self.connect((self.ofdm_frame_acq,2), (self,2))		  # equalizer: hestimates #
  	    self.connect((self.ofdm_frame_acq,3), (self,3))		  # ref: method=-1
	# apurv++ ends #


	# apurv++ log the fine frequency offset corrected symbols #
	#self.connect((self.ofdm_frame_acq, 1), gr.file_sink(gr.sizeof_char, "ofdm_timing_frame_acq_c.dat"))
        #self.connect((self.ofdm_frame_acq, 2), gr.file_sink(gr.sizeof_gr_complex*occupied_tones, "ofdm_hestimates_c.dat"))
	#self.connect(self.fft_demod, gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_receiver-fft_out_c.dat"))
	#self.connect(self.chan_filt, gr.file_sink(gr.sizeof_gr_complex, "ofdm_receiver-chan_filt_c.dat"))
	#self.connect(self, gr.file_sink(gr.sizeof_gr_complex, "ofdm_input_c.dat"))
	# apurv++ end #

        if logging:
            self.connect(self.chan_filt, gr.file_sink(gr.sizeof_gr_complex, "ofdm_receiver-chan_filt_c.dat"))
            self.connect(self.fft_demod, gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_receiver-fft_out_c.dat"))
            self.connect(self.ofdm_frame_acq,
                         gr.file_sink(gr.sizeof_gr_complex*occupied_tones, "ofdm_receiver-frame_acq_c.dat"))
            self.connect((self.ofdm_frame_acq,1), gr.file_sink(1, "ofdm_receiver-found_corr_b.dat"))
            self.connect(self.sampler, gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_receiver-sampler_c.dat"))
            #self.connect(self.sigmix, gr.file_sink(gr.sizeof_gr_complex, "ofdm_receiver-sigmix_c.dat"))
            self.connect(self.nco, gr.file_sink(gr.sizeof_gr_complex, "ofdm_receiver-nco_c.dat"))
