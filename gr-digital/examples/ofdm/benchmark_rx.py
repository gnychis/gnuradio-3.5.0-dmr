#!/usr/bin/env python
#
# Copyright 2006,2007,2011 Free Software Foundation, Inc.
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

from gnuradio import gr, blks2
from gnuradio import eng_notation
from gnuradio.eng_option import eng_option
from optparse import OptionParser

from gnuradio import digital

# from current dir
from receive_path import receive_path
from uhd_interface import uhd_receiver

import struct, sys, os
print os.getpid()
#raw_input("Press enter to continue")

class my_top_block(gr.top_block):
    def __init__(self, callback, fwd_callback, options):
        gr.top_block.__init__(self)

        if(options.rx_freq is not None):
            self.source = uhd_receiver(options.args,
                                       options.bandwidth,
                                       options.rx_freq, options.rx_gain,
                                       options.spec, options.antenna,
                                       options.verbose)
        elif(options.from_file is not None):
            self.source = gr.file_source(gr.sizeof_gr_complex, options.from_file)
        else:
            self.source = gr.null_source(gr.sizeof_gr_complex)

        # Set up receive path
        # do this after for any adjustments to the options that may
        # occur in the sinks (specifically the UHD sink)
        self.rxpath = receive_path(callback, fwd_callback, options)

        self.connect(self.source, self.rxpath)
	#self.connect(self.rxpath)
        

# /////////////////////////////////////////////////////////////////////////////
#                                   main
# /////////////////////////////////////////////////////////////////////////////

def main():

    global n_rcvd, n_right, batch_size, n_batch_correct, n_correct, n_total_batches, ack_sock
        
    n_rcvd = 0
    n_right = 0

    if 1:
       # to count the number of batches received correctly #
       batch_size = 1
       n_batch_correct = 0
       n_correct = 0
       n_total_batches = 0
       #ack_sock = socket.socket(socket.PF_INET, socket.SOCK_STREAM)
       #ack_sock.connect(("128.83.141.213", 9000))

    def send_ack(flow, batch):
	tb.rxpath.send_ack(flow, batch)	

    def rx_callback(ok, payload, valid_timestamp, timestamp_sec, timestamp_frac_sec):
        global n_rcvd, n_right, batch_size, n_batch_correct, n_correct, n_total_batches
        n_rcvd += 1
        (pktno,) = struct.unpack('!H', payload[0:2])
        if ok:
            n_right += 1
	    n_correct += 1	

        if 1:
	    # count the correct num of batches (works only for batch_size = 2) #
	    if (pktno + 1) % batch_size == 0:
	      n_total_batches += 1
	      # end of batch #
	      batch_ok = 0
	      if n_correct == batch_size:
	         n_batch_correct += 1
		 batch_ok = 1
		 #send_ack(0, 0)
	      print "batch ok: %r \t pktno: %d \t n_rcvd: %d \t n_right: %d \t correct_batches: %d \t total_batches: %d " % (batch_ok, pktno, n_rcvd, n_right, n_batch_correct, n_total_batches)
	      n_correct = 0
	
	if 0:
	      print "ok: %r \t pktno: %d \t n_rcvd: %d \t n_right: %d " % (ok, pktno, n_rcvd, n_right)
	

    def fwd_callback():
            print "fwd_callback (wrapper) invoked!"

    parser = OptionParser(option_class=eng_option, conflict_handler="resolve")
    expert_grp = parser.add_option_group("Expert")
    parser.add_option("","--discontinuous", action="store_true", default=False,
                      help="enable discontinuous")
    parser.add_option("","--from-file", default=None,
                      help="input file of samples to demod")

    parser.add_option("", "--snr", type="eng_float", default=30, help="set the SNR of the channel in dB [default=%default]")

    receive_path.add_options(parser, expert_grp)
    uhd_receiver.add_options(parser)
    digital.ofdm_demod.add_options(parser, expert_grp)

    (options, args) = parser.parse_args ()

    if options.from_file is None:
        if options.rx_freq is None:
            sys.stderr.write("You must specify -f FREQ or --freq FREQ\n")
            parser.print_help(sys.stderr)
            sys.exit(1)

    # build the graph
    tb = my_top_block(rx_callback, fwd_callback, options)

    r = gr.enable_realtime_scheduling()
    if r != gr.RT_OK:
        print "Warning: failed to enable realtime scheduling"

    tb.start()                      # start flow graph
    tb.wait()                       # wait for it to finish

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
