#!/usr/bin/env python
#
# Copyright 2005,2006,2011 Free Software Foundation, Inc.
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

from gnuradio import gr
from gnuradio import eng_notation
from gnuradio.eng_option import eng_option
from optparse import OptionParser
import time, struct, sys

from gnuradio import digital

# from current dir
from transmit_path import transmit_path
from uhd_interface import uhd_transmitter
from receive_path import receive_path
from uhd_interface import uhd_receiver

import struct, sys, os
print os.getpid()
#raw_input("Press enter to continue")

class my_top_block(gr.top_block):
    def __init__(self, callback, fwd_callback, options):
    #def __init__(self, options):
        gr.top_block.__init__(self)

        if(options.tx_freq is not None):
            self.sink = uhd_transmitter(options.args,
                                        options.bandwidth,
                                        options.tx_freq, options.tx_gain,
                                        options.spec, options.antenna,
                                        options.verbose)

	if(options.rx_freq is not None):
            self.source = uhd_receiver(options.args,
                                       options.bandwidth,
                                       options.rx_freq, options.rx_gain,
                                       options.spec, options.antenna,
                                       options.verbose)

        elif(options.to_file is not None):
            self.sink = gr.file_sink(gr.sizeof_gr_complex, options.to_file)
        else:
            self.sink = gr.null_sink(gr.sizeof_gr_complex)

        # do this after for any adjustments to the options that may
        # occur in the sinks (specifically the UHD sink)
        self.txpath = transmit_path(options)
	self.rxpath = receive_path(callback, fwd_callback, options)

        self.connect(self.txpath, self.sink)
	self.connect(self.source, self.rxpath)
        
# /////////////////////////////////////////////////////////////////////////////
#                                   main
# /////////////////////////////////////////////////////////////////////////////

def main():

    def fwd_callback():
        print "fwd_callback (wrapper) invoked!"

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
              print "batch ok: %r \t pktno: %d \t n_rcvd: %d \t n_right: %d \t correct_batches: %d \t total_batches: %d \t valid_ts: %d \t sec: %d \t frac_sec: %f" % (batch_ok, pktno, n_rcvd, n_right, n_batch_correct, n_total_batches, valid_timestamp, timestamp_sec, timestamp_frac_sec)
              n_correct = 0

    
    def send_pkt(payload='', eof=False):
        return tb.txpath.send_pkt(payload, 0, eof)

    def okToTx():
	return tb.rxpath.okToTx()
   
    parser = OptionParser(option_class=eng_option, conflict_handler="resolve")
    expert_grp = parser.add_option_group("Expert")
    parser.add_option("-s", "--size", type="eng_float", default=400,
                      help="set packet size [default=%default]")
    parser.add_option("-M", "--megabytes", type="eng_float", default=1.0,
                      help="set megabytes to transmit [default=%default]")
    parser.add_option("","--discontinuous", action="store_true", default=False,
                      help="enable discontinuous mode")
    parser.add_option("","--from-file", default=None,
                      help="use intput file for packet contents")
    parser.add_option("","--to-file", default=None,
                      help="Output file for modulated samples")
    digital.ofdm_mod.add_options(parser, expert_grp)
    transmit_path.add_options(parser, expert_grp)
    uhd_transmitter.add_options(parser)
 
    digital.ofdm_demod.add_options(parser, expert_grp)
    receive_path.add_options(parser, expert_grp)
    uhd_receiver.add_options(parser)

    (options, args) = parser.parse_args ()
    print "options: ", options   

    # build the graph
    #tb = my_top_block(options)
    tb = my_top_block(rx_callback, fwd_callback, options)
    
    r = gr.enable_realtime_scheduling()
    if r != gr.RT_OK:
        print "Warning: failed to enable realtime scheduling"

    tb.start()                       # start flow graph
    
    # generate and send packets
    nbytes = int(1e6 * options.megabytes)
    n = 0
    pktno = 0
    pkt_size = int(options.size)

    while(okToTx() == False):
	time.sleep(0.01)

    while n < nbytes:
        if options.from_file is None:
            if(pktno % 2 == 0):
                data = (pkt_size) * chr(3 & 0xff)
            else:
                data = (pkt_size) * chr(4 & 0xff)
            #data = (pkt_size - 2) * chr(pktno & 0xff) 
            #data = (pkt_size - 2) * chr(0x34)
        else:
            data = source_file.read(pkt_size - 2)
            if data == '':
                break;

        #payload = struct.pack('!H', pktno & 0xffff) + data
	payload = data
	
        send_pkt(payload)
        n += len(payload)
        sys.stderr.write('.')
        #if options.discontinuous and pktno % 5 == 4:
        #    time.sleep(1)
        pktno += 1
	time.sleep(0.65)
	#time.sleep(0.1)

    send_pkt(eof=True)
    tb.wait()                       # wait for it to finish

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
