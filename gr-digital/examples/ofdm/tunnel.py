#!/usr/bin/env python
#
# Copyright 2005,2006,2011 Free Software Foundation, Inc.
# 
# This file is part of GNU Radio
# 
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
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


# /////////////////////////////////////////////////////////////////////////////
#
#    This code sets up up a virtual ethernet interface (typically gr0),
#    and relays packets between the interface and the GNU Radio PHY+MAC
#
#    What this means in plain language, is that if you've got a couple
#    of USRPs on different machines, and if you run this code on those
#    machines, you can talk between them using normal TCP/IP networking.
#
# /////////////////////////////////////////////////////////////////////////////


from gnuradio import gr, digital
from gnuradio import eng_notation
from gnuradio.eng_option import eng_option
from optparse import OptionParser

# from current dir
from receive_path import receive_path
from transmit_path import transmit_path
from uhd_interface import uhd_transmitter
from uhd_interface import uhd_receiver

import os, sys
import random, time, struct

import struct, sys, os
print os.getpid()
#raw_input("Press enter to continue")

# /////////////////////////////////////////////////////////////////////////////
#                             the flow graph
# /////////////////////////////////////////////////////////////////////////////

class my_top_block(gr.top_block):
    def __init__(self, callback, fwd_callback, options):
        gr.top_block.__init__(self)

        self.source = uhd_receiver(options.args,
                                   options.bandwidth,
                                   options.rx_freq, options.rx_gain,
                                   options.spec, options.antenna,
                                   options.verbose)

        self.sink = uhd_transmitter(options.args,
                                    options.bandwidth,
                                    options.tx_freq, options.tx_gain,
                                    options.spec, options.antenna,
                                    options.verbose)

        self.txpath = transmit_path(options)
        self.rxpath = receive_path(callback, fwd_callback, options)

        self.connect(self.txpath, self.sink)
        self.connect(self.source, self.rxpath)


    def get_pkt_to_fwd(self):
        self.rxpath.make_packet()

    def carrier_sensed(self):
        """
        Return True if the receive path thinks there's carrier
        """
        return self.rxpath.carrier_sensed()

    def set_freq(self, target_freq):
        """
        Set the center frequency we're interested in.
        """
        self.u_snk.set_freq(target_freq)
        self.u_src.set_freq(target_freq)
        

# /////////////////////////////////////////////////////////////////////////////
#                           Carrier Sense MAC
# /////////////////////////////////////////////////////////////////////////////

class cs_mac(object):
    """
    Prototype carrier sense MAC

    Reads packets from the TUN/TAP interface, and sends them to the PHY.
    Receives packets from the PHY via phy_rx_callback, and sends them
    into the TUN/TAP interface.

    Of course, we're not restricted to getting packets via TUN/TAP, this
    is just an example.
    """
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.tb = None             # top block (access to PHY)

    def set_flow_graph(self, tb):
        self.tb = tb

    def phy_rx_callback(self, ok, payload, valid_timestamp, timestamp_sec, timestamp_frac_sec):
        """
        Invoked by thread associated with PHY to pass received packet up.

        @param ok: bool indicating whether payload CRC was OK
        @param payload: contents of the packet (string)
        """
        if self.verbose:
            print "Rx: ok = %r  len(payload) = %4d valid_ts: %d \t sec: %d \t frac_sec: %f" % (ok, len(payload), valid_timestamp, timestamp_sec, timestamp_frac_sec)

    def fwd_callback(self, packet):
        """
        Invoked by thread associated with the out queue. The resulting
        packet needs to be sent out using the transmitter flowgraph. 
        The packet could be either a DATA pkt or an ACK pkt. In both cases, 
        only the pkt-hdr needs to be modulated

        @param packet: the pkt to be forwarded through the transmitter chain    
        """
        #print "fwd_callback invoked in tunnel.py"
        if packet.type() == 1:
           print "<tunnel.py> tx DATA!"
           #time.sleep(0.02)                                               #IFS
           #time.sleep(40)
           self.tb.txpath.send_pkt(packet, 1, False)
        elif packet.type() == 2:
           print "<tunnel.py> tx ACK!"
           self.tb.txpath.send_pkt(packet, 1, False)
        else:
           print "<tunnel.py> unknown pkt type:", packet.type()

    def main_loop(self):
        """
        Main loop for MAC.
        Only returns if we get an error reading from TUN.
        """

        min_delay = 0.005               # seconds
        difs_delay = 0.05

	sent = 0
        while 1:

	    sent = 0
            # emulate the CSMA loosely  
            while sent == 0:

                # DIFS
                sensed = 0
                start_difs_time = time.clock()
                #print "start DIFS"

                while (time.clock() - start_difs_time) < difs_delay:
                   if self.tb.carrier_sensed():
                      #print "DIFS broken"
                      sensed = 1
                   if sensed == 1:
                      break

                # DIFS failed, start all over again     
                if sensed == 1:
                   #print "restart DIFS"
                   continue

                # select a random slot
                rand_slot = random.randint(0, 10)

                # backoff
                #print "rand slot : ", rand_slot
                while rand_slot > 0:
                    curr_backoff_time = time.clock()
                    while(time.clock() - curr_backoff_time) < min_delay:
                       if self.tb.carrier_sensed():
                          curr_backoff_time = time.clock()                   #reset current slot                     
                    #print "rand_slot count down"
                    rand_slot = rand_slot - 1


                """
                get the packet to be forwarded (if available)
                will poke the ofdm_frame_sink (ofdm.py), which inserts a packet into the fwd_queue, and 
                triggers the fwd_callback 
                """
                #self.tb.get_pkt_to_fwd()               
                sent = 1

		#self.tb.txpath.send_pkt(struct.pack('!H', 0) + 48 * chr(0 & 0xff), 0, False)
		time.sleep(0.001)

            """transmission happens instead in fwd_callback"""

# /////////////////////////////////////////////////////////////////////////////
#                                   main
# /////////////////////////////////////////////////////////////////////////////

def main():

    parser = OptionParser (option_class=eng_option, conflict_handler="resolve")
    expert_grp = parser.add_option_group("Expert")

    parser.add_option("-m", "--modulation", type="choice", choices=['bpsk', 'qpsk'],
                      default='bpsk',
                      help="Select modulation from: bpsk, qpsk [default=%%default]")
    
    parser.add_option("-v","--verbose", action="store_true", default=False)
    expert_grp.add_option("-c", "--carrier-threshold", type="eng_float", default=30,
                          help="set carrier detect threshold (dB) [default=%default]")
    expert_grp.add_option("", "--snr", type="eng_float", default=30,
                          help="set the SNR of the channel in dB [default=%default]")

    digital.ofdm_mod.add_options(parser, expert_grp)
    digital.ofdm_demod.add_options(parser, expert_grp)
    transmit_path.add_options(parser, expert_grp)
    receive_path.add_options(parser, expert_grp)
    uhd_receiver.add_options(parser)
    uhd_transmitter.add_options(parser)

    (options, args) = parser.parse_args ()
    if len(args) != 0:
        parser.print_help(sys.stderr)
        sys.exit(1)

    if options.rx_freq is None or options.tx_freq is None:
        sys.stderr.write("You must specify -f FREQ or --freq FREQ\n")
        parser.print_help(sys.stderr)
        sys.exit(1)

    # Attempt to enable realtime scheduling
    r = gr.enable_realtime_scheduling()
    if r == gr.RT_OK:
        realtime = True
    else:
        realtime = False
        print "Note: failed to enable realtime scheduling"

    # instantiate the MAC
    mac = cs_mac(verbose=True)


    # build the graph (PHY)
    tb = my_top_block(mac.phy_rx_callback, mac.fwd_callback, options)

    mac.set_flow_graph(tb)    # give the MAC a handle for the PHY

    print "modulation:     %s"   % (options.modulation,)
    print "freq:           %s"      % (eng_notation.num_to_str(options.tx_freq))

    tb.rxpath.set_carrier_threshold(options.carrier_threshold)
    print "Carrier sense threshold:", options.carrier_threshold, "dB"
    
    tb.start()    # Start executing the flow graph (runs in separate threads)

    mac.main_loop()    # don't expect this to return...

    tb.stop()     # but if it does, tell flow graph to stop.
    tb.wait()     # wait for it to finish
                

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
