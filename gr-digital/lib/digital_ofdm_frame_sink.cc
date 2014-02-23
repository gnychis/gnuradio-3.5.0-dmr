/* -*- c++ -*- */
/*
 * Copyright 2007,2008,2010 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <digital_ofdm_frame_sink.h>
#include <gr_io_signature.h>
#include <gr_expj.h>
#include <gr_math.h>
#include <math.h>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <string.h>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>

#include <digital_crc32.h>
#include <gr_uhd_usrp_source.h>
#include <gr_uhd_usrp_sink.h>

using namespace arma;
using namespace std;

#define SCALE_FACTOR_PHASE 1e2
#define SCALE_FACTOR_AMP 1e4

#define UNCORRECTED_REPLAY 0
#define CORRECTED_REPLAY 1

//#define SEND_ACK_ETHERNET 1

#define VERBOSE 0
#define LOG_H 1

//#define SCALE 1e3

static const pmt::pmt_t SYNC_TIME = pmt::pmt_string_to_symbol("sync_time");
int last_noutput_items;

//uhd_usrp_source_sptr 
static boost::shared_ptr<uhd_usrp_sink> d_usrp_sink; //uhd_make_usrp_source(arg1, arg2);

inline void
digital_ofdm_frame_sink::enter_search()
{
  if (VERBOSE)
    fprintf(stderr, "@ enter_search\n");

  d_state = STATE_SYNC_SEARCH;
  d_curr_ofdm_symbol_index = 0;
  d_preamble_cnt = 0;
}

/* called everytime a preamble sync is received
   to allow multiple senders: do not reset the phase, slope, etc since they are used during demodulation */
inline void
digital_ofdm_frame_sink::enter_have_sync()
{
  if (VERBOSE)
    fprintf(stderr, "@ enter_have_sync\n");

  d_state = STATE_HAVE_SYNC;

  // clear state of demapper
  d_byte_offset = 0;
  d_partial_byte = 0;

  // apurv++ clear the header details //
  d_hdr_byte_offset = 0;
  memset(d_header_bytes, 0, HEADERBYTELEN);
  memset(&d_header, 0, HEADERBYTELEN);
#ifndef USE_PILOT
  // Resetting PLL (works only for one sender!)
  d_freq[0] = 0.0;
  d_phase[0] = 0.0;
  d_freq1[0] = 0.0;
  d_phase1[0] = 0.0;

  d_slope_phase1 = 0.0; d_slope_freq1 = 0.0; d_slope_angle1 = 0.0;
  d_start_phase1 = 0.0; d_start_freq1 = 0.0; d_start_angle1 = 0.0;
  d_end_phase1 = 0.0; d_end_freq1 = 0.0; d_end_angle1 = 0.0;
#endif

  d_hdr_ofdm_index = 0;
  fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));

}

int
digital_ofdm_frame_sink::okToTx() {
  if(d_ok_to_tx == 1) {
     d_ok_to_tx = 0;
     return 1;
  }
  return 0;
  //return d_ok_to_tx;
}

void
digital_ofdm_frame_sink::disableOkToTx() {
  d_ok_to_tx = 0;
}

inline void
digital_ofdm_frame_sink::reset_demapper() {
  //printf("reset demapper\n"); fflush(stdout);
  // clear state of demapper
  d_byte_offset = 0;
  d_partial_byte = 0;

#ifdef USE_PILOT
  d_pending_senders = 0;
  memset(d_freq, 0, sizeof(float) * MAX_SENDERS);
  memset(d_phase, 0, sizeof(float) * MAX_SENDERS);
  memset(d_slope_angle, 0, sizeof(float) * MAX_SENDERS);
  memset(d_start_angle, 0, sizeof(float) * MAX_SENDERS);
  memset(d_end_angle, 0, sizeof(float) * MAX_SENDERS);
#else
  // Resetting PLL
  for(int i = 0; i < d_batch_size; i++) {
     d_freq[i] = 0.0;
     d_phase[i] = 0.0;
     d_freq1[i] = 0.0;
     d_phase1[i] = 0.0;
  }
#endif

  fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));
  //d_save_flag = false;
}

/* extract the header info
   the header for now includes: 
   - pkt-type           (1 bit)
   - pktlen             (12 bits)
   - n_senders          (3 bits)
   - batch-number       (8 bits)
   - rx-id              (4 bits)
   - src-id             (4 bits)
   - coeffs             (4*batchsize)
   P.S: CRC has already been extracted by now and verified!

 --- 1 ---  ---------- 12 ---- ----- 3 ----- ------- 8 ------------- 4 ------   ------- 4 -------
|| pkt_type ||     pkt_len    || n_senders ||   batch_number ||    rx-id      ||     src-id     ||
---------------------------------------------------------------
*/
inline void
digital_ofdm_frame_sink::extract_header(gr_complex h)
{
  d_packet_whitener_offset = 0;                 // apurv++: hardcoded!

  d_pkt_type = d_header.pkt_type;
  d_flow = d_header.flow_id;

  d_packetlen = d_header.packetlen;
  d_batch_number = d_header.batch_number;

  d_nsenders = d_header.nsenders;	
  d_lead_sender = d_header.lead_sender;	

  d_pkt_num = d_header.pkt_num;
  d_prevLinkId = d_header.link_id;
  d_timing_offset = d_header.timing_offset;
  printf("\tpkt_num: %d \t\t\t\t batch_num: %d \t\t\t\t len: %d src: %d prev-link: %d timing_off: %f\n", d_header.pkt_num, d_header.batch_number, d_packetlen, d_header.src_id, d_prevLinkId, d_timing_offset); fflush(stdout);

  if (VERBOSE || 1)
    fprintf(stderr, " hdr details: (src: %d), (rx: %d), (batch_num: %d), (d_nsenders: %d), (d_packetlen: %d), (d_pkt_type: %d), (prev_hop: %d)\n",
                    d_header.src_id, d_header.dst_id, d_batch_number, d_nsenders, d_packetlen, d_header.pkt_type, d_header.prev_hop_id);

  // do appropriate conversions
  d_dst_id = ((int) d_header.dst_id) + '0';
  d_src_id = ((int) d_header.src_id) + '0';
  d_prev_hop_id = ((int) d_header.prev_hop_id) + '0';

  //printf("prev_hop: %c, src: %c, dst: %c\n", d_prev_hop_id, d_src_id, d_dst_id); fflush(stdout);

  if(d_h_coding) {
    HInfo hInfo;
    hInfo.pkt_num = d_pkt_num; //d_batch_number;
    hInfo.h_value = gr_complex(1.0, 0.0)/h;			//h: equalizer, channel=1/eq
    hInfo.timing_slope = d_timing_offset + get_eq_slope();

    HKey hkey(d_prev_hop_id, d_id);
    updateHInfo(hkey, hInfo, false);

    // only transmit after all the synch transmitters headers have been extracted //
    if(d_nsenders == 1 || (d_nsenders > 1 && d_lead_sender == 0))
       txHInfo();
  } // d_h_coding
}

unsigned char digital_ofdm_frame_sink::slicer_hdr(const gr_complex x)
{
  unsigned int table_size = d_hdr_sym_value_out.size();
  unsigned int min_index = 0;
  float min_euclid_dist = norm(x - d_hdr_sym_position[0]);
  float euclid_dist = 0;

  for (unsigned int j = 1; j < table_size; j++){
    euclid_dist = norm(x - d_hdr_sym_position[j]);
    if (euclid_dist < min_euclid_dist){
      min_euclid_dist = euclid_dist;
      min_index = j;
    }
  }
#if 0
  gr_complex closest = d_sym_position[min_index];
  printf("x: (%f, %f), closest: (%f, %f), evm: (%f, %f)\n", x.real(), x.imag(), closest.real(), closest.imag(), min_euclid_dist); fflush(stdout);
#endif
  return d_hdr_sym_value_out[min_index];
}

/* uses pilots - from rawofdm */
void digital_ofdm_frame_sink::equalize_interpolate_dfe(const gr_complex *in, gr_complex *out) 
{
  gr_complex phase_error = 0.0;

  int sender_index = 0;
  if(d_pending_senders > 0) {
    // implies this is not the first header in this transmission //
    sender_index = d_pktInfo->n_senders - d_pending_senders;
  }

  float cur_pilot = 1.0;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); i++) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    phase_error += (in[di] * conj(pilot_sym));
  } 

  if(d_hdr_ofdm_index == 0) {
	printf("sender index: %d, d_freq: %f, d_phase: %f\n", sender_index, d_freq[sender_index], d_phase[sender_index]); fflush(stdout);
  }

  // update phase equalizer
  float angle = arg(phase_error);
  d_freq[sender_index] = d_freq[sender_index] - d_freq_gain*angle;
  d_phase[sender_index] = d_phase[sender_index] + d_freq[sender_index] - d_phase_gain*angle;
  if (d_phase[sender_index] >= 2*M_PI) d_phase[sender_index] -= 2*M_PI;
  else if (d_phase[sender_index] <0) d_phase[sender_index] += 2*M_PI;

  gr_complex carrier = gr_expj(-angle);

  // update DFE based on pilots
  cur_pilot = 1.0;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); i++) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    gr_complex sigeq = in[di] * carrier * d_dfe[i];
    // FIX THE FOLLOWING STATEMENT
    if (norm(sigeq)> 0.001)
      d_dfe[i] += d_eq_gain * (pilot_sym/sigeq - d_dfe[i]);
  }

  // equalize all data using interpolated dfe and demap into bytes
  unsigned int pilot_index = 0;
  int pilot_carrier_lo = 0;
  int pilot_carrier_hi = d_pilot_carriers[0];
  gr_complex pilot_dfe_lo = d_dfe[0];
  gr_complex pilot_dfe_hi = d_dfe[0];

  /* alternate implementation of lerp */
  float denom = 1.0/(pilot_carrier_hi - pilot_carrier_lo);					// 1.0/(x_b - x_a)

  for (unsigned int i = 0; i < d_data_carriers.size(); i++) {
      int di = d_data_carriers[i];
      if(di > pilot_carrier_hi) {					// 'di' is beyond the current pilot_hi
	  pilot_index++;						// move to the next pilot

	  pilot_carrier_lo = pilot_carrier_hi;				// the new pilot_lo
	  pilot_dfe_lo = pilot_dfe_hi;					// the new pilot_dfe_lo

	 if (pilot_index < d_pilot_carriers.size()) {
	     pilot_carrier_hi = d_pilot_carriers[pilot_index];
	     pilot_dfe_hi = d_dfe[pilot_index];
	 }
	 else {
	     pilot_carrier_hi = d_occupied_carriers;			// cater to the di's which are beyond the last pilot_carrier
	 }
	 denom = 1.0/(pilot_carrier_hi - pilot_carrier_lo);
      }

      float alpha = float(di - pilot_carrier_lo) * denom;		// (x - x_a)/(x_b - x_a)
      gr_complex dfe = pilot_dfe_lo + alpha * (pilot_dfe_hi - pilot_dfe_lo);	// y = y_a + (y_b - y_a) * (x - x_a)/(x_b - x_a) 
      gr_complex sigeq = in[di] * carrier * dfe;
      out[i] = sigeq;
  }

  /* for the slope of the angle, to be used if a gap exists between the header and the data for this sender */
  if(d_hdr_ofdm_index == 0) {
      d_start_angle[sender_index] = angle;
  }
  else if(d_hdr_ofdm_index == d_num_hdr_ofdm_symbols-1) {
      d_end_angle[sender_index] = angle;
      d_slope_angle[sender_index] = (d_end_angle[sender_index] - d_start_angle[sender_index])/((float) d_hdr_ofdm_index);	
      //printf("(after header) sender index: %d, d_phase: %f, d_freq: %f\n", sender_index, d_phase[sender_index], d_freq[sender_index]); fflush(stdout);
  }
}

// apurv++ comment: demaps an entire OFDM symbol in one go (with pilots) //
unsigned int digital_ofdm_frame_sink::demapper_pilot(const gr_complex *in,
                                               unsigned char *out)
{

  gr_complex *rot_out = (gr_complex*) malloc(sizeof(gr_complex) * d_data_carriers.size());		// only the data carriers //
  memset(rot_out, 0, sizeof(gr_complex) * d_data_carriers.size());
  equalize_interpolate_dfe(in, rot_out);


  unsigned int i=0, bytes_produced=0;

  while(i < d_data_carriers.size()) {
    if(d_nresid > 0) {
      d_partial_byte |= d_resid;
      d_byte_offset += d_nresid;
      d_nresid = 0;
      d_resid = 0;
    }

    while((d_byte_offset < 8) && (i < d_data_carriers.size())) {

      unsigned char bits = slicer_hdr(rot_out[i]);

      i++;
      if((8 - d_byte_offset) >= d_hdr_nbits) {
        d_partial_byte |= bits << (d_byte_offset);
        d_byte_offset += d_hdr_nbits;
      }
      else {
        d_nresid = d_hdr_nbits-(8-d_byte_offset);
        int mask = ((1<<(8-d_byte_offset))-1);
        d_partial_byte |= (bits & mask) << d_byte_offset;
        d_resid = bits >> (8-d_byte_offset);
        d_byte_offset += (d_hdr_nbits - d_nresid);
      }
      //printf("demod symbol: %.4f + j%.4f   bits: %x   partial_byte: %x   byte_offset: %d   resid: %x   nresid: %d\n", 
      //     in[i-1].real(), in[i-1].imag(), bits, d_partial_byte, d_byte_offset, d_resid, d_nresid);
    }

    if(d_byte_offset == 8) {
      //printf("demod byte: %x \n\n", d_partial_byte);
      out[bytes_produced++] = d_partial_byte;
      d_byte_offset = 0;
      d_partial_byte = 0;
    }
  }

  free(rot_out);
  return bytes_produced;
}

digital_ofdm_frame_sink_sptr
digital_make_ofdm_frame_sink(const std::vector<gr_complex> &hdr_sym_position,
                        const std::vector<unsigned char> &hdr_sym_value_out,
			const std::vector<gr_complex> &data_sym_position,
                        const std::vector<unsigned char> &data_sym_value_out,
			const std::vector<std::vector<gr_complex> > &preamble,
                        gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
			unsigned int occupied_carriers, unsigned int fft_length,
                        float phase_gain, float freq_gain, unsigned int id, 
			unsigned int batch_size, unsigned int decode_flag, 
			int fwd_index, int replay_flag,
			int exp_size, int fec_n, int fec_k, int degree, int h_coding)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_frame_sink(hdr_sym_position, hdr_sym_value_out,
							data_sym_position, data_sym_value_out,
							preamble,
                                                        target_queue, fwd_queue,
							occupied_carriers, fft_length,
                                                        phase_gain, freq_gain, id,
							batch_size, decode_flag, fwd_index, replay_flag,
							exp_size, fec_n, fec_k, degree, h_coding));
}


digital_ofdm_frame_sink::digital_ofdm_frame_sink(const std::vector<gr_complex> &hdr_sym_position,
                                       const std::vector<unsigned char> &hdr_sym_value_out,
		                       const std::vector<gr_complex> &data_sym_position,
                		       const std::vector<unsigned char> &data_sym_value_out,
				       const std::vector<std::vector<gr_complex> > &preamble,
                                       gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
				       unsigned int occupied_carriers, unsigned int fft_length,
                                       float phase_gain, float freq_gain, unsigned int id,
				       unsigned int batch_size, unsigned int decode_flag, 
				       int fwd_index, int replay_flag,
				       int exp_size, int fec_n, int fec_k, int degree, int h_coding)
  : gr_sync_block ("ofdm_frame_sink",
                   //gr_make_io_signature2 (2, 2, sizeof(gr_complex)*occupied_carriers, sizeof(char)),  // apurv--
                   gr_make_io_signature4 (2, 4, sizeof(gr_complex)*occupied_carriers, sizeof(char), sizeof(gr_complex)*occupied_carriers, sizeof(gr_complex)*fft_length), //apurv++
                   gr_make_io_signature (0, 1, sizeof(gr_complex)*occupied_carriers)),
    d_target_queue(target_queue), d_occupied_carriers(occupied_carriers),
    d_byte_offset(0), d_partial_byte(0),
    d_resid(0), d_nresid(0),d_phase_gain(phase_gain),d_freq_gain(freq_gain),
    d_eq_gain(0.05), 
    d_batch_size(batch_size),
    d_decode_flag(decode_flag),
    d_replay_flag(replay_flag),
    d_active_batch(-1),
    d_last_batch_acked(-1),
    d_pkt_num(0),
    d_id(id+'0'),
    d_fft_length(fft_length),
    d_out_queue(fwd_queue),
    d_fwd_index(fwd_index),
    d_expected_size(exp_size),
    d_fec_n(fec_n),
    d_fec_k(fec_k),
    d_degree(degree),
    d_h_coding(h_coding),
    d_preamble(preamble),
    d_preamble_cnt(0),
    d_ok_to_tx(0)
{
   assign_subcarriers();
   d_dfe.resize(d_pilot_carriers.size());
   fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));

  // make sure we stay in the limit currently imposed by the occupied_carriers
  if(d_data_carriers.size() > d_occupied_carriers) {
    throw std::invalid_argument("digital_ofdm_mapper_bcv: subcarriers allocated exceeds size of occupied carriers");
  }

  d_bytes_out = new unsigned char[d_occupied_carriers];
  set_hdr_sym_value_out(hdr_sym_position, hdr_sym_value_out);
  set_data_sym_value_out(data_sym_position, data_sym_value_out);

  enter_search();

  //apurv check //
  d_num_hdr_ofdm_symbols = (HEADERBYTELEN * 8) / d_data_carriers.size();
  if((HEADERBYTELEN * 8) % d_data_carriers.size() != 0) {	       // assuming bpsk
     printf("d_data_carriers: %d, HEADERBYTELEN: %d\n", d_data_carriers.size(), HEADERBYTELEN); fflush(stdout);
     assert(false);
  }


  printf("HEADERBYTELEN: %d hdr_ofdm_symbols: %d, d_data_carriers.size(): %d\n", HEADERBYTELEN, d_num_hdr_ofdm_symbols, d_data_carriers.size());
  fflush(stdout);

  // apurv- for offline analysis/logging //
  d_file_opened = false;
  d_fp_coeff = NULL;
  d_fp_solved = NULL;
  d_fp_rx_symbols = NULL; 
  d_fp_dfe_symbols = NULL;
  d_fp_hestimates = NULL;

  d_in_estimates = (gr_complex*) malloc(sizeof(gr_complex) * occupied_carriers);
  memset(d_in_estimates, 0, sizeof(gr_complex) * occupied_carriers);  

#ifdef LOG_H
  assert(open_hestimates_log());
#endif

  int len = 400;
  d_save_flag = false;

  populateCompositeLinkInfo();
  populateCreditInfo();
  num_acks_sent = 0;
  num_data_fwd = 0;

  assert(openLogTxFwdSymbols());

  d_demod_log_file = false;
  d_fp_demod = NULL;

  memset(d_phase, 0, sizeof(float) * MAX_BATCH_SIZE);
  memset(d_freq, 0, sizeof(float) * MAX_BATCH_SIZE);

  memset(d_phase1, 0, sizeof(float) * MAX_BATCH_SIZE); 
  memset(d_freq1, 0, sizeof(float) * MAX_BATCH_SIZE); 

#ifdef SEND_ACK_ETHERNET
  d_ack_sock = open_client_sock(9000, "128.83.141.213", false);
  if(d_ack_sock != -1) {
     printf("@ backend ethernet connected for ACK!\n"); fflush(stdout);
  }
#endif

  
  /* initialize the d_euclid_dist for a batch 
  int n_entries = pow(2.0, double(d_batch_size));
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
      for(unsigned int j = 0; j < n_entries; j++) {
          d_euclid_dist[i][j] = 0.0;
      }
  } */
  int n_entries = pow(double (d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));
  memset(d_euclid_dist, 0, sizeof(float) * d_data_carriers.size() * n_entries * MAX_OFDM_SYMBOLS);

  memset(d_batch_euclid_dist, 0, sizeof(float) * d_batch_size * d_data_carriers.size() * n_entries * MAX_OFDM_SYMBOLS);
  memset(d_flag_euclid_dist, 0, sizeof(bool) * d_data_carriers.size() * MAX_OFDM_SYMBOLS);

  d_fp_sync_symbols = NULL;
  d_sync_file_opened = false;

  std::string arg("");
  d_usrp = uhd::usrp::multi_usrp::make(arg);
  d_flow = -1;

  //d_usrp_sink = get_usrp_sink_instance();
  //assert(d_usrp_sink);
  //printf("sample rate: %f\n", d_usrp_sink->get_samp_rate()); 

  reset_demapper();

  fill_all_carriers_map();   

  d_coeff_unpacked_open = false;
  d_fp_coeff_unpacked = NULL;
  d_coeff_open = false;
  d_fp_coeff_y = NULL;
  d_fp_coeff_y1 = NULL;

  d_fp_training = NULL;
  d_training_log_open = false;

  d_num_pkts_correct = 0;
  d_total_pkts_received = 0;

#ifdef MIMO_RECEIVER
  open_mimo_sock();
#endif

  openCorrectSymbolLog();
  openRxSymbolLog();
  openDFESymbolLog();

  if(NUM_TRAINING_SYMBOLS > 0) {
     memset(d_training_symbols, 0, NUM_TRAINING_SYMBOLS * MAX_OCCUPIED_CARRIERS * sizeof(gr_complex));
     generateKnownSymbols();
     d_parse_pkt = false;
  }
 

  memset(d_true_symbols, 0, sizeof(gr_complex) * d_data_carriers.size() * MAX_OFDM_SYMBOLS);
  d_log_SER_symbols_open = false;

  d_agg_total_symbols = 0; d_agg_correct_symbols = 0;

  if(d_h_coding)
     prepare_H_coding();

  d_out_pkt_time = uhd::time_spec_t(0.0);
  d_last_pkt_time = uhd::time_spec_t(0.0);

  d_fp_comb_log = NULL;
}

void
digital_ofdm_frame_sink::assign_subcarriers() {
  int dc_tones = 8;
  int num_pilots = 8;
  int pilot_gap = 11;

  int half1_end = (d_occupied_carriers-dc_tones)/2;     //40
  int half2_start = half1_end+dc_tones;                 //48

  // first half
  for(int i = 0; i < half1_end; i++) {
     if(i%pilot_gap == 0)
        d_pilot_carriers.push_back(i);
     else
        d_data_carriers.push_back(i);
  }

  // second half
  for(int i = half2_start, j = 0; i < d_occupied_carriers; i++, j++) {
     if(j%pilot_gap == 0)
        d_pilot_carriers.push_back(i);
     else
        d_data_carriers.push_back(i);
  }

  /* debug carriers */
  printf("pilot carriers: \n");
  for(int i = 0; i < d_pilot_carriers.size(); i++) {
     printf("%d ", d_pilot_carriers[i]); fflush(stdout);
  }
  printf("\n");

  printf("data carriers: \n");
  for(int i = 0; i < d_data_carriers.size(); i++) {
     printf("%d ", d_data_carriers[i]); fflush(stdout);
  }
  printf("\n");
}

void
digital_ofdm_frame_sink::fill_all_carriers_map() {
  d_all_carriers.resize(d_occupied_carriers);

  unsigned int p = 0, d = 0, dc = 0;

  for(unsigned int i = 0; i < d_occupied_carriers; i++) {
      int carrier_index = i;

      if(d_data_carriers[d] == carrier_index) {
	 d_all_carriers[i] = 0;
	 d++;
      } 
      else if(d_pilot_carriers[p] == carrier_index) {
	 d_all_carriers[i] = 1;
	 p++;
      }
      else {
	 d_all_carriers[i] = 2;
	 dc++;
      }      
  }

  printf("d: %d, p: %d, dc: %d\n", d, p, dc); fflush(stdout);

  assert(d == d_data_carriers.size());
  assert(p == d_pilot_carriers.size());
  assert(dc == d_occupied_carriers - d_data_carriers.size() - d_pilot_carriers.size());
}

digital_ofdm_frame_sink::~digital_ofdm_frame_sink ()
{
  delete [] d_bytes_out;
}

bool
digital_ofdm_frame_sink::set_hdr_sym_value_out(const std::vector<gr_complex> &sym_position,
                                      const std::vector<unsigned char> &sym_value_out)
{
  if (sym_position.size() != sym_value_out.size())
    return false;

  if (sym_position.size()<1)
    return false;

  d_hdr_sym_position  = sym_position;
  d_hdr_sym_value_out = sym_value_out;
  d_hdr_nbits = (unsigned int)ceil(log10(float(d_hdr_sym_value_out.size())) / log10((float)2.0));		// I've got no idea!!!!!
  printf("size: %d, d_hdr_nbits: %d\n", d_hdr_sym_value_out.size(), d_hdr_nbits); fflush(stdout);

  return true;
}

bool
digital_ofdm_frame_sink::set_data_sym_value_out(const std::vector<gr_complex> &sym_position,
                                      const std::vector<unsigned char> &sym_value_out)
{
  if (sym_position.size() != sym_value_out.size())
    return false;

  if (sym_position.size()<1)
    return false;

  d_data_sym_position  = sym_position;
  d_data_sym_value_out = sym_value_out;
  d_data_nbits = (unsigned int)ceil(log10(float(d_data_sym_value_out.size())) / log10((float)2.0));               // I've got no idea!!!!!
  //printf("size: %d, d_data_nbits: %d\n", d_data_sym_value_out.size(), d_data_nbits); fflush(stdout);

  return true;
}

int
digital_ofdm_frame_sink::work (int noutput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items)
{
  gr_complex *in = (gr_complex *) input_items[0];
  const char *sig = (const char *) input_items[1];

  last_noutput_items=noutput_items;

  /* channel estimates if applicable */
  gr_complex *in_estimates = NULL;
  bool use_estimates = false;
  if(input_items.size() >= 3) {
       in_estimates = (gr_complex *) input_items[2];
       use_estimates = true;
  }

  /* apurv++: extra input from sampler */
  gr_complex *in_sampler = NULL;
  if(input_items.size() >= 4) {
       in_sampler = (gr_complex *) input_items[3];
  }

  if (VERBOSE)
    fprintf(stderr,">>> Entering state machine, d_state: %d\n", d_state);

  unsigned ii = 0;

  unsigned int bytes=0;
  unsigned int n_data_carriers = d_data_carriers.size();
  switch(d_state) {

  case STATE_SYNC_SEARCH:    // Look for flag indicating beginning of pkt
    if (VERBOSE)
      fprintf(stderr,"SYNC Search, noutput=%d\n", noutput_items);

    if (sig[0]) {  // Found it, set up for header decode
      enter_have_sync();
      test_timestamp(1);
      memcpy(d_in_estimates, in_estimates, sizeof(gr_complex) * d_occupied_carriers);  // use in HAVE_HEADER state //
      d_preamble_cnt++;
#if LOG_H
      if(use_estimates && 0) {
          int count = ftell(d_fp_hestimates);
          count = fwrite_unlocked(in_estimates, sizeof(gr_complex), d_occupied_carriers, d_fp_hestimates);
      }
#endif
    }
    break;      // don't demodulate the preamble, so break!

  case STATE_HAVE_SYNC:
    if(d_preamble_cnt < d_preamble.size()) {
	if(d_preamble_cnt == d_preamble.size()-1) {
	   /* last preamble - calculate SNR */
	   equalizeSymbols(&in[0], d_in_estimates);
	   calculate_snr(in);
	}
	else {
	   /* log hestimates (**disable if calculate_equalizer is not being called for this preamble**) */
#if LOG_H
           if(use_estimates) {
	      memcpy(d_in_estimates, in_estimates, sizeof(gr_complex) * d_occupied_carriers);
              //int count = ftell(d_fp_hestimates);
              //count = fwrite_unlocked(in_estimates, sizeof(gr_complex), d_occupied_carriers, d_fp_hestimates);
           }
#endif
	}
	d_preamble_cnt++;
	break;
    }
    //test_timestamp(1);

    /* apurv++: equalize hdr - and use the estimates only if NOT *found sync in HAVE_SYNC* */
    if(use_estimates && !sig[0]) {
       equalizeSymbols(&in[0], d_in_estimates);
    }

    // only demod after getting the preamble signal; otherwise, the 
    // equalizer taps will screw with the PLL performance
    bytes = demapper_pilot(&in[0], d_bytes_out);
    d_hdr_ofdm_index++;

    if (sig[0]) {
       printf("ERROR -- Found SYNC in HAVE_SYNC\n");
       reset_demapper();
       enter_search();
       break;
    }

    /* add the demodulated bytes to header_bytes */
    memcpy(d_header_bytes+d_hdr_byte_offset, d_bytes_out, bytes);
    d_hdr_byte_offset += bytes;

    //printf("hdrbytelen: %d, hdrdatalen: %d, d_hdr_byte_offset: %d, bytes: %d\n", HEADERBYTELEN, HEADERDATALEN, d_hdr_byte_offset, bytes); fflush(stdout);

    /* header processing */

    if (d_hdr_byte_offset == HEADERBYTELEN) {
        if (header_ok()) {						 // full header received
          printf("HEADER OK prev_hop: %d pkt_num: %d batch_num: %d dst_id: %d norm_factor: %f\n", d_header.prev_hop_id, d_header.pkt_num, d_header.batch_number, d_header.dst_id, d_header.factor); 
          fflush(stdout);
          extract_header(d_in_estimates[0]);						// fills local fields with header info

          assert(d_nsenders > 0);

	  /* if the batch is already full rank, no need to process the packet any further */
	  FlowInfo *flowInfo = getFlowInfo(false, d_flow);

	  /* ensure if I'm in the current session, then the pkt num matches the d_save_pkt_num */
	  //printf("d_save_flag: %d, d_pkt_num: %d, d_save_pkt_num: %d\n", d_save_flag, d_pkt_num, d_save_pkt_num); fflush(stdout);
	  if(d_save_flag) {
 	      if(d_pkt_num != d_save_pkt_num) {
		 reset_demapper();
		 resetPktInfo(d_pktInfo);
		 flowInfo->innovative_pkts.pop_back();
 	      }
	  }

	  /* first pkt in session should always be from the lead sender */
	  if(d_lead_sender == 1 && d_pkt_type == DATA_TYPE) 
	  {
		printf("lead sender: %d, senders: %d!\n", d_header.src_id, d_nsenders); fflush(stdout);
		if(!shouldProcess()) {
		   enter_search();
		   break;
		}

	        assert(d_fwd || d_dst);

	        /* stale batch ? */
        	if(d_header.batch_number < d_active_batch) {
                    printf("STALE BATCH %d --\n", d_header.batch_number); fflush(stdout);
		    enter_search();
		    reset_demapper();
		    d_ok_to_tx = 1;
		    break;
	        } 
	  	else if(d_header.batch_number <= d_last_batch_acked)
	  	{
		    printf("BATCH %d already ACKed --\n", d_header.batch_number); fflush(stdout);
		    enter_search();
		    reset_demapper();
		    d_ok_to_tx = 1;
		    break;
		}
		else if(d_header.batch_number > d_active_batch)
  	  	    prepareForNewBatch();				// ensure the FlowInfo is in place

		/* interested in pkt! create a new PktInfo for the flow */
		d_pktInfo = createPktInfo();
		d_pending_senders = d_nsenders;
		d_save_flag = true;         				// indicates that a session is on, e'one will be saved (haha)
		d_save_pkt_num = d_pkt_num;
	  }
	  else if(d_pkt_type == DATA_TYPE){
		printf("following sender: %d, pending_senders: %d, nsenders: %d!\n", d_header.src_id, d_pending_senders, d_nsenders); fflush(stdout);
	  }
 	  else if(d_pkt_type == ACK_TYPE) {
		printf("received ACK from %d, batch: %d\n", d_header.src_id, d_batch_number); fflush(stdout);
	  }

	  if(d_save_flag) {
	     save_coefficients();					// saves in d_pktInfo

	     if(d_pending_senders == 1) {
		d_pending_senders = 0;
		d_save_flag = false;

                d_state = STATE_HAVE_HEADER;
		d_parse_pkt = true;
		//d_state = STATE_HAVE_NULL; d_null_symbols = 0;
		d_curr_ofdm_symbol_index = 0;
		d_num_ofdm_symbols = ceil(((float) (d_packetlen * 8))/(d_data_carriers.size() * d_data_nbits));
		assert(d_num_ofdm_symbols < MAX_OFDM_SYMBOLS); 				// FIXME - arbitrary '70'.. 
		printf("d_num_ofdm_symbols: %d, actual sc size: %d, d_data_nbits: %d\n", d_num_ofdm_symbols, d_data_carriers.size(), d_data_nbits);
		fflush(stdout);

		/* rx symbols */
		d_pktInfo->symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_num_ofdm_symbols * d_occupied_carriers);
		memset(d_pktInfo->symbols, 0, sizeof(gr_complex) * d_num_ofdm_symbols * d_occupied_carriers);
		//printf("allocated pktInfo rxSymbols\n"); fflush(stdout);
	     } else {
		assert(d_pending_senders > 1);				// more senders remain - switch to preamble look-up
		d_pending_senders--;
		enter_search();
	     }

             if(NUM_TRAINING_SYMBOLS > 0) {
                d_state = STATE_HAVE_TRAINING;
                d_null_symbols = 0;
             }

	  } else {
	     enter_search(); 						// don't care
	     reset_demapper();
	     break;
	  } 
        }
        else {
          printf("HEADER NOT OK --\n"); fflush(stdout);
          enter_search();           					// bad header
	  reset_demapper();
        }
    }
    break;

#if 0
  case STATE_HAVE_TRAINING:
    memcpy(&d_training_symbols[d_null_symbols*MAX_OCCUPIED_CARRIERS], &in[0], sizeof(gr_complex) * d_occupied_carriers);
    d_null_symbols++;
    if(d_null_symbols == NUM_TRAINING_SYMBOLS) {
        //calculate_fine_offset();
        calculateSNR();
	if(d_parse_pkt) {
            d_state = STATE_HAVE_HEADER;
	    d_parse_pkt = false;
	} else {
	    enter_search();
	}
    }
    break;

  case STATE_HAVE_NULL:
    d_null_symbols++;
    if(d_null_symbols == 1)
	d_state = STATE_HAVE_HEADER;
    break;
#endif

  case STATE_HAVE_HEADER:						// process the frame after header
    if (sig[0]) 
        printf("ERROR -- Found SYNC in HAVE_HEADER, length of %d\n", d_packetlen);

    assert(d_curr_ofdm_symbol_index < d_num_ofdm_symbols);
    assert(d_pending_senders == 0);

    /* de-normalize the signal */
    //denormalizeSignal(in);

    storePayload(&in[0], in_sampler);
    d_curr_ofdm_symbol_index++;

    if(VERBOSE) {
       printf("d_curr_ofdm_symbol_index: %d, d_num_ofdm_symbols: %d\n", d_curr_ofdm_symbol_index, d_num_ofdm_symbols);
       fflush(stdout);
    }
	 
    if(d_curr_ofdm_symbol_index == d_num_ofdm_symbols) {		// last ofdm symbol in pkt
 	FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  	assert(flowInfo != NULL);

        // only logs the symbols //
        logFrequencyDomainRxSymbols(false);

	if(d_dst) {
	   d_ok_to_tx = 1;
	   demodulate_ILP_2(flowInfo); 
	}
	else if(d_fwd) 
	{
	   bool isCarrierCorrected = false;
	   isCarrierCorrected = do_carrier_correction();
	   //debug_carrier_correction();
	   if(isCarrierCorrected) {
	      printf("Carrier correction done on the **current** packet\n"); fflush(stdout);
	   } else {
	      printf("Carrier correction failed since there are mutiple senders involved!\n"); fflush(stdout);
	   }

	    //correctSignal(flowInfo);

	    printf("check credit: fwd\n"); fflush(stdout);
	    // increment credit counter for the incoming pkt (flow based) //
	    updateCredit();
	    makePacket(true); 
	}

	// offline analysis  - time domain logging //
	enter_search();     						// current pkt done, next packet
	reset_demapper();
	d_save_flag = false;
        break;
    }

    break;
 
  default:
    assert(0);

  } // switch

  return 1;
}

/* equalization that was supposed to be done by the acquisition block! */
inline void
digital_ofdm_frame_sink::equalizeSymbols(gr_complex *in, gr_complex *in_estimates)
{
   for(unsigned int i = 0; i < d_occupied_carriers; i++) 
        in[i] = in[i] * in_estimates[i];
}

/*
   - extract header crc
   - calculate header crc
   - verify if both are equal
*/
bool
digital_ofdm_frame_sink::header_ok()
{
  //printf("header_ok start\n"); fflush(stdout);
  dewhiten(d_header_bytes, HEADERBYTELEN);
  memcpy(&d_header, d_header_bytes, d_hdr_byte_offset);
  unsigned int rx_crc = d_header.hdr_crc;
 
  unsigned char *header_data_bytes = (unsigned char*) malloc(HEADERDATALEN);
  memcpy(header_data_bytes, d_header_bytes, HEADERDATALEN);
  unsigned int calc_crc = digital_crc32(header_data_bytes, HEADERDATALEN);
  free(header_data_bytes);
 
  if(rx_crc != calc_crc) {
	printf("\nrx_crc: %u, calc_crc: %u, \t\t\t\t\tpkt_num: %d \tbatch_num: %d\n", rx_crc, calc_crc, d_header.pkt_num, d_header.batch_number); fflush(stdout);
  }

  /* DEBUG
  for(unsigned int k = 0; k < d_batch_size; k++)
	printf("[%u %u] ", d_header.coeffs[k].real(), d_header.coeffs[k].imag());
  printf("\n"); fflush(stdout);
  printf("\nrx_crc: %u, calc_crc: %u, \t\t\t\t\tpkt_num: %d \tbatch_num: %d\n", rx_crc, calc_crc, d_header.pkt_num, d_header.batch_number); fflush(stdout);
  printf("hdr details: (src: %d), (rx: %d), (batch_num: %d), (d_nsenders: %d), (d_packetlen: %d), (d_pkt_type: %d)\n", d_header.src_id, d_header.dst_id, d_header.batch_number, d_header.nsenders, d_header.packetlen, d_header.pkt_type);

  printf("header_ok end\n"); fflush(stdout);
  */

  return (rx_crc == calc_crc);
}

void
digital_ofdm_frame_sink::dewhiten(unsigned char *bytes, const int len)
{
  for(int i = 0; i < len; i++)
      bytes[i] = bytes[i] ^ random_mask_tuple[i];
}


/* creates/assigns a new FlowInfo entry (if needed). 
   clears coeff matrix and innovativePktVector */
void
digital_ofdm_frame_sink::prepareForNewBatch()
{
  //printf("prepareForNewBatch, batch: %d\n", d_header.batch_number); fflush(stdout);
  d_active_batch = d_header.batch_number;

  FlowInfo *flow_info = getFlowInfo(true, d_flow);
  /* reset the credit for the old batch */
  if(d_fwd) {
      CreditInfo *cInfo = findCreditInfo(flow_info->flowId);
      assert(cInfo);
      cInfo->credit = 0.0;
  }

  /* ensure the flowInfo is updated/created */  
  flow_info->flowId = d_header.flow_id;
  flow_info->src = d_src_id; //d_header.src_id;
  flow_info->dst = d_dst_id; //d_header.dst_id;
  flow_info->active_batch = d_header.batch_number;
  flow_info->last_batch_acked = flow_info->active_batch - 1;

  /* clean innovative pktInfo */
  InnovativePktInfoVector rx_vec = flow_info->innovative_pkts;
  int size = rx_vec.size();
  for(int i = 0; i < size; i++) {
	//printf("i: %d size: %d\n", i, size); fflush(stdout);
      PktInfo *pInfo = rx_vec[i];
      resetPktInfo(pInfo);		// clears the symbols as well
      free(pInfo);
  }

  flow_info->innovative_pkts.clear();

  /* clean reduced_info */
  vector<gr_complex*> red_coeffs = flow_info->reduced_coeffs;
  for(unsigned int i = 0; i < red_coeffs.size(); i++) {
      gr_complex* coeff = red_coeffs.at(i);
      free(coeff);	
  }
  flow_info->reduced_coeffs.clear();

  int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));
  memset(d_euclid_dist, 0, sizeof(float) * d_data_carriers.size() * n_entries * MAX_OFDM_SYMBOLS);

  memset(d_batch_euclid_dist, 0, sizeof(float) * d_batch_size * d_data_carriers.size() * n_entries * MAX_OFDM_SYMBOLS);
  memset(d_flag_euclid_dist, 0, sizeof(bool) * d_data_carriers.size() * MAX_OFDM_SYMBOLS);

  //printf("prepareForNewBatch ends, size: %d, %d\n", flow_info->innovative_pkts.size(), d_flowInfoVector.size()); fflush(stdout);
  d_avg_evm_error = 0.0;
  d_total_batches_received++;
  memset(d_crc, 0, sizeof(int) * MAX_BATCH_SIZE);
}


/* stores 1 OFDM symbol */
void
digital_ofdm_frame_sink::storePayload(gr_complex *in, gr_complex *in_sampler)
{
  unsigned int offset = d_curr_ofdm_symbol_index * d_occupied_carriers;
  memcpy(d_pktInfo->symbols + offset, in, sizeof(gr_complex) * d_occupied_carriers);
}

inline void
digital_ofdm_frame_sink::loadRxMatrix(cx_mat &RX, unsigned int ofdm_symbol_index, unsigned int subcarrier_index, FlowInfo *flowInfo)
{
  long index = ofdm_symbol_index * d_occupied_carriers + subcarrier_index;

  InnovativePktInfoVector innovativePkts = flowInfo->innovative_pkts;
  assert(innovativePkts.size() == d_batch_size); 

  for(unsigned int i = 0; i < d_batch_size; i++)
  {
      PktInfo *pInfo = innovativePkts[i];
      gr_complex *rx_symbols_this_batch = pInfo->symbols;
      RX(i, 0) = rx_symbols_this_batch[index];
  }
}

inline void
digital_ofdm_frame_sink::saveTxMatrix(cx_mat TX, unsigned int ofdm_symbol_index, unsigned int subcarrier_index, vector<gr_complex*> out_sym_vec)
{
  unsigned long offset = ofdm_symbol_index * d_occupied_carriers + subcarrier_index;
  for(unsigned int k = 0; k < d_batch_size; k++)
  {
      gr_complex *symbols = out_sym_vec[k];
      symbols[offset] = TX(k, 0);
  }
}

/*
// TEST VERSION!!!! //
void
digital_ofdm_frame_sink::decodePayload()
{
  // initialize the vector that holds the decoded symbols //
  assert(d_batch_size == 1);
  for(unsigned int k = 0; k < d_batch_size; k++)
  {
        gr_complex *symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers * d_num_ofdm_symbols);
        memset(symbols, 0, sizeof(gr_complex) * d_occupied_carriers * d_num_ofdm_symbols);
        out_rx_sym_vec.push_back(symbols);
  }

  gr_complex *rx_symbols_this_batch = in_rx_sym_vec[0];           // get the symbols for batch
  gr_complex *tx_symbols_this_batch = out_rx_sym_vec[0];

  float angle_rad = d_header.coeffs[0] * (M_PI/180);
  gr_complex t = gr_expj(angle_rad);

  //printf("decodePayload (%f, %f)\n", t.real(), t.imag()); fflush(stdout);

  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
     for(unsigned int i = 0; i < d_occupied_carriers; i++) {
	long index = o * d_occupied_carriers + i;
	tx_symbols_this_batch[index] = rx_symbols_this_batch[index]/t;	
     }
  }

  unsigned char *bytes_out = (unsigned char*) malloc(sizeof(unsigned char) * d_occupied_carriers);
  for(unsigned int k = 0; k < d_batch_size; k++) {
     gr_complex *out_symbols = out_rx_sym_vec[k];
     int packetlen_cnt = 0;
     unsigned char packet[MAX_PKT_LEN];
	
     for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
         unsigned int offset = o * d_occupied_carriers;

         memset(bytes_out, 0, sizeof(unsigned char) * d_occupied_carriers);
         unsigned int bytes_decoded = demapper(&out_symbols[offset], bytes_out);

	 

         unsigned int jj = 0;
         while(jj < bytes_decoded) {
              packet[packetlen_cnt++] = bytes_out[jj++];

              if (packetlen_cnt == d_packetlen){              // packet is filled
                  gr_message_sptr msg = gr_make_message(0, d_packet_whitener_offset, 0, packetlen_cnt);
                  memcpy(msg->msg(), packet, packetlen_cnt);
                  d_target_queue->insert_tail(msg);               // send it
                  msg.reset();                            // free it up
              }
        }
     }
  }
  free(bytes_out);
  d_last_batch_acked = d_active_batch; 
}
*/

/*
void
digital_ofdm_frame_sink::decodePayload_multiple(FlowInfo *flowInfo)
{
  //printf("decodePayload\n"); fflush(stdout);

  // initialize the vector that holds the decoded symbols //
  vector<gr_complex*> out_sym_vec;
  for(unsigned int k = 0; k < d_batch_size; k++)
  {
        gr_complex *symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers * d_num_ofdm_symbols);
        memset(symbols, 0, sizeof(gr_complex) * d_occupied_carriers * d_num_ofdm_symbols);
        out_sym_vec.push_back(symbols);
  }

  // the coeff_mat needs to be updated to bring hestimates in as well. Ensure the reduced_coeffs vector size is d_batch_size //
  vector<gr_complex*> reduced_coeffs = flowInfo->reduced_coeffs;
  if(reduced_coeffs.size() != d_batch_size) {
	printf("reduced_coeffs.size: %d, d_batch_size: %d\n", reduced_coeffs.size(), d_batch_size); fflush(stdout);
  }
  assert(reduced_coeffs.size() == d_batch_size);

  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++)
  {
      for(unsigned int i = 0; i < d_occupied_carriers; i++)
      {
          // initialize //
          cx_mat RX = randu<cx_mat>(d_batch_size, 1);
          cx_mat TX = randu<cx_mat>(d_batch_size, 1);

          loadRxMatrix(RX, o, i, flowInfo);
	  cx_mat coeff_mat = flowInfo->coeff_mat;
	  updateCoeffMatrix(flowInfo, i, coeff_mat);

          // solve now //
          TX = solve(coeff_mat, RX);

          // verify //
          //verifySolution(TX, RX);

          saveTxMatrix(TX, o, i, out_sym_vec);
      }
  }

  // for offline analysis - log e'thing: 1) coefficients, 2) solved symbols 3) rx symbols //
  //log(flowInfo, out_sym_vec);

  demodulate(out_sym_vec);

  // TODO: need to do a crc check for the final decoded pkt 
  d_last_batch_acked = d_active_batch;
}

void
digital_ofdm_frame_sink::decodePayload_single(FlowInfo *flowInfo)
{
  printf("decodePayload_single, flow: %d\n", flowInfo->flowId); fflush(stdout);

  // initialize the vector that holds the decoded symbols //
  vector<gr_complex*> out_sym_vec;
  for(unsigned int k = 0; k < d_batch_size; k++)
  {
        gr_complex *symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers * d_num_ofdm_symbols);
        memset(symbols, 0, sizeof(gr_complex) * d_occupied_carriers * d_num_ofdm_symbols);
        out_sym_vec.push_back(symbols);
  }


  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++)
  {
      for(unsigned int i = 0; i < d_occupied_carriers; i++)
      {
          // initialize //
          cx_mat RX = randu<cx_mat>(d_batch_size, 1);
          cx_mat TX = randu<cx_mat>(d_batch_size, 1);

          loadRxMatrix(RX, o, i, flowInfo);

          // solve now //
          TX = solve(flowInfo->coeff_mat, RX);

	  
	  if(i == 0 && o == 0) {
	     for(int k = 0; k < d_batch_size; k++) {
		 complex<double> t_coeff = flowInfo->coeff_mat(k, 0);
	         //printf(" [%d] (%lf, %lf) * (%lf, %lf) = (%lf, %lf) \n", k, t_coeff.real(), t_coeff.imag(), TX(k, 0).real(), TX(k, 0).imag(), RX(k, 0).real(), RX(k, 0).imag()); 
		 //fflush(stdout); 
	     }
	  }
	

	  // verify //
 	  //verifySolution(TX, RX, flowInfo->coeff_mat);

          saveTxMatrix(TX, o, i, out_sym_vec);
      }
  }

  // for offline analysis - log e'thing: 1) coefficients, 2) solved symbols 3) rx symbols //
  //log(flowInfo, out_sym_vec);

  demodulate(out_sym_vec);

  // TODO: need to do a crc check for the final decoded pkt 
  d_last_batch_acked = d_active_batch;
}

void
digital_ofdm_frame_sink::demodulate(vector<gr_complex*> out_sym_vec)
{
  printf("demodulate\n"); fflush(stdout);

  // run the demapper now: demap each packet within the batch //
  unsigned char *bytes_out = (unsigned char*) malloc(sizeof(unsigned char) * d_occupied_carriers);
  for(unsigned int k = 0; k < d_batch_size; k++) {
     gr_complex *out_symbols = out_sym_vec[k];
     int packetlen_cnt = 0;
     unsigned char packet[MAX_PKT_LEN];
     //printf("d_partial_byte: %d, d_byte_offset: %d\n", d_partial_byte, d_byte_offset); 
     //printf("----------------- pkt# %d\n ----------------------- \n", k); fflush(stdout);
     for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
         unsigned int offset = o * d_occupied_carriers;

         memset(bytes_out, 0, sizeof(unsigned char) * d_occupied_carriers);
         unsigned int bytes_decoded = demapper(&out_symbols[offset], bytes_out);

	 //log_demod(bytes_out, bytes_decoded, &out_symbols[offset], d_occupied_carriers);
                  
         unsigned int jj = 0;
         while(jj < bytes_decoded) {
              packet[packetlen_cnt++] = bytes_out[jj++];

              if (packetlen_cnt == d_packetlen){              // packet is filled
		
                  gr_message_sptr msg = gr_make_message(0, d_packet_whitener_offset, 0, packetlen_cnt);
                  memcpy(msg->msg(), packet, packetlen_cnt);

                  // With a good header, let's now check for the preamble sync timestamp
                  std::vector<gr_tag_t> rx_sync_tags;
                  const uint64_t nread = this->nitems_read(0);
                  this->get_tags_in_range(rx_sync_tags, 0, nread, nread+last_noutput_items, SYNC_TIME);
                  if(rx_sync_tags.size()>0) {
                    size_t t = rx_sync_tags.size()-1;
                    const pmt::pmt_t &value = rx_sync_tags[t].value;
                    uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
                    double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));
                    msg->set_timestamp(sync_secs, sync_frac_of_secs);
                  } else {
                    //std::cerr << "---- Header received, with no sync timestamp?\n";
                  }

                  d_target_queue->insert_tail(msg);               // send it
                  msg.reset();                            // free it up
              }
        }
     }
     //printf(" ----------------------------------------------------------\n"); fflush(stdout);
  }
  free(bytes_out);

  //sendACK(d_flow, d_batch_number);

  // clear out_sym_vec //
  for(unsigned int i = 0; i < d_batch_size; i++) 
     free(out_sym_vec[i]);
  out_sym_vec.clear();
}


void digital_ofdm_frame_sink::log_demod(unsigned char *bytes, int num_bytes, gr_complex *symbols, int num_symbols) {
  if(!d_demod_log_file) {
      const char *filename1 = "demod_bytes.dat";
      int fd;
      if ((fd = open (filename1, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename1);
	 assert(false);
      }
      else {
         if((d_fp_demod = fdopen (fd, true ? "wb" : "w")) == NULL) {
             fprintf(stderr, "demod file cannot be opened\n");
             close(fd);
	     assert(false);
         }
      }

      const char *filename2 = "demod_symbols.dat";
      if ((fd = open (filename2, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename2);
         assert(false);
      }
      else {
         if((d_fp_demod_symbols = fdopen (fd, true ? "wb" : "w")) == NULL) {
             fprintf(stderr, "demod file cannot be opened\n");
             close(fd);
             assert(false);
         }
      }

      d_demod_log_file = true;
  }

  int count = fwrite_unlocked(bytes, sizeof(unsigned char), num_bytes, d_fp_demod);
  assert(count == num_bytes);

  count = fwrite_unlocked(symbols, sizeof(gr_complex), num_symbols, d_fp_demod_symbols);
  assert(count == num_symbols);
}

void
digital_ofdm_frame_sink::log(FlowInfo *flowInfo, vector<gr_complex*> out_sym_vec)
{
  if(!d_file_opened)
  {
      d_file_opened = open_log();
      assert(d_file_opened);
      fprintf(stderr, "log fileS opened!\n");
  }
  assert(d_fp_coeff != NULL);
  assert(d_fp_solved != NULL);
  assert(d_fp_rx_symbols != NULL);


  // log coefficients -- the coefficients are stored in coeff_mat, where
  // rows: contain cofficients for one packet
  // cols: packets considered for the batch 
  // Need to store the coefficients such that, coeffs(packet 1) ... coeffs (packet2) .. so on
  //
  // Steps: Find transpose of the matrix coeff_mat
  //        Get the memory pointer for the transposed matrix
  //	    Log it	

  cx_mat coeff_mat = flowInfo->coeff_mat;
  assert(flowInfo);
  int count = ftell(d_fp_coeff);
  cx_mat coeff_mat_t = coeff_mat.t();
  int items = coeff_mat_t.n_rows * coeff_mat_t.n_cols;


  gr_complex *log_arr = (gr_complex*) malloc(sizeof(gr_complex) * items);
  for(int i = 0; i < items; i++)
      log_arr[i] = coeff_mat_t.memptr()[i];
 
  count = fwrite_unlocked(log_arr, sizeof(gr_complex), items, d_fp_coeff);
  free(log_arr);
    
  // log solved tx symbols //
  count = ftell(d_fp_solved);
  for(unsigned k = 0; k < d_batch_size; k++) {
     gr_complex *tx_symbols = out_sym_vec[k];
     count = fwrite_unlocked(tx_symbols, sizeof(gr_complex), d_occupied_carriers * d_num_ofdm_symbols, d_fp_solved);
  }

  // log rx input symbols //
  count = ftell(d_fp_rx_symbols);
  InnovativePktInfoVector pInfoVector = flowInfo->innovative_pkts;
  for(unsigned k = 0; k < d_batch_size; k++) {
     gr_complex *rx_symbols = pInfoVector[k]->symbols;
     count = fwrite_unlocked(rx_symbols, sizeof(gr_complex), d_occupied_carriers * d_num_ofdm_symbols, d_fp_rx_symbols);
  }  
}

// ensures that all the log files (for offline analysis) have been opened, 
//   P.S: If one is open, all are open :) 
bool
digital_ofdm_frame_sink::open_log()
{
  printf("open_log called1\n"); fflush(stdout);

  // rx coefficients //
  const char *filename1 = "rx_coeff.dat";
  int fd;
  if ((fd = open (filename1, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename1);
     return false;
  }
  else {
      if((d_fp_coeff = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "coeff file cannot be opened\n");
            close(fd);
            return false;
      }
  }

  // solved tx symbols //
  const char *filename2 = "rx_solved.dat";
  if ((fd = open (filename2, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename2);
     return false;
  }
  else {
      if((d_fp_solved = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "rx solved file cannot be opened\n");
            close(fd);
            return false;
      }
  }

  // unsolved rx symbols files //
  const char *filename3 = "rx_symbols.dat";
  if ((fd = open (filename3, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename3);
     return false;
  }
  else {
      if((d_fp_rx_symbols = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "rx symbols file cannot be opened\n");
            close(fd);
            return false;
      }
  }

  return true;
}
*/

bool
digital_ofdm_frame_sink::open_hestimates_log()
{
  // hestimates - only for those for which the header is OK //
  const char *filename = "ofdm_hestimates.dat";
  int fd;
  if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     return false;
  }
  else {
      if((d_fp_hestimates = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "hestimates file cannot be opened\n");
            close(fd);
            return false;
      }
  }
  return true;
}

/* node can be either of 3: 
   - final destination of the flow
   - a forwarder
   - a triggered forwarder
*/
bool
digital_ofdm_frame_sink::shouldProcess() {
   //TODO: code up!
  /*
  if(d_id == d_header.dst_id) {
     printf("Destination! d_id: %d, d_header.dst_id: %d\n", d_id, d_header.dst_id); fflush(stdout);
     d_dst = true;
     d_fwd = false;
     return true;
  }
  else {
      d_dst = false;
      // designated forwarder, only if it is one of the destinations of the dst_link (composite) in the header //
      int num_links = d_compositeLinkVector.size();
      assert(num_links > 0);
      CompositeLink *cLink = getCompositeLink(d_prevLinkId);
      assert(cLink);
      vector<unsigned int> ids = cLink->dstIds;
      for(int i = 0; i < ids.size(); i++) {
	 if(ids.at(i) == d_id) {
	     printf("shouldProcess as a neighbor!\n"); fflush(stdout);
	     d_neighbor = true;
	     return true;
	 } 
      }
      printf("shouldProcess returns false!\n"); fflush(stdout);
      return false;
  }*/

  d_dst = false; d_fwd = false;
  int num_links = d_compositeLinkVector[d_flow].size();
  assert(num_links > 0);
  CompositeLink *cLink = getCompositeLink(d_prevLinkId);
  assert(cLink);
  //vector<unsigned int> ids = cLink->dstIds;
  NodeIds ids = cLink->dstIds;
  for(int i = 0; i < ids.size(); i++) {
     //printf("curr: %c, d_id: %c, dst_id: %c\n", ids.at(i), d_id, d_dst_id); fflush(stdout);
     if(ids.at(i) == d_id) {
	//if(d_id == d_header.dst_id)   {
	if(d_id == d_dst_id) {
	     printf("destination!!\n"); fflush(stdout);
	     d_dst = true;
	}
	else {
	     printf("forwarder!!\n"); fflush(stdout);
	     d_fwd = true;
	}
	return true;
     }
  }

  return false;
}

inline PktInfo*
digital_ofdm_frame_sink::createPktInfo() {
  //printf("createPktInfo start\n"); fflush(stdout);
  FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  assert(flowInfo);

  PktInfo *pktInfo = (PktInfo*) malloc(sizeof(PktInfo));
  memset(pktInfo, 0, sizeof(PktInfo));

  pktInfo->n_senders = d_nsenders;
  pktInfo->prevLinkId = d_prevLinkId;

  /*
  unsigned int n_senders = pktInfo->n_senders;
  for(unsigned int i = 0; i < n_senders; i++)
  {
      // coeffs 
      gr_complex *coeffs = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size);
      pktInfo->coeffs.push_back(coeffs);

      // hestimates 
      gr_complex *hestimates = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
      pktInfo->hestimates.push_back(hestimates);
  }
  */

  /* add to flowInfo */
  flowInfo->innovative_pkts.push_back(pktInfo);
  
  pktInfo->symbols = NULL;
  //printf("createPktInfo end\n"); fflush(stdout);
  return pktInfo;
}

inline void
digital_ofdm_frame_sink::resetPktInfo(PktInfo* pktInfo) {
  //printf("resetPktInfo\n"); fflush(stdout);
  int num_senders = pktInfo->n_senders;

  /* clear all */ 
  pktInfo->senders.clear();
  pktInfo->norm_factor.clear();
  int size = pktInfo->coeffs.size();
  for(int i = 0; i < size; i++) {
     free(pktInfo->coeffs[i]);
     free(pktInfo->hestimates[i]);
  }
  
  pktInfo->coeffs.clear();
  pktInfo->hestimates.clear(); 

  if(num_senders > 0 && pktInfo->symbols != NULL)
     free(pktInfo->symbols);
  
  /* reinit */
  //pktInfo->n_senders = d_nsenders;
  //printf("resetPktInfo done\n"); fflush(stdout);
}

// for the sender, save the hestimate and the coefficients it included in the hdr //
void
digital_ofdm_frame_sink::save_coefficients()
{
  NodeId sender_id = d_prev_hop_id;
  assert(sender_id >= 0);

  gr_complex *hestimates = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
#if 1
  memcpy(hestimates, d_in_estimates, sizeof(gr_complex) * d_occupied_carriers);
#else 
  for(int i = 0; i < d_occupied_carriers; i++) {
     hestimates[i] = gr_complex(1.0, 0.0);
  }
#endif


#ifdef DEBUG
  float atten = 0.0;
  for(int i = 0; i < d_occupied_carriers; i++) {
     atten += abs(hestimates[i]);
     //printf("(atten: %f, phase: %f) <--> complex(%f, %f) \n", abs(hestimates[i]), arg(hestimates[i]), hestimates[i].real(), hestimates[i].imag()); fflush(stdout);
     if(i == 0)
	printf("[0] (%.2f, %.2f) , atten: %f, phase: %f\n", hestimates[i].real(), hestimates[i].imag(), abs(hestimates[i]), arg(hestimates[i]));
  }
  atten /= ((float) d_occupied_carriers);
  printf("sender: %c, avg_atten: %.3f\n", sender_id, atten); fflush(stdout);
#endif

  int num_carriers = d_data_carriers.size();                 // each header will have lsq_coeffs of batch1, batch2 and so on..
  gr_complex *coeffs = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * num_carriers);

  if(sender_id == d_src_id) {
     // if packet is received directly from the source, then no need to do any LSQ //
     for(unsigned int k = 0; k < d_batch_size; k++) {
	for(unsigned int s = 0; s < num_carriers; s++) {
	    int index = k*num_carriers + s;
	    COEFF hdr_coeff = d_header.coeffs[k];
	    coeffs[index] = ToPhase_c(hdr_coeff);
	}
     }
     //print_coeffs(coeffs);
  }
  else {
     gr_complex lsq_coeffs[MAX_DEGREE * MAX_BATCH_SIZE];
     assert(d_degree+1 <= MAX_DEGREE);

     // convert the COEFF into gr_complex //
     printf("LSQ coefficients: \n"); fflush(stdout);
     for(unsigned int k = 0; k < d_batch_size; k++) {
         for(unsigned int i = 0; i < d_degree+1; i++) {
	     int index = k*(d_degree+1) + i;
	     //lsq_coeffs[index] = ToPhase_c(d_header.coeffs[index]);

	     // differential de-scaling: phase 1. Phase 2 is done in unpackCoefficients //
	     float amp = (float) (d_header.coeffs[index].amplitude)/pow(10.0, i+1);
	     float ph = (float) (d_header.coeffs[index].phase)/SCALE_FACTOR_PHASE;
	     lsq_coeffs[index] = amp * gr_expj(ph * M_PI/180); 
	     //printf("batch: %d, i: %d -- [A: %d, P: %d] ---- (%f, %f) \n", k, i, d_header.coeffs[index].amplitude, d_header.coeffs[index].phase, lsq_coeffs[index].real(), lsq_coeffs[index].imag());
	 }
	 unpackCoefficients_LSQ(&lsq_coeffs[k*(d_degree+1)], &coeffs[k*num_carriers], num_carriers, d_degree);
     }
  }

  //print_coeffs(coeffs);

  d_pktInfo->senders.push_back(sender_id);
  d_pktInfo->norm_factor.push_back(d_header.factor);
  d_pktInfo->coeffs.push_back(coeffs);
  d_pktInfo->hestimates.push_back(hestimates);					// push all the estimates //

  //printf("save_coeffs end\n"); fflush(stdout);
  //debugPktInfo(d_pktInfo, sender_id);
}

void
digital_ofdm_frame_sink::print_coeffs(gr_complex *coeffs) {
  int num_carriers = d_data_carriers.size(); 
  printf("degree: %d\n", d_degree); fflush(stdout);
  for(unsigned i = 0; i < num_carriers; i++) {
     for(unsigned int k = 0; k < d_batch_size; k++) {
        int index = i + k*num_carriers;
        float amp = abs(coeffs[index]);
        float phase = ToPhase_f(coeffs[index]);
        printf("(A: %f, P: %f) <-> (%f, %f)\n", amp, phase, coeffs[index].real(), coeffs[index].imag());
     }
  }
}

/* simple linear interpolation 
   <in_coeffs>: contain <data_carriers/COMPRESSION_FACTOR> entries for each batch
   <out_coeffs>: <d_occupied_carriers> entries for each batch
*/
void
digital_ofdm_frame_sink::interpolate_coeffs_lerp(gr_complex* in_coeffs, gr_complex *out_coeffs)
{
   printf("interpolate_coeffs_lerp\n"); fflush(stdout);

   int carrier_entries = d_data_carriers.size()/COMPRESSION_FACTOR;

   /*
   int start = 0;
   int x_a = d_data_carriers[start];
   int x_b = d_data_carriers[start];

   gr_complex y_a = in_coeffs[0];
   gr_complex y_b = in_coeffs[0]; 
   float denom = 1.0/(x_b - x_a);
   unsigned int index = 1;
   */

   for(unsigned int k = 0; k < d_batch_size; k++) {
      int start = 0;
      int x_a = 0;
      int x_b = 0;
      float denom = 1.0/(x_b - x_a);

      unsigned int index = 1;

      gr_complex y_a = in_coeffs[k];
      gr_complex y_b = in_coeffs[k];

      for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
          int di = d_data_carriers[i];
          printf("di: %d, x_a: %d, x_b: %d\n", di, x_a, x_b); fflush(stdout);

          if(i == x_a || i == x_b) {
             out_coeffs[di*d_batch_size+k] = in_coeffs[(i/COMPRESSION_FACTOR) * d_batch_size + k];
             //printf("\t\tout_coeffs: (%f, %f)\n", out_coeffs[di].real(), out_coeffs[di].imag()); fflush(stdout);
          }
          else {
             if(i > x_b) {
                x_a = x_b; y_a = y_b;
                if(index < carrier_entries) {
                     x_b = index * COMPRESSION_FACTOR;
                     y_b = in_coeffs[index*d_batch_size+k];
                     index++;
                 }
                 else {
                     x_b = d_data_carriers.size() - 1;
                 }
             }
             denom = 1.0/(x_b - x_a);
             float alpha = float(i - x_a) * denom;               // (x - x_a)/(x_b - x_a)
             out_coeffs[di*d_batch_size+k] = y_a + alpha * (y_b - y_a);
             //printf("\t\tout_coeffs: (%f, %f)\n", out_coeffs[di].real(), out_coeffs[di].imag()); fflush(stdout);
          }
      }
      assert(index == carrier_entries);
   }

#if 0
   // debug interpolated coeffs //
   printf("before interpolation\n"); fflush(stdout);
   for(int i = 0; i < carrier_entries; i++) {
      for(unsigned int k = 0; k < d_batch_size; k++) {
         printf("coeff[%d]: (%f, %f)\t", k, in_coeffs[i*d_batch_size+k].real(), in_coeffs[i*d_batch_size+k].imag()); fflush(stdout);
      }
      printf("\n");
   }
   printf("------------------- \n"); fflush(stdout);
   printf("after interpolation\n"); fflush(stdout);
   for(int i = 0; i < d_occupied_carriers; i++) {
      printf("subcarrier: %d ", i); fflush(stdout);
      for(int k = 0; k < d_batch_size; k++) {
          printf("index:%d, coeff[%d]: (%f, %f)\t", i*d_batch_size+k, k, out_coeffs[i*d_batch_size+k].real(), out_coeffs[i*d_batch_size+k].imag()); fflush(stdout);
      }
      printf("\n");
   } 
#endif
}

/* interpolate <in_coeffs> to get <out_coeffs>. 
   <in_coeffs>: contain <data_carriers/2> entries for each batch
   <out_coeffs>: <d_occupied_carriers> entries for each batch 
*/
void
digital_ofdm_frame_sink::interpolate_coeffs(gr_complex* in_coeffs, gr_complex *out_coeffs)
{
   printf("interpolate_coeffs\n"); fflush(stdout);

   /* deduce the coefficient of each data_subcarrier. Ignore the pilot subcarriers; keep filling up the coefficients
   of remaining subcarriers (i.e data) in each of the half_occupied_tones. 
   Assumptions:
   - 1st half of the header_coeffs belong to the corresponding half_occupied_tones */
   unsigned int half_coeffs = d_data_carriers.size()/4;	

   int start, end, sc_index;
   for(unsigned int k = 0; k < d_batch_size; k++) {  
      sc_index = 0, start = 0, end = half_coeffs;

      while(1) {
	  for(unsigned int i = start; i < end; i++) {
	     int curr_coeff_index = i*d_batch_size+k;
	     int next_coeff_index = (i+1)*d_batch_size+k;	

	     int curr_sc = d_data_carriers[sc_index];
	     int next_sc = d_data_carriers[sc_index+1];

	     out_coeffs[curr_sc*d_batch_size+k] = in_coeffs[curr_coeff_index];
	     if(i < end - 1) {
	         out_coeffs[next_sc*d_batch_size+k] = (in_coeffs[curr_coeff_index] + in_coeffs[next_coeff_index])/gr_complex(2.0, 0.0);
	     }
	     else {
		 out_coeffs[next_sc*d_batch_size+k] = in_coeffs[curr_coeff_index];
	     }

	     sc_index += 2;
	     //printf("i: %d, curr_sc: %d, next_sc: %d, sc_index: %d\n", i, curr_sc, next_sc, sc_index); fflush(stdout);
	  }

	  if(end == (d_data_carriers.size()/2)) break;

	  // update to fill the 2nd half now //
	  start = half_coeffs;
	  end = d_data_carriers.size()/2;
      }
   }

   assert(sc_index == d_data_carriers.size());
  
   
#if 0
   // debug interpolated coeffs //
   printf("before interpolation\n"); fflush(stdout);
   for(int i = 0; i < d_data_carriers.size()/2; i++) {
      for(unsigned int k = 0; k < d_batch_size; k++) {
         printf("coeff[%d]: (%f, %f)\t", k, in_coeffs[i*d_batch_size+k].real(), in_coeffs[i*d_batch_size+k].imag()); fflush(stdout);
      }
      printf("\n");
   }
   printf("------------------- \n"); fflush(stdout);
   printf("after interpolation\n"); fflush(stdout);
   for(int i = 0; i < d_occupied_carriers; i++) {
      printf("subcarrier: %d ", i); fflush(stdout);
      for(int k = 0; k < d_batch_size; k++) {
          printf("index:%d, coeff[%d]: (%f, %f)\t", i*d_batch_size+k, k, out_coeffs[i*d_batch_size+k].real(), out_coeffs[i*d_batch_size+k].imag()); fflush(stdout);
      }
      printf("\n");
   } 
#endif
}

void 
digital_ofdm_frame_sink::debugPktInfo(PktInfo *pktInfo, unsigned char sender)
{
  int num_senders = pktInfo->n_senders;
  //printf("debugPktInfo senders: %d\n", num_senders);
 
  vector<unsigned char> senders_vec = pktInfo->senders;
  vector<gr_complex*> coeffs_vec = pktInfo->coeffs;

  for(int i = 0; i < num_senders; i++) {
     gr_complex* coeffs = coeffs_vec.at(i);
     if(senders_vec.at(i) == sender) {
        printf("coeffs for sender%d : ", senders_vec.at(i));
        for(unsigned int k = 0; k < d_batch_size; k++) {
   	    printf("(%f, %f) ", coeffs[k].real(), coeffs[k].imag());	
	    //printf("(%f, %f) ", coeffs_vec[i][k].real(), coeffs_vec[i][k].imag()); fflush(stdout);
        }
        printf("\n");
	break;
     }
  }
}

void
digital_ofdm_frame_sink::debugFlowInfo(FlowInfo* flowInfo) {
  printf("debugFlowInfo: src: %d, dst: %d, flow: \n", flowInfo->src, flowInfo->dst, flowInfo->flowId); 
  printf("activeBatch: %d, last_batch_acked: %d\n", flowInfo->active_batch, flowInfo->last_batch_acked);

  InnovativePktInfoVector inno_pkts = flowInfo->innovative_pkts;
  printf("# of innovative pkts: %d\n", inno_pkts.size());
  for(int i = 0; i < inno_pkts.size(); i++) {
     PktInfo *pInfo = inno_pkts[i];
     printf("inno pkt#%d, senders: %d\n", i, pInfo->n_senders);
     printf("coeffs: ");
     for(int j = 0; j < pInfo->coeffs.size(); j++) {
	printf("(%f, %f) ", (pInfo->coeffs[j])->real(), (pInfo->coeffs[j])->imag());
     }
  }
}

/* generate symbols to forward: 
   0. Add the symbols into the 'd_pre_encoded_pkt_store': vector of innovative pkts seen for this flow
   1. Search the outgoing queue to see if any packet is already queued up for this composite-link/flow-id
   2. If yes:
	- update the pre-encoded pkt with this new innovative pkt
	- update the header as well
	- also add a copy of the updated pkt at the end of the queue (since the credit had incremented again!)
	- decrement the credit
   3. If no:
	- generate a new encoded pkt 
	- add the header
	- enqueue the pkt at the end of the outgoing queue
	- decrement the credit
*/


/* add the pkts to the innovative-store of the corresponding flow-id - all the packets belong to the same batch */
inline FlowInfo*
digital_ofdm_frame_sink::getFlowInfo(bool create, unsigned char flowId)
{ 
  //printf("getFlowInfo start iterating for flow%d\n", d_flow); fflush(stdout);
  vector<FlowInfo*>::iterator it = d_flowInfoVector.begin();
  FlowInfo *flow_info = NULL;
  while(it != d_flowInfoVector.end()) {
     flow_info = *it;
     if(flow_info->flowId == flowId)
	return flow_info;
     it++;
  }
  
  if(flow_info == NULL && create)
  {
     flow_info = (FlowInfo*) malloc(sizeof(FlowInfo));
     memset(flow_info, 0, sizeof(FlowInfo));
     flow_info->pkts_fwded = 0;
     d_flowInfoVector.push_back(flow_info);
  }

  return flow_info; 
}

/* the below functions are pretty much flicked from the mapper code */  
void
digital_ofdm_frame_sink::encodeSignal(gr_complex *symbols, gr_complex coeff)
{
  int n_data_carriers = d_data_carriers.size();
  printf("encodeSignal start\n"); fflush(stdout);
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++) {
     for(unsigned int j = 0; j < n_data_carriers; j++) {
	 //unsigned int index = (i*d_occupied_carriers) + j;
	 unsigned int index = (i*d_occupied_carriers) + d_data_carriers[j];
#ifdef SCALE
	 symbols[index] *= (coeff * gr_complex(SCALE));
#else
	 gr_complex t = symbols[index] * coeff;
         if(i == 0 && 0) {
           printf("(%f, %f) = (%f, %f) * (%f, %f)\n", t.real(), t.imag(), symbols[index].real(), symbols[index].imag(), coeff.real(), coeff.imag()); fflush(stdout);
	 }
	 symbols[index] = t;
#endif
     }
  }
}

void
digital_ofdm_frame_sink::combineSignal(gr_complex *out, gr_complex* symbols, int n)
{
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++)
     for(unsigned int j = 0; j < d_occupied_carriers; j++) {
#if 0
	 if(d_all_carriers[j] < 2) {
#else
	 if(d_all_carriers[j] == 0 || (d_all_carriers[j] == 1 && n == 0)) {
#endif
            unsigned int index = (i*d_occupied_carriers) + j;
	    gr_complex t = out[index] + symbols[index];
	    if(i == 0 && 0)
	    {
		printf("o: %d, sc: %d, (%f, %f) = (%f, %f) + (%f, %f)\n", i, j, t.real(), t.imag(), out[index].real(), out[index].imag(), symbols[index].real(), symbols[index].imag()); 
		fflush(stdout);
	    }
	    out[index] = t;
	}
     }

}

/* denormalize the pilot and data separately */
inline void
digital_ofdm_frame_sink::denormalizeSignal(gr_complex *in) {

  // data //
  for(int i = 0; i < d_data_carriers.size(); i++)
      in[d_data_carriers[i]] *= gr_complex(d_header.factor);
}


/* alternate version: based on the average magnitude seen in 1 OFDM symbol, either amp or de-amp the signal */
inline float
digital_ofdm_frame_sink::normalizeSignal(gr_complex* out, int k, int num_in_senders)
{
  int num_data_carriers = d_data_carriers.size();
  float factor = getNormalizationFactor(out, num_in_senders);
  printf("norm factor: %f\n", factor); fflush(stdout);

  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++)
     for(unsigned int j = 0; j < num_data_carriers; j++) {
         unsigned int index = (i*d_occupied_carriers) + d_data_carriers[j];
	 out[index] /= gr_complex(factor);
  }

  return factor;
}

inline float
digital_ofdm_frame_sink::getTimingOffset() {
  NodeIds rx_ids;
  get_nextHop_rx(rx_ids);

  if(rx_ids.size() == 1) {
     HInfo *hInfo = getHInfo(d_id, rx_ids[0]);
     return hInfo->timing_slope;
  }
  else {
     float to = 0.0;
     for(int i = 0; i < rx_ids.size(); i++) {
         HInfo *hInfo = getHInfo(d_id, rx_ids[i]);
         to += hInfo->timing_slope;
     }
     return to/((float) rx_ids.size());
   }
  return 0.0;
}

#if 0
void
digital_ofdm_frame_sink::generateCodeVector(MULTIHOP_HDR_TYPE &header)
{
  for(unsigned int k = 0; k < d_batch_size; k++)
  {
      float cv = rand() % 360 + 1;
      printf("[%f] ", cv);
      d_header.coeffs[k].phase = cv * SCALE_FACTOR_PHASE;
      d_header.coeffs[k].amplitude = SCALE_FACTOR_AMP;
  }
}
#endif

bool
digital_ofdm_frame_sink::isLeadSender() {
   return (d_fwd_index == 0 || d_fwd_index == 1);
}

/* handle the 'lead sender' case */
void
digital_ofdm_frame_sink::makeHeader(MULTIHOP_HDR_TYPE &header, unsigned char *header_bytes, FlowInfo *flowInfo, unsigned int nextLinkId, float timing_offset)
{
   printf("batch#: %d, makeHeader for pkt: %d, len: %d\n", flowInfo->active_batch, d_pkt_num, d_packetlen); fflush(stdout);
   header.src_id = flowInfo->src; 
   header.dst_id = flowInfo->dst;       
   header.prev_hop_id = d_id;	
   header.batch_number = flowInfo->active_batch;

   header.packetlen = d_packetlen;
   if(d_fwd_index == 0)
      header.nsenders = 1;                                    
   else
      header.nsenders = 2;

   header.pkt_type = DATA_TYPE;
   if(isLeadSender()) 
        header.lead_sender = 1;
   else
	header.lead_sender = 0;

   //header.pkt_num = flowInfo->pkts_fwded;	      // TODO: used for debugging, need to maintain pkt_num for flow 
   header.pkt_num = d_pkt_num;
   header.link_id = nextLinkId;

   header.timing_offset = timing_offset;	   // if d_h_coding is not used, timing_offset = 0
 
   flowInfo->pkts_fwded++;

   for(int i = 0; i < PADDING_SIZE; i++)
        header.pad[i] = 0;

   unsigned char header_data_bytes[HEADERDATALEN];
   memcpy(header_data_bytes, &header, HEADERDATALEN);                         // copy e'thing but the crc

   unsigned int calc_crc = digital_crc32(header_data_bytes, HEADERDATALEN);
   header.hdr_crc = calc_crc;

   memcpy(header_bytes, header_data_bytes, HEADERDATALEN);                            // copy header data
   memcpy(header_bytes+HEADERDATALEN, &calc_crc, sizeof(int));                // copy header crc

   memcpy(header_bytes+HEADERDATALEN+sizeof(int), header.pad, PADDING_SIZE);
   printf("len: %d, crc: %u\n", d_packetlen, calc_crc);

   whiten(header_bytes, HEADERBYTELEN);
   //debugHeader(header_bytes);
}


void
digital_ofdm_frame_sink::debugHeader(unsigned char *header_bytes)
{
   printf("debugHeader\n"); fflush(stdout);
   whiten(header_bytes, HEADERBYTELEN);    // dewhitening effect !
   MULTIHOP_HDR_TYPE header;
   memset(&header, 0, HEADERBYTELEN);
   assert(HEADERBYTELEN == (HEADERDATALEN + sizeof(int) + PADDING_SIZE));
   memcpy(&header, header_bytes, HEADERBYTELEN);

   printf("batch#: %d, debug crc: %u\n", header.batch_number, header.hdr_crc);

   for(unsigned int k = 0; k < d_batch_size; k++) {
	COEFF coeff = header.coeffs[k];
        printf("[%f] ", ToPhase_f(coeff));
   }
   printf("\n");
   whiten(header_bytes, HEADERBYTELEN);
}


/* forwarder operations */
/* utility function: if 'src', then only checks the srcIds of the composite links, else checks the destinations */
inline bool
digital_ofdm_frame_sink::isSameNodeList(vector<unsigned char> ids1, vector<unsigned char> ids2)
{
   if(ids1.size() != ids2.size())
	return false;

   int size = ids1.size();
   for(int i = 0; i < size; i++) {
	if(ids1[i] != ids2[i])
	    return false;
   }
   return true;
}

CompositeLink*
digital_ofdm_frame_sink::getCompositeLink(int id)
{
   CompositeLinkVector::iterator it = d_compositeLinkVector[d_flow].begin();
   while(it != d_compositeLinkVector[d_flow].end()) {
	CompositeLink *cLink = *it;
	if(cLink->linkId == id)
	    return cLink;
	it++;
   }
   return NULL;
}

inline void
digital_ofdm_frame_sink::populateCreditInfo()
{
    // flowId   credit   previousLink-id   nextLink-id //
   printf("populateCreditInfo\n"); fflush(stdout);
   FILE *fl = fopen ( "credit_info.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
	assert(false);
   }

   char line[128];
   const char *delim = " ";
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token;
        while (strtok_res != NULL) {
           token.push_back(strtok_res);
           strtok_res = strtok (NULL, delim);
        }

	CreditInfo* cInfo = (CreditInfo*) malloc(sizeof(CreditInfo));
	memset(cInfo, 0, sizeof(CreditInfo));
        cInfo->flowId = atoi(token[0]);
        cInfo->delta = atof(token[1]);

        int previousLinkId = atoi(token[2]);
        CompositeLink *cLink = getCompositeLink(previousLinkId);
        assert(cLink);
        memcpy(&(cInfo->previousLink), cLink, sizeof(CompositeLink));

        int nextLinkId = atoi(token[3]);
        cLink = getCompositeLink(nextLinkId);
        assert(cLink);
        memcpy(&(cInfo->nextLink), cLink, sizeof(CompositeLink));

	d_creditInfoVector.push_back(cInfo);
   }

   fclose (fl);
}

inline void
digital_ofdm_frame_sink::populateCompositeLinkInfo()
{
   // link-id   #of-src     src1   src2   src3    #of-dst   dst1   dst2    dst3  flowId  //
   printf("populateCompositeLinkInfo\n"); fflush(stdout);

   FILE *fl = fopen ( "compositeLink_info.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
        exit (1);
   }

   char line[128];
   const char *delim = " ";
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token_vec;
        while (strtok_res != NULL) {
           token_vec.push_back(strtok_res);
           //printf ("%s\n", strtok_res);
           strtok_res = strtok (NULL, delim);
        }
	
	printf("size: %d\n", token_vec.size()); fflush(stdout);

        CompositeLink *cLink = (CompositeLink*) malloc(sizeof(CompositeLink));
	memset(cLink, 0, sizeof(CompositeLink));
        cLink->linkId = atoi(token_vec[0]);      
        printf("linkId: %d\n", cLink->linkId); fflush(stdout);
        
        int num_src = atoi(token_vec[1]);
        printf("num_src: %d\n", num_src); fflush(stdout);
        for(int i = 0; i < num_src; i++) {
           //unsigned int srcId = atoi(token_vec[i+2]);
	   NodeId srcId = atoi(token_vec[i+2]) + '0';
           printf("srcId: %c, size: %d\n", srcId, cLink->srcIds.size()); fflush(stdout);
           cLink->srcIds.push_back(srcId);
	   if(srcId == d_id)
	     d_outCLinks.push_back(cLink->linkId);
        }

        int num_dst = atoi(token_vec[num_src+2]);
        printf("num_dst: %d\n", num_dst); fflush(stdout);
        for(int i = 0; i < num_dst; i++) {
           //unsigned int dstId = atoi(token_vec[num_src+3+i]);
	   NodeId dstId = atoi(token_vec[num_src+3+i]) + '0';
           printf("dstId: %c\n", dstId); fflush(stdout);
           cLink->dstIds.push_back(dstId);
	   if(dstId == d_id)
	     d_inCLinks.push_back(cLink->linkId);
        }
        
	int flowId = atoi(token_vec[3+num_src+num_dst]);
        d_compositeLinkVector[flowId].push_back(cLink);


        printf("\n");
   }

    fclose (fl);
}

/* perform CreditTable lookup and update the CreditInfo entries for the (prev-link, flow)*/
void
digital_ofdm_frame_sink::updateCredit()
{
   printf("updateCredit for prev-Link: %d\n", d_pktInfo->prevLinkId); fflush(stdout);
   int size = d_creditInfoVector.size();
   assert(size > 0);
   CreditInfo* creditInfo = NULL;
  
   bool updated = false;

   /* find creditInfo entry */
   for(int i = 0; i < size; i++) {
      creditInfo = d_creditInfoVector[i];
      //printf("credit flow: %d, d_flow: %d\n", creditInfo->flowId, d_flow); fflush(stdout);
      if(creditInfo->flowId == d_flow) {
	 CompositeLink link = creditInfo->previousLink;
	 assert(d_pktInfo && d_pktInfo->senders.size() > 0);
	
	 printf("iterating link: %d\n", link.linkId); fflush(stdout);
	 if(link.linkId == d_pktInfo->prevLinkId) {
	     updated = true;
	     int n_senders = d_pktInfo->senders.size();
	     creditInfo->credit += (creditInfo->delta) * (n_senders/((link.srcIds).size())); 
	 }
      }
      printf("updated credit: %f\n", creditInfo->credit); fflush(stdout);
   }
   assert(updated);
}

/* called from the outside directly by the wrapper, asking for any pkt to forward */
void
digital_ofdm_frame_sink::makePacket(bool sync_send)
{
   printf("makePacket syncSend: %d\n", sync_send); fflush(stdout);
  
   /* check if any data needs to be sent out */
   /* credit check*/
   int count = d_creditInfoVector.size();
   CreditInfo* creditInfo = NULL;
   bool found = false;
   if(count > 0) {
      for(int i = 0; i < count; i++) {
	  creditInfo = d_creditInfoVector[i];
#if 1
	  if(creditInfo->credit >= 1.0 && creditInfo->flowId == d_flow) {
             found = true;
             break;
          }
#else
	  if(creditInfo->previousLink.linkId == d_prevLinkId) {
	     found = true; 
	     break;
	  }
#endif
      }
   } 

   /* credit >= 1.0 found, packet needs to be created! */
   if(found) {
      assert(creditInfo);
      printf("sink::makePacket - DATA to send, flow: %d, credit: %f\n", creditInfo->flowId, creditInfo->credit); fflush(stdout);

      /* create the pkt now! */
      FlowInfo *flowInfo = getFlowInfo(false, creditInfo->flowId);
      assert(flowInfo);

#if 1
      while(creditInfo->credit >= 1.0) {
         encodePktToFwd(creditInfo, sync_send);
	 creditInfo->credit -= 1.0;
	 num_data_fwd++; 
      }
#else	
      encodePktToFwd(creditInfo, sync_send);
      creditInfo->credit -= 1.0;
      num_data_fwd++;
#endif
   }
   //printf("sink::makePacket - nothing to send\n"); fflush(stdout);
}

#ifdef USE_PILOT
/* reduce the coefficients using both the hestimates and the coeff used by the node upstream */
inline void
digital_ofdm_frame_sink::reduceCoefficients(FlowInfo *flowInfo) {

  /* get the interpolated coefficients  */
  int num_senders = d_pktInfo->n_senders;
  printf("reduceCoefficients begin :: senders: %d\n", num_senders); fflush(stdout); 

  vector<gr_complex*> interpolated_coeffs;
  for(int k = 0; k < num_senders; k++) {
      gr_complex *rx_coeffs = d_pktInfo->coeffs.at(k);
      gr_complex *out_coeffs = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * d_occupied_carriers);
      memset(out_coeffs, 0, sizeof(gr_complex) *  d_batch_size * d_occupied_carriers);

      //interpolate_coeffs(rx_coeffs, out_coeffs);
      interpolate_coeffs_lerp(rx_coeffs, out_coeffs);
      interpolated_coeffs.push_back(out_coeffs);
  }

  /* now perform the reduction */
  gr_complex *coeff = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * d_occupied_carriers);
  memset(coeff, 0, sizeof(gr_complex) * d_batch_size * d_occupied_carriers);
  flowInfo->reduced_coeffs.push_back(coeff);

  for(unsigned int i = 0; i < d_occupied_carriers; i++)             // input signature: d_occupied_carriers * hestimates
  {
     gr_complex this_coeff;
     for(unsigned int j = 0; j < d_batch_size; j++)
     {
	int coeff_index = (i * d_batch_size + j);
	for(int k = 0; k < num_senders; k++)
	{
	    gr_complex *estimates = d_pktInfo->hestimates[k];                    // estimates for 'kth' sender
	    gr_complex *in_coeffs = interpolated_coeffs[k];                      // interpolated coeffs for 'kth' sender

	    if(k == 0) {
	      this_coeff = ((gr_complex(1.0, 0.0)/estimates[i]) * in_coeffs[coeff_index]);
	      //printf("sub: %d, (%f, %f) = 1/(%f, %f) * (%f, %f)\n", i, this_coeff.real(), this_coeff.imag(), estimates[i].real(), estimates[i].imag(), in_coeffs[coeff_index].real(), in_coeffs[coeff_index].imag()); fflush(stdout);
	    }
	    else {
	      this_coeff += ((gr_complex(1.0, 0.0)/estimates[i]) * in_coeffs[coeff_index]);
	    }
	}

	// each subcarrier will have 'd_batch_size' # of coefficients //
	coeff[coeff_index] = this_coeff;
	//printf("i: %d, batch: %d, k: %d, coeff_index: %d, coeff: (%f, %f)\n", i, j, k, coeff_index, this_coeff.real(), this_coeff.imag()); fflush(stdout);
     }
  }   
 
  /* clean up */
  for(int i = 0; i < num_senders; i++) {
      gr_complex *out_coeffs = interpolated_coeffs.at(i);
      free(out_coeffs);
  }
  interpolated_coeffs.clear();
  printf("reduceCoeffients end\n"); fflush(stdout);
}

inline void
digital_ofdm_frame_sink::packCoefficientsInHeader(MULTIHOP_HDR_TYPE& header, gr_complex* coeffs, 
						  int num_inno_pkts, FlowInfo *flowInfo)
{
  printf("packCoefficientsInHeader\n"); fflush(stdout);
  /* do for each subcarrier, since 'h' values are on subcarrier basis
     - half the subcarriers are really needed, the other half are interpolated */
  int num_coeffs_batch = d_data_carriers.size()/COMPRESSION_FACTOR;
  assert(d_data_carriers.size() % COMPRESSION_FACTOR == 0);

  COEFF *new_coeffs = (COEFF*) malloc(sizeof(COEFF) * d_batch_size * num_coeffs_batch);
  memset(new_coeffs, 0, sizeof(COEFF) * d_batch_size * num_coeffs_batch);

  reduceCoefficients(flowInfo);							// fill up flowInfo->reduced_coeff //

  /* how many subcarriers to fill */
  int dc_tones = d_occupied_carriers - (d_data_carriers.size()+d_pilot_carriers.size());
  unsigned int half_occupied_tones = (d_occupied_carriers - dc_tones)/2;

  printf("dc_tones: %d, half: %d\n", dc_tones, half_occupied_tones); fflush(stdout);
  int count = 0;
  int num_data_carriers = d_data_carriers.size();

  /* each 'subcarrier' contains 'd_batch_size' coeffs for each subcarrier */
  for(unsigned int s = 0; s < num_data_carriers; s+=COMPRESSION_FACTOR) { 
     int _carrier_index = d_data_carriers[s];
     //printf("subcarrier: %d\n", _carrier_index); fflush(stdout);
 
     /* generate <d_batch_size> coeffs */
     for(unsigned int i = 0; i < d_batch_size; i++) {
        gr_complex new_coeff_c;                                                 // for each native pkt in batch, ie P1 and P2 //

        for(int j = 0; j < num_inno_pkts; j++) {
           gr_complex *pkt_coeffs = flowInfo->reduced_coeffs[j];        // reduced coeffs for this inno pkt //

           int coeff_index = _carrier_index * d_batch_size + i;
           if(j == 0) {
                new_coeff_c = coeffs[j] * pkt_coeffs[coeff_index];
#ifdef DEBUG
		printf("subcarrier: %d, (%f, %f) * (%f, %f) ", _carrier_index, coeffs[j].real(), coeffs[j].imag(), pkt_coeffs[coeff_index].real(), pkt_coeffs[coeff_index].imag());
#endif
           }
           else {
                new_coeff_c += (coeffs[j] * pkt_coeffs[coeff_index]);
           }
        }

        new_coeffs[count].amplitude = abs(new_coeff_c) * SCALE_FACTOR_AMP;
        new_coeffs[count].phase = ToPhase_f(new_coeff_c) * SCALE_FACTOR_PHASE;

#ifdef DEBUG
        printf(" = [z=%f p=%f] <--> (%f, %f)\n ", abs(new_coeff_c), ToPhase_f(new_coeff_c), new_coeff_c.real(), new_coeff_c.imag()); fflush(stdout);
#endif
        count++;
     }
  }

  if(count != (d_batch_size * num_coeffs_batch)) {
        printf("count: %d, num_coeffs_batch: %d, batch_size: %d\n", count, num_coeffs_batch, d_batch_size); fflush(stdout);
        assert(false);
  }

  memcpy(header.coeffs, new_coeffs, sizeof(COEFF) * count);
  free(new_coeffs); 
 
  printf("packCoefficientsInHeader end\n"); fflush(stdout); 
}
#endif

inline void
digital_ofdm_frame_sink::calculate_fine_offset() {
  
  int num = NUM_TRAINING_SYMBOLS;		//2;
  /*
  for(int o = 0; o < num - 1; o++) {  
  gr_complex *rx_symbols1 = &(d_training_symbols[o*MAX_OCCUPIED_CARRIERS]);
  gr_complex *rx_symbols2 = &(d_training_symbols[(o+1) * MAX_OCCUPIED_CARRIERS]);

  gr_complex phase_error = 0.0;
  for(int i = 0; i < d_all_carriers.size(); i++) {
      if(d_all_carriers[i] < 2) {
	 //printf("PE: %.3f\n", arg(rx_symbols1[i] * conj(rx_symbols2[i])));
         phase_error += (rx_symbols1[i] * conj(rx_symbols2[i]));
      }
  }
  float angle = arg(phase_error);
  float freq_offset = -angle/(2 * M_PI);

  printf("angle: %.3f freq_offset: %.3f\n", angle, freq_offset);
  
  for(int i = 0; i < d_all_carriers.size(); i++) {
      if(d_all_carriers[i] < 2) {
	  //rx_symbols1[i] *= gr_expj(2 * M_PI * freq_offset * i * );
      }
  }
  }
  */
  // optionally log training symbols for offline analysis //
   if(!d_training_log_open) {
      const char *filename = "training_symbols.dat";
      int fd;
      if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename);
         assert(false);
      }
      else {
         if((d_fp_training = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "training log file cannot be opened\n");
            close(fd);
            assert(false);
         }
      }
      d_training_log_open = true;
   }

   assert(d_fp_training);
   int count = ftell(d_fp_training);
   count = fwrite_unlocked(&d_training_symbols[0], sizeof(gr_complex), num * MAX_OCCUPIED_CARRIERS, d_fp_training);
   //assert(count == num * MAX_OCCUPIED_CARRIERS);
   // logging ends //
}

#if 0
inline void 
digital_ofdm_frame_sink::debug_carrier_correction() {
  if(d_pktInfo->n_senders > 1) {
     return;
  }

  //calculate_fine_offset();

  gr_complex t_symbols[MAX_OCCUPIED_CARRIERS];
  gr_complex *hestimates = d_pktInfo->hestimates[0];

#if 0
     int gap = NULL_OFDM_SYMBOLS;               // t'-t

     gr_complex *hestimates = d_pktInfo->hestimates[0];	
     printf("d_slope_angle: %.3f, d_end_angle: %.3f\n", d_slope_angle[0], d_end_angle[0]); fflush(stdout);

     printf("Est angle -- \n"); fflush(stdout);	
     for(int i = 0; i < d_num_ofdm_symbols; i++) {
	int index = i * d_occupied_carriers;

	gr_complex *rx_symbols = &(d_pktInfo->symbols[index]);
	memcpy(t_symbols, rx_symbols, sizeof(gr_complex) * d_occupied_carriers);

	float cur_pilot = 1.0;
	gr_complex phase_error = 0.0;

	float est_angle = d_slope_angle[0] * gap + d_end_angle[0];
	gr_complex carrier = gr_expj(est_angle - d_end_angle[0]);
        for(int j = 0; j < d_pilot_carriers.size(); j++) {
           int pi = d_pilot_carriers[j];
           t_symbols[pi] = t_symbols[pi] * carrier * hestimates[pi];
	   gr_complex pilot_sym(cur_pilot, 0.0);
	   cur_pilot = -cur_pilot;
	   phase_error += (t_symbols[pi] * conj(pilot_sym));
	}
	printf("AN: %d [%.3f]\n", i, arg(phase_error)); fflush(stdout);
	gap++;
     }
#endif

     printf("True angle -- \n"); fflush(stdout);	
     for(int i = 0; i < d_num_ofdm_symbols; i++) {
	int index = i * d_occupied_carriers;

        gr_complex *rx_symbols = &(d_pktInfo->symbols[index]);
        memcpy(t_symbols, rx_symbols, sizeof(gr_complex) * d_occupied_carriers);

        float cur_pilot = 1.0;
        gr_complex phase_error = 0.0;

	for(int j = 0; j < d_pilot_carriers.size(); j++) {
	   int pi = d_pilot_carriers[j];
	   t_symbols[pi] = t_symbols[pi] * hestimates[pi];		// equalize the symbols
	
	   gr_complex pilot_sym(cur_pilot, 0.0);
	   cur_pilot = -cur_pilot;
	   
	   phase_error += (t_symbols[pi] * conj(pilot_sym));	   	
	}
	float true_angle = arg(phase_error);
	printf("AN: %d [%.3f]\n", i, true_angle); fflush(stdout);
     }
}
#endif

#if 1
inline bool
digital_ofdm_frame_sink::do_carrier_correction() {
  if(d_pktInfo->n_senders > 1) {
     adjust_H_estimate(0);
     return false;
  }

  return false;
#ifdef SRC_PILOT
  /* if I'm part of synchronized forwarders for this packet, I need to introduce an extra 
     rotation corresponding to (t'-t) in all the symbols for this packet */
  if(d_fwd_index > 0) {
     int gap = d_num_hdr_ofdm_symbols+d_preamble.size();              // t'-t

     float rot_val[MAX_OFDM_SYMBOLS];
     printf("d_num_ofdm_symbols: %d, gap: %d\n", d_num_ofdm_symbols, gap); fflush(stdout);
     assert(d_num_ofdm_symbols > gap);
     getResidualRotations(rot_val);

     for(int i = 0; i < d_num_ofdm_symbols; i++) {
        int index = i * d_occupied_carriers;
        gr_complex *rx_symbols = &(d_pktInfo->symbols[index]);
        gr_complex delta_carrier = gr_expj(rot_val[i+gap] - rot_val[i]);
        for(int j = 0; j < d_occupied_carriers; j++) {
           rx_symbols[j] *= delta_carrier;
        }
     }
  }

#if 0
     int i = 0;
     for(i = 0; (i+gap) < d_num_ofdm_symbols; i++) {
	int index = i * d_occupied_carriers;
	gr_complex *rx_symbols = &(d_pktInfo->symbols[index]);

	for(int j = 0; j < d_occupied_carriers; j++) {
	   gr_complex delta_carrier = gr_expj(rot_val[i+gap] - rot_val[i]);
	   rx_symbols[j] *= delta_carrier;
	}
     }

     float end_angle = rot_val[d_num_ofdm_symbols-1];
     float slope_angle = (end_angle - rot_val[0])/((float) d_num_ofdm_symbols);
    
     int count = 1; 
     while(i < d_num_ofdm_symbols) {
	int index = i * d_occupied_carriers;
	gr_complex *rx_symbols = &(d_pktInfo->symbols[index]);
	float est_angle = slope_angle * count + end_angle;

	for(int j = 0; j < d_occupied_carriers; j++) {
	   rx_symbols[j] *= gr_expj(est_angle - rot_val[i]);  
	}
	i++;
	count++;
     }
  }
#endif

  // debug only //
  gr_complex *hestimates = d_pktInfo->hestimates[0];
  for(int d = 0; d < d_num_ofdm_symbols; d++) {
     int index = d * d_occupied_carriers;
     gr_complex *rx_symbols = &(d_pktInfo->symbols[index]);

     float cur_pilot = 1.0;
     gr_complex phase_error = 0.0;

     for(int j = 0; j < d_pilot_carriers.size(); j++) {
         int pi = d_pilot_carriers[j];
         gr_complex pilot_sym(cur_pilot, 0.0);
         cur_pilot = -cur_pilot;
         phase_error += (rx_symbols[pi] * hestimates[pi] * conj(pilot_sym));
     }

     float new_angle = arg(phase_error);
     printf("new_angle: %d [%.3f]\n", d, new_angle); fflush(stdout);
  }

  return true;
#endif

  vector<gr_complex> dfe_pilot;
  dfe_pilot.resize(d_pilot_carriers.size());
  fill(dfe_pilot.begin(), dfe_pilot.end(), gr_complex(1.0, 0.0));

  for(int i = 0; i < d_num_ofdm_symbols; i++) {
     int index = i * d_occupied_carriers;
     vector <gr_complex> dfe_data; gr_complex carrier;
     gr_complex *rx_symbols = &(d_pktInfo->symbols[index]);

     track_pilot_dfe(rx_symbols, 0, carrier, dfe_pilot);
     //printf("ofdm_index: %d, d_end_angle: %f, d_phase: %f, d_freq: %f\n", i+1, d_end_angle[0], d_phase[0], d_freq[0]);
     fflush(stdout);

     interpolate_data_dfe(dfe_pilot, dfe_data, true, rx_symbols, carrier);
     assert(dfe_data.size() == d_data_carriers.size());
  }

  return true;
}
#else
/* done as a forwarder - if the packet is received only from one sender */
inline bool
digital_ofdm_frame_sink::do_carrier_correction() {
  if(d_pktInfo->n_senders > 1) {
     assert(d_pktInfo->n_senders == 2);
     adjust_H_estimate(1);
     return false;
  }

#ifdef SRC_PILOT
  /* if I'm part of synchronized forwarders for this packet, I need to introduce an extra 
     rotation corresponding to (t'-t) in all the symbols for this packet */
  if(d_fwd_index > 0) {
     int gap = d_num_hdr_ofdm_symbols+d_preamble.size();               // t'-t

     for(int i = 0; i < d_num_ofdm_symbols; i++) {
        // get the delta rotation for (t'-t) gap //
        float est_angle = d_slope_angle[0] * gap + d_end_angle[0];
        gr_complex carrier = gr_expj(est_angle - d_end_angle[0]);

        // add the extra rotation now //
        int index = i * d_occupied_carriers;
        gr_complex *rx_symbols = &(d_pktInfo->symbols[index]);

#ifndef DEBUG
        float cur_pilot = 1.0;
	gr_complex phase_error = 0.0;
	gr_complex *hestimates = d_pktInfo->hestimates[0];
#endif

        for(int j = 0; j < d_occupied_carriers; j++) {

#ifndef DEBUG
	   if(d_all_carriers[j] == 1) {
	      gr_complex pilot_sym(cur_pilot, 0.0);
	      cur_pilot = -cur_pilot;
	      phase_error += ((rx_symbols[j] * hestimates[j]) * conj(pilot_sym));
	   }
#endif
           rx_symbols[j] = rx_symbols[j] * carrier; 
        }

#ifndef DEBUG
	printf("o: %d, rotation: %.3f\n", i, arg(phase_error));
#endif

        gap++;
     }
  } 
  return true;
#endif

  assert(false); 			// code is not reachable - need to figure out what was the reason for this code!!!!! 

  vector<gr_complex> dfe_pilot;
  dfe_pilot.resize(d_pilot_carriers.size());
  fill(dfe_pilot.begin(), dfe_pilot.end(), gr_complex(1.0, 0.0)); 

  for(int i = 0; i < d_num_ofdm_symbols; i++) {
     int index = i * d_occupied_carriers;
     vector <gr_complex> dfe_data; gr_complex carrier;
     gr_complex *rx_symbols = &(d_pktInfo->symbols[index]);

     track_pilot_dfe(rx_symbols, 0, carrier, dfe_pilot);
     //printf("ofdm_index: %d, d_end_angle: %f, d_phase: %f, d_freq: %f\n", i+1, d_end_angle[0], d_phase[0], d_freq[0]);
     fflush(stdout);

     interpolate_data_dfe(dfe_pilot, dfe_data, true, rx_symbols, carrier); 
     assert(dfe_data.size() == d_data_carriers.size());
  }

  return true;
}
#endif

inline void
digital_ofdm_frame_sink::getResidualRotations(float *rot_val) {
     printf("getResidualRotations -- \n"); fflush(stdout);
     gr_complex *hestimates = d_pktInfo->hestimates[0];
     gr_complex t_symbols[MAX_OCCUPIED_CARRIERS];

     fmat Y = randu<fmat> (d_num_ofdm_symbols, 1);                              // for LSQ //     

     // calculate the residual rotations for all the OFDM symbols //
     for(int i = 0; i < d_num_ofdm_symbols; i++) {
        int index = i * d_occupied_carriers;

        gr_complex *rx_symbols = &(d_pktInfo->symbols[index]);
        memcpy(t_symbols, rx_symbols, sizeof(gr_complex) * d_occupied_carriers);

        float cur_pilot = 1.0;
        gr_complex phase_error = 0.0;

        for(int j = 0; j < d_pilot_carriers.size(); j++) {
           int pi = d_pilot_carriers[j];
           t_symbols[pi] = t_symbols[pi] * hestimates[pi];
           gr_complex pilot_sym(cur_pilot, 0.0);
           cur_pilot = -cur_pilot;
           phase_error += (t_symbols[pi] * conj(pilot_sym));
        }
        rot_val[i] = arg(phase_error);
        Y(i, 0) = rot_val[i];                                                   // for LSQ //
        printf("true_angle: %d (%.3f) \n", i, rot_val[i]); fflush(stdout);
     }


     // LSQ to determine the coefficients //
     int degree = 3;
     fmat A = randu<fmat> (degree+1, 1);
     fmat X = randu<fmat> (d_num_ofdm_symbols, degree+1);

     // build X //
     for(unsigned int i = 0; i < d_num_ofdm_symbols; i++) {
        for(unsigned int j = 0; j < degree+1; j++) {
           X(i, j) = pow(i, j);
        }
     }

     // get A //
     A = solve(X, Y);

     // now dump the future interpolated points //
     //for(int i = d_num_ofdm_symbols; i < NULL_OFDM_SYMBOLS + d_num_ofdm_symbols; i++) {
     for(int i = d_num_ofdm_symbols; i < (d_preamble.size()+d_num_hdr_ofdm_symbols+d_num_ofdm_symbols); i++) {
         for(unsigned int j = 0; j < degree+1; j++) {
             rot_val[i] += (A(j, 0) * pow(i, j));
         }
     }
}


inline float
digital_ofdm_frame_sink::getAvgAmplificationFactor(vector<gr_complex*> hestimates)
{
  unsigned int n_data_carriers = d_data_carriers.size();
  float avg_atten[MAX_SENDERS];
  memset(avg_atten, 0, sizeof(float) * MAX_SENDERS);

  int n_senders = hestimates.size();
  for(unsigned int i = 0; i < n_senders; i++) {
      gr_complex *hest = hestimates[i];
      for(unsigned int j = 0; j < n_data_carriers; j++) {
         avg_atten[i] += abs(hest[d_data_carriers[j]]);
      }
      avg_atten[i] /= (float) n_data_carriers;
      printf("sender: %d, avg_amp: %.3f\n", i, avg_atten[i]);
  }

  return avg_atten[0];             // attenuation //
}

#ifdef SRC_PILOT
/* just remove the channel effect from pilot and ensure it has a magnitude of 1.0 */
inline void
digital_ofdm_frame_sink::equalizePilot(gr_complex *in, PktInfo *pInfo) {
  //printf("equalizePilot \n"); fflush(stdout);

  int n_pilots = d_pilot_carriers.size();
  int num_senders = pInfo->hestimates.size();
  gr_complex *estimates = (pInfo->hestimates)[num_senders-1];

  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++) {

     float cur_pilot = 1.0;
     gr_complex phase_error = 0.0;

     for(unsigned int j = 0; j < n_pilots; j++) {
         unsigned int index = (i*d_occupied_carriers) + d_pilot_carriers[j];
	 //printf("i: %d, j: %d, sym: (%.3f, %.3f)\n", i, j, in[index].real(), in[index].imag()); fflush(stdout);
         in[index] *= (estimates[d_pilot_carriers[j]]);			// equalize the pilot

	 float amp = 1.0/abs(in[index]);
	 gr_complex factor = amp * gr_expj(0.0);
	 in[index] *= factor;						// amplify to mangitude of 1.0

	 // debug //
#if 0
	 gr_complex pilot_sym(cur_pilot, 0.0);
	 phase_error += ((in[index]) * conj(pilot_sym));

	 float angle = arg(in[index]) * 180/M_PI;			// rotation after equalization //
	 if(angle < 0) angle += 360.0;
	
	 float perfect_angle = arg(pilot_sym) * 180/M_PI;	 
	 cur_pilot = -cur_pilot;
	 float delta_rotation = perfect_angle - angle;
	 //printf("pilot %d, A=%.3f, P=%.3f, delta_P=%.3f\n", d_pilot_carriers[j], abs(in[index]), angle, delta_rotation); fflush(stdout);
	 //printf("(%d, rot=%.2f)  ", d_pilot_carriers[j], delta_rotation); fflush(stdout);
#endif
     }
     //float end_angle = arg(phase_error);
     //printf("\npilot_angle: %.3f", end_angle);
     //printf("\n"); fflush(stdout);
  }
  //printf(" --------------- equalizePilot ends --------------- \n"); fflush(stdout);
}
#endif

void
digital_ofdm_frame_sink::encodePktToFwd(CreditInfo *creditInfo, bool sync_send)
{
  unsigned char flowId = creditInfo->flowId;
#ifndef DEBUG
  printf("encodePktToFwd, flowId: %d\n", flowId); fflush(stdout);
#endif

  /* get FlowInfo */
  FlowInfo *flow_info = getFlowInfo(false, flowId);
  assert((flow_info != NULL) && (flow_info->flowId == flowId));
  InnovativePktInfoVector inno_pkts = flow_info->innovative_pkts;

  /* coefficients */
  unsigned int n_innovative_pkts = inno_pkts.size();
  assert(n_innovative_pkts > 0);
  gr_complex *coeffs = (gr_complex*) malloc(sizeof(gr_complex) * n_innovative_pkts);
  memset(coeffs, 0, sizeof(gr_complex) * n_innovative_pkts);

  /* final coded symbols */
  unsigned int n_symbols = d_num_ofdm_symbols * d_occupied_carriers;

  gr_complex *out_symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_symbols);
  memset(out_symbols, 0, sizeof(gr_complex) * n_symbols);

  // calculate the outgoing timestamp // 
  uint64_t sync_secs; double sync_frac_of_secs;
  if(sync_send) {
     calc_outgoing_timestamp(sync_secs, sync_frac_of_secs);
     //d_out_pkt_time = uhd::time_spec_t(sync_secs, sync_frac_of_secs);
  }

  if(d_h_coding) {
     // update pkt time stamp vector 
     d_pktTxInfoList.push_back(PktTxInfo(d_pkt_num, d_out_pkt_time));
     while(d_pktTxInfoList.size() > 300) {
        d_pktTxInfoList.pop_front();
     }

     reduceCoefficients_LSQ(flow_info);
     chooseCV_H(flow_info, coeffs);

     gr_complex *symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_symbols);
     for(unsigned int i = 0; i < n_innovative_pkts; i++) {
	 PktInfo *pInfo = inno_pkts[i];
         memcpy(symbols, pInfo->symbols, sizeof(gr_complex) * n_symbols);

	 encodeSignal(symbols, coeffs[i]);
#ifdef SRC_PILOT
	 equalizePilot(symbols, pInfo);                 // equalize the pilot tones to remove the channel effect //
#endif
	 combineSignal(out_symbols, symbols, i);
     }
     free(symbols); 
  }
#if 0
  else {
     /* pick new random <coeffs> for each innovative pkt to == new coded pkt */
     printf("selecting random coeffs::: --- \n"); fflush(stdout);
     float phase[MAX_BATCH_SIZE];
     for(unsigned int i = 0; i < n_innovative_pkts; i++) {
         PktInfo *pInfo = inno_pkts[i];

	 gr_complex *symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_symbols);
	 memcpy(symbols, pInfo->symbols, sizeof(gr_complex) * n_symbols);

	 phase[i] = rand() % 360 + 1;
	 float amp = getAvgAmplificationFactor(pInfo->hestimates);
         coeffs[i] = amp * gr_expj(phase[i] * M_PI/180);

#ifndef DEBUG
	 printf("generateCodeVector: [A: %f, P: %f] <-> (%f, %f)\n", amp, phase[i], coeffs[i].real(), coeffs[i].imag()); fflush(stdout);
#endif
	 encodeSignal(symbols, coeffs[i]);

#ifdef SRC_PILOT
	 equalizePilot(symbols, pInfo);			// equalize the pilot tones to remove the channel effect //
#endif
	 combineSignal(out_symbols, symbols, i);

	 free(symbols);
     }
     printf("--------------------------------------------------------------------- \n"); fflush(stdout);
  }
#endif

  //logGeneratedTxSymbols(out_symbols);
  float factor = normalizeSignal(out_symbols, n_innovative_pkts, ((d_fwd_index==0)?1:2));

  /* make header */
  unsigned char header_bytes[HEADERBYTELEN];
  MULTIHOP_HDR_TYPE header;
  memset(&header, 0, sizeof(MULTIHOP_HDR_TYPE));

#ifdef LSQ_COMPRESSION
  packCoefficientsInHeader_LSQ(header, coeffs, n_innovative_pkts, flow_info);
#else
  packCoefficientsInHeader(header, coeffs, n_innovative_pkts, flow_info);
#endif

  header.inno_pkts = n_innovative_pkts;
  header.factor = factor;

  float timing_offset = 0.0;
  if(d_h_coding) {
     timing_offset = getTimingOffset();
  }

  /* populate the 'header' struct and the 'header_bytes' */
  makeHeader(header, header_bytes, flow_info, creditInfo->nextLink.linkId, timing_offset);


  /* final message (type: 1-coded) */
  /* DO NOT include the DC tones */
  //unsigned int n_actual_symbols = (d_num_ofdm_symbols+1) * d_occupied_carriers;		// extra for NULL symbol at the end
  unsigned int n_actual_symbols = (d_num_ofdm_symbols) * d_occupied_carriers;
  gr_message_sptr out_msg = gr_make_message(DATA_TYPE, 0, 0, HEADERBYTELEN + (n_actual_symbols * sizeof(gr_complex)));
  memcpy(out_msg->msg(), header_bytes, HEADERBYTELEN);					            // copy header bytes

  int offset = HEADERBYTELEN;

  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++) {
     unsigned int index = i * d_occupied_carriers;
     for(unsigned int j = 0; j < d_occupied_carriers; j++) {
#if 0
	 if(d_all_carriers[j] == 1) {
	    std::cout<<"pilot: "<<j<<" -- "<< out_symbols[index+j]<<endl;
	 }
#endif
	 memcpy((out_msg->msg() + offset), (out_symbols + index + j), sizeof(gr_complex));
	 offset += sizeof(gr_complex);
     }
  }
  
  // set the tx timestamp, if its a sync send //
  if(sync_send) {
     out_msg->set_timestamp(sync_secs, sync_frac_of_secs);
     uhd::time_spec_t c_time = d_usrp->get_time_now();

     const pmt::pmt_t &rx_value = d_sync_tag.value;
     uint64_t rx_secs = pmt::pmt_to_uint64(pmt_tuple_ref(rx_value, 0));
     double rx_frac_secs = pmt::pmt_to_double(pmt_tuple_ref(rx_value,1));
     
     uhd::time_spec_t proc_time = c_time - uhd::time_spec_t(rx_secs, rx_frac_secs);
     uhd::time_spec_t rx_tx_time = uhd::time_spec_t(sync_secs, sync_frac_of_secs) - 
						uhd::time_spec_t(rx_secs, rx_frac_secs);
      	
     printf("FWD PACKET---- curr (%llu, %f) out(%llu, %f) proc(%llu, %f) rx-tx(%llu, %f)\n", 
		(uint64_t) c_time.get_full_secs(), c_time.get_frac_secs(), sync_secs, sync_frac_of_secs,
		(uint64_t) proc_time.get_full_secs(), proc_time.get_frac_secs(), 
		(uint64_t) rx_tx_time.get_full_secs(), rx_tx_time.get_frac_secs()); 
     fflush(stdout);
  }

  if(d_h_coding) {
     out_msg->set_timing_offset(timing_offset);
  }

  d_out_queue->insert_tail(out_msg);
  sleep(0.2);
  printf("count: %d\n", d_out_queue->count());
  out_msg.reset();
 
  printf("SINK got the packet!\n"); fflush(stdout);

  free(coeffs);
  free(out_symbols);
} //encodePktToFwd

inline bool
digital_ofdm_frame_sink::openLogTxFwdSymbols() {
  const char *filename1 = "tx_symbols_fwd.dat";
  int fd;
  if ((fd = open (filename1, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename1);
     return false;
  }
  else {
      if((d_fp_symbols_test = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "coeff file cannot be opened\n");
            close(fd);
            return false;
      }
  }
  return true;
}

void
digital_ofdm_frame_sink::logGeneratedTxSymbols(gr_complex *out)
{
  gr_complex *log_symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_data_carriers.size());
 
  int count = 0;
  for(int i = 0; i < d_num_ofdm_symbols; i++) {
     int index = 0;
     memset(log_symbols, 0, sizeof(gr_complex) * d_data_carriers.size());
     for(int j = 0; j < d_all_carriers.size(); j++) { 
        if(d_all_carriers[j] == 0) {
           memcpy(log_symbols+index, out+d_data_carriers[index]+i*d_occupied_carriers, sizeof(gr_complex));
           index++;
	}
     }
     //printf("index: %d, d_data_carriers: %d\n", index, d_data_carriers.size()); fflush(stdout);
     assert(index == d_data_carriers.size());
     count += fwrite_unlocked(log_symbols, sizeof(gr_complex), d_data_carriers.size(), d_fp_symbols_test); 
  }

  //printf("count: %d written to tx_symbols.dat, total: %d \n", count, ftell(d_fp_symbols_test)); fflush(stdout);
  free(log_symbols);
}

float
digital_ofdm_frame_sink::getNormalizationFactor(gr_complex *symbols, int nsenders) {
  int n_data_carriers = d_data_carriers.size();
 
  float avg_amp = 0.0;
  float avg_mag = 0.0;
  for(unsigned int i = 3; i < d_num_ofdm_symbols; i++) {		// ignore the top 2 OFDM symbols
     for(unsigned int j = 0; j < n_data_carriers; j++) {
         unsigned int index = (i*d_occupied_carriers) + d_data_carriers[j];
	 avg_amp += abs(symbols[index]); 
	 avg_mag += norm(symbols[index]);
     }
  }
  
  int denom = (d_num_ofdm_symbols-2) * n_data_carriers;
  
  //avg_amp = avg_amp/((float) n_data_carriers);
  avg_amp = avg_amp/((float) denom);
  avg_mag /= ((float) denom);

  float factor = sqrt(avg_mag) * sqrt(nsenders);
  printf("getNormalizationFactor, %f = (%f*%f)\n", factor, sqrt(avg_mag), sqrt(nsenders)); fflush(stdout);
  //return avg_amp/0.7;
  return factor;
}

/* just to test */
#if 0
inline void
digital_ofdm_frame_sink::logSymbols(gr_complex *symbols, int num) {
  
  int count = ftell(d_fp_symbols_test);
  count = fwrite_unlocked(&symbols[0], sizeof(gr_complex), num, d_fp_symbols_test);  
}

inline bool
digital_ofdm_frame_sink::openLogSymbols() {
  const char *filename1 = "tx_symbols_fwd.dat";
  int fd;
  if ((fd = open (filename1, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename1);
     return false;
  }
  else {
      if((d_fp_symbols_test = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "coeff file cannot be opened\n");
            close(fd);
            return false;
      }
  }
  return true;
}
#endif

inline void
digital_ofdm_frame_sink::decodeSignal(gr_complex *symbols, gr_complex coeff)
{
  //printf("decodeSignal, coeff: [%f]\n", ToPhase_f(coeff)); fflush(stdout);
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++)
     for(unsigned int j = 0; j < d_occupied_carriers; j++) {
         unsigned int index = (i*d_occupied_carriers) + j;
         symbols[index] /= coeff;
     }
  //printf("decodeSignal done!\n"); fflush(stdout);
}

/* degrees (float) -> phase (complex) */
inline gr_complex
digital_ofdm_frame_sink::ToPhase_c(float phase_deg)
{
  return gr_expj(phase_deg * (M_PI/180));
}

/* phase (complex) -> degrees (float) */
inline float
digital_ofdm_frame_sink::ToPhase_f(gr_complex phase_c) {
  float phase = arg(phase_c) * 180/M_PI;
  if(phase < 0) phase += 360.0;
 
  return phase;
}

/* COEFF -> degrees (float) */
inline float
digital_ofdm_frame_sink::ToPhase_f(COEFF coeff) {
  float phase = ((float) coeff.phase)/SCALE_FACTOR_PHASE;
  return phase;
}

/* COEFF -> phase (complex) */
inline gr_complex
digital_ofdm_frame_sink::ToPhase_c(COEFF coeff) {
  float phase = ((float) coeff.phase)/SCALE_FACTOR_PHASE;
  float amp = ((float) coeff.amplitude)/SCALE_FACTOR_AMP;

  //printf("z=%f\n", amp); fflush(stdout);
  float angle_rad = phase * M_PI/180;
  return amp * gr_expj(angle_rad);
}


inline CreditInfo*
digital_ofdm_frame_sink::findCreditInfo(unsigned char flowId) {
  CreditInfo *creditInfo = NULL;
  if(d_creditInfoVector.size() == 0)
	return creditInfo;

  CreditInfoVector::iterator it = d_creditInfoVector.begin();
  
  while(it != d_creditInfoVector.end()) {
        creditInfo = *it;
	if(creditInfo->flowId == flowId)
	    return creditInfo;	
        it++;
  } 

  return NULL;
}

/* sends the ACK on the backend ethernet */
void 
digital_ofdm_frame_sink::send_ack(unsigned char flow, unsigned char batch) {
  MULTIHOP_ACK_HDR_TYPE ack_header;

  memset(&ack_header, 0, sizeof(ack_header));
 
  FlowInfo *flowInfo = getFlowInfo(false, flow);
  assert(flowInfo);

  ack_header.src_id = flowInfo->src;                                    // flow-src
  ack_header.dst_id = flowInfo->dst;                                    // flow-dst

  ack_header.prev_hop_id = d_id;
  ack_header.batch_number = flowInfo->active_batch;                     // active-batch got ACKed
  ack_header.pkt_type = ACK_TYPE;
  ack_header.flow_id = flow;

  for(int i = 0; i < PADDING_SIZE; i++)
       ack_header.pad[i] = 0;

  char ack_buf[ACK_HEADERDATALEN];
  memcpy(ack_buf, &ack_header, ACK_HEADERDATALEN);

  // send on the backend //
  if (send(d_ack_sock, (char *)&ack_buf[0], sizeof(ack_buf), 0) < 0) {
    printf("Error: failed to send ACK\n");
    //exit(1);
  } else
    printf("@@@@@@@@@@@@@@@@ sent ACK (%d bytes) for batch %d @@@@@@@@@@@@@@@@@@@@\n", ACK_HEADERDATALEN, flowInfo->active_batch); fflush(stdout);
}

#ifndef SEND_ACK_ETHERNET
/* receiver does this - creates an ACK which contains: 
   flowId, batch_to_be_ACKed, flow_src_id, flow_dst_id (my_id). 
   Insert the ACK in the out_queue, which would trigger the callback
   into the wrapper. 
   For now: just add a header which contains the above details
*/
void
digital_ofdm_frame_sink::sendACK(unsigned char flow, unsigned char batch) {
  //printf("sendACK, flow: %d, batch: %d, ACK_TYPE: %d\n", flow, batch, ACK_TYPE); fflush(stdout);
  /* make header */
  MULTIHOP_ACK_HDR_TYPE ack_header;

  memset(&ack_header, 0, sizeof(ack_header));

  FlowInfo *flowInfo = getFlowInfo(false, flow);
  assert(flowInfo);

  ack_header.src_id = flowInfo->src;					// flow-src
  ack_header.dst_id = flowInfo->dst;					// flow-dst

  ack_header.prev_hop_id = d_id;				
  ack_header.batch_number = flowInfo->active_batch;			// active-batch got ACKed
  ack_header.pkt_type = ACK_TYPE;
  ack_header.flow_id = flow;

  for(int i = 0; i < PADDING_SIZE; i++)
       ack_header.pad[i] = 0;

  /* copy e'thing except CRC */
  unsigned char ack_header_bytes[ACK_HEADERBYTELEN];
  memcpy(ack_header_bytes, &ack_header, ACK_HEADERDATALEN);

  /* generate CRC */
  unsigned int calc_crc = digital_crc32(ack_header_bytes, ACK_HEADERDATALEN);
  ack_header.hdr_crc = calc_crc;
 
  /* copy the crc and padding */
  memcpy(ack_header_bytes+ACK_HEADERDATALEN, &calc_crc, sizeof(int));
  memcpy(ack_header_bytes+ACK_HEADERDATALEN+sizeof(int), ack_header.pad, ACK_PADDING_SIZE); 
  whiten(ack_header_bytes, ACK_HEADERBYTELEN);

  /* final message (type: 1-coded) */
  gr_message_sptr out_msg = gr_make_message(ACK_TYPE, 0, 0, ACK_HEADERBYTELEN);		// 3: ACK TYPE
  memcpy(out_msg->msg(), ack_header_bytes, ACK_HEADERBYTELEN);				// copy header

  /* insert into the out-queue */
  d_ack_queue.push_back(out_msg);
  //out_msg.reset();

  /* update the flowInfo */
  flowInfo->last_batch_acked = flowInfo->active_batch;
  flowInfo->active_batch += 1;
}
#endif // ACK on ETHERNET

void
digital_ofdm_frame_sink::whiten(unsigned char *bytes, const int len)
{
   for(int i = 0; i < len; i++)
        bytes[i] = bytes[i] ^ random_mask_tuple[i];
}


// should only be called from slicer ILP //
void
digital_ofdm_frame_sink::getSymOutBits_ILP(unsigned char *bits, int index) {
  if(d_batch_size == 1) {
     assert(index < 2);
     switch(index) {
        case 0: bits[0] = 0; break;
	case 1: bits[0] = 1; break;
     }
     return;
  }  
  else if(d_batch_size == 2) { 
     assert(index < 4);
     switch(index) {
	case 0: bits[0] = 0; bits[1] = 0; break;
	case 1: bits[0] = 0; bits[1] = 1; break;
	case 2: bits[0] = 1; bits[1] = 0; break;
	case 3: bits[0] = 1; bits[1] = 1; break;
     }
     return;
  } 
  else if(d_batch_size == 3) {
     assert(index < 8);
     switch(index) {
        case 0: bits[0] = 0; bits[1] = 0; bits[2] = 0; break;
        case 1: bits[0] = 0; bits[1] = 0; bits[2] = 1; break;
        case 2: bits[0] = 0; bits[1] = 1; bits[2] = 0; break;
        case 3: bits[0] = 0; bits[1] = 1; bits[2] = 1; break;
	case 4: bits[0] = 1; bits[1] = 0; bits[2] = 0; break;
	case 5: bits[0] = 1; bits[1] = 0; bits[2] = 1; break;
	case 6: bits[0] = 1; bits[1] = 1; bits[2] = 0; break;
	case 7: bits[0] = 1; bits[1] = 1; bits[2] = 1; break;
     }
     return;
  }
  assert(false);
}

void
digital_ofdm_frame_sink::getSymOutBits_ILP_QPSK(unsigned char *bits, int index) {
  if(d_batch_size == 1) {
     assert(index < 4);
     switch(index) {
        case 0: bits[0] = 0; break;
        case 1: bits[0] = 1; break;
        case 2: bits[0] = 2; break;
        case 3: bits[0] = 3; break;
     }
     return;
  } 
  else if(d_batch_size == 2) { 
     assert(index < 16);
     switch(index) {
        case 0: bits[0] = 0; bits[1] = 0; break;
        case 1: bits[0] = 0; bits[1] = 1; break;
        case 2: bits[0] = 0; bits[1] = 2; break;
        case 3: bits[0] = 0; bits[1] = 3; break;
        case 4: bits[0] = 1; bits[1] = 0; break;
        case 5: bits[0] = 1; bits[1] = 1; break;
        case 6: bits[0] = 1; bits[1] = 2; break;
        case 7: bits[0] = 1; bits[1] = 3; break;
        case 8: bits[0] = 2; bits[1] = 0; break;
        case 9: bits[0] = 2; bits[1] = 1; break;
        case 10: bits[0] = 2; bits[1] = 2; break;
        case 11: bits[0] = 2; bits[1] = 3; break;
        case 12: bits[0] = 3; bits[1] = 0; break;
        case 13: bits[0] = 3; bits[1] = 1; break;
        case 14: bits[0] = 3; bits[1] = 2; break;
        case 15: bits[0] = 3; bits[1] = 3; break;
     }
     return;
  } 
  assert(false);
}

/* plain qam-16 demod */
void
digital_ofdm_frame_sink::getSymOutBits_ILP_QAM16(unsigned char *bits, int index) {
  if(d_batch_size == 1) {
     assert(index < 16);
     switch(index) {
        case 0: bits[0] = 0; break;
        case 1: bits[0] = 1; break;
        case 2: bits[0] = 2; break;
        case 3: bits[0] = 3; break;
        case 4: bits[0] = 4; break;
        case 5: bits[0] = 5; break;
        case 6: bits[0] = 6; break;
        case 7: bits[0] = 7; break;
        case 8: bits[0] = 8; break;
        case 9: bits[0] = 9; break;
        case 10: bits[0] = 10;break;
        case 11: bits[0] = 11; break;
        case 12: bits[0] = 12; break;
        case 13: bits[0] = 13; break;
        case 14: bits[0] = 14; break;
        case 15: bits[0] = 15; break;
     }
     return;
  }
  else if(d_batch_size == 2) {
     assert(false);
  }
  assert(false);
}

/* alternative decoding strategy, where you try and decode after every packet received. In case we are lucky, we can decode
   after receiving just 1 packet. Else keep trying to decode with every successive packet received. Gradually, the strategy should 
   converge to the right decoding.

   Need to save the total error (hamming distance) seen for each modulation symbol. For each packet received thereafter, compute the 
   updated error and try and make a decision. 
 */
void
digital_ofdm_frame_sink::demodulate_ILP_2(FlowInfo *flowInfo)
{
  //printf("demodulate_ILP_2, pkt_no: %d, senders: %d\n", d_pkt_num, d_pktInfo->n_senders); fflush(stdout);
  // initialize 'bytes decoded' for every batch //
  vector<unsigned char*> bytes_out_vec;
  for(unsigned int k = 0; k < d_batch_size; k++) {
     unsigned char *bytes = (unsigned char*) malloc(sizeof(unsigned char) * d_occupied_carriers);
     bytes_out_vec.push_back(bytes);
  }

#ifdef USE_PILOT
  vector<gr_complex> dfe_pilot[MAX_SENDERS];
  for(unsigned int i = 0; i < MAX_SENDERS; i++) {
     dfe_pilot[i].resize(d_pilot_carriers.size());
     fill(dfe_pilot[i].begin(), dfe_pilot[i].end(), gr_complex(1.0,0.0));
  }

  vector<gr_complex*> interpolated_coeffs;
  int num_senders = d_pktInfo->n_senders;
#ifdef LSQ_COMPRESSION
  // coefficients are already complete for each sender on all subcarriers at d_pktInfo->coeffs //
  for(int k = 0; k < num_senders; k++) {
      gr_complex *rx_coeffs = d_pktInfo->coeffs.at(k);
      interpolated_coeffs.push_back(rx_coeffs);
  }
#else
  /* get the interpolated coefficients  */
  for(int k = 0; k < num_senders; k++) {
      gr_complex *rx_coeffs = d_pktInfo->coeffs.at(k);
      gr_complex *out_coeffs = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * d_occupied_carriers);
      memset(out_coeffs, 0, sizeof(gr_complex) *  d_batch_size * d_occupied_carriers);

      interpolate_coeffs_lerp(rx_coeffs, out_coeffs);
      interpolated_coeffs.push_back(out_coeffs);
  }
#endif  
#else	// USE_PILOT
  assert(false);
#endif
  // run the demapper for every OFDM symbol (all batches within the symbol are demapped at once!) //
  int packetlen_cnt[MAX_BATCH_SIZE];
  memset(packetlen_cnt, 0, sizeof(int) * MAX_BATCH_SIZE);

  unsigned char packet[MAX_BATCH_SIZE][MAX_PKT_LEN];

  /* check what batches to propagate to user app */
  bool batch_correct = true, batch_decoded = false;
  gr_message_sptr msg[MAX_BATCH_SIZE];
  std::string decoded_msg[MAX_BATCH_SIZE];

#ifdef SRC_PILOT
  if(num_senders > 1) {
    adjust_H_estimate(0);
  }
  reset_demapper();
#endif

#ifdef MIMO_RECEIVER
  // first fill up the d_euclid_dist //
  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
      unsigned int bytes_decoded = demapper_ILP_2_pilot_SRC(o, bytes_out_vec, flowInfo, dfe_pilot[0], interpolated_coeffs);   // same # of bytes decoded/batch
  } 
  waitForDecisionFromMimo(packet);

  int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size));
  memset(d_euclid_dist, 0, sizeof(float) * d_data_carriers.size() * n_entries * MAX_OFDM_SYMBOLS);
#if 0
  batch_decoded = true;
  for(unsigned int k = 0; k < d_batch_size; k++) {
      msg[k] = gr_make_message(0, d_packet_whitener_offset, 0, d_packetlen);
      memcpy(msg[k]->msg(), packet[k], d_packetlen);
      bool crc_valid = crc_check(msg[k]->to_string(), decoded_msg[k]);
      printf("crc valid: %d\n", crc_valid); fflush(stdout);
      if(crc_valid == 0) {
	 batch_correct = false;
      }
  }
#endif
#else // MIMO_RECEIVER
  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
#ifdef USE_PILOT
#ifdef SRC_PILOT
      unsigned int bytes_decoded = demapper_ILP_2_pilot_SRC(o, bytes_out_vec, flowInfo, dfe_pilot[0], interpolated_coeffs);   // same # of bytes decoded/batch
#else
      unsigned int bytes_decoded = demapper_ILP_2_pilot(o, bytes_out_vec, flowInfo, dfe_pilot, interpolated_coeffs);   // same # of bytes decoded/batch
#endif
#else	// no pilots //
      assert(false);
#endif

      unsigned int jj = 0;
      while(jj < bytes_decoded) {
         for(unsigned int k = 0; k < d_batch_size; k++) {
              packet[k][packetlen_cnt[k]++] = bytes_out_vec[k][jj];
              if ((packetlen_cnt[k]) == d_packetlen) {
		  batch_decoded = true;							// the batch is decoded, might not be correct though //
                  msg[k] = gr_make_message(0, d_packet_whitener_offset, 0, (packetlen_cnt[k]));
                  memcpy(msg[k]->msg(), packet[k], (packetlen_cnt[k]));

                  // With a good header, let's now check for the preamble sync timestamp
		  //set_msg_timestamp(msg);

		  bool crc_valid = crc_check(msg[k]->to_string(), decoded_msg[k]);
		  if(d_crc[k] == 0) {
		      d_crc[k] = ((crc_valid == true) ? 1:0);
		      if(crc_valid) d_num_pkts_correct++;
		  }
		  //printf("crc valid: %d, d_crc: %d\n", crc_valid, d_crc[k]); fflush(stdout);
		  if(d_crc[k] == 0) {
		     batch_correct = false;						// mark the batch as incorrect! //
		  }
#if 0
                  d_target_queue->insert_tail(msg[k]);
                  msg[k].reset();
#endif
              }
         }//for
         jj++;
      } //while
  }

  //calculateSER(packet, d_packetlen);

  d_avg_evm_error /= ((double) d_num_ofdm_symbols);
  d_total_pkts_received++;

  if(batch_correct) {
     d_last_batch_acked = d_active_batch;
     d_correct_batches++;
  }

  int num_inno_pkts = flowInfo->innovative_pkts.size();
  printf("***** Active Batch: %d Batch OK: %d, Inno Pkts: %d, evm: %f, #Batches (total: %d right: %d) #Pkts (total: %d, right: %d) ******** \n", d_active_batch, batch_correct, num_inno_pkts, d_avg_evm_error, d_total_batches_received, d_correct_batches, d_total_pkts_received, d_num_pkts_correct); fflush(stdout);

#ifdef SEND_ACK_ETHERNET
  if(batch_correct || num_inno_pkts == d_batch_size + 1) {
     d_last_batch_acked = d_active_batch;
     send_ack(d_flow, d_active_batch);
  }
#endif

  d_avg_evm_error = 0.0;

  // escalate the decoded msgs up //
  assert(batch_decoded);
  for(unsigned int k = 0; k < d_batch_size; k++) {
      printf("decoded batch: %d, pkt: %d\t", d_active_batch, k); fflush(stdout);
      print_msg(decoded_msg[k]);
      printf("\n"); fflush(stdout);
      //if(batch_correct) d_target_queue->insert_tail(msg[k]);
      msg[k].reset();
  } 
#endif  // MIMO_RECEIVER

#ifdef USE_PILOT
#ifndef LSQ_COMPRESSION
  for(int i = 0; i < num_senders; i++) {
      gr_complex *out_coeffs = interpolated_coeffs.at(i);
      free(out_coeffs);
  }
  interpolated_coeffs.clear();
#endif
#else
  /* cleaning */
  for(unsigned int i = 0; i < d_batch_size; i++) {
     free(bytes_out_vec[i]);
  }
  bytes_out_vec.clear();

  /* batched_sym_position */
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
      gr_complex *sym_position =  batched_sym_position[i];
      free(sym_position);
  }
#endif
  
  //printf("demodulate_ILP_2 end, pkt_no: %d\n\n", d_pkt_num); fflush(stdout);
}

inline void
digital_ofdm_frame_sink::set_msg_timestamp(gr_message_sptr msg) {
  unsigned int tag_port = 1;
  std::vector<gr_tag_t> rx_sync_tags;
  const uint64_t nread = this->nitems_read(tag_port);
  this->get_tags_in_range(rx_sync_tags, tag_port, nread, nread+last_noutput_items, SYNC_TIME);
  if(rx_sync_tags.size()>0) {
     size_t t = rx_sync_tags.size()-1;
     printf("set_timestamp (SINK):: found %d tags, offset: %ld, nread: %ld, output_items: %ld\n", rx_sync_tags.size(), rx_sync_tags[t].offset, nread, last_noutput_items); fflush(stdout);
     const pmt::pmt_t &value = rx_sync_tags[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));
     msg->set_timestamp(sync_secs, sync_frac_of_secs);
  } else {
     std::cerr << "SINK ---- Header received, with no sync timestamp?\n";
  }

  std::vector<gr_tag_t> rx_sync_tags2;
  this->get_tags_in_range(rx_sync_tags2, tag_port, 0, nread, SYNC_TIME);
  if(rx_sync_tags2.size()>0) {
     size_t t = rx_sync_tags2.size()-1;
     printf("test_timestamp2 (SINK):: found %d tags, offset: %ld, nread: %ld\n", rx_sync_tags2.size(), rx_sync_tags2[t].offset, nread); fflush(stdout);
     const pmt::pmt_t &value = rx_sync_tags2[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));
  } else {
     std::cerr << "SINK ---- Header received, with no sync timestamp2?\n";
  }
}

#ifdef USE_PILOT
inline void
digital_ofdm_frame_sink::buildMap_pilot(FlowInfo *flowInfo, gr_complex* sym_position, 
					vector<gr_complex*> interpolated_coeffs,
					vector<gr_complex>* dfe, gr_complex* carrier, 
					int subcarrier_index, int o) {

  int num_senders = d_pktInfo->n_senders;
  int subcarrier = d_data_carriers[subcarrier_index];
  int n_data_carriers = d_data_carriers.size();

  gr_complex coeffs[5];
  for(unsigned int j = 0; j < d_batch_size; j++)
  {
#ifdef LSQ_COMPRESSION
      int coeff_index = j * n_data_carriers + subcarrier_index;			// for LSQ, arrangement of coeffs is different: for all batches(sub1, sub2 .. subN)
#else
      int coeff_index = subcarrier * d_batch_size + j;				// non LSQ, arrangment is: for all_subcarriers(batch1, batch2 .. batchN)
#endif
      for(int k = 0; k < num_senders; k++)
      {
          gr_complex *estimates = d_pktInfo->hestimates[k];                    // estimates for 'kth' sender
          gr_complex *in_coeffs = interpolated_coeffs[k];                      // interpolated coeffs for 'kth' sender

	  gr_complex coeff = (gr_complex(1.0, 0.0)/(estimates[subcarrier] * carrier[k] * dfe[k][subcarrier_index])) * in_coeffs[coeff_index];
	  if(k == 0) coeffs[j] = coeff;
          else coeffs[j] += coeff;
      }

#if 0
      // debug //
      if(o == 0) {
         float amp = abs(coeffs[j]);
         float ph = arg(coeffs[j]) * 180/M_PI;
         if(ph < 0) ph += 360.0;
         printf("batch: %d, amp: %.2f, ph: %.2f\t", j, amp, ph);
      }
#endif	
  }
  if(o == 0) printf("\n"); fflush(stdout);

  if(d_batch_size == 1) {
         int j = 0;
         sym_position[j++] = ((coeffs[0] *  d_data_sym_position[0]));
         sym_position[j++] = ((coeffs[0] *  d_data_sym_position[1]));;
  }
  if(d_batch_size == 2) {
         int j = 0;
         sym_position[j++] = (((coeffs[0] *  d_data_sym_position[0])) + (coeffs[1] *  d_data_sym_position[0]));
         sym_position[j++] = (((coeffs[0] *  d_data_sym_position[0])) + (coeffs[1] *  d_data_sym_position[1]));
         sym_position[j++] = (((coeffs[0] *  d_data_sym_position[1])) + (coeffs[1] *  d_data_sym_position[0]));
         sym_position[j++] = (((coeffs[0] *  d_data_sym_position[1])) + (coeffs[1] *  d_data_sym_position[1]));
  }
  else if (d_batch_size == 3) {
         int j = 0;
         sym_position[j++] = coeffs[0]* d_data_sym_position[0] + coeffs[1]* d_data_sym_position[0] + coeffs[2]* d_data_sym_position[0];
         sym_position[j++] = coeffs[0]* d_data_sym_position[0] + coeffs[1]* d_data_sym_position[0] + coeffs[2]* d_data_sym_position[1];
         sym_position[j++] = coeffs[0]* d_data_sym_position[0] + coeffs[1]* d_data_sym_position[1] + coeffs[2]* d_data_sym_position[0];
         sym_position[j++] = coeffs[0]* d_data_sym_position[0] + coeffs[1]* d_data_sym_position[1] + coeffs[2]* d_data_sym_position[1];
         sym_position[j++] = coeffs[0]* d_data_sym_position[1] + coeffs[1]* d_data_sym_position[0] + coeffs[2]* d_data_sym_position[0];
         sym_position[j++] = coeffs[0]* d_data_sym_position[1] + coeffs[1]* d_data_sym_position[0] + coeffs[2]* d_data_sym_position[1];
         sym_position[j++] = coeffs[0]* d_data_sym_position[1] + coeffs[1]* d_data_sym_position[1] + coeffs[2]* d_data_sym_position[0];
         sym_position[j++] = coeffs[0]* d_data_sym_position[1] + coeffs[1]* d_data_sym_position[1] + coeffs[2]* d_data_sym_position[1];
  }
}

// normal version used when (1) pairwise pilot correction is done, and (2) each pilot symbol corresponds to a particular sender //
void digital_ofdm_frame_sink::track_pilot_dfe(gr_complex *in, int sender,
					  gr_complex& carrier, vector<gr_complex>& dfe_pilot) {
  gr_complex phase_error = 0.0;
  float cur_pilot = 1.0;
  gr_complex *estimates = d_pktInfo->hestimates[sender]; 			// FIXME!! - different pilots for different senders, eh?

  for (unsigned int i = 0; i < d_pilot_carriers.size(); i++) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    in[di] *= estimates[di];							// equalize the pilot! 

    // some debugging //
#if 1
    float angle = arg(in[di]) * 180/M_PI;                       // rotation after equalization //
    if(angle < 0) angle += 360.0;
    float perfect_angle = arg(pilot_sym) * 180/M_PI; 	
    float delta_rotation = perfect_angle - angle;
    printf("pilot %d, A=%.3f, P=%.3f, delta_P=%.3f\n", di, abs(in[di]), angle, delta_rotation); fflush(stdout);
#endif

    phase_error += (in[di] * conj(pilot_sym));
  }

  // update phase equalizer
  float angle = arg(phase_error);
  d_freq[sender] = d_freq[sender] - d_freq_gain*angle;
  d_phase[sender] = d_phase[sender] + d_freq[sender] - d_phase_gain*angle;
  if (d_phase[sender] >= 2*M_PI) d_phase[sender] -= 2*M_PI;
  else if (d_phase[sender] <0) d_phase[sender] += 2*M_PI;

  carrier = gr_expj(-angle);

  // update DFE based on pilots
  cur_pilot = 1.0;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); i++) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    gr_complex sigeq = in[di] * carrier * dfe_pilot[i];
    // FIX THE FOLLOWING STATEMENT
    if (norm(sigeq)> 0.001)
      dfe_pilot[i] += d_eq_gain * (pilot_sym/sigeq - dfe_pilot[i]);
  }
  d_end_angle[sender] = angle;
}

inline void
digital_ofdm_frame_sink::interpolate_data_dfe(vector<gr_complex> dfe_pilot, vector<gr_complex>& dfe_data, bool correct, gr_complex *in, gr_complex carrier) {
  // equalize all data using interpolated dfe and demap into bytes
  unsigned int pilot_index = 0;
  int pilot_carrier_lo = 0;
  int pilot_carrier_hi = d_pilot_carriers[0];
  gr_complex pilot_dfe_lo = dfe_pilot[0];
  gr_complex pilot_dfe_hi = dfe_pilot[0];

   /* alternate implementation of lerp */
   float denom = 1.0/(pilot_carrier_hi - pilot_carrier_lo);                                     // 1.0/(x_b - x_a)

   for (unsigned int i = 0; i < d_data_carriers.size(); i++) {
      int di = d_data_carriers[i];
      if(di > pilot_carrier_hi) {                                       // 'di' is beyond the current pilot_hi
          pilot_index++;                                                // move to the next pilot

          pilot_carrier_lo = pilot_carrier_hi;                          // the new pilot_lo
          pilot_dfe_lo = pilot_dfe_hi;                                  // the new pilot_dfe_lo

         if (pilot_index < d_pilot_carriers.size()) {
             pilot_carrier_hi = d_pilot_carriers[pilot_index];
             pilot_dfe_hi = dfe_pilot[pilot_index];
         }
         else {
             pilot_carrier_hi = d_occupied_carriers;                    // cater to the di's which are beyond the last pilot_carrier
         }
         denom = 1.0/(pilot_carrier_hi - pilot_carrier_lo);
      }

      float alpha = float(di - pilot_carrier_lo) * denom;               // (x - x_a)/(x_b - x_a)

      gr_complex dfe = pilot_dfe_lo + alpha * (pilot_dfe_hi - pilot_dfe_lo);    // y = y_a + ( x - x_a)/(x_b - x_a) * (y_b - y_a)

      if(correct) {
	 assert(in != NULL);
         in[di] = in[di] * carrier * dfe;
      }
      dfe_data.push_back(dfe);
  }
}

#if 0
/* called for each OFDM symbol */
unsigned int
digital_ofdm_frame_sink::demapper_ILP_2_pilot(unsigned int ofdm_symbol_index, vector<unsigned char*> out_vec,
                                              FlowInfo *flowInfo, vector<gr_complex>* dfe_pilot, 
					      vector<gr_complex*> interpolated_coeffs) {
  //printf("digital_ofdm_frame_sink::demapper_ILP_2_pilot, senders: %d\n", d_nsenders); fflush(stdout);

  unsigned int bytes_produced = 0;
  assert(d_batch_size <= MAX_BATCH_SIZE);

  InnovativePktInfoVector innovativePkts = flowInfo->innovative_pkts;
  int pkt_index = innovativePkts.size() - 1;

  // get the current packet //
  PktInfo *pInfo = innovativePkts[pkt_index];
  gr_complex *rx_symbols_this_batch = pInfo->symbols;

  gr_complex *sym_vec = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
  memcpy(sym_vec, &rx_symbols_this_batch[ofdm_symbol_index * d_occupied_carriers], sizeof(gr_complex) * d_occupied_carriers);

  gr_complex carrier[MAX_SENDERS];
  vector<gr_complex> dfe_data[MAX_SENDERS];

  int n_data_carriers = d_data_carriers.size();
  for(unsigned int s = 0; s < d_nsenders; s++) {

     /* using slope of angle1, estimate the angle after the *gap*, and deduce d_phase and d_freq */
     if(ofdm_symbol_index == 0 && NULL_OFDM_SYMBOLS != 0 && s==0 && d_nsenders > 1) {	// only for lead forwarder, and multiple senders //
        int gap = NULL_OFDM_SYMBOLS;

        float est_angle = d_slope_angle[s] * gap + d_end_angle[s];
        float est_freq = d_freq[s] - d_freq_gain * est_angle;
        float est_phase = d_phase[s] + est_freq - d_phase_gain * est_angle;


        if(est_phase >= 2*M_PI) est_phase -= 2*M_PI;
        if(est_phase < 0) est_phase += 2*M_PI;

        d_phase[s] = est_phase; d_freq[s] = est_freq;                   // get the d_phase1 and d_freq1 values after the gap, to compare with the actual d_phase and d_freq values

        //printf("sender: %d, estimated angle: %f, d_phase: %f, d_freq: %f\n", s, est_angle, est_phase, est_freq); fflush(stdout);
     }

     /* if multiple senders: then odd pilots belong to the lead forwarder, and even pilots to the slave forwarder */
     if(d_nsenders == 1 || (s==0 && (ofdm_symbol_index+1)%2==1) || (s==1 && (ofdm_symbol_index+1)%2==0)) {
	//printf("track_pilot_dfe, sender: %d, ofdm_symbol_index: %d\n", s, (ofdm_symbol_index+1)); fflush(stdout);
     	track_pilot_dfe(sym_vec, s, carrier[s], dfe_pilot[s]);
     }
     else {
	// calculate the updated angle based on the slope (gap of 2 symbols), and consequently derive d_phase and d_freq for the sender //
	int gap = 2;
	float est_angle = d_slope_angle[s] * gap + d_end_angle[s];
	float est_freq = d_freq[s] - d_freq_gain * est_angle;
	float est_phase = d_phase[s] + est_freq - d_phase_gain * est_angle;
	if(est_phase >= 2*M_PI) est_phase -= 2*M_PI;
	if(est_phase < 0) est_phase += 2*M_PI;
	d_phase[s] = est_phase; d_freq[s] = est_freq;
	d_end_angle[s] = est_angle;
	carrier[s] = gr_expj(-est_angle);
     }	

     //printf("d_end_angle: %.3f\n", d_end_angle[s]); fflush(stdout);
     //printf("sender: %d, d_end_angle: %f, d_phase: %f, d_freq: %f\n", s, d_end_angle[s], d_phase[s], d_freq[s]); fflush(stdout);

     interpolate_data_dfe(dfe_pilot[s], dfe_data[s], false, NULL, gr_complex(0.0, 0.0));
     assert(dfe_data[s].size() == n_data_carriers); 
  }

  int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));
  gr_complex sigrot, closest_sym;

  /* for every packet of the batch, 'd_batch_size' # of partial bytes and bits will be decoded */
  unsigned int partial_byte[MAX_BATCH_SIZE];
  unsigned char bits[MAX_BATCH_SIZE];
  memset(partial_byte, 0, sizeof(unsigned int) * MAX_BATCH_SIZE);
  memset(bits, 0, sizeof(unsigned char) * MAX_BATCH_SIZE);

  unsigned int byte_offset = 0;

  // symbol position map needs to be built each time, because of DFE tracking //
  gr_complex *sym_position = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);

  for(unsigned int i = 0; i < n_data_carriers; i++) {

     // demodulate 1-byte at a time //
     if (byte_offset < 8) {

        // build the map for each subcarrier //
	 buildMap_pilot(flowInfo, sym_position, interpolated_coeffs, dfe_data, carrier, i, ofdm_symbol_index);

	 // slice it //
	 sigrot = sym_vec[d_data_carriers[i]];
	 slicer_ILP_2(sigrot, flowInfo, closest_sym, bits, sym_position, ofdm_symbol_index, i);

         assert((8 - byte_offset) >= d_data_nbits);

         for(unsigned int k = 0; k < d_batch_size; k++)
              partial_byte[k] |= bits[k] << (byte_offset);

         byte_offset += d_data_nbits;
     }

     if(byte_offset == 8) {
        for(int k = 0; k < d_batch_size; k++) {
	    if(d_pkt_num == 2 && 0) printf("k: %d, demod_byte: %x (%d) \n", k, partial_byte[k], partial_byte[k]); fflush(stdout);
	    //assert(false);
            out_vec[k][bytes_produced] = partial_byte[k];
            partial_byte[k] = 0;
        }
        bytes_produced++;
        byte_offset = 0;
     }
  }

  free(sym_position);
  assert(byte_offset == 0);

  return bytes_produced;

}
#endif //if 0

#endif

/* calculate the new euclid_dist after putting this packet's symbols in. Also update the batch history for d_euclid_dist */
void
digital_ofdm_frame_sink::slicer_ILP_2(gr_complex x, FlowInfo *flowInfo, gr_complex& closest_sym, unsigned char *bits,
                                      gr_complex* batched_sym_position, 
				      unsigned int ofdm_index, unsigned int subcarrier_index)
{
  int inno_pkts = flowInfo->innovative_pkts.size();

  unsigned int min_index = 0;
  assert(d_num_ofdm_symbols < MAX_OFDM_SYMBOLS);						// upper capped for now! 

  d_euclid_dist[ofdm_index][subcarrier_index][0] += abs(x - batched_sym_position[0]);
  float min_euclid_dist = d_euclid_dist[ofdm_index][subcarrier_index][0];

  //if(subcarrier_index == 0 && d_pkt_num == 1) 
  if(0) 
  {
     gr_complex t = batched_sym_position[0];
     printf("initialized, dist: %f, x: (%f, %f), sym: (%f, %f) \n", min_euclid_dist, x.real(), x.imag(), t.real(), t.imag()); fflush(stdout); 
  }

  // for each table entry, find the min(total_error, k) //
  unsigned int table_size = pow(double(d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));
  for(unsigned int j = 1; j < table_size; j++) {
      float euclid_dist = d_euclid_dist[ofdm_index][subcarrier_index][j] + abs(x - batched_sym_position[j]);

      //if(d_pkt_num == 1 && subcarrier_index == 0) 
      if(0)
      {
	  printf("euclid_dist: %f, x (%f, %f), sym (%f, %f) j: %d, min_euclid_dist: %f\n", euclid_dist, x.real(), x.imag(), batched_sym_position[j].real(), batched_sym_position[j].imag(), j, min_euclid_dist); fflush(stdout);
      }

      if (euclid_dist < min_euclid_dist) {
           min_euclid_dist = euclid_dist;
           min_index = j;
       }
	
       d_euclid_dist[ofdm_index][subcarrier_index][j] = euclid_dist;			// update d_euclid_dist with this batch's info //
  }

  // assign closest_sym and bits // 
  closest_sym = batched_sym_position[min_index];            // closest_sym: the ideal position where R should hv been at //

  if(d_data_nbits == 1)
      getSymOutBits_ILP(bits, min_index);
  else if(d_data_nbits == 2)
      getSymOutBits_ILP_QPSK(bits, min_index);
  else if(d_data_nbits == 4) {
      assert(d_batch_size == 1);
      getSymOutBits_ILP_QAM16(bits, min_index);
  }

  d_avg_evm_error += min_euclid_dist;
  //printf("xxx %f %f %f %f\n", x.real(), x.imag(), closest_sym.real(), closest_sym.imag()); fflush(stdout);
  //if(d_pkt_num == 1 && subcarrier_index == 0) 
  if(0) 
  {
       printf("min_euclid_dist: %f, ofdm_symbol: %d, subcarrier_index: %d, x: (%.2f, %.2f), closest: (%.2f, %.2f), min_index: %d \n\n", min_euclid_dist, ofdm_index, subcarrier_index, x.real(), x.imag(), closest_sym.real(), closest_sym.imag(), min_index);
       fflush(stdout); 
  }
}

/* just some test code */
void
digital_ofdm_frame_sink::test_decode_signal(gr_complex *in, vector<gr_complex*> interpolated_coeffs) {
  int num_senders = d_pktInfo->n_senders;
  assert(num_senders == 1); 
  assert(d_batch_size == 1);

  for(unsigned int i = 0; i < d_data_carriers.size(); i++) 
  {
     for(unsigned int j = 0; j < d_batch_size; j++)
     {
	 int subcarrier = d_data_carriers[i];
         int coeff_index = subcarrier * d_batch_size + j;
         for(int k = 0; k < num_senders; k++)
         {
             gr_complex *estimates = d_pktInfo->hestimates[k];                    // estimates for 'kth' sender
             gr_complex *in_coeffs = interpolated_coeffs[k];                      // interpolated coeffs for 'kth' sender

             gr_complex this_coeff = (gr_complex(1.0, 0.0)/estimates[subcarrier])*in_coeffs[coeff_index];
	     in[subcarrier] /= this_coeff;
         }
     }
  }

  /* the corresponding equalization needs to be done on the pilot carriers as well */
  for(unsigned int i = 0; i < d_pilot_carriers.size(); i++) {
     gr_complex *estimates = d_pktInfo->hestimates[0];
     int subcarrier = d_pilot_carriers[i];
     in[d_pilot_carriers[i]] /= (gr_complex(1.0, 0.0)/estimates[subcarrier]);
  }
}

/* crc related stuff - so that ofdm_packet_utils.py can be removed completely! */
bool
digital_ofdm_frame_sink::crc_check(std::string msg, std::string& decoded_str)
{
  int fec_n = d_fec_n, fec_k = d_fec_k;
  int exp_len = d_expected_size;
  
  int len = msg.length();
  unsigned char *msg_data = (unsigned char*) malloc(len+1);
  memset(msg_data, 0, len+1);
  memcpy(msg_data, msg.data(), len); 

  /* dewhiten the msg first */
  //printf("crc_check begin! len: %d\n", len); fflush(stdout);
  dewhiten(msg_data, len);
  std::string dewhitened_msg((const char*) msg_data, len);

  /*
  for(int i = 0; i < len; i++) {
     printf("%02x ", (unsigned char) dewhitened_msg[i]); fflush(stdout);
  } 
  printf("\n"); fflush(stdout);
  */

  /* perform any FEC if required */
  //decoded_msg = dewhitened_msg;

  // remove the training symbol (1 OFDM symbol at the start of the payload 
  // refer to ofdm_packet_utils.py at the transmitter for the addition of this symbol
  int num_training_bytes = 2*(d_data_carriers.size() * d_data_nbits)/8;
  dewhitened_msg = dewhitened_msg.substr(num_training_bytes);
  exp_len -= num_training_bytes;

  if(fec_n > 0 && fec_k > 0) {
	decoded_str = digital_rx_wrapper(dewhitened_msg, fec_n, fec_k, 1, exp_len);
  } else {
	decoded_str = dewhitened_msg;
  }

  if(len < 4) {
	free(msg_data);
	return false;
  }

  /* now, calculate the crc based on the received msg */
  std::string msg_wo_crc = decoded_str.substr(0, exp_len-4); 
  unsigned int calculated_crc = digital_crc32(msg_wo_crc);
  char hex_crc[15];
  snprintf(hex_crc, sizeof(hex_crc), "%08x", calculated_crc);
  //printf("hex calculated crc: %s  int calculated crc: %u\n", hex_crc, calculated_crc); 

 
  /* get the msg crc from the end of the msg */
  std::string expected_crc = decoded_str.substr(exp_len-4, 4);
  std::string hex_exp_crc = "";

  for(int i = 0; i < expected_crc.size(); i++) {
        char tmp[3];
        snprintf(tmp, sizeof(tmp), "%02x", (unsigned char) expected_crc[i]);
	//printf("%s", test); fflush(stdout);
	hex_exp_crc.append(tmp);
  }

  //printf("hex exp crc (%d): %s\n", sizeof(hex_exp_crc), hex_exp_crc.c_str()); fflush(stdout);
  free(msg_data);
  bool res =  (hex_exp_crc.compare(hex_crc) == 0);

  //printf("crc_check end: %d\n", res); fflush(stdout);
  return res;
  //return (hex_exp_crc.compare(hex_crc) == 0);
}

inline void
digital_ofdm_frame_sink::print_msg(std::string msg) {
  int len = msg.length();
  for(int i = 0; i < len; i++) {
     printf("%02x", (unsigned char) msg[i]); 
  }  
  printf("\n"); fflush(stdout);
}

/* returns true if a tag was found */
inline void
digital_ofdm_frame_sink::test_timestamp(int output_items) {
  //output_items = last_noutput_items;
  unsigned int tag_port = 1;
  std::vector<gr_tag_t> rx_sync_tags;
  const uint64_t nread1 = nitems_read(tag_port);
//get_tags_in_range(rx_sync_tags, tag_port, nread1, nread1+output_items, SYNC_TIME);
  get_tags_in_range(rx_sync_tags, tag_port, nread1, nread1+17, SYNC_TIME); 

  //printf("(SINK) nread1: %llu, output_items: %d\n", nread1, output_items); fflush(stdout);
  if(rx_sync_tags.size()>0) {
     size_t t = rx_sync_tags.size()-1;
     uint64_t offset = rx_sync_tags[t].offset;

     //printf("test_timestamp1 (SINK):: found %d tags, offset: %llu, output_items: %d, nread1: %llu\n", rx_sync_tags.size(), rx_sync_tags[t].offset, output_items, nread1); fflush(stdout);

     const pmt::pmt_t &value = rx_sync_tags[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));

     d_sync_tag.value = rx_sync_tags[t].value;
     d_sync_tag.offset = offset;
  }
 
  //std::cerr << "SINK---- Header received, with no sync timestamp1?\n";
  //assert(false);
}

#if 0
/* to test the triggered sending behavior. Creates a packet with a dumb header, uses the samples recorded earlier to 
   to generate the packet and transmit */
void
digital_ofdm_frame_sink::test_sync_send(CreditInfo *creditInfo) {
  printf("test_sync_send ---------- \n"); fflush(stdout);
  int fd = 0;
  if(!d_sync_file_opened) {
      const char *filename1 = "data_tx_symbols.dat";
      //int fd;
      if ((fd = open (filename1, O_RDONLY|OUR_O_LARGEFILE|OUR_O_BINARY, 0664)) < 0) {
         perror(filename1);
	 assert(false);
      }
      else {
         if((d_fp_sync_symbols = fdopen (fd, true ? "rb" : "r")) == NULL) {
             fprintf(stderr, "sync data file cannot be opened\n");
             close(fd);
	     assert(false);
         }
      }

      d_sync_file_opened = true;  
  }

  FlowInfo *flow_info = getFlowInfo(false, creditInfo->flowId);
  assert(d_fp_sync_symbols != NULL);
  
  /* read data samples to send from a file: 'data' content of 1 packet = 6 oFDM symbols */
  int d_symbols = 7;
  int n_actual_symbols = d_symbols * d_data_carriers.size();

  /*
  gr_complex *out_symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_actual_symbols);
  int count = fread_unlocked(out_symbols, sizeof(gr_complex), n_actual_symbols, d_fp_sync_symbols);

  assert(count == n_actual_symbols);
  */

  /* pick new random <coeff> and encode the data symbols */
  assert(d_batch_size == 1);

  float phase = rand() % 360 + 1;
  float amp = 1.0;
  gr_complex coeff = amp * gr_expj(phase * M_PI/180);
  for(int i = 0; i < n_actual_symbols; i++) {
	//out_symbols[i] *= coeff;
  }

  /* make header */
  unsigned char header_bytes[HEADERBYTELEN];
  MULTIHOP_HDR_TYPE header;
  memset(&header, 0, sizeof(MULTIHOP_HDR_TYPE));

  header.inno_pkts = 1;

  //float factor = normalizeSignal1(out_symbols, 1);
  header.factor = 1.0;

  /* put the coeff into the header */
  header.coeffs[0].phase = phase * SCALE_FACTOR_PHASE;
  header.coeffs[0].amplitude = amp * SCALE_FACTOR_AMP;

  int num_carriers = d_data_carriers.size()/2;
  for(unsigned int s = 1; s < num_carriers; s++) {
      for(unsigned int k = 0; k < d_batch_size; k++) {
          int index = s * d_batch_size + k;
          header.coeffs[index].phase = header.coeffs[k].phase;
          header.coeffs[index].amplitude = header.coeffs[k].amplitude;
      }
  }

  /* populate the 'header' struct and the 'header_bytes' */
  makeHeader(header, header_bytes, flow_info, creditInfo->nextLink.linkId);
  //debugHeader(header_bytes); 

  /* the outgoing message 
  gr_message_sptr out_msg = gr_make_message(DATA_TYPE, 0, 0, HEADERBYTELEN + (n_actual_symbols * sizeof(gr_complex)));
  memcpy(out_msg->msg(), header_bytes, HEADERBYTELEN);

  int offset = HEADERBYTELEN;
  memcpy(out_msg->msg() + HEADERBYTELEN, out_symbols, sizeof(gr_complex) * n_actual_symbols); */

  /* just send the header for now (no payload)  */
  gr_message_sptr out_msg = gr_make_message(DATA_TYPE, 0, 0, HEADERBYTELEN);
  memcpy(out_msg->msg(), header_bytes, HEADERBYTELEN);  
  

  /* set the outgoing timestamp */
  uint64_t sync_secs;
  double sync_frac_of_secs;
  calc_outgoing_timestamp(sync_secs, sync_frac_of_secs);

  printf("TX timestamp (%llu, %f) \n", sync_secs, sync_frac_of_secs); fflush(stdout);

  /* 
  double tx_duration = (2*(d_num_hdr_ofdm_symbols+1) + d_num_ofdm_symbols) * (d_fft_length+cp_length) * time_per_sample;
  int tx_sec = (int) tx_duration;
  double tx_frac_sec = tx_duration - (int) tx_duration; */

  out_msg->set_timestamp(sync_secs, sync_frac_of_secs);
  //printf("FWD PACKET---- TX timestamp (%llu, %f), proc_time(%llu, %f)", sync_secs, sync_frac_of_secs, (uint64_t) proc_time.get_full_secs(), proc_time.get_frac_secs());
  fflush(stdout);

  d_out_queue->insert_tail(out_msg);
  out_msg.reset();

  //free(out_symbols);
}
#endif

inline void
digital_ofdm_frame_sink::calc_outgoing_timestamp(uint64_t &sync_secs, double &sync_frac_of_secs) {
  /* set the outgoing timestamp */
  const pmt::pmt_t &value = d_sync_tag.value;
  sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
  sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));

  /* calculate duration to send out preamble+header */ 
  int decimation = 128;
  double rate = 1.0/decimation;

  int null_ofdm_symbols = 2000;

  int cp_length = d_fft_length/4;

  /* if lead forwarder, then go after the 'guard' duration, else wait for lead sender's header+preamble */
  uint32_t num_samples = (null_ofdm_symbols * (d_fft_length+cp_length));
  uint32_t gap_samples = 0; 

  if(d_fwd_index == 2) {
      gap_samples = (d_preamble.size()+d_num_hdr_ofdm_symbols) * (d_fft_length+cp_length);
  }
  num_samples += gap_samples;

  /* calculate the duration reqd to send this packet */
  double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
  double duration = num_samples * time_per_sample;
  uhd::time_spec_t duration_time = uhd::time_spec_t((uint64_t) duration, duration - (uint64_t) duration);
  printf("RX timestamp (%llu, %f), duration: %f\n", sync_secs, sync_frac_of_secs, duration); fflush(stdout);

  uhd::time_spec_t c_time = d_usrp->get_time_now();
  uhd::time_spec_t proc_time = c_time - uhd::time_spec_t(sync_secs, sync_frac_of_secs);

  /* add the duration to the current timestamp */
  sync_secs += (int) duration;
  sync_frac_of_secs += (duration - (int) duration);
  if(sync_frac_of_secs>1) {
     sync_secs += (uint64_t)sync_frac_of_secs;
     sync_frac_of_secs -= (uint64_t)sync_frac_of_secs;
  }
  d_out_pkt_time = uhd::time_spec_t(sync_secs, sync_frac_of_secs);

  /* ensure the interval from the last packet sent is atleast the duration */
  /* would happen if multiple packets were being sent in one-go coz credits told you so */
  uhd::time_spec_t interval_time = d_out_pkt_time - d_last_pkt_time;
  if(interval_time < duration_time) {
     uhd::time_spec_t delta_time = (duration_time - interval_time);
     d_out_pkt_time += delta_time;

     /* extremely hacky: if fwd_index == 2, then I don't want the (preamble+header) gap again, 
	it was already accounted for when the first packet of this burst was sent out */
     d_out_pkt_time -= (gap_samples*time_per_sample);
  }

  printf("FWD PACKET -- curr(%llu, %f), interval (%llu, %f), duration (%llu, %f), out(%llu, %f), last(%llu, %f)\n", 
			(uint64_t) c_time.get_full_secs(), c_time.get_frac_secs(), 
			(uint64_t) interval_time.get_full_secs(), interval_time.get_frac_secs(),
			(uint64_t) duration_time.get_full_secs(), duration_time.get_frac_secs(),
			(uint64_t) d_out_pkt_time.get_full_secs(), d_out_pkt_time.get_frac_secs(),
			(uint64_t) d_last_pkt_time.get_full_secs(), d_last_pkt_time.get_frac_secs());  
  fflush(stdout);

  sync_secs = d_out_pkt_time.get_full_secs();
  sync_frac_of_secs = d_out_pkt_time.get_frac_secs();
  d_last_pkt_time = d_out_pkt_time;
}

#ifdef LSQ_COMPRESSION
inline void
digital_ofdm_frame_sink::getCoefficients_LSQ(gr_complex *in_coeffs, COEFF *out_coeffs, unsigned int obs, unsigned int degree) {
  cx_fmat A = randu<cx_fmat> (degree+1, 1);
  cx_fmat Y = randu<cx_fmat> (obs, 1);
  cx_fmat X = randu<cx_fmat> (obs, degree+1);

  // build X //
  for(unsigned int i = 0; i < obs; i++) {
     for(unsigned int j = 0; j < degree+1; j++) {
        X(i, j) = pow(i, j);
     }
  }

  // build Y //
  for(unsigned int i = 0; i < obs; i++) {
     Y(i, 0) = in_coeffs[i] * gr_complex(1e3, 0.0);		// scaling phase 1
  }

  // get A //
  A = solve(X, Y);

#ifdef DEBUG
  A.print("A = "); printf("\n");
  Y.print("Y = "); printf("\n");
  X.print("X = "); printf("\n");

  // print calculated y again //
  for(unsigned int i = 0; i < obs; i++) {
     gr_complex y(0.0, 0.0);
     for(unsigned int j = 0; j < degree+1; j++) {
        y += (A(j, 0) * gr_complex(pow(i, j), 0.0));
     }
     //printf("(%.3f, %.3f) [%.3f]  [%.3f]\n", y.real(), y.imag(), abs(y), arg(y));
     cout<<i<<" "<<abs(y)<<" "<<arg(y)<<endl;
  }
#endif

  for(unsigned int i = 0; i < degree+1; i++) {
     //out_coeffs[i].amplitude = abs(A(i, 0))  * SCALE_FACTOR_AMP;
     out_coeffs[i].amplitude = abs(A(i, 0)) * pow(10, i+1);      // scaling phase 2
     out_coeffs[i].phase = ToPhase_f(A(i, 0)) * SCALE_FACTOR_PHASE;
  }
}

inline void
digital_ofdm_frame_sink::dumpCoeffs_LSQ(gr_complex *coeff, int n, cx_fmat A) {
   // dump the coeff first //
   if(!d_coeff_open) {
      const char *filename = "coeff.dat";
      int fd;
      if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename);
         assert(false);
      }
      else {
         if((d_fp_coeff_y = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "coeff file cannot be opened\n");
            close(fd);
            assert(false);
         }
      }

      const char *filename1 = "coeff-y.dat";
      int fd1;
      if ((fd1 = open (filename1, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename1);
         assert(false);
      }
      else {
         if((d_fp_coeff_y1 = fdopen (fd1, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "coeff file cannot be opened\n");
            close(fd1);
            assert(false);
         }
      }
      d_coeff_open = true;
   }

   assert(d_fp_coeff_y);
   int count = ftell(d_fp_coeff_y);
   count = fwrite_unlocked(&coeff[0], sizeof(gr_complex), n, d_fp_coeff_y);
   assert(count == n);

   // now dump the LSQ curve fitting points //
   gr_complex *y = (gr_complex*) malloc(sizeof(gr_complex) * n);
   memset(y, 0, sizeof(gr_complex) * n);
   for(unsigned int i = 0; i < n; i++) {
      for(unsigned int j = 0; j < d_degree+1; j++) {
	  y[i] += (A(j, 0) * gr_complex(pow(i, j), 0.0));
      }
   }   

   assert(d_fp_coeff_y1);
   count = ftell(d_fp_coeff_y1);
   count = fwrite_unlocked(&y[0], sizeof(gr_complex), n, d_fp_coeff_y1);
   assert(count == n);

   free(y);
}


inline void
digital_ofdm_frame_sink::unpackCoefficients_LSQ(gr_complex *in_coeffs, gr_complex *out_coeffs, unsigned int obs, unsigned int degree) {

   printf("unpackCoefficients_LSQ degree: %d, obs: %d\n", degree, obs); fflush(stdout);
   memset(out_coeffs, 0, sizeof(gr_complex) * obs);
   for(unsigned int i = 0; i < obs; i++) {
      for(unsigned int j = 0; j < degree+1; j++) {
	  out_coeffs[i] += (in_coeffs[j] * gr_complex(pow(i, j), 0.0));
      }
      out_coeffs[i] /= gr_complex(1e3, 0.0);					// descaling, phase 2
      //printf("(%f, %f)\n", out_coeffs[i].real(), out_coeffs[i].imag());
   }

#if 0
   if(!d_coeff_unpacked_open) {
      const char *filename = "unpacked_coeff.dat";
      int fd;
      if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename);
         assert(false);
      }
      else {
         if((d_fp_coeff_unpacked = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "coeff file cannot be opened\n");
            close(fd);
            assert(false);
         }
      }
      d_coeff_unpacked_open = true;
   }

   int count = ftell(d_fp_coeff_unpacked);
   count = fwrite_unlocked(out_coeffs, sizeof(gr_complex), obs, d_fp_coeff_unpacked);
   assert(count == obs);
#endif
}

inline void
digital_ofdm_frame_sink::packCoefficientsInHeader_LSQ(MULTIHOP_HDR_TYPE& header, gr_complex* coeffs, int num_inno_pkts, FlowInfo *flowInfo) {
  printf("packCoefficientsInHeader_LSQ\n"); fflush(stdout);
  //reduceCoefficients_LSQ(flowInfo);                                                 // fill up flowInfo->reduced_coeff //

  /* now reduce the coefficients w.r.t. the new selected coeffs */
  int num_carriers = d_data_carriers.size();
  COEFF lsq_coeffs[MAX_BATCH_SIZE*MAX_DEGREE];

  for(unsigned int k = 0; k < d_batch_size; k++) {
      gr_complex new_coeffs[MAX_DATA_CARRIERS];
      for(unsigned int s = 0; s < num_carriers; s++) {
	 for(int j = 0; j < num_inno_pkts; j++) {
	    gr_complex *pkt_coeffs = flowInfo->reduced_coeffs[j];  		// reduced coeffs for this inno pkt //
	    new_coeffs[s] += (coeffs[j] * pkt_coeffs[s + k*num_carriers]);
 	    //printf("(%.3f, %.3f) = (%.3f, %.3f) * (%.3f, %.3f)\n", new_coeffs[s].real(), new_coeffs[s].imag(), coeffs[j].real(), coeffs[j].imag(), pkt_coeffs[s + k*num_carriers].real(), pkt_coeffs[s + k*num_carriers].imag()); fflush(stdout);
	 } 
      }

      getCoefficients_LSQ(new_coeffs, &lsq_coeffs[k*(d_degree+1)], num_carriers, d_degree);
  }

  memcpy(header.coeffs, lsq_coeffs, sizeof(COEFF) * (d_degree+1) * d_batch_size);
  printf("packCoefficientsInHeader end\n"); fflush(stdout);
}

/* reduction means to combine the 'h' and 'coeff' of different senders. Eg with 2 senders
   (c1.h11+c2.h21)P1 + (d1.h11+d2.h21)P2, where the (..) are the reduced coeffs and 
   (c1, d1) and (c2, d2) are the coefficients of S1 and S2 respectively */
inline void
digital_ofdm_frame_sink::reduceCoefficients_LSQ(FlowInfo *flowInfo) {

  int num_senders = d_pktInfo->n_senders;
  printf("reduceCoefficients_LSQ begin :: senders: %d\n", num_senders); fflush(stdout);

  /* now perform the reduction */
  unsigned int num_carriers = d_data_carriers.size();

  gr_complex *coeff = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * num_carriers);
  memset(coeff, 0, sizeof(gr_complex) * d_batch_size * num_carriers);
  flowInfo->reduced_coeffs.push_back(coeff);

  for(unsigned int k = 0; k < d_batch_size; k++) {
      for(unsigned int i = 0; i < num_carriers; i++) {
	  unsigned int di = d_data_carriers[i];
	  int index = i + k*num_carriers;
	  for(unsigned int j = 0; j < num_senders; j++) {
	      gr_complex *estimates = d_pktInfo->hestimates[j];                    // estimates for 'kth' sender
	      gr_complex *rx_coeffs = d_pktInfo->coeffs.at(j);

	      coeff[index] += (((gr_complex(1.0, 0.0)/estimates[di]) * rx_coeffs[index])/(d_pktInfo->norm_factor[j]));
	      //printf("(%f, %f) += (1.0, 0.0)/(%f, %f) * (%f, %f)\n", coeff[index].real(), coeff[index].imag(),
		//					estimates[di].real(), estimates[di].imag(), 
		//					rx_coeffs[index].real(), rx_coeffs[index].imag()); fflush(stdout);
	  }
	  //printf("---\n"); fflush(stdout);
      }
  }

#ifdef DEBUG
  // debug //
  printf("reduced coefficients LSQ\n"); fflush(stdout);
  for(unsigned int k = 0; k < d_batch_size; k++) {
      for(unsigned int i = 0; i < num_carriers; i++) {
	 int index = i + k*num_carriers;
	 printf(" (%f, %f) \n", coeff[index].real(), coeff[index].imag()); 
	 fflush(stdout);
      }
  }
#endif

  printf("reduceCoeffients end\n"); fflush(stdout);
}
#endif

#ifdef SRC_PILOT
// demapper version where only the pilot rotation from source is used //
inline void
digital_ofdm_frame_sink::buildMap_pilot_SRC(FlowInfo *flowInfo, gr_complex* sym_position,
                                        vector<gr_complex*> interpolated_coeffs,
                                        vector<gr_complex> dfe, gr_complex carrier,
                                        int subcarrier_index, int o) {

  int num_senders = d_pktInfo->n_senders;
  int subcarrier = d_data_carriers[subcarrier_index];
  int n_data_carriers = d_data_carriers.size();

  gr_complex coeffs[5];
  for(unsigned int j = 0; j < d_batch_size; j++)
  {
#ifdef LSQ_COMPRESSION
      int coeff_index = j * n_data_carriers + subcarrier_index;                 // for LSQ, arrangement of coeffs is different: for all batches(sub1, sub2 .. subN)
#else
      int coeff_index = subcarrier * d_batch_size + j;                          // non LSQ, arrangment is: for all_subcarriers(batch1, batch2 .. batchN)
#endif
      for(int k = 0; k < num_senders; k++)
      {
          gr_complex *estimates = d_pktInfo->hestimates[k];                    // estimates for 'kth' sender
          gr_complex *in_coeffs = interpolated_coeffs[k];                      // interpolated coeffs for 'kth' sender

          gr_complex coeff = ((gr_complex(1.0, 0.0)/(estimates[subcarrier])) * in_coeffs[coeff_index])/(d_pktInfo->norm_factor[k]);
          if(k == 0) coeffs[j] = coeff;
          else coeffs[j] += coeff;
      }
      coeffs[j] *= (gr_complex(1.0, 0.0)/(carrier * dfe[subcarrier_index]));
  }

  if(d_batch_size == 1) {
         int j = 0;
	 printf("d_data_sym_position.size: %d\n", d_data_sym_position.size()); fflush(stdout);
	 for(unsigned int m1 = 0; m1 < d_data_sym_position.size(); m1++) {
	     sym_position[j++] = (coeffs[0] * d_data_sym_position[m1]);
	 }
  }
  else if(d_batch_size == 2) {
         int j = 0;
	 if(o == 0 && subcarrier_index == 0 && 0) 
	       cout << "delta_coeff: " << (arg(coeffs[0]) - arg(coeffs[1])) * 180/M_PI << " -- " << arg(coeffs[0]) * 180/M_PI << " --- " << arg(coeffs[1]) * 180/M_PI<< endl;
	 for(unsigned int m1 = 0; m1 < d_data_sym_position.size(); m1++) {
	     for(unsigned int m2 = 0; m2 < d_data_sym_position.size(); m2++) {
		gr_complex pos = (coeffs[0] * d_data_sym_position[m1]) + (coeffs[1] * d_data_sym_position[m2]);
		//std::cout<<"sym_pos:: " << pos <<" = " << coeffs[0] << " * " << d_data_sym_position[m1] << " + " << coeffs[1] << " * " << d_data_sym_position[m2] << endl;
		sym_position[j++] = pos; 
                //printf("sym_position[%d]: (%f, %f)\n", j, pos.real(), pos.imag());
	     }
         }
  }
  else if (d_batch_size == 3) {
         int j = 0;
         sym_position[j++] = coeffs[0]* d_data_sym_position[0] + coeffs[1]* d_data_sym_position[0] + coeffs[2]* d_data_sym_position[0];
         sym_position[j++] = coeffs[0]* d_data_sym_position[0] + coeffs[1]* d_data_sym_position[0] + coeffs[2]* d_data_sym_position[1];
         sym_position[j++] = coeffs[0]* d_data_sym_position[0] + coeffs[1]* d_data_sym_position[1] + coeffs[2]* d_data_sym_position[0];
         sym_position[j++] = coeffs[0]* d_data_sym_position[0] + coeffs[1]* d_data_sym_position[1] + coeffs[2]* d_data_sym_position[1];
         sym_position[j++] = coeffs[0]* d_data_sym_position[1] + coeffs[1]* d_data_sym_position[0] + coeffs[2]* d_data_sym_position[0];
         sym_position[j++] = coeffs[0]* d_data_sym_position[1] + coeffs[1]* d_data_sym_position[0] + coeffs[2]* d_data_sym_position[1];
         sym_position[j++] = coeffs[0]* d_data_sym_position[1] + coeffs[1]* d_data_sym_position[1] + coeffs[2]* d_data_sym_position[0];
         sym_position[j++] = coeffs[0]* d_data_sym_position[1] + coeffs[1]* d_data_sym_position[1] + coeffs[2]* d_data_sym_position[1];
  }
}

unsigned int
digital_ofdm_frame_sink::demapper_ILP_2_pilot_SRC(unsigned int ofdm_symbol_index, vector<unsigned char*> out_vec,
	                                          FlowInfo *flowInfo, vector<gr_complex> dfe_pilot,
        	                                  vector<gr_complex*> interpolated_coeffs) {

  unsigned int bytes_produced = 0;
  assert(d_batch_size <= MAX_BATCH_SIZE);

  InnovativePktInfoVector innovativePkts = flowInfo->innovative_pkts;
  int pkt_index = innovativePkts.size() - 1;

  // get the current packet //
  PktInfo *pInfo = innovativePkts[pkt_index];
  gr_complex *rx_symbols_this_batch = pInfo->symbols;

  gr_complex *sym_vec = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
  memcpy(sym_vec, &rx_symbols_this_batch[ofdm_symbol_index * d_occupied_carriers], sizeof(gr_complex) * d_occupied_carriers);

  gr_complex carrier;
  vector<gr_complex> dfe_data;

  int n_data_carriers = d_data_carriers.size();
  track_pilot_dfe_SRC(sym_vec, pInfo->hestimates, carrier, dfe_pilot, ofdm_symbol_index);
  interpolate_data_dfe(dfe_pilot, dfe_data, false, NULL, gr_complex(0.0, 0.0));

#if 0
  logDFECorrectedSignals(sym_vec, dfe_data, carrier, pInfo);
#endif

  int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));
  gr_complex sigrot, closest_sym;

  /* for every packet of the batch, 'd_batch_size' # of partial bytes and bits will be decoded */
  unsigned int partial_byte[MAX_BATCH_SIZE];
  unsigned char bits[MAX_BATCH_SIZE];
  memset(partial_byte, 0, sizeof(unsigned int) * MAX_BATCH_SIZE);
  memset(bits, 0, sizeof(unsigned char) * MAX_BATCH_SIZE);

  unsigned int byte_offset = 0;

  // symbol position map needs to be built each time, because of DFE tracking //
  gr_complex *sym_position = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);

  for(unsigned int i = 0; i < n_data_carriers; i++) {

     // demodulate 1-byte at a time //
     if (byte_offset < 8) {

        // build the map for each subcarrier //
         buildMap_pilot_SRC(flowInfo, sym_position, interpolated_coeffs, dfe_data, carrier, i, ofdm_symbol_index);

         // slice it //
         sigrot = sym_vec[d_data_carriers[i]];
         slicer_ILP_2(sigrot, flowInfo, closest_sym, bits, sym_position, ofdm_symbol_index, i);
	
         assert((8 - byte_offset) >= d_data_nbits);

         for(unsigned int k = 0; k < d_batch_size; k++)
              partial_byte[k] |= bits[k] << (byte_offset);

         byte_offset += d_data_nbits;
     }

     if(byte_offset == 8) {
        for(int k = 0; k < d_batch_size; k++) {
            //printf("k: %d, demod_byte: %x (%d) \n", k, partial_byte[k], partial_byte[k]); fflush(stdout);
            //assert(false);
            out_vec[k][bytes_produced] = partial_byte[k];
            partial_byte[k] = 0;
        }
        bytes_produced++;
        byte_offset = 0;
     }
  }

  free(sym_position);
  assert(byte_offset == 0);

  return bytes_produced;
}

// used (1) only the pilot from the source is considered, everyone relays the pilots (2) multiple senders info is used to equalize the pilots //
/* Note - 
   1. When multiple senders are present, it gets a little tricky w.r.t the lead sender involved in the transmission. The symbol 'x', which 
      would have undergone a rotation at position "t" from the source, is now instead at position "t'". So, simple technique of cancelling the 
      intermediate node's frequency offset does not quite work. We first need to remove the effect of rotation at the new location of "t'" and then only 
      the rotation from the source at position "t" would be left
*/
void digital_ofdm_frame_sink::track_pilot_dfe_SRC(gr_complex *in, vector<gr_complex*> hestimates,
                                          gr_complex& carrier, vector<gr_complex>& dfe_pilot, int o) {
  gr_complex phase_error = 0.0;
  float cur_pilot = 1.0;

  int n_senders = hestimates.size();
  int sender = 0;										// basically the source //

  for (unsigned int i = 0; i < d_pilot_carriers.size(); i++) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];

    gr_complex total_H(0.0, 0.0);
    assert(n_senders <= 2);
    for(unsigned int j = n_senders-1; j < n_senders; j++) {
    //for(unsigned int j = 0; j < 1; j++) {		// REMOVEEEEEEEE
        gr_complex *estimates = hestimates[j];
	in[di] *= estimates[di];
	total_H += estimates[di];
    }
#if 0 
    if(n_senders == 2) {			// REMOVEEEEEEEEEEE
	in[di] = in[di]/total_H;
    }
#endif
    // some debugging //
#if 0
    float angle = arg(in[di]) * 180/M_PI;                       // rotation after equalization //
    if(angle < 0) angle += 360.0;
    float perfect_angle = arg(pilot_sym) * 180/M_PI;
    float delta_rotation = perfect_angle - angle;
    //printf("pilot %d, delta_P=%.3f\t", di, delta_rotation); fflush(stdout);
    printf("(%d, rot=%.2f)  ", di, delta_rotation); fflush(stdout);
#endif

    phase_error += (in[di] * conj(pilot_sym));
  }

  // update phase equalizer
  float angle = arg(phase_error);
  carrier = gr_expj(-angle);

#if 0
  // dump the frequency offset value //
  float freq_offset = angle/(2 * M_PI * (o+NULL_OFDM_SYMBOLS));
  printf("(@f: %.5f)  ", freq_offset); fflush(stdout);
#endif

  // update DFE based on pilots
  cur_pilot = 1.0;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); i++) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    gr_complex sigeq = in[di] * carrier * dfe_pilot[i];
    // FIX THE FOLLOWING STATEMENT
    if (norm(sigeq)> 0.001)
      dfe_pilot[i] += d_eq_gain * (pilot_sym/sigeq - dfe_pilot[i]);
  }
  d_end_angle[sender] = angle;
  //printf("  d_end_angle: %f\n", d_end_angle[sender]); fflush(stdout);
}

inline void
digital_ofdm_frame_sink::adjust_H_estimate(int sender)
{
  gr_complex *hestimate = d_pktInfo->hestimates[sender];
  float delta_angle = d_end_angle[sender] - d_start_angle[sender] + ((NUM_TRAINING_SYMBOLS+1) * d_slope_angle[sender]);
  printf("adjust_H_estimate: %.3f\n", delta_angle); fflush(stdout);
  for(unsigned int i = 0; i < d_occupied_carriers; i++) {
     hestimate[i] *= gr_expj(-delta_angle);
  }
}
#endif

inline void
digital_ofdm_frame_sink::openDFESymbolLog() {
  const char *filename = "rx_dfe_symbols.dat";
  int fd;
  if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     assert(false);
  }
  else {
      if((d_fp_dfe_symbols = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "rx dfe symbols file cannot be opened\n");
            close(fd);
            assert(false);
      }
  }
}

inline void
digital_ofdm_frame_sink::logDFECorrectedSignals(gr_complex *in, vector<gr_complex> dfe_data, gr_complex carrier, PktInfo *pInfo) {
  int count = ftell(d_fp_dfe_symbols);
  float amp = getAvgAmplificationFactor(pInfo->hestimates);
  int k = 0;
  count = 0;
  //equalizeSymbols(in, pInfo->hestimates[0]);
  for(unsigned int j = 0; j < d_occupied_carriers; j++) {
     if(d_all_carriers[j] == 0) {
        gr_complex symbol = in[j] * d_in_estimates[j] * dfe_data[k++] * carrier; // * gr_complex(amp, 0.0);
        count += fwrite_unlocked(&symbol, sizeof(gr_complex), 1, d_fp_dfe_symbols);
     }
  }
  //printf("logDFESymbols, count: %d\n", ftell(d_fp_dfe_symbols)); fflush(stdout);
}

inline void
digital_ofdm_frame_sink::openRxSymbolLog() {
  const char *filename = "rx_symbols.dat";
  int fd;
  if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     assert(false);
  }
  else {
      if((d_fp_rx_symbols = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "rx symbols file cannot be opened\n");
            close(fd);
	    assert(false);
      }
  }
}

/* logs only the /data/ symbols (data subcarriers) for offline analysis */
inline void 
digital_ofdm_frame_sink::logFrequencyDomainRxSymbols(bool equalize) {
  int count = ftell(d_fp_rx_symbols);

  //float h_slope = get_eq_slope();

#if 0
  float *ph_offset = (float*) malloc(sizeof(float)*d_data_carriers.size());
  memset(ph_offset, 0, sizeof(float)*d_data_carriers.size());
  get_phase_offset(ph_offset);
#endif

  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++) {
     int index = 0;
     gr_complex *rx_symbols = &(d_pktInfo->symbols[i*d_occupied_carriers]);
     for(unsigned int j = 0; j < d_occupied_carriers; j++) {
        if(d_all_carriers[j] == 0) {
           if(equalize) {
#if 1 
		// equalization based on the estimate //
              gr_complex symbol = rx_symbols[j] * d_in_estimates[j];
	      count = fwrite_unlocked(&symbol, sizeof(gr_complex), 1, d_fp_rx_symbols);
#else
		// equalization based on the slope of the estimate //
	      //gr_complex rotation = gr_expj(h_slope*index);		// derotation - hence "-"h_slope
	      //gr_complex symbol = rx_symbols[j] * rotation;

		// based on the consecutive ph_offset //
	      gr_complex symbol = rx_symbols[j] * gr_expj(ph_offset[index]); 
	      count = fwrite_unlocked(&symbol, sizeof(gr_complex), 1, d_fp_rx_symbols);
#endif
           }
           else {
              count = fwrite_unlocked(&rx_symbols[j], sizeof(gr_complex), 1, d_fp_rx_symbols);		// write first
	      //rx_symbols[j] = rx_symbols[j] * gr_expj(-d_timing_offset*index);				// de-rotate later
	   }
	   index++;
        }
     }
#if 0
     equalizeSymbols(rx_symbols, d_in_estimates);
#endif

  }
  //printf("logFreq, count: %d, timing_offset: %f\n", ftell(d_fp_rx_symbols), d_timing_offset); fflush(stdout);

  //free(ph_offset);
}

/* the slope could be +ve or -ve. The heursitic to check for it is as follows: 
   - if(phase[0] < phase[last]) : slope can be inc. or dec. (unwrapped)

   - if(phase[0] < phase[mid])
	if(phase[mid] < phase[last])
	   slope = +ve
	else
	   slope = -ve
     else (phase[0] > phase[mid])
	 if(phase[mid] > phase[last])
	   slope = -ve
	 else 
	   slope = +ve
*/


inline float
digital_ofdm_frame_sink::get_eq_slope() {
  int n = d_data_carriers.size();

  float phase_0 = arg(d_in_estimates[d_data_carriers[0]]);
  float phase_last = arg(d_in_estimates[d_data_carriers[n-1]]);

  bool pos_slope = true;
  bool unwrap = false;

  // pretty crappy implementation but should work - essentially, the phase change between 
  // subcarriers seperated by a gap (6) is measured, if a wrap happens, it will be noticeable 
  // here. Multiple wrap will not happen in the slope. 
  float p1 = arg(d_in_estimates[d_data_carriers[0]]);
  for(int i = 4; i < n; i+=4) {
     float p2 = arg(d_in_estimates[d_data_carriers[i]]);
     if((p2-p1) > M_PI) { 
	pos_slope = false;
	unwrap = true;
	//printf("p1: %f, p2: %f, i: %d\n", p1, p2, i);
	break;
     } else if((p2-p1) < -M_PI) {
	pos_slope = true;
	unwrap = true;
	//printf("p1: %f, p2: %f, i: %d\n", p1, p2, i);
	break;
     }

     pos_slope = (p2-p1>0)?true:false;
     p1 = p2;
  }

  if(unwrap) {
    if(pos_slope) 
	phase_last += 2*M_PI;
    else
	phase_last -= 2*M_PI;
  }

  float slope_val = (phase_last - phase_0)/((float) n);
  printf("get_eq_slope -- is+ve: %d, slope: %f, phase_0: %f, phase_last: %f, unwrap: %d\n", pos_slope, slope_val, phase_0, phase_last, unwrap);
  return slope_val;
}

/* utility function */
inline void
digital_ofdm_frame_sink::get_phase_offset(float *ph_offset) {
  int n = d_data_carriers.size();
  bool pos_slope = (get_eq_slope() > 0) ? true:false;		// equalizer slope
  
  float start = arg(d_in_estimates[d_data_carriers[0]]);
  float pi_delta = (pos_slope == true)?2*M_PI:-2*M_PI;

  for(int i = 1; i < n; i++) {
#if 0
     float cur = arg(gr_complex(1.0, 0.0)/d_in_estimates[d_data_carriers[i]]);
     float prev = arg(gr_complex(1.0, 0.0)/d_in_estimates[d_data_carriers[i-1]]);
#endif
     float cur = arg(d_in_estimates[d_data_carriers[i]]);
     float prev = arg(d_in_estimates[d_data_carriers[i-1]]);

     float diff = cur - prev;
     if(abs(diff) >= M_PI) 
	cur += pi_delta;
        //ph_offset[i] += pi_delta;
     ph_offset[i] = cur - start;
     //printf("prev: %f, start: %f, diff: %f, cur: %f, phase_offset: %f\n", prev, start, diff, cur, ph_offset[i]); fflush(stdout);
  }
}

#if 0
/* based on EVM which in turn is calculated from the training symbols sent in the same constellation 
   as the data is sent
   ref: http://www.eng.usf.edu/~hmahmoud/pdf/pubs/hmahmoud_WCOMM09.pdf
   "Error Vector Magnitude to SNR Conversion for Nondata-Aided Receivers"
*/ 
inline void
digital_ofdm_frame_sink::calculateSNR1() {
   float evm = 0.0; 
   for(unsigned int i = 0; i < NUM_TRAINING_SYMBOLS; i++) {
       int ti = i * d_occupied_carriers;
       for(unsigned int j = 0; j < d_occupied_carriers; j+=2) {
           std::cout<< "training: " << d_training_symbols[ti+j] << " known: " << d_known_symbols[j] << " diff: " << d_training_symbols[ti+j] - d_known_symbols[j] << endl;
           evm += pow(abs(d_training_symbols[ti+j] - d_known_symbols[j]), 2.0);
       }
   }
   evm /= (d_occupied_carriers);

   float P_0 = 0.0;
   int M = d_hdr_sym_position.size();
   /* calculate p0 */
   for(unsigned int i = 0; i < M; i++) {
       P_0 += pow(abs(d_hdr_sym_position[i]), 2.0);
   }
   P_0 /= M;

   float evm_rms = sqrt(evm/P_0);
   float snr = 1/(pow(evm_rms, 2.0));
   printf("SNR: %f, EVM_RMS: %f, EVM: %f, P: %f\n", snr, evm_rms, evm , P_0); fflush(stdout);
}
#endif

inline void
digital_ofdm_frame_sink::calculateSNR() {
  assert(NUM_TRAINING_SYMBOLS > 0);
  float noise_var = 0.0;
  float signal_power = 0.0;

  int n_data_carriers = d_data_carriers.size();
  for(unsigned int i = 0; i < n_data_carriers; i++) {
      int di = d_data_carriers[i];
      //std::cout << " amp_NV: training:: " << d_training_symbols[i] << " " << d_training_symbols[i+d_occupied_carriers] << endl;
      noise_var += norm(d_training_symbols[di] - d_training_symbols[di+d_occupied_carriers]);
      signal_power += norm(d_training_symbols[di]);
  }

  noise_var = noise_var/(2*n_data_carriers);
  signal_power /= (n_data_carriers);
  float snr_ratio = signal_power/noise_var;

  float snr_db = 10*log10(snr_ratio);
  printf("SNR_DB: %.3f, signal: %.6f, noise_var: %.6f\n", snr_db, signal_power, noise_var); fflush(stdout);
}

/* generate 2 (base modulation) training symbols (helps in deducing snr and noise variance) */
inline void
digital_ofdm_frame_sink::generateKnownSymbols() {
   assert(NUM_TRAINING_SYMBOLS > 0);
   memset(d_known_symbols, 0, sizeof(gr_complex) * d_occupied_carriers);

   int M = d_hdr_sym_position.size();

   int k = 0;
   for(unsigned int i = 0; i < d_occupied_carriers; i++) {
      d_known_symbols[i] = d_hdr_sym_position[k%M];
      k++;
   }
}

inline float
digital_ofdm_frame_sink::getMinDistanceInConstellation(gr_complex *constell) {
   int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size));

   // find the min dist between any 2 points //
   float min_dt = 1000.0;
   for(int i = 0; i < n_entries; i++) {
      for(int j = i + 1; j < n_entries; j++) {
	  float dt = abs(constell[i] - constell[j]);
	  if(dt < min_dt) {
	      min_dt = dt;
	  }
      }
   }
   return min_dt;
}

/* calculates both BER and SER for each batch from the corresponding demodulated msgs
   - first modulate the message again (based on the native modulation)
   - compare against the symbols
*/
void
digital_ofdm_frame_sink::calculateSER(unsigned char msg[][MAX_PKT_LEN], int num_bytes) {
   int carriers = d_data_carriers.size();

   // read the true native symbols for each batch //
   if(!d_log_SER_symbols_open) {
      FILE *fp = NULL;
      const char *filename = "native_symbols.dat";
      int fd;
      if ((fd = open (filename, O_RDONLY|OUR_O_LARGEFILE|OUR_O_BINARY, 0664)) < 0) {
         perror(filename);
         assert(false);
      }
      else {
         if((fp = fdopen (fd, true ? "rb" : "r")) == NULL) {
            close(fd);
            assert(false);
         }
      }
      d_log_SER_symbols_open = true;

      int count = fread_unlocked(d_true_symbols, sizeof(gr_complex), d_batch_size * d_num_ofdm_symbols * carriers, fp);
      assert(count == d_batch_size * d_num_ofdm_symbols * carriers);
   }

   // modulate the symbols //
   gr_complex *symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * d_num_ofdm_symbols * carriers);
   memset(symbols, 0, sizeof(gr_complex) * d_batch_size * d_num_ofdm_symbols * carriers);

   for(unsigned int k = 0; k < d_batch_size; k++) 
      modulateMsg(&symbols[k*d_num_ofdm_symbols*carriers], msg[k], num_bytes);

   int correct_symbols = 0;
   for(unsigned int i = 0; i < d_num_ofdm_symbols; i++) {
       int index = i * carriers;
       for(unsigned int j = 0; j < carriers; j++) {
	   //cout<<"true: " << d_true_symbols[index+j]<<" rx: "<<symbols[index+j]<<endl;
	   if(d_true_symbols[index+j] == symbols[index+j]) 
	      correct_symbols++;	  
       }
   }

   int total_symbols = (d_num_ofdm_symbols) * carriers;
   float SER = ((float)correct_symbols)/(total_symbols);

   d_agg_total_symbols += total_symbols;
   d_agg_correct_symbols += correct_symbols;
   float agg_SER = ((float)d_agg_correct_symbols)/(d_agg_total_symbols);
   printf("SER:: %.3f, correct_symbols: %d, total_symbols: %d -------------- aggSER: %.3f\n", SER, correct_symbols, total_symbols, agg_SER); fflush(stdout);

   free(symbols);
}

void
digital_ofdm_frame_sink::modulateMsg(gr_complex* out, unsigned char *msg, int num_bytes)
{
  unsigned int msg_offset = 0, bit_offset = 0, resid = 0, nresid = 0;
  unsigned char msgbytes;
  unsigned int i = 0;
  while(1) {

    //unsigned int i = 0;
    unsigned char bits = 0;

    while((msg_offset < num_bytes)) { //&& (i < d_data_carriers.size())) 

       // need new data to process
       if(bit_offset == 0) {
          msgbytes = msg[msg_offset];
       }

       if(nresid > 0) {
          // take the residual bits, fill out nbits with info from the new byte, and put them in the symbol
         resid |= (((1 << nresid)-1) & msgbytes) << (d_data_nbits - nresid);
         bits = resid;

	 out[i] = d_data_sym_position[bits];
	 i++;

	 bit_offset += nresid;
	 nresid = 0;
	 resid = 0;
       }
       else {
         if((8 - bit_offset) >= d_data_nbits) {  // test to make sure we can fit nbits
           // take the nbits number of bits at a time from the byte to add to the symbol
           bits = ((1 << d_data_nbits)-1) & (msgbytes >> bit_offset);
	   bit_offset += d_data_nbits;

	   out[i] = d_data_sym_position[bits];
	   i++;
	 }
	 else {  // if we can't fit nbits, store them for the next 
           // saves d_nresid bits of this message where d_nresid < d_data_nbits
           unsigned int extra = 8-bit_offset;
	   resid = ((1 << extra)-1) & (msgbytes >> bit_offset);
	   bit_offset += extra;
	   nresid = d_data_nbits - extra;
	 }
       }

       if(bit_offset == 8) {
         bit_offset = 0;
         msg_offset++;
       }
    }

    if(msg_offset == num_bytes) {
	break;
    }
  }
}

#ifdef MIMO_RECEIVER
inline void
digital_ofdm_frame_sink::open_mimo_sock() 
{

  /* mimo server details */
  const char *src_ip_addr = "128.83.120.84"; //"128.83.143.15";
  int port = 9000;
  d_mimo_sock = open_client_sock(port, src_ip_addr, false);
  printf("open_mimo_sock success!\n"); fflush(stdout);
}

inline void
digital_ofdm_frame_sink::waitForDecisionFromMimo(unsigned char packet[MAX_BATCH_SIZE][MAX_PKT_LEN])
{
  printf("mimo: waitForDecisionFromMimo\n"); fflush(stdout);

  int n_entries = d_num_ofdm_symbols * d_data_carriers.size() * pow(double(d_data_sym_position.size()), double(d_batch_size));
  int buf_size = sizeof(float) * n_entries + 3 * sizeof(unsigned int);	// euclid_dist + batch + num_ofdm_symbols
  char *eth_buf = (char*) malloc(buf_size);
  memset(eth_buf, 0, buf_size);

  memcpy(&eth_buf[0], &d_active_batch, sizeof(unsigned int));
  memcpy(&eth_buf[4], &d_num_ofdm_symbols, sizeof(unsigned int));
  memcpy(&eth_buf[8], &d_pkt_num, sizeof(unsigned int));

  int offset = 12;
  int items = d_data_carriers.size() * pow(double(d_data_sym_position.size()), double(d_batch_size));
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++) {
      memcpy(&eth_buf[offset], d_euclid_dist[i], sizeof(float) * items);
      offset += (sizeof(float) * items);
  }
  
  // send on the backend //
  printf("mimo: size: %d, offset: %d, d_num_ofdm_symbols: %d\n", buf_size, offset, d_num_ofdm_symbols); fflush(stdout);
  int bytes_sent = send(d_mimo_sock, (char *)&eth_buf[0], buf_size, 0);
  if (bytes_sent < 0) {
    printf("mimo: Error: failed to send to MIMO, errno: %d\n", errno);
    assert(false);
  } else
    printf("mimo: @@@@@@@@@@@@@@@@ sent to MIMO (%d bytes) for batch %d, pkt_num: %d @@@@@@@@@@@@@@@@@@@@\n", bytes_sent, d_active_batch, d_pkt_num); fflush(stdout); 

#if 0
  // now wait for the decoded pkt //
  for(unsigned int k = 0; k < d_batch_size; k++) {
      int nbytes = recv(d_mimo_sock, packet[k], out_bytes, MSG_PEEK);
      if(nbytes == out_bytes) 
	 nbytes = recv(d_mimo_sock, packet[k], out_bytes, 0);
      else 
	 nbytes = recv(d_mimo_sock, packet[k], out_bytes, MSG_WAITALL);

      assert(nbytes == out_bytes);
  }
#endif
}
#endif

float
digital_ofdm_frame_sink::calc_CV_dt(gr_complex cv1, gr_complex cv2) {
   int M = d_data_sym_position.size();

   float energy = 0.0;
   float min_dt = 1000.0;

   int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size));
   int index = 0;
   gr_complex *comb_mod = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);
   for(int m1 = 0; m1 < M; m1++) {
      gr_complex p1 = cv1 * d_data_sym_position[m1];
      for(int m2 = 0; m2 < M; m2++) {
         gr_complex p2 = cv2 * d_data_sym_position[m2];
         comb_mod[index++] = p1 + p2;
	 energy += norm(p1+p2);
      }
   }
   assert(index == n_entries);
   energy = energy/((float)n_entries);

#if 1
   float factor = 1.0/sqrt(energy);
   for(int i = 0; i < n_entries; i++) 
      comb_mod[i] = comb_mod[i]*factor;
#endif

   for(int i = 0; i < n_entries; i++) {
      for(int j = i+1; j < n_entries; j++) {
         float dt = abs(comb_mod[i] - comb_mod[j]);
         if(dt < min_dt) {
            min_dt = dt;
         }
      }
   }

#if 0
   if(d_fp_comb_log == NULL) {
      char *filename = "ideal_constellation.dat";
      int fd;
      assert((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) >= 0);
      assert((d_fp_comb_log = fdopen(fd, true ? "wb" : "w")) != NULL);
   }
   assert(d_fp_comb_log != NULL);
   int count = ftell(d_fp_comb_log);
   count = fwrite_unlocked(comb_mod, sizeof(gr_complex), n_entries, d_fp_comb_log);
   assert(count == n_entries);
#endif

   free(comb_mod);
   return min_dt;
}

/* the lead sender selects the coeffs and sends them over the socket to the 
   slave sender. The slave then uses that knowledge to figure out its own 
   coefficients */
inline NodeId
digital_ofdm_frame_sink::get_coFwd()
{
   assert(d_fwd_index >= 1);
   assert(d_outCLinks.size() == 1);                     // assume only 1 cofwd for now //

   int coLinkId = d_outCLinks[0];
   CompositeLink *cLink = getCompositeLink(coLinkId);
   int n_coFwds = cLink->srcIds.size();
   assert(n_coFwds == 2);

   for(int i = 0; i < n_coFwds; i++) {
      if(cLink->srcIds[i] != d_id)
	 return cLink->srcIds[i];
   } 
   assert(false);
}

/* spits all the next-hop rx-ids */
inline void
digital_ofdm_frame_sink::get_nextHop_rx(NodeIds &rx_ids) {
   printf("get_nextHop_rx start\n"); fflush(stdout);
   assert(d_outCLinks.size() == 1);
   CompositeLink *link = getCompositeLink(d_outCLinks[0]);
   NodeIds dstIds = link->dstIds;
   NodeIds::iterator it = dstIds.begin();
   while(it != dstIds.end()) {
      rx_ids.push_back(*it);
      it++;
   }

   printf("get_nextHop_rx -- # rx_ids: %d\n", rx_ids.size()); fflush(stdout);
}

inline void
digital_ofdm_frame_sink::send_coeff_info_eth(CoeffInfo *coeffs)
{
   NodeIds rx_ids;
   get_nextHop_rx(rx_ids);
   int num_rx = rx_ids.size(); assert(num_rx > 0);

   // convert the *coeffs into CoeffInfo //
   int buf_size = sizeof(CoeffInfo) * num_rx;
   printf("send_coeff_info_eth to node %c\n", get_coFwd()); fflush(stdout);
   char *_buf = (char*) malloc(buf_size);
   memcpy(_buf, coeffs, buf_size);

   int bytes_sent = send(d_coeff_tx_sock, (char *)&_buf[0], buf_size, 0);
   printf("Coeffs sent, bytes_sent: %d, errno: %d\n", bytes_sent, errno); fflush(stdout);
   free(_buf);
}

/* the lead sender sends out 'batch_size' # of coeffs for each rx they will encounter.  */
inline void
digital_ofdm_frame_sink::get_coeffs_from_lead(CoeffInfo *coeffs)
{
   NodeIds rx_ids;
   get_nextHop_rx(rx_ids); 
   int num_rx = rx_ids.size(); assert(num_rx > 0);

   printf("get_coeffs_from_lead, num_rx: %d\n", num_rx); fflush(stdout);

   int buf_size = sizeof(CoeffInfo) * num_rx;
   char *_buf = (char*) malloc(buf_size);
   memset(_buf, 0, buf_size);

   int nbytes = recv(d_coeff_rx_sock, _buf, buf_size, MSG_PEEK);
   int offset = 0;
   if(nbytes > 0) {
      while(1) {
         nbytes = recv(d_coeff_rx_sock, _buf+offset, buf_size-offset, 0);
	 if(nbytes > 0) offset += nbytes;
         if(offset == buf_size) {
	    memcpy(coeffs, _buf, buf_size); 
	    break;
         }
      }
      assert(offset == buf_size);
   }

   assert(offset != 0);
   printf("get_coeffs_from_lead end -- offset: %d\n", offset); fflush(stdout);
   free(_buf);
}

inline void
digital_ofdm_frame_sink::smart_selection_local(gr_complex *coeffs, CoeffInfo *cInfo, FlowInfo *flowInfo) {
  // get the predicted value of h for each of the receivers //
  vector<unsigned char> rx_ids;
  get_nextHop_rx(rx_ids);
  vector<gr_complex> h_vec;
  for(int i = 0; i < rx_ids.size(); i++) {
      NodeId rx_id = rx_ids[i];
      gr_complex h = predictH(d_id, rx_id);         //h-val from me to this rx
      h_vec.push_back(h);
      printf("h: (%.2f, %.2f)\n", h.real(), h.imag());
  }

  InnovativePktInfoVector inno_pkts = flowInfo->innovative_pkts;
  unsigned int n_inno_pkts = inno_pkts.size();

  printf("smart_selection_local start, #inno_pkts: %d\n", n_inno_pkts); fflush(stdout);
  int num_carriers = d_data_carriers.size();

  // to cap the max iterations //
  int max_iter = 2000, iter = 0;
  gr_complex best_coeff[10];				// arbitrarily high
  assert(n_inno_pkts <= 10);
  CoeffInfo best_cInfo;
  float max_min_dt = 0.0;
  float threshold = 0.8;

  while(1) {

     gr_complex reduced_coeffs[MAX_BATCH_SIZE];
     memset(reduced_coeffs, 0, sizeof(gr_complex)*MAX_BATCH_SIZE);
     for(unsigned int i = 0; i < n_inno_pkts; i++) {
	gr_complex *pkt_coeffs = flowInfo->reduced_coeffs[i];               // reduced coeffs for this inno pkt //

	int nsenders = inno_pkts[i]->n_senders;
        float amp = abs((inno_pkts[i]->hestimates[0])[d_data_carriers[0]]);
        if(nsenders > 1) {
           for(int j = 1; j < nsenders; j++) {
               amp += abs((inno_pkts[i]->hestimates[j])[d_data_carriers[0]]);
           }
        }
        amp /= ((float) nsenders);

	float phase = (rand() % 360 + 1) * M_PI/180;
	coeffs[i] = (amp * ToPhase_c(phase));	

	for(unsigned int k = 0; k < d_batch_size; k++) {
	   reduced_coeffs[k] += (coeffs[i] * pkt_coeffs[k*num_carriers]);
	}
     }

     /* ensure its a good selection over all the next-hop rx 
	include the channel effect for this rx */
     float dt = 0.0;
     for(int i = 0; i < rx_ids.size(); i++) {
        cInfo->rx_id = rx_ids[i];
        for(int k = 0; k < d_batch_size; k++) {
           cInfo->coeffs[k] = reduced_coeffs[k] * h_vec[i];
        }

        dt += calc_CV_dt(cInfo->coeffs[0], cInfo->coeffs[1]);
     }
     dt /= ((float) rx_ids.size());
 
     bool good = (dt >= threshold)?true:false;

     // record if this is the best yet //
     if(!good) {
        if(dt > max_min_dt) {
           max_min_dt = dt;
           for(int k = 0; k < d_batch_size; k++) 
	       best_cInfo.coeffs[k] = cInfo->coeffs[k];

	   for(int k = 0; k < n_inno_pkts; k++) 
	       best_coeff[k] = coeffs[k];	
        }
     }
     else max_min_dt = dt;
 
     iter++;
     if(good || iter == max_iter) {
       printf("good: %d, iter: %d, max_min_dt: %f\n", good, iter, max_min_dt); fflush(stdout);

       if(iter == max_iter) {
	  for(int k = 0; k < d_batch_size; k++) 
	     cInfo->coeffs[k] = best_cInfo.coeffs[k];

	  for(int k = 0; k < n_inno_pkts; k++) 
	     coeffs[k] = best_coeff[k];
       }
       break;
     }
  }

  printf("smart_selection_local cv: \n"); fflush(stdout);
  for(int k = 0; k < n_inno_pkts; k++) {
     printf("(%f, %f) \n", coeffs[k].real(), coeffs[k].imag()); fflush(stdout);
  }
}

#if 0
inline void
digital_ofdm_frame_sink::smart_selection_local(gr_complex *coeffs, CoeffInfo *cInfo, FlowInfo *flowInfo) {

  // get the predicted value of h for each of the receivers //
  vector<unsigned char> rx_ids;
  get_nextHop_rx(rx_ids);
  vector<gr_complex> h_vec;
  for(int i = 0; i < rx_ids.size(); i++) {
      NodeId rx_id = rx_ids[i];
      gr_complex h = predictH(d_id, rx_id);         //h-val from me to this rx
      h_vec.push_back(h);
      printf("h: (%.2f, %.2f)\n", h.real(), h.imag());
  }

  InnovativePktInfoVector inno_pkts = flowInfo->innovative_pkts;
  unsigned int n_inno_pkts = inno_pkts.size();

  printf("smart_selection_local start, #inno_pkts: %d\n", n_inno_pkts); fflush(stdout);
  int num_carriers = d_data_carriers.size();

  /* randomly choose every coefficient except for the last one and keep reducing to latest value */
  gr_complex reduced_coeffs[MAX_BATCH_SIZE];
  memset(reduced_coeffs, 0, sizeof(gr_complex)*MAX_BATCH_SIZE);

  for(unsigned int i = 0; i < n_inno_pkts-1; i++) {
     //float amp = getAvgAmplificationFactor((inno_pkts[i])->hestimates);
     float amp = abs((inno_pkts[i]->hestimates[0])[d_data_carriers[0]]);

     float phase = (rand() % 360 + 1) * M_PI/180;

     coeffs[i] = (amp * ToPhase_c(phase));
     gr_complex *pkt_coeffs = flowInfo->reduced_coeffs[i];               // reduced coeffs for this inno pkt //
     for(unsigned int k = 0; k < d_batch_size; k++) {
        reduced_coeffs[k] += (coeffs[i] * pkt_coeffs[k*num_carriers]);
	printf("(%f, %f) += (%f, %f) * (%f, %f)\n",
			reduced_coeffs[k].real(), reduced_coeffs[k].imag(), coeffs[i].real(), coeffs[i].imag(),
			pkt_coeffs[k*num_carriers].real(), pkt_coeffs[k*num_carriers].imag()); fflush(stdout);
     }
  }
 
  printf(" -- till here, red_coeffs (%.2f, %.2f) (%.2f, %.2f)\n", reduced_coeffs[0].real(), reduced_coeffs[0].imag(), reduced_coeffs[1].real(), reduced_coeffs[1].imag()); fflush(stdout);

  // tackle the last packet now //
  int last = n_inno_pkts-1;
  gr_complex *pkt_coeffs = flowInfo->reduced_coeffs[last];
  float amp = abs((inno_pkts[last]->hestimates[0])[d_data_carriers[0]]); //getAvgAmplificationFactor((inno_pkts[last])->hestimates);
  //float amp = getAvgAmplificationFactor((inno_pkts[last])->hestimates);

  // to cap the max iterations //
  int max_iter = 1000, iter = 0;
  gr_complex best_coeff;
  CoeffInfo best_cInfo;
  float max_min_dt = 0.0;

  while(1) {
     float phase = (rand() % 360 + 1) * M_PI/180;
     coeffs[last] = (amp * ToPhase_c(phase));

     // ensure its a good selection over all the next-hop rx //
     bool good = 1;
     for(int i = 0; i < rx_ids.size(); i++) {

	cInfo->rx_id = rx_ids[i];

	// include the channel effect for this rx //
	gr_complex t_reduced_coeffs[MAX_BATCH_SIZE];		// temporary working copy
	for(int k = 0; k < d_batch_size; k++) {
	   t_reduced_coeffs[k] = reduced_coeffs[k] + (coeffs[last] * pkt_coeffs[k*num_carriers]);
	   cInfo->coeffs[k] = t_reduced_coeffs[k] * h_vec[i];
	   printf("(%f, %f) = (%f, %f) * (%f, %f) \n", cInfo->coeffs[k].real(), cInfo->coeffs[k].imag(),
							coeffs[last].real(), coeffs[last].imag(),
							pkt_coeffs[k*num_carriers].real(), pkt_coeffs[k*num_carriers].imag());
	   fflush(stdout);
	}	

	if(d_batch_size == 1)
	    break;
	
	float dt = 1000.0;
        if(!is_CV_good(cInfo->coeffs[0], cInfo->coeffs[1], dt)) {
	    printf("cv1 (%f, %f), cv2 (%f, %f), dt: %f\n", cInfo->coeffs[0].real(), cInfo->coeffs[0].imag(),
						cInfo->coeffs[1].real(), cInfo->coeffs[1].imag(), dt); fflush(stdout);
	    good = 0;
	    if(dt > max_min_dt) {
		max_min_dt = dt;
		best_coeff = coeffs[last];
		best_cInfo.coeffs[0] = cInfo->coeffs[0];
		best_cInfo.coeffs[1] = cInfo->coeffs[1];
	    }
            break;
        } 
        else {
	    printf("good! - dt: %f\n", dt); fflush(stdout);
	    max_min_dt = dt;
	}
     }

     iter++;
     if(good || iter == max_iter) {
       printf("good: %d, iter: %d, max_min_dt: %f\n", good, iter, max_min_dt); fflush(stdout);

       if(iter == max_iter) {
          coeffs[last] = best_coeff;
          cInfo->coeffs[0] = best_cInfo.coeffs[0];
          cInfo->coeffs[1] = best_cInfo.coeffs[1]; 
       }
       break;
     }
  }

  for(int k = 0; k < d_batch_size; k++) {
     printf("(%f, %f) \n", coeffs[k].real(), coeffs[k].imag()); fflush(stdout);
  }
  printf("smart selection local end -- \n"); fflush(stdout);
}
#endif

/* more exhaustive now, since it accounts for co-ordinating transmitters, multiple rx (if any) */
inline void
digital_ofdm_frame_sink::smart_selection_global(gr_complex *my_coeffs, CoeffInfo *others_coeffs, FlowInfo *flowInfo) {
  InnovativePktInfoVector inno_pkts = flowInfo->innovative_pkts;
  unsigned int n_inno_pkts = inno_pkts.size();
  printf("smart_selection_global start\n"); fflush(stdout);

  NodeIds rx_ids;
  get_nextHop_rx(rx_ids); 

  // get the predicted value of h for each of the receivers //
  vector<gr_complex> h_vec;
  for(int i = 0; i < rx_ids.size(); i++) {
      NodeId rx_id = rx_ids[i];
      gr_complex h = predictH(d_id, rx_id);         //h-val from me to this rx
      h_vec.push_back(h);
      printf("h: (%.2f, %.2f)\n", h.real(), h.imag());
  }

  int num_carriers = d_data_carriers.size();

  // to cap the max iterations //
  int max_iter = 2000, iter = 0;
  gr_complex best_coeff[10];			    // arbitrary (will not have more than 10 inno pkts)
  assert(n_inno_pkts <= 10);
  float max_min_dt = 0.0;
  bool threshold = 0.8;

  while(1) {
     gr_complex new_coeffs[MAX_BATCH_SIZE];
     for(int i = 0; i < n_inno_pkts; i++) {
        float amp = abs((inno_pkts[i]->hestimates[0])[d_data_carriers[0]]);
        float phase = (rand() % 360 + 1) * M_PI/180;
        my_coeffs[i] = (amp * ToPhase_c(phase));

        gr_complex *pkt_coeffs = flowInfo->reduced_coeffs[i];
        for(unsigned int k = 0; k < d_batch_size; k++)
           new_coeffs[k] += (my_coeffs[i] * pkt_coeffs[k*num_carriers]);
     }

     // now the receiver based stuff //
     float dt = 0.0;;
     for(int i = 0; i < rx_ids.size(); i++) {

        gr_complex *o_coeffs = others_coeffs[i].coeffs;     // other senders coeffs for this rx
        if(others_coeffs[i].rx_id != rx_ids[i]) {
           printf(" others.rx_id: %c, rx_id: %c\n", others_coeffs[i].rx_id, rx_ids[i]); fflush(stdout);
           assert(false);
        }

        // combine with the other sender //    
        for(unsigned int k = 0; k < d_batch_size; k++) {
           new_coeffs[k] = (new_coeffs[k] * h_vec[i]) + o_coeffs[k];
        }
        dt += calc_CV_dt(new_coeffs[0], new_coeffs[1]);
     }
     dt /= ((float) rx_ids.size());

     bool good = (dt >= threshold)?true:false;
     if(!good) {
        if(dt > max_min_dt) {
           max_min_dt = dt;
           for(int k = 0; k < n_inno_pkts; k++)
               best_coeff[k] = my_coeffs[k];
        }
     }
     else max_min_dt = dt;

     iter++;
     if(good || iter == max_iter) {
        printf("good: %d, iter: %d, max_min_dt: %f\n", good, iter, max_min_dt); fflush(stdout);
        for(int k = 0; k < n_inno_pkts; k++)
           my_coeffs[k] = best_coeff[k];
        break;
     }
  } // end while

  printf("smart_selection_global cv: \n"); fflush(stdout);
  for(int k = 0; k < n_inno_pkts; k++) {
     printf("(%f, %f) \n", my_coeffs[k].real(), my_coeffs[k].imag()); fflush(stdout);
  }
}

inline void
digital_ofdm_frame_sink::chooseCV_H(FlowInfo *flowInfo, gr_complex *coeffs) {
  printf("chooseCV_H -- \n"); fflush(stdout);
  assert(d_batch_size <= 2);

  // first check if any outstanding HInfo available //
  vector<unsigned int>:: iterator it = d_h_rx_socks.begin();
  while(it != d_h_rx_socks.end()) {
     check_HInfo_rx_sock(*it);
     it++;
  }

  if(d_fwd_index <=1) {
     CoeffInfo reduced_coeffs[MAX_RX];
     smart_selection_local(coeffs, reduced_coeffs, flowInfo);
     if(d_fwd_index == 1) 
        send_coeff_info_eth(reduced_coeffs);
  } else {
     assert(d_fwd_index == 2);
     CoeffInfo lead_coeffs[MAX_RX];
     get_coeffs_from_lead(lead_coeffs);
     smart_selection_global(coeffs, lead_coeffs, flowInfo);
  }
  printf("chooseCV_H done -- \n"); fflush(stdout);
}

inline HInfo*
digital_ofdm_frame_sink::getHInfo(NodeId tx_id, NodeId rx_id) {
  HKey hkey(tx_id, rx_id);
  HInfoMap::iterator it = d_HInfoMap.find(hkey);
  assert(it != d_HInfoMap.end());
  return (HInfo*) it->second;
}

/* predict the value of H as a fn(slope, samples since last tx) */
inline gr_complex 
digital_ofdm_frame_sink::predictH(NodeId tx_id, NodeId rx_id) {

  // now try and predict H based on interval_samples //
  HInfo *hInfo = getHInfo(tx_id, rx_id);
  gr_complex h_val = hInfo->h_value;				// latest update from downstream //
  int h_pkt_num = hInfo->pkt_num;
  float slope = hInfo->slope;

  if(h_pkt_num == -1 || slope == 0.0)
     return gr_complex(1.0, 0.0);

  // calculate the # of samples since the pkt_num that corresponds to the latest known h_val //
  uhd::time_spec_t h_val_ts = getPktTimestamp(h_pkt_num);
  uhd::time_spec_t interval = d_out_pkt_time - h_val_ts;
  time_t full_secs = interval.get_full_secs();
  double frac_secs = interval.get_frac_secs();
  double total_time = full_secs + frac_secs;

  int decimation = 128;
  double rate = 1.0/decimation;
  double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
  uint64_t interval_samples = total_time/time_per_sample;
  std::cout<<"time/sample: "<<time_per_sample<<" samples: "<<interval_samples<<" total time: "<<total_time<<endl;

  printf("predictH -- pkt_num: %d, slope: %f\n", d_pkt_num, slope); fflush(stdout);

  float old_angle = arg(h_val);
  float new_angle = old_angle + interval_samples * slope;

  printf("old_angle: %.2f, new_angle: %.2f\n", old_angle, new_angle); fflush(stdout);
  while(new_angle > M_PI)
	  new_angle = new_angle - 2*M_PI;

  assert(new_angle <= M_PI && new_angle >= -M_PI);

  printf("predictH: pkt_num: %d, slope: %.8f, samples: %llu, old_angle: %.2f, new_angle: %.2f\n", d_pkt_num, slope, interval_samples, old_angle, new_angle); fflush(stdout);
  //printf("predictH: pkt_num: %d, slope: %f, old_angle: %.2f, new_angle: %.2f\n", d_pkt_num, slope, old_angle, new_angle); fflush(stdout);

  return gr_expj(new_angle);
}

/* opens the h_tx and h_rx sockets 
   figure out the upstream and downstream nodes and open sockets accordingly
*/
inline void
digital_ofdm_frame_sink::prepare_H_coding() {
   populateEthernetAddress();   
   EthInfoMap::iterator it;

   int num_out_links = d_outCLinks.size();
   if(num_out_links > 0) {
      assert(num_out_links == 1);
      /* create 1 rx client socket per downstream nodes to receive from */
      for(int i = 0; i < num_out_links; i++) {
         CompositeLink *link = getCompositeLink(d_outCLinks[i]);
   
         NodeIds dst_ids = link->dstIds;
         for(int j = 0; j < dst_ids.size(); j++) {
	    it = d_ethInfoMap.find(dst_ids[j]);
	    assert(it != d_ethInfoMap.end());
	    EthInfo *ethInfo = (EthInfo*) it->second;
	    int rx_sock = open_client_sock(ethInfo->port, ethInfo->addr, false);

	    //d_h_rx_socks.insert(pair<int, int>(dst_ids[j], rx_sock));		// dstId will send HInfo
	    d_h_rx_socks.push_back(rx_sock);
	 }
      }
   }

   // open a server socket (since I'll be transmitting to upstream nodes). 
   // upstream nodes are the clients that'll transmit to me
   assert(d_inCLinks.size() == 1);
   CompositeLink *link = getCompositeLink(d_inCLinks[0]);
   int num_clients = link->srcIds.size();
   it = d_ethInfoMap.find(d_id);
   assert(it != d_ethInfoMap.end());
   EthInfo *ethInfo = (EthInfo*) it->second;
   open_server_sock(ethInfo->port, d_h_tx_socks, num_clients);		// fill up d_h_tx_socks
   printf("# h_tx_socks: %d\n", d_h_tx_socks.size()); fflush(stdout);

#if 0
   // opens 1 tx port to transmit to 1 upstream node //
   for(int i = 0; i < d_inCLinks.size(); i++) {
      CompositeLink *link = getCompositeLink(d_outCLinks[i]);

      vector<unsigned int> src_ids = link->srcIds;
      for(int j = 0; j < src_ids.size(); j++) {
         it = d_ethInfoMap.find(src_ids[j]);
         assert(it != d_ethInfoMap.end());
	 EthInfo *ethInfo = (EthInfo*) it->second;
         int tx_sock = open_server_sock(ethInfo->port);
         d_h_tx_socks.push_back(tx_sock);
      }
   }
#endif

   printf("opened H sockets for exchanging H information!\n"); fflush(stdout);

   /* create 1 tx socket to send the coeffs over, if lead sender */
   if(d_fwd_index == 1) {
      it = d_ethInfoMap.find(d_id);
      assert(it != d_ethInfoMap.end());
      EthInfo *ethInfo = (EthInfo*) it->second;
      vector<unsigned int> conn_socks;
      open_server_sock(ethInfo->port + 50, conn_socks, 1);		// only 1 client needs to be connected
      d_coeff_tx_sock = conn_socks[0];
   }
   /* else open 1 rx socket to receive from the lead sender */
   else if (d_fwd_index == 2) {
      NodeId co_id = get_coFwd();
      it = d_ethInfoMap.find(co_id);
      assert(it != d_ethInfoMap.end());
      EthInfo *ethInfo = (EthInfo*) it->second;
      d_coeff_rx_sock = open_client_sock(ethInfo->port+50, ethInfo->addr, true);

      /* set a timeout on this - if it expires, then I'll go by myself */
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 8e5;
      assert(setsockopt(d_coeff_rx_sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval))==0);

   }

   initHInfoMap();
}

// called when (1) receive a packet on the air, and (2) receive HInfo packet over ethernet //
inline void
digital_ofdm_frame_sink::updateHInfo(HKey hkey, HInfo _hInfo, bool update_slope) {
   //printf("updateHInfo, size: %d, find(%c, %c)\n", d_HInfoMap.size(), hkey.first, hkey.second); fflush(stdout);
   HInfoMap::iterator it = d_HInfoMap.find(hkey);
   assert(it != d_HInfoMap.end());
   HInfo *hInfo = (HInfo*) it->second;  

   float old_angle = arg(hInfo->h_value);
   float new_angle = arg(_hInfo.h_value);

   if(update_slope) {
      // discard if a repetitive report //
      if(_hInfo.pkt_num == hInfo->pkt_num) {
	  printf("updateHInfo discarded, pkt_num: %d\n", hInfo->pkt_num); fflush(stdout);
          return;
      }


      // calculate the slope, etc only if this is not the first update //
      if(hInfo->pkt_num > -1) {
         uhd::time_spec_t new_ts = getPktTimestamp(_hInfo.pkt_num);
	 uhd::time_spec_t old_ts = getPktTimestamp(hInfo->pkt_num);

	 uhd::time_spec_t interval = new_ts - old_ts;
	 time_t full_secs = interval.get_full_secs();
	 double frac_secs = interval.get_frac_secs();
	 double total_time = full_secs + frac_secs;

	 int decimation = 128;
	 double rate = 1.0/decimation;
	 double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
	 uint64_t interval_samples = total_time/time_per_sample;
	 std::cout<<"time/sample: "<<time_per_sample<<" samples: "<<interval_samples<<" total time: "<<total_time<<endl;

	 printf(" -- old_angle: %.2f, new_angle: %.2f\n", old_angle, new_angle); fflush(stdout);
         if(new_angle < old_angle) {
            new_angle = new_angle + 2*M_PI;
         }
         float diff = new_angle - old_angle;

         float slope = diff/interval_samples;
         if(hInfo->slope > 0.0)
            hInfo->slope = (hInfo->slope + slope)/2.0;
         else
            hInfo->slope = slope;

         printf("-- interval_samples: %d\n", interval_samples); fflush(stdout);
      }
   }

   printf("updateHInfo - (%c, %c) pkt_num: %d, old_pkt: %d, slope: %f, old_angle: %.2f, new_angle: %.2f, timing_off: %f \n", 
		hkey.first, hkey.second, _hInfo.pkt_num, hInfo->pkt_num, hInfo->slope, old_angle, arg(_hInfo.h_value), _hInfo.timing_slope); fflush(stdout);

   // now update the hInfo //
   hInfo->pkt_num = _hInfo.pkt_num;
   hInfo->h_value = _hInfo.h_value;

   // timing slope - update only if received as part of the extract_header, else update only if non-zero
   if(!update_slope)
      hInfo->timing_slope = _hInfo.timing_slope;					// different from below 'slope' - this is within 1 OFDM symbol //
   else {
      //if(hInfo->timing_slope == 0)
	 hInfo->timing_slope = _hInfo.timing_slope;
   }
}

inline uhd::time_spec_t
digital_ofdm_frame_sink::getPktTimestamp(int pkt_num) {
   assert(d_pktTxInfoList.size() > 0);
   PktTxInfoList::iterator it = d_pktTxInfoList.begin();
   while(it != d_pktTxInfoList.end()) {
       PktTxInfo item = *it;
       if(item.first == pkt_num)
           return item.second;
       it++;
   }
   printf("getPktTimestamp failed for pkt_num: %d\n", pkt_num); fflush(stdout);
   assert(false);
}

// initializes the HInfoMap to have keys for all the relevant entries //
inline void
digital_ofdm_frame_sink::initHInfoMap() {
   int num_out_links = d_outCLinks.size(); assert(num_out_links <= 1);
   int num_in_links = d_inCLinks.size(); assert(num_in_links == 1);

   if(num_out_links == 1) {
      CompositeLink *link = getCompositeLink(d_outCLinks[0]);
      NodeIds dstIds = link->dstIds;
      NodeIds::iterator it = dstIds.begin();
      while(it != dstIds.end()) {
	 unsigned char rx_id = *it;
	 HKey hkey(d_id, rx_id);
	 HInfo *hInfo = (HInfo*) malloc(sizeof(HInfo));
	 memset(hInfo, 0, sizeof(HInfo));
	 hInfo->pkt_num = -1;
	 d_HInfoMap.insert(pair<HKey, HInfo*>(hkey, hInfo));
	 it++;
      }
   }
  
   CompositeLink *link = getCompositeLink(d_inCLinks[0]);
   NodeIds srcIds = link->srcIds;
   NodeIds::iterator it = srcIds.begin();
   while(it != srcIds.end()) {
      unsigned char tx_id = *it;
      HKey hkey(tx_id, d_id);
      HInfo *hInfo = (HInfo*) malloc(sizeof(HInfo));
      memset(hInfo, 0, sizeof(HInfo));
      hInfo->pkt_num = -1;
      d_HInfoMap.insert(pair<HKey, HInfo*> (hkey, hInfo));
      it++;
   }
  
   printf("initHInfoMap size: %d\n", d_HInfoMap.size());
}

/* check if any outstanding msgs on HInfo rx sock, if yes, then extract those HInfo 
   structures and insert them into HInfoMap. Upto 2 HInfo can be extracted every time 
*/
inline void
digital_ofdm_frame_sink::check_HInfo_rx_sock(int rx_sock) {

   int buf_size = sizeof(HInfo) + (sizeof(NodeId) * 2);;                // extra byte for the sender's+rx id
   char *_buf = (char*) malloc(buf_size);
   memset(_buf, 0, buf_size);

   int n_extracted = 0; 
   int nbytes = recv(rx_sock, _buf, buf_size, MSG_PEEK);
   NodeId rxId[2];
   if(nbytes > 0) {
      int offset = 0;
      while(1) {
         nbytes = recv(rx_sock, _buf+offset, buf_size-offset, 0);
	 if(nbytes > 0) offset += nbytes;
         if(offset == buf_size) {

            rxId[n_extracted] = _buf[0];                               // copy the sender's id
	    NodeId id = _buf[1];

            // updateHInfo only if it is my H report //
            if(id == d_id) {
               HKey hkey(d_id, rxId[n_extracted]);

               HInfo *hInfo = (HInfo*) malloc(sizeof(HInfo));
               memcpy(hInfo, _buf+2, sizeof(HInfo));                     // copy HInfo

               updateHInfo(hkey, *hInfo, true);
               free(hInfo);
            }
	    else 
	       printf("ignore check_HInfo_rx, d_id: %c, id: %d\n", d_id, id); fflush(stdout);

	    n_extracted++;
	    
	    if(n_extracted == 1) {
       	      // see if there's another packet - I'll try and get only one more, rest later //
	      memset(_buf, 0, buf_size);
              nbytes = recv(rx_sock, _buf, buf_size, MSG_PEEK);
	      if(nbytes > 0) {
		 offset = 0;
	         continue;
	      }
	    }
	    break;
      	 }
      }
      assert(offset == buf_size);
   }

   printf("check_HInfo_rx, nextracted: %d\n", n_extracted); fflush(stdout);
   free(_buf);
} 

// broadcast the HInfo to all the upstream clients, d_h_tx_socks are the clients to be transmitted to //
inline void
digital_ofdm_frame_sink::txHInfo() {
   int num_clients = d_h_tx_socks.size();
   assert(num_clients >= 1);

   int buf_size = (sizeof(HInfo) + 2*sizeof(NodeId))*num_clients;            	// also send (tx-id, rx-id) for each client
   char *_buf = (char*) malloc(buf_size);

   // ids of clients 
   assert(d_inCLinks.size() == 1);                                              // can't envision more than 1 link that i'm part of
   CompositeLink *clink = getCompositeLink(d_inCLinks[0]);
   assert(clink);
   NodeIds tx_ids = clink->srcIds;
   assert(tx_ids.size() == num_clients);

   printf("txHInfo to num_clients: %d, buf_size: %d\n", num_clients, buf_size); fflush(stdout);

   // build the payload
   NodeIds::iterator it = tx_ids.begin();
   int offset = 0;
   while(it != tx_ids.end()) {
       NodeId tx_id = *it;
       HInfo *hInfo = getHInfo(tx_id, d_id);

       // copy my-id first //
       memcpy(_buf+offset, &d_id, 1);	
       offset += 1;

       // copy tx-id now //
       memcpy(_buf+offset, &tx_id, 1);
       offset += 1;

       // copy the hInfo now //
       memcpy(_buf+offset, hInfo, sizeof(HInfo));
       offset += sizeof(HInfo);
       it++;
       printf(" -- HInfo details: pkt: %d, val: (%.2f, %.2f)\n", 
			hInfo->pkt_num, hInfo->h_value.real(), hInfo->h_value.imag()); fflush(stdout);
   }

   // send msgs
   vector<unsigned int>::iterator _it = d_h_tx_socks.begin();
   while(_it != d_h_tx_socks.end()) {
   	int bytes_sent = send(*_it, _buf, buf_size, 0);
        printf("HInfo sent, bytes_sent: %d, errno: %d\n", bytes_sent, errno); fflush(stdout);
	_it++;
   }


#if 0
   // debug
   HInfo _hInfo;// = (HInfo*) malloc(buf_size);
   memcpy(&_hInfo, _buf+1, buf_size-1);
   printf("batch: %d, (%.2f, %.2f), (%.2f)\n", _hInfo.batch_num, _hInfo.h_value.real(), _hInfo.h_value.imag(), _hInfo.slope);
   fflush(stdout);
#endif

   free(_buf);
   printf("txHInfo done to num_clients: %d--- \n", d_h_tx_socks.size()); fflush(stdout);
}

inline void
digital_ofdm_frame_sink::populateEthernetAddress()
{
   // node-id ethernet-addrees port-on-which-it-is-listening //
   printf("populateEthernetAddress\n"); fflush(stdout);
   FILE *fl = fopen ( "eth_add.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
        exit (1);
   }

   char line[128];
   const char *delim = " ";
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token_vec;
        while (strtok_res != NULL) {
           token_vec.push_back(strtok_res);
           //printf ("%s\n", strtok_res);
           strtok_res = strtok (NULL, delim);
        }

	EthInfo *ethInfo = (EthInfo*) malloc(sizeof(EthInfo));
	memset(ethInfo, 0, sizeof(EthInfo));
        NodeId node_id = (NodeId) atoi(token_vec[0]) + '0';
        memcpy(ethInfo->addr, token_vec[1], 16);
	unsigned int port = atoi(token_vec[2]);
	ethInfo->port = port;

        d_ethInfoMap.insert(pair<NodeId, EthInfo*> (node_id, ethInfo));

        std::cout<<"Inserting in the map: node " << node_id << " addr: " << ethInfo->addr << endl;
   }

   fclose (fl);

#if 0
   // debug
   EthInfoMap::iterator it = d_ethInfoMap.begin();
   while(it != d_ethInfoMap.end()) {
      std::cout << " key: " << it->first << " d_id: " << d_id << endl;
      it++;
   } 
#endif
}


/* util function */
inline void
digital_ofdm_frame_sink::open_server_sock(int sock_port, vector<unsigned int>& connected_clients, int num_clients) {
  printf("sink:: open_server_sock start, #clients: %d\n", num_clients); fflush(stdout);
  int sockfd, _sock;
  struct sockaddr_in dest;

  /* create socket */
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  fcntl(sockfd, F_SETFL, O_NONBLOCK);

  /* ensure the socket address can be reused, even after CTRL-C the application */
  int optval = 1;
  int ret = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

  if(ret == -1) {
        printf("@ digital_ofdm_mapper_bcv::setsockopt ERROR\n"); fflush(stdout);
  }

  /* initialize structure dest */
  bzero(&dest, sizeof(dest));
  dest.sin_family = AF_INET;

  /* assign a port number to socket */
  dest.sin_port = htons(sock_port);
  dest.sin_addr.s_addr = INADDR_ANY;

  bind(sockfd, (struct sockaddr*)&dest, sizeof(dest));

  /* make it listen to socket with max 20 connections */
  listen(sockfd, 1);
  assert(sockfd != -1);

  struct sockaddr_in client_addr;
  int addrlen = sizeof(client_addr);

  int conn = 0;
  while(conn != num_clients) {
    _sock = accept(sockfd, (struct sockaddr*)&client_addr, (socklen_t*)&addrlen);
    if(_sock != -1) {
      printf(" -- connected client: %d\n", conn); fflush(stdout);
      connected_clients.push_back(_sock);
      conn++;
    }
  }

  printf("open_server_sock done.. #clients: %d\n", num_clients);
  assert(connected_clients.size() == num_clients);
}

/* util function */
inline int
digital_ofdm_frame_sink::open_client_sock(int port, const char *addr, bool blocking) {
  int sockfd;
  struct sockaddr_in dest;

  /* create socket */
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if(!blocking)
     fcntl(sockfd, F_SETFL, O_NONBLOCK);

  assert(sockfd != -1);

  /* initialize value in dest */
  bzero(&dest, sizeof(struct sockaddr_in));
  dest.sin_family = PF_INET;
  dest.sin_port = htons(port);
  dest.sin_addr.s_addr = inet_addr(addr);
  /* Connecting to server */
  connect(sockfd, (struct sockaddr*)&dest, sizeof(struct sockaddr_in));
  return sockfd;
}

inline void
digital_ofdm_frame_sink::calculate_snr(gr_complex *in) {
  const std::vector<gr_complex> &ks = d_preamble[d_preamble_cnt];

  float err = 0.0;
  float pow = 0.0;

  for(int i=0; i<d_occupied_carriers;i++) {
     //printf("ks: (%f, %f), in: (%f, %f)\n", ks[i].real(), ks[i].imag(), in[i].real(), in[i].imag());
   
     err += norm(ks[i]-in[i]);
     pow += norm(ks[i]);
  }

  printf("calculate_snr: %f\n", 10*log10(pow/err)); fflush(stdout);
}

#if 0
/* perform a combining of 'n' packets from the inno_pkts received 
   also replace these 'n' packets with the resulting packet in inno_pkts */
void
digital_ofdm_frame_sink::mrc_combining(FlowInfo *flowInfo, int n) {
   InnovativePktInfoVector inno_pkts = flow_info->innovative_pkts;
   int n_inno_pkts = inno_pkts.size();
   assert(n_inno_pkts >= n);

   unsigned int n_symbols = d_num_ofdm_symbols * d_occupied_carriers;
   gr_complex *mrc_symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_symbols);
   memset(mrc_symbols, 0, sizeof(gr_complex) * n_symbols);

   for(unsigned int i = 0; i < n; i++) {
      PktInfo *pInfo = inno_pkts[i];
      gr_complex *symbols = pInfo->symbols;
      
      for(unsigned int j = 0; j < n_symbols; j++) {
	 mrc_symbols[j] += (0.5 * symbols[j]);
      }
   }
}
#endif

/* forwarder - attempt to correct the signal before forwarding */
inline void
digital_ofdm_frame_sink::correctSignal(FlowInfo *fInfo) {

  vector<gr_complex> dfe_pilot[MAX_SENDERS];
  for(unsigned int i = 0; i < MAX_SENDERS; i++) {
     dfe_pilot[i].resize(d_pilot_carriers.size());
     fill(dfe_pilot[i].begin(), dfe_pilot[i].end(), gr_complex(1.0,0.0));
  }

  vector<gr_complex*> interpolated_coeffs;
  int num_senders = d_pktInfo->n_senders;
  // coefficients are already complete for each sender on all subcarriers at d_pktInfo->coeffs //
  for(int k = 0; k < num_senders; k++) {
      gr_complex *rx_coeffs = d_pktInfo->coeffs.at(k);
      interpolated_coeffs.push_back(rx_coeffs);
  }

  reset_demapper();

  InnovativePktInfoVector innoPkts = fInfo->innovative_pkts;

  // get the current packet //
  int pkt_index = innoPkts.size() - 1;
  PktInfo *pInfo = innoPkts[pkt_index];
  gr_complex *rx_symbols_this_batch = pInfo->symbols;

  gr_complex *sym_vec = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
  // symbol position map needs to be built each time, because of DFE tracking //

  int n_carriers = d_data_carriers.size();
  int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size));

  unsigned char bits[MAX_BATCH_SIZE];
  gr_complex *sym_position = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);
  memset(d_euclid_dist, 0, sizeof(float) * n_carriers * n_entries * MAX_OFDM_SYMBOLS);		// for now on every packet (rather every batch) //
  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
      gr_complex carrier;
      vector<gr_complex> dfe_data;
      memcpy(sym_vec, &rx_symbols_this_batch[o * d_occupied_carriers], sizeof(gr_complex) * d_occupied_carriers);
      track_pilot_dfe_SRC(sym_vec, pInfo->hestimates, carrier, dfe_pilot[0], o);
      interpolate_data_dfe(dfe_pilot[0], dfe_data, false, NULL, gr_complex(0.0, 0.0));

      gr_complex sigrot, closest_sym[MAX_DATA_CARRIERS];
      gr_complex *actual_symbols = &rx_symbols_this_batch[o * d_occupied_carriers];
      for(unsigned int i = 0; i < n_carriers; i++) {
	  buildMap_pilot_SRC(fInfo, sym_position, interpolated_coeffs, dfe_data, carrier, i, o);
	  sigrot = sym_vec[d_data_carriers[i]];
	  slicer_ILP_2(sigrot, fInfo, closest_sym[i], bits, sym_position, o, i);
	
 	  actual_symbols[d_data_carriers[i]] = closest_sym[i];			// the copy into actual step!! 
      }
      int count = ftell(d_fp_corr_symbols);
      count = fwrite_unlocked(closest_sym, sizeof(gr_complex), n_carriers, d_fp_corr_symbols);
  }

  free(sym_vec);
  free(sym_position);
}

inline void
digital_ofdm_frame_sink::openCorrectSymbolLog() {
  const char *filename = "corrected_rx_symbols.dat";
  int fd;
  if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     assert(false);
  }
  else {
      if((d_fp_corr_symbols = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "rx symbols file cannot be opened\n");
            close(fd);
            assert(false);
      }
  }
}
