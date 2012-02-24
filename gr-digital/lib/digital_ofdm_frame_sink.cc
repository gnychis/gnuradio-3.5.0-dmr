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

#include <digital_crc32.h>

using namespace arma;
using namespace std;

#define VERBOSE 0
static const pmt::pmt_t SYNC_TIME = pmt::pmt_string_to_symbol("sync_time");
int last_noutput_items;

inline void
digital_ofdm_frame_sink::enter_search()
{
  if (VERBOSE)
    fprintf(stderr, "@ enter_search\n");

  d_state = STATE_SYNC_SEARCH;
  d_curr_ofdm_symbol_index = 0;

  // offline analysis //
  d_log_pkt = false;
  d_log_ofdm_index = 0;
}

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

  // Resetting PLL
  d_freq[0] = 0.0;
  d_phase[0] = 0.0;
  fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));

}

inline void
digital_ofdm_frame_sink::reset_demapper() {
  // clear state of demapper
  d_byte_offset = 0;
  d_partial_byte = 0;

  // Resetting PLL
  for(int i = 0; i < d_batch_size; i++) {
     d_freq[i] = 0.0;
     d_phase[i] = 0.0;
  }

  fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));
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
digital_ofdm_frame_sink::extract_header()
{
  d_packet_whitener_offset = 0;                 // apurv++: hardcoded!

  d_pkt_type = d_header.pkt_type;
  d_flow = d_header.flow_id;

  d_packetlen = d_header.packetlen;
  d_batch_number = d_header.batch_number;

  d_nsenders = d_header.nsenders;	
  d_lead_sender = d_header.lead_sender;	

  d_nsenders = 1;				// TESTING - REMOVEEEEEEEEEEEEEEE
  d_lead_sender = 1;				// TESTING - REMOVEEEEEEEEEEEEEEE

  d_pkt_num = d_header.pkt_num;

  printf("\tpkt_num: %d \t\t\t\t batch_num: %d \t\t\t\t len: %d src: %d\n", d_header.pkt_num, d_header.batch_number, d_packetlen, d_header.src_id); fflush(stdout);

  if (VERBOSE)
    fprintf(stderr, " hdr details: (src: %d), (rx: %d), (batch_num: %d), (d_nsenders: %d), (d_packetlen: %d), (d_pkt_type: %d)\n",
                    d_header.src_id, d_header.dst_id, d_batch_number, d_nsenders, d_packetlen, d_header.pkt_type);
}

unsigned char digital_ofdm_frame_sink::slicer(const gr_complex x)
{
  unsigned int table_size = d_sym_value_out.size();
  unsigned int min_index = 0;
  float min_euclid_dist = norm(x - d_sym_position[0]);
  float euclid_dist = 0;

  for (unsigned int j = 1; j < table_size; j++){
    euclid_dist = norm(x - d_sym_position[j]);
    if (euclid_dist < min_euclid_dist){
      min_euclid_dist = euclid_dist;
      min_index = j;
    }
  }
  return d_sym_value_out[min_index];
}

/* uses pilots - from rawofdm */
void digital_ofdm_frame_sink::equalize_interpolate_dfe(const gr_complex *in, gr_complex *out) 
{
  gr_complex carrier = gr_expj(d_phase[0]);
  gr_complex phase_error = 0.0;

  // update CFO PLL based on pilots
  // TODO: FIXME: 802.11a-1999 p.23 (25) defines p_{0..126v} which is cyclic
  int cur_pilot = 1;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); ++i) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    //gr_complex sigeq = in[di] * carrier * d_dfe[i];
    phase_error += in[di] * conj(pilot_sym);
  }

  // update phase equalizer
  float angle = arg(phase_error);
  d_freq[0] = d_freq[0] - d_freq_gain*angle;
  d_phase[0] = d_phase[0] + d_freq[0] - d_phase_gain*angle;
  if (d_phase[0] >= 2*M_PI) d_phase[0] -= 2*M_PI;
  else if (d_phase[0] <0) d_phase[0] += 2*M_PI;

  carrier = gr_expj(-angle);


  // update DFE based on pilots
  cur_pilot = 1;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); ++i) {
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


  float denom = 1.0 / (pilot_carrier_hi - pilot_carrier_lo);
  // TODO: create a map?
  // carrier index -> (low index, low weight, high index, high weight)
  for (unsigned int i = 0; i < d_data_carriers.size(); ++i) {
    int di = d_data_carriers[i];
    if (di > pilot_carrier_hi) {
      ++pilot_index;
      pilot_carrier_lo = pilot_carrier_hi;
      pilot_dfe_lo = pilot_dfe_hi;
      if (pilot_index < d_pilot_carriers.size()) {
        pilot_carrier_hi = d_pilot_carriers[pilot_index];
        pilot_dfe_hi = d_dfe[pilot_index];
      } else {
        pilot_carrier_hi = d_occupied_carriers;
      }
      denom = 1.0 / (pilot_carrier_hi - pilot_carrier_lo);
    }

    // smarter interpolation would be linear in polar coords (slerp!)
    float alpha = float(pilot_carrier_hi - di) * denom;
    gr_complex dfe = pilot_dfe_hi + alpha * (pilot_dfe_lo - pilot_dfe_hi);
    gr_complex sigeq = in[di] * carrier * dfe;

    out[i] = sigeq;
  }
}

#ifdef USE_PILOT
// apurv++ comment: demaps an entire OFDM symbol in one go //
unsigned int digital_ofdm_frame_sink::demapper(const gr_complex *in,
                                          unsigned char *out)
{

  gr_complex *rot_out = (gr_complex*) malloc(sizeof(gr_complex) * d_data_carriers.size());		// only the data carriers //
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

      unsigned char bits = slicer(rot_out[i]);

      i++;
      if((8 - d_byte_offset) >= d_nbits) {
        d_partial_byte |= bits << (d_byte_offset);
        d_byte_offset += d_nbits;
      }
      else {
        d_nresid = d_nbits-(8-d_byte_offset);
        int mask = ((1<<(8-d_byte_offset))-1);
        d_partial_byte |= (bits & mask) << d_byte_offset;
        d_resid = bits >> (8-d_byte_offset);
        d_byte_offset += (d_nbits - d_nresid);
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
#else 		/* DEFAULT version (without pilots) */
unsigned int digital_ofdm_frame_sink::demapper(const gr_complex *in,
                                          unsigned char *out)
{
  unsigned int i=0, bytes_produced=0;
  gr_complex carrier;

  carrier=gr_expj(d_phase[0]);

  gr_complex accum_error = 0.0;

  while(i < d_data_carriers.size()) {
    if(d_nresid > 0) {
      d_partial_byte |= d_resid;
      d_byte_offset += d_nresid;
      d_nresid = 0;
      d_resid = 0;
    }

    while((d_byte_offset < 8) && (i < d_data_carriers.size())) {
      gr_complex sigrot = in[d_data_carriers[i]]*carrier*d_dfe[i];

      if(d_derotated_output != NULL){
        d_derotated_output[i] = sigrot;
      }

      unsigned char bits = slicer(sigrot);

      gr_complex closest_sym = d_sym_position[bits];

      accum_error += sigrot * conj(closest_sym);

      // FIX THE FOLLOWING STATEMENT
      if (norm(sigrot)> 0.001) d_dfe[i] +=  d_eq_gain*(closest_sym/sigrot-d_dfe[i]);

      i++;
      if((8 - d_byte_offset) >= d_nbits) {
        d_partial_byte |= bits << (d_byte_offset);
        d_byte_offset += d_nbits;
      }
      else {
        d_nresid = d_nbits-(8-d_byte_offset);
        int mask = ((1<<(8-d_byte_offset))-1);
        d_partial_byte |= (bits & mask) << d_byte_offset;
        d_resid = bits >> (8-d_byte_offset);
        d_byte_offset += (d_nbits - d_nresid);
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

  float angle = arg(accum_error);

  d_freq[0] = d_freq[0] - d_freq_gain*angle;
  d_phase[0] = d_phase[0] + d_freq[0] - d_phase_gain*angle;
  if (d_phase[0] >= 2*M_PI) d_phase[0] -= 2*M_PI;
  if (d_phase[0] <0) d_phase[0] += 2*M_PI;

  return bytes_produced;
}
#endif

digital_ofdm_frame_sink_sptr
digital_make_ofdm_frame_sink(const std::vector<gr_complex> &sym_position,
                        const std::vector<unsigned char> &sym_value_out,
                        gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
			unsigned int occupied_carriers, unsigned int fft_length,
                        float phase_gain, float freq_gain, unsigned int id, 
			unsigned int batch_size, unsigned int decode_flag)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_frame_sink(sym_position, sym_value_out,
                                                        target_queue, fwd_queue,
							occupied_carriers, fft_length,
                                                        phase_gain, freq_gain, id,
							batch_size, decode_flag));
}


digital_ofdm_frame_sink::digital_ofdm_frame_sink(const std::vector<gr_complex> &sym_position,
                                       const std::vector<unsigned char> &sym_value_out,
                                       gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
				       unsigned int occupied_carriers, unsigned int fft_length,
                                       float phase_gain, float freq_gain, unsigned int id,
				       unsigned int batch_size, unsigned int decode_flag)
  : gr_sync_block ("ofdm_frame_sink",
                   //gr_make_io_signature2 (2, 2, sizeof(gr_complex)*occupied_carriers, sizeof(char)),  // apurv--
                   gr_make_io_signature4 (2, 4, sizeof(gr_complex)*occupied_carriers, sizeof(char), sizeof(gr_complex)*occupied_carriers, sizeof(gr_complex)*fft_length), //apurv++
                   gr_make_io_signature (1, 1, sizeof(gr_complex)*occupied_carriers)),
    d_target_queue(target_queue), d_occupied_carriers(occupied_carriers),
    d_byte_offset(0), d_partial_byte(0),
    d_resid(0), d_nresid(0),d_phase_gain(phase_gain),d_freq_gain(freq_gain),
    d_eq_gain(0.05), 
    d_batch_size(batch_size),
    d_decode_flag(decode_flag),
    d_active_batch(-1),
    d_last_batch_acked(-1),
    d_pkt_num(0),
    d_id(id),
    d_fft_length(fft_length),
    d_out_queue(fwd_queue)
{
#ifdef USE_PILOT
  std::string carriers = "FE7F";                //2-DC subcarriers      // apurv--
#else
  std::string carriers = "F00F";                //8-DC subcarriers      // apurv++
#endif

  // A bit hacky to fill out carriers to occupied_carriers length
  int diff = (d_occupied_carriers - 4*carriers.length());
  while(diff > 7) {
    carriers.insert(0, "f");
    carriers.insert(carriers.length(), "f");
    diff -= 8;
  }

  // if there's extras left to be processed
  // divide remaining to put on either side of current map
  // all of this is done to stick with the concept of a carrier map string that
  // can be later passed by the user, even though it'd be cleaner to just do this
  // on the carrier map itself
  int diff_left=0;
  int diff_right=0;

  // dictionary to convert from integers to ascii hex representation
  char abc[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
  if(diff > 0) {
    char c[2] = {0,0};

    diff_left = (int)ceil((float)diff/2.0f);  // number of carriers to put on the left side
    c[0] = abc[(1 << diff_left) - 1];         // convert to bits and move to ASCI integer
    carriers.insert(0, c);

    diff_right = diff - diff_left;            // number of carriers to put on the right side
    c[0] = abc[0xF^((1 << diff_right) - 1)];  // convert to bits and move to ASCI integer
    carriers.insert(carriers.length(), c);
  }
#ifdef USE_PILOT
  /* pilot configuration */
  int num_pilots = 6;
  unsigned int pilot_index = 0;                      // tracks the # of pilots carriers added   
  unsigned int data_index = 0;                       // tracks the # of data carriers added
  unsigned int count = 0;                            // tracks the total # of carriers added
  unsigned int pilot_gap = 12;
  unsigned int start_offset = 5;
#endif

  // It seemed like such a good idea at the time...
  // because we are only dealing with the occupied_carriers
  // at this point, the diff_left in the following compensates
  // for any offset from the 0th carrier introduced
  unsigned int i,j,k;
  for(i = 0; i < (d_occupied_carriers/4)+diff_left; i++) {
    char c = carriers[i];
    for(j = 0; j < 4; j++) {
      k = (strtol(&c, NULL, 16) >> (3-j)) & 0x1;
      if(k) {

	int carrier_index = 4*i + j - diff_left;
#ifdef USE_PILOT
        // check if it should be pilot, else add as data //
        if(count == (start_offset + (pilot_index * pilot_gap))) {               // check notes below !
           d_pilot_carriers.push_back(carrier_index);
           pilot_index++;
        }
        else {
           d_data_carriers.push_back(carrier_index);  // use this subcarrier
           data_index++;
        }
        count++;
#else
        d_data_carriers.push_back(carrier_index);
#endif
      }
    }
  }

#ifdef USE_PILOT
  assert(pilot_index + data_index == count);

  /* debug carriers */
  printf("pilot carriers: \n");
  for(int i = 0; i < d_pilot_carriers.size(); i++) {
     printf("%d ", d_pilot_carriers[i]); fflush(stdout);
  }
  printf("\n");
  assert(d_pilot_carriers.size() == pilot_index);

  printf("data carriers: \n");
  for(int i = 0; i < d_data_carriers.size(); i++) {
     printf("%d ", d_data_carriers[i]); fflush(stdout);
  }
  printf("\n");
  assert(d_data_carriers.size() == data_index);

  // hack the above to populate the d_pilots_carriers map and remove the corresponding entries from d_data_carriers //
  /* if # of data carriers = 72
        # of pilots        = 6
        gap b/w pilots     = 72/6 = 12
        start pilot        = 5
        other pilots       = 5, 17, 29, .. so on
        P.S: DC tones should not be pilots!
  */
   d_dfe.resize(d_pilot_carriers.size());
#else
   d_dfe.resize(occupied_carriers);
#endif
   fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));


  // make sure we stay in the limit currently imposed by the occupied_carriers
  if(d_data_carriers.size() > d_occupied_carriers) {
    throw std::invalid_argument("digital_ofdm_mapper_bcv: subcarriers allocated exceeds size of occupied carriers");
  }

  d_bytes_out = new unsigned char[d_occupied_carriers];
  set_sym_value_out(sym_position, sym_value_out);

  enter_search();

  //apurv check //
  printf("HEADERBYTELEN: %d d_data_carriers.size(): %d\n", HEADERBYTELEN, d_data_carriers.size());

  fflush(stdout);
  assert((HEADERBYTELEN * 8) % d_data_carriers.size() == 0);           // assuming bpsk

  // apurv- for offline analysis/logging //
  d_file_opened = false;
  d_fp_coeff = NULL;
  d_fp_solved = NULL;
  d_fp_rx_symbols = NULL; 
  d_fp_hestimates = NULL;

  d_in_estimates = (gr_complex*) malloc(sizeof(gr_complex) * occupied_carriers);
  memset(d_in_estimates, 0, sizeof(gr_complex) * occupied_carriers);  

  assert(open_hestimates_log());

  d_log_pkt = false;
  d_log_ofdm_index = 0;
  d_fp_sampler = NULL;
  d_fp_timing = NULL;
  assert(open_sampler_log());
  pkt_symbols = (gr_complex*) malloc(sizeof(gr_complex) * 50 *d_fft_length);
  memset(pkt_symbols, 0, sizeof(gr_complex) * 50 *d_fft_length);
  timing_symbols = (char*) malloc(sizeof(char) * 50 * d_fft_length);
  memset(timing_symbols, 0, sizeof(char) * 50 * d_fft_length);

  d_save_flag = false;

  populateCreditInfo();
  num_acks_sent = 0;
  num_data_fwd = 0;

  assert(openLogSymbols());

  d_demod_log_file = false;
  d_fp_demod = NULL;

  memset(d_phase, 0, sizeof(float) * MAX_BATCH_SIZE);
  memset(d_freq, 0, sizeof(float) * MAX_BATCH_SIZE);
}

void
digital_ofdm_frame_sink::fill_all_carriers_map() {
  d_all_carriers.resize(d_occupied_carriers);

  unsigned int left_half_fft_guard = (d_fft_length - d_occupied_carriers)/2;

  unsigned int p = 0, d = 0, dc = 0;

  for(unsigned int i = 0; i < d_occupied_carriers; i++) {
      int carrier_index = left_half_fft_guard + i;
	
      if(d_data_carriers[d] == carrier_index) {
	 d_all_carriers.push_back(0);
	 d++;
      } 
      else if(d_pilot_carriers[p] == carrier_index) {
	 d_all_carriers.push_back(1);
	 p++;
      }
      else {
	 d_all_carriers.push_back(2);
	 dc++;
      }      
  }

  assert(d == d_data_carriers.size());
  assert(p == d_pilot_carriers.size());
  assert(dc == d_occupied_carriers - d_data_carriers.size() - d_pilot_carriers.size());
}

digital_ofdm_frame_sink::~digital_ofdm_frame_sink ()
{
  delete [] d_bytes_out;
}

bool
digital_ofdm_frame_sink::set_sym_value_out(const std::vector<gr_complex> &sym_position,
                                      const std::vector<unsigned char> &sym_value_out)
{
  if (sym_position.size() != sym_value_out.size())
    return false;

  if (sym_position.size()<1)
    return false;

  d_sym_position  = sym_position;
  d_sym_value_out = sym_value_out;
  d_nbits = (unsigned int)ceil(log10(float(d_sym_value_out.size())) / log10((float)2.0));		// I've got no idea!!!!!
  printf("size: %d, d_nbits: %d\n", d_sym_value_out.size(), d_nbits); fflush(stdout);

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

  /* optional: dump derotated output */
  if(output_items.size() >= 1)
    d_derotated_output = (gr_complex *)output_items[0];
  else
    d_derotated_output = NULL;

  if (VERBOSE)
    fprintf(stderr,">>> Entering state machine, d_state: %d\n", d_state);

  unsigned int bytes=0;
  switch(d_state) {

  case STATE_SYNC_SEARCH:    // Look for flag indicating beginning of pkt
    if (VERBOSE)
      fprintf(stderr,"SYNC Search, noutput=%d\n", noutput_items);

    if (sig[0]) {  // Found it, set up for header decode
      enter_have_sync();
      if(input_items.size() >= 4) {
	  printf("log_pkt: true\n"); fflush(stdout);
	    /* offline analysis start */
	    d_log_pkt = true;
	    d_log_ofdm_index = 1;
	    memcpy(pkt_symbols, in_sampler, sizeof(gr_complex) * d_fft_length);			//preamble
	    memset(timing_symbols, 0, sizeof(char) * d_fft_length);
	    timing_symbols[0]  = 1;
	    //memcpy(timing_symbols, sig, sizeof(char) * d_fft_length);
	    /* offline analysis dump end */
      }
    }
    break;      // don't demodulate the preamble, so break!

  case STATE_HAVE_SYNC:
    //printf("STATE_HAVE_SYNC\n"); fflush(stdout);

    /* apurv++: equalize hdr */
    if(use_estimates) {
        equalizeSymbols(&in[0], &in_estimates[0]);
        memcpy(d_in_estimates, in_estimates, sizeof(gr_complex) * d_occupied_carriers);	// use in HAVE_HEADER state //
    }

    /* offline dump */
    if(d_log_pkt) {
	memcpy(pkt_symbols + (d_log_ofdm_index*d_fft_length), in_sampler, sizeof(gr_complex) * d_fft_length);	// hdr
	d_log_ofdm_index++;
    } 

    // only demod after getting the preamble signal; otherwise, the 
    // equalizer taps will screw with the PLL performance
    bytes = demapper(&in[0], d_bytes_out);      // demap one ofdm symbol
                                                // PS: header is integer number of ofdm symbols
    if (VERBOSE && sig[0])
	printf("ERROR -- Found SYNC in HAVE_SYNC\n");

    /* add the demodulated bytes to header_bytes */
    memcpy(d_header_bytes+d_hdr_byte_offset, d_bytes_out, bytes);
    d_hdr_byte_offset += bytes;

    //printf("hdrbytelen: %d, hdrdatalen: %d, d_hdr_byte_offset: %d, bytes: %d\n", HEADERBYTELEN, HEADERDATALEN, d_hdr_byte_offset, bytes); fflush(stdout);

    /* header processing */

    /* check if ACK header 
    if (d_hdr_byte_offset == ACK_HEADERBYTELEN) {
	if (ack_header_ok()) {
	   printf("ACK HEADER OK--\n"); fflush(stdout);
	   extract_ack_header(); 
	   processACK();
	   enter_search();
	   break;
	}
	else {
	   memset(&d_ack_header, 0, sizeof(d_ack_header));
	}
    }
	*/
    if (d_hdr_byte_offset == HEADERBYTELEN) {
        if (header_ok()) {						 // full header received

	  /* log hestimates */
	  if(use_estimates) {
	      int count = ftell(d_fp_hestimates);
	      count = fwrite_unlocked(&in_estimates[0], sizeof(gr_complex), d_occupied_carriers, d_fp_hestimates);
   	  }

          printf("HEADER OK--\n"); fflush(stdout);
          extract_header();						// fills local fields with header info

	
          assert(d_nsenders > 0);

	  /* if the batch is already full rank, no need to process the packet any further */
	  FlowInfo *flowInfo = getFlowInfo(false, d_flow);
	  if(flowInfo && flowInfo->active_batch == d_header.batch_number && isFullRank(flowInfo)) {
		printf("batch%d is already full rank, discard pkt\n", flowInfo->active_batch); 
		fflush(stdout);
	        enter_search();
		break;
	  }


	  /* first pkt in session should always be from the lead sender */
	  if(d_lead_sender == 1 && d_pkt_type == DATA_TYPE) 
	  {
		printf("lead sender: %d, senders: %d!\n", d_header.src_id, d_nsenders); fflush(stdout);
	  	if(!amDest_or_Fwder()) {
                   printf("neither destination nor forwarder, ignore!\n"); fflush(stdout);
                   enter_search();
                   break;
	      	} 	

	        assert(d_fwd || d_dst);	

		//printf("d_dst: %d\n", d_dst); fflush(stdout);
	        /* stale batch ? */
        	if(d_header.batch_number < d_active_batch) {
                    printf("STALE BATCH %d --\n", d_header.batch_number); fflush(stdout);
		    enter_search();
		    break;
	        } 
	  	else if(d_header.batch_number == d_last_batch_acked)
	  	{
		    printf("BATCH %d already ACKed --\n", d_header.batch_number); fflush(stdout);
		    enter_search();
		    break;
		}
		else if(d_header.batch_number > d_active_batch)
  	  	    prepareForNewBatch();				// ensure the FlowInfo is in place

		/* interested in pkt! create a new PktInfo for the flow */
		d_pktInfo = createPktInfo();
		d_pending_senders = d_nsenders;
		d_save_flag = true;         				// indicates that a session is on, e'one will be saved (haha)
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

		if(isInnovative()) {					// last header: innovative check
                     d_state = STATE_HAVE_HEADER;
		     d_curr_ofdm_symbol_index = 0;
		     d_num_ofdm_symbols = ceil(((float) (d_packetlen * 8))/(d_data_carriers.size() * d_nbits));
		     printf("d_num_ofdm_symbols: %d, actual sc size: %d, d_nbits: %d\n", d_num_ofdm_symbols, d_data_carriers.size(), d_nbits);
		     fflush(stdout);

		    /* rx symbols */
		     d_pktInfo->symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_num_ofdm_symbols * d_occupied_carriers);
		     memset(d_pktInfo->symbols, 0, sizeof(gr_complex) * d_num_ofdm_symbols * d_occupied_carriers);
                }
	 	else {
		     resetPktInfo(d_pktInfo);				// non-innovative; screw it
		     FlowInfo *flowInfo = getFlowInfo(false, d_flow);
		     assert(flowInfo);
		     flowInfo->innovative_pkts.pop_back(); 
		     free(d_pktInfo);

		     enter_search();
		     break;
		}	
	     } else {
		assert(d_pending_senders > 1);				// more senders remain - switch to preamble look-up
		d_pending_senders--;
		enter_search();
	     }
	  } else {
	     enter_search(); 						// don't care
	     break;
	  } 
        }
        else {
          printf("HEADER NOT OK --\n"); fflush(stdout);
          enter_search();           					// bad header
        }
    }
    break;

  case STATE_HAVE_HEADER:						// process the frame after header
    //printf("STATE_HAVE_HEADER\n"); fflush(stdout);
    if (sig[0]) 
        printf("ERROR -- Found SYNC in HAVE_HEADER, length of %d\n", d_packetlen);

    if(d_nsenders == 1 && use_estimates) {	
       //printf("equalizing payload\n"); fflush(stdout);
       equalizeSymbols(&in[0], &d_in_estimates[0]);
    }

    if(d_curr_ofdm_symbol_index >= d_num_ofdm_symbols) {
	assert(false);
        enter_search();         					// something went crazy
        break;
    }

    assert(d_curr_ofdm_symbol_index < d_num_ofdm_symbols);
    assert(d_pending_senders == 0);

    
    for(unsigned int i = 0; i < d_occupied_carriers; i++)
	in[i] *= gr_complex(d_batch_size);				// de-normalize
    

    storePayload(&in[0], in_sampler);
    d_curr_ofdm_symbol_index++;

    if(VERBOSE) {
       printf("d_curr_ofdm_symbol_index: %d, d_num_ofdm_symbols: %d\n", d_curr_ofdm_symbol_index, d_num_ofdm_symbols);
       fflush(stdout);
    }

    if(d_curr_ofdm_symbol_index == d_num_ofdm_symbols) {		// last ofdm symbol in pkt
 	FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  	assert(flowInfo != NULL);

        if(isFullRank(flowInfo) && d_dst)
	{
#ifdef USE_ILP
	    if(d_batch_size == 1)
		decodePayload_single(flowInfo);
	    else
	        demodulate_ILP(flowInfo);
#else
	    if(d_nsenders == 1)
                decodePayload_single(flowInfo);
	    else
		decodePayload_multiple(flowInfo);
#endif
	}
	if(d_fwd) 
	{
	    printf("check credit: fwd\n"); fflush(stdout);
	    // increment credit counter for the incoming pkt (flow based) //
	    float credit = updateCredit();
	    printf("credit after update: %f\n", credit);
	    makePacket();                                               // REMOVEEEEEEEEEEEE
	}

	
	// offline analysis //
 	if(d_log_pkt) {
	  if(d_log_ofdm_index != d_num_ofdm_symbols+5) {
		printf("d_log_ofdm_index: %d, d_num_ofdm_symbols+4: %d\n", d_log_ofdm_index, d_num_ofdm_symbols+4); 
		fflush(stdout); 
		assert(false);
	  }
	  int count1 = ftell(d_fp_sampler);
          count1 = fwrite_unlocked(pkt_symbols, sizeof(gr_complex), d_log_ofdm_index*d_fft_length, d_fp_sampler);
	  memset(pkt_symbols, 0, sizeof(gr_complex) * d_log_ofdm_index*d_fft_length);

          count1 = ftell(d_fp_timing);
          count1 = fwrite_unlocked(timing_symbols, sizeof(char), d_log_ofdm_index*d_fft_length, d_fp_timing);
          memset(timing_symbols, 0, sizeof(char) * d_log_ofdm_index*d_fft_length);
        }
	enter_search();     						// current pkt done, next packet
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

/* check if destination is my id */
bool
digital_ofdm_frame_sink::isMyPacket()
{
   return (d_id == d_header.dst_id);
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
  dewhiten(HEADERBYTELEN);
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
digital_ofdm_frame_sink::dewhiten(const int len)
{
  for(int i = 0; i < len; i++)
      d_header_bytes[i] = d_header_bytes[i] ^ random_mask_tuple[i];
}

/* PS: if it is innovative, only then will be called, else this row in the coeff_mat will be overwritten! */
bool
digital_ofdm_frame_sink::isInnovative()
{
  //printf("isInnovative start\n"); fflush(stdout);
  assert(d_nsenders >= 1);

  // if multiple senders are involved, then the coeff/hest combinations must be reduced first //
  bool multiple_senders = (d_nsenders > 1);
  if(multiple_senders) {
	bool res = isInnovativeAfterReduction();
	return res;
  }

  // for single sender only //
  
  FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  assert(flowInfo);
  unsigned int packets_in_batch = flowInfo->innovative_pkts.size();

  unsigned int old_rank = rank(flowInfo->coeff_mat);
  printf("old_rank: %d, packets_in_batch: %d\n", old_rank, packets_in_batch); fflush(stdout);

  /* coefficients in this packet */
  for(unsigned int k = 0; k < d_batch_size; k++)
  {
      printf("[%f] ", ToPhase_f(d_header.coeffs[k])); 
      flowInfo->coeff_mat(packets_in_batch-1, k) = d_header.coeffs[k];
  }
  printf("\n");
  unsigned int curr_rank = rank(flowInfo->coeff_mat);
  printf("curr_rank: %d\n", curr_rank); fflush(stdout);
 
  bool res =  (curr_rank > old_rank);

  /* debug */
  if(res) {
    for(unsigned int k = 0; k < d_batch_size; k++)
    {
        gr_complex t = d_header.coeffs[k];
        float angle_deg = ToPhase_f(t);
        float angle_rad = angle_deg * (M_PI/180);
        printf(" degrees: %f \t\t radians: %f \t\t phase: (%f, %f)\n", angle_deg, angle_rad, t.real(), t.imag());
    }	
  }

  printf("isInnovative end\n"); fflush(stdout);
  return res;
}

bool
digital_ofdm_frame_sink::isFullRank(FlowInfo *flowInfo)
{
  //printf("test fullRank\n"); fflush(stdout);
  bool res = (rank(flowInfo->coeff_mat) == d_batch_size);

  if(res)
      flowInfo->coeff_mat.print();

  printf("batch# %d, isFullRank: %d\n", flowInfo->active_batch, res); fflush(stdout);
  return res;
}


/* creates/assigns a new FlowInfo entry (if needed). 
   clears coeff matrix and innovativePktVector */
void
digital_ofdm_frame_sink::prepareForNewBatch()
{
  printf("prepareForNewBatch, batch: %d\n", d_header.batch_number); fflush(stdout);
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
  flow_info->src = d_header.src_id;
  flow_info->dst = d_header.dst_id;
  flow_info->active_batch = d_header.batch_number;
  flow_info->last_batch_acked = flow_info->active_batch - 1;

  /* clean coeff matrix */
  flow_info->coeff_mat = randu<cx_mat>(d_batch_size, d_batch_size);
  flow_info->coeff_mat.zeros();  

  /* clean innovative pktInfo */
  InnovativePktInfoVector rx_vec = flow_info->innovative_pkts;
  int size = rx_vec.size();
  for(int i = 0; i < size; i++) {
	printf("i: %d size: %d\n", i, size); fflush(stdout);
      PktInfo *pInfo = rx_vec[i];
      resetPktInfo(pInfo);		// clears the symbols as well
      free(pInfo);
  }
printf("here5\n"); fflush(stdout);
  flow_info->innovative_pkts.clear();

  /* clean reduced_info */
  vector<gr_complex*> red_coeffs = flow_info->reduced_coeffs;
  for(unsigned int i = 0; i < red_coeffs.size(); i++) {
      gr_complex* coeff = red_coeffs.at(i);
      free(coeff);	
  }
  flow_info->reduced_coeffs.clear();

  printf("prepareForNewBatch ends, size: %d, %d\n", flow_info->innovative_pkts.size(), d_flowInfoVector.size()); fflush(stdout);
}


/* stores 1 OFDM symbol */
void
digital_ofdm_frame_sink::storePayload(gr_complex *in, gr_complex *in_sampler)
{
  unsigned int offset = d_curr_ofdm_symbol_index * d_occupied_carriers;
  memcpy(d_pktInfo->symbols + offset, in, sizeof(gr_complex) * d_occupied_carriers);

  // offline analysis //
  if(d_log_pkt) {
      memcpy(pkt_symbols + (d_log_ofdm_index*d_fft_length), in_sampler, sizeof(gr_complex) * d_fft_length);   
      d_log_ofdm_index++;
  }
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

// used when multiple senders are present //
inline void
digital_ofdm_frame_sink::updateCoeffMatrix(FlowInfo *flowInfo, unsigned int subcarrier_index)
{
  // every entry in reduced_coeffs holds is an array of gr_complex (d_batch_size * subcarriers)
  // # of entries in reduced_coeffs = # of innovative packets stored 

  assert(d_nsenders > 1);
  
  for(unsigned int i = 0;  i < d_batch_size; i++)
  {
     for(unsigned int j = 0; j < d_batch_size; j++)
     {
	gr_complex *coeffs = flowInfo->reduced_coeffs[i];	// get the reduced_coeffs for this packet
	flowInfo->coeff_mat(i, j) = coeffs[subcarrier_index * d_batch_size + j];  // get the coeffs for this subcarrier
     }		
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

inline void
digital_ofdm_frame_sink::verifySolution(cx_mat TX, cx_mat RX, cx_mat coeff_mat)
{
  for(unsigned int i = 0; i < d_batch_size; i++)
  {
     gr_complex res(0, 0);
     for(unsigned int j = 0; j < d_batch_size; j++)
	res += TX(j, 0) * coeff_mat(i, j);
   
	 
     if(RX(i,0).real() != res.real() || RX(i,0).imag() != res.imag()) {
	printf("RX (%f, %f),  TX (%f, %f)\n", RX(i,0).real(), RX(i,0).imag(), res.real(), res.imag());
     }
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

void
digital_ofdm_frame_sink::decodePayload_multiple(FlowInfo *flowInfo)
{
  //printf("decodePayload\n"); fflush(stdout);

  /* initialize the vector that holds the decoded symbols */
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
	  updateCoeffMatrix(flowInfo, i);

          // solve now //
          TX = solve(flowInfo->coeff_mat, RX);

          // verify //
          //verifySolution(TX, RX);

          saveTxMatrix(TX, o, i, out_sym_vec);
      }
  }
  /* // debug 
  printf("debug combined coeff: \n");
  for(unsigned int i = 0;  i < d_batch_size; i++)
  {
     for(unsigned int j = 0; j < d_batch_size; j++)
     {
        complex<double> coeff = flowInfo->coeff_mat(i, j);
	float phase = arg(coeff);
	float phase_deg = phase * (180/M_PI);
	printf("%f ", phase_deg);
     }
     printf("\n");
  }*/

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

  /* initialize the vector that holds the decoded symbols */
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
  log(flowInfo, out_sym_vec);

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
     reset_demapper();
     //printf("----------------- pkt# %d\n ----------------------- \n", k); fflush(stdout);
     for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
         unsigned int offset = o * d_occupied_carriers;

         memset(bytes_out, 0, sizeof(unsigned char) * d_occupied_carriers);
         unsigned int bytes_decoded = demapper(&out_symbols[offset], bytes_out);

	 log_demod(bytes_out, bytes_decoded, &out_symbols[offset], d_occupied_carriers);
                  
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

  /* currently, an ACK is sent out for every batch, even if its not correctly decoded */
  //sendACK(d_flow, d_batch_number);

  /* clear out_sym_vec */
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

bool
digital_ofdm_frame_sink::open_sampler_log()
{
  // time domain symbols - only for those for which the header is OK //
  const char *filename = "sink_time_domain_symbols.dat";
  int fd;
  if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     return false;
  }
  else {
      if((d_fp_sampler = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "sampler symbols file cannot be opened\n");
            close(fd);
            return false;
      }
  }

  const char *filename1 = "sink_timing.dat";
  if ((fd = open (filename1, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename1);
     return false;
  }
  else {
      if((d_fp_timing = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "sampler symbols file cannot be opened\n");
            close(fd);
            return false;
      }
  }

  return true;
}


bool
digital_ofdm_frame_sink::amDest_or_Fwder() {
   //TODO: code up!

  if(d_id == d_header.dst_id) {
     printf("Destination! d_id: %d, d_header.dst_id: %d\n", d_id, d_header.dst_id); fflush(stdout);
     d_dst = true;
     d_fwd = false;
  }
  else {
     printf("Forwarder! d_id: %d, d_header.dst_id: %d\n", d_id, d_header.dst_id); fflush(stdout);
     d_fwd = true;
     d_dst = false;							
  }

  return true;
}

inline PktInfo*
digital_ofdm_frame_sink::createPktInfo() {
  printf("createPktInfo start\n"); fflush(stdout);
  FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  assert(flowInfo);

  PktInfo *pktInfo = (PktInfo*) malloc(sizeof(PktInfo));
  memset(pktInfo, 0, sizeof(PktInfo));

  pktInfo->n_senders = d_nsenders;

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
  int num_senders = pktInfo->n_senders;

  /* clear all */
  pktInfo->senders.clear();
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
  pktInfo->n_senders = d_nsenders;
}

// for the sender, save the hestimate and the coefficients it included in the hdr //
void
digital_ofdm_frame_sink::save_coefficients()
{
  //printf("save_coefficients start\n"); fflush(stdout);
  unsigned char sender_id = d_header.prev_hop_id;
  assert(sender_id >= 0);
 
  gr_complex *hestimates = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
  memcpy(hestimates, d_in_estimates, sizeof(gr_complex) * d_occupied_carriers);

  gr_complex *coeffs = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size);
  // coeffs are in degrees - convert into appropriate phase shifts //
  for(unsigned int k = 0; k < d_batch_size; k++)
  {
      coeffs[k] = d_header.coeffs[k];
      printf("[%f] (%f, %f) ", ToPhase_f(coeffs[k]), coeffs[k].real(), coeffs[k].imag());
  }
  printf("\n");

  d_pktInfo->senders.push_back(sender_id);
  d_pktInfo->coeffs.push_back(coeffs);
  d_pktInfo->hestimates.push_back(hestimates);
  //printf("save_coefficients end for sender:%d\n", sender_id); fflush(stdout);

  debugPktInfo(d_pktInfo, sender_id);
}


// reduce the coefficients to a simpler form, in case multiple senders are present //

// @D: R1 = (h_b * C_b1 + h_c * C_c1) x P1 + (h_b * C_b2 + h_c * C_c2) x P2
//     R2 = (h_b'*C_b1' + h_c'*C_c1') x P1 + (h_b'*C_b2' + h_c'*C_c2') x P2

bool
digital_ofdm_frame_sink::isInnovativeAfterReduction()
{
  /* only applicable for multiple senders */
  int num_senders = d_pktInfo->n_senders;
  assert(num_senders > 1);

  FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  unsigned int packets_in_batch = flowInfo->innovative_pkts.size();

  /* retrieve flow's coeff matrix */
  cx_mat coeff_mat = flowInfo->coeff_mat;
  unsigned int old_rank = rank(coeff_mat);
  
  // debug 
  /*
  printf("Current FlowInfo --- \n"); fflush(stdout);
  for(int i = 0; i < old_rank+1; i++)
  {
     PktInfo *pInfo = flowInfo->innovative_pkts.at(i);
     debugPktInfo(pInfo);
  }*/

  //printf("isInnovativeAfterReduction, nsenders: %d, pks_in_batch: %d, old_rank: %d\n", num_senders, packets_in_batch, old_rank); fflush(stdout);
  int mid_subcarrier_index = d_data_carriers[d_occupied_carriers/2];                   // TODO: very ad-hoc right now! 
  for(unsigned int i = 0; i < d_batch_size; i++)
  {
      gr_complex reduced_coeff;
      for(int j = 0; j < num_senders; j++)
      {
	  gr_complex *estimates = d_pktInfo->hestimates[j];
	  gr_complex *coeffs = d_pktInfo->coeffs[j];				     	// 'coeffs' has 'batch_size' entries	   		   
	  if(j == 0)
		reduced_coeff = (estimates[mid_subcarrier_index] * coeffs[i]);	    	
		//reduced_coeff = coeffs[i];						// REMOVEEEEEEEE: ONLY FOR TESTING
	  else
		reduced_coeff += (estimates[mid_subcarrier_index] * coeffs[i]);	
		//reduced_coeff += coeffs[i];						// REMOVEEEEEEEE: ONLY FOR TESTING
      }
      coeff_mat(packets_in_batch-1, i) = reduced_coeff;
  }

  flowInfo->coeff_mat = coeff_mat;							// reassign

  /* rank increased ? */
  unsigned int new_rank = rank(coeff_mat);
  printf("isInnovativeAfterReduction, nsenders: %d, pks_in_batch: %d, old_rank: %d, new_rank: %d\n", num_senders, packets_in_batch, old_rank, new_rank); fflush(stdout);
  printf("new_rank: %d\n", new_rank);

  /* yes: save the hestimates of each subcarrier to be used during solve() - also save the coefficients, separately in clean form */
  if(new_rank > old_rank) {

      gr_complex *coeff = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * d_occupied_carriers);
      memset(coeff, 0, sizeof(gr_complex) * d_batch_size * d_occupied_carriers);

      flowInfo->reduced_coeffs.push_back(coeff);

      for(unsigned int i = 0; i < d_occupied_carriers; i++)             // input signature: d_occupied_carriers * hestimates
      {
          gr_complex this_coeff;
          for(unsigned int j = 0; j < d_batch_size; j++)
          {
	      for(int k = 0; k < num_senders; k++)
              {
                   gr_complex *estimates = d_pktInfo->hestimates[k];
                   gr_complex *coeffs = d_pktInfo->coeffs[k];

                   if(k == 0)
                        this_coeff = ((gr_complex(1.0, 0.0)/estimates[i]) * coeffs[j]);		
			//this_coeff = coeffs[j];				// REMOVEEEEEEEEEEEEE
                   else
                        this_coeff += ((gr_complex(1.0, 0.0)/estimates[i]) * coeffs[j]);	
			//this_coeff += coeffs[j];				// REMOVEEEEEEEEEEE

              }

              // each subcarrier will have 'd_batch_size' # of coefficients //
              coeff[i * d_batch_size + j] = this_coeff;
          }
      }

      return true;
  }
  return false;
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

/*
bool
digital_ofdm_frame_sink::isInnovativeAfterReduction1(FlowInfo *flowInfo)
{
  // only applicable for multiple senders 

  unsigned int packets_in_batch = in_rx_sym_vec.size();
  unsigned int old_rank = rank(flowInfo->coeff_mat);

  std::pair<gr_complex*, gr_complex*> sender_details;
  std::map<unsigned char, std::pair<gr_complex*, gr_complex*> >::iterator it;  

  int mid_subcarrier_index = d_data_carriers[d_occupied_carriers/2];			// TODO: very ad-hoc right now! //
  for(unsigned int i = 0; i < d_batch_size; i++) 
  {
      gr_complex reduced_coeff;
      int count = 0;
      for(it = d_sender_details.begin(); it != d_sender_details.end(); it++) 
      {
	  sender_details = it->second;
	  gr_complex *estimates = sender_details.first;
	  gr_complex coeff = ((gr_complex*) sender_details.second)[i];

	  if(count == 0) 
	      reduced_coeff = (estimates[mid_subcarrier_index] * coeff);   // just take the hestimate of the middle subcarrier for now //
	  else 
	      reduced_coeff += (estimates[mid_subcarrier_index] * coeff); 

	  count++;
      }

      flowInfo->coeff_mat(packets_in_batch, i) = reduced_coeff;
  }

  unsigned int new_rank = rank(flowInfo->coeff_mat);
  if(new_rank > old_rank) {
      // need to save the hestimates of each subcarrier to be used during solve() - also save the coefficients, separately in clean form 
	
      gr_complex *reduced_coeff = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * d_occupied_carriers);
      memset(reduced_coeff, 0, sizeof(gr_complex) * d_batch_size * d_occupied_carriers);

      d_reduced_coeffs.push_back(reduced_coeff);

      for(unsigned int i = 0; i < d_occupied_carriers; i++)		// input signature: d_occupied_carriers * hestimates
      {
	  gr_complex this_coeff;
	  for(unsigned int j = 0; j < d_batch_size; j++)
	  {
	      for(it = d_sender_details.begin(); it != d_sender_details.end(); it++) 	
	      {
		   sender_details = it->second;
		   gr_complex *estimates = sender_details.first;
		   gr_complex coeff = ((gr_complex*) sender_details.second)[i];		   

		   if(it == d_sender_details.begin())
			this_coeff = (estimates[i] * coeff);	
		   else
			this_coeff += (estimates[i] * coeff);
	      }

	      // each subcarrier will have 'd_batch_size' # of coefficients //
	      reduced_coeff[i * d_batch_size + j] = this_coeff;	      
	  }
      }
      return true;
  }

  return false;
}
*/

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
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++)
     for(unsigned int j = 0; j < d_occupied_carriers; j++) {
	 unsigned int index = (i*d_occupied_carriers) + j;
	 symbols[index] *= coeff;
     }
 
}

void
digital_ofdm_frame_sink::combineSignal(gr_complex *out, gr_complex* symbols)
{
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++)
     for(unsigned int j = 0; j < d_occupied_carriers; j++) {
         unsigned int index = (i*d_occupied_carriers) + j;
	 out[index] += symbols[index];
     }

}

inline void
digital_ofdm_frame_sink::normalizeSignal(gr_complex* out)
{
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++)
     for(unsigned int j = 0; j < d_occupied_carriers; j++) {
         unsigned int index = (i*d_occupied_carriers) + j;
         out[index] /= gr_complex(d_batch_size);         // FIXME: cannot / by d_batch_size, must / by innovative pkts used in creating this signal!
  }
}

void
digital_ofdm_frame_sink::generateCodeVector(MULTIHOP_HDR_TYPE &header)
{
  for(unsigned int k = 0; k < d_batch_size; k++)
  {
      float cv = rand() % 360 + 1;
      header.coeffs[k] = ToPhase_c(cv);
      printf("[%f] ", cv);
  }
}

bool
digital_ofdm_frame_sink::isLeadSender() {
   //TODO: decide!!!
   return 1;
}

/* handle the 'lead sender' case */
void
digital_ofdm_frame_sink::makeHeader(MULTIHOP_HDR_TYPE &header, unsigned char *header_bytes, FlowInfo *flowInfo)
{
   printf("batch#: %d, makeHeader for pkt: %d, len: %d\n", flowInfo->active_batch, d_pkt_num, d_packetlen); fflush(stdout);
   header.src_id = flowInfo->src; 
   header.dst_id = flowInfo->dst;       
   header.prev_hop_id = d_id;	
   header.batch_number = flowInfo->active_batch;

   header.packetlen = d_packetlen;// - 1;                // -1 for '55' appended (TODO)
   header.nsenders = 1;       			      //TODO: hardcoded!
   header.pkt_type = DATA_TYPE;
   if(isLeadSender()) 
        header.lead_sender = 1;
   else
	header.lead_sender = 0;

   header.pkt_num = flowInfo->pkts_fwded;	      // TODO: used for debugging, need to maintain pkt_num for flow 
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

   for(unsigned int k = 0; k < d_batch_size; k++)
        printf("[%f] ", ToPhase_f(header.coeffs[k]));
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

/* populates the credit info at boot. TODO - complete this */
inline void
digital_ofdm_frame_sink::populateCreditInfo()
{
   CreditInfo *creditInfo = (CreditInfo*) malloc(sizeof(CreditInfo));
   creditInfo->flowId = 0; 
   creditInfo->credit = 0.0;
  
   CompositeLink previousLink;
   previousLink.linkId = 0;
   previousLink.srcIds.push_back(0);
   previousLink.dstIds.push_back(d_id);

   CompositeLink nextLink;
   nextLink.linkId = 1;
   nextLink.srcIds.push_back(d_id);
   nextLink.dstIds.push_back(2);  

   memcpy(&(creditInfo->previousLink), &previousLink, sizeof(CompositeLink));
   memcpy(&(creditInfo->nextLink), &nextLink, sizeof(CompositeLink));

   d_creditInfoVector.push_back(creditInfo);
}

inline float
digital_ofdm_frame_sink::updateCredit()
{
   int size = d_creditInfoVector.size();
   assert(size > 0);
   CreditInfo* creditInfo = NULL;

   /* find creditInfo entry */
   for(int i = 0; i < size; i++) {
      creditInfo = d_creditInfoVector[i];
      //printf("credit flow: %d, d_flow: %d\n", creditInfo->flowId, d_flow); fflush(stdout);
      if(creditInfo->flowId == d_flow) {
	 CompositeLink link = creditInfo->previousLink;
	 assert(d_pktInfo && d_pktInfo->senders.size() > 0);
	 if(isSameNodeList(link.srcIds, d_pktInfo->senders)) {
		break;
	 }
      }
   }

   assert(creditInfo);							// the entire credit table must have been created at boot time

   creditInfo->credit += 1.0;   
   return creditInfo->credit;
}

/* called from the outside directly by the wrapper, asking for any pkt to forward */
void
digital_ofdm_frame_sink::makePacket()
{
   /* check if any ACKs need to be sent out first */
   int count = d_ack_queue.size();
   if(count > 0) {// && num_acks_sent == 0) {				
	printf("sink::makePacket - ACK to send count: %d\n", count); fflush(stdout); 
	gr_message_sptr out_msg = d_ack_queue.front();
	d_out_queue->insert_tail(out_msg);
	d_ack_queue.pop_front();
	out_msg.reset();
	num_acks_sent++;
	return;	
   } 

  
   /* check if any data needs to be sent out */
   /* credit check*/
   count = d_creditInfoVector.size();
   CreditInfo* creditInfo = NULL;
   bool found = false;
   if(count > 0) {
	for(int i = 0; i < count; i++) {
	   creditInfo = d_creditInfoVector[i];
	   if(creditInfo->credit >= 1.0) {
	 	found = true;
		break;
	   }
	}
   } 

   /* credit >= 1.0 found, packet needs to be created! */
   if(found){// && num_data_fwd == 0) {
	assert(creditInfo);
	//printf("sink::makePacket - DATA to send\n"); fflush(stdout);

        /* create the pkt now! */
        FlowInfo *flowInfo = getFlowInfo(false, creditInfo->flowId);
        assert(flowInfo);

	encodePktToFwd(creditInfo->flowId);
	creditInfo->credit -= 1.0;				
	//printf("sink::makePacket - DATA end\n"); fflush(stdout);
	num_data_fwd++;
	return;
   }
   else { 
	/* insert a simple EOF msg indicating that no pkts to send for now 
	gr_message_sptr out_msg = gr_make_message(2, 0, 0, 0);			// type = 2, ie EOF
	d_out_queue->insert_tail(out_msg);
	out_msg.reset(); */
   }
   //printf("sink::makePacket - nothing to send\n"); fflush(stdout);
}

void
digital_ofdm_frame_sink::printSymbols(gr_complex* symbols, int num) {
  printf("print symbols-- \n");
  for(int j = 0; j < num; j++) {
     printf("(%.3f, %.3f)\n", symbols[j].real(), symbols[j].imag()); fflush(stdout);
  }
  printf("print symbols end --\n"); fflush(stdout);
}

void
digital_ofdm_frame_sink::encodePktToFwd(unsigned char flowId)
{
  printf("encodePktToFwd, flowId: %d\n", flowId); fflush(stdout);

  /* get FlowInfo */
  FlowInfo *flow_info = getFlowInfo(false, flowId);
  assert((flow_info != NULL) && (flow_info->flowId == flowId));
  InnovativePktInfoVector inno_pkts = flow_info->innovative_pkts;

  /* coefficients */
  unsigned int n_innovative_pkts = inno_pkts.size();
  printf("n_innovative_pkts: %d\n", n_innovative_pkts); fflush(stdout);
  assert(n_innovative_pkts > 0);
  gr_complex *coeffs = (gr_complex*) malloc(sizeof(gr_complex) * n_innovative_pkts);
  memset(coeffs, 0, sizeof(gr_complex) * n_innovative_pkts);

  //printf("final coded symbols\n");  fflush(stdout);

  /* final coded symbols */
  unsigned int n_symbols = d_num_ofdm_symbols * d_occupied_carriers;

  gr_complex *out_symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_symbols);
  memset(out_symbols, 0, sizeof(gr_complex) * n_symbols);

  /* pick new random <coeffs> for each innovative pkt to == new coded pkt */
  for(unsigned int i = 0; i < n_innovative_pkts; i++) {
     PktInfo *pInfo = inno_pkts[i];

     //printf("printSymbols for inno_pkt#%d\n", i); fflush(stdout);
     //printSymbols(pInfo->symbols, n_symbols);
     gr_complex *symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_symbols);
     memcpy(symbols, pInfo->symbols, sizeof(gr_complex) * n_symbols);

     float cv = rand() % 360 + 1;
     //float cv = 0;
     coeffs[i] = ToPhase_c(cv);
     printf("[%f] ", cv); fflush(stdout);
     encodeSignal(symbols, coeffs[i]);

     
     gr_complex *tmp_symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_symbols);
     memcpy(tmp_symbols, out_symbols, sizeof(gr_complex) * n_symbols);

     combineSignal(out_symbols, symbols);

     /* 
     printf("Innovative pkt: %d\n", i); fflush(stdout);
     printf("---------------------------------------\n"); fflush(stdout);
     for(unsigned int j = 0 ; j < d_num_ofdm_symbols; j++) {
	printf("ofdm_symbol: %d\n", j); fflush(stdout);
	for(unsigned int k = 0; k < d_occupied_carriers; k++) {
	    unsigned int ii = (j * d_occupied_carriers) + k;
	    printf(" [%d] (%f, %f) + (%f, %f) = (%f, %f)\n", k, symbols[ii].real(), symbols[ii].imag(), tmp_symbols[ii].real(), tmp_symbols[ii].imag(), out_symbols[ii].real(), out_symbols[ii].imag());
	}
     }*/

     free(tmp_symbols); 
     free(symbols);
  }
  //printf("\nfinal coded symbols\n"); fflush(stdout);

  //normalizeSignal(out_symbols);
  //printf("final coded symbols, print: \n"); fflush(stdout);
  //printSymbols(out_symbols, n_symbols);


  /* make header */
  unsigned char header_bytes[HEADERBYTELEN];
  MULTIHOP_HDR_TYPE header;
  memset(&header, 0, sizeof(MULTIHOP_HDR_TYPE));

  /* copy the coeffs into the 'header' 
     slightly tricky: coefficients when forwarded need to be simplified (refer notes)
  */
  gr_complex *new_coeffs = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size);
  memset(new_coeffs, 0, sizeof(gr_complex) * d_batch_size);
 
  /* after fix: need to get batch_size # of new coeffs = # of innovative pkts rx'ed */
  for(unsigned int i = 0; i < d_batch_size; i++) {
     gr_complex new_coeff_c;                                    // for each native pkt in batch, ie P1 and P2 //

     for(int j = 0; j < n_innovative_pkts; j++) {
	PktInfo *pInfo = inno_pkts[j];
	vector<gr_complex*> pkt_coeffs = pInfo->coeffs;
	
	int n_senders = pInfo->n_senders;

	gr_complex tmp_coeff_c;	
	for(int k = 0; k < n_senders; k++) {
	   if(k == 0) 
		tmp_coeff_c =  (coeffs[j] * pkt_coeffs[k][i]);
	   else
		tmp_coeff_c += (coeffs[j] * pkt_coeffs[k][i]);
	}	

	new_coeff_c += tmp_coeff_c;
     } 	

     new_coeffs[i] = new_coeff_c;

     printf("= [%f] ", ToPhase_f(new_coeffs[i])); fflush(stdout);	
  }

  memcpy(header.coeffs, new_coeffs, sizeof(gr_complex) * d_batch_size);

  /* populate the 'header' struct and the 'header_bytes' */
  makeHeader(header, header_bytes, flow_info);


  /* final message (type: 1-coded) */
  /* DO NOT include the DC tones */
  unsigned int n_actual_symbols = d_num_ofdm_symbols * d_data_carriers.size();
  gr_message_sptr out_msg = gr_make_message(DATA_TYPE, 0, 0, HEADERBYTELEN + (n_actual_symbols * sizeof(gr_complex)));
  memcpy(out_msg->msg(), header_bytes, HEADERBYTELEN);					            // copy header bytes

  int offset = HEADERBYTELEN;

#ifdef USE_PILOT
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++) {
     unsigned int index = i * d_occupied_carriers;
     for(unsigned int j = 0; j < d_all_carriers.size(); j++) {
	if(d_all_carriers[j] == 0) {
	    memcpy((out_msg->msg() + offset), (out_symbols + index + j), sizeof(gr_complex));
	    offset += sizeof(gr_complex);
	}
     }
  }
#else
  int dc_tones = d_occupied_carriers - d_data_carriers.size();
  assert(dc_tones > 0 && (d_occupied_carriers % 2 == 0) && (dc_tones % 2 == 0));		// even occupied_tones and dc_tones //
  unsigned int half_occupied_tones = (d_occupied_carriers - dc_tones)/2;
  
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++) {
      unsigned int index = i * d_occupied_carriers;
      /* copy 1st half of the OFDM symbol */
      memcpy((out_msg->msg() + offset), (out_symbols + index), sizeof(gr_complex) * half_occupied_tones);

      offset += (sizeof(gr_complex) * half_occupied_tones);
      index += (half_occupied_tones + dc_tones);						// bypass the DC tones //

      /* copy 2nd half of the OFDM symbol */
      memcpy((out_msg->msg() + offset), (out_symbols + index), sizeof(gr_complex) * half_occupied_tones);
      offset += (sizeof(gr_complex) * half_occupied_tones);
  }
#endif

  logSymbols(out_symbols, n_symbols);

  /********************* debug start ********************
  vector<gr_complex*> debug_sym_vec;

  gr_complex *test_symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_symbols);
  memcpy(test_symbols, out_symbols, sizeof(gr_complex) * n_symbols);

  decodeSignal(test_symbols, ToPhase_c(new_coeffs[0]));
  debug_sym_vec.push_back(test_symbols);
  demodulate(debug_sym_vec);
  logSymbols(test_symbols, n_symbols);
  ******************** debug end *************************/

  //memcpy(out_msg->msg() + HEADERBYTELEN, (void*) out_symbols, sizeof(gr_complex) * n_symbols);      // copy payload symbols
  d_out_queue->insert_tail(out_msg);
  out_msg.reset();

  free(coeffs);
  free(new_coeffs);
  free(out_symbols);

  //printf("encodePktToFwd end\n"); fflush(stdout);
}

/* just to test */
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

/* ack processing (at the forwarders) */
void
digital_ofdm_frame_sink::processACK() {
  FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  if(flowInfo == NULL)
	return;

  if(flowInfo->last_batch_acked < d_batch_number) {
	flowInfo->last_batch_acked = d_batch_number;
  }

  if(flowInfo->active_batch <= d_batch_number) {
      flowInfo->active_batch = d_batch_number+1;

      /* clear the innovative pkts stored for this batch */
      InnovativePktInfoVector inn_pkts = flowInfo->innovative_pkts;
      int size = inn_pkts.size();
      for(int i = 0; i < size; i++) {        
	   PktInfo *pktInfo = inn_pkts[i]; 
 	   int num_senders = pktInfo->n_senders;

	   /* clear all */
	   pktInfo->senders.clear();
	   for(int i = 0; i < num_senders; i++) {
	       free(pktInfo->coeffs[i]);
	       free(pktInfo->hestimates[i]);
	   }

	   pktInfo->coeffs.clear();
	   pktInfo->hestimates.clear();

	   if(num_senders > 0 && pktInfo->symbols != NULL)
	      free(pktInfo->symbols);

	   free(pktInfo);
      }
      flowInfo->innovative_pkts.clear();

      flowInfo->coeff_mat.zeros();

      /* clear the reduced coeffs vector */
      vector<gr_complex*> red_coeffs = flowInfo->reduced_coeffs;
      size = red_coeffs.size();
      for(int i = 0; i < size; i++) {
	 gr_complex *coeffs = red_coeffs[i];
	 free(coeffs);
      }
      red_coeffs.clear();	
 
      /* credit reset */
      CreditInfo *cInfo = findCreditInfo(d_flow);
      cInfo->credit = 0.0;	     
 
  }

  /* if ACK needs to be fwd'ed, then enqueue it */
  if(shouldFowardACK(d_flow, d_ack_header.prev_hop_id))
     forwardACK();	
}

inline CreditInfo*
digital_ofdm_frame_sink::findCreditInfo(unsigned char flowId) {
  //printf("findCreditInfo, flow: %d, vec_size: %d\n", flowId, d_creditInfoVector.size()); fflush(stdout);
  CreditInfo *creditInfo = NULL;
  if(d_creditInfoVector.size() == 0)
	return creditInfo;

  CreditInfoVector::iterator it = d_creditInfoVector.begin();
  
  while(it != d_creditInfoVector.end()) {
        creditInfo = *it;
	//printf("iterating flow:%d, flowId: %d\n", creditInfo->flowId, flowId); fflush(stdout);
	if(creditInfo->flowId == flowId)
	    return creditInfo;	
        it++;
  } 

  return NULL;
}

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

/* just forwards the ACK received ie d_ack_header, updates 
   the prev_hop_id to itself and tail_inserts in the queue 
*/
void
digital_ofdm_frame_sink::forwardACK() {
  //printf("forward ACK\n"); fflush(stdout);

  d_ack_header.prev_hop_id = d_id;

  /* copy e'thing except CRC */
  unsigned char ack_header_bytes[ACK_HEADERBYTELEN];
  memcpy(ack_header_bytes, &d_ack_header, ACK_HEADERDATALEN);

  /* generate CRC */
  unsigned int calc_crc = digital_crc32(ack_header_bytes, ACK_HEADERDATALEN);
  d_ack_header.hdr_crc = calc_crc;

  /* copy the crc and padding */
  memcpy(ack_header_bytes+ACK_HEADERDATALEN, &calc_crc, sizeof(int));
  memcpy(ack_header_bytes+ACK_HEADERDATALEN+sizeof(int), d_ack_header.pad, ACK_PADDING_SIZE);
 
  whiten(ack_header_bytes, ACK_HEADERBYTELEN);

  /* final message (type: 1-coded) */
  gr_message_sptr out_msg = gr_make_message(ACK_TYPE, 0, 0, 0);                // 3: ACK TYPE
  memcpy(out_msg->msg(), ack_header_bytes, HEADERBYTELEN);     	               // copy header

  /* insert into the out-queue */
  d_ack_queue.push_back(out_msg);
  out_msg.reset();
}

/* extracts ACK header */
inline void
digital_ofdm_frame_sink::extract_ack_header()
{
  d_pkt_type = d_ack_header.pkt_type;
  d_flow = d_ack_header.flow_id;
  d_batch_number = d_ack_header.batch_number;
}

/* checks the ACK_hdr is OK */
bool
digital_ofdm_frame_sink::ack_header_ok()
{
  dewhiten(ACK_HEADERBYTELEN);
  memcpy(&d_ack_header, d_header_bytes, d_hdr_byte_offset);
  unsigned int rx_crc = d_ack_header.hdr_crc;

  unsigned char *header_data_bytes = (unsigned char*) malloc(ACK_HEADERDATALEN);
  memcpy(header_data_bytes, d_header_bytes, ACK_HEADERDATALEN);
  unsigned int calc_crc = digital_crc32(header_data_bytes, ACK_HEADERDATALEN);
  free(header_data_bytes);

  return (rx_crc == calc_crc);
}

/* for the ACK forwarding decision */
bool
digital_ofdm_frame_sink::shouldFowardACK(unsigned char flowId, unsigned char prevHop)
{
  if(d_ack_route_map.size() == 0)
	return false;

  map<unsigned char, unsigned char>::iterator it = d_ack_route_map.find(flowId);
  if(it->second == prevHop)
	return true;

  return false;
}

void
digital_ofdm_frame_sink::fillAckRouteMap() {
  // TODO! 
}

void
digital_ofdm_frame_sink::whiten(unsigned char *bytes, const int len)
{
   for(int i = 0; i < len; i++)
        bytes[i] = bytes[i] ^ random_mask_tuple[i];
}


/*
// Integer Programming //
*/

void
digital_ofdm_frame_sink::slicer_ILP(gr_complex *x, gr_complex *closest_sym, unsigned char *bits, 
				    vector<gr_complex*> batched_sym_position)
{
  //printf("slicer_ILP start\n"); fflush(stdout);
  unsigned int min_index = 0;
  float min_euclid_dist = 0.0;

  // initialize //
  for(unsigned int k = 0; k < d_batch_size; k++) {
     min_euclid_dist += norm(x[k] - batched_sym_position[k][0]);
  } 
 
  //printf("initialized, min_euclid_dist: %f\n", min_euclid_dist); fflush(stdout); 
  
  // for each table entry, find the min(total_error, k) //
  unsigned int table_size = pow(2.0, double(d_batch_size));
  for(unsigned int j = 1; j < table_size; j++) {
      float euclid_dist = 0;
      for(unsigned int k = 0; k < d_batch_size; k++) {
	  euclid_dist += norm(x[k] - batched_sym_position[k][j]);
      }

      if (euclid_dist < min_euclid_dist) {
           min_euclid_dist = euclid_dist;
           min_index = j;
       }
  }

  //printf("min_euclid_dist: %f\n", min_euclid_dist); fflush(stdout);  
 
  // assign closest_sym and bits // 
  for(unsigned int k = 0; k < d_batch_size; k++) {
	closest_sym[k] = batched_sym_position[k][min_index];		// closest_sym: the ideal position where R should hv been at //
	getSymOutBits_ILP(bits, min_index);
  }

  //printf("slicer_ILP end\n"); fflush(stdout);
}


// should only be called from slicer ILP //
void
digital_ofdm_frame_sink::getSymOutBits_ILP(unsigned char *bits, int index) {
  
  if(d_batch_size == 2) { 
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
digital_ofdm_frame_sink::demodulate_ILP(FlowInfo *flowInfo)
{
  printf("demodulate_ILP\n"); fflush(stdout);

  // initialize for every batch //
  vector<unsigned char*> bytes_out_vec;
  vector<gr_complex*> out_sym_vec;

  for(unsigned int k = 0; k < d_batch_size; k++) {
     unsigned char *bytes = (unsigned char*) malloc(sizeof(unsigned char) * d_occupied_carriers);
     gr_complex *symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);

     bytes_out_vec.push_back(bytes);
     out_sym_vec.push_back(symbols);
  }
  printf("init1\n"); fflush(stdout);

  // Since the 'h' values change per subcarrier, reduced coeffs are per subcarrier, hence, there
  // the total # of maps = batch_size * occupied_carriers & each map will have 2^batch_size entries
  vector<vector<gr_complex*> > batched_sym_position;

  if(d_nsenders > 1) {
     for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
        updateCoeffMatrix(flowInfo, d_data_carriers[i]);                  // reduce coeffs only when #senders>1, else the pkt will be equalized!

	// build the map for each batch on each subcarrier //
	vector<gr_complex*> sym_position;
        buildMap_ILP(flowInfo->coeff_mat, sym_position);
	batched_sym_position.push_back(sym_position);
     }
     assert(batched_sym_position.size() == d_data_carriers.size());
  } 
  else {
     assert(d_nsenders == 1);
     // no need to build a map for each subcarrier, since the 'h' coeffs hv been already taken care off (equalized) //
     vector<gr_complex*> sym_position;
     buildMap_ILP(flowInfo->coeff_mat, sym_position);
     batched_sym_position.push_back(sym_position);
  }

  debugMap_ILP(batched_sym_position); 

  // run the demapper for every OFDM symbol (all batches within the symbol are demapped at once!) //
  int packetlen_cnt[MAX_BATCH_SIZE];
  memset(packetlen_cnt, 0, sizeof(int) * MAX_BATCH_SIZE);

  vector<gr_complex> dfe_vec[MAX_BATCH_SIZE];
  for(unsigned int k = 0; k < MAX_BATCH_SIZE; k++) {
     dfe_vec[k].resize(d_occupied_carriers);
     fill(dfe_vec[k].begin(), dfe_vec[k].end(), gr_complex(1.0,0.0));
  }
  
  unsigned char packet[MAX_BATCH_SIZE][MAX_PKT_LEN];
  reset_demapper();
  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
      unsigned int bytes_decoded = demapper_ILP(o, bytes_out_vec, batched_sym_position, flowInfo, dfe_vec);   // same # of bytes decoded/batch

      unsigned int jj = 0;
      while(jj < bytes_decoded) {
	 for(unsigned int k = 0; k < d_batch_size; k++) {
	      packet[k][packetlen_cnt[k]++] = bytes_out_vec[k][jj];
	      if ((packetlen_cnt[k]) == d_packetlen) {	
		  gr_message_sptr msg = gr_make_message(0, d_packet_whitener_offset, 0, (packetlen_cnt[k]));
		  memcpy(msg->msg(), packet[k], (packetlen_cnt[k]));
		  d_target_queue->insert_tail(msg);
		  msg.reset();
	      }
         }//for
	 jj++;
      } //while
  }

  /* cleaning */
  for(unsigned int i = 0; i < d_batch_size; i++) {
     free(out_sym_vec[i]);
     free(bytes_out_vec[i]);
  }
  out_sym_vec.clear();
  bytes_out_vec.clear();

  /* batched_sym_position */
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
     for(unsigned int k = 0; k < d_batch_size; k++) {
	   gr_complex *sym_position =  batched_sym_position[i][k];
	   free(sym_position);
     }
 
     if(d_nsenders == 1) break;
  }
 
  d_last_batch_acked = d_active_batch;
  printf("demodulate_ILP done!\n"); fflush(stdout);
}


// sym_vec: input to demapper; each row in the vector belongs to a batch
// out_vec: output bytes; each row belongs to a batch 
unsigned int
digital_ofdm_frame_sink::demapper_ILP(unsigned int ofdm_symbol_index, vector<unsigned char*> out_vec, 
				      vector<vector<gr_complex*> > batched_sym_position, FlowInfo *flowInfo, 
				      vector<gr_complex> *dfe_vec) {

  printf("demapper_ILP, ofdm_symbol_index: %d\n", ofdm_symbol_index); fflush(stdout);
  unsigned int bytes_produced = 0;

  gr_complex carrier[MAX_BATCH_SIZE], accum_error[MAX_BATCH_SIZE];
  assert(d_batch_size <= MAX_BATCH_SIZE);

  gr_complex* sym_vec[MAX_BATCH_SIZE];

  InnovativePktInfoVector innovativePkts = flowInfo->innovative_pkts;
  for(unsigned int i = 0; i < d_batch_size; i++) {
     carrier[i] = gr_expj(d_phase[i]);
     accum_error[i] = 0.0;

     PktInfo *pInfo = innovativePkts[i];
     gr_complex *rx_symbols_this_batch = pInfo->symbols;

     sym_vec[i] = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
     memset(sym_vec[i], 0, sizeof(gr_complex) * d_occupied_carriers);

     memcpy(sym_vec[i], &rx_symbols_this_batch[ofdm_symbol_index * d_occupied_carriers], sizeof(gr_complex) * d_occupied_carriers);
  }

  printf("demapper_ILP: sym_vec done\n"); fflush(stdout);
  gr_complex sigrot[MAX_BATCH_SIZE], closest_sym[MAX_BATCH_SIZE];
  unsigned int partial_byte[MAX_BATCH_SIZE];
  unsigned char bits[MAX_BATCH_SIZE];

  memset(partial_byte, 0, sizeof(unsigned int) * MAX_BATCH_SIZE);
  memset(bits, 0, sizeof(unsigned char) * MAX_BATCH_SIZE);

  unsigned int byte_offset = 0;
  printf("ILP demod ofdm symbol now.. \n"); fflush(stdout);
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {

     // demodulate 1-byte at a time //
     if (byte_offset < 8) {

	  // find sigrot for each batch //
          for(unsigned int k = 0; k < d_batch_size; k++) {
	      sigrot[k] = sym_vec[k][d_data_carriers[i]] * carrier[k] * dfe_vec[k][i];
	  }

	  //printf("before slicer\n"); fflush(stdout);
	  // runs the slicer (min-error algo) to get all bits of all the batches on *this* subcarrier //
	  if(d_nsenders == 1) 
		slicer_ILP(sigrot, closest_sym, bits, batched_sym_position[0]);	
 	   else 
  	        slicer_ILP(sigrot, closest_sym, bits, batched_sym_position[i]);

	  // update accum_error for each batch //
	  for(unsigned int k = 0; k < d_batch_size; k++) {
	     accum_error[k] += sigrot[k] * conj(closest_sym[k]); 
	     if (norm(sigrot[k])> 0.001) dfe_vec[k][i] +=  d_eq_gain*(closest_sym[k]/sigrot[k]-dfe_vec[k][i]);

	     assert((8 - byte_offset) >= d_nbits);
	     partial_byte[k] |= bits[k] << (byte_offset);
	  }

	  byte_offset += d_nbits;
	  
	  // debug // 
	  /*
	  for(unsigned int k = 0; k < d_batch_size; k++) {
  	      printf("partial_byte: %x, byte_offset: %d, bits: %x\n", partial_byte[k], byte_offset, bits[k]); fflush(stdout);
	  }*/
     }
  
     if(byte_offset == 8) {
	for(int k = 0; k < d_batch_size; k++) {
	    //printf("k: %d, demod_byte: %x (%d) \n", k, partial_byte[k], partial_byte[k]); fflush(stdout);
            out_vec[k][bytes_produced] = partial_byte[k];
	    partial_byte[k] = 0;
	}
	bytes_produced++;
	byte_offset = 0; 
     }
  }

  assert(byte_offset == 0); 

  
  // update the accum_error, etc //
  for(unsigned int k = 0; k < d_batch_size; k++) {
     float angle = arg(accum_error[k]);
     d_freq[k] = d_freq[k] - d_freq_gain*angle;
     d_phase[k] = d_phase[k] + d_freq[k] - d_phase_gain*angle;
     if (d_phase[k] >= 2*M_PI) d_phase[k] -= 2*M_PI;
     if (d_phase[k] <0) d_phase[k] += 2*M_PI;
  }

  return bytes_produced;
}

// build the combined symbol map. Needs to be called for every new batch. There will 
// 2^d_batch_size entries in each map 
void
digital_ofdm_frame_sink::buildMap_ILP(cx_mat coeffs, vector<gr_complex*> &batched_sym_position) {
  int n_entries = pow(2.0, double(d_batch_size));

  printf("buildMap_ILP, entries: %d, nsenders: %d\n", n_entries, d_nsenders); fflush(stdout); 
  if(d_batch_size == 2) {
     for(unsigned int i = 0; i < d_batch_size; i++) {
         gr_complex *sym_position = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);
	
         int j = 0;
	 sym_position[j++] = coeffs(i, 0) * (comp_d) d_sym_position[0] + coeffs(i, 1) * (comp_d) d_sym_position[0];
	 sym_position[j++] = coeffs(i, 0) * (comp_d) d_sym_position[0] + coeffs(i, 1) * (comp_d) d_sym_position[1];
	 sym_position[j++] = coeffs(i, 0) * (comp_d) d_sym_position[1] + coeffs(i, 1) * (comp_d) d_sym_position[0];
	 sym_position[j++] = coeffs(i, 0) * (comp_d) d_sym_position[1] + coeffs(i, 1) * (comp_d) d_sym_position[1];

	 batched_sym_position.push_back(sym_position);
     }
  } 
  else if (d_batch_size == 3) {
     for(unsigned int i = 0; i < d_batch_size; i++) {
	 gr_complex *sym_position = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);

         int j = 0;
         sym_position[j++] = coeffs(i, 0)* (comp_d)d_sym_position[0] + coeffs(i, 1)* (comp_d)d_sym_position[0] + coeffs(i, 2)* (comp_d)d_sym_position[0];
         sym_position[j++] = coeffs(i, 0)* (comp_d)d_sym_position[0] + coeffs(i, 1)* (comp_d)d_sym_position[0] + coeffs(i, 2)* (comp_d)d_sym_position[1];
         sym_position[j++] = coeffs(i, 0)* (comp_d)d_sym_position[0] + coeffs(i, 1)* (comp_d)d_sym_position[1] + coeffs(i, 2)* (comp_d)d_sym_position[0];
         sym_position[j++] = coeffs(i, 0)* (comp_d)d_sym_position[0] + coeffs(i, 1)* (comp_d)d_sym_position[1] + coeffs(i, 2)* (comp_d)d_sym_position[1];
         sym_position[j++] = coeffs(i, 0)* (comp_d)d_sym_position[1] + coeffs(i, 1)* (comp_d)d_sym_position[0] + coeffs(i, 2)* (comp_d)d_sym_position[0];
         sym_position[j++] = coeffs(i, 0)* (comp_d)d_sym_position[1] + coeffs(i, 1)* (comp_d)d_sym_position[0] + coeffs(i, 2)* (comp_d)d_sym_position[1];
         sym_position[j++] = coeffs(i, 0)* (comp_d)d_sym_position[1] + coeffs(i, 1)* (comp_d)d_sym_position[1] + coeffs(i, 2)* (comp_d)d_sym_position[0];
         sym_position[j++] = coeffs(i, 0)* (comp_d)d_sym_position[1] + coeffs(i, 1)* (comp_d)d_sym_position[1] + coeffs(i, 2)* (comp_d)d_sym_position[1];

	 batched_sym_position.push_back(sym_position);
     }
  }
  printf("buildMap_ILP done\n"); fflush(stdout);
}

void
digital_ofdm_frame_sink::debugMap_ILP(vector<vector<gr_complex*> > batched_sym_position) {
  printf("debugMap_ILP\n"); fflush(stdout);
  if(d_nsenders == 1) {
	assert(batched_sym_position.size() == 1);
  }

  for(int i = 0; i < batched_sym_position.size(); i++) {
      vector<gr_complex*> batch_position = batched_sym_position[i];
      if(batch_position.size() != d_batch_size) {
	printf("batch_position.size: %d, d_batch_size: %d\n", batch_position.size(), d_batch_size); fflush(stdout);
	assert(false);
      }
      assert(batch_position.size() == d_batch_size);
      for(int j = 0; j < batch_position.size(); j++) {
	 gr_complex *positions = batch_position[j];
	 int n_entries = pow(2.0, double(d_batch_size));

	 /*
	 for(int k = 0; k < n_entries; k++) {
	    printf("k: %d, (%f, %f) \n", k, positions[k].real(), positions[k].imag()); fflush(stdout);	
	 }*/
      }
  }
  printf("debugMap_ILP done\n"); fflush(stdout);
} 

