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

using namespace arma;
using namespace std;

#define SCALE_FACTOR_PHASE 1e2
#define SCALE_FACTOR_AMP 1e4

#define UNCORRECTED_REPLAY 0
#define CORRECTED_REPLAY 1

//#define SEND_ACK_ETHERNET 1

#define VERBOSE 0
//#define SCALE 1e3

static const pmt::pmt_t SYNC_TIME = pmt::pmt_string_to_symbol("sync_time");
int last_noutput_items;

//uhd_usrp_source_sptr 
static boost::shared_ptr<uhd_usrp_source> d_usrp_rx; //uhd_make_usrp_source(arg1, arg2);

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

inline void
digital_ofdm_frame_sink::reset_demapper() {
  printf("reset demapper\n"); fflush(stdout);
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
digital_ofdm_frame_sink::extract_header()
{
  d_packet_whitener_offset = 0;                 // apurv++: hardcoded!

  d_pkt_type = d_header.pkt_type;
  d_flow = d_header.flow_id;

  d_packetlen = d_header.packetlen;
  d_batch_number = d_header.batch_number;

  d_nsenders = d_header.nsenders;	
  d_lead_sender = d_header.lead_sender;	

  //d_nsenders = 1;				// TESTING - REMOVEEEEEEEEEEEEEEE
  //d_lead_sender = 1;				// TESTING - REMOVEEEEEEEEEEEEEEE

  d_pkt_num = d_header.pkt_num;
  d_prevLinkId = d_header.link_id;

  printf("\tpkt_num: %d \t\t\t\t batch_num: %d \t\t\t\t len: %d src: %d prev-link: %d\n", d_header.pkt_num, d_header.batch_number, d_packetlen, d_header.src_id, d_prevLinkId); fflush(stdout);

  if (VERBOSE)
    fprintf(stderr, " hdr details: (src: %d), (rx: %d), (batch_num: %d), (d_nsenders: %d), (d_packetlen: %d), (d_pkt_type: %d)\n",
                    d_header.src_id, d_header.dst_id, d_batch_number, d_nsenders, d_packetlen, d_header.pkt_type);
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

/* experiment to compare the dfe performance for pilot vs no-pilot subcarriers case 
   In each case, use 72 data subcarriers - 
   - w/ pilots --> use the 12 subcarriers as pilot subcarriers (dfe done based on pilot) 
   - w/o pilots --> use the 12 subcarriers as null carriers (dfe done based on data)
*/
void digital_ofdm_frame_sink::log_dfe_pilot(gr_complex *dfe_log) 
{
   if(!d_log_dfe_pilot_open) {
      const char *filename = "dfe_pilot.dat";
      int fd;
      if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename);
         assert(false);
      }
      else {
         if((d_fp_dfe_pilot = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "dfe file cannot be opened\n");
            close(fd);
            assert(false);
         }
      }
      d_log_dfe_pilot_open = true;
   }

   assert(d_fp_dfe_pilot);
   int count = ftell(d_fp_dfe_pilot);
   //count = fwrite_unlocked(&dfe_log[0], sizeof(gr_complex), d_data_carriers.size(), d_fp_dfe_pilot);
   //assert(count == d_data_carriers.size());

   count = fwrite_unlocked(&d_dfe[0], sizeof(gr_complex), d_pilot_carriers.size(), d_fp_dfe_pilot);
   assert(count == d_pilot_carriers.size());
}

void digital_ofdm_frame_sink::log_dfe_data()
{
   if(!d_log_dfe_data_open) {
      const char *filename = "dfe_data.dat";
      int fd;
      if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename);
         assert(false);
      }
      else {
         if((d_fp_dfe_data = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "dfe file cannot be opened\n");
            close(fd);
            assert(false);
         }
      }
      d_log_dfe_data_open = true;
   }

   int count = ftell(d_fp_dfe_data);
   //printf("size: %d\n", d_dfe.size()); fflush(stdout);
   count = fwrite_unlocked(&d_dfe[0], sizeof(gr_complex), d_data_carriers.size(), d_fp_dfe_data);
   assert(count == d_data_carriers.size());
   //gr_complex *test = (gr_complex*) malloc(sizeof(gr_complex) * d_data_carriers.size());
   //memcpy(test, &d_dfe[0], sizeof(gr_complex) * d_data_carriers.size());
   //free(test);
}

/*
void digital_ofdm_frame_sink::open_dfe_log()
{
  printf("open_dfe_log\n"); fflush(stdout);
  const char *filename1 = "dfe_pilot.dat";
  int fd;
  if ((fd = open (filename1, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename1);
     assert(false);
  }
  else {
      if((d_fp_dfe_pilot = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "dfe file cannot be opened\n");
            close(fd);
            assert(false);
      }
  }

  const char *filename2 = "dfe_data.dat";
  int fd2;
  if ((fd2 = open (filename2, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename2);
     assert(false);
  }
  else {
      if((d_fp_dfe_data = fdopen (fd2, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "dfe file cannot be opened\n");
            close(fd2);
            assert(false);
      }
  }
}
*/

#ifdef USE_PILOT
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
      printf("(after header) sender index: %d, d_phase: %f, d_freq: %f\n", sender_index, d_phase[sender_index], d_freq[sender_index]); fflush(stdout);
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
#else 		// DEFAULT version (without pilots) //
unsigned int digital_ofdm_frame_sink::demapper(const gr_complex *in,
                                          unsigned char *out)
{
  unsigned int i=0, bytes_produced=0;
  gr_complex carrier;

  carrier=gr_expj(d_phase[0]);

  gr_complex accum_error = 0.0;
  gr_complex accum_error1 = 0.0; 

  while(i < d_data_carriers.size()) {
    if(d_nresid > 0) {
      d_partial_byte |= d_resid;
      d_byte_offset += d_nresid;
      d_nresid = 0;
      d_resid = 0;
    }

    while((d_byte_offset < 8) && (i < d_data_carriers.size())) {
      gr_complex sigrot = in[d_data_carriers[i]]*carrier*d_dfe[i];
      //printf("sigrot: (%f, %f)\t in: (%f, %f)\n", sigrot.real(), sigrot.imag(), in[d_data_carriers[i]].real(), in[d_data_carriers[i]].imag()); fflush(stdout);

      if(d_derotated_output != NULL){
        d_derotated_output[i] = sigrot;
      }

      unsigned char bits = slicer_hdr(sigrot);

      gr_complex closest_sym = d_hdr_sym_position[bits];

      accum_error += sigrot * conj(closest_sym);
      accum_error1 += in[d_data_carriers[i]] * conj(closest_sym);

      // FIX THE FOLLOWING STATEMENT
      if (norm(sigrot)> 0.001) {
	d_dfe[i] +=  d_eq_gain*(closest_sym/sigrot-d_dfe[i]);
      }

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

  //printf("log_dfe_data called\n"); fflush(stdout);
  //log_dfe_data();
  float angle = arg(accum_error);
  float angle1 = arg(accum_error1);

  d_freq[0] = d_freq[0] - d_freq_gain*angle;
  d_phase[0] = d_phase[0] + d_freq[0] - d_phase_gain*angle;
  if (d_phase[0] >= 2*M_PI) {
	d_phase[0] -= 2*M_PI;
  }
  if (d_phase[0] <0) {
	d_phase[0] += 2*M_PI;
  }

  /* for debug - also calculate the phase had there been no compensation */
  d_freq1[0] = d_freq1[0] - d_freq_gain*angle1;
  d_phase1[0] = d_phase1[0] + d_freq1[0] - d_phase_gain*angle1;
  /*
  if (d_phase1[0] >= 2*M_PI) {
    d_phase1[0] -= 2*M_PI;
  }
  if (d_phase1[0] <0) {
    d_phase1[0] += 2*M_PI;
  } */
  printf("angle: %f, d_phase: %f d_freq: %f -------- angle1: %f, d_phase1: %f, d_freq1: %f\n", angle, d_phase[0], d_freq[0], angle1, d_phase1[0], d_freq1[0]); fflush(stdout);

  if(d_hdr_ofdm_index == 0) {
    d_start_angle1 = angle1; d_start_freq1 = d_freq1[0]; d_start_phase1 = d_phase1[0];
  } else if(d_hdr_ofdm_index == d_num_hdr_ofdm_symbols-1) {
    d_end_angle1 = angle1; d_end_freq1 = d_freq1[0]; d_end_phase1 = d_phase1[0];
    d_end_angle = angle;

    d_slope_angle1 = (d_end_angle1 - d_start_angle1)/((float) d_hdr_ofdm_index);
    d_slope_freq1 = (d_end_freq1 - d_start_freq1)/((float) d_hdr_ofdm_index);
    d_slope_phase1 = (d_end_phase1 - d_start_phase1)/((float) d_hdr_ofdm_index);
    //printf("d_start_phase1: %f, d_end_phase1: %f, diff: %f\n", d_start_phase1, d_end_phase1, (d_end_phase1 - d_start_phase1)); fflush(stdout); 
    printf("d_end_angle: %f, d_end_angle1: %f, d_end_phase1: %f, d_end_freq1: %f\n", d_end_angle, d_end_angle1, d_end_phase1, d_end_freq1); fflush(stdout);
    printf("slope_angle1: %f, slope_freq1: %f, slope_phase1: %f, d_hdr_ofdm_index: %d\n", d_slope_angle1, d_slope_freq1, d_slope_phase1, d_hdr_ofdm_index); fflush(stdout);
 } 
 
  /*
  gr_complex phase_error = 0.0;
  float cur_pilot = 1.0;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); i++) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    phase_error += (in[di] * conj(pilot_sym));
    //printf("(%f, %f) \t", in[di].real(), in[di].imag()); 
  }
  printf("\n");
  printf("accum_error: %f, accum_error1: %f, phase_error: %f\n", arg(accum_error), arg(accum_error1), arg(phase_error)); fflush(stdout);
  */
  return bytes_produced;
}
#endif

digital_ofdm_frame_sink_sptr
digital_make_ofdm_frame_sink(const std::vector<gr_complex> &hdr_sym_position,
                        const std::vector<unsigned char> &hdr_sym_value_out,
			const std::vector<gr_complex> &data_sym_position,
                        const std::vector<unsigned char> &data_sym_value_out,
                        gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
			unsigned int occupied_carriers, unsigned int fft_length,
                        float phase_gain, float freq_gain, unsigned int id, 
			unsigned int batch_size, unsigned int decode_flag, 
			int fwd_index, int replay_flag,
			int exp_size, int fec_n, int fec_k, int degree)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_frame_sink(hdr_sym_position, hdr_sym_value_out,
							data_sym_position, data_sym_value_out,
                                                        target_queue, fwd_queue,
							occupied_carriers, fft_length,
                                                        phase_gain, freq_gain, id,
							batch_size, decode_flag, fwd_index, replay_flag,
							exp_size, fec_n, fec_k, degree));
}


digital_ofdm_frame_sink::digital_ofdm_frame_sink(const std::vector<gr_complex> &hdr_sym_position,
                                       const std::vector<unsigned char> &hdr_sym_value_out,
		                       const std::vector<gr_complex> &data_sym_position,
                		       const std::vector<unsigned char> &data_sym_value_out,
                                       gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
				       unsigned int occupied_carriers, unsigned int fft_length,
                                       float phase_gain, float freq_gain, unsigned int id,
				       unsigned int batch_size, unsigned int decode_flag, 
				       int fwd_index, int replay_flag,
				       int exp_size, int fec_n, int fec_k, int degree)
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
    d_id(id),
    d_fft_length(fft_length),
    d_out_queue(fwd_queue),
    d_fwd_index(fwd_index),
    d_expected_size(exp_size),
    d_fec_n(fec_n),
    d_fec_k(fec_k),
    d_degree(degree)
{
  std::string carriers = "F00F";                //8-DC subcarriers      // apurv++
  //std::string carriers = "FC3F";		  // 4-dc

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
#ifdef USE_PILOT			// check!

#if 1
  /* pilot configuration */
  int num_pilots = 8; //4;
  unsigned int pilot_index = 0;                      // tracks the # of pilots carriers added   
  unsigned int data_index = 0;                       // tracks the # of data carriers added
  unsigned int count = 0;                            // tracks the total # of carriers added
  unsigned int pilot_gap = 11; //18;
  unsigned int start_offset = 0; //8;
#else
  /* pilot configuration */
  int num_pilots = 4;
  unsigned int pilot_index = 0;                      // tracks the # of pilots carriers added   
  unsigned int data_index = 0;                       // tracks the # of data carriers added
  unsigned int count = 0;                            // tracks the total # of carriers added
  unsigned int pilot_gap = 14;
  unsigned int start_offset = 2; //8;
#endif
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
#ifdef USE_PILOT			// check!
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
        # of pilots        = 4
        gap b/w pilots     = 72/4 = 18
        start pilot        = 8
        other pilots       = 8, , 26, .. so on
        P.S: DC tones should not be pilots!
  */
   d_dfe.resize(d_pilot_carriers.size());
   //d_dfe.resize(d_data_carriers.size());
#else
   d_dfe.resize(d_data_carriers.size());
#endif
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
  d_fp_hestimates = NULL;

  d_in_estimates = (gr_complex*) malloc(sizeof(gr_complex) * occupied_carriers);
  memset(d_in_estimates, 0, sizeof(gr_complex) * occupied_carriers);  

  //assert(open_hestimates_log());

  d_log_pkt = false;
  d_log_ofdm_index = 0;
  d_fp_sampler = NULL;
  d_fp_timing = NULL;
  //assert(open_sampler_log());

  d_fp_corrected_symbols = NULL;
  d_fp_uncorrected_symbols = NULL;
  //d_fp_corr_log = NULL;
  //assert(open_corrected_symbols_log());

  int len = 400;
  
  pkt_symbols = (gr_complex*) malloc(sizeof(gr_complex) * len *d_fft_length);
  memset(pkt_symbols, 0, sizeof(gr_complex) * len *d_fft_length);
  timing_symbols = (char*) malloc(sizeof(char) * len * d_fft_length);
  memset(timing_symbols, 0, sizeof(char) * len * d_fft_length);
  

  d_corrected_symbols = (gr_complex*) malloc(sizeof(gr_complex) * len *d_fft_length);
  memset(d_corrected_symbols, 0, sizeof(gr_complex) * len *d_fft_length);

  d_uncorrected_symbols = (gr_complex*) malloc(sizeof(gr_complex) * len *d_fft_length);
  memset(d_uncorrected_symbols, 0, sizeof(gr_complex) * len *d_fft_length);


  d_save_flag = false;

  populateCompositeLinkInfo();
  populateCreditInfo();
  num_acks_sent = 0;
  num_data_fwd = 0;

  //assert(openLogSymbols());

  d_demod_log_file = false;
  d_fp_demod = NULL;

  memset(d_phase, 0, sizeof(float) * MAX_BATCH_SIZE);
  memset(d_freq, 0, sizeof(float) * MAX_BATCH_SIZE);

  memset(d_phase1, 0, sizeof(float) * MAX_BATCH_SIZE); 
  memset(d_freq1, 0, sizeof(float) * MAX_BATCH_SIZE); 

#ifdef SEND_ACK_ETHERNET
  d_src_ip_addr = "128.83.141.213";
  d_src_sock_port = 9000;
  d_ack_sock = create_ack_sock();
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
  /*
  int n_data_carriers = d_data_carriers.size();
  int n_entries = pow(2.0, double(d_batch_size));
  d_euclid_dist = (float**) malloc(sizeof(float*) * n_data_carriers);
  for(int i = 0; i < n_data_carriers; i++) {
     d_euclid_dist[i] = (float*) malloc(sizeof(float) * n_entries); 	
  }
  */
  d_fp_dfe_data = NULL; d_fp_dfe_pilot = NULL;
  d_log_dfe_data_open = false; d_log_dfe_pilot_open = false;

  d_fp_sync_symbols = NULL;
  d_sync_file_opened = false;

  std::string arg("");
  d_usrp = uhd::usrp::multi_usrp::make(arg);
  d_flow = -1;

  d_usrp_rx = get_usrp_source_instance();
  assert(d_usrp_rx);
  printf("sample rate: %f\n", d_usrp_rx->get_samp_rate()); 
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
  printf("size: %d, d_data_nbits: %d\n", d_data_sym_value_out.size(), d_data_nbits); fflush(stdout);

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
  unsigned int n_data_carriers = d_data_carriers.size();
  switch(d_state) {

  case STATE_SYNC_SEARCH:    // Look for flag indicating beginning of pkt
    if (VERBOSE)
      fprintf(stderr,"SYNC Search, noutput=%d\n", noutput_items);

    if (sig[0]) {  // Found it, set up for header decode
      enter_have_sync();
      test_timestamp(1);
      if(input_items.size() >= 4) {
	  printf("log_pkt: true\n"); fflush(stdout);
	  /* offline analysis start */
	  d_log_pkt = true;
	  d_log_ofdm_index = 1;
	  memcpy(pkt_symbols, in_sampler, sizeof(gr_complex) * d_fft_length);					//preamble
	  memset(timing_symbols, 0, sizeof(char) * d_fft_length);
	  timing_symbols[0]  = 1;

	  unsigned int left_guard = (d_fft_length - d_occupied_carriers)/2;
	  memcpy(d_corrected_symbols + left_guard, in, sizeof(gr_complex) * d_occupied_carriers);		// DFE corrected symbols log (f domain) //
	  memcpy(d_uncorrected_symbols + left_guard, in, sizeof(gr_complex) * d_occupied_carriers);
	  /* offline analysis dump end */
      }
    }
    break;      // don't demodulate the preamble, so break!

  case STATE_HAVE_SYNC:
    //printf("STATE_HAVE_SYNC\n"); fflush(stdout);
    //test_timestamp(1);

    /* apurv++: equalize hdr - and use the estimates only if NOT *found sync in HAVE_SYNC* */
    if(use_estimates && !sig[0]) {
       unsigned int left_guard = (d_fft_length - d_occupied_carriers)/2;
       memcpy(d_corrected_symbols + (d_log_ofdm_index*d_fft_length) + left_guard, in, sizeof(gr_complex) * d_occupied_carriers);    // (f domain) //
       memcpy(d_uncorrected_symbols + (d_log_ofdm_index*d_fft_length) + left_guard, in, sizeof(gr_complex) * d_occupied_carriers);
       equalizeSymbols(&in[0], &in_estimates[0]);
       memcpy(d_in_estimates, in_estimates, sizeof(gr_complex) * d_occupied_carriers);	// use in HAVE_HEADER state //
    }

    // only demod after getting the preamble signal; otherwise, the 
    // equalizer taps will screw with the PLL performance
#ifdef USE_PILOT	
    bytes = demapper_pilot(&in[0], d_bytes_out);
#else
    bytes = demapper(&in[0], d_bytes_out);      // demap one ofdm symbol
#endif
    d_hdr_ofdm_index++;

    /* offline dump */
    if(d_log_pkt) {
        memcpy(pkt_symbols + (d_log_ofdm_index*d_fft_length), in_sampler, sizeof(gr_complex) * d_fft_length);           // hdr
        d_log_ofdm_index++;
    }

    if (VERBOSE && sig[0]) {
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

	  /* log hestimates */
	  if(use_estimates && 0) {
	      int count = ftell(d_fp_hestimates);
	      count = fwrite_unlocked(&in_estimates[0], sizeof(gr_complex), d_occupied_carriers, d_fp_hestimates);
   	  }

          printf("HEADER OK prev_hop: %d pkt_num: %d dst_id: %d\n", d_header.prev_hop_id, d_header.pkt_num, d_header.dst_id); fflush(stdout);
          extract_header();						// fills local fields with header info

          assert(d_nsenders > 0);

	  /* if the batch is already full rank, no need to process the packet any further */
	  FlowInfo *flowInfo = getFlowInfo(false, d_flow);

	  /* commented -- to enable more packets than the full rank, since the batch might still be corrupt!
	  if(flowInfo && flowInfo->active_batch == d_header.batch_number && isFullRank(flowInfo)) {
		printf("batch%d is already full rank, discard pkt\n", flowInfo->active_batch); 
		fflush(stdout);
	        enter_search();
		break;
	  }*/

	  /* ensure if I'm in the current session, then the pkt num matches the d_save_pkt_num */
	  printf("d_save_flag: %d, d_pkt_num: %d, d_save_pkt_num: %d\n", d_save_flag, d_pkt_num, d_save_pkt_num); fflush(stdout);
	  if(d_save_flag) {
 	      if(d_pkt_num != d_save_pkt_num) {
		 reset_demapper();
		 resetPktInfo(d_pktInfo);
		 flowInfo->innovative_pkts.pop_back();
		 /*
		 int last_index = d_pktInfo->coeffs.size() - 1;
		 free((d_pktInfo->hestimates)[last_index]);
		 free((d_pktInfo->coeffs)[last_index]);
		 d_pktInfo->hestimates.pop_back();
		 d_pktInfo->coeffs.pop_back();*/
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
		    break;
	        } 
	  	else if(d_header.batch_number <= d_last_batch_acked)
	  	{
		    printf("BATCH %d already ACKed --\n", d_header.batch_number); fflush(stdout);
		    enter_search();
		    reset_demapper();
		    break;
		}
		else if(d_header.batch_number > d_active_batch)
  	  	    prepareForNewBatch();				// ensure the FlowInfo is in place

                if(d_fwd && 0) {
                   assert(!d_dst);

                   // TODO: check if the credit allows, only then proceed - need to be made way more robust //
                   makePacket(true);
                   enter_search();                      // perhaps block the work thread instead for the tx time duration?? //
                   break;
                }

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

#ifdef USE_PILOT
		/* changes from the '#else' version (this flag forces to):
		1. no need for isInnovative(), since we are not using Gaussian elimination anymore
		2. coefficients have been saved already, post processing (to reduced) is delayed until actual demapping is done */
		printf("marking inno check as false\n"); fflush(stdout);
		if(1) {
#endif
                     d_state = STATE_HAVE_HEADER;
		     //d_state = STATE_HAVE_NULL; d_null_symbols = 0;
		     d_curr_ofdm_symbol_index = 0;
		     d_num_ofdm_symbols = ceil(((float) (d_packetlen * 8))/(d_data_carriers.size() * d_data_nbits));
		     assert(d_num_ofdm_symbols < MAX_OFDM_SYMBOLS); 				// FIXME - arbitrary '70'.. 
		     printf("d_num_ofdm_symbols: %d, actual sc size: %d, d_data_nbits: %d\n", d_num_ofdm_symbols, d_data_carriers.size(), d_data_nbits);
		     fflush(stdout);

		     /* rx symbols */
		     d_pktInfo->symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_num_ofdm_symbols * d_occupied_carriers);
		     memset(d_pktInfo->symbols, 0, sizeof(gr_complex) * d_num_ofdm_symbols * d_occupied_carriers);
		     printf("allocated pktInfo rxSymbols\n"); fflush(stdout);
                }
	 	else {
		     assert(0);
		     resetPktInfo(d_pktInfo);				// non-innovative; screw it
		     FlowInfo *flowInfo = getFlowInfo(false, d_flow);
		     assert(flowInfo);
		     flowInfo->innovative_pkts.pop_back(); 
		     free(d_pktInfo);
		     enter_search();
		     break;
		}
	     } else {
		if(d_pending_senders <= 1) {
		   printf("d_pending_senders: %d\n", d_pending_senders); fflush(stdout);
		   assert(false);
		}
		assert(d_pending_senders > 1);				// more senders remain - switch to preamble look-up
		d_pending_senders--;
		enter_search();
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

  case STATE_HAVE_NULL:
    //adjust_H_estimate(&in_estimates[0]);
    //equalizeSymbols(&in[0], &in_estimates[0]);
    memcpy(&d_training_symbols[d_null_symbols*MAX_OCCUPIED_CARRIERS], &in[0], sizeof(gr_complex) * d_occupied_carriers);
    d_null_symbols++;
    if(d_null_symbols == NULL_OFDM_SYMBOLS) {
	//calculate_fine_offset();
        d_state = STATE_HAVE_HEADER;
    }
    break;

  case STATE_HAVE_HEADER:						// process the frame after header
    if (sig[0]) 
        printf("ERROR -- Found SYNC in HAVE_HEADER, length of %d\n", d_packetlen);

    /* disable equalization for now  
    if(d_nsenders == 1 && use_estimates) {	
       equalizeSymbols(&in[0], &d_in_estimates[0]);
    } */

    if(d_curr_ofdm_symbol_index >= d_num_ofdm_symbols) {
	assert(false);
        enter_search();         					// something went crazy
	reset_demapper();
        break;
    }

    assert(d_curr_ofdm_symbol_index < d_num_ofdm_symbols);
    assert(d_pending_senders == 0);

    /* de-normalize the signal */	
#if 0
    for(unsigned int i = 0; i < n_data_carriers; i++)
        in[d_data_carriers[i]] *= gr_complex(d_header.factor);
#endif

    /* normalization is only performed for dst. Since we collect the fwder traces in the dst mode, 
       norm did not be done *again* when fwder traces are replayed */
    if(d_dst && d_replay_flag == -1) {
	//printf("denormalizing the signal, factor: %f\n", d_header.factor); fflush(stdout);
	int n_data_carriers = d_data_carriers.size();
        for(unsigned int i = 0; i < n_data_carriers; i++)
            in[d_data_carriers[i]] *= gr_complex(d_header.factor * sqrt(d_batch_size));
    }

    storePayload(&in[0], in_sampler);
    d_curr_ofdm_symbol_index++;

    if(VERBOSE) {
       printf("d_curr_ofdm_symbol_index: %d, d_num_ofdm_symbols: %d\n", d_curr_ofdm_symbol_index, d_num_ofdm_symbols);
       fflush(stdout);
    }
	 
    if(d_curr_ofdm_symbol_index == d_num_ofdm_symbols) {		// last ofdm symbol in pkt
 	FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  	assert(flowInfo != NULL);

	/* just log *this* innovative packets symbols after correction */	
	if(d_dst && 0) {
	    logCorrectedSymbols(flowInfo);
	}


#ifdef USE_PILOT
	if(d_dst) {
	   demodulate_ILP_2(flowInfo); 
	}
#else	
        //if(isFullRank(flowInfo) && d_dst)
	if(d_dst) 
	{
#ifdef USE_ILP
	    demodulate_ILP_2(flowInfo);	// demodulate_ILP_2(flowInfo); 
	    /*
	    if(d_batch_size == 1)
		decodePayload_single(flowInfo);
	    else
	        demodulate_ILP(flowInfo); */
#else
	    if(d_pktInfo->n_senders == 1)
                decodePayload_single(flowInfo);
	    else
		decodePayload_multiple(flowInfo);
#endif
	}
#endif
	if(d_fwd) 
	{
	   bool isCarrierCorrected = false;
	   isCarrierCorrected = do_carrier_correction();
	   //debug_carrier_correction();
	   if(isCarrierCorrected) {
	      printf("Carrier correction done on the **current** packet\n"); fflush(stdout);
	   } else {
	      printf("Carrier correction failed since there are mutiple senders involved!\n"); fflush(stdout);
	   }

	    printf("check credit: fwd\n"); fflush(stdout);
	    // increment credit counter for the incoming pkt (flow based) //
	    updateCredit();
	    makePacket(true); 
		/*
	    if(checkPacketError(flowInfo) && 0) {
	       updateCredit();
	       makePacket();                                               // REMOVEEEEEEEEEEEE
	    }
	    else {
		// discard complete packet! //
                resetPktInfo(d_pktInfo);                           // non-innovative; screw it
		FlowInfo *flowInfo = getFlowInfo(false, d_flow);
		assert(flowInfo);
		flowInfo->innovative_pkts.pop_back();
		free(d_pktInfo);
		
		int rc_index = flowInfo->reduced_coeffs.size();
		free(flowInfo->reduced_coeffs[rc_index-1]);
		flowInfo->reduced_coeffs.pop_back();
	    } */
	}

	
	// offline analysis  - time domain logging //
 	if(d_log_pkt) {
	  if(d_log_ofdm_index != d_num_ofdm_symbols+d_num_hdr_ofdm_symbols+1) {
		printf("d_log_ofdm_index: %d, d_num_ofdm_symbols: %d, d_num_hdr_ofdm_symbols: %d\n", d_log_ofdm_index, d_num_ofdm_symbols, d_num_hdr_ofdm_symbols); 
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

/* PS: if it is innovative, only then will be called, else this row in the coeff_mat will be overwritten! */
bool
digital_ofdm_frame_sink::isInnovative()
{
  //printf("isInnovative start\n"); fflush(stdout);
  assert(d_nsenders >= 1);

  /* update: after header extension (coeffs per subcarrier), use the same innovative check for e'one) */
  printf("before res\n"); fflush(stdout);
  bool res = isInnovativeAfterReduction();
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

  /* re-initialize d_euclid_dist for the this new batch 
  int n_entries = pow(2.0, double(d_batch_size));
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
      for(unsigned int j = 0; j < n_entries; j++) {
          d_euclid_dist[i][j] = 0.0;
      }
  } */
 
  int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));
  memset(d_euclid_dist, 0, sizeof(float) * d_data_carriers.size() * n_entries * MAX_OFDM_SYMBOLS);

  printf("prepareForNewBatch ends, size: %d, %d\n", flow_info->innovative_pkts.size(), d_flowInfoVector.size()); fflush(stdout);
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

/* used when multiple senders are present 
   update: even with single sender now (after bigger header) */
inline void
digital_ofdm_frame_sink::updateCoeffMatrix(FlowInfo *flowInfo, unsigned int subcarrier_index, cx_mat &coeff_mat)
{
  //printf("updateCoeffMatrix, subcarrier: %d, red_size: %d, \n", subcarrier_index, flowInfo->reduced_coeffs.size()); fflush(stdout);
  // every entry in reduced_coeffs holds is an array of gr_complex (d_batch_size * subcarriers)
  // # of entries in reduced_coeffs = # of innovative packets stored 

  //assert(d_nsenders > 1);
  
  for(unsigned int i = 0;  i < d_batch_size; i++)
  {
     for(unsigned int j = 0; j < d_batch_size; j++)
     {
	gr_complex *coeffs = flowInfo->reduced_coeffs[i];	// get the reduced_coeffs for this packet
	coeff_mat(i, j) = coeffs[subcarrier_index * d_batch_size + j];  // get the coeffs for this subcarrier
     }		
  }

  //printf("updateCoeffMatrix end\n"); fflush(stdout);
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
  int num_links = d_compositeLinkVector.size();
  assert(num_links > 0);
  CompositeLink *cLink = getCompositeLink(d_prevLinkId);
  assert(cLink);
  vector<unsigned int> ids = cLink->dstIds;
  for(int i = 0; i < ids.size(); i++) {
     printf("curr: %d, d_id: %d\n", ids.at(i), d_id); fflush(stdout);
     if(ids.at(i) == d_id) {
	if(d_id == d_header.dst_id)   {
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
  //printf("save_coefficients start\n"); fflush(stdout);
  unsigned char sender_id = d_header.prev_hop_id;
  assert(sender_id >= 0);

  gr_complex *hestimates = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
  memcpy(hestimates, d_in_estimates, sizeof(gr_complex) * d_occupied_carriers);

#ifndef DEBUG
  //printf("save_coeffs - hestimates::: --------------------------------------- \n"); fflush(stdout);
  float atten = 0.0;
  for(int i = 0; i < d_occupied_carriers; i++) {
     atten += abs(hestimates[i]);
     //printf("(atten: %f, phase: %f) <--> complex(%f, %f) \n", abs(hestimates[i]), arg(hestimates[i]), hestimates[i].real(), hestimates[i].imag()); fflush(stdout);
  }
  atten /= ((float) d_occupied_carriers);
  //printf("hestimates end ------------------------------------------------------------ \n"); fflush(stdout);
  printf("sender: %d, avg_atten: %.3f\n", sender_id, atten); fflush(stdout);
#endif

#ifdef LSQ_COMPRESSION
  int num_carriers = d_data_carriers.size();                 // each header will have lsq_coeffs of batch1, batch2 and so on..
  gr_complex *coeffs = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * num_carriers);

  if(d_header.src_id == d_header.prev_hop_id) {
     // if packet is received directly from the source, then no need to do any LSQ //
     for(unsigned int k = 0; k < d_batch_size; k++) {
	for(unsigned int s = 0; s < num_carriers; s++) {
	    int index = k*num_carriers + s;
	    COEFF hdr_coeff = d_header.coeffs[k];
	    coeffs[index] = ToPhase_c(hdr_coeff);
	}
     }
#if 0
  printf("degree: %d\n", d_degree); fflush(stdout);
  for(unsigned i = 0; i < 1; i++) {
     for(unsigned int k = 0; k < d_batch_size; k++) {
        int index = i + k*num_carriers;
        float amp = abs(coeffs[index]);
        float phase = ToPhase_f(coeffs[index]);
        printf("(A: %f, P: %f) <-> (%f, %f)\n", amp, phase, coeffs[index].real(), coeffs[index].imag());
     }
  }
#endif

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

#if 0
  printf("degree: %d\n", d_degree); fflush(stdout);
  for(unsigned i = 0; i < num_carriers; i++) {
     for(unsigned int k = 0; k < d_batch_size; k++) {
	int index = i + k*num_carriers;
	float amp = abs(coeffs[index]);
	float phase = ToPhase_f(coeffs[index]);
	printf("(A: %f, P: %f) <-> (%f, %f)\n", amp, phase, coeffs[index].real(), coeffs[index].imag());
     }
  }
#endif

#else
  // for each subcarrier, record 'd_batch_size' coeffs //
  int num_carriers = d_data_carriers.size()/COMPRESSION_FACTOR;
  gr_complex *coeffs = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * num_carriers);
  memset(coeffs, 0, sizeof(gr_complex) * d_batch_size * num_carriers);

  /*
  int num_carriers = d_data_carriers.size()/2;
  gr_complex *coeffs = d_pktInfo->coeffs.back(); */
  printf("save_coeffs: coeffs on the 1st subcarrier - entries: %d\n", num_carriers); fflush(stdout);
   
  for(unsigned i = 0; i < num_carriers; i++) {
     for(unsigned int k = 0; k < d_batch_size; k++) {
         int index = i * d_batch_size + k;
	 COEFF hdr_coeff = d_header.coeffs[index];
         coeffs[index] = ToPhase_c(hdr_coeff);
#ifdef DEBUG
         float amp = ((float) hdr_coeff.amplitude)/SCALE_FACTOR_AMP;
         float phase = ((float) hdr_coeff.phase)/SCALE_FACTOR_PHASE;
         printf("(A: %f, P: %f) <-> (%f, %f)", amp, phase, coeffs[index].real(), coeffs[index].imag());
#endif
     }
  }
  fflush(stdout); printf("\n"); 
#endif

  d_pktInfo->senders.push_back(sender_id);
  d_pktInfo->coeffs.push_back(coeffs);
  d_pktInfo->hestimates.push_back(hestimates);					// push all the estimates //

  //printf("save_coefficients end for sender:%d\n", sender_id); fflush(stdout);

  /* a packet might contain one or more headers. Each header is first demodulated, and will have its
   own d_phase and d_freq (for equalization). These need to be stored, and used later when the 
   data portion is demodulated */
  d_pktInfo->phase_eq_vec.push_back(d_phase[0]);
  d_pktInfo->freq_eq_vec.push_back(d_freq[0]);

  printf("save_coeffs end\n"); fflush(stdout);
  //debugPktInfo(d_pktInfo, sender_id);
}


// reduce the coefficients to a simpler form, in case multiple senders are present //

// @D: R1 = (h_b * C_b1 + h_c * C_c1) x P1 + (h_b * C_b2 + h_c * C_c2) x P2
//     R2 = (h_b'*C_b1' + h_c'*C_c1') x P1 + (h_b'*C_b2' + h_c'*C_c2') x P2

bool
digital_ofdm_frame_sink::isInnovativeAfterReduction()
{
  int num_senders = 1; //d_pktInfo->n_senders;			// REMOVEEEEEEEEEEEEE
  FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  unsigned int packets_in_batch = flowInfo->innovative_pkts.size();

  /* retrieve flow's coeff matrix */
  cx_mat coeff_mat = flowInfo->coeff_mat;
  unsigned int old_rank = rank(coeff_mat);
  
  /* check the rank from the coeffs devised from just the 1st subcarrier */ 
  for(unsigned int i = 0; i < packets_in_batch; i++) {
      gr_complex reduced_coeff;
      for(int j = 0; j < num_senders; j++)
      {
	  gr_complex *estimates = d_pktInfo->hestimates[j];
	  gr_complex *coeffs = d_pktInfo->coeffs[j];				     	// 'coeffs' has '(batch_size * subcarriers/2)' entries	   		   
	  if(j == 0)
		reduced_coeff = (estimates[0] * coeffs[i]);	    	
	  else
		reduced_coeff += (estimates[0] * coeffs[i]);	
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

      vector<gr_complex*> interpolated_coeffs;
      for(int k = 0; k < num_senders; k++) {
	 gr_complex *rx_coeffs = d_pktInfo->coeffs.at(k);
	 gr_complex *out_coeffs = (gr_complex*) malloc(sizeof(gr_complex) * d_batch_size * d_occupied_carriers);
	 memset(out_coeffs, 0, sizeof(gr_complex) *  d_batch_size * d_occupied_carriers);

	 interpolate_coeffs(rx_coeffs, out_coeffs);				// 
	 interpolated_coeffs.push_back(out_coeffs);
      }	
      
      for(unsigned int i = 0; i < d_occupied_carriers; i++)             // input signature: d_occupied_carriers * hestimates
      {
          gr_complex this_coeff;
          for(unsigned int j = 0; j < d_batch_size; j++)
          {
	      int coeff_index = (i * d_batch_size + j);
	      for(int k = 0; k < num_senders; k++)
              {
                   gr_complex *estimates = d_pktInfo->hestimates[k];			// estimates for 'kth' sender
		   gr_complex *in_coeffs = interpolated_coeffs[k];			// interpolated coeffs for 'kth' sender

                   if(k == 0) {
                        this_coeff = ((gr_complex(1.0, 0.0)/estimates[i]) * in_coeffs[coeff_index]);
		   }
                   else {
                        this_coeff += ((gr_complex(1.0, 0.0)/estimates[i]) * in_coeffs[coeff_index]);
		   }
              }

              // each subcarrier will have 'd_batch_size' # of coefficients //
              coeff[coeff_index] = this_coeff;
          }
      }

      /* clean up */
      for(int i = 0; i < num_senders; i++) {
	 gr_complex *out_coeffs = interpolated_coeffs.at(i);
	 free(out_coeffs);
      }
      interpolated_coeffs.clear();	

      return true;
  }
  return false;
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
	    if(0)
	    {
		printf("o: %d, sc: %d, (%f, %f) = (%f, %f) + (%f, %f)\n", i, j, t.real(), t.imag(), out[index].real(), out[index].imag(), symbols[index].real(), symbols[index].imag()); 
		fflush(stdout);
	    }
	    out[index] = t;
	}
     }

}

/*
inline void
digital_ofdm_frame_sink::normalizeSignalXXX(gr_complex* out, int k)
{
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++)
     for(unsigned int j = 0; j < d_occupied_carriers; j++) {
         unsigned int index = (i*d_occupied_carriers) + j;
#ifdef SCALE
         out[index] /= (gr_complex(k)*gr_complex(SCALE));         // FIXME: cannot / by d_batch_size, must / by innovative pkts used in creating this signal!
#else
	 out[index] /= gr_complex(k);
	 //if(i == 0 && j == 0) 
	 {
		printf("o: %d, sc: %d, normalized signal: (%f, %f) [z=%f, p=%f]\n", i, j, out[index].real(), out[index].imag(), abs(out[index]), arg(out[index])); 
		fflush(stdout);
	 }
#endif
  }
}
*/

/* alternate version: based on the average magnitude seen in 1 OFDM symbol, either amp or de-amp the signal */
inline float
digital_ofdm_frame_sink::normalizeSignal(gr_complex* out, int k, int num_in_senders)
{
  int num_data_carriers = d_data_carriers.size();
#if 0
  float avg_energy = 0.0;
  for(unsigned int j = 0; j < num_data_carriers; j++) {
      avg_energy += abs(out[d_data_carriers[j]]);
  }
  avg_energy /= (float)(num_data_carriers);			   // ignore the DC tones //

  float factor = 0.9/avg_energy;
#endif

  float factor = sqrt(k) * sqrt(num_in_senders);						   // TODO: unsure of what factor to use :| //
  printf("norm factor: %f\n", factor); fflush(stdout);

  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++)
     for(unsigned int j = 0; j < num_data_carriers; j++) {
         unsigned int index = (i*d_occupied_carriers) + d_data_carriers[j];
	 out[index] /= gr_complex(factor);

         /*if(i == 0 && j == 0) 
         {
               printf("o: %d, sc: %d, normalized signal: (%f, %f) [z=%f, p=%f]\n", i, j, out[index].real(), out[index].imag(), abs(out[index]), arg(out[index])); 
               fflush(stdout);
         }*/
  }

  return factor;
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
digital_ofdm_frame_sink::makeHeader(MULTIHOP_HDR_TYPE &header, unsigned char *header_bytes, FlowInfo *flowInfo, unsigned int nextLinkId)
{
   printf("batch#: %d, makeHeader for pkt: %d, len: %d\n", flowInfo->active_batch, d_pkt_num, d_packetlen); fflush(stdout);
   header.src_id = flowInfo->src; 
   header.dst_id = flowInfo->dst;       
   header.prev_hop_id = d_id;	
   header.batch_number = flowInfo->active_batch;

   header.packetlen = d_packetlen;// - 1;                // -1 for '55' appended (TODO)
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
   CompositeLinkVector::iterator it = d_compositeLinkVector.begin();
   while(it != d_compositeLinkVector.end()) {
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
        cInfo->credit = atof(token[1]);

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
   // link-id   #of-src     src1   src2   src3    #of-dst   dst1   dst2    dst3    //
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
           unsigned int srcId = atoi(token_vec[i+2]);
           printf("srcId: %d, size: %d\n", srcId, cLink->srcIds.size()); fflush(stdout);
           cLink->srcIds.push_back(srcId);
        }

        int num_dst = atoi(token_vec[num_src+2]);
        printf("num_dst: %d\n", num_dst); fflush(stdout);
        for(int i = 0; i < num_dst; i++) {
           unsigned int dstId = atoi(token_vec[num_src+3+i]);
           printf("dstId: %d\n", dstId); fflush(stdout);
           cLink->dstIds.push_back(dstId);
        }
        
        d_compositeLinkVector.push_back(cLink);


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
	     creditInfo->credit += (1.0) * (n_senders/((link.srcIds).size())); 
	 }
      }
   }
   assert(updated);
}

/* called from the outside directly by the wrapper, asking for any pkt to forward */
void
digital_ofdm_frame_sink::makePacket(bool sync_send)
{
   printf("makePacket syncSend: %d\n", sync_send); fflush(stdout);
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
	   if(creditInfo->previousLink.linkId == d_prevLinkId) {
		found = true; break;
	   }
	   /*
	   if(creditInfo->credit >= 1.0) {
	 	found = true;
		break;
	   } */
	}
   } 

   /* credit >= 1.0 found, packet needs to be created! */
   if(found){// && num_data_fwd == 0) {
	assert(creditInfo);
	printf("sink::makePacket - DATA to send, flow: %d\n", creditInfo->flowId); fflush(stdout);

        /* create the pkt now! */
        FlowInfo *flowInfo = getFlowInfo(false, creditInfo->flowId);
        assert(flowInfo);

	if(sync_send && 0) {
	   test_sync_send(creditInfo);
	}
	else {
	   encodePktToFwd(creditInfo, sync_send);
	}
	creditInfo->credit -= 1.0;				
	//printf("sink::makePacket - DATA end\n"); fflush(stdout);
	num_data_fwd++;
	return;
   }
   //printf("sink::makePacket - nothing to send\n"); fflush(stdout);
}

#if 0
void
digital_ofdm_frame_sink::printSymbols(gr_complex* symbols, int num) {
  printf("print symbols-- \n");
  for(int j = 0; j < num; j++) {
     printf("(%.3f, %.3f)\n", symbols[j].real(), symbols[j].imag()); fflush(stdout);
  }
  printf("print symbols end --\n"); fflush(stdout);
}

/* forwarder evaluates if it should discard or keep the packet to forward */
void
digital_ofdm_frame_sink::buildIdealPosMap(FlowInfo *flowInfo, vector<comp_d> &ideal_map) {
  unsigned int i = flowInfo->innovative_pkts.size() - 1;
  cx_mat coeffs = flowInfo->coeff_mat;
  if(d_batch_size == 2) {

     ideal_map.push_back(((coeffs(i, 0) * (comp_d) d_sym_position[0])) + (coeffs(i, 1) * (comp_d) d_sym_position[0]));
     ideal_map.push_back(((coeffs(i, 0) * (comp_d) d_sym_position[0])) + (coeffs(i, 1) * (comp_d) d_sym_position[1]));
     ideal_map.push_back(((coeffs(i, 0) * (comp_d) d_sym_position[1])) + (coeffs(i, 1) * (comp_d) d_sym_position[0]));
     ideal_map.push_back(((coeffs(i, 0) * (comp_d) d_sym_position[1])) + (coeffs(i, 1) * (comp_d) d_sym_position[1]));
  }
  else if (d_batch_size == 3) {
     ideal_map.push_back(coeffs(i, 0)* (comp_d)d_sym_position[0] + coeffs(i, 1)* (comp_d)d_sym_position[0] + coeffs(i, 2)* (comp_d)d_sym_position[0]);
     ideal_map.push_back(coeffs(i, 0)* (comp_d)d_sym_position[0] + coeffs(i, 1)* (comp_d)d_sym_position[0] + coeffs(i, 2)* (comp_d)d_sym_position[1]);
     ideal_map.push_back(coeffs(i, 0)* (comp_d)d_sym_position[0] + coeffs(i, 1)* (comp_d)d_sym_position[1] + coeffs(i, 2)* (comp_d)d_sym_position[0]);
     ideal_map.push_back(coeffs(i, 0)* (comp_d)d_sym_position[0] + coeffs(i, 1)* (comp_d)d_sym_position[1] + coeffs(i, 2)* (comp_d)d_sym_position[1]);
     ideal_map.push_back(coeffs(i, 0)* (comp_d)d_sym_position[1] + coeffs(i, 1)* (comp_d)d_sym_position[0] + coeffs(i, 2)* (comp_d)d_sym_position[0]);
     ideal_map.push_back(coeffs(i, 0)* (comp_d)d_sym_position[1] + coeffs(i, 1)* (comp_d)d_sym_position[0] + coeffs(i, 2)* (comp_d)d_sym_position[1]);
     ideal_map.push_back(coeffs(i, 0)* (comp_d)d_sym_position[1] + coeffs(i, 1)* (comp_d)d_sym_position[1] + coeffs(i, 2)* (comp_d)d_sym_position[0]);
     ideal_map.push_back(coeffs(i, 0)* (comp_d)d_sym_position[1] + coeffs(i, 1)* (comp_d)d_sym_position[1] + coeffs(i, 2)* (comp_d)d_sym_position[1]);
  }
}

inline bool 
digital_ofdm_frame_sink::isSymbolError(gr_complex& x, vector<comp_d> ideal_pos_map) {
  unsigned int table_size = ideal_pos_map.size();
  unsigned int min_index = 0;
  float min_euclid_dist = norm((comp_d) x - ideal_pos_map[0]);
  float euclid_dist = 0;

  for (unsigned int j = 1; j < table_size; j++){
    euclid_dist = norm((comp_d) x - ideal_pos_map[j]);
    if (euclid_dist < min_euclid_dist){
      min_euclid_dist = euclid_dist;
      min_index = j;
    }
  }

  if(min_euclid_dist < 100) {
    x = ideal_pos_map[min_index];
    return true;
  }
  return false;
}

inline bool
digital_ofdm_frame_sink::isFECGroupCorrect(int mod_symbols_group, 
					   gr_complex *rx_symbols, vector<comp_d> ideal_pos_map) {
 
  int FEC_N = 255; int SYM_SIZE = 8;
  int FEC_K = 223;

  // get # of FEC symbols in the current FEC_group //
  int num_fec_symbols = (mod_symbols_group*d_nbits)/SYM_SIZE;
  assert((mod_symbols_group*d_nbits) % SYM_SIZE == 0);

  // guarantee integer # of mod_symbols in FEC_symbol //
  int mod_symbols_in_FEC_symbol = SYM_SIZE/d_nbits;
  assert(SYM_SIZE % d_nbits == 0); 			// will not work with qam8, qam64

  // if a mod_symbol is in error, mark the entire FEC symbol in error //
  int n_errors = 0;
  for(int i = 0; i < num_fec_symbols; i++) {
     for(int j = 0; j < mod_symbols_in_FEC_symbol; j++) {
	 int index = (i * mod_symbols_in_FEC_symbol) + j;
	 if(isSymbolError(rx_symbols[index], ideal_pos_map)) {
		n_errors++;
		break;
	 }
     }
  }

  if(n_errors > (floor) ((float) FEC_N - FEC_K)/2)
	return false;

  return true;
}

bool
digital_ofdm_frame_sink::checkPacketError(FlowInfo *flow_info) 
{
  vector<comp_d> ideal_pos_map;
  buildIdealPosMap(flow_info, ideal_pos_map);
  
  gr_complex *rx_symbols = d_pktInfo->symbols;
  int num_symbols = d_num_ofdm_symbols * d_occupied_carriers;

  int FEC_N = 255; int SYM_SIZE = 8;

  int mod_symbols_in_FEC_group = (FEC_N * SYM_SIZE) / d_nbits;
  int num_fec_groups = ceil(((float)num_symbols)/mod_symbols_in_FEC_group);

  for(int i = 0; i < num_fec_groups; i++) {
     int mod_symbols_this_group = mod_symbols_in_FEC_group;
     if(i == num_fec_groups - 1) {
	 mod_symbols_this_group = num_symbols - (i*mod_symbols_in_FEC_group);
     }
 
     int index = i * mod_symbols_in_FEC_group;
     if(!isFECGroupCorrect(mod_symbols_this_group, &rx_symbols[index], ideal_pos_map)) {
	return false;
     }
  }

  return true;
}
#endif

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

#if 0
/* Store the coeffs in the header (reduced with earlier rx coeffs) 
   need to scale up the amplitude and phase (in degrees) and store in the header */
inline void 
digital_ofdm_frame_sink::packCoefficientsInHeader1(MULTIHOP_HDR_TYPE& header, 
					          gr_complex* coeffs, FlowInfo *flowInfo) 
{
  printf("packCoefficientsInHeader1\n"); fflush(stdout);
  /* do for each subcarrier, since 'h' values are on subcarrier basis
     - half the subcarriers are really needed, the other half are interpolated */
  int num_coeffs_batch = d_data_carriers.size()/2;
  assert(d_data_carriers.size() % 2 == 0);

  COEFF *new_coeffs = (COEFF*) malloc(sizeof(COEFF) * d_batch_size * num_coeffs_batch);
  memset(new_coeffs, 0, sizeof(COEFF) * d_batch_size * num_coeffs_batch);

  int count = 0;
  InnovativePktInfoVector inno_pkts = flowInfo->innovative_pkts;

  /* how many subcarriers to fill */
  int dc_tones = d_occupied_carriers - d_data_carriers.size();
  unsigned int half_occupied_tones = (d_occupied_carriers - dc_tones)/2;

  /* each 'subcarrier' contains 'd_batch_size' coeffs for each subcarrier */
  for(unsigned s = 0; s < d_occupied_carriers; s+=2) {
 
     if(s >= half_occupied_tones && s < half_occupied_tones + dc_tones) {
	 continue;								// bypass the DC tones //
     }

     /* generate <d_batch_size> coeffs */
     for(unsigned int i = 0; i < d_batch_size; i++) {
        gr_complex new_coeff_c;                                    		// for each native pkt in batch, ie P1 and P2 //

        for(int j = 0; j < inno_pkts.size(); j++) {
	   gr_complex *pkt_coeffs = flowInfo->reduced_coeffs[j];	// reduced coeffs for this inno pkt //

	   int coeff_index = s * d_batch_size + i;
	   if(j == 0) {
		new_coeff_c = coeffs[j] * pkt_coeffs[coeff_index];
		//printf("(%f, %f) = (%f, %f) * (%f, %f)\n", new_coeff_c.real(), new_coeff_c.imag(), coeffs[j].real(), coeffs[j].imag(), pkt_coeffs[coeff_index].real(), pkt_coeffs[coeff_index].imag()); fflush(stdout);
	   }
	   else {
		new_coeff_c += (coeffs[j] * pkt_coeffs[coeff_index]);
	   }
        }

	new_coeffs[count].amplitude = abs(new_coeff_c) * SCALE_FACTOR_AMP;
	new_coeffs[count].phase = ToPhase_f(new_coeff_c) * SCALE_FACTOR_PHASE;

	//printf("= [z=%f p=%f] ", abs(new_coeff_c), ToPhase_f(new_coeff_c)); fflush(stdout);

	count++;
     }
  }

  if(count != (d_batch_size * num_coeffs_batch)) {
	printf("count: %d, num_coeffs_batch: %d, batch_size: %d\n", count, num_coeffs_batch, d_batch_size); fflush(stdout);
	assert(false);
  }

  memcpy(header.coeffs, new_coeffs, sizeof(COEFF) * count);
  free(new_coeffs);
}

/* used by forwarder: fixes the symbols of each innovative packet to the closest possible locations instead */
void
digital_ofdm_frame_sink::correctStoredSignal(FlowInfo *flowInfo) 
{
  printf("correctOutgoingSignal start\n"); fflush(stdout);
  // prep for building the map //
  vector<vector<gr_complex*> > batched_sym_position;

  int dc_tones = d_occupied_carriers - d_data_carriers.size();
  unsigned int half_occupied_tones = (d_occupied_carriers - dc_tones)/2;

  for(unsigned int i = 0; i < d_occupied_carriers; i++) {

        if(i >= half_occupied_tones && i < half_occupied_tones + dc_tones) {
            continue;                                                                                   // bypass the DC tones //
        }
	
	cx_mat coeff_mat = flowInfo->coeff_mat;
        updateCoeffMatrix(flowInfo, i, coeff_mat);

        // build the map for each batch on each subcarrier //
        vector<gr_complex*> sym_position;
        buildMap_ILP(coeff_mat, sym_position);
        batched_sym_position.push_back(sym_position);
  }

  InnovativePktInfoVector innovativePkts = flowInfo->innovative_pkts;
  //debugMap_ILP(batched_sym_position, innovativePkts.size()); 
 
  assert(batched_sym_position.size() == d_data_carriers.size());
  vector<gr_complex> dfe_vec[MAX_BATCH_SIZE];
  gr_complex* sym_vec[MAX_BATCH_SIZE];
  gr_complex* closest_sym_vec[MAX_BATCH_SIZE];

  // initialization //
  for(unsigned int k = 0; k < innovativePkts.size(); k++) {
     dfe_vec[k].resize(d_occupied_carriers);
     fill(dfe_vec[k].begin(), dfe_vec[k].end(), gr_complex(1.0,0.0));

     sym_vec[k] = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
     closest_sym_vec[k] = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
  }
  
  // signal correction //
  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
      gr_complex carrier[MAX_BATCH_SIZE], accum_error[MAX_BATCH_SIZE];
      for(unsigned int i = 0; i < innovativePkts.size(); i++) {	
          PktInfo *pInfo = innovativePkts[i];
	  gr_complex *rx_symbols_this_batch = pInfo->symbols;

	  /* TODO what about when multiple senders are present!! */
	  d_phase[i] = (pInfo->phase_eq_vec)[0];
	  d_freq[i] = (pInfo->freq_eq_vec)[0];

	  carrier[i] = gr_expj(d_phase[i]);
	  accum_error[i] = 0.0;

	  memset(closest_sym_vec[i], 0, sizeof(gr_complex) * d_occupied_carriers);
	  memcpy(sym_vec[i], &rx_symbols_this_batch[o * d_occupied_carriers], sizeof(gr_complex) * d_occupied_carriers);	
      }

      gr_complex sigrot[MAX_BATCH_SIZE]; 
      for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
	  // find sigrot for each batch //
          for(unsigned int k = 0; k < innovativePkts.size(); k++) {
              sigrot[k] = sym_vec[k][d_data_carriers[i]] * carrier[k] * dfe_vec[k][i];
          }

          findClosestSymbol(sigrot, closest_sym_vec, innovativePkts.size(), batched_sym_position[i], i, o);

	  // update accum_error, dfe_vec //
	  for(unsigned int k = 0; k < innovativePkts.size(); k++) {
             accum_error[k] += (sigrot[k] * conj(closest_sym_vec[k][d_data_carriers[i]]));
             if (norm(sigrot[k])> 0.001) dfe_vec[k][i] +=  d_eq_gain*(closest_sym_vec[k][d_data_carriers[i]]/sigrot[k]-dfe_vec[k][i]);
          }
      } 

      // update the PLL settings //
      for(unsigned int k = 0; k < innovativePkts.size(); k++) {

	 // update outgoing with closest symbol //
	 PktInfo *pInfo = innovativePkts[k];
	 gr_complex *rx_symbols_this_batch = pInfo->symbols;
	 memcpy(&rx_symbols_this_batch[o * d_occupied_carriers], closest_sym_vec[k], sizeof(gr_complex) * d_occupied_carriers);

         float angle = arg(accum_error[k]);
	 d_freq[k] = d_freq[k] - d_freq_gain*angle;
	 d_phase[k] = d_phase[k] + d_freq[k] - d_phase_gain*angle;
	 if (d_phase[k] >= 2*M_PI) d_phase[k] -= 2*M_PI;
	 if (d_phase[k] <0) d_phase[k] += 2*M_PI;

	 pInfo->phase_eq_vec[0] = d_phase[k];
	 pInfo->freq_eq_vec[0] = d_freq[k];
      }
  }
  
  // cleanup //
  for(unsigned int k = 0; k < innovativePkts.size(); k++) {
        free(sym_vec[k]);
        free(closest_sym_vec[k]);

	// also reset the state of d_phase, d_freq for each, since these symbols have already been *corrected* //
	PktInfo *pInfo = innovativePkts[k];
	(pInfo->phase_eq_vec)[0] = 0.0;
	(pInfo->freq_eq_vec)[0] = 0.0;
  }

  // debug //
  if(0) {
  printf("Corrected signals .. \n"); fflush(stdout);
  for(unsigned int k = 0; k < innovativePkts.size(); k++) {
      PktInfo *pInfo = innovativePkts[k];
      gr_complex *rx_symbols = pInfo->symbols;

      for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
 	  for(unsigned int i = 0; i < d_occupied_carriers; i++) {
		if(i >= half_occupied_tones && i < half_occupied_tones + dc_tones) {
	            continue;                                                                                   // bypass the DC tones //
          	}
		gr_complex _symbol = rx_symbols[o*d_occupied_carriers+i];
		printf("k: %d, o: %d, sc: %d, (%f, %f)\n", k, o, i, _symbol.real(), _symbol.imag());
		fflush(stdout);
	  }
      }
  }
  }
}

void
digital_ofdm_frame_sink::findClosestSymbol(gr_complex *x, gr_complex **closest_sym_vec, int num_pkts,
                                    vector<gr_complex*> batched_sym_position, unsigned int subcarrier_index, unsigned int o_index) {
  unsigned int min_index = 0;
  float min_euclid_dist = 0.0;

  // for each table entry, find the min(error, k) //
  unsigned int table_size = pow(2.0, double(d_batch_size));
  for(unsigned int k = 0; k < num_pkts; k++) {

      // initialize //
      min_euclid_dist = norm(x[k] - batched_sym_position[k][0]);

      float euclid_dist = 0.0;
      for(unsigned int j = 1; j < table_size; j++) {
	 euclid_dist = norm(x[k] - batched_sym_position[k][j]);
	 if (euclid_dist < min_euclid_dist) {
	     min_euclid_dist = euclid_dist;
	     min_index = j;
	 }
      }

      closest_sym_vec[k][d_data_carriers[subcarrier_index]] = batched_sym_position[k][min_index];
      if(o_index == 0 && subcarrier_index == 0) {
	 printf("closest match for (%f, %f) ----> (%f, %f) \n", x[k].real(), x[k].imag(), batched_sym_position[k][min_index].real(), batched_sym_position[k][min_index].imag());
	 fflush(stdout);
      }

      // reinit min_index for next batch //
      min_index = 0;
  }
}
#endif

inline void
digital_ofdm_frame_sink::calculate_fine_offset() {
  
  int num = NULL_OFDM_SYMBOLS;		//2;
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
     int gap = NULL_OFDM_SYMBOLS;               // t'-t

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
     int gap = NULL_OFDM_SYMBOLS;               // t'-t

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
     for(int i = d_num_ofdm_symbols; i < NULL_OFDM_SYMBOLS + d_num_ofdm_symbols; i++) {
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
     float end_angle = arg(phase_error);
     //printf("\nangle: %.3f", end_angle);
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

  /*
  if(d_replay_flag != CORRECTED_REPLAY) 
     correctStoredSignal(flow_info);
  */

  /* coefficients */
  unsigned int n_innovative_pkts = inno_pkts.size();
  assert(n_innovative_pkts > 0);
  gr_complex *coeffs = (gr_complex*) malloc(sizeof(gr_complex) * n_innovative_pkts);
  memset(coeffs, 0, sizeof(gr_complex) * n_innovative_pkts);

  /* final coded symbols */
  unsigned int n_symbols = d_num_ofdm_symbols * d_occupied_carriers;

  gr_complex *out_symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_symbols);
  memset(out_symbols, 0, sizeof(gr_complex) * n_symbols);

  /* pick new random <coeffs> for each innovative pkt to == new coded pkt */
  printf("selecting random coeffs::: --- \n"); fflush(stdout);
  float phase[MAX_BATCH_SIZE];
  for(unsigned int i = 0; i < n_innovative_pkts; i++) {
     PktInfo *pInfo = inno_pkts[i];

     gr_complex *symbols = (gr_complex*) malloc(sizeof(gr_complex) * n_symbols);
     memcpy(symbols, pInfo->symbols, sizeof(gr_complex) * n_symbols);

     phase[i] = rand() % 360 + 1;
     float amp = getAvgAmplificationFactor(pInfo->hestimates);;
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

  /* populate the 'header' struct and the 'header_bytes' */
  makeHeader(header, header_bytes, flow_info, creditInfo->nextLink.linkId);

  /* final message (type: 1-coded) */
  /* DO NOT include the DC tones */
  //unsigned int n_actual_symbols = (d_num_ofdm_symbols+1) * d_occupied_carriers;		// extra for NULL symbol at the end
  unsigned int n_actual_symbols = (d_num_ofdm_symbols) * d_occupied_carriers;
  gr_message_sptr out_msg = gr_make_message(DATA_TYPE, 0, 0, HEADERBYTELEN + (n_actual_symbols * sizeof(gr_complex)));
  memcpy(out_msg->msg(), header_bytes, HEADERBYTELEN);					            // copy header bytes

  //printf("here, d_packetlen: %d, HEADERBYTELEN: %d, n_actual_symbols: %d\n", d_packetlen, HEADERBYTELEN, n_actual_symbols); fflush(stdout);

  int offset = HEADERBYTELEN;
  //memcpy(out_msg->msg() + offset, &out_symbols, n_actual_symbols * sizeof(gr_complex));

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
  
  // extra for null symbol at the end //
  //memset((out_msg->msg() + offset), 0, d_occupied_carriers/8);

/*
#ifdef USE_PILOT
  printf("pilots first: \n"); fflush(stdout);
  assert(d_all_carriers.size() == d_occupied_carriers);
  for(unsigned int i = 0; i < d_num_ofdm_symbols; i++) {
     unsigned int index = i * d_occupied_carriers;
     for(unsigned int j = 0; j < d_occupied_carriers; j++) {
#ifdef SRC_PILOT
	if(1) {
#else
	if(d_all_carriers[j] == 0) {
#endif
	    memcpy((out_msg->msg() + offset), (out_symbols + index + j), sizeof(gr_complex));
	    if(d_all_carriers[j] == 1) {
		std::cout<<out_symbols[index+j]<<" offset: "<<offset<<endl;
	    }
	    offset += sizeof(gr_complex);
	}
     }
  }
#endif
*/

#if 0
  int dc_tones = d_occupied_carriers - d_data_carriers.size();
  assert(dc_tones > 0 && (d_occupied_carriers % 2 == 0) && (dc_tones % 2 == 0));                // even occupied_tones and dc_tones //
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

  //logSymbols(out_symbols, n_symbols);

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

  // set the tx timestamp, if its a sync send //
  if(sync_send) {
     uint64_t sync_secs; 
     double sync_frac_of_secs;
     calc_outgoing_timestamp(sync_secs, sync_frac_of_secs);

     printf("RX timestamp: secs: %llu, frac_of_secs: %f\n", sync_secs, sync_frac_of_secs); fflush(stdout);
     out_msg->set_timestamp(sync_secs, sync_frac_of_secs);
     printf("FWD PACKET---- TX timestamp: secs: %llu, frac_of_secs: %f\n", sync_secs, sync_frac_of_secs); 
     fflush(stdout);
  }

  d_out_queue->insert_tail(out_msg);
  out_msg.reset();

  free(coeffs);
  free(out_symbols);
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


#if 0
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
#endif

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

#ifdef SEND_ACK_ETHERNET
int
digital_ofdm_frame_sink::create_ack_sock() {
  int sockfd;
  struct sockaddr_in dest;

  /* create socket */
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if(sockfd != -1) {
    printf("@ socket created at the destination SUCCESS\n"); fflush(stdout);
  }

  /* initialize value in dest */
  bzero(&dest, sizeof(struct sockaddr_in));
  dest.sin_family = PF_INET;
  dest.sin_port = htons(d_src_sock_port);
  dest.sin_addr.s_addr = inet_addr(d_src_ip_addr);
  /* Connecting to server */
  connect(sockfd, (struct sockaddr*)&dest, sizeof(struct sockaddr_in));
  return sockfd;
}

#else

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

#if 0
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
  dewhiten(d_header_bytes, ACK_HEADERBYTELEN);
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
#endif

void
digital_ofdm_frame_sink::whiten(unsigned char *bytes, const int len)
{
   for(int i = 0; i < len; i++)
        bytes[i] = bytes[i] ^ random_mask_tuple[i];
}


#if 0
/*
// Integer Programming //
*/
void
digital_ofdm_frame_sink::slicer_ILP(gr_complex *x, gr_complex *closest_sym, unsigned char *bits, 
				    vector<gr_complex*> batched_sym_position, unsigned int ofdm_index, unsigned int subcarrier_index,
				    gr_complex** sym_vec)
{
  unsigned int min_index = 0;
  float min_euclid_dist = 0.0;

  // initialize //
  for(unsigned int k = 0; k < d_batch_size; k++) {
     min_euclid_dist += norm(x[k] - batched_sym_position[k][0]);
     /*
     if(ofdm_index == 5 && (subcarrier_index == 32 || subcarrier_index == 57 || subcarrier_index == 63) && 0) {
	  gr_complex orig_x = sym_vec[k][d_data_carriers[subcarrier_index]];
          printf("j: 0, k: %d, x: (%f, %f), pos: (%f, %f) dist: %f, orig_x: (%f, %f)\n", k, x[k].real(), x[k].imag(), batched_sym_position[k][0].real(), batched_sym_position[k][0].imag(), norm(x[k] - batched_sym_position[k][0]), orig_x.real(), orig_x.imag());
          }
      */
  }
 
  //printf("initialized, min_euclid_dist: %f\n", min_euclid_dist); fflush(stdout); 
  
  // for each table entry, find the min(total_error, k) //
  unsigned int table_size = pow(2.0, double(d_batch_size));
  for(unsigned int j = 1; j < table_size; j++) {
      float euclid_dist = 0;
      for(unsigned int k = 0; k < d_batch_size; k++) {
	  euclid_dist += norm(x[k] - batched_sym_position[k][j]);
	  /*
	  if(ofdm_index == 5 && (subcarrier_index == 32 || subcarrier_index == 57 || subcarrier_index == 63) && 0) {
		gr_complex orig_x = sym_vec[k][d_data_carriers[subcarrier_index]];
		printf("j: %d, b: %d, x: (%f, %f), pos: (%f, %f) dist: %f orig_x: (%f, %f)\n", j, k, x[k].real(), x[k].imag(), batched_sym_position[k][j].real(), batched_sym_position[k][j].imag(), norm(x[k] - batched_sym_position[k][j]), orig_x.real(), orig_x.imag());
	  }*/
      }

      if (euclid_dist < min_euclid_dist) {
           min_euclid_dist = euclid_dist;
           min_index = j;
       }
  }

  /*
  if(ofdm_index == 5 && (subcarrier_index == 32 || subcarrier_index == 57 || subcarrier_index == 63) && 0)
          printf("ofdm_index: %d, subcarrier_index: %d, min_euclid_dist: %f, min_index: %d\n", ofdm_index, subcarrier_index, min_euclid_dist, min_index);
          fflush(stdout);
  */

  //printf("min_euclid_dist: %f\n", min_euclid_dist); fflush(stdout);  
 
  // assign closest_sym and bits // 
  for(unsigned int k = 0; k < d_batch_size; k++) {
	closest_sym[k] = batched_sym_position[k][min_index];		// closest_sym: the ideal position where R should hv been at //
	getSymOutBits_ILP(bits, min_index);
  }

  /*    // debug //
  if(ofdm_index == 5 && (subcarrier_index == 32 || subcarrier_index == 57 || subcarrier_index == 63) && 0) {
          printf("ofdm_index: %d, subcarrier_index: %d, min_euclid_dist: %f, min_index: %d, bit: %d, x: (%f, %f), closest_sym: (%f, %f) || x: (%f, %f), closest_sym: (%f, %f)\n", ofdm_index, subcarrier_index, min_euclid_dist, min_index, bits[0], x[0].real(), x[0].imag(), closest_sym[0].real(), closest_sym[0].imag(), x[1].real(), x[1].imag(), closest_sym[1].real(), closest_sym[1].imag());
          fflush(stdout);
  }*/
}

inline void
digital_ofdm_frame_sink::getCandidateSymbols(gr_complex *x, vector<gr_complex*> batched_sym_position, 
					     vector<int> &filtered_batches)
{
   unsigned int table_size = pow(2.0, double(d_batch_size));

   // calculate the threshold reqd for filtering //
   float threshold[MAX_BATCH_SIZE];
   memset(threshold, 0, sizeof(float) * MAX_BATCH_SIZE);

   // min distance in the current table b/w 2 constellation points in each batch //
   float avg_dist[MAX_BATCH_SIZE];
   memset(avg_dist, 0, sizeof(float) * MAX_BATCH_SIZE);

   for(unsigned int k = 0; k < d_batch_size; k++) {
      int min_i = 0; 
      int min_j = 0;
      for(unsigned int i = 0; i < table_size; i++) {
         for(unsigned int j = i+1; j < table_size; j++) {
	    float dist = norm(batched_sym_position[k][i] - batched_sym_position[k][j])/2.0;
	    avg_dist[k] += norm(batched_sym_position[k][i] - batched_sym_position[k][j]);

	    if(dist < threshold[k] || (j==1)) {
		threshold[k] = dist;
		min_i = i;
		min_j = j;
	    }
	 }
      } 	

      int num = (table_size) * (table_size-1)/2;				// # of handshakes
      avg_dist[k] = avg_dist[k]/(float)num;						// avg dist between 2 constellation points
	
      if(1) {
	  gr_complex pos_i = batched_sym_position[k][min_i];
	  gr_complex pos_j = batched_sym_position[k][min_j];
          printf("active_batch: %d, batch: %d threshold: %f s1(%f, %f) s2(%f, %f)\n", d_active_batch, k, threshold[k], pos_i.real(), pos_i.imag(), pos_j.real(), pos_j.imag()); fflush(stdout);
      }
   } 
 
   if(1) {
	printf("\nprinting table.. active_batch: %d\n", d_active_batch); fflush(stdout);
	for(unsigned int k = 0; k < d_batch_size; k++) {
	    printf("for batch: %d, num_entries: %d\n", k, table_size); fflush(stdout);
	    for(unsigned int j = 0; j < table_size; j++) {
		printf("(%f, %f)\n", batched_sym_position[k][j].real(), batched_sym_position[k][j].imag()); 
	    }
	    printf("-------------------------\n"); fflush(stdout);
	}
   }

 
   // among all the batches, include only those, for which the closest sym is within the threshold //
   float min_batch = 0;
   float overall_min_dist = norm(x[0] - batched_sym_position[0][0]);

   for(unsigned int k = 0; k < d_batch_size; k++) {
      float min_euclid_dist = norm(x[k] - batched_sym_position[k][0]);
      for(unsigned int j = 1; j < table_size; j++) {
	 float dist = norm(x[k] - batched_sym_position[k][j]);
	 if(dist < min_euclid_dist) {
		min_euclid_dist = dist;
	 }
      } 

      // threshold check //
      printf("\nactive_batch: %d min_euclid_dist: %f, threshold: %f\n", d_active_batch, min_euclid_dist, threshold[k]); fflush(stdout);
      if(min_euclid_dist < threshold[k] || min_euclid_dist < avg_dist[k]) {
	  filtered_batches.push_back(k);
      }

      if(min_euclid_dist < overall_min_dist) {
	 overall_min_dist = min_euclid_dist;
	 min_batch = k;
      }
   }

   

   // if none of the batches managed to make it, at least include the best of the worst //
   if(filtered_batches.size() == 0) {
	printf("active_batch: %d, OOPS.. size=0, so pushing best batch: %d\n", d_active_batch, min_batch); fflush(stdout);
	filtered_batches.push_back(min_batch);
   }
}

/* the optimized version, where we only work on the symbols we're resonably confident in */
void
digital_ofdm_frame_sink::slicer_ILP_opt(gr_complex *x, gr_complex *closest_sym, unsigned char *bits,
                                    vector<gr_complex*> batched_sym_position, unsigned int ofdm_index, unsigned int subcarrier_index,
                                    gr_complex** sym_vec)
{
  unsigned int min_index = 0;
  float min_euclid_dist = 0.0;

  // scale //
  /*
  for(unsigned int k = 0; k < d_batch_size; k++) {
	x[k] *= gr_complex(SCALE, SCALE);
  }*/


  // consider symbols of batches for which the euclidean dist is within a threshold //
  vector<int> filt_batches;
  getCandidateSymbols(x, batched_sym_position, filt_batches);
  int size = filt_batches.size();

#ifdef DEBUG
  printf("active_batch: %d, slicer_ILP_opt: filtered size: %d\n\n", d_active_batch, filt_batches.size()); fflush(stdout);
#endif

  // initialize using the first table entry //
  for(unsigned int k = 0; k < size; k++) {
     int b = filt_batches[k];
     min_euclid_dist += norm(x[b] - batched_sym_position[b][0]);
  }

  // for each table entry, find the min(total_error, k) //
  unsigned int table_size = pow(2.0, double(d_batch_size));
  for(unsigned int j = 1; j < table_size; j++) {
      float euclid_dist = 0.0;
      for(unsigned int k = 0; k < size; k++) {
	  int b = filt_batches[k];
          euclid_dist += norm(x[b] - batched_sym_position[b][j]);
      }

      if (euclid_dist < min_euclid_dist) {
           min_euclid_dist = euclid_dist;
           min_index = j;
       }
  }

  // demap into bits //
  getSymOutBits_ILP(bits, min_index);
#ifdef DEBUG
  printf("\nmin_euclid_dist: %f, min_index: %d\n", min_euclid_dist, min_index); fflush(stdout);
#endif
  // assign closest_sym // 
  for(unsigned int k = 0; k < d_batch_size; k++)
        closest_sym[k] = batched_sym_position[k][min_index];            // closest_sym: the ideal position where R should hv been at //

}
#endif

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

#if 0
void
digital_ofdm_frame_sink::demodulate_ILP(FlowInfo *flowInfo)
{
  printf("demodulate_ILP\n"); fflush(stdout);
  // initialize for every batch //
  vector<unsigned char*> bytes_out_vec;

  int n_data_carriers = d_data_carriers.size();
  for(unsigned int k = 0; k < d_batch_size; k++) {
     unsigned char *bytes = (unsigned char*) malloc(sizeof(unsigned char) * d_occupied_carriers);		// take all occupied carriers (including pilot/dc)
     bytes_out_vec.push_back(bytes);
  }

  // Since the 'h' values change per subcarrier, reduced coeffs are per subcarrier, hence, there
  // the total # of maps = batch_size * occupied_carriers & each map will have 2^batch_size entries
  vector<vector<gr_complex*> > batched_sym_position;
  for(unsigned int i = 0; i < n_data_carriers; i++) {
     int subcarrier_index = d_data_carriers[i];
     cx_mat coeff_mat = flowInfo->coeff_mat;
     updateCoeffMatrix(flowInfo, subcarrier_index, coeff_mat);

     // build the map for each batch on each subcarrier //
     vector<gr_complex*> sym_position;
     buildMap_ILP(coeff_mat, sym_position);
     batched_sym_position.push_back(sym_position);
  }
  assert(batched_sym_position.size() == d_data_carriers.size());
  
  //debugMap_ILP(batched_sym_position, d_batch_size); 

  // run the demapper for every OFDM symbol (all batches within the symbol are demapped at once!) //
  int packetlen_cnt[MAX_BATCH_SIZE];
  memset(packetlen_cnt, 0, sizeof(int) * MAX_BATCH_SIZE);

  vector<gr_complex> dfe_vec[MAX_BATCH_SIZE];
  for(unsigned int k = 0; k < MAX_BATCH_SIZE; k++) {
     //dfe_vec[k].resize(d_occupied_carriers);
     dfe_vec[k].resize(n_data_carriers);
     fill(dfe_vec[k].begin(), dfe_vec[k].end(), gr_complex(1.0,0.0));
  }
  

  unsigned char packet[MAX_BATCH_SIZE][MAX_PKT_LEN];
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
     free(bytes_out_vec[i]);
  }
  bytes_out_vec.clear();

  /* batched_sym_position */
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
     for(unsigned int k = 0; k < d_batch_size; k++) {
	   gr_complex *sym_position =  batched_sym_position[i][k];
	   free(sym_position);
     }
  }
 
  d_last_batch_acked = d_active_batch;
}


// sym_vec: input to demapper; each row in the vector belongs to a batch
// out_vec: output bytes; each row belongs to a batch 
unsigned int
digital_ofdm_frame_sink::demapper_ILP(unsigned int ofdm_symbol_index, vector<unsigned char*> out_vec, 
				      vector<vector<gr_complex*> > batched_sym_position, FlowInfo *flowInfo, 
				      vector<gr_complex> *dfe_vec) {

  //printf("demapper_ILP, ofdm_symbol_index: %d\n", ofdm_symbol_index); fflush(stdout);
  unsigned int bytes_produced = 0;

  gr_complex carrier[MAX_BATCH_SIZE], accum_error[MAX_BATCH_SIZE], accum_error1[MAX_BATCH_SIZE], carrier1[MAX_BATCH_SIZE];
  assert(d_batch_size <= MAX_BATCH_SIZE);

  gr_complex* sym_vec[MAX_BATCH_SIZE];

  InnovativePktInfoVector innovativePkts = flowInfo->innovative_pkts;

  float est_angle = 0.0, est_phase = 0.0, est_freq = 0.0;
  for(unsigned int i = 0; i < d_batch_size; i++) {
     PktInfo *pInfo = innovativePkts[i];
     gr_complex *rx_symbols_this_batch = pInfo->symbols;

     // need to check how it works (works only for 1 sender) //
     d_phase[i] = (pInfo->phase_eq_vec)[0];
     d_freq[i] = (pInfo->freq_eq_vec)[0];

     // from the slope of the angle1, estimate the angle after the *gap*, and deduce d_phase and d_freq //
     if(ofdm_symbol_index == 0 && NULL_OFDM_SYMBOLS != 0) {
	int gap = NULL_OFDM_SYMBOLS;
	est_angle = d_slope_angle1 * gap + d_end_angle;

	est_angle = d_slope_angle1 * gap + d_end_angle;	
	est_freq = d_freq[i] - d_freq_gain * est_angle;
	est_phase = d_phase[i] + est_freq - d_phase_gain * est_angle;


	if(est_phase >= 2*M_PI) est_phase -= 2*M_PI;
	if(est_phase < 0) est_phase += 2*M_PI;

	d_phase[i] = est_phase; d_freq[i] = est_freq;			// get the d_phase1 and d_freq1 values after the gap, to compare with the actual d_phase and d_freq values

	printf("estimated angle: %f, d_phase: %f, d_freq: %f\n", est_angle, est_phase, est_freq); fflush(stdout);
     }

     carrier[i] = gr_expj(d_phase[i]); carrier1[i] = gr_expj(d_phase1[i]);
     accum_error[i] = 0.0; accum_error1[i] = 0.0;

     sym_vec[i] = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
     memset(sym_vec[i], 0, sizeof(gr_complex) * d_occupied_carriers);
     memcpy(sym_vec[i], &rx_symbols_this_batch[ofdm_symbol_index * d_occupied_carriers], sizeof(gr_complex) * d_occupied_carriers);
  }


  gr_complex sigrot[MAX_BATCH_SIZE], closest_sym[MAX_BATCH_SIZE];

  gr_complex sigrot1[MAX_BATCH_SIZE];

  unsigned int partial_byte[MAX_BATCH_SIZE];
  unsigned char bits[MAX_BATCH_SIZE];

  memset(partial_byte, 0, sizeof(unsigned int) * MAX_BATCH_SIZE);
  memset(bits, 0, sizeof(unsigned char) * MAX_BATCH_SIZE);

  gr_complex *log_closest_sym = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
  memset(log_closest_sym, 0, sizeof(gr_complex) * d_occupied_carriers);


  unsigned int byte_offset = 0;
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {

     // demodulate 1-byte at a time //
     if (byte_offset < 8) {

	  // find sigrot for each batch //
          for(unsigned int k = 0; k < d_batch_size; k++) {
	      if(d_replay_flag != CORRECTED_REPLAY) { 
	          sigrot[k] = sym_vec[k][d_data_carriers[i]] * carrier[k] * dfe_vec[k][i];
		  sigrot1[k] = sym_vec[k][d_data_carriers[i]] * carrier1[k] * dfe_vec[k][i];
	      }
	      else {
		  /* the derotation need not be done in the replay mode, since the traces were collected in the 
		     dst mode, where the derotation was already performed */
		  sigrot[k] = sym_vec[k][d_data_carriers[i]];
	      }
	  }

	  slicer_ILP(sigrot, closest_sym, bits, batched_sym_position[i], ofdm_symbol_index, i, sym_vec);

	  // update accum_error for each batch //
	  for(unsigned int k = 0; k < d_batch_size; k++) {
	     
#ifdef DEBUG
	     float error = abs(sigrot[k] * conj(closest_sym[k]));
	     float error_no_rot = abs(sym_vec[k][d_data_carriers[i]] * conj(closest_sym[k]));
	     gr_complex sig = sym_vec[k][d_data_carriers[i]];
	     printf("error: %f, error_no_rot: %f, d_phase: %f, d_freq: %f\n", error, error_no_rot, d_phase[k], d_freq[k]); fflush(stdout);
	     printf("actual_sym: (%f, %f) rot_sym: (%f, %f), closest_sym: (%f, %f)\n", sig.real(), sig.imag(), sigrot[k].real(), sigrot[k].imag(), closest_sym[k].real(), closest_sym[k].imag());
	     fflush(stdout);

	     float eucd_dt = norm(sigrot[k] - closest_sym[k]);
	     float eucd_dt_no_rot = norm(sym_vec[k][d_data_carriers[i]] - closest_sym[k]);
	     printf("sym: (%f, %f) rot_sym: (%f, %f), closest_sym: (%f, %f), eucl_dt: %f, eucl_dt_no_rot: %f\n\n", sig.real(), sig.imag(), sigrot[k].real(), sigrot[k].imag(), closest_sym[k].real(), closest_sym[k].imag(), eucd_dt, eucd_dt_no_rot); fflush(stdout);
#endif	     

	     accum_error[k] += (sigrot[k] * conj(closest_sym[k]));
	     if (norm(sigrot[k])> 0.001) dfe_vec[k][i] +=  d_eq_gain*(closest_sym[k]/sigrot[k]-dfe_vec[k][i]);

         /* calculate the accum_error1 in case the estimated phase shift was used */
         //accum_error1[k] += (sym_vec[k][d_data_carriers[i]] * conj(closest_sym[k]));
         accum_error1[k] += (sigrot1[k] * conj(closest_sym[k]));
		 if (norm(sigrot1[k])> 0.001) dfe_vec[k][i] +=  d_eq_gain*(closest_sym[k]/sigrot1[k]-dfe_vec[k][i]);

	     assert((8 - byte_offset) >= d_data_nbits);
	     partial_byte[k] |= bits[k] << (byte_offset);
	  }

	  byte_offset += d_data_nbits;
     }
  
     if(byte_offset == 8) {
	for(int k = 0; k < d_batch_size; k++) {
#ifdef DEBUG
	    printf("k: %d, demod_byte: %x (%d) \n", k, partial_byte[k], partial_byte[k]); fflush(stdout);
#endif
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

     PktInfo *pInfo = innovativePkts[k];
     pInfo->phase_eq_vec[0] = d_phase[k];
     pInfo->freq_eq_vec[0] = d_freq[k];

     //if(ofdm_symbol_index >= NULL_OFDM_SYMBOLS-1) {
     /* tracking the error, had there been no compensation */
     float angle1 = arg(accum_error1[k]);
     d_freq1[k] = d_freq1[k] - d_freq_gain*angle1;
     d_phase1[k] = d_phase1[k] + d_freq1[k] - d_phase_gain*angle1;
     if (d_phase1[k] >= 2*M_PI) d_phase1[k] -= 2*M_PI;
     if (d_phase1[k] <0) d_phase1[k] += 2*M_PI;
     //printf("atten: %f, angle: %f, phase: %f, freq: %f -------- atten1: %f angle1: %f, phase1: %f, freq1: %f\n", abs(accum_error[0]), angle, d_phase[0], d_freq[0], abs(accum_error1[0]), angle1, d_phase1[0], d_freq1[0]); fflush(stdout);
     //}
  }

  return bytes_produced;
}

// build the combined symbol map. Needs to be called for every new batch. There will 
// 2^d_batch_size entries in each map 
void
digital_ofdm_frame_sink::buildMap_ILP(cx_mat coeffs, vector<gr_complex*> &batched_sym_position) {
  int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));

  if(d_batch_size == 1) {
     for(unsigned int i = 0; i < d_batch_size; i++) {
	 gr_complex *sym_position = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);
	 int j = 0;
	 sym_position[j++] = ((coeffs(i, 0) * (comp_d) d_sym_position[0]));
	 sym_position[j++] = ((coeffs(i, 0) * (comp_d) d_sym_position[1]));;
	 batched_sym_position.push_back(sym_position);  
       }
  }
  if(d_batch_size == 2) {
     for(unsigned int i = 0; i < d_batch_size; i++) {
         gr_complex *sym_position = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);
	
         int j = 0;
#ifdef SCALE
	 sym_position[j++] = (((coeffs(i, 0) * (comp_d) d_sym_position[0]) * (comp_d)(SCALE)) + (coeffs(i, 1) * (comp_d) d_sym_position[0] * (comp_d)(SCALE)));
	 sym_position[j++] = (((coeffs(i, 0) * (comp_d) d_sym_position[0]) * (comp_d)(SCALE)) + (coeffs(i, 1) * (comp_d) d_sym_position[1] * (comp_d)(SCALE)));
	 sym_position[j++] = (((coeffs(i, 0) * (comp_d) d_sym_position[1]) * (comp_d)(SCALE)) + (coeffs(i, 1) * (comp_d) d_sym_position[0] * (comp_d)(SCALE)));
	 sym_position[j++] = (((coeffs(i, 0) * (comp_d) d_sym_position[1]) * (comp_d)(SCALE)) + (coeffs(i, 1) * (comp_d) d_sym_position[1] * (comp_d)(SCALE)));
#else
         sym_position[j++] = (((coeffs(i, 0) * (comp_d) d_sym_position[0])) + (coeffs(i, 1) * (comp_d) d_sym_position[0]));
         sym_position[j++] = (((coeffs(i, 0) * (comp_d) d_sym_position[0])) + (coeffs(i, 1) * (comp_d) d_sym_position[1]));
         sym_position[j++] = (((coeffs(i, 0) * (comp_d) d_sym_position[1])) + (coeffs(i, 1) * (comp_d) d_sym_position[0]));
         sym_position[j++] = (((coeffs(i, 0) * (comp_d) d_sym_position[1])) + (coeffs(i, 1) * (comp_d) d_sym_position[1]));
#endif
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
}

void
digital_ofdm_frame_sink::debugMap_ILP(vector<vector<gr_complex*> > batched_sym_position, int pkts) {
  printf("debugMap_ILP\n"); fflush(stdout);

  for(int i = 0; i < batched_sym_position.size(); i++) {
      vector<gr_complex*> batch_position = batched_sym_position[i];
      if(batch_position.size() != d_batch_size) {
	printf("batch_position.size: %d, d_batch_size: %d\n", batch_position.size(), d_batch_size); fflush(stdout);
	assert(false);
      }
      assert(batch_position.size() == d_batch_size);
      for(int j = 0; j < pkts; j++) {
	 gr_complex *positions = batch_position[j];
	 int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));

	 
	 for(int k = 0; k < n_entries; k++) {
	    printf("k: %d, (%f, %f) \n", k, positions[k].real(), positions[k].imag()); fflush(stdout);	
	 }
      }
  }
  printf("debugMap_ILP done\n"); fflush(stdout);
} 

/* for offline analysis - logs the frequency domain symbols */
void
digital_ofdm_frame_sink::logCorrectedSymbols(FlowInfo *flowInfo) {

  // prep for building the map //
  vector<gr_complex*> batched_sym_position;

  int dc_tones = d_occupied_carriers - d_data_carriers.size();
  unsigned int half_occupied_tones = (d_occupied_carriers - dc_tones)/2;

  assert(d_batch_size == 2);
  int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));

  cx_mat coeffs = randu<cx_mat>(1, d_batch_size);

  int pkt_index = flowInfo->innovative_pkts.size() - 1;

  for(unsigned int i = 0; i < d_occupied_carriers; i++) {

        if(i >= half_occupied_tones && i < half_occupied_tones + dc_tones) {
            continue;                                                                                   // bypass the DC tones //
        }

        gr_complex *red_coeffs = flowInfo->reduced_coeffs[pkt_index];       // get the reduced_coeffs for this packet
	coeffs.zeros();
	for(int j = 0; j < d_batch_size; j++) 	
           coeffs(0, j) = red_coeffs[i * d_batch_size + j];

        // build the map for each batch on each subcarrier //
        int j = 0;
	gr_complex *sym_position = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);
        sym_position[j++] = (((coeffs(0, 0) * (comp_d) d_sym_position[0])) + (coeffs(0, 1) * (comp_d) d_sym_position[0]));
        sym_position[j++] = (((coeffs(0, 0) * (comp_d) d_sym_position[0])) + (coeffs(0, 1) * (comp_d) d_sym_position[1]));
        sym_position[j++] = (((coeffs(0, 0) * (comp_d) d_sym_position[1])) + (coeffs(0, 1) * (comp_d) d_sym_position[0]));
        sym_position[j++] = (((coeffs(0, 0) * (comp_d) d_sym_position[1])) + (coeffs(0, 1) * (comp_d) d_sym_position[1]));
        batched_sym_position.push_back(sym_position);
  }

  assert(batched_sym_position.size() == d_data_carriers.size());

  // initialization //
  vector<gr_complex> dfe_vec;
  dfe_vec.resize(d_occupied_carriers);
  fill(dfe_vec.begin(), dfe_vec.end(), gr_complex(1.0,0.0));

  gr_complex* sym_vec = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
  gr_complex* closest_sym = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);;

  float phase = (d_pktInfo->phase_eq_vec)[0];
  float freq = (d_pktInfo->freq_eq_vec)[0];

  int offset = (d_num_hdr_ofdm_symbols+1) * d_fft_length;
  unsigned int left_guard = (d_fft_length - d_occupied_carriers)/2;

  // signal correction //
  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
      gr_complex carrier, accum_error;
      gr_complex *rx_symbols = d_pktInfo->symbols;

      carrier = gr_expj(phase);
      accum_error = 0.0;

      memset(closest_sym, 0, sizeof(gr_complex) * d_occupied_carriers);
      memcpy(sym_vec, &rx_symbols[o * d_occupied_carriers], sizeof(gr_complex) * d_occupied_carriers);

      gr_complex sigrot;
      for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
          sigrot = sym_vec[d_data_carriers[i]] * carrier * dfe_vec[i];
          matchSymbol(sigrot, closest_sym[d_data_carriers[i]], batched_sym_position[i]);
#ifdef DEBUG
	  printf("sigrot: (%f, %f), closest: (%f, %f)\n", sigrot.real(), sigrot.imag(), closest_sym[d_data_carriers[i]].real(), closest_sym[d_data_carriers[i]].imag());
	  fflush(stdout);
#endif

          // update accum_error, dfe_vec //
          accum_error += (sigrot * conj(closest_sym[d_data_carriers[i]]));
          if (norm(sigrot)> 0.001) dfe_vec[i] +=  d_eq_gain*(closest_sym[d_data_carriers[i]]/sigrot-dfe_vec[i]);
      }

      // update the PLL settings //
      float angle = arg(accum_error);
      freq = freq - d_freq_gain*angle;
      phase = phase + freq - d_phase_gain*angle;
      if (phase >= 2*M_PI) phase -= 2*M_PI;
      if (phase <0) phase += 2*M_PI;

      // copy to log later //
      memcpy(d_corrected_symbols + offset + left_guard, closest_sym, sizeof(gr_complex) * d_occupied_carriers);
      memcpy(d_uncorrected_symbols + offset + left_guard, sym_vec, sizeof(gr_complex) * d_occupied_carriers);
      offset += d_fft_length;
  }

  // log now //
  assert(offset = (d_num_ofdm_symbols+d_num_hdr_ofdm_symbols+1) * d_fft_length);
  int count = ftell(d_fp_corrected_symbols);
  count = fwrite_unlocked(d_corrected_symbols, sizeof(gr_complex), offset, d_fp_corrected_symbols); 
  count = ftell(d_fp_uncorrected_symbols);
  count = fwrite_unlocked(d_uncorrected_symbols, sizeof(gr_complex), offset, d_fp_uncorrected_symbols);

  // cleanup //
  free(sym_vec);
  free(closest_sym);

  for(unsigned int i = 0; i < d_data_carriers.size(); i++) 
	free(batched_sym_position[i]);
  batched_sym_position.clear();
}

bool
digital_ofdm_frame_sink::open_corrected_symbols_log()
{
  const char *filename = "corrected_symbols-f-domain.dat";
  int fd;
  if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     return false;
  }
  else {
      if((d_fp_corrected_symbols = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "corrected symbols file cannot be opened\n");
            close(fd);
            return false;
      }
  }

  const char *filename1 = "uncorrected_symbols-f-domain.dat";
  int fd1;
  if ((fd1 = open (filename1, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename1);
     return false;
  }
  else {
      if((d_fp_uncorrected_symbols = fdopen (fd1, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "corrected symbols file cannot be opened\n");
            close(fd1);
            return false;
      }
  }


  /*
  const char *filename1 = "accum_error.dat";
  int fd1;
  if ((fd1 = open (filename1, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename1);
     return false;
  }
  else {
      if((d_fp_corr_log = fdopen (fd1, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "correction log file cannot be opened\n");
            close(fd1);
            return false;
      }
  }
  */
  return true;
}

void
digital_ofdm_frame_sink::matchSymbol(gr_complex x, gr_complex &closest_sym, 
                                           gr_complex* sym_position) {
  unsigned int min_index = 0;
  float min_euclid_dist = 0.0;

  // for each table entry, find the min(error, k) //
  unsigned int table_size = pow(2.0, double(d_batch_size));

  // initialize //
  min_euclid_dist = norm(x - sym_position[0]);

  float euclid_dist = 0.0;
  for(unsigned int j = 1; j < table_size; j++) {
      euclid_dist = norm(x - sym_position[j]);
      if (euclid_dist < min_euclid_dist) {
          min_euclid_dist = euclid_dist;
          min_index = j;
      }
  }

  closest_sym = sym_position[min_index];
}
#endif

/* alternative decoding strategy, where you try and decode after every packet received. In case we are lucky, we can decode
   after receiving just 1 packet. Else keep trying to decode with every successive packet received. Gradually, the strategy should 
   converge to the right decoding.

   Need to save the total error (hamming distance) seen for each modulation symbol. For each packet received thereafter, compute the 
   updated error and try and make a decision. 
 */
void
digital_ofdm_frame_sink::demodulate_ILP_2(FlowInfo *flowInfo)
{
  printf("demodulate_ILP_2, pkt_no: %d, senders: %d\n", d_pkt_num, d_pktInfo->n_senders); fflush(stdout);
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
#else
  // contains the map for each subcarrier
  // each map has 'n_entries = 2^batch_size' entries
  vector<gr_complex*>  batched_sym_position;

  int dc_tones = d_occupied_carriers - d_data_carriers.size();
  unsigned int half_occupied_tones = (d_occupied_carriers - dc_tones)/2;

  int pkt_index = flowInfo->innovative_pkts.size() - 1;
  gr_complex *pkt_coeffs = flowInfo->reduced_coeffs[pkt_index];  

  for(unsigned int i = 0; i < d_occupied_carriers; i++) {

        if(i >= half_occupied_tones && i < half_occupied_tones + dc_tones) {
            continue;                                                                                   // bypass the DC tones //
        }

        // build the map for each subcarrier //
	int n_entries = pow(double(d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));
	gr_complex *sym_position = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);

        buildMap_ILP_2(&(pkt_coeffs[i * d_batch_size]), sym_position);
        batched_sym_position.push_back(sym_position);
  }

  assert(batched_sym_position.size() == d_data_carriers.size());

  // for DFE PLL //
  vector<gr_complex> dfe_vec;
  dfe_vec.resize(d_occupied_carriers);
  fill(dfe_vec.begin(), dfe_vec.end(), gr_complex(1.0,0.0));
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

  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
#ifdef USE_PILOT

#ifdef SRC_PILOT
      unsigned int bytes_decoded = demapper_ILP_2_pilot_SRC(o, bytes_out_vec, flowInfo, dfe_pilot[0], interpolated_coeffs);   // same # of bytes decoded/batch
#else
      unsigned int bytes_decoded = demapper_ILP_2_pilot(o, bytes_out_vec, flowInfo, dfe_pilot, interpolated_coeffs);   // same # of bytes decoded/batch
#endif

#else	// no pilots //
      unsigned int bytes_decoded = demapper_ILP_2(o, bytes_out_vec, batched_sym_position, flowInfo, dfe_vec);   // same # of bytes decoded/batch 
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

		  printf("crc valid: %d, d_crc: %d\n", crc_valid, d_crc[k]); fflush(stdout);
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
  printf("demodulate_ILP_2 end, pkt_no: %d\n\n", d_pkt_num); fflush(stdout);
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

void
digital_ofdm_frame_sink::buildMap_ILP_2(gr_complex *coeffs, gr_complex* sym_position) {
  
  assert(false); 		// disable for now //
#if 0
  if(d_batch_size == 1) {
         int j = 0;
         sym_position[j++] = ((coeffs[0] *  d_data_sym_position[0]));
         sym_position[j++] = ((coeffs[0] *  d_data_sym_position[1]));
  }
  if(d_batch_size == 2) {
         int j = 0;
#ifdef SCALE
         sym_position[j++] = (((coeffs[0] *  d_sym_position[0]) * (SCALE)) + (coeffs[1] *  d_sym_position[0] * (SCALE)));
         sym_position[j++] = (((coeffs[0] *  d_sym_position[0]) * (SCALE)) + (coeffs[1] *  d_sym_position[1] * (SCALE)));
         sym_position[j++] = (((coeffs[0] *  d_sym_position[1]) * (SCALE)) + (coeffs[1] *  d_sym_position[0] * (SCALE)));
         sym_position[j++] = (((coeffs[0] *  d_sym_position[1]) * (SCALE)) + (coeffs[1] *  d_sym_position[1] * (SCALE)));
#else
         sym_position[j++] = (((coeffs[0] *  d_sym_position[0])) + (coeffs[1] *  d_sym_position[0]));
         sym_position[j++] = (((coeffs[0] *  d_sym_position[0])) + (coeffs[1] *  d_sym_position[1]));
         sym_position[j++] = (((coeffs[0] *  d_sym_position[1])) + (coeffs[1] *  d_sym_position[0]));
         sym_position[j++] = (((coeffs[0] *  d_sym_position[1])) + (coeffs[1] *  d_sym_position[1]));
#endif
  }
  else if (d_batch_size == 3) {
         int j = 0;
         sym_position[j++] = coeffs[0]* d_sym_position[0] + coeffs[1]* d_sym_position[0] + coeffs[2]* d_sym_position[0];
         sym_position[j++] = coeffs[0]* d_sym_position[0] + coeffs[1]* d_sym_position[0] + coeffs[2]* d_sym_position[1];
         sym_position[j++] = coeffs[0]* d_sym_position[0] + coeffs[1]* d_sym_position[1] + coeffs[2]* d_sym_position[0];
         sym_position[j++] = coeffs[0]* d_sym_position[0] + coeffs[1]* d_sym_position[1] + coeffs[2]* d_sym_position[1];
         sym_position[j++] = coeffs[0]* d_sym_position[1] + coeffs[1]* d_sym_position[0] + coeffs[2]* d_sym_position[0];
         sym_position[j++] = coeffs[0]* d_sym_position[1] + coeffs[1]* d_sym_position[0] + coeffs[2]* d_sym_position[1];
         sym_position[j++] = coeffs[0]* d_sym_position[1] + coeffs[1]* d_sym_position[1] + coeffs[2]* d_sym_position[0];
         sym_position[j++] = coeffs[0]* d_sym_position[1] + coeffs[1]* d_sym_position[1] + coeffs[2]* d_sym_position[1];
  }
#endif
  //printf("buildMap_ILP done\n"); fflush(stdout);
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

  //printf("buildMap_pilot start, num_senders: %d\n", num_senders); fflush(stdout);

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

#if 1
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
  //printf("buildMap_pilot end\n"); fflush(stdout);
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

     printf("d_end_angle: %.3f\n", d_end_angle[s]); fflush(stdout);
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
	 slicer_ILP_2(sigrot, closest_sym, bits, sym_position, ofdm_symbol_index, i);

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

#endif

/* incremental version: demap every packet that comes in, and try and decode the complete batch out of it. Every
   new packet received for the batch is plugged in to decode the complete batch. Eventually we hope to converge at the 
   correct decision */
unsigned int
digital_ofdm_frame_sink::demapper_ILP_2(unsigned int ofdm_symbol_index, vector<unsigned char*> out_vec,
                                        vector<gr_complex*> batched_sym_position, FlowInfo *flowInfo,
                                        vector<gr_complex> dfe_vec) {

  unsigned int bytes_produced = 0;
  assert(d_batch_size <= MAX_BATCH_SIZE);

  InnovativePktInfoVector innovativePkts = flowInfo->innovative_pkts;
  int pkt_index = innovativePkts.size() - 1;

  // get the current packet //
  PktInfo *pInfo = innovativePkts[pkt_index];
  gr_complex *rx_symbols_this_batch = pInfo->symbols;

  gr_complex *sym_vec = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
  memcpy(sym_vec, &rx_symbols_this_batch[ofdm_symbol_index * d_occupied_carriers], sizeof(gr_complex) * d_occupied_carriers);

  // need to check how it works (works only for 1 sender) //
  d_phase[pkt_index] = (pInfo->phase_eq_vec)[0];
  d_freq[pkt_index] = (pInfo->freq_eq_vec)[0];

  gr_complex carrier = gr_expj(d_phase[pkt_index]);
  gr_complex accum_error = 0.0;

  /* for the current packet */
  gr_complex sigrot, closest_sym;

  /* for every packet of the batch, 'd_batch_size' # of partial bytes and bits will be decoded */
  unsigned int partial_byte[MAX_BATCH_SIZE];
  unsigned char bits[MAX_BATCH_SIZE];
  memset(partial_byte, 0, sizeof(unsigned int) * MAX_BATCH_SIZE);
  memset(bits, 0, sizeof(unsigned char) * MAX_BATCH_SIZE);

  unsigned int byte_offset = 0;
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {

     // demodulate 1-byte at a time //
     if (byte_offset < 8) {

          if(d_replay_flag != CORRECTED_REPLAY) {
               sigrot = sym_vec[d_data_carriers[i]] * carrier * dfe_vec[i];
          }
          else {
             /* the derotation need not be done in the replay mode, since the traces were collected in the 
                dst mode, where the derotation was already performed */
                sigrot = sym_vec[d_data_carriers[i]];
          }

          slicer_ILP_2(sigrot, closest_sym, bits, batched_sym_position[i], ofdm_symbol_index, i);

          // update accum_error for this packet //
          accum_error += (sigrot * conj(closest_sym));
          if (norm(sigrot)> 0.001) dfe_vec[i] +=  d_eq_gain*(closest_sym/sigrot-dfe_vec[i]);

          assert((8 - byte_offset) >= d_data_nbits);

	  for(unsigned int k = 0; k < d_batch_size; k++)
              partial_byte[k] |= bits[k] << (byte_offset);

          byte_offset += d_data_nbits;
     }

     if(byte_offset == 8) {
        for(int k = 0; k < d_batch_size; k++) {
#ifndef DEBUG
            printf("k: %d, demod_byte: %x (%d) \n", k, partial_byte[k], partial_byte[k]); fflush(stdout);
#endif
            out_vec[k][bytes_produced] = partial_byte[k];
            partial_byte[k] = 0;
        }
        bytes_produced++;
        byte_offset = 0;
     }
  }

  assert(byte_offset == 0);

  // update the accum_error, etc //
  float angle = arg(accum_error);
  d_freq[pkt_index] = d_freq[pkt_index] - d_freq_gain*angle;
  d_phase[pkt_index] = d_phase[pkt_index] + d_freq[pkt_index] - d_phase_gain*angle;
  if (d_phase[pkt_index] >= 2*M_PI) d_phase[pkt_index] -= 2*M_PI;
  if (d_phase[pkt_index] <0) d_phase[pkt_index] += 2*M_PI;

  pInfo->phase_eq_vec[0] = d_phase[pkt_index];
  pInfo->freq_eq_vec[0] = d_freq[pkt_index];

  return bytes_produced;
}

/* calculate the new euclid_dist after putting this packet's symbols in. Also update the batch history for d_euclid_dist */
void
digital_ofdm_frame_sink::slicer_ILP_2(gr_complex x, gr_complex& closest_sym, unsigned char *bits,
                                      gr_complex* batched_sym_position, 
				      unsigned int ofdm_index, unsigned int subcarrier_index)
{
  printf("slicer_ILP_2\n"); fflush(stdout);
  unsigned int min_index = 0;
  assert(d_num_ofdm_symbols < MAX_OFDM_SYMBOLS);						// upper capped for now! 
  d_euclid_dist[ofdm_index][subcarrier_index][0] += norm(x - batched_sym_position[0]);
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
      float euclid_dist = d_euclid_dist[ofdm_index][subcarrier_index][j] + norm(x - batched_sym_position[j]);

      //if(d_pkt_num == 1 && subcarrier_index == 0) 
      if(1)
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

  d_avg_evm_error += min_euclid_dist;

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
  printf("crc_check begin! len: %d\n", len); fflush(stdout);
  dewhiten(msg_data, len);
  std::string dewhitened_msg((const char*) msg_data, len);

  /*
  printf("ofdm_frame_sink:: dewhitened msg, len: %d\n", dewhitened_msg.length()); 
  for(int i = 0; i < len; i++) {
     printf("%02x ", (unsigned char) dewhitened_msg[i]); fflush(stdout);
  } 
  printf("\n"); fflush(stdout); */

  /* perform any FEC if required */
  //decoded_msg = dewhitened_msg;
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

  printf("crc_check end: %d\n", res); fflush(stdout);
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

  printf("(SINK) nread1: %llu, output_items: %d\n", nread1, output_items); fflush(stdout);
  if(rx_sync_tags.size()>0) {
     size_t t = rx_sync_tags.size()-1;
     uint64_t offset = rx_sync_tags[t].offset;

     printf("test_timestamp1 (SINK):: found %d tags, offset: %llu, output_items: %d, nread1: %llu\n", rx_sync_tags.size(), rx_sync_tags[t].offset, output_items, nread1); fflush(stdout);

     const pmt::pmt_t &value = rx_sync_tags[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));

     d_sync_tag.value = rx_sync_tags[t].value;
     d_sync_tag.offset = offset;
  }
 
  //std::cerr << "SINK---- Header received, with no sync timestamp1?\n";
  //assert(false);
}

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
  if(d_fwd_index == 2) {
      num_samples += (d_num_hdr_ofdm_symbols+1) * (d_fft_length+cp_length);
  }

  double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
  double duration = num_samples * time_per_sample;
  printf("RX timestamp (%llu, %f), duration: %f\n", sync_secs, sync_frac_of_secs, duration); fflush(stdout);

  uhd::time_spec_t c_time = d_usrp->get_time_now();
  uhd::time_spec_t proc_time = c_time - uhd::time_spec_t(sync_secs, sync_frac_of_secs);

  printf("FWD PACKET -- c_time(%llu, %f), proc_time (%llu, %f)\n", (uint64_t) c_time.get_full_secs(), c_time.get_frac_secs(), (uint64_t) proc_time.get_full_secs(), proc_time.get_frac_secs()); fflush(stdout);

  /* add the duration to the current timestamp */
  sync_secs += (int) duration;
  sync_frac_of_secs = duration - (int) duration + sync_frac_of_secs;
  if(sync_frac_of_secs>1) {
     sync_secs += (uint64_t)sync_frac_of_secs;
     sync_frac_of_secs -= (uint64_t)sync_frac_of_secs;
  }
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
  reduceCoefficients_LSQ(flowInfo);                                                 // fill up flowInfo->reduced_coeff //

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

	      coeff[index] += ((gr_complex(1.0, 0.0)/estimates[di]) * rx_coeffs[index]);
	  }
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

  printf("buildMap_pilot start, num_senders: %d\n", num_senders); fflush(stdout);

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

          gr_complex coeff = (gr_complex(1.0, 0.0)/(estimates[subcarrier])) * in_coeffs[coeff_index];
          if(k == 0) coeffs[j] = coeff;
          else coeffs[j] += coeff;
      }
      coeffs[j] *= (gr_complex(1.0, 0.0)/(carrier * dfe[subcarrier_index]));
  }
  //if(o == 0) printf("\n"); fflush(stdout);

  if(d_batch_size == 1) {
         int j = 0;
#if 0
         sym_position[j++] = ((coeffs[0] *  d_data_sym_position[0]));
         sym_position[j++] = ((coeffs[0] *  d_data_sym_position[1]));
#endif
	 printf("d_data_sym_position.size: %d\n", d_data_sym_position.size()); fflush(stdout);
	 for(unsigned int m1 = 0; m1 < d_data_sym_position.size(); m1++) {
	     sym_position[j++] = (coeffs[0] * d_data_sym_position[m1]);
	 }
  }
  if(d_batch_size == 2) {
         int j = 0;
#if 0
         sym_position[j++] = (((coeffs[0] *  d_data_sym_position[0])) + (coeffs[1] *  d_data_sym_position[0]));
         sym_position[j++] = (((coeffs[0] *  d_data_sym_position[0])) + (coeffs[1] *  d_data_sym_position[1]));
         sym_position[j++] = (((coeffs[0] *  d_data_sym_position[1])) + (coeffs[1] *  d_data_sym_position[0]));
         sym_position[j++] = (((coeffs[0] *  d_data_sym_position[1])) + (coeffs[1] *  d_data_sym_position[1]));
#endif
	 for(unsigned int m1 = 0; m1 < d_data_sym_position.size(); m1++) {
	     for(unsigned int m2 = 0; m2 < d_data_sym_position.size(); m2++) {
		gr_complex pos = (coeffs[0] * d_data_sym_position[m1]) + (coeffs[1] * d_data_sym_position[m2]);
		std::cout<<"sym_pos:: " << pos <<" = " << coeffs[0] << " * " << d_data_sym_position[m1] << " + " << coeffs[1] << " * " << d_data_sym_position[m2] << endl;
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
  //printf("buildMap_pilot end\n"); fflush(stdout);
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
         slicer_ILP_2(sigrot, closest_sym, bits, sym_position, ofdm_symbol_index, i);

         assert((8 - byte_offset) >= d_data_nbits);

         for(unsigned int k = 0; k < d_batch_size; k++)
              partial_byte[k] |= bits[k] << (byte_offset);

         byte_offset += d_data_nbits;
     }

     if(byte_offset == 8) {
        for(int k = 0; k < d_batch_size; k++) {
            printf("k: %d, demod_byte: %x (%d) \n", k, partial_byte[k], partial_byte[k]); fflush(stdout);
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
  float delta_angle = d_end_angle[sender] - d_start_angle[sender] + d_slope_angle[sender];
  printf("adjust_H_estimate: %.3f\n", delta_angle); fflush(stdout);
  for(unsigned int i = 0; i < d_occupied_carriers; i++) {
     hestimate[i] *= gr_expj(-delta_angle);
  }
}
#endif
