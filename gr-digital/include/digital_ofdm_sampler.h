/* -*- c++ -*- */
/*
 * Copyright 2007 Free Software Foundation, Inc.
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

#ifndef INCLUDED_DIGITAL_OFDM_SAMPLER_H
#define INCLUDED_DIGITAL_OFDM_SAMPLER_H

#include <digital_api.h>
#include <gr_sync_block.h>

#include <uhd/usrp/multi_usrp.hpp>

class digital_ofdm_sampler;
typedef boost::shared_ptr<digital_ofdm_sampler> digital_ofdm_sampler_sptr;

DIGITAL_API digital_ofdm_sampler_sptr digital_make_ofdm_sampler (unsigned int fft_length, 
					   unsigned int symbol_length,
					   unsigned int timeout=1000);

/*!
 * \brief does the rest of the OFDM stuff
 * \ingroup ofdm_blk
 */
class DIGITAL_API digital_ofdm_sampler : public gr_block
{
  friend DIGITAL_API digital_ofdm_sampler_sptr digital_make_ofdm_sampler (unsigned int fft_length, 
						    unsigned int symbol_length,
						    unsigned int timeout);

  digital_ofdm_sampler (unsigned int fft_length, 
		   unsigned int symbol_length,
		   unsigned int timeout);

 private:
  enum state_t {STATE_NO_SIG, STATE_PREAMBLE, STATE_FRAME};

  state_t d_state;
  unsigned int d_timeout_max;
  unsigned int d_timeout;
  unsigned int d_fft_length;
  unsigned int d_symbol_length;

 public:
  void forecast (int noutput_items, gr_vector_int &ninput_items_required);

  int general_work (int noutput_items,
		    gr_vector_int &ninput_items,
		    gr_vector_const_void_star &input_items,
		    gr_vector_void_star &output_items);

  // apurv++ for logging the preamble // 
  FILE *d_fp; 
  int d_fd;
  bool d_file_opened;  

  void log_preamble(gr_complex *, long);
  bool open_log();
  // apurv++ ends //

  uint64_t d_prev_index;
  uhd::usrp::multi_usrp::sptr d_usrp;
 
};

#endif
