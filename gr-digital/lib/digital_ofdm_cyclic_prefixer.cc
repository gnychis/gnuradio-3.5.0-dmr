/* -*- c++ -*- */
/*
 * Copyright 2004,2006,2010,2011 Free Software Foundation, Inc.
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

#include <digital_ofdm_cyclic_prefixer.h>
#include <gr_io_signature.h>

digital_ofdm_cyclic_prefixer_sptr
digital_make_ofdm_cyclic_prefixer (size_t input_size, size_t output_size)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_cyclic_prefixer (input_size,
								      output_size));
}

digital_ofdm_cyclic_prefixer::digital_ofdm_cyclic_prefixer (size_t input_size,
							    size_t output_size)
  : gr_sync_interpolator ("ofdm_cyclic_prefixer",
			  //gr_make_io_signature (1, 1, input_size*sizeof(gr_complex)),
			  gr_make_io_signature2 (1, 2, input_size*sizeof(gr_complex), sizeof(short)),		// apurv++ burst tagger
			  //gr_make_io_signature (1, 1, sizeof(gr_complex)),
			  gr_make_io_signature2 (1, 2, sizeof(gr_complex), sizeof(short)),			// apurv++ burst tagger
			  output_size), 
    d_input_size(input_size),
    d_output_size(output_size)
{
}

int
digital_ofdm_cyclic_prefixer::work (int noutput_items,
				    gr_vector_const_void_star &input_items,
				    gr_vector_void_star &output_items)
{
  gr_complex *in = (gr_complex *) input_items[0];
  gr_complex *out = (gr_complex *) output_items[0];

  /* apurv start - for burst tagger */
  bool _trigger = false;
  if(input_items.size() >= 2 && output_items.size() >= 2) 
	_trigger = true;

  short *in_trigger = NULL;
  if(_trigger) {
      in_trigger = (short*) input_items[1];
  }

  short *out_trigger = NULL;
  if(_trigger) {
      out_trigger = (short*) output_items[1];
  }
  /* apurv end */
 
  size_t cp_size = d_output_size - d_input_size;
  unsigned int i=0, j=0;

  j = cp_size;
  for(i=0; i < d_input_size; i++,j++) {
    out[j] = in[i];
    if(_trigger) 
        out_trigger[j] = 1;
  }

  j = d_input_size - cp_size;
  for(i=0; i < cp_size; i++, j++) {
    out[i] = in[j];
    if(_trigger)
        out_trigger[i] = 1;				// apurv++ out_trigger = 1 for every ofdm symbol (look below)
  }

  /* apurv start -  except the last one in the packet */ 
  if(_trigger && in_trigger[0] == 0) {
    out_trigger[d_output_size-1] = 0;
  }
  /* apurv end */
 
  return d_output_size;
}
