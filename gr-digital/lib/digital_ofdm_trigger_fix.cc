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

#include <digital_ofdm_trigger_fix.h>
#include <gr_io_signature.h>

digital_ofdm_trigger_fix_sptr
digital_make_ofdm_trigger_fix (size_t input_size, size_t output_size)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_trigger_fix (input_size,
								      output_size));
}

digital_ofdm_trigger_fix::digital_ofdm_trigger_fix (size_t input_size,
							    size_t output_size)
  : gr_sync_interpolator ("ofdm_trigger_fix",
			  gr_make_io_signature (1, 1, input_size*sizeof(gr_complex)),
			  gr_make_io_signature (1, 1, sizeof(gr_complex)),
			  output_size), 
    d_input_size(input_size),
    d_output_size(output_size)

{
}

int
digital_ofdm_trigger_fix::work (int noutput_items,
				    gr_vector_const_void_star &input_items,
				    gr_vector_void_star &output_items)
{
  gr_complex *in = (gr_complex *) input_items[0];
  gr_complex *out = (gr_complex *) output_items[0];
  size_t cp_size = d_output_size - d_input_size;
  unsigned int i=0;

  out[i] = in[0];
  for(i=1; i < d_output_size; i++) {
    out[i] = gr_complex(0,0);
  }

  return d_output_size;
}
