/* -*- c++ -*- */
/*
 * Copyright 2004,2010 Free Software Foundation, Inc.
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

// WARNING: this file is machine generated.  Edits will be over written

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cstdio>
#include <gr_threshold_ff.h>
#include <gr_io_signature.h>

gr_threshold_ff_sptr
gr_make_threshold_ff (float lo, float hi, float initial_state, int fft_length)
{
  return gnuradio::get_initial_sptr(new gr_threshold_ff (lo, hi, initial_state, fft_length));
}

gr_threshold_ff::gr_threshold_ff (float lo, float hi, float initial_state, int fft_length)
  : gr_sync_block ("threshold_ff",
		   gr_make_io_signature (1, 1, sizeof (float)),
		   gr_make_io_signature (1, 1, sizeof (float))),
    d_lo (lo), d_hi (hi), d_last_state (initial_state), d_fft_length(fft_length)
{

  d_index = 0; 
  d_prev_hi = 0.0;
  assert(d_fft_length > 0);
}

int
gr_threshold_ff::work (int noutput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items)
{
  const float *in = (const float *) input_items[0];
  float *out = (float *) output_items[0];

  /*
  for(int i=0; i<noutput_items; i++) {
    if (in[i] > d_hi) {
      out[i] = 1.0;
      d_last_state = 1.0;
    } else if (in[i] < d_lo) {
      out[i] = 0.0;
      d_last_state = 0.0;
    } else
      out[i] = d_last_state;
  }*/

  /* apurv++ start */
  int prev_index = 0;

  /* bypass the 1st item if d_last_state == 1 (as processed in last iteration) */
  int start = 0;
  if(d_last_state == 1.0)
     start = 1;

  for(int i=start; i<noutput_items; i++) {

    out[i] = 0.0;

    if(in[i] > d_hi) {
      //printf("i: %d, d_last_state: %f, prev_index: %d\n", i, d_last_state, prev_index);
	/* peak detected */
      if(d_last_state == 0.0) {
	 /* completely new peak */
	 d_last_state = 1.0;
	 prev_index = i;
	 d_prev_hi = in[i];
      }
      else if(d_last_state == 1.0) {
	 /* close enough to the previous one? */
	 if((i-prev_index) <= d_fft_length) {
	    if(in[i] >= d_prev_hi) {
	       /* new peak is greater than previous one */
	       //out[i] = 1.0;
	       //d_last_state = 0.0;
	       d_prev_hi = in[i];
	       prev_index = i;
	    }
	 }
	 else {
	    //assert((i-prev_index) > d_fft_length);
	    out[prev_index] = 1.0;					// the prev_index peak was genuine //
	    prev_index = i;					// now examine the new 'prev_index' //
	    d_prev_hi = in[i];
	 }
      }
    }
    else {
       if(d_last_state == 1.0 && ((i-prev_index) > d_fft_length)) {
	    /* reset: not seen any peak close enough to previous */
	    d_last_state = 0.0;
	    out[prev_index] = 1.0;
       }
    }
  }
  /* apurv++ end */

 
  if(d_last_state == 1.0) { 
     noutput_items = prev_index;
  }

   

  return noutput_items;
}
