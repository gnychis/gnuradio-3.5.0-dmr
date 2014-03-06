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


gr_threshold_ff_sptr
gr_make_threshold_ff (const std::vector<float> &lo, const std::vector<float> &hi, float initial_state, int fft_length, int type, int gap)
{
  return gnuradio::get_initial_sptr(new gr_threshold_ff (lo, hi, initial_state, fft_length, type, gap));
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


gr_threshold_ff::gr_threshold_ff (const std::vector<float> &lo, const std::vector<float> &hi, float initial_state, int fft_length, int type, int gap)
  : gr_sync_block ("threshold_ff",
                   gr_make_io_signature (1, 1, sizeof (float)),
                   gr_make_io_signature (1, 1, sizeof (float))),
    d_lo_vec(lo), d_hi_vec(hi), d_last_state (initial_state), d_fft_length(fft_length), d_threshold_type(type), d_peak_gap(gap)
{

  d_index = 0;
  d_prev_hi = 0.0;
  assert(d_fft_length > 0);


  d_hi = d_hi_vec[0];
  d_lo = d_lo_vec[0];

  d_samples_passed = 0;
  d_threshold_index = 0;
  d_prev_peak_index = -1;
  d_gap = 0; d_round = 0;

  switch(d_threshold_type) {
    case TWO_FLOW_SINGLE_TRANSMISSION:
	assert(d_hi_vec.size() == 2);
        break;
    case ONE_FLOW_JOINT_TRANSMISSION:
        assert(d_hi_vec.size() == 2);
        break;
    case TWO_FLOW_JOINT_TRANSMISSION:
       	assert(d_hi_vec.size() == 4);
	break;
    default:
	assert(d_hi_vec.size() == 1);
	break;
  }
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
  int peak_index = 0;

  /* bypass the 1st item if d_last_state == 1 (as processed in last iteration) */
  int start = 0;
  if(d_last_state == 1.0)
     start = 1;

  d_round++;
  for(int i=start; i<noutput_items; i++) {

    out[i] = 0.0;
    d_samples_passed++;

    if(in[i] > d_hi) {
	/* peak detected */
      if(d_last_state == 0.0 && (d_samples_passed >= d_gap)) {
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
	       d_prev_hi = in[i];
	       prev_index = i;
	    }
	 }
	 else {
	    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
	    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"); fflush(stdout);
	    //assert(false);
	    //assert((i-prev_index) > d_fft_length);
	    peak_index = prev_index;
	    d_samples_passed -= (i-peak_index);
	    out[prev_index] = 1.0;					// the prev_index peak was genuine //
	    prev_index = i;						// now examine the new 'prev_index' //
	    d_prev_hi = in[i];
	 }
      }
    } // if(in[i] > d_hi)
    else {
       if(d_last_state == 1.0 && ((i-prev_index) > d_fft_length)) {
	    /* reset: not seen any peak close enough to previous */
	    d_last_state = 0.0;
	    peak_index = prev_index;
	    out[prev_index] = 1.0;
	    d_samples_passed -= (i-peak_index);
       }
    }

    /* apurv++ hack:, multiple peaks */
    if(d_threshold_type > 1 && (d_threshold_index % 2 == 1)) {
       if(d_samples_passed == 1320) {
	  printf(" round: %ld, @@@@@@@@@@@@@@@@@@@ fixing the threshold @@@@@@@@@@@@@@@@@ \n", d_round); fflush(stdout);
	  peak_index = i;
	  out[peak_index] = 1.0;
	  d_last_state = 0.0;
       }
    }

    if(out[peak_index] == 1.0 && (d_last_state == 0.0) && (d_samples_passed >= d_gap)) {
       switch(d_threshold_type) {
	  case TWO_FLOW_SINGLE_TRANSMISSION:
		d_gap = d_peak_gap; //3.6e5;
		break;
	  case ONE_FLOW_JOINT_TRANSMISSION:
		if(d_threshold_index == 0) {
		   d_gap = 1200+120;
		}
		else {
		   d_gap = d_peak_gap;
		}
		break;
	  case TWO_FLOW_JOINT_TRANSMISSION:
		if(d_threshold_index == 0 || d_threshold_index == 2) {
		   d_gap = 1200+120;
		}
		else {
		   d_gap = d_peak_gap;
		}
		break;
	  default:
		d_gap = d_peak_gap; 
		break;
       }

       printf("round: %ld, using threshold:::::::::::: %f, i: %d, peak_index: %d, gap: %d, d_samples_passed: %ld, noutput_items: %d (new_samples_passed: %d)\n", d_round, d_hi, i, peak_index, d_gap, d_samples_passed, noutput_items, (i-peak_index)); fflush(stdout);
       d_threshold_index = (d_threshold_index+1) % d_hi_vec.size();
       d_hi = d_hi_vec[d_threshold_index];
       d_lo = d_lo_vec[d_threshold_index];
       //d_samples_passed = 0;
       d_samples_passed = (i-peak_index);
    }
    /* apurv++ hack end */
  }
  /* apurv++ end */

 
  if(d_last_state == 1.0) {
     d_samples_passed -= (noutput_items-prev_index);
     noutput_items = prev_index;
  }

  return noutput_items;
}
