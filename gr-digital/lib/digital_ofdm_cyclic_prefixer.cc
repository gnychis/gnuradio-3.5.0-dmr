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

  //test_timestamp(1, in_trigger[0]);			// enable to debug stream tags //
  return d_output_size;
}

/* just for debugging - to ensure the tags are being propgated correctly! */
inline void
digital_ofdm_cyclic_prefixer::test_timestamp(int output_items, short trigger) {
  //printf("test_timestamp (CP), output_items: %d, nread1: %llu, trigger: %d\n", output_items, nitems_read(1), trigger); fflush(stdout);
  unsigned int tag_port = 1;
  std::vector<gr_tag_t> rx_tags;
  const uint64_t nread1 = nitems_read(tag_port);
  get_tags_in_range(rx_tags, tag_port, nread1, nread1+output_items, pmt::pmt_string_to_symbol("tx_time"));

  for(int t = 0; t < rx_tags.size(); t++) {
     uint64_t offset = rx_tags[t].offset;
     const pmt::pmt_t &value = rx_tags[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));
     printf("test_timestamp1 (CP):: found %d tags, offset: %llu, output_items: %d, nread1: %llu, (%llu, %f)\n",                
                rx_tags.size(), rx_tags[t].offset, output_items, nread1, (uint64_t) sync_secs, sync_frac_of_secs); fflush(stdout);	
  }

#if 0
  if(rx_tags.size()>0) {
     size_t t = rx_tags.size()-1;
     uint64_t offset = rx_tags[t].offset;

     const pmt::pmt_t &value = rx_tags[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));
     printf("test_timestamp1 (CP):: found %d tags, offset: %llu, output_items: %d, nread1: %llu, (%llu, %f)\n", 
		rx_tags.size(), rx_tags[t].offset, output_items, nread1, (uint64_t) sync_secs, sync_frac_of_secs); fflush(stdout);

  } else {
     //std::cerr << "ACQ---- Header received, with no sync timestamp1?\n";
  }
#endif
}
