/* -*- c++ -*- */
/*
 * Copyright 2010 Free Software Foundation, Inc.
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

#include <gr_burst_tagger.h>
#include <gr_io_signature.h>
#include <cstdio>
#include <string.h>

gr_burst_tagger_sptr
gr_make_burst_tagger(size_t itemsize)
{
  return gnuradio::get_initial_sptr(new gr_burst_tagger(itemsize));
}

gr_burst_tagger::gr_burst_tagger(size_t itemsize)
  : gr_sync_block ("burst_tagger",
		   gr_make_io_signature2 (2, 2, itemsize, sizeof(short)),
		   gr_make_io_signature (1, 1, itemsize)),
    d_itemsize(itemsize), d_state(false)
{
  std::stringstream str;
  str << name() << unique_id();

  d_key = pmt::pmt_string_to_symbol("burst");
  d_id  = pmt::pmt_string_to_symbol(str.str());
}

gr_burst_tagger::~gr_burst_tagger()
{
}

int
gr_burst_tagger::work(int noutput_items,
		      gr_vector_const_void_star &input_items,
		      gr_vector_void_star &output_items)
{
  const char *signal  = (const char*)input_items[0];
  const short *trigger = (const short*)input_items[1];
  char *out = (char*)output_items[0];

  memcpy(out, signal, noutput_items * d_itemsize);

  //printf("noutput_items: %d\n", noutput_items); fflush(stdout); 
  for(int i = 0; i < noutput_items; i++) {
    if(trigger[i] > 0) {
      if(d_state == false) {
	printf("burst_tagger: SOB, i: %d, noutput_items: %d, nitems_written: %d\n", i, noutput_items, nitems_written(0)); fflush(stdout);
	d_state = true;
	pmt::pmt_t value = pmt::PMT_T;
	d_key = pmt::pmt_string_to_symbol("tx_sob");
	d_id = pmt::pmt_string_to_symbol(this->name());
	add_item_tag(0, nitems_written(0)+i, d_key, value, d_id);

	modify_timestamp(noutput_items, i);
      }
    }
    else {
       if(d_state == true) {
	 printf("burst_tagger: EOB, i: %d, noutput_items: %d, nitems_written: %d\n", i, noutput_items, nitems_written(0)); fflush(stdout);
	 d_state = false;
	 //pmt::pmt_t value = pmt::PMT_F;
	 d_key = pmt::pmt_string_to_symbol("tx_eob");
	 d_id = pmt::pmt_string_to_symbol(this->name());
	 pmt::pmt_t value = pmt::PMT_T;
	 add_item_tag(0, nitems_written(0)+i, d_key, value, d_id);
       }
    }
  }

  return noutput_items;
}

/* removes the tag from port1 and puts it onto port0 - port 1 has some issues (I think) */
inline void
gr_burst_tagger::modify_timestamp(int output_items, int index) {
  //printf("modify_timestamp, output_items: %d\n", output_items); fflush(stdout);
  unsigned int tag_port = 1;
  std::vector<gr_tag_t> rx_tags;
  const uint64_t nread1 = nitems_read(tag_port);
  get_tags_in_range(rx_tags, tag_port, nread1, nread1+output_items, pmt::pmt_string_to_symbol("tx_time"));

  set_tag_propagation_policy(TPP_DONT);                 // stop this tag propagating downstream //

  if(rx_tags.size()>0) {
     size_t t = rx_tags.size()-1;
     uint64_t offset = rx_tags[t].offset;

     printf("test_timestamp1 (BURST):: found %d tags, offset: %llu, output_items: %d, nread1: %llu, nwritten1: %llu, index: %d\n", rx_tags.size(), rx_tags[t].offset, output_items, nread1, nitems_written(0), index); fflush(stdout);

     const pmt::pmt_t &value = rx_tags[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));


     // instead, add the time tag now //
     const pmt::pmt_t _key = pmt::pmt_string_to_symbol("tx_time");
     const pmt::pmt_t _value = pmt::pmt_make_tuple(
                  pmt::pmt_from_uint64(sync_secs),
                  pmt::pmt_from_double(sync_frac_of_secs)
                  );

     const pmt::pmt_t srcid = pmt::pmt_string_to_symbol(this->name());
     add_item_tag(0/*chan0*/, nitems_written(0)+index, _key, _value, srcid);

  } else {
     assert(false);
     //std::cerr << "ACQ---- Header received, with no sync timestamp1?\n";
  }
}
