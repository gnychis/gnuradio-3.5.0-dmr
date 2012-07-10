/* -*- c++ -*- */
/*
 * Copyright 2007,2010 Free Software Foundation, Inc.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <digital_ofdm_insert_preamble.h>
#include <gr_io_signature.h>
#include <stdexcept>
#include <iostream>
#include <string.h>
#include <cstdio>

digital_ofdm_insert_preamble_sptr
digital_make_ofdm_insert_preamble(int fft_length,
			     const std::vector<std::vector<gr_complex> > &preamble)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_insert_preamble(fft_length,
								  preamble));
}

digital_ofdm_insert_preamble::digital_ofdm_insert_preamble
       (int fft_length,
	const std::vector<std::vector<gr_complex> > &preamble)
  : gr_block("ofdm_insert_preamble",
	     gr_make_io_signature2(2, 2,
				   sizeof(gr_complex)*fft_length,
				   sizeof(char)),
	     gr_make_io_signature4(1, 4,
				   sizeof(gr_complex)*fft_length,
				   sizeof(short),					// apurv++: for george (burst tagger implementation)
				   sizeof(char)*fft_length,					// apurv++
				   sizeof(char))),
    d_fft_length(fft_length),
    d_preamble(preamble),
    d_state(ST_IDLE),
    d_nsymbols_output(0),
    d_pending_flag(0),
    d_p_index(0)
{
  // sanity check preamble symbols
  for (size_t i = 0; i < d_preamble.size(); i++){
    if (d_preamble[i].size() != (size_t) d_fft_length)
      throw std::invalid_argument("digital_ofdm_insert_preamble: invalid length for preamble symbol");
  }

  enter_idle();
}


digital_ofdm_insert_preamble::~digital_ofdm_insert_preamble()
{
}

int
digital_ofdm_insert_preamble::general_work (int noutput_items,
				       gr_vector_int &ninput_items_v,
				       gr_vector_const_void_star &input_items,
				       gr_vector_void_star &output_items)
{
  int ninput_items = std::min(ninput_items_v[0], ninput_items_v[1]);
  const gr_complex *in_sym = (const gr_complex *) input_items[0];
  const unsigned char *in_flag = (const unsigned char *) input_items[1];

  gr_complex *out_sym = (gr_complex *) output_items[0];
  unsigned char *out_flag = 0;
  if (output_items.size() >= 4)
    out_flag = (unsigned char *) output_items[3];


  /* for burst tagger trigger */
  unsigned short *burst_trigger = NULL;
  if(output_items.size() >= 2) {
     burst_trigger = (unsigned short*) output_items[1];
     memset(burst_trigger, 0, sizeof(short));
  }
  /* end */
  
  /* apurv++ */
  unsigned char *out_signal = NULL;
  if (output_items.size() >= 3){
    out_signal = (unsigned char*) output_items[2];
    memset(out_signal, 0, sizeof(char) * d_fft_length);
  }
  /* apurv++ end */

  int no = 0;	// number items output
  int ni = 0;	// number items read from input


#define write_out_flag() 			\
  do { if (out_flag) 				\
          out_flag[no] = d_pending_flag; 	\
       d_pending_flag = 0; 			\
  } while(0)


  while (no < noutput_items && ni < ninput_items){
    switch(d_state){
    case ST_IDLE:
      if ((in_flag[ni] == 1))	// this is first symbol of new payload
	enter_preamble();
      else
	ni++;			// eat one input symbol
      break;
      
    case ST_PREAMBLE:
      assert(in_flag[ni] == 1);

      if (d_nsymbols_output >= (int) d_preamble.size()){
	// we've output all the preamble
	enter_first_payload();
      }
      else {
	memcpy(&out_sym[no * d_fft_length],
	       &d_preamble[d_nsymbols_output][0],
	       d_fft_length*sizeof(gr_complex));

	write_out_flag();

        /* apurv++ start */
        // for burst tagger trigger: mark all the samples of the preamble as '1' //
        if(output_items.size() >= 2) {
          burst_trigger[no] = 1;
        }

	if (output_items.size() >= 3){
          memset(&out_signal[no * d_fft_length], 0, sizeof(char) * d_fft_length);
          out_signal[no * d_fft_length] = 1;
	}
	/* apurv++ end */

	no++;
	d_nsymbols_output++;
      }
      break;
      
    case ST_FIRST_PAYLOAD:
      // copy first payload symbol from input to output
      memcpy(&out_sym[no * d_fft_length],
	     &in_sym[ni * d_fft_length],
	     d_fft_length * sizeof(gr_complex));


      /* apurv++ start */
      // for burst tagger trigger: mark all the samples of the data as '1' as well //
      if(output_items.size() >= 2) {
         burst_trigger[no] = 1;
      }

      if (output_items.size() >= 3){
        memset(&out_signal[no * d_fft_length], 0, sizeof(char) * d_fft_length);
      }
      /* apurv++ end */


      write_out_flag();
      no++;
      ni++;
      enter_payload();
      break;
      
    case ST_PAYLOAD:
      if ((in_flag[ni] == 1)){	// this is first symbol of a new payload
	enter_preamble();
	break;
      }

      // copy a symbol from input to output
      memcpy(&out_sym[no * d_fft_length],
	     &in_sym[ni * d_fft_length],
	     d_fft_length * sizeof(gr_complex));

      /* apurv++ start */
      // for burst tagger trigger: mark all samples of data as '1' as well //
      if(output_items.size() >= 2) {
        burst_trigger[no] = 1;
        // handle the last OFDM symbol //
        if(in_flag[ni] == 2) {
           // in this case, mark the last sample as '0' to mark the end of the packet //
           burst_trigger[no] = 0;
        }
      }

      if (output_items.size() >= 3){
        memset(&out_signal[no * d_fft_length], 0, sizeof(char) * d_fft_length);
      }
      /* apurv++ end */

      write_out_flag();
      no++;
      ni++;
      break;

    default:
      std::cerr << "digital_ofdm_insert_preamble: (can't happen) invalid state, resetting\n";
      enter_idle();
    }
  }

  track_and_modify_timestamp(ni);

  consume_each(ni);
  return no;
}

void
digital_ofdm_insert_preamble::enter_idle()
{
  d_state = ST_IDLE;
  d_nsymbols_output = 0;
  d_pending_flag = 0;
}

void
digital_ofdm_insert_preamble::enter_preamble()
{
  d_state = ST_PREAMBLE;
  d_nsymbols_output = 0;
  d_pending_flag = 1;
}

void
digital_ofdm_insert_preamble::enter_first_payload()
{
  d_state = ST_FIRST_PAYLOAD;
}

void
digital_ofdm_insert_preamble::enter_payload()
{
  d_state = ST_PAYLOAD;
}

/* extract the timestamp and re-insert it at the correct offset - the offset will change because of the preamble OFDM symbol
   added in this block */
inline void
digital_ofdm_insert_preamble::track_and_modify_timestamp(int output_items) {
  //printf("test_timestamp (INS_PREAMBLE), output_items: %d, nread1: %llu\n", output_items, nitems_read(1)); fflush(stdout);
  unsigned int tag_port = 1;
  std::vector<gr_tag_t> rx_tags;
  const uint64_t nread1 = nitems_read(tag_port);
  get_tags_in_range(rx_tags, tag_port, nread1, nread1+output_items, pmt::pmt_string_to_symbol("tx_time"));

  set_tag_propagation_policy(TPP_DONT);			// stop this tag propagating downstream //

  if(rx_tags.size()>0) {
     size_t t = rx_tags.size()-1;
     uint64_t offset = rx_tags[t].offset;

     //printf("test_timestamp1 (INS_PREAMBLE):: found %d tags, offset: %llu, output_items: %d, nread1: %llu\n", rx_tags.size(), rx_tags[t].offset, output_items, nread1); fflush(stdout);

     const pmt::pmt_t &value = rx_tags[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));

     // instead, add the time tag now //
     offset += d_p_index;				// since the preamble symbol is added in this block!
     const pmt::pmt_t _key = pmt::pmt_string_to_symbol("tx_time");
     const pmt::pmt_t _value = pmt::pmt_make_tuple(
                  pmt::pmt_from_uint64(sync_secs),
                  pmt::pmt_from_double(sync_frac_of_secs)
                  );

     const pmt::pmt_t srcid = pmt::pmt_string_to_symbol(this->name());
     add_item_tag(1/*chan0*/, offset, _key, _value, srcid);

     d_p_index += 1;					// for the preamble! 
  } else {
     //std::cerr << "ACQ---- Header received, with no sync timestamp1?\n";
  }
}
