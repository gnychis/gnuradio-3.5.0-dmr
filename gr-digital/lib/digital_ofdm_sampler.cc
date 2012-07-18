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

// george includes
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <gruel/pmt.h>
static const pmt::pmt_t TIME_KEY = pmt::pmt_string_to_symbol("rx_time");
static const pmt::pmt_t SYNC_TIME = pmt::pmt_string_to_symbol("sync_time");
#define VERBOSE 0
// Keep track of the RX timestamp
double lts_frac_of_secs;
uint64_t lts_secs;
uint64_t lts_samples_since;

uint64_t last_sync_sec;
double last_sync_frac_sec;

#include <digital_ofdm_sampler.h>
#include <gr_io_signature.h>
#include <gr_expj.h>
#include <cstdio>
#include <boost/math/special_functions/trunc.hpp>
#include <math.h>

// apurv for logging the preamble //
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdexcept>
#include <stdio.h>

#ifdef HAVE_IO_H
#include <io.h>
#endif
#ifdef O_BINARY
#define OUR_O_BINARY O_BINARY
#else
#define OUR_O_BINARY 0
#endif

#ifdef O_LARGEFILE
#define OUR_O_LARGEFILE O_LARGEFILE
#else
#define OUR_O_LARGEFILE 0
#endif
// apurv for logging ends //

digital_ofdm_sampler_sptr
digital_make_ofdm_sampler (unsigned int fft_length, 
		      unsigned int symbol_length,
		      unsigned int timeout)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_sampler (fft_length, symbol_length, timeout));
}

digital_ofdm_sampler::digital_ofdm_sampler (unsigned int fft_length, 
				  unsigned int symbol_length,
				  unsigned int timeout)
  : gr_block ("ofdm_sampler",
	      gr_make_io_signature3 (2, 3, sizeof (gr_complex), sizeof(char), sizeof(gr_complex)),
	      gr_make_io_signature3 (2, 3, sizeof (gr_complex)*fft_length, sizeof(char)*fft_length, sizeof(gr_complex)*fft_length)),
    d_state(STATE_NO_SIG), d_timeout_max(timeout), d_fft_length(fft_length), d_symbol_length(symbol_length),
    d_fp(NULL), d_fd(0), d_file_opened(false) 
{
  lts_samples_since=0;
  set_relative_rate(1.0/(double) fft_length);   // buffer allocator hint

  last_sync_sec = 0;
  last_sync_frac_sec = 0.0;
  d_prev_index = 0;

  std::string arg("");
  d_usrp = uhd::usrp::multi_usrp::make(arg);
}

void
digital_ofdm_sampler::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  // FIXME do we need more
  //int nreqd  = (noutput_items-1) * d_symbol_length + d_fft_length;
  int nreqd  = d_symbol_length + d_fft_length;
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++)
    ninput_items_required[i] = nreqd;
}


int
digital_ofdm_sampler::general_work (int noutput_items,
			       gr_vector_int &ninput_items,
			       gr_vector_const_void_star &input_items,
			       gr_vector_void_star &output_items)
{
  gr_complex *iptr = (gr_complex *) input_items[0];
  const char *trigger = (const char *) input_items[1];

  gr_complex *optr = (gr_complex *) output_items[0];
  char *outsig = (char *) output_items[1];

  // Use the stream tags to the timestamp
  std::vector<gr_tag_t> rx_time_tags;
  const uint64_t nread = this->nitems_read(0); //number of items read on port 0
  this->get_tags_in_range(rx_time_tags, 0, nread, nread+ninput_items[0], TIME_KEY);

  // See if there is a RX timestamp (only on first block or after underrun)
  if(rx_time_tags.size()>0) {
    size_t t = rx_time_tags.size()-1;

    // Take the last timestamp
    const uint64_t sample_offset = rx_time_tags[t].offset;  // distance from sample to timestamp in samples
    const pmt::pmt_t &value = rx_time_tags[t].value;

    // If the offset is greater than 0, this is a bit odd and complicated, so let's throw an error
    // and if this is common, George will fix it.
    if(sample_offset>0) {
	 std::cerr << "----- ERROR:  RX Time offset > 0, George will fix if this is common\n";
	 exit(-1);
    }

    // Now, compute the actual time in seconds and fractional seconds of the preamble
    lts_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));
    lts_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));

    printf("(SAMPLER): found TIME_KEY.. lts_secs: %llu, lts_frac_of_secs: %f\n", lts_secs, lts_frac_of_secs); fflush(stdout);
  }

  //FIXME: we only process a single OFDM symbol at a time; after the preamble, we can 
  // process a few at a time as long as we always look out for the next preamble.

  unsigned int index=d_fft_length;  // start one fft length into the input so we can always look back this far

  outsig[0] = 0; // set output to no signal by default

  unsigned int delta = 0;

  // Search for a preamble trigger signal during the next symbol length
  while((d_state != STATE_PREAMBLE) && (index <= (d_symbol_length+d_fft_length))) {
    if(trigger[index]) {

      unsigned int allowed_misalignment = 2;
      unsigned int gap = 4200;
      unsigned left_boundary = gap - allowed_misalignment;
      unsigned right_boundary = gap + allowed_misalignment;

      uint64_t samples_passed = lts_samples_since + index;
      unsigned int obs_gap = samples_passed - d_prev_index;

      if(obs_gap > left_boundary && obs_gap < right_boundary) {
	  samples_passed = d_prev_index + gap;
	  index += (gap - obs_gap);
	  delta = gap - obs_gap; 
	  printf("index: %u, gap: %d, obs_gap: %d, delta: %d\n", index, gap, obs_gap, delta); fflush(stdout);
      }


      d_prev_index = samples_passed;


      outsig[0] = 1; // tell the next block there is a preamble coming
      d_state = STATE_PREAMBLE;

      // The analog to digital converter is 400 million samples / sec.  That translates to 
      // 2.5ns of time for every sample.
      int decimation = 128;
      double rate = 1.0/decimation;

      double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
      //uint64_t samples_passed = lts_samples_since + index;
      double elapsed = samples_passed * time_per_sample;

      printf("sample index (PREAMBLE): %llu\n", samples_passed); fflush(stdout);      

      // Use the last time stamp to calculate the time of the premable synchronization
      uint64_t sync_sec = (int)elapsed + lts_secs;
      double sync_frac_sec = elapsed - (int)elapsed + lts_frac_of_secs;
      if(sync_frac_sec>1) {
        sync_sec += (uint64_t)sync_frac_sec; 
        sync_frac_sec -= (uint64_t)sync_frac_sec;
      }

      uint64_t interval_sec = sync_sec - last_sync_sec;
      double interval_frac_sec = sync_frac_sec - last_sync_frac_sec;

      if(VERBOSE || 1) {
        std::cout << "got a preamble.... calculating timestamp of sync\n";
        std::cout << "... relative_rate: " << rate << "\n";
        std::cout << "... time_per_sample: " << time_per_sample << "\n";
        std::cout << "... samples_passed: " << samples_passed << "\n";
        std::cout << "... elapsed: "<< elapsed << "\n";
        std::cout << "... sync_sec: "<< sync_sec << "\n";
        std::cout << "... sync_fs: "<< sync_frac_sec << "\n";
	if(last_sync_sec != 0) {
	   std::cout << "... interval_sec: " << interval_sec << "... interval_frac_sec: " << interval_frac_sec << "... last_sync_sec: " << last_sync_sec << "... last_sync_frac_sec: "<< last_sync_frac_sec << std::endl;
	}
      }

      // Pack up our time of synchronization, pass it along using the stream tags
      gr_tag_t tag;   // create a new tag
      tag.srcid = pmt::pmt_string_to_symbol(name());    // to know the source block that created tag
      tag.offset=nitems_written(1);     // the offset in the sample stream that we found this tag
                                                 // 17 is a magic number which was the offset i found them at (decode length?)
      tag.key=SYNC_TIME;    // the "key" of the tag, which I've defined to be "SYNC_TIME"

      /*
      tag.value = pmt::pmt_make_tuple(
          pmt::pmt_from_uint64((int)elapsed),      // FPGA clock in seconds that we found the sync
          pmt::pmt_from_double(elapsed - (int)elapsed)  // FPGA clock in fractional seconds that we found the sync
        ); */

      tag.value = pmt::pmt_make_tuple(
          pmt::pmt_from_uint64(sync_sec),      // FPGA clock in seconds that we found the sync
          pmt::pmt_from_double(sync_frac_sec)  // FPGA clock in fractional seconds that we found the sync
        );

      //add_item_tag(1, tag);

      add_item_tag(1, tag.offset, tag.key, tag.value, tag.srcid);
      uhd::time_spec_t proc_time = d_usrp->get_time_now() - uhd::time_spec_t(sync_sec, sync_frac_sec);
  
      printf("(SAMPLER) RX timestamp (%llu, %f), proc_time (%llu, %f)\n", sync_sec, sync_frac_sec, (uint64_t) proc_time.get_full_secs(), proc_time.get_frac_secs());
      fflush(stdout);

      if(1) 
        std::cout << "--- added sync tag in ofdm_sampler stream at " << nitems_written(1) << "\n";
      if(1)
        std::cout << "--- found sync at: " << (int)elapsed << " and " << elapsed-(int)elapsed << " (" << elapsed << ")\n";

    }
    else
      index++;
  }

  unsigned int i, pos, ret;
  switch(d_state) {
  case(STATE_PREAMBLE):
    // When we found a preamble trigger, get it and set the symbol boundary here
    for(i = (index - d_fft_length + 1); i <= index; i++) {
      *optr++ = iptr[i];
    }
   
    //log_preamble(iptr, (index-d_fft_length+1));
 
    d_timeout = d_timeout_max; // tell the system to expect at least this many symbols for a frame
    d_state = STATE_FRAME;
 
    consume_each(index - d_fft_length + 1); // consume up to one fft_length away to keep the history
    lts_samples_since += index - d_fft_length + 1;
    ret = 1;
    break;
    
  case(STATE_FRAME):
    // use this state when we have processed a preamble and are getting the rest of the frames
    //FIXME: we could also have a power squelch system here to enter STATE_NO_SIG if no power is received

    // skip over fft length history and cyclic prefix
    pos = d_symbol_length;         // keeps track of where we are in the input buffer
    while(pos < d_symbol_length + d_fft_length) {
      *optr++ = iptr[pos++];
    }

    if(d_timeout-- == 0) {
      printf("TIMEOUT\n");
      d_state = STATE_NO_SIG;
    }

    consume_each(d_symbol_length); // jump up by 1 fft length and the cyclic prefix length
    lts_samples_since += d_symbol_length;
    ret = 1;
    break;

  case(STATE_NO_SIG):
  default:
    consume_each(index-d_fft_length); // consume everything we've gone through so far leaving the fft length history
    lts_samples_since += index-d_fft_length;
    ret = 0;
    break;
  }

  return ret;
}

void
digital_ofdm_sampler::log_preamble(gr_complex *iptr, long index) {
  if(!d_file_opened)
  {
      d_file_opened = open_log();
      assert(d_file_opened);
      fprintf(stderr, "preambles file opened!\n");
  }
  assert(d_fp != NULL);
  int count = ftell(d_fp);
  count = fwrite_unlocked(iptr + index, sizeof(gr_complex), d_fft_length, d_fp);	// to log preamble
  //count = fwrite_unlocked(iptr + index, sizeof(gr_complex), d_symbol_length, d_fp);	// log anything ;)
}

bool
digital_ofdm_sampler::open_log()
{
  printf("open_log called\n"); fflush(stdout);

  // open the preamble log file //
  char *filename = "preamble.dat";
  if ((d_fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     return false;
  }
  else {
      if((d_fp = fdopen (d_fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "preambles file cannot be opened\n");
            close(d_fd);
            return false;
      }
  }
  return true;
}
