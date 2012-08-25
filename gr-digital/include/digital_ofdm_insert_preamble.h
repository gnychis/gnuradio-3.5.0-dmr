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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef INCLUDED_DIGITAL_OFDM_INSERT_PREAMBLE_H
#define INCLUDED_DIGITAL_OFDM_INSERT_PREAMBLE_H

#include <digital_api.h>
#include <gr_block.h>
#include <vector>

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

class digital_ofdm_insert_preamble;
typedef boost::shared_ptr<digital_ofdm_insert_preamble> digital_ofdm_insert_preamble_sptr;

DIGITAL_API digital_ofdm_insert_preamble_sptr
digital_make_ofdm_insert_preamble(int fft_length, unsigned int fwd_index,
			     const std::vector<std::vector<gr_complex> > &preamble);

/*!
 * \brief insert "pre-modulated" preamble symbols before each payload.
 * \ingroup sync_blk
 * \ingroup ofdm_blk
 *
 * <pre>
 * input 1: stream of vectors of gr_complex [fft_length]
 *          These are the modulated symbols of the payload.
 *
 * input 2: stream of char.  The LSB indicates whether the corresponding
 *          symbol on input 1 is the first symbol of the payload or not.
 *          It's a 1 if the corresponding symbol is the first symbol,
 *          otherwise 0.
 *
 * N.B., this implies that there must be at least 1 symbol in the payload.
 *
 *
 * output 1: stream of vectors of gr_complex [fft_length]
 *           These include the preamble symbols and the payload symbols.
 *
 * output 2: stream of char.  The LSB indicates whether the corresponding
 *           symbol on input 1 is the first symbol of a packet (i.e., the
 *           first symbol of the preamble.)   It's a 1 if the corresponding
 *           symbol is the first symbol, otherwise 0.
 * </pre>
 *
 * \param fft_length length of each symbol in samples.
 * \param preamble   vector of symbols that represent the pre-modulated preamble.
 */

class DIGITAL_API digital_ofdm_insert_preamble : public gr_block
{
  friend DIGITAL_API digital_ofdm_insert_preamble_sptr
  digital_make_ofdm_insert_preamble(int fft_length, unsigned int fwd_index,
			       const std::vector<std::vector<gr_complex> > &preamble);

protected:
  digital_ofdm_insert_preamble(int fft_length, unsigned int fwd_index,
			  const std::vector<std::vector<gr_complex> > &preamble);

private:
  enum state_t {
    ST_IDLE,
    ST_NULL_SYMBOLS,
    ST_PREAMBLE,
    ST_FIRST_PAYLOAD,
    ST_PAYLOAD
  };

  int						d_fft_length;
  const std::vector<std::vector<gr_complex> > 	d_preamble;
  state_t					d_state;
  int						d_nsymbols_output;
  int						d_pending_flag;

  void enter_idle();
  void enter_preamble();
  void enter_first_payload();
  void enter_payload();
  

public:
  ~digital_ofdm_insert_preamble();

  int general_work (int noutput_items,
		    gr_vector_int &ninput_items,
		    gr_vector_const_void_star &input_items,
		    gr_vector_void_star &output_items);

  /* timestamping related stuff */
  void track_and_modify_timestamp(int output_items);
  uint64_t d_p_index;

  /* log the preamble being transmitted (enable if required) */
  FILE *d_fp;
  void log_preamble();
  void open_log();

  unsigned int d_fwd_index;
  bool d_timestamp;
};

#endif /* INCLUDED_GR_OFDM_INSERT_PREAMBLE_H */
