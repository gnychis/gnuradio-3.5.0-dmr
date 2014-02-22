/* -*- c++ -*- */
/*
 * Copyright 2006,2007 Free Software Foundation, Inc.
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
GR_SWIG_BLOCK_MAGIC(digital,ofdm_mapper_bcv);

digital_ofdm_mapper_bcv_sptr
digital_make_ofdm_mapper_bcv (const std::vector<gr_complex> &hdr_constellation,
                         const std::vector<gr_complex> &data_constellation,
                         const std::vector<std::vector<gr_complex> > &preamble,
                         unsigned int msgq_limit,
                         unsigned int bits_per_symbol,
                         unsigned int fft_length,
                         unsigned int id=1,
                         unsigned int source_flag=0,
                         unsigned int batch_size=1,
                         unsigned int encode_flag=1,
                         int fwd_index=0, unsigned int dst_id=2,
                         unsigned int degree=4,
                         unsigned int mimo=0, int h_coding=0, int flow=0) throw(std::exception);


class digital_ofdm_mapper_bcv : public gr_sync_block
{
 protected:
  digital_ofdm_mapper_bcv (const std::vector<gr_complex> &hdr_constellation,
                      const std::vector<gr_complex> &data_constellation,
                      const std::vector<std::vector<gr_complex> > &preamble,
                      unsigned int msgq_limit,
                      unsigned int bits_per_symbol,
                      unsigned int fft_length,
                      unsigned int id,
                      unsigned int source_flag,
                      unsigned int batch_size,
                      unsigned int encode_flag,
                      int fwd_index, unsigned int dst_id,
                      unsigned int degree,
                      unsigned int mimo, int h_coding, int flow);

 public:
  gr_msg_queue_sptr msgq();
  void set_batch_to_send(int batch_id);

  int work(int noutput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

  int isACKSocketOpen();
};
