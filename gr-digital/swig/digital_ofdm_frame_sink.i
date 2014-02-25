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
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

GR_SWIG_BLOCK_MAGIC(digital,ofdm_frame_sink);

digital_ofdm_frame_sink_sptr 
digital_make_ofdm_frame_sink(const std::vector<gr_complex> &hdr_sym_position, 
			const std::vector<unsigned char> &hdr_sym_value_out,
                        const std::vector<gr_complex> &data_sym_position,
                        const std::vector<unsigned char> &data_sym_value_out,
                        const std::vector<std::vector<gr_complex> > &preamble,
			gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
                        unsigned int occupied_tones, unsigned int fft_length,
			float phase_gain=0.25, float freq_gain=0.25*0.25/4,
                        unsigned int id=1, 
                        unsigned int batch_size=1, unsigned int decode_flag=1, 
                        int fwd_index=0, int replay_flag=0,
                        int exp_size=400, int fec_n=0, int fec_k=0, int degree=4, int h_coding=0, int _source=0);

class digital_ofdm_frame_sink : public gr_sync_block
{
 protected:
  digital_ofdm_frame_sink(const std::vector<gr_complex> &hdr_sym_position, 
		     const std::vector<unsigned char> &hdr_sym_value_out,
                     const std::vector<gr_complex> &data_sym_position,
                     const std::vector<unsigned char> &data_sym_value_out,
                     const std::vector<std::vector<gr_complex> > &preamble,
		     gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
                     unsigned int occupied_tones, unsigned int fft_length,
		     float phase_gain, float freq_gain, unsigned int id,
                     unsigned int batch_size, unsigned int decode_flag,
                     int fwd_index, int replay_flag,
                     int exp_size, int fec_n, int fec_k, int degree, int h_coding, int _source);

 public:
  ~digital_ofdm_frame_sink();
  /*void makePacket();*/
  void send_ack(unsigned char flow_id, unsigned char batch_id);
  int okToTx();
  void disableOkToTx();
};
