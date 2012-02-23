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

#ifndef INCLUDED_DIGITAL_OFDM_MAPPER_BCV_H
#define INCLUDED_DIGITAL_OFDM_MAPPER_BCV_H

#include <digital_api.h>
#include <cstdio>
#include <gr_sync_block.h>
#include <gr_message.h>
#include <gr_msg_queue.h>
//#include <digital_ofdm_frame_sink.h> 

/* apurv++ define header type */
/*
#define MAX_BATCH_SIZE  8 
#define BATCH_SIZE      3 
#define PADDING_SIZE    2 
#define ACK_PADDING_SIZE 2 

#define DATA_TYPE       0
#define ACK_TYPE        1
#define TRIGGER_TYPE    2

#pragma pack(1)
typedef struct multihop_hdr_type {

  unsigned char src_id : 3;
  unsigned char dst_id  : 3;
  unsigned char flow_id : 2;

  unsigned short packetlen: 12;
  unsigned char batch_number : 8;
  unsigned char nsenders : 2;

  unsigned char pkt_type : 1;
  unsigned char lead_sender: 1;
  unsigned char prev_hop_id;                       // can be reduced to bit-field //

  unsigned int pkt_num;
  unsigned int coeffs[BATCH_SIZE];
  unsigned int hdr_crc;
  unsigned char pad[PADDING_SIZE];                 // to ensure size % (occupied_carriers-dc_carriers) = 0

} MULTIHOP_HDR_TYPE;

#pragma pack(1)
typedef struct ack_multihop_hdr_type {

  unsigned char pkt_type : 1;
  unsigned char flow_id : 4;
  unsigned char batch_number : 8;

  unsigned char src_id : 4;
  unsigned char dst_id  : 4;
  unsigned char prev_hop_id: 3;                         // ACK's prev hop  

  unsigned int hdr_crc;

  unsigned char pad[ACK_PADDING_SIZE];                 // to ensure size % (occupied_carriers-dc_carriers) = 0

} MULTIHOP_ACK_HDR_TYPE;
*/
/* apurv++ end header type */

class digital_ofdm_mapper_bcv;
typedef boost::shared_ptr<digital_ofdm_mapper_bcv> digital_ofdm_mapper_bcv_sptr;

DIGITAL_API digital_ofdm_mapper_bcv_sptr 
digital_make_ofdm_mapper_bcv (const std::vector<gr_complex> &constellation, unsigned msgq_limit, 
			 unsigned occupied_carriers, unsigned int fft_length);

/*!
 * \brief take a stream of bytes in and map to a vector of complex
 * constellation points suitable for IFFT input to be used in an ofdm
 * modulator.  Abstract class must be subclassed with specific mapping.
 * \ingroup modulation_blk
 * \ingroup ofdm_blk
 */

class DIGITAL_API digital_ofdm_mapper_bcv : public gr_sync_block
{
  friend DIGITAL_API digital_ofdm_mapper_bcv_sptr
  digital_make_ofdm_mapper_bcv (const std::vector<gr_complex> &constellation, unsigned msgq_limit, 
			   unsigned occupied_carriers, unsigned int fft_length);
 protected:
  digital_ofdm_mapper_bcv (const std::vector<gr_complex> &constellation, unsigned msgq_limit, 
		      unsigned occupied_carriers, unsigned int fft_length);

 private:
  std::vector<gr_complex> d_constellation;
  gr_msg_queue_sptr	d_msgq;
  gr_message_sptr	d_msg;
  unsigned		d_msg_offset;
  bool			d_eof;
  
  unsigned int 		d_occupied_carriers;
  unsigned int 		d_fft_length;
  unsigned int 		d_bit_offset;
  int			d_pending_flag;

  unsigned long  d_nbits;
  unsigned char  d_msgbytes;
  
  unsigned char d_resid;
  unsigned int d_nresid;

  std::vector<int> d_subcarrier_map;

  int randsym();

 public:
  ~digital_ofdm_mapper_bcv(void);

  gr_msg_queue_sptr	msgq() const { return d_msgq; }

  int work(int noutput_items,
	   gr_vector_const_void_star &input_items,
	   gr_vector_void_star &output_items);

  
  /* apurv++ starts */
  bool d_ack;
  bool d_data;
  void generateOFDMSymbol(gr_complex* out, int len);
  void copyOFDMSymbol(gr_complex *out, int len);

  /* data */
  static const int HEADERBYTELEN = sizeof(MULTIHOP_HDR_TYPE);
  static const int HEADERDATALEN = HEADERBYTELEN-PADDING_SIZE-4;

  /* ack */
  static const int ACK_HEADERBYTELEN = sizeof(MULTIHOP_ACK_HDR_TYPE);
  static const int ACK_HEADERDATALEN = ACK_HEADERBYTELEN - ACK_PADDING_SIZE-4;

  unsigned int d_ofdm_index;
  bool d_default;
};

#endif
