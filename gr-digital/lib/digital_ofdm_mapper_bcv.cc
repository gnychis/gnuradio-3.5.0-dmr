/* -*- c++ -*- */
/*
 * Copyright 2006,2007,2008,2010 Free Software Foundation, Inc.
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

#include <digital_ofdm_mapper_bcv.h>
#include <gr_io_signature.h>
#include <stdexcept>
#include <string.h>
#include <cstdio> 
#include <gr_expj.h>
#include <time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>

#include "armadillo"

using namespace arma;
//#define TESTING 0
//#define SCALE 1e3

#define SCALE_FACTOR_PHASE 1e2
#define SCALE_FACTOR_AMP 1e4

//#define ACK_ON_ETHERNET 1
//#define TRIGGER_ON_ETHERNET 1

digital_ofdm_mapper_bcv_sptr
digital_make_ofdm_mapper_bcv (const std::vector<gr_complex> &hdr_constellation, 
			 const std::vector<gr_complex> &data_constellation,
			 const std::vector<std::vector<gr_complex> > &preamble,
			 unsigned int msgq_limit, 
                         unsigned int occupied_carriers, unsigned int fft_length, unsigned int id,
			 unsigned int source,
			 unsigned int batch_size,
			 unsigned int encode_flag,
			 int fwd_index, unsigned int dst_id, unsigned int degree,
			 unsigned int mimo, int h_coding)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_mapper_bcv (hdr_constellation, data_constellation, preamble, msgq_limit, occupied_carriers, fft_length, id, source, batch_size, encode_flag, fwd_index, dst_id, degree, mimo, h_coding));
}

// Consumes 1 packet and produces as many OFDM symbols of fft_length to hold the full packet
digital_ofdm_mapper_bcv::digital_ofdm_mapper_bcv (const std::vector<gr_complex> &hdr_constellation, 
					const std::vector<gr_complex> &data_constellation,
					const std::vector<std::vector<gr_complex> > &preamble,
					unsigned int msgq_limit, 
					unsigned int occupied_carriers, unsigned int fft_length, unsigned int id,
					unsigned int source,
					unsigned int batch_size,
					unsigned int encode_flag,
					int fwd_index, unsigned int dst_id, unsigned int degree, 
					unsigned int mimo, int h_coding)
  : gr_sync_block ("ofdm_mapper_bcv",
		   gr_make_io_signature (0, 0, 0),
		   gr_make_io_signature4 (1, 4, sizeof(gr_complex)*fft_length, sizeof(short), sizeof(char), sizeof(char)*fft_length)),
    d_hdr_constellation(hdr_constellation),
    d_data_constellation(data_constellation),
    d_msgq(gr_make_msg_queue(msgq_limit)), d_eof(false),
    d_occupied_carriers(occupied_carriers),
    d_fft_length(fft_length),
    d_pending_flag(0),
    d_batch_q_num(-1),
    d_batch_size(batch_size),
    d_batch_to_send(0),
    d_packets_sent_for_batch(0),	// for testing only //
    d_source(source),
    d_modulated(true),	// start afresh //
    d_send_null(false),
    d_pkt_num(0),
    d_id(id + '0'),
    d_encode_flag(encode_flag),
    d_sock_opened(false),
    d_time_tag(false),
    d_trigger_sock_opened(false),
    d_fwd_index(fwd_index),
    d_dst_id(dst_id),
    d_degree(degree),
    d_mimo(mimo),
    d_h_coding(h_coding),
    d_preambles_sent(0),
    d_preamble(preamble),
    d_timing_offset(0.0)
{
  if (!(d_occupied_carriers <= d_fft_length))
    throw std::invalid_argument("digital_ofdm_mapper_bcv: occupied carriers must be <= fft_length");

  // sanity check preamble symbols
  for (size_t i = 0; i < d_preamble.size(); i++){
    if (d_preamble[i].size() != (size_t) d_fft_length)
      throw std::invalid_argument("digital_ofdm_mapper_bcv: invalid length for preamble symbol");
  }

  // MIMO_TX
  if(d_mimo != 0) {
     setup_mimo_tx();
  }

#ifdef ACK_ON_ETHERNET
  d_sock_fd = openACKSocket();
#endif

  std::string arg("");
  d_usrp = uhd::usrp::multi_usrp::make(arg);
  d_out_pkt_time = uhd::time_spec_t(0.0);
  d_last_pkt_time = uhd::time_spec_t(0.0);

  assign_subcarriers();

  // make sure we stay in the limit currently imposed by the occupied_carriers
  if(d_data_carriers.size() > d_occupied_carriers) {
    throw std::invalid_argument("digital_ofdm_mapper_bcv: subcarriers allocated exceeds size of occupied carriers");
  }
  fill_all_carriers_map();  

  d_hdr_nbits = (unsigned long)ceil(log10(float(d_hdr_constellation.size())) / log10((float) 2.0));
  d_data_nbits = (unsigned long)ceil(log10(float(d_data_constellation.size())) / log10((float) 2.0));

  printf("d_data_nbits: %d, const size: %d, d_hdr_bits: %d, const size: %d\n", d_data_nbits, d_data_constellation.size(), d_hdr_nbits, d_hdr_constellation.size()); fflush(stdout);

  // for offline analysis - apurv++ //
  d_fp_native_symbols = NULL;
  d_fp_coeff = NULL;
  d_init_log = false;
  d_num_ofdm_symbols = 0;
  d_ofdm_symbol_index = 0;
  // apurv++ end //

  d_num_hdr_symbols = (HEADERBYTELEN*8)/d_data_carriers.size();

  /* for mimo_tx, rand_seed = 1, else rand_seed = d_id */
  if(d_mimo != 0) 
     srand(1);
  else
     srand(d_id);

  d_time_pkt_sent = 0;
  d_log_open = false;
  //d_log_open_native = false;

  if(NUM_TRAINING_SYMBOLS > 0)
     generateKnownSymbols();

  populateCompositeLinkInfo();

  if(d_h_coding && d_source == 1)
     prepare_H_coding();

  //opening files //
  int fd;
  assert((fd = open ("tx_symbols_before_norm.dat", O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) >= 0);
  assert((d_fp_log_BN = fdopen (fd, true ? "wb" : "w")) != NULL);  

  fd = 0;
  assert((fd = open ("tx_symbols_after_norm.dat", O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) >= 0);
  assert((d_fp_log_AN = fdopen (fd, true ? "wb" : "w")) != NULL);
}

digital_ofdm_mapper_bcv::~digital_ofdm_mapper_bcv(void)
{
}

int
digital_ofdm_mapper_bcv::work(int noutput_items,
  			      gr_vector_const_void_star &input_items,
			      gr_vector_void_star &output_items)
{
  if(d_source) {
     return work_source(noutput_items, input_items, output_items);
  }

  
  gr_complex *out = (gr_complex *)output_items[0];
  
  if(d_eof) {
    return -1;
  }

  /* optional - for out timing signal required only for debugging */
  unsigned char *out_signal = NULL;
  if (output_items.size() >= 3){
    out_signal = (unsigned char*) output_items[3];
    memset(out_signal, 0, sizeof(char) * d_fft_length);
  }
  /* end */

  if(!d_msg[0]) {
    d_msg[0] = d_msgq->delete_head();			   // block, waiting for a message
    d_hdr_ofdm_index = 0;
    d_data_ofdm_index = 0;
    d_msg_offset[0] = 0;
    d_bit_offset[0] = 0;
    d_nresid[0] = 0;
    d_resid[0] = 0;
    d_pending_flag = 1;					   // new packet, write start of packet flag
    
    if((d_msg[0]->length() == 0) && (d_msg[0]->type() == 1)) {
      assert(false);
      d_msg[0].reset();
      return -1; 					   // We're done; no more messages coming.
    }

    /* apurv++: identify ACK or DATA */
    if(d_msg[0]->type() == ACK_TYPE) {
      printf("ofdm_mapper: sending ACK, msglen: %d, ACK_HEADERBYTELEN: %d\n", d_msg[0]->length(), ACK_HEADERBYTELEN); 
      fflush(stdout);
      d_ack = true;
      d_data = false;
      d_default = false;
    }
    else if(d_msg[0]->type() == DATA_TYPE) {
      printf("ofdm_mapper: sending DATA, type: %d, HEADERBYTELEN: %d, msglen: %d\n", d_msg[0]->type(), HEADERBYTELEN, d_msg[0]->length()); fflush(stdout);
      d_ack = false;
      d_data = true;
      d_default = false;
    }
    else {
      printf("ofdm mapper: send default tx\n"); fflush(stdout);
      d_default = true;
      d_ack = false;
      d_data = false;
    }

    d_send_null = false;
    d_null_symbol_cnt = 0;
    d_training_symbol_cnt = 0;
    d_preambles_sent = 0;
    make_time_tag(d_msg[0]);

    // forwarder needs to extract it from the message already created in frame_sink //
    if(d_h_coding) {
      d_timing_offset = (d_msg[0])->timing_offset();
    }
  }

  char *out_flag = 0;
  if(output_items.size() >= 3)
    out_flag = (char *) output_items[2];
  
  /* for burst tagger trigger */
  unsigned short *burst_trigger = NULL;
  if(output_items.size() >= 2) {
     burst_trigger = (unsigned short*) output_items[1];
     memset(burst_trigger, 0, sizeof(short));
  }
  burst_trigger[0] = 1;
  /* end */

  bool tx_pilot = false;

  /* single OFDM symbol: Initialize all bins to 0 to set unused carriers */
  memset(out, 0, d_fft_length*sizeof(gr_complex));
  if(d_ack || d_default) {
	/* ack */
     generateOFDMSymbol(out, d_msg[0]->length());  			// entire msg needs to be modulated for ACK
  }
  else if(d_data) {

#ifdef SRC_PILOT
     // slave fwd will transmit NULL symbols prior to the header, so that by the time data is sent out, both the senders
     // are at the same offset (to counter the fine offset rotation) 
     if(d_fwd_index == 2 && (d_null_symbol_cnt < (d_preamble.size()+d_num_hdr_symbols)) && 0) {
        d_null_symbol_cnt++;
	d_pending_flag = 3;
     }
     else
#endif
        /* preambles */
     if(d_preambles_sent < d_preamble.size()) {
	generatePreamble(out);
	d_preambles_sent++;
	tx_pilot = false;
	if(d_preambles_sent == 1 && output_items.size() >= 3) out_signal[0]=1;
     }
	/* data */
     else if(d_msg_offset[0] < HEADERBYTELEN) {
	generateOFDMSymbol(out, HEADERBYTELEN);
	tx_pilot = true;
#ifdef SRC_PILOT
	if(d_hdr_ofdm_index == 0) {
	   d_pending_flag = 1;
	}
#endif
	d_hdr_ofdm_index++;
     }
     else if(d_training_symbol_cnt < NUM_TRAINING_SYMBOLS) {
        memcpy(out, d_known_symbols, sizeof(gr_complex) * d_fft_length);
        d_training_symbol_cnt++;
     }
     else if(d_fwd_index == 1 && (d_null_symbol_cnt < (d_preamble.size()+d_num_hdr_symbols+NUM_TRAINING_SYMBOLS))) {	// lead_fwder: send NULL symbols to accomodate slave fwders
	d_null_symbol_cnt++;
     }
     else if(!d_send_null) {

	/* header has already been modulated, just send the payload *symbols* as it is */
 	copyOFDMSymbol(out, d_msg[0]->length());		
	//logGeneratedTxSymbols(out);

	//logNativeTxSymbols(out);
	//printf("data--------- offset: %d\n", d_msg_offset[0]); fflush(stdout);
	d_data_ofdm_index+=1;

	// fwd=1 disables sending of pilot //
 	if(d_fwd_index == 1) {
	    for(int i = 0; i < d_pilot_carriers.size(); i++) {
        	out[d_pilot_carriers[i]] = gr_complex(0.0, 0.0);
     	    }
	}
     }  
  }

  if(tx_pilot) {
      double cur_pilot = 1.0;
      for(int i = 0; i < d_pilot_carriers.size(); i++) {
         out[d_pilot_carriers[i]] = gr_complex(cur_pilot, 0.0);
         cur_pilot = -cur_pilot;
      }
  }
  
  if(d_send_null) {
      d_msg[0].reset();
      d_pending_flag = 2;                                               // marks the last OFDM data symbol (for burst tagger trigger) //
      d_time_tag = false;
      d_send_null = false;
  }
  /* complete message modulated */
  else if(d_msg_offset[0] == d_msg[0]->length()) {
      //d_msg[0].reset();
      printf("num_ofdm_symbols: %d\n", d_hdr_ofdm_index + d_data_ofdm_index); fflush(stdout); 
      //d_pending_flag = 2;						// marks the last OFDM data symbol (for burst tagger trigger) //
      //d_time_tag = false;
      d_send_null = true;
  }

  if(d_h_coding) {
     removeTimingOffset(out, d_timing_offset);
  }

  if (out_flag)
    out_flag[0] = d_pending_flag;

  if(d_pending_flag == 2)
    burst_trigger[0] = 0;

  d_pending_flag = 0;

  return 1;  // produced symbol
} // work() //forwarder

/*
void
digital_ofdm_mapper_bcv::copyOFDMSymbol(gr_complex *out, int len)
{
  printf("copyOfdmsymbol\n"); fflush(stdout);
  unsigned int i = 0;
  unsigned int n_carriers = d_data_carriers.size();
#ifdef SRC_PILOT
  n_carriers += d_pilot_carriers.size();
#endif
  std::cout<<"ncarriers: "<<n_carriers<<endl;

  int dc = 0, pi = 0;
  unsigned int dc_tones = d_occupied_carriers - (d_data_carriers.size() + d_pilot_carriers.size());

  while((d_msg_offset[0] < len) && (i < n_carriers)) {
     if(d_all_carriers[i] == 0) {
	 cout<<"data offset: "<<d_msg_offset[0]<<" pos: "<<d_data_carriers[dc]<<endl;
         memcpy(&out[d_data_carriers[dc++]], d_msg[0]->msg() + d_msg_offset[0], sizeof(gr_complex));
     }
     else if(d_all_carriers[i] == 1) {
	 cout<<"pilot offset: "<<d_msg_offset[0]<<" pos: "<<d_pilot_carriers[pi]<<endl;
	 memcpy(&out[d_pilot_carriers[pi++]], d_msg[0]->msg() + d_msg_offset[0], sizeof(gr_complex));
     }
     d_msg_offset[0] += sizeof(gr_complex);
     i++;
  }

  if(d_msg_offset[0] == len) {
    while(i < n_carriers) {   // finish filling out the symbol
#ifdef SRC_PILOT
    assert(false);
#endif
      out[d_data_carriers[i]] = d_constellation[randsym()];
      i++;
    }
  }
}
*/

void
digital_ofdm_mapper_bcv::copyOFDMSymbol(gr_complex *out, int len)
{
  unsigned int start_offset = d_data_carriers[0];
  unsigned int n_bytes = sizeof(gr_complex) * d_occupied_carriers;

  assert(d_msg_offset[0] + n_bytes <= len);

#ifdef USE_PILOT
  start_offset = (d_data_carriers[0] < d_pilot_carriers[0]) ? d_data_carriers[0]:d_pilot_carriers[0];
#endif
  //printf("start_offset: %d, msg_offset: %d, d: %d, p: %d\n", start_offset, d_msg_offset[0], d_data_carriers[0], d_pilot_carriers[0]); fflush(stdout);
  memcpy(&out[start_offset], d_msg[0]->msg() + d_msg_offset[0], n_bytes);
  d_msg_offset[0] += n_bytes;
}

/* builds a single OFDM symbol */
void
digital_ofdm_mapper_bcv::generateOFDMSymbol(gr_complex* out, int len)
{
  unsigned int i = 0;
  unsigned char bits = 0;
  while((d_msg_offset[0] < len) && (i < d_data_carriers.size())) {

    // need new data to process
    if(d_bit_offset[0] == 0) {
      d_msgbytes[0] = d_msg[0]->msg()[d_msg_offset[0]];
      //printf("mod message byte: %x\n", d_msgbytes);
    }

    if(d_nresid[0] > 0) {
      // take the residual bits, fill out nbits with info from the new byte, and put them in the symbol
      d_resid[0] |= (((1 << d_nresid[0])-1) & d_msgbytes[0]) << (d_hdr_nbits - d_nresid[0]);
      bits = d_resid[0];

      out[d_data_carriers[i]] = d_hdr_constellation[bits];
      i++;

      d_bit_offset[0] += d_nresid[0];
      d_nresid[0] = 0;
      d_resid[0] = 0;
      //     bits, d_resid[0], d_nresid[0], d_bit_offset[0]);
    }
    else {
      if((8 - d_bit_offset[0]) >= d_hdr_nbits) {  // test to make sure we can fit nbits
        // take the nbits number of bits at a time from the byte to add to the symbol
        bits = ((1 << d_hdr_nbits)-1) & (d_msgbytes[0] >> d_bit_offset[0]);
        d_bit_offset[0] += d_hdr_nbits;

        out[d_data_carriers[i]] = d_hdr_constellation[bits];
        i++;
      }
      else {  // if we can't fit nbits, store them for the next 
        // saves d_nresid[0] bits of this message where d_nresid[0] < d_nbits
        unsigned int extra = 8-d_bit_offset[0];
        d_resid[0] = ((1 << extra)-1) & (d_msgbytes[0] >> d_bit_offset[0]);
        d_bit_offset[0] += extra;
        d_nresid[0] = d_hdr_nbits - extra;
      }

    }

    //printf("d_bit_offset[0]: %d, d_msg_offset[0]: %d\n", d_bit_offset[0], d_msg_offset[0]); fflush(stdout);

    if(d_bit_offset[0] == 8) {
      d_bit_offset[0] = 0;
      d_msg_offset[0]++;
    }
  }

  // Ran out of data to put in symbol
  if (d_msg_offset[0] == len) {
    if(d_nresid[0] > 0) {
      d_resid[0] |= 0x00;
      bits = d_resid[0];
      d_nresid[0] = 0;
      d_resid[0] = 0;
    }

    while(i < d_data_carriers.size() && (d_ack || d_default)) {   // finish filling out the symbol
      out[d_data_carriers[i]] = d_hdr_constellation[randsym()];

      i++;
    }

    assert(d_bit_offset[0] == 0);
  }
}


/***************************** integrating SOURCE related functionality to this mapper **************************/
int digital_ofdm_mapper_bcv::randsym()
{
  return (rand() % d_hdr_constellation.size());
}

int
digital_ofdm_mapper_bcv::work_source(int noutput_items,
	  			     gr_vector_const_void_star &input_items,
  			             gr_vector_void_star &output_items)
{
  /* ack over ethernet */
  struct sockaddr_in client_addr;
  int addrlen = sizeof(client_addr);
  
#ifdef ACK_ON_ETHERNET 
  while (!d_sock_opened) {
    d_client_fd = accept(d_sock_fd, (struct sockaddr*)&client_addr, (socklen_t*)&addrlen);
    if (d_client_fd != -1) {
      printf("@ attached rcvr with ACK socket!\n"); fflush(stdout);
      d_sock_opened = true;

      /* make the socket non-blocking */
      //int flags = fcntl(d_client_fd, F_GETFL, 0);
      //fcntl(d_client_fd, F_SETFL, flags|O_NONBLOCK);

      struct timeval tv;
      tv.tv_sec = 1;
      tv.tv_usec = 5e5; 
      assert(setsockopt(d_client_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval))==0);
      break;
    }
  }

  /* once a packet has been *completely* modulated, check for ACK on the backend ethernet socket */
  if(d_modulated) {
    memset(d_ack_buf, 0, sizeof(d_ack_buf));
    int nbytes = recv(d_client_fd, d_ack_buf, sizeof(d_ack_buf), 0);
    printf("n_bytes: %d\n", nbytes); fflush(stdout);
#if 1
    if(nbytes >= 0) {
#if 0
       if(nbytes == ACK_HEADERBYTELEN) {
    	  nbytes = recv(d_client_fd, d_ack_buf, sizeof(d_ack_buf), 0);
	  fflush(stdout);
       }
       else {
 	  nbytes = recv(d_client_fd, d_ack_buf, sizeof(d_ack_buf), MSG_WAITALL); 
       }
#endif
       printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ received nbytes: %d as ACK @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \n", nbytes);

       MULTIHOP_ACK_HDR_TYPE ack_header;
       memcpy(&ack_header, d_ack_buf, sizeof(d_ack_buf));
       printf("ACK details, batch: %d, flow: %d\n", ack_header.batch_number, ack_header.flow_id); fflush(stdout);


       printf("batch: %d ACKed, packets sent for batch: %d, \n", ack_header.batch_number, d_packets_sent_for_batch); fflush(stdout);
       if(d_batch_to_send <= ack_header.batch_number) {
	  d_batch_to_send = ack_header.batch_number + 1;
	  d_packets_sent_for_batch = 0;
       }
    }
#endif
  }
#endif

  //int packetlen = 0;		//only used for make_header//

#ifndef ACK_ON_ETHERNET
  if(d_packets_sent_for_batch == (d_batch_size+1))
  {
	d_packets_sent_for_batch = 0;
	d_batch_to_send += 1;
  }
#endif

  //printf("d_packets_sent: %d, d_batch_size: %d, d_batch_q_num: %d, d_batch_to_send: %d\n", d_packets_sent_for_batch, d_batch_size, d_batch_q_num, d_batch_to_send); fflush(stdout);

  /* if trigged batch_to_send is different from current batch_q_num, 
     then new batch needs to be sent out. 
     0. flush the current batchQ, reset the msgs
     1. dequeue the fresh K pkts from the msgQ
     2. reset the flags
  */
  if(d_batch_q_num != d_batch_to_send)
  {
	printf("start batch: %d, d_batch_q_num: %d\n", d_batch_to_send, d_batch_q_num); fflush(stdout);
	flushBatchQ();
	for(unsigned int b = 0; b < d_batch_size; b++)
	{
	    d_msg[b].reset();
	    d_msg[b] = d_msgq->delete_head();		// dequeue the pkts from the queue //
	    assert(d_msg[b]->length() > 0);	
	    d_packetlen = d_msg[b]->length();		// assume: all pkts in batch are of same length //
	    d_num_ofdm_symbols = ceil(((float) ((d_packetlen) * 8))/(d_data_carriers.size() * d_data_nbits)); // include 'x55'
	    printf("d_num_ofdm_symbols: %d, d_num_hdr_symbols: %d\n", d_num_ofdm_symbols, d_num_hdr_symbols); fflush(stdout);
	}

	initializeBatchParams();
	d_pending_flag = 1;				// start of packet flag //
	d_batch_q_num = d_batch_to_send;
  }
  else if(d_modulated)
  {
	printf("continue batch: %d, pkts sent: %d\n", d_batch_to_send, d_packets_sent_for_batch); fflush(stdout);
	// no change in the batch number, but this pkt has been sent out completely (retx) //
	initializeBatchParams();
	d_pending_flag = 1;				// start of packet flag //
  }

  d_modulated = false;

  // if start of a packet: then add a header and whiten the entire packet //
  if(d_pending_flag == 1)
  {
     printf("d_packets_sent_for_batch: %d, d_batch_to_send: %d, d_batch_q_num: %d\n", d_packets_sent_for_batch, d_batch_to_send, d_batch_q_num); fflush(stdout);

     fflush(stdout);
     if(((HEADERBYTELEN*8) % d_data_carriers.size()) != 0) {	// assuming bpsk //
	  printf("HEADERBYTELEN: %d, size: %d\n", HEADERBYTELEN, d_data_carriers.size()); fflush(stdout);
	  assert(false);
     }

     makeHeader();
     initHeaderParams();

     d_preambles_sent = d_ofdm_symbol_index = d_null_symbol_cnt = d_hdr_ofdm_index = d_mimo_wait_cnt = 0; 
     make_time_tag1();
     d_pkt_num++;
  }

  bool tx_pilot = false;

  // the final result will be in 'out' //
  gr_complex *out = (gr_complex *)output_items[0];
  memset(out, 0, sizeof(gr_complex) * d_fft_length);

  /* for burst tagger trigger */
  unsigned short *burst_trigger = NULL;
  if(output_items.size() >= 2) {
     burst_trigger = (unsigned short*) output_items[1];
     memset(burst_trigger, 0, sizeof(short));
  }
  burst_trigger[0] = 1;
  /* end */

  /* optional - for out timing signal required only for debugging */
  unsigned char *out_signal = NULL;
  if (output_items.size() >= 3){
    out_signal = (unsigned char*) output_items[3];
    memset(out_signal, 0, sizeof(char) * d_fft_length);
  }
  /* end */

  if(d_preambles_sent < d_preamble.size()) {
     generatePreamble(out);
     d_preambles_sent++;
     tx_pilot = false;
     if(d_preambles_sent == 1 && output_items.size() >= 3) out_signal[0]=1;
  }
  else if(d_hdr_byte_offset < HEADERBYTELEN) {
      generateOFDMSymbolHeader(out); 					// send the header symbols out first //
      tx_pilot = true;
  }
  else if(d_mimo == 1 && (d_mimo_wait_cnt < (d_preamble.size()+d_num_hdr_symbols))) {
      d_mimo_wait_cnt++;
  }
  else if(!d_send_null) {
       // offline analysis //
      if(!d_init_log) {
           assert(open_log());
           d_init_log = true;
      }

      assert(d_pending_flag == 0);
	// send the data symbols now //
      std::vector<gr_complex*> symbols_vec;
      for(unsigned int k = 0; k < d_batch_size; k++)
      {
	   gr_complex *t_out = (gr_complex *) malloc(sizeof(gr_complex) * d_fft_length);
	   memset(t_out, 0, sizeof(gr_complex) * d_fft_length);

	   generateOFDMSymbolData(t_out, k);
	   symbols_vec.push_back(t_out);

           int in = (k*d_num_ofdm_symbols + d_ofdm_symbol_index) * d_fft_length;
           memcpy(&d_native_symbols[in], t_out, sizeof(gr_complex) * d_fft_length);
      }

      assert(symbols_vec.size() == d_batch_size);

      /* encoding process starts */
      for(unsigned int k = 0; k < d_batch_size; k++)
      {
	   gr_complex *symbols = symbols_vec.at(k);
	   if(d_encode_flag)
		encodeSignal(symbols, k); 

	   if(k == 0)
	      memcpy(out, symbols, sizeof(gr_complex) * d_fft_length);
	   else
	      combineSignal(out, symbols);
	   free(symbols);
      }
      /* encoding process ends */

      logGeneratedTxSymbols(out, d_fp_log_BN);
      normalizeSignal(out, d_batch_size);						// normalize the outgoing signal //
      logGeneratedTxSymbols(out, d_fp_log_AN); 

 	// offline, timekeeping, etc //
      assert(d_ofdm_symbol_index < d_num_ofdm_symbols);
      d_ofdm_symbol_index++;

#if 0
      // uncomment if need to disable sending a NULL symbol at the end //
      if(d_modulated)
      {
	  d_pending_flag = 2;                                             // marks the last OFDM data symbol (for burst tagger trigger) //
	  d_packets_sent_for_batch += 1;
	  //log();             // for offline analysis //
	  assert(d_ofdm_symbol_index == d_num_ofdm_symbols);
	  d_ofdm_symbol_index = 0;

	  struct timeval now;
	  gettimeofday(&now, 0);
	  d_time_pkt_sent = (now.tv_sec * 1e6) + now.tv_usec;
	  printf("d_time_pkt_sent: %llu\n", d_time_pkt_sent); fflush(stdout);
      }
#endif
	// etc end //
	tx_pilot = (d_mimo == 1)?false:true;
  }

#if 1
  // comment if need to disable sending a NULL symbol at the end //
  else if (d_send_null) {
      d_pending_flag = 2;
      d_packets_sent_for_batch += 1;
      assert(d_ofdm_symbol_index == d_num_ofdm_symbols);
      d_send_null = false;
      d_modulated = true;
	/*
      if(!d_log_open_native) {
         logNativeTxSymbols();
         d_log_open_native = true;
      } */
  }
#endif

#ifdef USE_PILOT
  //printf("ofdm_symbol_index: %d, tx_pilot: %d\n", d_ofdm_symbol_index, tx_pilot); fflush(stdout);
  if(tx_pilot) {
      double cur_pilot = 1.0;
      for(int i = 0; i < d_pilot_carriers.size(); i++) {
         out[d_pilot_carriers[i]] = gr_complex(cur_pilot, 0.0);
         cur_pilot = -cur_pilot;
      }
  }
  else if(d_ofdm_symbol_index > 0) {
      for(int i = 0; i < d_pilot_carriers.size(); i++) {
	out[d_pilot_carriers[i]] = gr_complex(0.0, 0.0);
      }
  }
#endif

  if(d_h_coding) {
     removeTimingOffset(out, d_header.timing_offset);
  }
 
  char *out_flag = 0;
  if(output_items.size() >= 3)
    out_flag = (char *) output_items[2];

  if (out_flag)
    out_flag[0] = d_pending_flag;

  if(d_pending_flag == 2) {
    burst_trigger[0] = 0;
  }

  d_pending_flag = 0;  
  return 1;  // produced symbol
}

void
digital_ofdm_mapper_bcv::logNativeTxSymbols()
{
   /* one time logging */
   FILE *fp = NULL;
   char *filename = "native_symbols.dat";
   int fd;
   if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
       perror(filename);
       assert(false);
   }
   else {
       if((fp = fdopen (fd, true ? "wb" : "w")) == NULL) {
           close(fd);
           assert(false);
       }
   }

#if 0
  int count = ftell(fp);
  count = fwrite_unlocked(d_native_symbols, sizeof(gr_complex), d_batch_size * d_num_ofdm_symbols * d_fft_length, fp);
  count = ftell(fp);
#else
  int carriers = d_data_carriers.size();

  gr_complex *log_symbols = (gr_complex*) malloc(sizeof(gr_complex) * carriers);
  memset(log_symbols, 0, sizeof(gr_complex) * carriers);

  for(unsigned int k = 0; k < d_batch_size; k++) {
      for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
          gr_complex *out = &d_native_symbols[(k*d_num_ofdm_symbols+o)*d_fft_length];
          for(int i = 0; i < carriers; i++)
                memcpy(log_symbols+i, out+d_data_carriers[i], sizeof(gr_complex));

          int count = fwrite_unlocked(log_symbols, sizeof(gr_complex), carriers, fp);
          count = ftell(fp);
      }
  }
  free(log_symbols);
#endif

  close(fd);
  fp = NULL;
}

void
digital_ofdm_mapper_bcv::assign_subcarriers() {
  int dc_tones = 8;
  int num_pilots = 8;
  int pilot_gap = 11;

  int half1_end = (d_occupied_carriers-dc_tones)/2;     //40
  int half2_start = half1_end+dc_tones;                 //48

  int off = (d_fft_length-d_occupied_carriers)/2;       //4

  // first half
  for(int i = 0; i < half1_end; i++) {
     if(i%pilot_gap == 0)
        d_pilot_carriers.push_back(i+off);
     else
        d_data_carriers.push_back(i+off);
  }

  // second half
  for(int i = half2_start, j = 0; i < d_occupied_carriers; i++, j++) {
     if(j%pilot_gap == 0)
        d_pilot_carriers.push_back(i+off);
     else
        d_data_carriers.push_back(i+off);
  }

  /* debug carriers */
  printf("pilot carriers: \n");
  for(int i = 0; i < d_pilot_carriers.size(); i++) {
     printf("%d ", d_pilot_carriers[i]); fflush(stdout);
  }
  printf("\n");

  printf("data carriers: \n");
  for(int i = 0; i < d_data_carriers.size(); i++) {
     printf("%d ", d_data_carriers[i]); fflush(stdout);
  }
  printf("\n");
}

void
digital_ofdm_mapper_bcv::fill_all_carriers_map() {
  d_all_carriers.resize(d_occupied_carriers);

  unsigned int left_half_fft_guard = (d_fft_length - d_occupied_carriers)/2;

  unsigned int p = 0, d = 0, dc = 0;

  for(unsigned int i = 0; i < d_occupied_carriers; i++) {
      int carrier_index = left_half_fft_guard + i;

      if(d_data_carriers[d] == carrier_index) {
         d_all_carriers[i] = 0;
         d++;
      }
      else if(d_pilot_carriers[p] == carrier_index) {
         d_all_carriers[i] = 1;
         p++;
      }
      else {
         d_all_carriers[i] = 2;
         dc++;
      }
  }

#ifdef DEBUG
  printf("fill_all_carriers: --- -\n"); fflush(stdout);
  for(int i = 0; i < d_all_carriers.size(); i++) {
	printf("i: %d ----- val: %d\n", i, d_all_carriers[i]); fflush(stdout);
  }
  printf(" ---------------------------------------------------- \n"); fflush(stdout);
#endif

  assert(d == d_data_carriers.size());
  assert(p == d_pilot_carriers.size());
  assert(dc == d_occupied_carriers - d_data_carriers.size() - d_pilot_carriers.size());
}

void 
digital_ofdm_mapper_bcv::logGeneratedTxSymbols(gr_complex *out, FILE *fp1) 
{
  printf("digital_ofdm_mapper_bcv::logGeneratedTxSymbols\n"); fflush(stdout);
#if 0
  if(fp1 == NULL) {
      printf("opening file\n"); fflush(stdout);
      int fd;
      if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename);
            assert(false);
      }
      else {
         if((fp1 = fdopen (fd, true ? "wb" : "w")) == NULL) {
              fprintf(stderr, "log file cannot be opened\n");
              close(fd);
              assert(false);
         }
      }
  }
#endif

  gr_complex *log_symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_data_carriers.size());
  memset(log_symbols, 0, sizeof(gr_complex) * d_data_carriers.size());

  int index = 0;
  for(int i = 0; i < d_all_carriers.size(); i++) {
     if(d_all_carriers[i] == 0) {
	memcpy(log_symbols+index, out+d_data_carriers[index], sizeof(gr_complex));
        index++;
     }
  }
  assert(index == d_data_carriers.size());

  int count = fwrite_unlocked(log_symbols, sizeof(gr_complex), d_data_carriers.size(), fp1);
  printf("count: %d written to tx_symbols.dat, total: %d \n", count, ftell(fp1)); fflush(stdout);
  
  free(log_symbols);
}

/***********************************************************************
 apurv++ starts here, implementing MORE kind of an extension

#### ANALOG domain #####
send(batch_num, K)
  var batch_q[K]        #queue containing modulated symbols of K pkts
  var batch_q_num       #batch num of the batch_queue

  if(batch_q_num != batch_num):
         flush(batch_q)         #releases the batch queue contents
         dequeue(msg_q, K)      #dequeues K new packets from digital queue
         foreach msg dequeued:
                signal = modulate(msg)  #digital to analog modulation
                insert(signal, batch_q) #insert analog signal (of symbols)
                batch_q_num = batch_num #update batch_q number

  encoded_signal = empty        #encoding starts
  foreach signal in batch_q:
         choose random coefficient C
         foreach symbol in signal:
                symbol' = symbol * C

         encoded_signal = encoded_signal + signal
*************************************************************************/

void
digital_ofdm_mapper_bcv::initializeBatchParams()
{
  memset(d_msg_offset, 0, sizeof(unsigned) * MAX_BATCH_SIZE); 
  memset(d_bit_offset, 0, sizeof(int) * MAX_BATCH_SIZE);
  memset(d_nresid, 0, sizeof(int) * MAX_BATCH_SIZE);
  memset(d_resid, 0, sizeof(char) * MAX_BATCH_SIZE);
}

void
digital_ofdm_mapper_bcv::flushBatchQ()
{

}


// generate an OFDM symbol *each* for the packets involved //
void
digital_ofdm_mapper_bcv::generateOFDMSymbolData(gr_complex* out, int k)
{
  // Build a single symbol: P.S: this depends on the all msgs being equal in length ! 
  unsigned int i = 0;
  unsigned char bits = 0;
  while((d_msg_offset[k] < d_msg[k]->length()) && (i < d_data_carriers.size())) {

    // need new data to process
    if(d_bit_offset[k] == 0) {
      d_msgbytes[k] = d_msg[k]->msg()[d_msg_offset[k]];
    }

    if(d_nresid[k] > 0) {
      // take the residual bits, fill out nbits with info from the new byte, and put them in the symbol
      d_resid[k] |= (((1 << d_nresid[k])-1) & d_msgbytes[k]) << (d_data_nbits - d_nresid[k]);
      bits = d_resid[k];

      out[d_data_carriers[i]] = d_data_constellation[bits];
      i++;

      d_bit_offset[k] += d_nresid[k];
      d_nresid[k] = 0;
      d_resid[k] = 0;
    }
    else {
      if((8 - d_bit_offset[k]) >= d_data_nbits) {  // test to make sure we can fit nbits
        // take the nbits number of bits at a time from the byte to add to the symbol
        bits = ((1 << d_data_nbits)-1) & (d_msgbytes[k] >> d_bit_offset[k]);
        d_bit_offset[k] += d_data_nbits;

        out[d_data_carriers[i]] = d_data_constellation[bits];
        i++;
      }
      else {  // if we can't fit nbits, store them for the next 
        // saves d_nresid bits of this message where d_nresid < d_data_nbits
        unsigned int extra = 8-d_bit_offset[k];
        d_resid[k] = ((1 << extra)-1) & (d_msgbytes[k] >> d_bit_offset[k]);
        d_bit_offset[k] += extra;
        d_nresid[k] = d_data_nbits - extra;
      }

    }

    if(d_bit_offset[k] == 8) {
      d_bit_offset[k] = 0;
      d_msg_offset[k]++;
    }
  }

  // Ran out of data to put in symbol
  if (d_msg_offset[k] == d_msg[k]->length()) {
    if(d_nresid[k] > 0) {
      d_resid[k] |= 0x00;
      bits = d_resid[k];
      d_nresid[k] = 0;
      d_resid[k] = 0;
    }

#if 0
    while(i < d_data_carriers.size()) {   // finish filling out the symbol
      out[d_data_carriers[i]] = d_constellation[randsym()];
      i++;
    }
#endif

    //printf("complete pkt modulated\n"); fflush(stdout);
    //d_modulated = true;		// complete msg has been demodulated //
    d_send_null = true;			// send a null symbol at the end //

    assert(d_bit_offset[k] == 0);

  }
}

void
digital_ofdm_mapper_bcv::initHeaderParams()
{
  d_hdr_resid = 0;
  d_hdr_nresid = 0;
  d_hdr_byte_offset = 0;
  d_hdr_byte = 0;
  d_hdr_bit_offset = 0;
}

/* this is a little dumb right now to have an almost duplicate 
   implementation. Keep it till I figure out a better way 
    generate an OFDM symbol *each* for the packets involved 
*/
void
digital_ofdm_mapper_bcv::generateOFDMSymbolHeader(gr_complex* out)
{
  unsigned int i = 0;
  unsigned char bits = 0;
  while((d_hdr_byte_offset < HEADERBYTELEN) && (i < d_data_carriers.size())) {

    // need new data to process
    if(d_hdr_bit_offset == 0) {
      d_hdr_byte = d_header_bytes[d_hdr_byte_offset];
    }

    if(d_hdr_nresid > 0) {
      // take the residual bits, fill out nbits with info from the new byte, and put them in the symbol
      d_hdr_resid |= (((1 << d_hdr_nresid)-1) & d_hdr_byte) << (d_hdr_nbits - d_hdr_nresid);
      bits = d_hdr_resid;

      out[d_data_carriers[i]] = d_hdr_constellation[bits];
      i++;

      d_hdr_bit_offset += d_hdr_nresid;
      d_hdr_nresid = 0;
      d_hdr_resid = 0;
    }
    else {
      if((8 - d_hdr_bit_offset) >= d_hdr_nbits) {  // test to make sure we can fit nbits
        // take the nbits number of bits at a time from the byte to add to the symbol
        bits = ((1 << d_hdr_nbits)-1) & (d_hdr_byte >> d_hdr_bit_offset);
        d_hdr_bit_offset += d_hdr_nbits;

        out[d_data_carriers[i]] = d_hdr_constellation[bits];
	//printf("fill sc: %d\n", d_data_carriers[i]); fflush(stdout);
        i++;
      }
      else {  // if we can't fit nbits, store them for the next 
        // saves d_hdr_nresid bits of this message where d_hdr_nresid < d_nbits
        unsigned int extra = 8-d_hdr_bit_offset;
        d_hdr_resid = ((1 << extra)-1) & (d_hdr_byte >> d_hdr_bit_offset);
        d_hdr_bit_offset += extra;
        d_hdr_nresid = d_hdr_nbits - extra;
      }

    }

    if(d_hdr_bit_offset == 8) {
      d_hdr_bit_offset = 0;
      d_hdr_byte_offset++;
    }
  }

  // Ran out of data to put in symbol
  if (d_hdr_byte_offset == HEADERBYTELEN) {
    printf("hdr done!\n"); fflush(stdout);
    if(d_hdr_nresid > 0) {
      d_hdr_resid |= 0x00;
      bits = d_hdr_resid;
      d_hdr_nresid = 0;
      d_hdr_resid = 0;
    }

    while(i < d_data_carriers.size()) {   // finish filling out the symbol
      out[d_data_carriers[i]] = d_hdr_constellation[randsym()];

      i++;
    }

    assert(d_hdr_bit_offset == 0);
  }
}

/* phase (complex) -> degrees (float) */
inline float
digital_ofdm_mapper_bcv::ToPhase_f(gr_complex phase_c) {
  float phase = arg(phase_c) * 180/M_PI;
  if(phase < 0) phase += 360.0;

  return phase;
}

/* radians (float) -> phase (complex) */
inline gr_complex
digital_ofdm_mapper_bcv::ToPhase_c(float phase_rad)
{
  return gr_expj(phase_rad);
}

/* COEFF -> degrees (float) */
inline float
digital_ofdm_mapper_bcv::ToPhase_f(COEFF coeff) {
  float phase = ((float) coeff.phase)/SCALE_FACTOR_PHASE;
  return phase;
}

/* COEFF -> phase (complex) */
inline gr_complex
digital_ofdm_mapper_bcv::ToPhase_c(COEFF coeff) {
  float phase = ((float) coeff.phase)/SCALE_FACTOR_PHASE;
  float amp = ((float) coeff.amplitude)/SCALE_FACTOR_AMP;

  float angle_rad = phase * M_PI/180;
  return amp * gr_expj(angle_rad);
}

float
digital_ofdm_mapper_bcv::getNormalizationFactor() {
  float avg_amp = 0.0;
  float avg_mag = 0.0;
  for(int i = 0; i < d_data_carriers.size(); i++) {
     gr_complex sym(0.0, 0.0);
     for(int k = 0; k < d_batch_size; k++) {
        gr_complex cv = ToPhase_c(d_header.coeffs[k]);
        sym += (cv*d_data_constellation[randsym()]);
     }
     avg_amp += abs(sym);
     avg_mag += norm(sym);
  }

  avg_amp = avg_amp/((float) d_data_carriers.size());
  avg_mag /= ((float) d_data_carriers.size());

  printf("getNormalizationFactor, avg_amp: %f\n", avg_amp); fflush(stdout);
  //return avg_amp/0.85;
  return sqrt(avg_mag);
}

#if 1 
void 
digital_ofdm_mapper_bcv::generateCodeVector()
{
  // for each subcarrier, record 'd_batch_size' coeffs //
  int num_carriers = d_data_carriers.size()/COMPRESSION_FACTOR;

  printf("generateCodeVector -- \n"); fflush(stdout);

  if(d_h_coding) {
     gr_complex coeffs[2];
     chooseCV_H(coeffs);
     for(unsigned int k = 0; k < d_batch_size; k++) {
         float cv = arg(coeffs[k]) * 180/M_PI;
	 if(cv < 0) cv += 360.0;

	 d_header.coeffs[k].phase = cv * SCALE_FACTOR_PHASE;
	 d_header.coeffs[k].amplitude = 1.0 * SCALE_FACTOR_AMP;
	 float rad = cv * M_PI/180;
	 gr_complex t_coeff = ToPhase_c(rad);
	 printf("cv: %f degrees <-> %f radians <-> (%f, %f) \n", cv, rad, t_coeff.real(), t_coeff.imag()); fflush(stdout);
     }
  }
  else {
     float cv = 0.0;
     for(unsigned int k = 0; k < d_batch_size; k++) {
         cv = rand() % 360 + 1;				// degree

	 float LO = 0.5; float HI = 1.5;
	 float amp = 1.0; //LO + (float)rand()/((float)RAND_MAX/(HI-LO));

	 // store the scaled versions in header (space constraints) //
	 d_header.coeffs[k].phase = cv * SCALE_FACTOR_PHASE;
	 d_header.coeffs[k].amplitude = amp * SCALE_FACTOR_AMP;
#ifndef DEBUG
         float rad = cv * M_PI/180;
         gr_complex t_coeff = ToPhase_c(rad);
         printf("cv: %f degrees <-> %f radians <-> (%f, %f) \n", cv, rad, t_coeff.real(), t_coeff.imag()); fflush(stdout);
#endif	// DEBUG
     }
  } // if(d_h_coding)


#ifdef LSQ_COMPRESSION
  // source just blindly copies the coefficients, no point doing LSQ at the source //
  for(unsigned int s = 1; s < d_degree; s++) {
      for(unsigned int k = 0; k < d_batch_size; k++) {
          int index = s * d_batch_size + k;
          d_header.coeffs[index].phase = d_header.coeffs[k].phase;
          d_header.coeffs[index].amplitude = d_header.coeffs[k].amplitude;
      }
  }
#else
  // for now copy the <d_batch_size> coeffs for all the coeff entries for the header //
  for(unsigned int s = 1; s < num_carriers; s++) {
      for(unsigned int k = 0; k < d_batch_size; k++) {
          int index = s * d_batch_size + k;
          d_header.coeffs[index].phase = d_header.coeffs[k].phase;
          d_header.coeffs[index].amplitude = d_header.coeffs[k].amplitude;
      }
  }
#endif
  printf("\n");
}
#endif

void
digital_ofdm_mapper_bcv::encodeSignal(gr_complex *symbols, unsigned int batch_num)
{
  COEFF coeff = d_header.coeffs[batch_num];
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
#ifdef SCALE
      symbols[d_data_carriers[i]] *= (ToPhase_c(coeff) * gr_complex(SCALE));
#else
      gr_complex sym = symbols[d_data_carriers[i]];
      //gr_complex 
      if(d_ofdm_symbol_index == 0 && 0)
            printf("(%.2f, %.2f)  * (%.2f, %.2f) = ", sym.real(), sym.imag(), ToPhase_c(coeff).real(), ToPhase_c(coeff).imag()); 
      symbols[d_data_carriers[i]] *= ToPhase_c(coeff);
#endif
      if(d_ofdm_symbol_index == 0 && 0)
         printf(" (%.8f, %.8f)\n", symbols[d_data_carriers[i]].real(), symbols[d_data_carriers[i]].imag()); fflush(stdout);
  }
}

inline void
digital_ofdm_mapper_bcv::normalizeSignal(gr_complex* out, int k)
{
#if 0 
  // just output the avg_amp in the first OFDM symbol //
  if(d_ofdm_symbol_index == 0) {
     float avg_amp = 0.0;
     for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
        avg_amp += abs(out[d_data_carriers[i]]);
     }
     avg_amp /= ((float) d_data_carriers.size());
     printf(" -- avg_amp (before normalization): %f\n", avg_amp); fflush(stdout);
  }


  // the straightforward way //
  float factor = sqrt(k) * sqrt(d_header.nsenders);
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
     out[d_data_carriers[i]] /= gr_complex(factor);
  }

  // just output the avg_amp in the first OFDM symbol //
  if(d_ofdm_symbol_index == 0) {
     float avg_amp = 0.0;
     for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
        avg_amp += abs(out[d_data_carriers[i]]);
     }
     avg_amp /= ((float) d_data_carriers.size());
     printf(" -- avg_amp (after normalization): %f\n", avg_amp); fflush(stdout);
  }
#else
  // just output the avg_amp in the first OFDM symbol //
  if(d_ofdm_symbol_index == 0) {
     float avg_amp = 0.0;
     for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
        avg_amp += abs(out[d_data_carriers[i]]);
     }
     avg_amp /= ((float) d_data_carriers.size());
     printf(" -- avg_amp (before normalization): %f\n", avg_amp); fflush(stdout);
  }

  float factor = d_header.factor;
  printf("normalizeSignal, factor: %f\n", factor); fflush(stdout);
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
     out[d_data_carriers[i]] /= gr_complex(factor);
  }

  if(d_ofdm_symbol_index == 0 || 1) {
     float avg_amp = 0.0;
     for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
        avg_amp += abs(out[d_data_carriers[i]]);
     }
     avg_amp /= ((float) d_data_carriers.size());
     printf(" -- avg_amp (after normalization): %f\n", avg_amp); fflush(stdout);
  }

#endif
}

inline void
digital_ofdm_mapper_bcv::removeTimingOffset(gr_complex *out, float t_o) {
  unsigned int index = (d_fft_length - d_occupied_carriers)/2;			// start filling from here

  //printf("removeTimingOffset, pkt_num: %d, offset: %f\n", d_pkt_num-1, t_o);
  if(t_o != 0) {
     for(unsigned int i = 0; i < d_occupied_carriers; i++) {
        out[index++] *= gr_expj(t_o*i);
     }
  }
}

void
digital_ofdm_mapper_bcv::combineSignal(gr_complex *out, gr_complex* symbols)
{
  printf("combineSignal ------------------------------------ \n"); fflush(stdout);
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
      if(d_ofdm_symbol_index == 0 && 0) {
         printf("(%.8f, %.8f) + (%.8f, %.8f) = ", out[d_data_carriers[i]].real(), out[d_data_carriers[i]].imag(), symbols[d_data_carriers[i]].real(), symbols[d_data_carriers[i]].imag()); fflush(stdout);
      }
     out[d_data_carriers[i]] += symbols[d_data_carriers[i]];
     if(d_ofdm_symbol_index == 0 && 0)  printf(" (%.8f, %.8f)\n", out[d_data_carriers[i]].real(), out[d_data_carriers[i]].imag()); fflush(stdout);
  }
}

/* populate header_data_bytes and also d_header 
   add all the header details:
	src_id
	rx_id
	packet_len
	batch_number
	n_senders
	pkt_type
	hdr crc
*/
void
digital_ofdm_mapper_bcv::makeHeader()
{
   memset(&d_header, 0, sizeof(d_header));
   d_header.dst_id = d_dst_id;
   d_header.flow_id = 0;
   d_header.inno_pkts = d_batch_size;

   d_header.lead_sender = (d_mimo == 2)?0:1;
   d_header.src_id = d_id;
   d_header.prev_hop_id = d_id;

   d_header.packetlen = d_packetlen;		// -1 for '55' appended (ref ofdm_packet_utils)
   d_header.batch_number = d_batch_to_send;
   d_header.nsenders = (d_mimo == 0)?1:2;
   d_header.pkt_type = DATA_TYPE;

   //d_header.factor = sqrt(d_header.nsenders);
  
   d_header.pkt_num = d_pkt_num;
   d_header.link_id = 0;
 
   generateCodeVector();	// also fills up d_header.coeffs

#if 1
   if(d_h_coding)
     d_header.timing_offset = getTimingOffset();
#endif

   printf("makeHeader for batch: %d, pkt: %d, len: %d, timing_off: %f\n", d_batch_to_send, d_pkt_num, d_packetlen, d_header.timing_offset); fflush(stdout);
   d_header.factor = getNormalizationFactor() * sqrt(d_header.nsenders);

   d_last_pkt_time = d_out_pkt_time;
   printf("\t Using code vectors: ");
   for(unsigned int k = 0; k < d_batch_size; k++) {
      COEFF coeff = d_header.coeffs[k];
      float amp = ((float) coeff.amplitude)/SCALE_FACTOR_AMP;
      float phase = ((float) coeff.phase)/SCALE_FACTOR_PHASE;
      printf("(A: %f, P: %f) ", amp, phase);
   }

   for(int i = 0; i < PADDING_SIZE; i++)
	d_header.pad[i] = 0; 
 
   unsigned char header_data_bytes[HEADERDATALEN];
   memcpy(header_data_bytes, &d_header, HEADERDATALEN);				// copy e'thing but the crc

   unsigned int calc_crc = digital_crc32(header_data_bytes, HEADERDATALEN);
   d_header.hdr_crc = calc_crc;

   memcpy(d_header_bytes, header_data_bytes, HEADERDATALEN);				// copy header data
   memcpy(d_header_bytes+HEADERDATALEN, &calc_crc, sizeof(int));		// copy header crc

   memcpy(d_header_bytes+HEADERDATALEN+sizeof(int), d_header.pad, PADDING_SIZE);
   printf("len: %d, crc: %u\n", d_packetlen, calc_crc);

   //debugHeader();  
   whiten();
   //debugHeader();
}

inline float
digital_ofdm_mapper_bcv::getTimingOffset()
{
   NodeIds rx_ids;
   get_nextHop_rx(rx_ids);

   if(rx_ids.size() == 1) {
      HInfo *hInfo = getHInfo(d_id, rx_ids[0]);
      return hInfo->timing_slope;
   }
   else {
      float to = 0.0;
      for(int i = 0; i < rx_ids.size(); i++) {
          HInfo *hInfo = getHInfo(d_id, rx_ids[i]);
          to += hInfo->timing_slope;
      }
      return to/((float) rx_ids.size());
   }
   return 0;
}

void
digital_ofdm_mapper_bcv::debugHeader()
{
   whiten();	// dewhitening effect !
   memset(&d_header, 0, HEADERBYTELEN);
   assert(HEADERBYTELEN == (HEADERDATALEN + sizeof(int) + PADDING_SIZE));
   memcpy(&d_header, d_header_bytes, HEADERBYTELEN);

   printf("debug crc: %u\n", d_header.hdr_crc); 
  
   for(unsigned int k = 0; k < d_batch_size; k++) {
      COEFF coeff = d_header.coeffs[k];
      float phase = ((float) coeff.phase)/SCALE_FACTOR_PHASE;
      float amp = ((float) coeff.amplitude)/SCALE_FACTOR_AMP;
      printf("(A: %f, P: %f) ", amp, phase);
   }

   printf("\n"); 
   whiten();
}

void
digital_ofdm_mapper_bcv::whiten()
{
   for(int i = 0; i < HEADERBYTELEN; i++)
   {
	d_header_bytes[i] = d_header_bytes[i] ^ random_mask_tuple1[i];
   }
}

void
digital_ofdm_mapper_bcv::log()
{
  assert(d_fp_coeff != NULL);
  assert(d_fp_native_symbols != NULL);
  assert(d_num_ofdm_symbols > 0);

  // log batch coefficients for each outgoing coded pkt //
  int count = ftell(d_fp_coeff);
  count = fwrite_unlocked(d_header.coeffs, sizeof(COEFF), MAX_BATCH_SIZE, d_fp_coeff);

  // log tx symbols of each native packet //
  count = ftell(d_fp_native_symbols);
  for(unsigned k = 0; k < d_batch_size; k++) {
     gr_complex *tx_symbols = d_log_symbols[k];
     count = fwrite_unlocked(tx_symbols, sizeof(gr_complex), d_occupied_carriers * d_num_ofdm_symbols, d_fp_native_symbols);
  }
}


// ensures that all the log files (for offline analysis) have been opened, 
//   P.S: If one is open, all are open :) 
bool
digital_ofdm_mapper_bcv::open_log()
{
  printf("open_log called\n"); fflush(stdout);

  /*
  // rx coefficients //
  char *filename = "tx_coeff.dat";
  int fd;
  if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     return false;
  }
  else {
      if((d_fp_coeff = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "coeff file cannot be opened\n");
            close(fd);
            return false;
      }
  }

  // tx symbols for each native packet //
  filename = "native_tx_symbols.dat";
  if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     return false;
  }
  else {
      if((d_fp_native_symbols = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "tx native file cannot be opened\n");
            close(fd);
            return false;
      }
  }
  */
   // initialize the symbol vector for logging //
   for(unsigned int i = 0; i < d_batch_size; i++) {
     gr_complex *symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_num_ofdm_symbols * d_occupied_carriers);
     d_log_symbols.push_back(symbols);
  }

  return true;
}

/* ack processing (at the flow-source) */
void
digital_ofdm_mapper_bcv::processACK(gr_message_sptr ackMsg) {
  printf("mapper: processACK\n"); 

  assert(ackMsg->type() == ACK_TYPE);
  MULTIHOP_ACK_HDR_TYPE ack_header;
  memcpy(&ack_header, ackMsg->msg(), ACK_HEADERBYTELEN);
  
  assert(ack_header.dst_id == d_id);
  if(d_batch_to_send == ack_header.batch_number) {
     d_batch_to_send += 1;
     d_packets_sent_for_batch = 0;
  }

  ackMsg.reset();
}

/* ack over ethernet */
int
digital_ofdm_mapper_bcv::openACKSocket() {
  int sock_port = 9000;
  int sockfd;
  struct sockaddr_in dest;

  /* create socket */
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  fcntl(sockfd, F_SETFL, O_NONBLOCK);
 
  /* ensure the socket address can be reused, even after CTRL-C the application */ 
  int optval = 1;
  int ret = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

  if(ret == -1) {
	printf("@ digital_ofdm_mapper_bcv::setsockopt ERROR\n"); fflush(stdout);
  }

  /* initialize structure dest */
  bzero(&dest, sizeof(dest));
  dest.sin_family = AF_INET;

  /* assign a port number to socket */
  dest.sin_port = htons(sock_port);
  dest.sin_addr.s_addr = INADDR_ANY;

  bind(sockfd, (struct sockaddr*)&dest, sizeof(dest));

  /* make it listen to socket with max 20 connections */
  listen(sockfd, 20);
  if(sockfd != -1) {
    printf("ACK Socket OPEN success\n"); fflush(stdout);
  }
  else {
    printf("ACK Socket OPEN error\n"); fflush(stdout);
  }

  /* allocate d_ack_buf */
  d_ack_buf = (char*) malloc(sizeof(char) * sizeof(ACK_HEADERDATALEN));
  return sockfd;
}

int
digital_ofdm_mapper_bcv::isACKSocketOpen() {
  if(!d_sock_opened) 
    return 0;

  return 1;
}

/* in the forwarding mode */
void
digital_ofdm_mapper_bcv::make_time_tag(gr_message_sptr msg) {
  assert(!d_time_tag);
  long _time_secs = msg->preamble_sec();
  double _time_fracs = msg->preamble_frac_sec();

  assert(msg->timestamp_valid());

  const pmt::pmt_t key = pmt::pmt_string_to_symbol("tx_time");
  const pmt::pmt_t value = pmt::pmt_make_tuple(
		  pmt::pmt_from_uint64(_time_secs),
		  pmt::pmt_from_double(_time_fracs)
		  );
  const pmt::pmt_t srcid = pmt::pmt_string_to_symbol(this->name());
  add_item_tag(1/*chan0*/, nitems_written(1), key, value, srcid);
  printf("(MAPPER) make_time_tag, at offset: %llu\n", nitems_written(1)); fflush(stdout);
}

/* trigger on the ethernet */
inline void // client /
digital_ofdm_mapper_bcv::create_trigger_sock() {
  int sockfd;
  struct sockaddr_in dest;
  printf("ofdm_mapper:: create_trigger_sock\n"); fflush(stdout);

  /* create socket */
  d_trigger_sock = socket(PF_INET, SOCK_STREAM, 0);
  if(d_trigger_sock != -1) {
    printf("@ TRIGGER socket created at the destination SUCCESS\n"); fflush(stdout);
  } else {
    assert(false);
  }

  /* initialize value in dest */
  bzero(&dest, sizeof(struct sockaddr_in));
  dest.sin_family = PF_INET;
  dest.sin_port = htons(d_trigger_src_sock_port);
  dest.sin_addr.s_addr = inet_addr(d_trigger_src_ip_addr);
  /* Connecting to server */
  int ret = connect(d_trigger_sock, (struct sockaddr*)&dest, sizeof(struct sockaddr_in));
  assert(ret == 0);

  d_trigger_buf = (char*) malloc(sizeof(TRIGGER_MSG_TYPE));

  test_socket();
}

inline void
digital_ofdm_mapper_bcv::test_socket() {
  printf("test_socket on ethernet\n"); fflush(stdout);
  int count = 5;

  while(count != 0) {
		  char msg_buf[3]; 
		  int nbytes = recv(d_trigger_sock, msg_buf, sizeof(msg_buf), 0);
		  if(nbytes > 0) {
				  printf("(MAPPER): test_socket received request %d bytes\n", nbytes); fflush(stdout);

				  char reply_buf[] = "R";
				  if (send(d_trigger_sock, reply_buf, sizeof(reply_buf), 0) < 0) {
						  printf("(MAPPER): test_socket reply failed\n"); fflush(stdout);
				  }
		  }
  }
  printf("test_socket done\n"); fflush(stdout);
}

/* in the work_source() mode to just test if trigger on ethernet works */
inline void
digital_ofdm_mapper_bcv::make_time_tag1() {
  assert(!d_time_tag);

  /* calculate the time reqd to send the preamble+header */
  int decimation = 128;
  double rate = 1.0/decimation;
 
  int num_ofdm_symbols_to_wait = 4500; //400; //3000;

  int cp_length = d_fft_length/4;
  int symbol_length = d_fft_length + cp_length;
  uint32_t num_samples = num_ofdm_symbols_to_wait * symbol_length;
  double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
  double duration = num_samples * time_per_sample;// * 3;

  uint64_t sync_secs = (uint64_t) duration;
  double sync_frac_of_secs = duration - (uint64_t) duration;
  uhd::time_spec_t duration_time = uhd::time_spec_t(sync_secs, sync_frac_of_secs);

  uhd::time_spec_t c_time = d_usrp->get_time_now();           // current time //
  uhd::time_spec_t out_time = c_time + duration_time;         // scheduled out time //

  /* check if the interval from the last packet sent is atleast the duration */
  uhd::time_spec_t interval_time = out_time - d_last_pkt_time;    // interval between the transmissions //
  if(interval_time < duration_time) {
     uhd::time_spec_t extra_gap_time = duration_time - interval_time;
     out_time += extra_gap_time;
     interval_time += extra_gap_time;
  }

  uint64_t interval_samples = (interval_time.get_frac_secs() * 1e8)/decimation;
  switch(d_mimo) {
    case 0: break;
    case 1:
            send_mimo_trigger(out_time);
            break;
    case 2:
            out_time = rcv_mimo_trigger();
	    printf("rx again: out_time (%llu, %f)\n", (uint64_t) out_time.get_full_secs(), out_time.get_frac_secs()); fflush(stdout); 
#ifndef PRE_NULL_SYMBOLS
            num_samples = (d_preamble.size()+d_num_hdr_symbols+NUM_TRAINING_SYMBOLS) * (d_fft_length+cp_length);
            duration = num_samples * time_per_sample;
            sync_secs = (uint64_t) duration;
            sync_frac_of_secs = duration - (uint64_t) duration;
            out_time += uhd::time_spec_t(sync_secs, sync_frac_of_secs);
#endif
            break;
    default: assert(false);
  }

  printf("timestamp: curr (%llu, %f), out (%llu, %f) , last_time (%llu, %f), interval(%lld, %f), interval_samples(%llu), duration(%llu, %f)\n", (uint64_t) c_time.get_full_secs(), c_time.get_frac_secs(), (uint64_t) out_time.get_full_secs(), out_time.get_frac_secs(), (uint64_t) d_last_pkt_time.get_full_secs(), d_last_pkt_time.get_frac_secs(), (int64_t) interval_time.get_full_secs(), interval_time.get_frac_secs(), interval_samples, sync_secs, sync_frac_of_secs); fflush(stdout);

  assert(out_time > c_time);
  d_out_pkt_time = out_time;

  const pmt::pmt_t key = pmt::pmt_string_to_symbol("tx_time");
  const pmt::pmt_t value = pmt::pmt_make_tuple(
		  pmt::pmt_from_uint64((uint64_t) out_time.get_full_secs()),
		  pmt::pmt_from_double(out_time.get_frac_secs())
		  );
  const pmt::pmt_t srcid = pmt::pmt_string_to_symbol(this->name());
  add_item_tag(1/*chan0*/, nitems_written(1), key, value, srcid);
  printf("(MAPPER) make_time_tag, at offset: %llu\n", nitems_written(1)); fflush(stdout);

  if(d_h_coding) {
     d_pktTxInfoList.push_back(PktTxInfo(d_pkt_num, out_time));  
     while(d_pktTxInfoList.size() > 300) {
        d_pktTxInfoList.pop_front();
     }
  }
}

/* at BSPK, generate 2 training symbols (helps in deducing snr and noise variance) */
inline void
digital_ofdm_mapper_bcv::generateKnownSymbols() {
   memset(d_known_symbols, 0, sizeof(gr_complex) * d_fft_length);
   int size = d_hdr_constellation.size();

   int k = 0;
   for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
      d_known_symbols[d_data_carriers[i]] = d_hdr_constellation[k%size];
      k++;
   }
}

void
digital_ofdm_mapper_bcv::setup_mimo_tx() {
  printf("setup_mimo_tx -- \n"); fflush(stdout);
  if(d_mimo == 1) {
     open_mimo_master_socket();
     if(d_mimo_master_sock != -1) {
        printf("MIMO MASTER connected to SLAVE\n"); fflush(stdout);
     }
  } else {
     d_mimo_slave_sock = open_mimo_slave_socket();
     if(d_mimo_slave_sock != -1) {
        printf("MIMO SLAVE connected to MASTER\n"); fflush(stdout);
     }
  }

  printf("setup_mimo tx done --\n"); fflush(stdout);
}

inline void
digital_ofdm_mapper_bcv::open_mimo_master_socket() {
  vector<unsigned int> conn_clients;
  open_server_sock(9001, conn_clients, 1);
  d_mimo_master_sock = conn_clients[0];
}

inline int
digital_ofdm_mapper_bcv::open_mimo_slave_socket() {
  return open_client_sock(9001, "128.83.141.213", true);		// define the addr 
}

inline void
digital_ofdm_mapper_bcv::send_mimo_trigger(uhd::time_spec_t out_time) {
   int buf_size = sizeof(uhd::time_spec_t);
   printf("MIMO: send_mimo_trigger, size: %d\n", buf_size); fflush(stdout);
   char *eth_buf = (char*) malloc(buf_size); 

   time_t secs = out_time.get_full_secs();
   double frac = out_time.get_frac_secs();

   memcpy(eth_buf, &secs, sizeof(time_t));
   memcpy(eth_buf+sizeof(time_t), &frac, sizeof(double));

   int bytes_sent = send(d_mimo_master_sock, (char *)&eth_buf[0], buf_size, 0);
   memcpy(&secs, eth_buf, sizeof(time_t));
   memcpy(&frac, eth_buf+sizeof(time_t), sizeof(double));

   uhd::time_spec_t _out_time(secs, frac);
   //memcpy(&_out_time, eth_buf, buf_size);
   printf("MIMO trigger sent, bytes_sent: %d, errno: %d, out_time: (%llu, %f)\n", bytes_sent, errno, (uint64_t) _out_time.get_full_secs(), _out_time.get_frac_secs()); fflush(stdout);
   free(eth_buf);
}

inline uhd::time_spec_t
digital_ofdm_mapper_bcv::rcv_mimo_trigger() {
   printf("MIMO: rcv_mimo_trigger\n"); fflush(stdout);
   uhd::time_spec_t out_time;
   int buf_size = sizeof(uhd::time_spec_t);
   char *eth_buf = (char*) malloc(buf_size);
   int nbytes = recv(d_mimo_slave_sock, eth_buf, buf_size, MSG_PEEK);
   if(nbytes > 0) {
      int offset = 0;
      while(1) {
         nbytes = recv(d_mimo_slave_sock, eth_buf+offset, buf_size-offset, 0);
	 if(nbytes > 0) offset += nbytes;
         if(offset == buf_size) {
            break;
         }
      }
      assert(offset == buf_size);
      memcpy(&out_time, eth_buf, sizeof(uhd::time_spec_t));
      printf("rx out_time: (%llu, %f)\n", (uint64_t)out_time.get_full_secs(), out_time.get_frac_secs()); fflush(stdout);
   }

   printf("MIMO slave received trigger, nbytes: %d\n", nbytes); fflush(stdout);
   free(eth_buf);
   return out_time;
}

#if 0
void
digital_ofdm_mapper_bcv::generateCodeVector()
{
  // for each subcarrier, record 'd_batch_size' coeffs //
  int num_carriers = d_data_carriers.size()/COMPRESSION_FACTOR;

  assert(d_batch_size == 2);

  float cv1 = rand() % 360 + 1;
  d_header.coeffs[0].phase = cv1 * SCALE_FACTOR_PHASE;
  d_header.coeffs[0].amplitude = SCALE_FACTOR_AMP;

  float cv2 = 0.0;
  while(1) {
     cv2 = rand() % 360 + 1;
     if(is_CV_good(ToPhase_c(cv1*M_PI/180), ToPhase_c(cv2*M_PI/180))) 
	break;
  }  

  // store the scaled versions in header (space constraints) //
  d_header.coeffs[1].phase = cv2 * SCALE_FACTOR_PHASE;
  d_header.coeffs[1].amplitude = SCALE_FACTOR_AMP;
  printf("generateCodeVector:: cv1: %.3f , cv2: %.3f\n", cv1, cv2); fflush(stdout);

#ifdef LSQ_COMPRESSION
  // source just blindly copies the coefficients, no point doing LSQ at the source //
  for(unsigned int s = 1; s < d_degree; s++) {
      for(unsigned int k = 0; k < d_batch_size; k++) {
          int index = s * d_batch_size + k;
          d_header.coeffs[index].phase = d_header.coeffs[k].phase;
          d_header.coeffs[index].amplitude = d_header.coeffs[k].amplitude;
      }
  }
#else
  // for now copy the <d_batch_size> coeffs for all the coeff entries for the header //
  for(unsigned int s = 1; s < num_carriers; s++) {
      for(unsigned int k = 0; k < d_batch_size; k++) {
          int index = s * d_batch_size + k;
          d_header.coeffs[index].phase = d_header.coeffs[k].phase;
          d_header.coeffs[index].amplitude = d_header.coeffs[k].amplitude;
      }
  }
#endif
  printf("\n");
}
#endif

/* returns the minimum distance between any 2 constellation points, given 
   cv1 and cv2 as forming the new constellation with points of the form; 
   p = cv1*x1 + cv2*x2
*/
float
digital_ofdm_mapper_bcv::calc_CV_dt(gr_complex cv1, gr_complex cv2) {
   int M = d_data_constellation.size();
   float min_dt = 1000.0;
   float energy = 0.0;

   int n_entries = pow(double(d_data_constellation.size()), double(d_batch_size));
   int index = 0;
   gr_complex *comb_mod = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);
   for(int m1 = 0; m1 < M; m1++) {
      gr_complex p1 = cv1 * d_data_constellation[m1];
      for(int m2 = 0; m2 < M; m2++) {
         gr_complex p2 = cv2 * d_data_constellation[m2];
         comb_mod[index++] = p1 + p2;
         energy += norm(p1+p2);
      }
   }
   assert(index == n_entries);
   energy /= ((float) n_entries);

#if 1
   float factor = 1.0/sqrt(energy);
   for(int i = 0; i < n_entries; i++)
      comb_mod[i] = comb_mod[i]*factor;
#endif

   float avg_dt = 0.0;
   int count = 0;
   for(int i = 0; i < n_entries; i++) {
      for(int j = i+1; j < n_entries; j++) {
         float dt = abs(comb_mod[i] - comb_mod[j]);
         if(dt < min_dt) {
            min_dt = dt;
         }
	 avg_dt += dt; count++;
      }
   }
	
   free(comb_mod);
   return min_dt;
}

/* util function */
inline void
digital_ofdm_mapper_bcv::open_server_sock(int sock_port, vector<unsigned int>& connected_clients, int num_clients) {
  int sockfd, _sock;
  struct sockaddr_in dest;

  printf("open_server_sock, #clients: %d\n", num_clients); fflush(stdout);
  /* create socket */
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  fcntl(sockfd, F_SETFL, O_NONBLOCK);

  /* ensure the socket address can be reused, even after CTRL-C the application */
  int optval = 1;
  int ret = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

  if(ret == -1) {
        printf("@ digital_ofdm_mapper_bcv::setsockopt ERROR\n"); fflush(stdout);
  }

  /* initialize structure dest */
  bzero(&dest, sizeof(dest));
  dest.sin_family = AF_INET;

  /* assign a port number to socket */
  dest.sin_port = htons(sock_port);
  dest.sin_addr.s_addr = INADDR_ANY;

  bind(sockfd, (struct sockaddr*)&dest, sizeof(dest));

  /* make it listen to socket with max 20 connections */
  listen(sockfd, 1);
  assert(sockfd != -1);

  struct sockaddr_in client_addr;
  int addrlen = sizeof(client_addr);

  int conn = 0;
  while(conn != num_clients) {
    _sock = accept(sockfd, (struct sockaddr*)&client_addr, (socklen_t*)&addrlen);
    if(_sock != -1) {
      connected_clients.push_back(_sock);
      conn++;
    }
  }
  
  printf("open_server_sock done, #clients: %d\n", num_clients); fflush(stdout);
  assert(connected_clients.size() == num_clients);
}

inline int
digital_ofdm_mapper_bcv::open_client_sock(int port, const char *addr, bool blocking) {
  int sockfd;
  struct sockaddr_in dest;

  /* create socket */
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if(!blocking)
     fcntl(sockfd, F_SETFL, O_NONBLOCK);

  assert(sockfd != -1);

  /* initialize value in dest */
  bzero(&dest, sizeof(struct sockaddr_in));
  dest.sin_family = PF_INET;
  dest.sin_port = htons(port);
  dest.sin_addr.s_addr = inet_addr(addr);
  /* Connecting to server */
  while(connect(sockfd, (struct sockaddr*)&dest, sizeof(struct sockaddr_in)) == -1)
	continue;
  printf("sockfd: %d, errno: %d\n", sockfd, errno); fflush(stdout);
  return sockfd;
}

inline NodeId
digital_ofdm_mapper_bcv::get_coFwd()
{
   assert(d_mimo >= 1);
   assert(d_outCLinks.size() == 1);                     // assume only 1 cofwd for now //

   int coLinkId = d_outCLinks[0];
   CompositeLink *cLink = getCompositeLink(coLinkId);
   int n_coFwds = cLink->srcIds.size();
   assert(n_coFwds == 2);

   for(int i = 0; i < n_coFwds; i++) {
      if(cLink->srcIds[i] != d_id)
         return cLink->srcIds[i];
   }
   assert(false);
}

/* spits all the next-hop rx-ids */
inline void
digital_ofdm_mapper_bcv::get_nextHop_rx(NodeIds &rx_ids) {
   printf("get_nextHop_rx start -- #links: %d\n", d_outCLinks.size()); fflush(stdout);
   assert(d_outCLinks.size() == 1);
   CompositeLink *link = getCompositeLink(d_outCLinks[0]);
   NodeIds dstIds = link->dstIds;
   NodeIds::iterator it = dstIds.begin();
   while(it != dstIds.end()) {
      rx_ids.push_back(*it);
      it++;
   }
   printf("get_nextHop_rx -- #: %d\n", rx_ids.size()); fflush(stdout);
}

inline void
digital_ofdm_mapper_bcv::send_coeff_info_eth(CoeffInfo *coeffs)
{
   NodeIds rx_ids;
   get_nextHop_rx(rx_ids);
   int num_rx = rx_ids.size(); assert(num_rx > 0);

   // convert the *coeffs into CoeffInfo //
   int buf_size = sizeof(CoeffInfo) * num_rx; 
   printf("send_coeff_info_eth to node%d\n", get_coFwd()); fflush(stdout);
   char *_buf = (char*) malloc(buf_size);
   memcpy(_buf, coeffs, buf_size);

   int bytes_sent = send(d_coeff_tx_sock, (char *)&_buf[0], buf_size, 0);
   printf("Coeffs sent, bytes_sent: %d, errno: %d\n", bytes_sent, errno); fflush(stdout);
   free(_buf);
}

/* the lead sender sends out 'batch_size' # of coeffs for each rx they will encounter.  */
inline void
digital_ofdm_mapper_bcv::get_coeffs_from_lead(CoeffInfo *coeffs)
{
   printf("get_coeffs_from_lead start\n"); fflush(stdout);
   NodeIds rx_ids;
   get_nextHop_rx(rx_ids);
   int num_rx = rx_ids.size(); assert(num_rx > 0);

   int buf_size = sizeof(CoeffInfo) * num_rx;
   char *_buf = (char*) malloc(buf_size);
   memset(_buf, 0, buf_size);

   int nbytes = recv(d_coeff_rx_sock, _buf, buf_size, MSG_PEEK);
  
   printf("buf_size: %d, nbytes: %d, sizeof(CoeffInfo): %d\n", buf_size, nbytes, sizeof(CoeffInfo)); fflush(stdout);
   int offset = 0;
   if(nbytes > 0) {
      while(1) {
         nbytes = recv(d_coeff_rx_sock, _buf+offset, buf_size-offset, 0);
	 if(nbytes > 0) offset += nbytes;
         if(offset == buf_size) {
            memcpy(coeffs, _buf, buf_size);
            break;
         }
      }
      assert(offset == buf_size);
   }

   assert(offset != 0);
   printf("get_coeffs_from_lead end -- offset: %d\n", offset); fflush(stdout);
   free(_buf);
}

inline void
digital_ofdm_mapper_bcv::smart_selection_local(gr_complex *coeffs, CoeffInfo *coeffInfo) {
  int num_carriers = d_data_carriers.size();
  printf("smart_selection_local start\n"); fflush(stdout);

  // get the predicted value of h for each of the receivers //
  NodeIds rx_ids;
  get_nextHop_rx(rx_ids);
  vector<gr_complex> h_vec;
  for(int i = 0; i < rx_ids.size(); i++) {
     gr_complex h = predictH(d_id, rx_ids[i]);
     h_vec.push_back(h);
  }

  // to cap the max iterations //
  int max_iter = 1000, iter = 0;
  gr_complex best_coeff[MAX_BATCH_SIZE];
  float max_min_dt = 0.0;
  float threshold = 5.8;


  gr_complex *reduced_coeffs;
  while(1) {

     for(int k = 0; k < d_batch_size; k++) {
        float amp = 1.0;
        float phase = (rand() % 360 + 1) * M_PI/180;
        coeffs[k] = (amp * ToPhase_c(phase));
     }

     // ensure its a good selection over all the next-hop rx //
     float dt = 0.0;     
     for(int i = 0; i < rx_ids.size(); i++) {
        CoeffInfo *cInfo = &(coeffInfo[i]);
        cInfo->rx_id = rx_ids[i];

        // include the channel effect //
        reduced_coeffs = cInfo->coeffs;
        for(int k = 0; k < d_batch_size; k++) {
           reduced_coeffs[k] = coeffs[k] * h_vec[i];
        }

        dt += calc_CV_dt(reduced_coeffs[0], reduced_coeffs[1]);
     }
     dt /= ((float) rx_ids.size());

     bool good = (dt >= threshold)?true:false;

     // record if this is the best yet //
     if(!good) {
	if(dt > max_min_dt) { 
	   max_min_dt = dt;
	   for(int k = 0; k < d_batch_size; k++)
	       best_coeff[k] = coeffs[k];
	}
     }
     else max_min_dt = dt;

     iter++;
     if(good || iter == max_iter) {
        printf("good: %d, iter: %d, max_min_dt: %f\n", good, iter, max_min_dt); fflush(stdout);
        for(int k = 0; k < d_batch_size; k++)
           coeffs[k] = best_coeff[k];
        break;
     }
  } // while

  if(d_batch_size == 2) {
     printf("smart selection local -- coeffs1 (%.2f, %.2f) coeffs2 (%.2f, %.2f) red1 (%.2f, %.2f) red2 (%.2f, %.2f)\n",
                                  coeffs[0].real(), coeffs[0].imag(), coeffs[1].real(), coeffs[1].imag(),
                                  reduced_coeffs[0].real(), reduced_coeffs[0].imag(), reduced_coeffs[1].real(), reduced_coeffs[1].imag());
     fflush(stdout);
  }
}
#if 0
inline void
digital_ofdm_mapper_bcv::smart_selection_local(gr_complex *coeffs, CoeffInfo *coeffInfo) {


  int num_carriers = d_data_carriers.size();
  printf("smart_selection_local start\n"); fflush(stdout);

  /* randomly choose every coefficient except for the last one and keep reducing to latest value */
  for(unsigned int i = 0; i < d_batch_size-1; i++) {
     float amp = 1.0;
     float phase = (rand() % 360 + 1) * M_PI/180;

     coeffs[i] = (amp * ToPhase_c(phase));
  }

  int last = d_batch_size-1;
  float amp = 1.0;

  // get the predicted value of h for each of the receivers //
  NodeIds rx_ids;
  get_nextHop_rx(rx_ids);
  vector<gr_complex> h_vec;
  for(int i = 0; i < rx_ids.size(); i++) {
     gr_complex h = predictH(d_id, rx_ids[i]);
     h_vec.push_back(h);
  }

  // to cap the max iterations //
  int max_iter = 1000, iter = 0;
  gr_complex best_coeff;
  float max_min_dt = 0.0;
  gr_complex *reduced_coeffs;

  while(1) {
     float phase = (rand() % 360 + 1) * M_PI/180;
     coeffs[last] = (amp * ToPhase_c(phase));

     // ensure its a good selection over all the next-hop rx //
     bool good = 1;
     for(int i = 0; i < rx_ids.size(); i++) {
	CoeffInfo *cInfo = &(coeffInfo[i]);
        cInfo->rx_id = rx_ids[i];

	// include the channel effect //
	reduced_coeffs = cInfo->coeffs;
        for(int k = 0; k < d_batch_size; k++) {
	   reduced_coeffs[k] = coeffs[k] * h_vec[i];
        }

	if(d_batch_size == 1)
           break;

	float dt = 1000.0;
        if(!is_CV_good(reduced_coeffs[0], reduced_coeffs[1], dt)) {
            good = 0;
	    if(dt > max_min_dt) {
                max_min_dt = dt;
                best_coeff = coeffs[last];
            }
            break;
        }
     }

     iter++;
     if(good || iter == max_iter) {
        printf("good: %d, iter: %d, max_min_dt: %f\n", good, iter, max_min_dt); fflush(stdout);
        coeffs[last] = best_coeff;
        break;
     }
  }

  if(d_batch_size == 2) {
     printf("smart selection local -- coeffs1 (%.2f, %.2f) coeffs2 (%.2f, %.2f) red1 (%.2f, %.2f) red2 (%.2f, %.2f)\n",
                                  coeffs[0].real(), coeffs[0].imag(), coeffs[1].real(), coeffs[1].imag(),
                                  reduced_coeffs[0].real(), reduced_coeffs[0].imag(), reduced_coeffs[1].real(), reduced_coeffs[1].imag());
     fflush(stdout);
  }
}
#endif

/* more exhaustive now, since it accounts for co-ordinating transmitters, multiple rx (if any) */
inline void
digital_ofdm_mapper_bcv::smart_selection_global(gr_complex *my_coeffs, CoeffInfo *others_coeffs) {
  printf("smart_selection_global start\n"); fflush(stdout);

  NodeIds rx_ids;
  get_nextHop_rx(rx_ids);
  vector<gr_complex> h_vec;
  for(int i = 0; i < rx_ids.size(); i++) {
     gr_complex h = predictH(d_id, rx_ids[i]);
     h_vec.push_back(h);
  }

  /* first, formulate the constant part, which does not change with different receivers */
  gr_complex new_coeffs[MAX_BATCH_SIZE];
  int num_carriers = d_data_carriers.size();

  // to cap the max iterations //
  int max_iter = 1000, iter = 0;
  gr_complex best_coeff[MAX_BATCH_SIZE];
  float max_min_dt = 0.0;
  float threshold = 5.8;

  /* as for the last one, take the global knowledge into account */
  while(1) {

     for(int k = 0; k < d_batch_size; k++) {
        float amp = 1.0;
        float phase = (rand() % 360 + 1) * M_PI/180;
        my_coeffs[k] = (amp * ToPhase_c(phase));
     }

     // now the receiver based stuff //
     float dt = 0.0;
     for(int i = 0; i < rx_ids.size(); i++) {
        gr_complex *o_coeffs = others_coeffs[i].coeffs;     // other senders coeffs for this rx
	assert(others_coeffs[i].rx_id == rx_ids[i]);	    // sanity

        // combine with the other sender //    
        for(unsigned int k = 0; k < d_batch_size; k++) {
           new_coeffs[k] = (my_coeffs[k] * h_vec[i]) + o_coeffs[k];
        }

	dt += calc_CV_dt(new_coeffs[0], new_coeffs[1]);
     }
     dt /= ((float) rx_ids.size());

     // record if this is the best yet //
     bool good = (dt >= threshold)?true:false;
     if(!good) {
        if(dt > max_min_dt) {
           max_min_dt = dt;
           for(int k = 0; k < d_batch_size; k++)
               best_coeff[k] = my_coeffs[k];
        }
     }
     else max_min_dt = dt;
     iter++;

     if(good || (iter == max_iter)) {
        printf("good: %d, iter: %d, min_dt: %f\n", good, iter, max_min_dt); fflush(stdout);

	if(iter == max_iter) {
	   for(int k = 0; k < d_batch_size; k++) {
               my_coeffs[k] = best_coeff[k];
	   }
	}
        break;
     }
  } //end while
  printf("smart_selection_global end -- \n"); fflush(stdout);
}

#if 0
/* more exhaustive now, since it accounts for co-ordinating transmitters, multiple rx (if any) */
inline void
digital_ofdm_mapper_bcv::smart_selection_global(gr_complex *my_coeffs, CoeffInfo *others_coeffs) {
  unsigned int n_inno_pkts = d_batch_size;
  printf("smart_selection_global start\n"); fflush(stdout);

  NodeIds rx_ids;
  get_nextHop_rx(rx_ids);

  /* first, formulate the constant part, which does not change with different receivers */
  gr_complex new_coeffs[MAX_BATCH_SIZE];
  int num_carriers = d_data_carriers.size();

  /* randomly choose every coefficient except for the last one and keep reducing to latest value */
  for(unsigned int i = 0; i < n_inno_pkts-1; i++) {
     float amp = 1.0;
     float phase = (rand() % 360 + 1) * M_PI/180;

     my_coeffs[i] = (amp * ToPhase_c(phase));
     new_coeffs[i] = my_coeffs[i];
  } /* step 1 done */

  // to cap the max iterations //
  int max_iter = 1000, iter = 0;
  gr_complex best_coeff;
  float max_min_dt = 0.0;

  /* as for the last one, take the global knowledge into account */
  int last = n_inno_pkts-1;
  float amp = 1.0;

  // predict H for all the receivers and 
  gr_complex h_arr[4];
  for(int i = 0; i < rx_ids.size(); i++) {
     h_arr[i] = predictH(d_id, rx_ids[i]);               // h-val from me to this rx
  }

  while(1) {
     float phase = (rand() % 360 + 1) * M_PI/180;
     my_coeffs[last] = (amp * ToPhase_c(phase));
     new_coeffs[last] = my_coeffs[last];

     // now the receiver based stuff //
     bool good = 1;
     for(int i = 0; i < rx_ids.size(); i++) {
        gr_complex *o_coeffs = others_coeffs[i].coeffs;     // other senders coeffs for this rx

        if(others_coeffs[i].rx_id != rx_ids[i]) {
	   printf("others_coeff.rx_id: %c, rx_id: %c\n", others_coeffs[i].rx_id, rx_ids[i]); fflush(stdout);
	   assert(false);
        }

        // combine with the other sender //    
        for(unsigned int k = 0; k < d_batch_size; k++) {
           new_coeffs[k] = (new_coeffs[k] * h_arr[i]) + o_coeffs[k];
        }
	
	float dt = 1000.0;
        if(!is_CV_good(new_coeffs[0], new_coeffs[1], dt)) {
            good = 0;
            if(dt > max_min_dt) {
               max_min_dt = dt;
               best_coeff = my_coeffs[last];
            }
            break;
        }
     }

     iter++;
     if(good == 1 || (iter == max_iter)) {
	printf("good: %d, iter: %d, min_dt: %f\n", good, iter, max_min_dt); fflush(stdout);
	my_coeffs[last] = best_coeff;
        break;
     }
  } //end while
  printf("smart_selection_global end -- \n"); fflush(stdout);
}
#endif

inline void
digital_ofdm_mapper_bcv::chooseCV_H(gr_complex *coeffs) {
  printf("chooseCV_H -- \n"); fflush(stdout);
  assert(d_batch_size == 2);

  // first check if any outstanding HInfo available //
  vector<unsigned int>:: iterator it = d_h_rx_socks.begin();
  while(it != d_h_rx_socks.end()) {
     check_HInfo_rx_sock(*it);
     it++;
  }

  if(d_mimo <=1) {
     CoeffInfo reduced_coeffs[MAX_RX];
     smart_selection_local(coeffs, reduced_coeffs);
     if(d_mimo == 1)
        send_coeff_info_eth(reduced_coeffs);
  } else {
     assert(d_mimo == 2);
     CoeffInfo lead_coeffs[MAX_RX];
     get_coeffs_from_lead(lead_coeffs);
     smart_selection_global(coeffs, lead_coeffs);
  }
}

inline HInfo*
digital_ofdm_mapper_bcv::getHInfo(NodeId tx_id, NodeId rx_id) {
  HKey hkey(tx_id, rx_id);
  HInfoMap::iterator it = d_HInfoMap.find(hkey);
  assert(it != d_HInfoMap.end());
  return (HInfo*) it->second;
}

/* predict the value of H as a fn(slope, samples since last tx) */
inline gr_complex
digital_ofdm_mapper_bcv::predictH(NodeId tx_id, NodeId rx_id) {
#if 0 
  // calculate the # of samples since last packet sent //
  uhd::time_spec_t interval = d_out_pkt_time - d_last_pkt_time;  
  time_t full_secs = interval.get_full_secs();
  double frac_secs = interval.get_frac_secs();
  double total_time = full_secs + frac_secs;

  int decimation = 128;
  double rate = 1.0/decimation;
  double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
  uint64_t interval_samples = total_time/time_per_sample;
  std::cout<<"time/sample: "<<time_per_sample<<" samples: "<<interval_samples<<" total time: "<<total_time<<endl;
  d_last_pkt_time = d_out_pkt_time;                                             // no use now, set it again
#endif

  // now try and predict H based on interval_samples //
  HInfo *hInfo = getHInfo(tx_id, rx_id);
  gr_complex h_val = hInfo->h_value;
  int h_pkt_num = hInfo->pkt_num;
  float slope = hInfo->slope;

  if(h_pkt_num == -1 || slope == 0.0)
     return gr_complex(1.0, 0.0);

  // calculate the # of samples since the pkt_num that corresponds to the latest known h_val //
  uhd::time_spec_t h_val_ts = getPktTimestamp(h_pkt_num);
  uhd::time_spec_t interval = d_out_pkt_time - h_val_ts;
  time_t full_secs = interval.get_full_secs();
  double frac_secs = interval.get_frac_secs();
  double total_time = full_secs + frac_secs;

  int decimation = 128;
  double rate = 1.0/decimation;
  double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
  uint64_t interval_samples = total_time/time_per_sample;
  std::cout<<"time/sample: "<<time_per_sample<<" samples: "<<interval_samples<<" total time: "<<total_time<<endl;
  d_last_pkt_time = d_out_pkt_time;                                             // no use now, set it again

  float old_angle = arg(h_val);
  float new_angle = old_angle + interval_samples * slope;

  printf("old_angle: %.2f, new_angle: %.2f\n", old_angle, new_angle); fflush(stdout);
  while(new_angle > M_PI)
	  new_angle = new_angle - 2*M_PI; 

  assert(new_angle <= M_PI && new_angle >= -M_PI); 

  printf("predictH: pkt_num: %d, slope: %.8f, samples: %llu, old_angle: %.2f, new_angle: %.2f\n", d_pkt_num, slope, interval_samples, old_angle, new_angle); fflush(stdout);
  //printf("predictH: pkt_num: %d, slope: %f, old_angle: %.2f, new_angle: %.2f\n", d_pkt_num, slope, old_angle, new_angle); fflush(stdout);

  return gr_expj(new_angle);	

#if 0  
  float o_angle = arg(h_val);
  float n_angle = o_angle * slope;
  
  if(n_angle > M_PI) {
     n_angle = -2*M_PI + n_angle;
  }

  h_value = abs(h_value) * gr_expj(n_angle);
  printf("predictH: new estimated H, mag: %.2f, angle: %.2f {%.2f}, [%.2f, %.2f]\n", abs(h_val), n_angle, n_angle*180/M_PI, h_val.real(), h_val.imag());
  fflush(stdout);
  return h_val;
  return gr_complex(1.0, 0.0);
#endif
}

/* opens the h_tx and h_rx sockets 
   figure out the upstream and downstream nodes and open sockets accordingly
*/
inline void
digital_ofdm_mapper_bcv::prepare_H_coding() {
   populateEthernetAddress();
   EthInfoMap::iterator it;

   int num_out_links = d_outCLinks.size();
   if(num_out_links > 0) {
      assert(num_out_links == 1);
      /* create 1 rx client socket per downstream nodes to receive from */
      for(int i = 0; i < num_out_links; i++) {
         CompositeLink *link = getCompositeLink(d_outCLinks[i]);

         NodeIds dst_ids = link->dstIds;
         for(int j = 0; j < dst_ids.size(); j++) {
            it = d_ethInfoMap.find(dst_ids[j]);
            assert(it != d_ethInfoMap.end());
            EthInfo *ethInfo = (EthInfo*) it->second;
            int rx_sock = open_client_sock(ethInfo->port, ethInfo->addr, false);
	
	    assert(rx_sock != -1);
            //d_h_rx_socks.insert(pair<int, int>(dst_ids[j], rx_sock));         // dstId will send HInfo
            d_h_rx_socks.push_back(rx_sock);
         }
      }
   }

   printf("opened H sockets for exchanging H information, #d_h_rx_socks: %d!\n", d_h_rx_socks.size()); fflush(stdout);

   /* create 1 tx socket to send the coeffs over, if lead sender */
   if(d_mimo == 1) {
      NodeId co_id = get_coFwd();
      printf("COEFF socket - mimo master waiting for clients .... \n"); fflush(stdout);
      it = d_ethInfoMap.find(d_id);
      assert(it != d_ethInfoMap.end());
      EthInfo *ethInfo = (EthInfo*) it->second;
      vector<unsigned int> conn_socks;
      open_server_sock(ethInfo->port + 50, conn_socks, 1);              // only 1 client needs to be connected
      printf("COEFF socket - mimo master has 1 client! \n"); fflush(stdout);
      d_coeff_tx_sock = conn_socks[0];
   }
   /* else open 1 rx socket to receive from the lead sender */
   else if (d_mimo == 2) {
      NodeId co_id = get_coFwd();
      printf("COEFF socket - mimo slave connecting to mimo master .... \n"); fflush(stdout);
      it = d_ethInfoMap.find(co_id);
      assert(it != d_ethInfoMap.end());
      EthInfo *ethInfo = (EthInfo*) it->second;
      d_coeff_rx_sock = open_client_sock(ethInfo->port+50, ethInfo->addr, true);
      printf("COEFF socket - mimo slave connected to mimo master!\n"); fflush(stdout);
   }

   initHInfoMap();
   //initHObsQMap();
}

#if 0
// called when (1) receive a packet on the air, and (2) receive HInfo packet over ethernet //
inline void
digital_ofdm_mapper_bcv::updateHInfo(HKey hkey, HInfo *_hInfo) {
   HInfoMap::iterator it = d_HInfoMap.find(hkey);
   assert(it != d_HInfoMap.end());
   HInfo *hInfo = (HInfo*) it->second;

   hInfo->pkt_num = _hInfo->pkt_num;

   if(hInfo->pkt_num > 0) {
      float diff = arg(_hInfo->h_value - hInfo->h_value);
      hInfo->slope = (hInfo->slope + diff)/2.0;
   }

#if 0
   NodeId rxId = hkey.second;
   HObsQMap::iterator it = d_hObsQMap.find(rxId);
   assert(it != d_hObsQMap.end());
   HObsQ hObsQ = it.second;
   hObsQ.push_back(pair<>);
#endif

   printf("updateHInfo - pkt: %d, slope: %f, old_val: (%.2f, %.2f), new_val: (%.2f, %.2f)\n", 
		hInfo->pkt_num, hInfo->slope, hInfo->h_value.real(), hInfo->h_value.imag(), _hInfo->h_value.real(), _hInfo->h_value.imag()); fflush(stdout);
   hInfo->h_value = _hInfo->h_value;
}
#endif

// called when receive HInfo packet over Ethernet from downstream receivers //
inline void
digital_ofdm_mapper_bcv::updateHInfo(HKey hkey, HInfo *_hInfo) {
   printf("updateHInfo (%c -> %c)\n", hkey.first, hkey.second); fflush(stdout);
   HInfoMap::iterator it = d_HInfoMap.find(hkey);
   assert(it != d_HInfoMap.end());
   HInfo *hInfo = (HInfo*) it->second;

   // discard if a repetitive report //
   if(_hInfo->pkt_num == hInfo->pkt_num)
        return;

   float old_angle = arg(hInfo->h_value);
   float new_angle = arg(_hInfo->h_value);

   // calculate the slope, etc only if this is not the first update //
   if(hInfo->pkt_num > -1) {
      uhd::time_spec_t new_ts = getPktTimestamp(_hInfo->pkt_num);
      uhd::time_spec_t old_ts = getPktTimestamp(hInfo->pkt_num);

      uhd::time_spec_t interval = new_ts - old_ts;
      time_t full_secs = interval.get_full_secs();
      double frac_secs = interval.get_frac_secs();
      double total_time = full_secs + frac_secs;

      int decimation = 128;
      double rate = 1.0/decimation;
      double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
      uint64_t interval_samples = total_time/time_per_sample;
      std::cout<<"time/sample: "<<time_per_sample<<" samples: "<<interval_samples<<" total time: "<<total_time<<endl;

      printf(" -- old_angle: %.2f, new_angle: %.2f\n", old_angle, new_angle); fflush(stdout);
      if(new_angle < old_angle) {
	  new_angle = new_angle + 2*M_PI;
      }
      float diff = new_angle - old_angle;
 
      float slope = diff/interval_samples;
      if(hInfo->slope > 0.0)
	hInfo->slope = (hInfo->slope + slope)/2.0;
      else
	hInfo->slope = slope;		

      printf("-- interval_samples: %d\n", interval_samples); fflush(stdout);
   }

   // now update the hInfo //
   printf("updateHInfo - pkt_num: %d, old_pkt: %d, slope: %f, old_angle: %.2f, new_angle: %.2f, timing_off: %f \n", _hInfo->pkt_num, hInfo->pkt_num, hInfo->slope, old_angle, arg(_hInfo->h_value), _hInfo->timing_slope); fflush(stdout);

   hInfo->pkt_num = _hInfo->pkt_num;
   hInfo->h_value = _hInfo->h_value;

   //if(hInfo->timing_slope == 0)
      hInfo->timing_slope = _hInfo->timing_slope;
}

// initializes the HInfoMap to have keys for all the relevant entries //
inline void
digital_ofdm_mapper_bcv::initHInfoMap() {
   int num_out_links = d_outCLinks.size(); assert(num_out_links == 1);
   assert(d_inCLinks.size() == 0);

   if(num_out_links == 1) {
      CompositeLink *link = getCompositeLink(d_outCLinks[0]);
      NodeIds dstIds = link->dstIds;
      NodeIds::iterator it = dstIds.begin();
      while(it != dstIds.end()) {
         unsigned char rx_id = *it;
         HKey hkey(d_id, rx_id);
         HInfo *hInfo = (HInfo*) malloc(sizeof(HInfo));
         memset(hInfo, 0, sizeof(HInfo));
	 hInfo->pkt_num = -1;					// init //
         d_HInfoMap.insert(pair<HKey, HInfo*>(hkey, hInfo));
         it++;
      }
   }

   printf("initHInfoMap size: %d\n", d_HInfoMap.size());
}

inline uhd::time_spec_t
digital_ofdm_mapper_bcv::getPktTimestamp(int pkt_num) {
   assert(d_pktTxInfoList.size() > 0);
   PktTxInfoList::iterator it = d_pktTxInfoList.begin();
   while(it != d_pktTxInfoList.end()) {
       PktTxInfo item = *it;
       if(item.first == pkt_num) 
	   return item.second;	
       it++;
   }
   printf("getPktTimestamp failed for pkt_num: %d\n", pkt_num); fflush(stdout);
   assert(false);
}

#if 0
// initializes the HObsQMap which maintains a Queue of H observations for every receiver //
inline void
digital_ofdm_mapper_bcv::initHObsQMap() {
   NodeIds rxIds;
   get_nextHop_rx(rxIds);
   
   assert(rxIds.size() >= 1);
   NodeIds::iterator it = rxIds.begin();
   while(it != rxIds.end()) {
      HObsQ q;
      d_hObsQMap.insert(pair<*it, q>);
      it++;
   }
}

inline void
digital_ofdm_mapper_bcv::updateHObsQMap() {
   
}
#endif

/* check if any outstanding msgs on HInfo rx sock, if yes, then extract those HInfo 
   structures and insert them into HInfoMap. Upto 2 HInfo can be extracted every time 
*/
inline void
digital_ofdm_mapper_bcv::check_HInfo_rx_sock(int rx_sock) {
   printf("check_HInfo_rx_sock start\n"); fflush(stdout);
   int buf_size = sizeof(HInfo) + (sizeof(NodeId) * 2);                // extra 2 bytes for the sender's+rx id
   char *_buf = (char*) malloc(buf_size);
   memset(_buf, 0, buf_size);

   int n_extracted = 0;
   int nbytes = recv(rx_sock, _buf, buf_size, MSG_PEEK);
   NodeId rxId[2];
   if(nbytes > 0) {
      printf(" -- msg rcvd, nbytes: %d\n", nbytes); fflush(stdout);
      int offset = 0;
      while(1) {
         nbytes = recv(rx_sock, _buf+offset, buf_size-offset, 0);
	 if(nbytes > 0) offset += nbytes;
	 printf("nbytes: %d, offset: %d\n", nbytes, offset); fflush(stdout);
         if(offset == buf_size) {

            rxId[n_extracted] = _buf[0];                               // copy the sender's id
	    NodeId id = _buf[1];

            // updateHInfo only if it is my H report //
            if(id == d_id) {
               HKey hkey(d_id, rxId[n_extracted]);

               HInfo *hInfo = (HInfo*) malloc(sizeof(HInfo));
               memcpy(hInfo, _buf+2, sizeof(HInfo));                     // copy HInfo

               updateHInfo(hkey, hInfo);
               free(hInfo);
            }

            n_extracted++;

            if(n_extracted == 1) {
              // see if there's another packet - I'll try and get only one more, rest later //
	      printf("n_extracted: %d\n", n_extracted); fflush(stdout);
              memset(_buf, 0, buf_size);
              nbytes = recv(rx_sock, _buf, buf_size, MSG_PEEK);
              if(nbytes > 0) {
		 printf("another packet rx, nbytes: %d\n", nbytes); fflush(stdout);
                 offset = 0;
                 continue;
              }
            }
            break;
         }
      }
      assert(offset == buf_size);
   }
   
   // just to find out how many packets remain to be read 
   char tmp_buf[200];
   nbytes = recv(rx_sock, tmp_buf, 200, MSG_PEEK);
   int waiting = nbytes/buf_size;


   printf("check_HInfo_rx, nextracted: %d, remaining: %d\n", n_extracted, waiting); fflush(stdout);
   free(_buf);
}

inline void
digital_ofdm_mapper_bcv::populateEthernetAddress()
{
   // node-id ethernet-addrees port-on-which-it-is-listening //
   printf("populateEthernetAddress\n"); fflush(stdout);
   FILE *fl = fopen ( "eth_add.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
        exit (1);
   }

   char line[128];
   const char *delim = " ";
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token_vec;
        while (strtok_res != NULL) {
           token_vec.push_back(strtok_res);
           //printf ("%s\n", strtok_res);
           strtok_res = strtok (NULL, delim);
        }

        EthInfo *ethInfo = (EthInfo*) malloc(sizeof(EthInfo));
        memset(ethInfo, 0, sizeof(EthInfo));
        NodeId node_id = (NodeId) atoi(token_vec[0]) + '0';
        memcpy(ethInfo->addr, token_vec[1], 16);
        unsigned int port = atoi(token_vec[2]);
        ethInfo->port = port;

        d_ethInfoMap.insert(pair<NodeId, EthInfo*> (node_id, ethInfo));

        std::cout<<"Inserting in the map: node " << node_id << " addr: " << ethInfo->addr << endl;
   }

    fclose (fl);
}

CompositeLink*
digital_ofdm_mapper_bcv::getCompositeLink(int id)
{
   CompositeLinkVector::iterator it = d_compositeLinkVector.begin();
   while(it != d_compositeLinkVector.end()) {
        CompositeLink *cLink = *it;
        if(cLink->linkId == id)
            return cLink;
        it++;
   }
   assert(false);
   return NULL;
}

inline void
digital_ofdm_mapper_bcv::populateCompositeLinkInfo()
{
   // link-id   #of-src     src1   src2   src3    #of-dst   dst1   dst2    dst3    //
   printf("populateCompositeLinkInfo\n"); fflush(stdout);

   FILE *fl = fopen ( "compositeLink_info.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
        exit (1);
   }

   char line[128];
   const char *delim = " ";
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token_vec;
        while (strtok_res != NULL) {
           token_vec.push_back(strtok_res);
           //printf ("%s\n", strtok_res);
           strtok_res = strtok (NULL, delim);
        }

        printf("size: %d\n", token_vec.size()); fflush(stdout);

        CompositeLink *cLink = (CompositeLink*) malloc(sizeof(CompositeLink));
        memset(cLink, 0, sizeof(CompositeLink));
        cLink->linkId = atoi(token_vec[0]);
        printf("linkId: %d\n", cLink->linkId); fflush(stdout);

        int num_src = atoi(token_vec[1]);
        printf("num_src: %d\n", num_src); fflush(stdout);
        for(int i = 0; i < num_src; i++) {
           NodeId srcId = atoi(token_vec[i+2]) + '0';
           printf("srcId: %c, size: %d\n", srcId, cLink->srcIds.size()); fflush(stdout);
           cLink->srcIds.push_back(srcId);
           if(srcId == d_id)
             d_outCLinks.push_back(cLink->linkId);
        }

        int num_dst = atoi(token_vec[num_src+2]);
        printf("num_dst: %d\n", num_dst); fflush(stdout);
        for(int i = 0; i < num_dst; i++) {
           NodeId dstId = atoi(token_vec[num_src+3+i]) + '0';
           printf("dstId: %c\n", dstId); fflush(stdout);
           cLink->dstIds.push_back(dstId);
           if(dstId == d_id)
             d_inCLinks.push_back(cLink->linkId);
        }

        d_compositeLinkVector.push_back(cLink);


        printf("\n");
   }

    fclose (fl);
}

inline void
digital_ofdm_mapper_bcv::generatePreamble(gr_complex *out) {
    memcpy(out, &d_preamble[d_preambles_sent][0], d_fft_length*sizeof(gr_complex));
}
