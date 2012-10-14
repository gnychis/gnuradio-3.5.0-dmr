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
			 unsigned int msgq_limit, 
                         unsigned int occupied_carriers, unsigned int fft_length, unsigned int id,
			 unsigned int source,
			 unsigned int batch_size,
			 unsigned int encode_flag,
			 int fwd_index, unsigned int dst_id, unsigned int degree,
			 unsigned int mimo)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_mapper_bcv (hdr_constellation, data_constellation, msgq_limit, occupied_carriers, fft_length, id, source, batch_size, encode_flag, fwd_index, dst_id, degree, mimo));
}

// Consumes 1 packet and produces as many OFDM symbols of fft_length to hold the full packet
digital_ofdm_mapper_bcv::digital_ofdm_mapper_bcv (const std::vector<gr_complex> &hdr_constellation, 
					const std::vector<gr_complex> &data_constellation,
					unsigned int msgq_limit, 
					unsigned int occupied_carriers, unsigned int fft_length, unsigned int id,
					unsigned int source,
					unsigned int batch_size,
					unsigned int encode_flag,
					int fwd_index, unsigned int dst_id, unsigned int degree, 
					unsigned int mimo)
  : gr_sync_block ("ofdm_mapper_bcv",
		   gr_make_io_signature (0, 0, 0),
		   gr_make_io_signature2 (1, 2, sizeof(gr_complex)*fft_length, sizeof(char))),
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
    d_id(id),
    d_encode_flag(encode_flag),
    d_sock_opened(false),
    d_time_tag(false),
    d_trigger_sock_opened(false),
    d_fwd_index(fwd_index),
    d_dst_id(dst_id),
    d_degree(degree),
    d_mimo(mimo)
{
  if (!(d_occupied_carriers <= d_fft_length))
    throw std::invalid_argument("digital_ofdm_mapper_bcv: occupied carriers must be <= fft_length");

  // MIMO_TX
  if(d_mimo != 0) {
     setup_mimo_tx();
  }

#ifdef ACK_ON_ETHERNET
  d_sock_fd = openACKSocket();
#endif

#ifdef TRIGGER_ON_ETHERNET
  d_trigger_src_ip_addr = "128.83.141.213";
  d_trigger_src_sock_port = 9001;
  d_trigger_sock = -1;
#endif
  std::string arg("");
  d_usrp = uhd::usrp::multi_usrp::make(arg);
  d_last_pkt_time = uhd::time_spec_t(0.0);

  // this is not the final form of this solution since we still use the occupied_tones concept,
  // which would get us into trouble if the number of carriers we seek is greater than the occupied carriers.
  // Eventually, we will get rid of the occupied_carriers concept.
  std::string carriers = "F00F";		// apurv++,  8 DC subcarriers
  //std::string carriers = "FC3F";              // apurv++,  4 DC subcarriers
 
  // A bit hacky to fill out carriers to occupied_carriers length
  int diff = (d_occupied_carriers - 4*carriers.length()); 
  while(diff > 7) {
    carriers.insert(0, "f");
    carriers.insert(carriers.length(), "f");
    diff -= 8;
  }

  // if there's extras left to be processed
  // divide remaining to put on either side of current map
  // all of this is done to stick with the concept of a carrier map string that
  // can be later passed by the user, even though it'd be cleaner to just do this
  // on the carrier map itself
  int diff_left=0;
  int diff_right=0;

  // dictionary to convert from integers to ascii hex representation
  char abc[16] = {'0', '1', '2', '3', '4', '5', '6', '7', 
		  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
  if(diff > 0) {
    char c[2] = {0,0};

    diff_left = (int)ceil((float)diff/2.0f);   // number of carriers to put on the left side
    c[0] = abc[(1 << diff_left) - 1];          // convert to bits and move to ASCI integer
    carriers.insert(0, c);
    
    diff_right = diff - diff_left;	       // number of carriers to put on the right side
    c[0] = abc[0xF^((1 << diff_right) - 1)];   // convert to bits and move to ASCI integer
        carriers.insert(carriers.length(), c);
  }
  
  // find out how many zeros to pad on the sides; the difference between the fft length and the subcarrier
  // mapping size in chunks of four. This is the number to pack on the left and this number plus any 
  // residual nulls (if odd) will be packed on the right. 
  diff = (d_fft_length/4 - carriers.length())/2; 

#ifdef USE_PILOT 
  /* pilot configuration */ 
  int num_pilots = 8; //12; //4;
  unsigned int pilot_index = 0;			     // tracks the # of pilots carriers added   
  unsigned int data_index = 0;			     // tracks the # of data carriers added
  unsigned int count = 0;			     // tracks the total # of carriers added
  unsigned int pilot_gap = 11; //7; //18;
  unsigned int start_offset = 0; //8;
#if 0
  /* pilot configuration */ 
  int num_pilots = 4;
  unsigned int pilot_index = 0;                      // tracks the # of pilots carriers added   
  unsigned int data_index = 0;                       // tracks the # of data carriers added
  unsigned int count = 0;                            // tracks the total # of carriers added
  unsigned int pilot_gap = 14; 
  unsigned int start_offset = 2;
#endif
#endif

  unsigned int i,j,k;
  //for(i = 0; i < (d_occupied_carriers/4)+diff_left; i++) {
  for(i = 0; i < carriers.length(); i++) {
    char c = carriers[i];                            // get the current hex character from the string
    for(j = 0; j < 4; j++) {                         // walk through all four bits
      k = (strtol(&c, NULL, 16) >> (3-j)) & 0x1;     // convert to int and extract next bit
      if(k) {                                        // if bit is a 1, 
	int carrier_index = 4*(i+diff) + j - diff_left;
	//int carrier_index = 4*i + j - diff_left;

#ifdef USE_PILOT
	// check if it should be pilot, else add as data //
	if(count == (start_offset + (pilot_index * pilot_gap))) {		// check notes below !
	   d_pilot_carriers.push_back(carrier_index);
	   pilot_index++;
	}
	else {
   	   d_data_carriers.push_back(carrier_index);  // use this subcarrier
	   data_index++;
	}
	count++;
#else
	d_data_carriers.push_back(carrier_index);
#endif	
      }
    }
  }

#ifdef USE_PILOT 
  assert(pilot_index + data_index == count);

  /* debug carriers */ 
  printf("pilot carriers (%d): \n", d_pilot_carriers.size()); 
  for(int i = 0; i < d_pilot_carriers.size(); i++) {
     printf("%d ", d_pilot_carriers[i]); fflush(stdout);
  }
  printf("\n");
  assert(d_pilot_carriers.size() == pilot_index);
  assert(d_data_carriers.size() == data_index); 
  
  // hack the above to populate the d_pilots_carriers map and remove the corresponding entries from d_data_carriers //
  /* if # of data carriers = 72
	# of pilots        = 6
	gap b/w pilots     = 72/6 = 12
 	start pilot	   = 5
	other pilots	   = 5, 17, 29, .. so on
	P.S: DC tones should not be pilots!
  */  
#endif
  printf("data carriers (%d): \n", d_data_carriers.size());
  for(int i = 0; i < d_data_carriers.size(); i++) {
     printf("%d ", d_data_carriers[i]); fflush(stdout);
  }
  printf("\n");

  // make sure we stay in the limit currently imposed by the occupied_carriers
  if(d_data_carriers.size() > d_occupied_carriers) {
    throw std::invalid_argument("digital_ofdm_mapper_bcv: subcarriers allocated exceeds size of occupied carriers");
  }
  fill_all_carriers_map();  

  d_hdr_nbits = (unsigned long)ceil(log10(float(d_hdr_constellation.size())) / log10(2.0));
  d_data_nbits = (unsigned long)ceil(log10(float(d_data_constellation.size())) / log10(2.0));

  // for offline analysis - apurv++ //
  d_fp_native_symbols = NULL;
  d_fp_coeff = NULL;
  d_init_log = false;
  d_num_ofdm_symbols = 0;
  d_ofdm_symbol_index = 0;
  // apurv++ end //

  /* for mimo_tx, rand_seed = 1, else rand_seed = d_id */
  if(d_mimo != 0) 
     srand(1);
  else
     srand(d_id);

  d_time_pkt_sent = 0;
  d_log_open = false;
  d_log_open_native = false;
  printf("NULL_SYMBOL_COUNT: %d\n", NULL_SYMBOL_COUNT); fflush(stdout);

  if(NUM_TRAINING_SYMBOLS > 0)
     generateKnownSymbols();
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
  }

  char *out_flag = 0;
  if(output_items.size() == 2)
    out_flag = (char *) output_items[1];
  
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
     if(d_fwd_index == 2 && (d_null_symbol_cnt < NULL_SYMBOL_COUNT) && 0) {
        d_null_symbol_cnt++;
	d_pending_flag = 3;
	if(!d_time_tag) {
	   make_time_tag(d_msg[0]);
	   d_time_tag = true;
	}
     }
     else
#endif
	/* data */
     if(d_msg_offset[0] < HEADERBYTELEN) {
	generateOFDMSymbol(out, HEADERBYTELEN);
        if(!d_time_tag) {
           make_time_tag(d_msg[0]);
           d_time_tag = true;
        }
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
     else if(d_fwd_index == 1 && (d_null_symbol_cnt < (NULL_SYMBOL_COUNT+NUM_TRAINING_SYMBOLS))) {	// lead_fwder: send NULL symbols to accomodate slave fwders
	d_null_symbol_cnt++;
     }
     else if(!d_send_null) {

	/* header has already been modulated, just send the payload *symbols* as it is */
 	copyOFDMSymbol(out, d_msg[0]->length());		
	logGeneratedTxSymbols(out);

	//logNativeTxSymbols(out);
	//printf("data--------- offset: %d\n", d_msg_offset[0]); fflush(stdout);
	d_data_ofdm_index+=1;

	/* if multiple forwarders (fwd_index>0) , then pilot subcarriers need to be shared */
	switch(d_fwd_index) {
#ifndef SRC_PILOT
	    case 0: tx_pilot = true; break;						// single fwder //
	    case 1: tx_pilot = (d_data_ofdm_index % 2 == 1)?true:false; break;		// lead: odd symbols //
	    case 2: tx_pilot = (d_data_ofdm_index % 2 == 0)?true:false; break;		// slave: even symbols //
#endif
	    default: break;
	}

#ifdef SRC_PILOT
	// fwd=1 disables sending of pilot //
 	if(d_fwd_index == 1) {
	    for(int i = 0; i < d_pilot_carriers.size(); i++) {
        	out[d_pilot_carriers[i]] = gr_complex(0.0, 0.0);
     	    }
	}
#endif
     }  
  }

#ifdef USE_PILOT
  if(tx_pilot) {
      double cur_pilot = 1.0;
      for(int i = 0; i < d_pilot_carriers.size(); i++) {
         out[d_pilot_carriers[i]] = gr_complex(cur_pilot, 0.0);
         cur_pilot = -cur_pilot;
      }
  }
#ifndef SRC_PILOT
  if(!tx_pilot) {
     for(int i = 0; i < d_pilot_carriers.size(); i++) {
	out[d_pilot_carriers[i]] = gr_complex(0.0, 0.0);
     }
  }
#endif
#endif
  
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

  if (out_flag)
    out_flag[0] = d_pending_flag;
  d_pending_flag = 0;

  return 1;  // produced symbol
}

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
	    printf("d_num_ofdm_symbols: %d\n", d_num_ofdm_symbols); fflush(stdout);
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

     d_time_tag = false;
     d_null_symbol_cnt = 0;
     d_hdr_ofdm_index = 0;
  }

  bool tx_pilot = false;

  // the final result will be in 'out' //
  gr_complex *out = (gr_complex *)output_items[0];
  memset(out, 0, sizeof(gr_complex) * d_fft_length);

#ifdef PRE_NULL_SYMBOLS
  if(d_null_symbol_cnt < NULL_SYMBOL_COUNT) {
     printf("(mapper): send NULL\n"); fflush(stdout);
     d_null_symbol_cnt++;
     d_pending_flag = 3;
     if(!d_time_tag) {
          make_time_tag1();
          d_time_tag = true;
     }
  }
  else
#endif
  if(d_hdr_byte_offset < HEADERBYTELEN) {
      if(!d_time_tag) {
           make_time_tag1();
           d_time_tag = true;
        }

      generateOFDMSymbolHeader(out); 					// send the header symbols out first //
#ifdef PRE_NULL_SYMBOLS
     if(d_hdr_ofdm_index == 0) {
        d_pending_flag = 1;
     }
     d_hdr_ofdm_index++;
#endif
      tx_pilot = true;
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

      normalizeSignal(out, d_batch_size);						// normalize the outgoing signal //
      logGeneratedTxSymbols(out); 

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
      d_ofdm_symbol_index = 0;
      d_send_null = false;
      d_modulated = true;
      if(!d_log_open_native) {
         logNativeTxSymbols();
         d_log_open_native = true;
      }
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
  else {
      for(int i = 0; i < d_pilot_carriers.size(); i++) {
	out[d_pilot_carriers[i]] = gr_complex(0.0, 0.0);
      }
  }
#endif
 
  char *out_flag = 0;
  if(output_items.size() == 2)
    out_flag = (char *) output_items[1];


  if (out_flag)
    out_flag[0] = d_pending_flag;

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
digital_ofdm_mapper_bcv::logGeneratedTxSymbols(gr_complex *out) 
{
  printf("digital_ofdm_mapper_bcv::logGeneratedTxSymbols\n"); fflush(stdout);
  if(!d_log_open) {
      char *filename = "tx_symbols.dat";
      int fd;
      if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename);
            assert(false);
      }
      else {
         if((d_fp_log = fdopen (fd, true ? "wb" : "w")) == NULL) {
              fprintf(stderr, "log file cannot be opened\n");
              close(fd);
              assert(false);
         }
      }
      d_log_open = true;
  }

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

  int count = fwrite_unlocked(log_symbols, sizeof(gr_complex), d_data_carriers.size(), d_fp_log);
  printf("count: %d written to tx_symbols.dat, total: %d \n", count, ftell(d_fp_log)); fflush(stdout);
  
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

#if 0
void 
digital_ofdm_mapper_bcv::generateCodeVector()
{
  // for each subcarrier, record 'd_batch_size' coeffs //
  int num_carriers = d_data_carriers.size()/COMPRESSION_FACTOR;

  float cv = 0.0;
  for(unsigned int k = 0; k < d_batch_size; k++) {
#if 0
      if(k > 0)
        cv += 90;
      else
        cv = rand() % 360 + 1;
      float amp = 1.0;
#else
      float cv = rand() % 360 + 1;                              // degree
      float LO = 0.5; float HI = 1.5;
      float amp = 1.0; //LO + (float)rand()/((float)RAND_MAX/(HI-LO));
#endif

      // store the scaled versions in header (space constraints) //
      d_header.coeffs[k].phase = cv * SCALE_FACTOR_PHASE;
      d_header.coeffs[k].amplitude = amp * SCALE_FACTOR_AMP;

#ifndef DEBUG
          float rad = cv * M_PI/180;
          gr_complex t_coeff = ToPhase_c(rad);
          printf("cv: %f degrees <-> %f radians <-> (%f, %f) \n", cv, rad, t_coeff.real(), t_coeff.imag()); fflush(stdout);
#endif
  }

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
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
      COEFF coeff = d_header.coeffs[batch_num];
#ifdef SCALE
      symbols[d_data_carriers[i]] *= (ToPhase_c(coeff) * gr_complex(SCALE));
#else
      symbols[d_data_carriers[i]] *= ToPhase_c(coeff);
#endif
      if(i == 0 && d_ofdm_symbol_index == 0)
         printf(" (%.8f, %.8f)\n", symbols[d_data_carriers[i]].real(), symbols[d_data_carriers[i]].imag()); fflush(stdout);
  }
}

inline void
digital_ofdm_mapper_bcv::normalizeSignal(gr_complex* out, int k)
{
  float factor = sqrt(k) * sqrt(d_header.nsenders);
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
     out[d_data_carriers[i]] /= gr_complex(factor);
  }
}

void
digital_ofdm_mapper_bcv::combineSignal(gr_complex *out, gr_complex* symbols)
{
  for(unsigned int i = 0; i < d_data_carriers.size(); i++) {
      /*
      if(i == 0 && d_ofdm_symbol_index == 0) {
         printf("(%.8f, %.8f) + (%.8f, %.8f) = ", out[d_data_carriers[i]].real(), out[d_data_carriers[i]].imag(), symbols[d_data_carriers[i]].real(), symbols[d_data_carriers[i]].imag()); fflush(stdout);
      }*/
     out[d_data_carriers[i]] += symbols[d_data_carriers[i]];
     //if(i == 0 && d_ofdm_symbol_index == 0)  printf(" (%.8f, %.8f)\n", out[d_data_carriers[i]].real(), out[d_data_carriers[i]].imag()); fflush(stdout);
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
   printf("makeHeader for batch: %d, pkt: %d, len: %d\n", d_batch_to_send, d_pkt_num, d_packetlen); fflush(stdout);
   memset(&d_header, 0, sizeof(d_header));
   d_header.dst_id = d_dst_id;
   d_header.flow_id = 0;
   d_header.inno_pkts = d_batch_size;

   d_header.lead_sender = (d_mimo == 2)?0:1;
   d_header.src_id = d_id;
   d_header.prev_hop_id = d_id;

   d_header.packetlen = d_packetlen - 1;		// -1 for '55' appended (ref ofdm_packet_utils)
   d_header.batch_number = d_batch_to_send;
   d_header.nsenders = (d_mimo == 0)?1:2;
   d_header.pkt_type = DATA_TYPE;

   d_header.factor = sqrt(d_header.nsenders);
  
   d_header.pkt_num = d_pkt_num++;
   d_header.link_id = 0;
 
   generateCodeVector();	// also fills up d_header.coeffs

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
 
  int num_ofdm_symbols_to_wait = 3000;

  int cp_length = d_fft_length/4;
  uint32_t num_samples = num_ofdm_symbols_to_wait * (d_fft_length+cp_length);
  double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
  double duration = num_samples * time_per_sample;

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

  int hdr_symbols = (HEADERBYTELEN * 8) / d_data_carriers.size();
  switch(d_mimo) {
    case 0: break;
    case 1:
            send_mimo_trigger(out_time);
            break;
    case 2:
            out_time = rcv_mimo_trigger();
	    printf("rx again: out_time (%llu, %f)\n", (uint64_t) out_time.get_full_secs(), out_time.get_frac_secs()); fflush(stdout); 
#ifndef PRE_NULL_SYMBOLS
            num_samples = (1+hdr_symbols+NUM_TRAINING_SYMBOLS) * (d_fft_length+cp_length);
            duration = num_samples * time_per_sample;
            sync_secs = (uint64_t) duration;
            sync_frac_of_secs = duration - (uint64_t) duration;
            out_time += uhd::time_spec_t(sync_secs, sync_frac_of_secs);
#endif
            break;
    default: assert(false);
  }

  printf("timestamp: curr (%llu, %f), out (%llu, %f) , last_time (%llu, %f), interval(%lld, %f), interval_samples(%llu), duration(%llu, %f)\n", (uint64_t) c_time.get_full_secs(), c_time.get_frac_secs(), (uint64_t) out_time.get_full_secs(), out_time.get_frac_secs(), (uint64_t) d_last_pkt_time.get_full_secs(), d_last_pkt_time.get_frac_secs(), (int64_t) interval_time.get_full_secs(), interval_time.get_frac_secs(), interval_samples, sync_secs, sync_frac_of_secs); fflush(stdout);

  d_last_pkt_time = out_time;

  const pmt::pmt_t key = pmt::pmt_string_to_symbol("tx_time");
  const pmt::pmt_t value = pmt::pmt_make_tuple(
		  pmt::pmt_from_uint64((uint64_t) out_time.get_full_secs()),
		  pmt::pmt_from_double(out_time.get_frac_secs())
		  );
  const pmt::pmt_t srcid = pmt::pmt_string_to_symbol(this->name());
  add_item_tag(1/*chan0*/, nitems_written(1), key, value, srcid);
  printf("(MAPPER) make_time_tag, at offset: %llu\n", nitems_written(1)); fflush(stdout);
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
}

inline void
digital_ofdm_mapper_bcv::open_mimo_master_socket() {
  int sock_port = 9001;
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
  listen(sockfd, 1);
  if(sockfd != -1) {
    printf("MIMO MASTER Socket OPEN success\n"); fflush(stdout);
  }
  else assert(false);

  struct sockaddr_in client_addr;
  int addrlen = sizeof(client_addr);
  while(1) {
    d_mimo_master_sock = accept(sockfd, (struct sockaddr*)&client_addr, (socklen_t*)&addrlen);
    if(d_mimo_master_sock != -1) {
      break;
    }
  }
}

inline int
digital_ofdm_mapper_bcv::open_mimo_slave_socket() {
  int sockfd;
  struct sockaddr_in dest;

  /* create socket */
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if(sockfd != -1) {
    printf("@ MIMO SLAVE socket OPEN success\n"); fflush(stdout);
  } else assert(false);

  /* initialize value in dest */
  bzero(&dest, sizeof(struct sockaddr_in));
  dest.sin_family = PF_INET;
  dest.sin_port = htons(9001);
  dest.sin_addr.s_addr = inet_addr("128.83.141.213");
  /* Connecting to server */
  connect(sockfd, (struct sockaddr*)&dest, sizeof(struct sockaddr_in));
  return sockfd;
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
         offset += nbytes;
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

#if 1 
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

bool
digital_ofdm_mapper_bcv::is_CV_good(gr_complex cv1, gr_complex cv2) {
   int M = d_data_constellation.size();
   float threshold = 0.7;

   float min_dt = 1000.0;
#if 0
   for(int m1 = 0; m1 < M; m1++) {
      gr_complex p1 = cv1 * d_data_constellation[m1];
      for(int m2 = 0; m2 < M; m2++) {
         gr_complex p2 = cv2 * d_data_constellation[m2];
         float dt = abs(p1 - p2);
         if(min_dt < dt) {
            min_dt = dt;
         }
         if(dt < threshold)
             return false;
      }
   }
#endif

   int n_entries = pow(double(d_data_constellation.size()), double(d_batch_size));
   int index = 0;
   gr_complex *comb_mod = (gr_complex*) malloc(sizeof(gr_complex) * n_entries);
   for(int m1 = 0; m1 < M; m1++) {
      gr_complex p1 = cv1 * d_data_constellation[m1];
      for(int m2 = 0; m2 < M; m2++) {
         gr_complex p2 = cv2 * d_data_constellation[m2];
         comb_mod[index++] = p1 + p2;
      }
   }
   assert(index == n_entries);

   for(int i = 0; i < n_entries; i++) {
      for(int j = i+1; j < n_entries; j++) {
         float dt = abs(comb_mod[i] - comb_mod[j]);
         if(dt < min_dt) {
            min_dt = dt;
         }
         if(min_dt < threshold)
            return false;
      }
   }

   printf("min_dt: %.3f\n", min_dt);
   return true;
}
