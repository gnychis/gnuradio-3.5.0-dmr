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

#ifndef INCLUDED_DIGITAL_OFDM_FRAME_SINK_H
#define INCLUDED_DIGITAL_OFDM_FRAME_SINK_H

#include <digital_api.h>
#include <gr_sync_block.h>
#include <gr_msg_queue.h>
#include <gr_message.h>
#include <cstdio>
#include "armadillo"

// logging headers //
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdexcept>
#include <stdio.h>
#include <deque>

#include <uhd/usrp/multi_usrp.hpp>

#define USE_PILOT 0
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

#define USE_ILP 0
#define MAX_SENDERS 4
//#define USE_HEADER_PLL 0

#define MAX_OFDM_SYMBOLS 170
#define MAX_OCCUPIED_CARRIERS 88
#define MAX_PKT_LEN 4096

//#define MIMO_RECEIVER 1

//#define MAX_OCCUPIED_CARRIERS 64
//#define MAX_DATA_CARRIERS 48

//#define H_PRECODING 1



// apurv for logging ends //

using namespace std;
using namespace arma;

class digital_ofdm_frame_sink;
typedef boost::shared_ptr<digital_ofdm_frame_sink> digital_ofdm_frame_sink_sptr;

DIGITAL_API digital_ofdm_frame_sink_sptr 
digital_make_ofdm_frame_sink (const std::vector<gr_complex> &hdr_sym_position, 
			 const std::vector<unsigned char> &hdr_sym_value_out,
                         const std::vector<gr_complex> &data_sym_position,
                         const std::vector<unsigned char> &data_sym_value_out,
			 const std::vector<std::vector<gr_complex> > &preamble,
			 gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
			 unsigned int occupied_tones, unsigned int fft_length,
			 float phase_gain=0.25, float freq_gain=0.25*0.25/4.0, unsigned int id=1, 
			 unsigned int batch_size=1, unsigned int decode_flag=1, 
			 int fwd_index=0, int replay_flag=0,
			 int exp_size=400, int fec_n=0, int fec_k=0, int degree=4, int h_coding=0);

typedef complex<double> comp_d;

/*
typedef struct composite_link_str {
  unsigned int n_prev_hops;
  unsigned int n_next_hops;
  unsigned char *prev_hops;
  unsigned char *next_hops;
  unsigned char flow_id;
} CompositeLink;
*/

/* encapsulates the sender details into it */
typedef struct innovative_pkt_str {
  unsigned int n_senders;
  vector<unsigned char> senders;
  vector<gr_complex*> coeffs;
  vector<gr_complex*> hestimates;		
  vector<float> phase_eq_vec;                   // PLL
  vector<float> freq_eq_vec;                    // PLL
  gr_complex* symbols;
  unsigned int prevLinkId;

  // *updated* after each header belonging to this inno pkt is decoded, one entry in the vector per header //
  vector<float*> error_rot_slope;
  vector<float*> error_rot_ref;
  vector<float*> error_amp_avg;

  vector<float> pll_slope_vec;
} PktInfo;

typedef vector<PktInfo*> InnovativePktInfoVector;

typedef struct flow_info_str {
  NodeId src;
  NodeId dst;
  unsigned char flowId;
  unsigned int active_batch;
  unsigned int last_batch_acked;
  
  InnovativePktInfoVector innovative_pkts;
  cx_mat coeff_mat;  					// only used for rank test, actual coefficients are in reduced_coeff
  vector<gr_complex*> reduced_coeffs;			// each row has (batch_size*occupied_carriers) entry, #rows = # in coeff_mat = # in innovative_pkts

  unsigned int pkts_fwded; 
  //vector<gr_complex *> innovative_pkts;
} FlowInfo;

typedef vector<FlowInfo*> FlowInfoVector;

#if 0
/* composite links */
typedef struct composite_link_str {
  unsigned char linkId;
  vector<unsigned int> srcIds;
  vector<unsigned int> dstIds;
} CompositeLink;
typedef vector<CompositeLink*> CompositeLinkVector;
#endif

/* credits */
typedef struct credit_str {
  unsigned char flowId;
  float credit;
  CompositeLink previousLink;
  CompositeLink nextLink;
} CreditInfo;
typedef vector<CreditInfo*> CreditInfoVector;

#if 0
//#ifdef H_PRECODING
/* for exchanging h-values over the ethernet */
// for recording the h-values between each pair of nodes. Every node maintains this info from 
// itself->upstream node, and sends this to all upstream nodes (only single hop upstream)
typedef struct h_info {
  unsigned int batch_num;
  float slope;
  gr_complex h_value;
} HInfo;

typedef pair<unsigned int, unsigned int> HKey;		// <from, to>
typedef map<HKey, HInfo*> HInfoMap;			// records between every pair of nodes

typedef struct eth_info {
  char addr[15];
  int port;
} EthInfo;

typedef map<unsigned char, EthInfo*> EthInfoMap;		// to store ethernet add

/* used by the co-ordinating transmitters to exchange coeff information */
typedef struct coeff_info {
  gr_complex coeffs[MAX_BATCH_SIZE];	// max batch size - these coeffs are reduced coeffs (2 for each rx) 
  unsigned int rx_id;
} CoeffInfo;

#endif

unsigned char random_mask_tuple[] = {
  255,  63,   0,  16,   0,  12,   0,   5, 192,   3,  16,   1, 204,   0,  85, 192,
   63,  16,  16,  12,  12,   5, 197, 195,  19,  17, 205, 204,  85, 149, 255,  47,
    0,  28,   0,   9, 192,   6, 208,   2, 220,   1, 153, 192, 106, 208,  47,  28,
   28,   9, 201, 198, 214, 210, 222, 221, 152,  89, 170, 186, 255,  51,   0,  21,
  192,  15,  16,   4,  12,   3,  69, 193, 243,  16,  69, 204,  51,  21, 213, 207,
   31,  20,   8,  15,  70, 132,  50, 227,  85, 137, 255,  38, 192,  26, 208,  11,
   28,   7,  73, 194, 182, 209, 182, 220, 118, 217, 230, 218, 202, 219,  23,  27,
   78, 139, 116, 103, 103, 106, 170, 175,  63,  60,  16,  17, 204,  12,  85, 197,
  255,  19,   0,  13, 192,   5, 144,   3,  44,   1, 221, 192,  89, 144,  58, 236,
   19,  13, 205, 197, 149, 147,  47,  45, 220,  29, 153, 201, 170, 214, 255,  30,
  192,   8,  80,   6, 188,   2, 241, 193, 132,  80,  99, 124,  41, 225, 222, 200,
   88,  86, 186, 190, 243,  48,  69, 212,  51,  31,  85, 200,  63,  22, 144,  14,
  236,   4,  77, 195, 117, 145, 231,  44,  74, 157, 247,  41, 134, 158, 226, 232,
   73, 142, 182, 228, 118, 203, 102, 215, 106, 222, 175,  24, 124,  10, 161, 199,
   56,  82, 146, 189, 173, 177, 189, 180, 113, 183, 100, 118, 171, 102, 255, 106,
  192,  47,  16,  28,  12,   9, 197, 198, 211,  18, 221, 205, 153, 149, 170, 239,
   63,  12,  16,   5, 204,   3,  21, 193, 207,  16,  84,  12,  63,  69, 208,  51,
   28,  21, 201, 207,  22, 212,  14, 223,  68,  88,  51, 122, 149, 227,  47,   9,
  220,   6, 217, 194, 218, 209, 155,  28, 107,  73, 239, 118, 204,  38, 213, 218,
  223,  27,  24,  11,  74, 135, 119,  34, 166, 153, 186, 234, 243,  15,   5, 196,
    3,  19,  65, 205, 240,  85, 132,  63,  35,  80,  25, 252,  10, 193, 199,  16,
   82, 140,  61, 165, 209, 187,  28, 115,  73, 229, 246, 203,   6, 215,  66, 222,
  177, 152, 116, 106, 167, 111,  58, 172,  19,  61, 205, 209, 149, 156, 111,  41,
  236,  30, 205, 200,  85, 150, 191,  46, 240,  28,  68,   9, 243,  70, 197, 242,
  211,   5, 157, 195,  41, 145, 222, 236,  88,  77, 250, 181, 131,  55,  33, 214,
  152,  94, 234, 184,  79,  50, 180,  21, 183,  79,  54, 180,  22, 247,  78, 198,
  180,  82, 247, 125, 134, 161, 162, 248, 121, 130, 162, 225, 185, 136, 114, 230,
  165, 138, 251,  39,   3,  90, 129, 251,  32,  67,  88,  49, 250, 148,  67,  47,
  113, 220,  36,  89, 219, 122, 219,  99,  27, 105, 203, 110, 215, 108,  94, 173,
  248, 125, 130, 161, 161, 184, 120, 114, 162, 165, 185, 187,  50, 243,  85, 133,
  255,  35,   0,  25, 192,  10, 208,   7,  28,   2, 137, 193, 166, 208, 122, 220,
   35,  25, 217, 202, 218, 215,  27,  30, 139,  72, 103, 118, 170, 166, 255,  58,
  192,  19,  16,  13, 204,   5, 149, 195,  47,  17, 220,  12,  89, 197, 250, 211,
    3,  29, 193, 201, 144,  86, 236,  62, 205, 208,  85, 156,  63,  41, 208,  30,
  220,   8,  89, 198, 186, 210, 243,  29, 133, 201, 163,  22, 249, 206, 194, 212,
   81, 159, 124, 104,  33, 238, 152,  76, 106, 181, 239,  55,  12,  22, 133, 206,
  227,  20,  73, 207, 118, 212,  38, 223,  90, 216,  59,  26, 147,  75,  45, 247,
   93, 134, 185, 162, 242, 249, 133, 130, 227,  33, 137, 216, 102, 218, 170, 219,
   63,  27,  80,  11, 124,   7,  97, 194, 168,  81, 190, 188, 112, 113, 228,  36,
   75,  91, 119, 123, 102, 163, 106, 249, 239,   2, 204,   1, 149, 192, 111,  16,
   44,  12,  29, 197, 201, 147,  22, 237, 206, 205, 148,  85, 175, 127,  60,  32,
   17, 216,  12,  90, 133, 251,  35,   3,  89, 193, 250, 208,  67,  28,  49, 201,
  212,  86, 223, 126, 216,  32,  90, 152,  59,  42, 147,  95,  45, 248,  29, 130,
  137, 161, 166, 248, 122, 194, 163,  17, 185, 204, 114, 213, 229, 159,  11,  40,
    7,  94, 130, 184,  97, 178, 168, 117, 190, 167,  48, 122, 148,  35,  47,  89,
  220,  58, 217, 211,  26, 221, 203,  25, 151,  74, 238, 183,  12, 118, 133, 230,
  227,  10, 201, 199,  22, 210, 142, 221, 164,  89, 187, 122, 243,  99,   5, 233,
  195,  14, 209, 196,  92,  83, 121, 253, 226, 193, 137, 144, 102, 236,  42, 205,
  223,  21, 152,  15,  42, 132,  31,  35,  72,  25, 246, 138, 198, 231,  18, 202,
  141, 151,  37, 174, 155,  60, 107,  81, 239, 124,  76,  33, 245, 216,  71,  26,
  178, 139,  53, 167,  87,  58, 190, 147,  48, 109, 212,  45, 159,  93, 168,  57,
  190, 146, 240, 109, 132,  45, 163,  93, 185, 249, 178, 194, 245, 145, 135,  44,
   98, 157, 233, 169, 142, 254, 228,  64,  75, 112,  55, 100,  22, 171,  78, 255,
  116,  64,  39, 112,  26, 164,  11,  59,  71,  83, 114, 189, 229, 177, 139,  52,
  103,  87, 106, 190, 175,  48, 124,  20,  33, 207,  88,  84,  58, 191,  83,  48,
61, 212,  17, 159,  76, 104,  53, 238, 151,  12, 110, 133, 236,  99,  13, 233,
  197, 142, 211,  36,  93, 219, 121, 155,  98, 235, 105, 143, 110, 228,  44,  75,
   93, 247, 121, 134, 162, 226, 249, 137, 130, 230, 225, 138, 200, 103,  22, 170,
  142, 255,  36,  64,  27, 112,  11, 100,   7, 107,  66, 175, 113, 188,  36, 113,
  219, 100,  91, 107, 123, 111,  99, 108,  41, 237, 222, 205, 152,  85, 170, 191,
   63,  48,  16,  20,  12,  15,  69, 196,  51,  19,  85, 205, 255,  21, 128,  15,
   32,   4,  24,   3,  74, 129, 247,  32,  70, 152,  50, 234, 149, 143,  47,  36,
   28,  27,  73, 203, 118, 215, 102, 222, 170, 216, 127,  26, 160,  11,  56,   7,
   82, 130, 189, 161, 177, 184, 116, 114, 167, 101, 186, 171,  51,  63,  85, 208,
   63,  28,  16,   9, 204,   6, 213, 194, 223,  17, 152,  12, 106, 133, 239,  35,
   12,  25, 197, 202, 211,  23,  29, 206, 137, 148, 102, 239, 106, 204,  47,  21,
  220,  15,  25, 196,  10, 211,  71,  29, 242, 137, 133, 166, 227,  58, 201, 211,
   22, 221, 206, 217, 148,  90, 239, 123,  12,  35,  69, 217, 243,  26, 197, 203,
   19,  23,  77, 206, 181, 148, 119,  47, 102, 156,  42, 233, 223,  14, 216,   4,
   90, 131, 123,  33, 227,  88,  73, 250, 182, 195,  54, 209, 214, 220,  94, 217,
  248,  90, 194, 187,  17, 179,  76, 117, 245, 231,   7,  10, 130, 135,  33, 162,
  152, 121, 170, 162, 255,  57, 128,  18, 224,  13, 136,   5, 166, 131,  58, 225,
  211,   8,  93, 198, 185, 146, 242, 237, 133, 141, 163,  37, 185, 219,  50, 219,
   85, 155, 127,  43,  96,  31, 104,   8,  46, 134, 156,  98, 233, 233, 142, 206,
  228,  84,  75, 127, 119,  96,  38, 168,  26, 254, 139,   0, 103,  64,  42, 176,
   31,  52,   8,  23,  70, 142, 178, 228, 117, 139, 103,  39, 106, 154, 175,  43,
   60,  31,  81, 200,  60,  86, 145, 254, 236,  64,  77, 240,  53, 132,  23,  35,
   78, 153, 244, 106, 199, 111,  18, 172,  13, 189, 197, 177, 147,  52, 109, 215,
  109, 158, 173, 168, 125, 190, 161, 176, 120, 116,  34, 167,  89, 186, 186, 243,
   51,   5, 213, 195,  31,  17, 200,  12,  86, 133, 254, 227,   0,  73, 192,  54,
  208,  22, 220,  14, 217, 196,  90, 211, 123,  29, 227,  73, 137, 246, 230, 198,
  202, 210, 215,  29, 158, 137, 168, 102, 254, 170, 192, 127,  16,  32,  12,  24,
    5, 202, 131,  23,  33, 206, 152,  84, 106, 191, 111,  48,  44,  20,  29, 207,
   73, 148,  54, 239,  86, 204,  62, 213, 208,  95,  28,  56,   9, 210, 134, 221,
  162, 217, 185, 154, 242, 235,   5, 143,  67,  36,  49, 219,  84,  91, 127, 123,
   96,  35, 104,  25, 238, 138, 204, 103,  21, 234, 143,  15,  36,   4,  27,  67,
   75, 113, 247, 100,  70, 171, 114, 255, 101, 128,  43,  32,  31,  88,   8,  58,
  134, 147,  34, 237, 217, 141, 154, 229, 171,  11,  63,  71,  80,  50, 188,  21,
  177, 207,  52,  84,  23, 127,  78, 160,  52, 120,  23,  98, 142, 169, 164, 126,
  251,  96,  67, 104,  49, 238, 148,  76, 111, 117, 236,  39,  13, 218, 133, 155,
   35,  43,  89, 223, 122, 216,  35,  26, 153, 203,  42, 215,  95,  30, 184,   8,
  114, 134, 165, 162, 251,  57, 131,  82, 225, 253, 136,  65, 166, 176, 122, 244,
   35,   7,  89, 194, 186, 209, 179,  28, 117, 201, 231,  22, 202, 142, 215,  36,
   94, 155, 120, 107,  98, 175, 105, 188,  46, 241, 220,  68,  89, 243, 122, 197,
  227,  19,   9, 205, 198, 213, 146, 223,  45, 152,  29, 170, 137, 191,  38, 240,
   26, 196,  11,  19,  71,  77, 242, 181, 133, 183,  35,  54, 153, 214, 234, 222,
  207,  24,  84,  10, 191,  71,  48,  50, 148,  21, 175,  79,  60,  52,  17, 215,
   76,  94, 181, 248, 119,   2, 166, 129, 186, 224, 115,   8,  37, 198, 155,  18,
  235,  77, 143, 117, 164,  39,  59,  90, 147, 123,  45, 227,  93, 137, 249, 166,
  194, 250, 209, 131,  28,  97, 201, 232,  86, 206, 190, 212, 112,  95, 100,  56,
   43,  82, 159, 125, 168,  33, 190, 152, 112, 106, 164,  47,  59,  92,  19, 121,
  205, 226, 213, 137, 159,  38, 232,  26, 206, 139,  20, 103,  79, 106, 180,  47,
   55,  92,  22, 185, 206, 242, 212,  69, 159, 115,  40,  37, 222, 155,  24, 107,
   74, 175, 119,  60,  38, 145, 218, 236,  91,  13, 251,  69, 131, 115,  33, 229,
  216,  75,  26, 183,  75,  54, 183,  86, 246, 190, 198, 240,  82, 196,  61, 147,
   81, 173, 252, 125, 129, 225, 160,  72, 120,  54, 162, 150, 249, 174, 194, 252,
   81, 129, 252,  96,  65, 232,  48,  78, 148,  52, 111,  87, 108,  62, 173, 208,
  125, 156,  33, 169, 216, 126, 218, 160,  91,  56,  59,  82, 147, 125, 173, 225,
  189, 136, 113, 166, 164, 122, 251,  99,   3, 105, 193, 238, 208,  76,  92,  53,
  249, 215,   2, 222, 129, 152,  96, 106, 168,  47,  62, 156,  16, 105, 204,  46,
  213, 220,  95,  25, 248,  10, 194, 135,  17, 162, 140, 121, 165, 226, 251,   9,
131,  70, 225, 242, 200,  69, 150, 179,  46, 245, 220,  71,  25, 242, 138, 197,
  167,  19,  58, 141, 211,  37, 157, 219,  41, 155,  94, 235, 120,  79,  98, 180,
   41, 183,  94, 246, 184,  70, 242, 178, 197, 181, 147,  55,  45, 214, 157, 158,
  233, 168,  78, 254, 180,  64, 119, 112,  38, 164,  26, 251,  75,   3, 119,  65,
  230, 176,  74, 244,  55,   7,  86, 130, 190, 225, 176,  72, 116,  54, 167,  86,
  250, 190, 195,  48,  81, 212,  60,  95,  81, 248,  60,  66, 145, 241, 172,  68,
  125, 243,  97, 133, 232,  99,  14, 169, 196, 126, 211,  96,  93, 232,  57, 142,
  146, 228, 109, 139, 109, 167, 109, 186, 173, 179,  61, 181, 209, 183,  28, 118,
  137, 230, 230, 202, 202, 215,  23,  30, 142, 136, 100, 102, 171, 106, 255, 111,
    0,  44,   0,  29, 192,   9, 144,   6, 236,   2, 205, 193, 149, 144, 111,  44,
   44,  29, 221, 201, 153, 150, 234, 238, 207,  12,  84,   5, 255,  67,   0,  49,
  192,  20,  80,  15, 124,   4,  33, 195,  88,  81, 250, 188,  67,  49, 241, 212,
   68,  95, 115, 120,  37, 226, 155,   9, 171,  70, 255, 114, 192,  37, 144,  27,
   44,  11,  93, 199, 121, 146, 162, 237, 185, 141, 178, 229, 181, 139,  55,  39,
   86, 154, 190, 235,  48,  79,  84,  52,  63,  87,  80,  62, 188,  16, 113, 204,
   36,  85, 219, 127,  27,  96,  11, 104,   7, 110, 130, 172,  97, 189, 232, 113,
  142, 164, 100, 123, 107,  99, 111, 105, 236,  46, 205, 220,  85, 153, 255,  42,
  192,  31,  16,   8,  12,   6, 133, 194, 227,  17, 137, 204, 102, 213, 234, 223,
   15,  24,   4,  10, 131,  71,  33, 242, 152,  69, 170, 179,  63,  53, 208,  23,
   28,  14, 137, 196, 102, 211, 106, 221, 239,  25, 140,  10, 229, 199,  11,  18,
  135,  77, 162, 181, 185, 183,  50, 246, 149, 134, 239,  34, 204,  25, 149, 202,
  239,  23,  12,  14, 133, 196,  99,  19, 105, 205, 238, 213, 140,  95,  37, 248,
   27,   2, 139,  65, 167, 112, 122, 164,  35,  59,  89, 211, 122, 221, 227,  25,
  137, 202, 230, 215,  10, 222, 135,  24,  98, 138, 169, 167,  62, 250, 144,  67,
   44,  49, 221, 212,  89, 159, 122, 232,  35,  14, 153, 196, 106, 211, 111,  29,
  236,   9, 141, 198, 229, 146, 203,  45, 151,  93, 174, 185, 188, 114, 241, 229,
  132,  75,  35, 119,  89, 230, 186, 202, 243,  23,   5, 206, 131,  20,  97, 207,
  104,  84,  46, 191,  92, 112,  57, 228,  18, 203,  77, 151, 117, 174, 167,  60,
  122, 145, 227,  44,  73, 221, 246, 217, 134, 218, 226, 219,   9, 155,  70, 235,
  114, 207, 101, 148,  43,  47,  95,  92,  56,  57, 210, 146, 221, 173, 153, 189,
  170, 241, 191,   4, 112,   3, 100,   1, 235,  64,  79, 112,  52,  36,  23,  91,
   78, 187, 116, 115, 103, 101, 234, 171,  15,  63,  68,  16,  51,  76,  21, 245,
  207,   7,  20,   2, 143,  65, 164,  48, 123,  84,  35, 127,  89, 224,  58, 200,
   19,  22, 141, 206, 229, 148,  75,  47, 119,  92,  38, 185, 218, 242, 219,   5,
  155,  67,  43, 113, 223, 100,  88,  43, 122, 159,  99,  40,  41, 222, 158, 216,
  104,  90, 174, 187,  60, 115,  81, 229, 252,  75,   1, 247,  64,  70, 176,  50,
  244,  21, 135,  79,  34, 180,  25, 183,  74, 246, 183,   6, 246, 130, 198, 225,
  146, 200, 109, 150, 173, 174, 253, 188,  65, 177, 240, 116,  68,  39, 115,  90,
  165, 251,  59,   3,  83,  65, 253, 240,  65, 132,  48,  99,  84,  41, 255,  94,
  192,  56,  80,  18, 188,  13, 177, 197, 180,  83,  55, 125, 214, 161, 158, 248,
  104,  66, 174, 177, 188, 116, 113, 231, 100,  74, 171, 119,  63, 102, 144,  42,
  236,  31,  13, 200,   5, 150, 131,  46, 225, 220,  72,  89, 246, 186, 198, 243,
   18, 197, 205, 147,  21, 173, 207,  61, 148,  17, 175,  76, 124,  53, 225, 215,
    8,  94, 134, 184,  98, 242, 169, 133, 190, 227,  48,  73, 212,  54, 223,  86,
  216,  62, 218, 144,  91,  44,  59,  93, 211, 121, 157, 226, 233, 137, 142, 230,
  228,  74, 203, 119,  23, 102, 142, 170, 228, 127,  11,  96,   7, 104,   2, 174,
  129, 188,  96, 113, 232,  36,  78, 155, 116, 107, 103, 111, 106, 172,  47,  61,
  220,  17, 153, 204, 106, 213, 239,  31,  12,   8,   5, 198, 131,  18, 225, 205,
  136,  85, 166, 191,  58, 240,  19,   4,  13, 195,  69, 145, 243,  44,  69, 221,
  243,  25, 133, 202, 227,  23,   9, 206, 134, 212,  98, 223, 105, 152,  46, 234,
  156,  79,  41, 244,  30, 199,  72,  82, 182, 189, 182, 241, 182, 196, 118, 211,
  102, 221, 234, 217, 143,  26, 228,  11,  11,  71,  71, 114, 178, 165, 181, 187,
   55,  51,  86, 149, 254, 239,   0,  76,   0,  53, 192,  23,  16,  14, 140,   4,
  101, 195, 107,  17, 239,  76,  76,  53, 245, 215,   7,  30, 130, 136,  97, 166,
  168, 122, 254, 163,   0, 121, 192,  34, 208,  25, 156,  10, 233, 199,  14, 210,
  132,  93, 163, 121, 185, 226, 242, 201, 133, 150, 227,  46, 201, 220,  86, 217,
254, 218, 192,  91,  16,  59,  76,  19, 117, 205, 231,  21, 138, 143,  39,  36,
   26, 155,  75,  43, 119,  95, 102, 184,  42, 242, 159,   5, 168,   3,  62, 129,
  208,  96,  92,  40,  57, 222, 146, 216, 109, 154, 173, 171,  61, 191,  81, 176,
   60, 116,  17, 231,  76,  74, 181, 247,  55,   6, 150, 130, 238, 225, 140,  72,
  101, 246, 171,   6, 255,  66, 192,  49, 144,  20, 108,  15, 109, 196,  45, 147,
   93, 173, 249, 189, 130, 241, 161, 132, 120,  99,  98, 169, 233, 190, 206, 240,
   84,  68,  63, 115,  80,  37, 252,  27,   1, 203,  64,  87, 112,  62, 164,  16,
  123,  76,  35, 117, 217, 231,  26, 202, 139,  23,  39,  78, 154, 180, 107,  55,
  111,  86, 172,  62, 253, 208,  65, 156,  48, 105, 212,  46, 223,  92,  88,  57,
  250, 146, 195,  45, 145, 221, 172,  89, 189, 250, 241, 131,   4,  97, 195, 104,
   81, 238, 188,  76, 113, 245, 228,  71,  11, 114, 135, 101, 162, 171,  57, 191,
   82, 240,  61, 132,  17, 163,  76, 121, 245, 226, 199,   9, 146, 134, 237, 162,
  205, 185, 149, 178, 239,  53, 140,  23,  37, 206, 155,  20, 107,  79, 111, 116,
   44,  39,  93, 218, 185, 155,  50, 235,  85, 143, 127,  36,  32,  27,  88,  11,
  122, 135,  99,  34, 169, 217, 190, 218, 240,  91,   4,  59,  67,  83, 113, 253,
  228,  65, 139, 112, 103, 100,  42, 171,  95,  63, 120,  16,  34, 140,  25, 165,
  202, 251,  23,   3,  78, 129, 244,  96,  71, 104,  50, 174, 149, 188, 111,  49,
  236,  20,  77, 207, 117, 148,  39,  47,  90, 156,  59,  41, 211,  94, 221, 248,
   89, 130, 186, 225, 179,   8, 117, 198, 167,  18, 250, 141, 131,  37, 161, 219,
   56,  91,  82, 187, 125, 179,  97, 181, 232, 119,  14, 166, 132, 122, 227,  99,
    9, 233, 198, 206, 210, 212,  93, 159, 121, 168,  34, 254, 153, 128, 106, 224,
   47,   8,  28,   6, 137, 194, 230, 209, 138, 220, 103,  25, 234, 138, 207,  39,
   20,  26, 143,  75,  36,  55,  91,  86, 187, 126, 243,  96,  69, 232,  51,  14,
  149, 196, 111,  19, 108,  13, 237, 197, 141, 147,  37, 173, 219,  61, 155,  81,
  171, 124, 127,  97, 224,  40,  72,  30, 182, 136, 118, 230, 166, 202, 250, 215,
    3,  30, 129, 200,  96,  86, 168,  62, 254, 144,  64, 108,  48,  45, 212,  29,
  159,  73, 168,  54, 254, 150, 192, 110, 208,  44,  92,  29, 249, 201, 130, 214,
  225, 158, 200, 104,  86, 174, 190, 252, 112,  65, 228,  48,  75,  84,  55, 127,
   86, 160,  62, 248,  16,  66, 140,  49, 165, 212, 123,  31,  99,  72,  41, 246,
  158, 198, 232,  82, 206, 189, 148, 113, 175, 100, 124,  43,  97, 223, 104,  88,
   46, 186, 156, 115,  41, 229, 222, 203,  24,  87,  74, 190, 183,  48, 118, 148,
   38, 239,  90, 204,  59,  21, 211,  79,  29, 244,   9, 135,  70, 226, 178, 201,
  181, 150, 247,  46, 198, 156,  82, 233, 253, 142, 193, 164,  80, 123, 124,  35,
   97, 217, 232,  90, 206, 187,  20, 115,  79, 101, 244,  43,   7,  95,  66, 184,
   49, 178, 148, 117, 175, 103,  60,  42, 145, 223,  44,  88,  29, 250, 137, 131,
   38, 225, 218, 200,  91,  22, 187,  78, 243, 116,  69, 231, 115,  10, 165, 199,
   59,  18, 147,  77, 173, 245, 189, 135,  49, 162, 148, 121, 175,  98, 252,  41,
  129, 222, 224,  88,  72,  58, 182, 147,  54, 237, 214, 205, 158, 213, 168,  95,
   62, 184,  16, 114, 140,  37, 165, 219,  59,  27,  83,  75, 125, 247,  97, 134,
  168,  98, 254, 169, 128, 126, 224,  32,  72,  24,  54, 138, 150, 231,  46, 202,
  156,  87,  41, 254, 158, 192, 104,  80,  46, 188,  28, 113, 201, 228,  86, 203,
  126, 215,  96,  94, 168,  56, 126, 146, 160, 109, 184,  45, 178, 157, 181, 169,
  183,  62, 246, 144,  70, 236,  50, 205, 213, 149, 159,  47,  40,  28,  30, 137,
  200, 102, 214, 170, 222, 255,  24,  64,  10, 176,   7,  52,   2, 151,  65, 174,
  176, 124, 116,  33, 231,  88,  74, 186, 183,  51,  54, 149, 214, 239,  30, 204,
    8,  85, 198, 191,  18, 240,  13, 132,   5, 163,  67,  57, 241, 210, 196,  93,
  147, 121, 173, 226, 253, 137, 129, 166, 224, 122, 200,  35,  22, 153, 206, 234,
  212,  79,  31, 116,   8,  39,  70, 154, 178, 235,  53, 143,  87,  36,  62, 155,
   80, 107, 124,  47,  97, 220,  40,  89, 222, 186, 216, 115,  26, 165, 203,  59,
   23,  83,  78, 189, 244, 113, 135, 100,  98, 171, 105, 191, 110, 240,  44,  68,
   29, 243,  73, 133, 246, 227,   6, 201, 194, 214, 209, 158, 220, 104,  89, 238,
  186, 204, 115,  21, 229, 207,  11,  20,   7,  79,  66, 180,  49, 183,  84, 118,
  191, 102, 240,  42, 196,  31,  19,  72,  13, 246, 133, 134, 227,  34, 201, 217,
  150, 218, 238, 219,  12,  91,  69, 251, 115,   3, 101, 193, 235,  16,  79,  76,
   52,  53, 215,  87,  30, 190, 136, 112, 102, 164,  42, 251,  95,   3, 120,   1,
  226, 128,  73, 160,  54, 248,  22, 194, 142, 209, 164,  92, 123, 121, 227,  98,
201, 233, 150, 206, 238, 212,  76,  95, 117, 248,  39,   2, 154, 129, 171,  32,
  127,  88,  32,  58, 152,  19,  42, 141, 223,  37, 152,  27,  42, 139,  95,  39,
  120,  26, 162, 139,  57, 167,  82, 250, 189, 131,  49, 161, 212, 120,  95,  98,
  184,  41, 178, 158, 245, 168,  71,  62, 178, 144, 117, 172,  39,  61, 218, 145,
  155,  44, 107,  93, 239, 121, 140,  34, 229, 217, 139,  26, 231,  75,  10, 183,
   71,  54, 178, 150, 245, 174, 199,  60,  82, 145, 253, 172,  65, 189, 240, 113,
  132,  36,  99,  91, 105, 251, 110, 195, 108,  81, 237, 252,  77, 129, 245, 160,
   71,  56,  50, 146, 149, 173, 175,  61, 188,  17, 177, 204, 116,  85, 231, 127,
   10, 160,   7,  56,   2, 146, 129, 173, 160, 125, 184,  33, 178, 152, 117, 170,
  167,  63,  58, 144,  19,  44,  13, 221, 197, 153, 147,  42, 237, 223,  13, 152,
    5, 170, 131,  63,  33, 208,  24,  92,  10, 185, 199,  50, 210, 149, 157, 175,
   41, 188,  30, 241, 200,  68,  86, 179, 126, 245, 224,  71,   8,  50, 134, 149,
  162, 239,  57, 140,  18, 229, 205, 139,  21, 167,  79,  58, 180,  19,  55,  77,
  214, 181, 158, 247,  40,  70, 158, 178, 232, 117, 142, 167,  36, 122, 155,  99,
   43, 105, 223, 110, 216,  44,  90, 157, 251,  41, 131,  94, 225, 248,  72,  66,
  182, 177, 182, 244, 118, 199, 102, 210, 170, 221, 191,  25, 176,  10, 244,   7,
    7,  66, 130, 177, 161, 180, 120, 119,  98, 166, 169, 186, 254, 243,   0,  69,
  192,  51,  16,  21, 204,  15,  21, 196,  15,  19,  68,  13, 243,  69, 133, 243,
   35,   5, 217, 195,  26, 209, 203,  28,  87,  73, 254, 182, 192, 118, 208,  38,
  220,  26, 217, 203,  26, 215,  75,  30, 183,  72, 118, 182, 166, 246, 250, 198,
  195,  18, 209, 205, 156,  85, 169, 255,  62, 192,  16,  80,  12,  60,   5, 209,
  195,  28,  81, 201, 252,  86, 193, 254, 208,  64,  92,  48,  57, 212,  18, 223,
   77, 152,  53, 170, 151,  63,  46, 144,  28, 108,   9, 237, 198, 205, 146, 213,
  173, 159,  61, 168,  17, 190, 140, 112, 101, 228,  43,  11,  95,  71, 120,  50,
  162, 149, 185, 175,  50, 252,  21, 129, 207,  32,  84,  24,  63,  74, 144,  55,
   44,  22, 157, 206, 233, 148,  78, 239, 116,  76,  39, 117, 218, 167,  27,  58,
  139,  83,  39, 125, 218, 161, 155,  56, 107,  82, 175, 125, 188,  33, 177, 216,
  116,  90, 167, 123,  58, 163,  83,  57, 253, 210, 193, 157, 144, 105, 172,  46,
  253, 220,  65, 153, 240, 106, 196,  47,  19,  92,  13, 249, 197, 130, 211,  33,
  157, 216, 105, 154, 174, 235,  60,  79,  81, 244,  60,  71,  81, 242, 188,  69,
  177, 243,  52,  69, 215, 115,  30, 165, 200, 123,  22, 163,  78, 249, 244,  66,
  199, 113, 146, 164, 109, 187, 109, 179, 109, 181, 237, 183,  13, 182, 133, 182,
  227,  54, 201, 214, 214, 222, 222, 216,  88,  90, 186, 187,  51,  51, 255,  63
};

/*!
 * \brief Takes an OFDM symbol in, demaps it into bits of 0's and 1's, packs
 * them into packets, and sends to to a message queue sink.
 * \ingroup sink_blk
 * \ingroup ofdm_blk
 *
 * NOTE: The mod input parameter simply chooses a pre-defined demapper/slicer. Eventually,
 * we want to be able to pass in a reference to an object to do the demapping and slicing
 * for a given modulation type.
 */
class DIGITAL_API digital_ofdm_frame_sink : public gr_sync_block
{
  friend DIGITAL_API digital_ofdm_frame_sink_sptr 
  digital_make_ofdm_frame_sink (const std::vector<gr_complex> &hdr_sym_position, 
			   const std::vector<unsigned char> &hdr_sym_value_out,
                           const std::vector<gr_complex> &data_sym_position,
                           const std::vector<unsigned char> &data_sym_value_out,
			   const std::vector<std::vector<gr_complex> > &preamble,
			   gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
			   unsigned int occupied_tones, unsigned int fft_length,
			   float phase_gain, float freq_gain, unsigned int id, 
			   unsigned int batch_size, unsigned int decode_flag,
			   int fwd_index, int replay_flag,
			   int exp_size, int fec_n, int fec_k, int degree, int h_coding);

 private:
  enum state_t {STATE_SYNC_SEARCH, STATE_HAVE_SYNC, STATE_HAVE_NULL, STATE_HAVE_TRAINING, STATE_HAVE_HEADER};

  //static const int MAX_PKT_LEN    = 4096;

  gr_msg_queue_sptr  d_target_queue;		// where to send the packet when received
  state_t            d_state;
  //unsigned int       d_header;			// header bits
  int		     d_headerbytelen_cnt;	// how many so far

  unsigned char      *d_bytes_out;              // hold the current bytes produced by the demapper    

  unsigned int       d_occupied_carriers;
  unsigned int       d_byte_offset;
  unsigned int       d_partial_byte;

  unsigned char      d_packet[MAX_PKT_LEN];	// assembled payload
  int 		     d_packetlen;		// length of packet
  int                d_packet_whitener_offset;  // offset into whitener string to use
  int		     d_packetlen_cnt;		// how many so far

  gr_complex * d_derotated_output;  // Pointer to output stream to send deroated symbols out

  // modulation parameters //
  std::vector<gr_complex>    d_hdr_sym_position;
  std::vector<unsigned char> d_hdr_sym_value_out;
  unsigned int d_hdr_nbits;

  std::vector<gr_complex>    d_data_sym_position;
  std::vector<unsigned char> d_data_sym_value_out;
  unsigned int d_data_nbits;

  std::vector<gr_complex>    d_dfe;


  unsigned char d_resid;
  unsigned int d_nresid;

  float d_phase[MAX_SENDERS];
  float d_freq[MAX_SENDERS];			// indexed by the number of concurrent senders (for individual phase tracking)
  float d_phase1[MAX_SENDERS];
  float d_freq1[MAX_SENDERS];

  float d_phase_gain;
  float d_freq_gain;
  float d_eq_gain;

  std::vector<int> d_data_carriers;
  std::vector<int> d_pilot_carriers;
  std::vector<int> d_all_carriers;			// tracks which are data(0), pilot(1) or dc(2) 

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
		     int exp_size, int fec_n, int fec_k, int degree, int h_coding);

  void enter_search();
  void enter_have_sync();
  void enter_have_header();
  
  bool header_ok();
  
  unsigned char slicer_hdr(const gr_complex x);
  unsigned int demapper(const gr_complex *in,
			unsigned char *out);

  bool set_hdr_sym_value_out(const std::vector<gr_complex> &sym_position, 
			 const std::vector<unsigned char> &sym_value_out);
  bool set_data_sym_value_out(const std::vector<gr_complex> &sym_position,
                         const std::vector<unsigned char> &sym_value_out);

 public:
  ~digital_ofdm_frame_sink();

  int work(int noutput_items,
	   gr_vector_const_void_star &input_items,
	   gr_vector_void_star &output_items);

 private:
  /* data */
  static const int HEADERBYTELEN = sizeof(MULTIHOP_HDR_TYPE);
  static const int HEADERDATALEN = HEADERBYTELEN - PADDING_SIZE - 4;

  /* ack */
  static const int ACK_HEADERBYTELEN = sizeof(MULTIHOP_ACK_HDR_TYPE);
  static const int ACK_HEADERDATALEN = ACK_HEADERBYTELEN - ACK_PADDING_SIZE-4;

 public:
  /* apurv start */
  MULTIHOP_HDR_TYPE d_header;
  bool d_save_flag;
  unsigned int d_save_pkt_num;
  unsigned int d_curr_ofdm_symbol_index;
  unsigned int d_num_ofdm_symbols;
  unsigned int d_num_hdr_ofdm_symbols;

  gr_complex *d_curr_rx_symbols;
  unsigned int d_batch_size;
  unsigned int d_decode_flag;
  int d_active_batch;
  int d_last_batch_acked;
  
  unsigned int d_hdr_byte_offset;
  unsigned char d_header_bytes[HEADERBYTELEN]; 
 
  unsigned int d_batch_number;
  unsigned int d_nsenders;
  unsigned char d_lead_sender;
  unsigned int d_pkt_num;

  NodeId d_id;
  NodeId d_dst_id;
  NodeId d_src_id;
  NodeId d_prev_hop_id;

  //unsigned char d_flow; 
  int d_flow;
  unsigned int d_pkt_type;
  unsigned int d_prevLinkId;

  gr_complex *d_in_estimates;		// will be updated only if preamble is not detected in between a pkt! //

  /* fwd/dst identification */
  bool d_fwd, d_dst;
  bool shouldProcess();  
  bool isMyPacket();

  /* internal store, stores the innovative packets seen for this flow */
  FlowInfoVector d_flowInfoVector;
  PktInfo *d_pktInfo;			     // current pkt being serviced 
  gr_msg_queue_sptr  d_out_queue;            // contains modulated packets to be forwarded/ACKs
  FlowInfo* getFlowInfo(bool create, unsigned char flowId);
  void resetPktInfo(PktInfo *pktInfo);
  PktInfo* createPktInfo();

  /* encoding the signal */
  void encodeSignal(gr_complex *symbols, gr_complex coeff);
  void combineSignal(gr_complex *out, gr_complex* symbols, int);
  float normalizeSignal(gr_complex* out, int k, int num_in_senders);
  void generateCodeVector(MULTIHOP_HDR_TYPE &header);

  /* make header and packet */
  void makeHeader(MULTIHOP_HDR_TYPE &header, unsigned char *header_bytes, FlowInfo *flowInfo, unsigned int nextLinkId, float timing_offset=0.0);

  /* decoding/demodulating */
  void demodulate(vector<gr_complex*> out_sym_vec);
  void decodePayload_single(FlowInfo *flowInfo);
  void decodePayload_multiple(FlowInfo *flowInfo); 

  /* linear system solving */
  void verifySolution(cx_mat TX, cx_mat RX, cx_mat coeff_mat);
  void saveTxMatrix(cx_mat TX, unsigned int ofdm_symbol_index, unsigned int subcarrier_index, vector<gr_complex*> out_sym_vec);
  void updateCoeffMatrix(FlowInfo *flowInfo, unsigned int subcarrier_index, cx_mat &coeff_mat);
  void loadRxMatrix(cx_mat &RX, unsigned int ofdm_symbol_index, unsigned int subcarrier_index, FlowInfo *flowInfo);
  void storePayload(gr_complex *in, gr_complex *in_sampler); 
  bool isFullRank(FlowInfo *flowInfo);
  bool isInnovative();
  /** multiple senders **/
  unsigned int d_pending_senders;
  void save_coefficients();
  bool isInnovativeAfterReduction();
 
  /* etc */
  void equalizeSymbols(gr_complex *in, gr_complex *in_estimates);
  void extract_header(gr_complex);
  void prepareForNewBatch();
  void debugPktInfo(PktInfo *pktInfo, unsigned char senderId);
  bool isSameNodeList(vector<unsigned char> ids1, vector<unsigned char> ids2);
  void dewhiten(unsigned char *bytes, const int len);
  void whiten(unsigned char *bytes, const int len);
  void debugHeader(unsigned char *header_bytes);
  gr_complex ToPhase_c(float phase_deg);
  float ToPhase_f(gr_complex phase_c);
  float ToPhase_f(COEFF coeff);
  gr_complex ToPhase_c(COEFF coeff);
 
  void decodeSignal(gr_complex* symbols, gr_complex coeff);

  /* for offline analysis/logging */
  bool d_file_opened;
  FILE *d_fp_rx_symbols, *d_fp_solved, *d_fp_coeff, *d_fp_hestimates;
  void log(FlowInfo *flowInfo, vector<gr_complex*> out_sym_vec);
  bool open_log();  
  bool open_hestimates_log();

  /* offline analysis/stitching to test time domain combination of signals */
  int d_fft_length;
  /* apurv end */

  /* fwder operations */
  void makePacket(bool sync_send=false); 		
  bool isLeadSender();
  void encodePktToFwd(CreditInfo *creditInfo, bool sync_send=false);
  

  /* credits */
  unsigned int num_data_fwd;
  CompositeLinkVector d_compositeLinkVector;
  CreditInfoVector d_creditInfoVector;								// per flow
  CompositeLink* getCompositeLink(int id);
  CreditInfo* findCreditInfo(unsigned char flowId);
  void populateCompositeLinkInfo();
  void populateCreditInfo();
  void updateCredit();
  vector<int> d_outCLinks, d_inCLinks;

  /* ACKs */
  unsigned int num_acks_sent;
  MULTIHOP_ACK_HDR_TYPE d_ack_header;
  deque<gr_message_sptr> d_ack_queue;
  map<unsigned char, unsigned char> d_ack_route_map;						// flow, prev_hop
  void processACK();
  void sendACK(unsigned char flow, unsigned char batch);  
  void forwardACK();
  bool shouldFowardACK(unsigned char flowId, unsigned char prevHop);
  void extract_ack_header();
  bool ack_header_ok();
  void fillAckRouteMap();


  /* whatever */
  FILE *d_fp_symbols_test;
  bool openLogSymbols();
  void logSymbols(gr_complex *symbols, int num);
  void debugFlowInfo(FlowInfo* flowInfo);
  void printSymbols(gr_complex *symbols, int num);
  void reset_demapper();


  FILE *d_fp_demod, *d_fp_demod_symbols;
  bool d_demod_log_file;
  void log_demod(unsigned char *bytes, int num_bytes, gr_complex *symbols, int num_symbols);


  /* ILP */
  void slicer_ILP(gr_complex *x, gr_complex *closest_sym, unsigned char *bits, vector<gr_complex*> batched_sym_position,
		  unsigned int ofdm_symbol_index, unsigned int subcarrier_index, gr_complex** sym_vec);
  void getSymOutBits_ILP(unsigned char *bits, int index);
  void getSymOutBits_ILP_QPSK(unsigned char *bits, int index);
  void getSymOutBits_ILP_QAM16(unsigned char *bits, int index);

  void demodulate_ILP(FlowInfo *flowInfo);
  unsigned int demapper_ILP(unsigned int ofdm_symbol_index, vector<unsigned char*> out_vec, 
			    vector<vector<gr_complex*> > batched_sym_position, FlowInfo *flowInfo, vector<gr_complex> *dfe_vec);
  void buildMap_ILP(cx_mat coeff, vector<gr_complex*> &batched_sym_position);
  void debugMap_ILP(vector<vector<gr_complex*> > batched_sym_position, int pkts);

  void slicer_ILP_opt(gr_complex *x, gr_complex *closest_sym, unsigned char *bits, vector<gr_complex*> batched_sym_position, 
                  unsigned int ofdm_index, unsigned int subcarrier_index, gr_complex** sym_vec);
  void getCandidateSymbols(gr_complex *x, vector<gr_complex*> batched_sym_position, vector<int> &filtered_batches);

  /* pilots */
  void assign_subcarriers();
  void fill_all_carriers_map();
  void equalize_interpolate_dfe(const gr_complex *in, gr_complex *out);

  /* fwd packet error check */
  bool checkPacketError(FlowInfo *flow_info);
  bool isFECGroupCorrect(int mod_symbols_group, gr_complex *rx_symbols, vector<comp_d> ideal_pos_map);
  bool isSymbolError(gr_complex& x, vector<comp_d> ideal_pos_map);
  void buildIdealPosMap(FlowInfo *flowInfo, vector<comp_d> &ideal_map);


  void packCoefficientsInHeader(MULTIHOP_HDR_TYPE& header, gr_complex* coeffs, int, FlowInfo *flowInfo);
  void interpolate_coeffs(gr_complex* in_coeffs, gr_complex *out_coeffs);   

  /* fwder: possibly change outgoing signal to closest symbols */
  void findClosestSymbol(gr_complex *x, gr_complex **closest_sym_vec, int num_pkts,
                                    vector<gr_complex*> batched_sym_position, unsigned int subcarrier_index, unsigned int o_index);
  void correctStoredSignal(FlowInfo *flowInfo);


  /* for offline combination in the frequency domain */
  void logCorrectedSymbols(FlowInfo *flowInfo);  
  void matchSymbol(gr_complex x, gr_complex &closest_sym, gr_complex* sym_position);
  bool open_corrected_symbols_log();

  int d_replay_flag;
  /* end offline */


  /* for extrapolating header PLL to be used in the payload (no pilot!) */
  float *d_start_rot_error_phase;
  float *d_avg_rot_error_amp;
  float *d_slope_rot_error_phase;
  vector<float*> d_error_vec;						// on each subcarrier, hold error rotations for each OFDM symbol //

  /* to send the ACK on the backend ethernet */
  void send_ack(unsigned char flow_id, unsigned char batch_id);
  int d_ack_sock;

  /* alternative way of doing ILP, more incremental in nature */
  //float **d_euclid_dist;						// [subcarrier][2^batch_size]; records the euclid dist seen on each subcarrier, for each possibility in the table! //
  float d_euclid_dist[MAX_OFDM_SYMBOLS][MAX_DATA_CARRIERS][16];					// for each ofdm symbol - on each subcarrier - 2^batch_size # of entries!!! 

  float d_batch_euclid_dist[MAX_BATCH_SIZE][MAX_OFDM_SYMBOLS][MAX_DATA_CARRIERS][16];
  bool d_flag_euclid_dist[MAX_OFDM_SYMBOLS][MAX_DATA_CARRIERS];				// if set, then atleast one symbol was confident

  unsigned int demapper_ILP_2(unsigned int ofdm_symbol_index, vector<unsigned char*> out_vec,
                      vector<gr_complex*> batched_sym_position, FlowInfo *flowInfo,
                      vector<gr_complex> dfe_vec);
  void slicer_ILP_2(gr_complex x, FlowInfo *flowInfo, gr_complex& closest_sym, unsigned char *bits,
                    gr_complex* batched_sym_position,
                    unsigned int ofdm_index, unsigned int subcarrier_index);
  void buildMap_ILP_2(gr_complex* coeffs, gr_complex* sym_position);
  void demodulate_ILP_2(FlowInfo *flowInfo);


  unsigned int demapper_pilot(const gr_complex *in, unsigned char *out);
  void log_dfe_data();
  void log_dfe_pilot(gr_complex*);
  unsigned int demapper_ILP_2_pilot(unsigned int ofdm_symbol_index, vector<unsigned char*> out_vec,
                                              FlowInfo *flowInfo, vector<gr_complex>* dfe_pilot,
                                              vector<gr_complex*> interpolated_coeffs);
  void interpolate_data_dfe(vector<gr_complex> dfe_pilot, vector<gr_complex>& dfe_data, bool correct, gr_complex *in, gr_complex carrier);
  void track_pilot_dfe(gr_complex *in, int sender, gr_complex& carrier, vector<gr_complex>& dfe_vec);
  void buildMap_pilot(FlowInfo *flowInfo, gr_complex* sym_position,
                                        vector<gr_complex*> interpolated_coeffs,
                                        vector<gr_complex>* dfe, gr_complex* carrier,
                                        int subcarrier_index, int o);

  void test_decode_signal(gr_complex *in, vector<gr_complex*> interpolated_coeffs);
  bool crc_check(std::string msg, std::string&);

  int d_null_symbol_count;
#ifdef USE_PILOT
  float d_slope_angle[MAX_SENDERS], d_start_angle[MAX_SENDERS], d_end_angle[MAX_SENDERS];
#else
  /* for average slope of angle1, phase1, freq1 */
  float d_slope_angle1, d_slope_freq1, d_slope_phase1;
  float d_start_angle1, d_start_freq1, d_start_phase1;
  float d_end_angle1, d_end_freq1, d_end_phase1;
  float d_end_angle;
#endif

  int d_hdr_ofdm_index;

  /* stream tags, timestamping, etc */
  FILE *d_fp_sync_symbols;
  bool d_sync_file_opened;
  gr_tag_t d_sync_tag;
  void set_msg_timestamp(gr_message_sptr msg);
  void test_timestamp(int);
  void test_sync_send(CreditInfo *creditInfo);


  /* usrp instance */
  uhd::usrp::multi_usrp::sptr d_usrp;

  int d_fwd_index;  
  void calc_outgoing_timestamp(uint64_t &sync_secs, double &sync_frac_of_secs); 
  void reduceCoefficients(FlowInfo *flowInfo);
  bool do_carrier_correction();

  void interpolate_coeffs_lerp(gr_complex* in_coeffs, gr_complex *out_coeffs);

  void print_msg(std::string); 
  int d_expected_size, d_fec_n, d_fec_k;
  int d_total_batches_received, d_correct_batches;
  double d_avg_evm_error;			// measure the evm error per batch //   
  int d_crc[MAX_BATCH_SIZE];			// keeps track of how many pkts decoded correctly in the batch //
  int d_num_pkts_correct;
  int d_total_pkts_received;

  float getAvgAmplificationFactor(vector<gr_complex*>);

#ifdef LSQ_COMPRESSION
  void getCoefficients_LSQ(gr_complex*, COEFF*, unsigned int, unsigned int);
  void reduceCoefficients_LSQ(FlowInfo*);
  void packCoefficientsInHeader_LSQ(MULTIHOP_HDR_TYPE&, gr_complex*, int, FlowInfo*);
  void unpackCoefficients_LSQ(gr_complex*, gr_complex*, unsigned int, unsigned int);
  unsigned int d_degree;
  gr_complex d_lsq_coeffs[MAX_BATCH_SIZE * MAX_DEGREE];

  // log - debugging //
  bool d_coeff_unpacked_open;
  FILE *d_fp_coeff_unpacked;

  void dumpCoeffs_LSQ(gr_complex *coeff, int n, cx_fmat A);
  bool d_coeff_open;
  FILE *d_fp_coeff_y;
  FILE *d_fp_coeff_y1;
#endif

#ifdef SRC_PILOT
  void equalizePilot(gr_complex*, PktInfo*);
  unsigned int demapper_ILP_2_pilot_SRC(unsigned int ofdm_symbol_index, vector<unsigned char*> out_vec,
                                                  FlowInfo *flowInfo, vector<gr_complex> dfe_pilot,
                                                  vector<gr_complex*> interpolated_coeffs);
  void track_pilot_dfe_SRC(gr_complex*, vector<gr_complex*>, gr_complex&, vector<gr_complex>&, int);
  void buildMap_pilot_SRC(FlowInfo *flowInfo, gr_complex* sym_position,
                                        vector<gr_complex*> interpolated_coeffs,
                                        vector<gr_complex> dfe, gr_complex carrier,
                                        int subcarrier_index, int o);

  void getResidualRotations(float*);
  void debug_carrier_correction();
  int d_null_symbols;
#endif

  /* post DFT approach to calculate the freq offset */
  void calculate_fine_offset(); 
  gr_complex d_training_symbols[NUM_TRAINING_SYMBOLS*MAX_OCCUPIED_CARRIERS];
  FILE *d_fp_training; 
  bool d_training_log_open;

  void adjust_H_estimate(int);

  void logFrequencyDomainRxSymbols(bool);
  void openRxSymbolLog();

  gr_complex d_known_symbols[NUM_TRAINING_SYMBOLS * MAX_OCCUPIED_CARRIERS];
  float getAvgAmplificationFactor_NV(vector<gr_complex*> hestimates);
  void calculateSNR();
  void generateKnownSymbols();
  bool d_parse_pkt;

  float getMinDistanceInConstellation(gr_complex *constell);

  void calculateSER(unsigned char msg[][MAX_PKT_LEN], int num_bytes);
  gr_complex d_true_symbols[MAX_BATCH_SIZE * MAX_OFDM_SYMBOLS * MAX_DATA_CARRIERS];
  bool d_log_SER_symbols_open; 
  void modulateMsg(gr_complex* out, unsigned char *msg, int num_bytes);

  long d_agg_total_symbols; 
  long d_agg_correct_symbols;

#ifdef MIMO_RECEIVER
  int d_mimo_sock;
  void open_mimo_sock();
  void waitForDecisionFromMimo(unsigned char packet[MAX_BATCH_SIZE][MAX_PKT_LEN]);
#endif

 void chooseCV(FlowInfo *flowInfo, gr_complex *coeffs);
 bool is_CV_good(gr_complex cv1, gr_complex cv2, float&);

 /* util functions */
 void open_server_sock(int sock_port, vector<unsigned int>& connected_clients, int num_clients);
 int open_client_sock(int port, const char *addr, bool blocking);

 void get_nextHop_rx(NodeIds &rx_ids);
 void updateHInfo(HKey, HInfo, bool);
 void initHInfoMap();
 void prepare_H_coding();
 gr_complex predictH(NodeId tx_id, NodeId rx_id);
 void populateEthernetAddress();
 void txHInfo();
 void check_HInfo_rx_sock(int);
 void send_coeff_info_eth(CoeffInfo*);
 NodeId get_coFwd();
 void get_coeffs_from_lead(CoeffInfo *coeffs);
 void smart_selection_local(gr_complex*, CoeffInfo*, FlowInfo*); 
 void smart_selection_global(gr_complex*, CoeffInfo*, FlowInfo*);
 HInfo* getHInfo(NodeId tx_id, NodeId rx_id);
 uhd::time_spec_t getPktTimestamp(int);

 void chooseCV_H(FlowInfo *flowInfo, gr_complex *coeffs);
 bool is_CV_good_H(gr_complex cv1, gr_complex cv2);

 HInfoMap d_HInfoMap;
 EthInfoMap d_ethInfoMap;

 // sockets used //
 int d_coeff_tx_sock, d_coeff_rx_sock;				// for tx coefficients between the co-ordinating transmitters //
 vector<unsigned int> d_h_tx_socks, d_h_rx_socks; 		// for tx HInfo between upstream/dowstream nodes //

 PktTxInfoList d_pktTxInfoList;
 int d_h_coding;

 // to record the pkt timestamps 
 uhd::time_spec_t d_last_pkt_time, d_out_pkt_time;

 FILE *d_fp_dfe_symbols;
 void logDFECorrectedSignals(gr_complex*, vector<gr_complex>, gr_complex, PktInfo*);
 void openDFESymbolLog();
 float getNormalizationFactor(gr_complex*, int);

 void logGeneratedTxSymbols(gr_complex *out);
 bool openLogTxFwdSymbols();

 float get_eq_slope();
 float d_timing_offset;
 void get_phase_offset(float *ph_offset);
 float getTimingOffset();
 void removeTimingOffset();

 // preamble related //
 void calculate_snr(gr_complex*);
 const std::vector<std::vector<gr_complex> >   d_preamble; 
 int d_preamble_cnt;


};


#endif /* INCLUDED_GR_OFDM_FRAME_SINK_H */
