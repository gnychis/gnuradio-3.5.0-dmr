/* -*- c++ -*- */
/*
 * Copyright 2005,2011 Free Software Foundation, Inc.
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

#ifndef INCLUDED_DIGITAL_CRC32_H
#define INCLUDED_DIGITAL_CRC32_H

#include <digital_api.h>
#include <string>
#include <gr_types.h>


/************************************** FEC related ********************************************/
#define DTYPE unsigned char
/* Reed-Solomon codec control block */
struct rs {
  unsigned int mm;              /* Bits per symbol */
  unsigned int nn;              /* Symbols per block (= (1<<mm)-1) */
  unsigned char *alpha_to;      /* log lookup table */
  unsigned char *index_of;      /* Antilog lookup table */
  unsigned char *genpoly;       /* Generator polynomial */
  unsigned int nroots;     /* Number of generator roots = number of parity symbols */
  unsigned char fcr;        /* First consecutive root, index form */
  unsigned char prim;       /* Primitive element, index form */
  unsigned char iprim;      /* prim-th root of 1, index form */
};

static inline int modnn(struct rs *rs,int x){
  while (x >= rs->nn) {
    x -= rs->nn;
    x = (x >> rs->mm) + (x & rs->nn);
  }
  return x;
}

#define MODNN(x) modnn(rs,x)

#define MM (rs->mm)
#define NN (rs->nn)
#define ALPHA_TO (rs->alpha_to)
#define INDEX_OF (rs->index_of)
#define GENPOLY (rs->genpoly)
#define NROOTS (rs->nroots)
#define FCR (rs->fcr)
#define PRIM (rs->prim)
#define IPRIM (rs->iprim)
#define A0 (NN)

static struct rs*
digital_init_rs(unsigned int symsize,unsigned int gfpoly,unsigned int fcr,
	       unsigned int prim,unsigned int nroots);
void
digital_free_rs(void *p);

std::string
digital_rx_wrapper(std::string buf, int fec_N, int fec_K, int bits_per_symbol, int expectedDataLen);

std::string
digital_decode_rs_fec(std::string buf, int n, int k, int dataLen);

int
convertToSymbols(unsigned char *symbols, int symSize, std::string s_bits);

int
digital_decode_rs(struct rs *rs, unsigned char *data);

std::string
getBits(const unsigned char *data, int len, const int symSize);

struct rs*
find_rs_handle(std::vector<struct rs*> rs_vec, int n, int k);

std::string
digital_encode_rs_fec(std::string buf, int N, int K);

std::string
getMsg(std::string str, unsigned char *data);

int
getSymSizeFromN_Syms(int n);

std::string
getBitString(const unsigned char *data, int len);

std::string
digital_tx_wrapper(std::string buf, int fec_N, int fec_K, int bits_per_symbol);

/*!
 * \brief update running CRC-32
 * \ingroup digital
 *
 * Update a running CRC with the bytes buf[0..len-1] The CRC should be
 * initialized to all 1's, and the transmitted value is the 1's
 * complement of the final running CRC.  The resulting CRC should be
 * transmitted in big endian order.
 */
DIGITAL_API unsigned int 
digital_update_crc32(unsigned int crc, const unsigned char *buf, size_t len);

DIGITAL_API unsigned int 
digital_update_crc32(unsigned int crc, const std::string buf);

DIGITAL_API unsigned int 
digital_crc32(const unsigned char *buf, size_t len);

DIGITAL_API unsigned int 
digital_crc32(const std::string buf);

#endif /* INCLUDED_CRC32_H */
