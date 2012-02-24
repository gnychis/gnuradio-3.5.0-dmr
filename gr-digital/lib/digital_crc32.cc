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

/*
 * See also ISO 3309 [ISO-3309] or ITU-T V.42 [ITU-V42] for a formal specification.
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <digital_crc32.h>

#include <stdlib.h>    
#include <string.h>
#include <bitset>
#include <strings.h>
#include <stdio.h>     
#include <vector>
#include <map>

#define NULL ((void *)0)
#define min(a,b)        ((a) < (b) ? (a) : (b))
   
// Automatically generated CRC function
// polynomial: 0x104C11DB7
unsigned int
digital_update_crc32(unsigned int crc, const unsigned char *data, size_t len)
{
    static const unsigned int table[256] = {
    0x00000000U,0x04C11DB7U,0x09823B6EU,0x0D4326D9U,
    0x130476DCU,0x17C56B6BU,0x1A864DB2U,0x1E475005U,
    0x2608EDB8U,0x22C9F00FU,0x2F8AD6D6U,0x2B4BCB61U,
    0x350C9B64U,0x31CD86D3U,0x3C8EA00AU,0x384FBDBDU,
    0x4C11DB70U,0x48D0C6C7U,0x4593E01EU,0x4152FDA9U,
    0x5F15ADACU,0x5BD4B01BU,0x569796C2U,0x52568B75U,
    0x6A1936C8U,0x6ED82B7FU,0x639B0DA6U,0x675A1011U,
    0x791D4014U,0x7DDC5DA3U,0x709F7B7AU,0x745E66CDU,
    0x9823B6E0U,0x9CE2AB57U,0x91A18D8EU,0x95609039U,
    0x8B27C03CU,0x8FE6DD8BU,0x82A5FB52U,0x8664E6E5U,
    0xBE2B5B58U,0xBAEA46EFU,0xB7A96036U,0xB3687D81U,
    0xAD2F2D84U,0xA9EE3033U,0xA4AD16EAU,0xA06C0B5DU,
    0xD4326D90U,0xD0F37027U,0xDDB056FEU,0xD9714B49U,
    0xC7361B4CU,0xC3F706FBU,0xCEB42022U,0xCA753D95U,
    0xF23A8028U,0xF6FB9D9FU,0xFBB8BB46U,0xFF79A6F1U,
    0xE13EF6F4U,0xE5FFEB43U,0xE8BCCD9AU,0xEC7DD02DU,
    0x34867077U,0x30476DC0U,0x3D044B19U,0x39C556AEU,
    0x278206ABU,0x23431B1CU,0x2E003DC5U,0x2AC12072U,
    0x128E9DCFU,0x164F8078U,0x1B0CA6A1U,0x1FCDBB16U,
    0x018AEB13U,0x054BF6A4U,0x0808D07DU,0x0CC9CDCAU,
    0x7897AB07U,0x7C56B6B0U,0x71159069U,0x75D48DDEU,
    0x6B93DDDBU,0x6F52C06CU,0x6211E6B5U,0x66D0FB02U,
    0x5E9F46BFU,0x5A5E5B08U,0x571D7DD1U,0x53DC6066U,
    0x4D9B3063U,0x495A2DD4U,0x44190B0DU,0x40D816BAU,
    0xACA5C697U,0xA864DB20U,0xA527FDF9U,0xA1E6E04EU,
    0xBFA1B04BU,0xBB60ADFCU,0xB6238B25U,0xB2E29692U,
    0x8AAD2B2FU,0x8E6C3698U,0x832F1041U,0x87EE0DF6U,
    0x99A95DF3U,0x9D684044U,0x902B669DU,0x94EA7B2AU,
    0xE0B41DE7U,0xE4750050U,0xE9362689U,0xEDF73B3EU,
    0xF3B06B3BU,0xF771768CU,0xFA325055U,0xFEF34DE2U,
    0xC6BCF05FU,0xC27DEDE8U,0xCF3ECB31U,0xCBFFD686U,
    0xD5B88683U,0xD1799B34U,0xDC3ABDEDU,0xD8FBA05AU,
    0x690CE0EEU,0x6DCDFD59U,0x608EDB80U,0x644FC637U,
    0x7A089632U,0x7EC98B85U,0x738AAD5CU,0x774BB0EBU,
    0x4F040D56U,0x4BC510E1U,0x46863638U,0x42472B8FU,
    0x5C007B8AU,0x58C1663DU,0x558240E4U,0x51435D53U,
    0x251D3B9EU,0x21DC2629U,0x2C9F00F0U,0x285E1D47U,
    0x36194D42U,0x32D850F5U,0x3F9B762CU,0x3B5A6B9BU,
    0x0315D626U,0x07D4CB91U,0x0A97ED48U,0x0E56F0FFU,
    0x1011A0FAU,0x14D0BD4DU,0x19939B94U,0x1D528623U,
    0xF12F560EU,0xF5EE4BB9U,0xF8AD6D60U,0xFC6C70D7U,
    0xE22B20D2U,0xE6EA3D65U,0xEBA91BBCU,0xEF68060BU,
    0xD727BBB6U,0xD3E6A601U,0xDEA580D8U,0xDA649D6FU,
    0xC423CD6AU,0xC0E2D0DDU,0xCDA1F604U,0xC960EBB3U,
    0xBD3E8D7EU,0xB9FF90C9U,0xB4BCB610U,0xB07DABA7U,
    0xAE3AFBA2U,0xAAFBE615U,0xA7B8C0CCU,0xA379DD7BU,
    0x9B3660C6U,0x9FF77D71U,0x92B45BA8U,0x9675461FU,
    0x8832161AU,0x8CF30BADU,0x81B02D74U,0x857130C3U,
    0x5D8A9099U,0x594B8D2EU,0x5408ABF7U,0x50C9B640U,
    0x4E8EE645U,0x4A4FFBF2U,0x470CDD2BU,0x43CDC09CU,
    0x7B827D21U,0x7F436096U,0x7200464FU,0x76C15BF8U,
    0x68860BFDU,0x6C47164AU,0x61043093U,0x65C52D24U,
    0x119B4BE9U,0x155A565EU,0x18197087U,0x1CD86D30U,
    0x029F3D35U,0x065E2082U,0x0B1D065BU,0x0FDC1BECU,
    0x3793A651U,0x3352BBE6U,0x3E119D3FU,0x3AD08088U,
    0x2497D08DU,0x2056CD3AU,0x2D15EBE3U,0x29D4F654U,
    0xC5A92679U,0xC1683BCEU,0xCC2B1D17U,0xC8EA00A0U,
    0xD6AD50A5U,0xD26C4D12U,0xDF2F6BCBU,0xDBEE767CU,
    0xE3A1CBC1U,0xE760D676U,0xEA23F0AFU,0xEEE2ED18U,
    0xF0A5BD1DU,0xF464A0AAU,0xF9278673U,0xFDE69BC4U,
    0x89B8FD09U,0x8D79E0BEU,0x803AC667U,0x84FBDBD0U,
    0x9ABC8BD5U,0x9E7D9662U,0x933EB0BBU,0x97FFAD0CU,
    0xAFB010B1U,0xAB710D06U,0xA6322BDFU,0xA2F33668U,
    0xBCB4666DU,0xB8757BDAU,0xB5365D03U,0xB1F740B4U,
    };
  
    while (len > 0)
    {
      crc = table[*data ^ ((crc >> 24) & 0xff)] ^ (crc << 8);
      data++;
      len--;
    }
    return crc;
}

unsigned int
digital_update_crc32(unsigned int crc, const std::string s)
{
  return digital_update_crc32(crc, (const unsigned char *) s.data(), s.size());
}
    
unsigned int
digital_crc32(const unsigned char *buf, size_t len)
{
  return digital_update_crc32(0xffffffff, buf, len) ^ 0xffffffff;
}

unsigned int
digital_crc32(const std::string s)
{
  return digital_crc32((const unsigned char *) s.data(), s.size());
}

/********************************* FEC Decoder code starts ***********************************/
/* complete wrapper that encapsulates the 'fec decoding' options. Turn
   the options on and off to achieve the desired effect.
*/
std::string
digital_rx_wrapper(std::string buf, int fec_N, int fec_K, int bits_per_symbol, int expectedDataLen)
{
    std::string decoded_str = digital_decode_rs_fec(buf, fec_N, fec_K, expectedDataLen);
    return decoded_str;
}

std::string
digital_decode_rs_fec(std::string buf, int n, int k, int dataLen)
{
    static std::vector<struct rs*> rs_vec;
    static struct rs *rs = find_rs_handle(rs_vec, n, k);
    assert(rs);
    int syms = rs->mm;

    //printf("nn = %d; nroots=%d, \n", rs->nn, rs->nroots);
     // get bits from the byte string //
    std::string s_bits = getBits((const unsigned char*) buf.data(), buf.size(), 8);

    int num_bits = s_bits.size();
    unsigned char *data = (unsigned char*) malloc(sizeof(unsigned char) * ceil(num_bits/syms));
    memset(data, 0, sizeof(unsigned char) * ceil(num_bits/syms));

    int len = convertToSymbols(data, syms, s_bits);
#ifdef DEBUG_FEC
    printf("in decode_rs_fec: %d---- \n", len);
    for(int i = 0; i < len; i++)
          printf("%d ", (unsigned char) data[i]);
    printf("\n");
#endif

    convertToSymbols(data, syms, s_bits);

#ifdef DEBUG_FEC
    printf("after ERRORS nsymbols: %d------ \n", len);
    for(int i = 0; i < len; i++)
          printf("%d ", data[i]);
    printf("\n");

    printf("received data %d symbols (symSize: %d)\n", len, syms);
    for(int i = 0; i < len; i++)
        printf("%d ", data[i]);
    printf("\n\n"); fflush(stdout);
#endif

  dataLen = dataLen * 8/syms;
  assert(len > dataLen);
  int num_blocks = ceil(((float)len/NN));

  fflush(stdout);

  int processedDataLen = 0;
  unsigned char data_block[NN];
  //memset(data_block, 0, NN);

  unsigned char *decoded_data = (unsigned char*) malloc(dataLen);
  memset(decoded_data, 0, dataLen);
  int pad_zeros = 0;

  for(int i=0; i<num_blocks; i++)
  {
     memset(data_block, 0, NN);
     int num_decoded = 0;
     if((processedDataLen + (NN - NROOTS))< dataLen)
        num_decoded = NN-NROOTS;
     else
     {
        if(i != num_blocks - 1)
           return buf;							// flag as incorrect reception //
        assert(i == num_blocks - 1);
        num_decoded = dataLen - processedDataLen;
     }

     // perform the FEC decoding on the current <data_block> now //
     memcpy(data_block, data + i*NN, num_decoded);

     // copy the parity (NROOTS) to the data_block (leave space for padding if reqd)//
     memcpy(data_block + (NN-NROOTS), data + i*NN + num_decoded, NROOTS);
     memcpy(decoded_data + processedDataLen, data_block, num_decoded);

#ifdef DEBUG_FEC
     printf("undecoded block[%d]: \n", i);
     for(int k = 0; k < NN; k++)
        printf("%d ", (unsigned char) data_block[k]);
     printf("\n");
#endif

     int retVal = digital_decode_rs(rs, data_block);
     if(retVal >= 0){
        memcpy(decoded_data + processedDataLen, data_block, num_decoded);
     }

#ifdef DEBUG_FEC
     printf("retVal: %d decoded block[%d]::::  \n", retVal, i);
     for(int k = 0; k < NN; k++) 
        printf("%d ", data_block[k]);
     printf("\n\n");
#endif

     processedDataLen += num_decoded;
  }

  if(processedDataLen != dataLen)
  {
     printf("nn%d nroots%d len%d num_blocks%d processedDataLen%d dataLen%d\n", NN, NROOTS,
                                                len, num_blocks, processedDataLen, dataLen);
     assert(false);
  }

#ifdef DEBUG_FEC
  printf("decoded data - \n");
  for(int i = 0; i < dataLen; i++)
      printf("%d ", decoded_data[i]);
  printf("\n\n"); fflush(stdout);
#endif

  // convert the symSize symbols back into bytes //
  unsigned char *decoded_bytes = (unsigned char *) malloc(sizeof(unsigned char) * dataLen);
  memset(decoded_bytes, 0, sizeof(unsigned char) * dataLen);

  std::string bits = getBits(decoded_data, dataLen, syms);
  int nbytes = convertToSymbols(decoded_bytes, 8, bits);

  free(decoded_data);

  std::string s1 = std::string((const char*) decoded_bytes, nbytes);
  return s1;
}

void digital_free_rs(void *p){
  struct rs *rs = (struct rs *)p;

  free(rs->alpha_to);
  free(rs->index_of);
  free(rs->genpoly);
  free(rs);
}

/* for reed solomon, converts the s_bits into symbols of 'symSize' bits each. Returns
 * the num symbols created in 'symbols'
 */
int
convertToSymbols(unsigned char *symbols, int symSize, std::string s_bits)
{
   int k = 0;
   for(int i = 0; i < s_bits.size(); i+=symSize)
   {
        std::bitset<sizeof(unsigned char) * 8> bits(s_bits.substr(i, symSize));
        symbols[k] = (unsigned char) bits.to_ulong();
        k++;
   }

   return k;
}

int
digital_decode_rs(struct rs *rs, unsigned char *data)
{

  int deg_lambda, el, deg_omega;
  int i, j, r,k;
  DTYPE u,q,tmp,num1,num2,den,discr_r;
  DTYPE lambda[NROOTS+1], s[NROOTS];    /* Err+Eras Locator poly
                                         * and syndrome poly */
  DTYPE b[NROOTS+1], t[NROOTS+1], omega[NROOTS+1];
  DTYPE root[NROOTS], reg[NROOTS+1], loc[NROOTS];
  int syn_error, count;

  /* form the syndromes; i.e., evaluate data(x) at roots of g(x) */
  for(i=0;i<NROOTS;i++)
    s[i] = data[0];

  for(j=1;j<NN;j++){
    for(i=0;i<NROOTS;i++){
      if(s[i] == 0){
        s[i] = data[j];
      } else {
        s[i] = data[j] ^ ALPHA_TO[MODNN(INDEX_OF[s[i]] + (FCR+i)*PRIM)];
      }
    }
  }
  /* Convert syndromes to index form, checking for nonzero condition */
  syn_error = 0;
  for(i=0;i<NROOTS;i++){
    syn_error |= s[i];
    s[i] = INDEX_OF[s[i]];
  }

  if (!syn_error) {
    /* if syndrome is zero, data[] is a codeword and there are no
     * errors to correct. So return data[] unmodified
     */
    count = 0;
    goto finish;
  }
  memset(&lambda[1],0,NROOTS*sizeof(lambda[0]));
  lambda[0] = 1;

  for(i=0;i<NROOTS+1;i++)
    b[i] = INDEX_OF[lambda[i]];

  /*
   * Begin Berlekamp-Massey algorithm to determine error+erasure
   * locator polynomial
   */
  r = 0;
  el = 0;
  while (++r <= NROOTS) {       /* r is the step number */
    /* Compute discrepancy at the r-th step in poly-form */
    discr_r = 0;
    for (i = 0; i < r; i++){
      if ((lambda[i] != 0) && (s[r-i-1] != A0)) {
        discr_r ^= ALPHA_TO[MODNN(INDEX_OF[lambda[i]] + s[r-i-1])];
      }
    }
    discr_r = INDEX_OF[discr_r];        /* Index form */
    if (discr_r == A0) {
      /* 2 lines below: B(x) <-- x*B(x) */
      memmove(&b[1],b,NROOTS*sizeof(b[0]));
      b[0] = A0;
    } else {
      /* 7 lines below: T(x) <-- lambda(x) - discr_r*x*b(x) */
      t[0] = lambda[0];
      for (i = 0 ; i < NROOTS; i++) {
        if(b[i] != A0)
          t[i+1] = lambda[i+1] ^ ALPHA_TO[MODNN(discr_r + b[i])];
        else
          t[i+1] = lambda[i+1];
      }
      if (2 * el <= r - 1) {
        el = r - el;
        /*
         * 2 lines below: B(x) <-- inv(discr_r) *
         * lambda(x)
         */
        for (i = 0; i <= NROOTS; i++)
          b[i] = (lambda[i] == 0) ? A0 : MODNN(INDEX_OF[lambda[i]] - discr_r + NN);
      } else {
        /* 2 lines below: B(x) <-- x*B(x) */
        memmove(&b[1],b,NROOTS*sizeof(b[0]));
        b[0] = A0;
      }
      memcpy(lambda,t,(NROOTS+1)*sizeof(t[0]));
    }
  }

  /* Convert lambda to index form and compute deg(lambda(x)) */
  deg_lambda = 0;
  for(i=0;i<NROOTS+1;i++){
    lambda[i] = INDEX_OF[lambda[i]];
    if(lambda[i] != A0)
      deg_lambda = i;
  }
  /* Find roots of the error+erasure locator polynomial by Chien search */
  memcpy(&reg[1],&lambda[1],NROOTS*sizeof(reg[0]));
  count = 0;            /* Number of roots of lambda(x) */
  for (i = 1,k=IPRIM-1; i <= NN; i++,k = MODNN(k+IPRIM)) {
    q = 1; /* lambda[0] is always 0 */
    for (j = deg_lambda; j > 0; j--){
      if (reg[j] != A0) {
        reg[j] = MODNN(reg[j] + j);
        q ^= ALPHA_TO[reg[j]];
      }
    }
    if (q != 0)
      continue; /* Not a root */
    /* store root (index-form) and error location number */
    root[count] = i;
    loc[count] = k;
    /* If we've already found max possible roots,
     * abort the search to save time
     */
    if(++count == deg_lambda)
      break;
  }
  if (deg_lambda != count) {
    /*
     * deg(lambda) unequal to number of roots => uncorrectable
     * error detected
     */
    count = -1;
    goto finish;
  }
  /*
   * Compute err+eras evaluator poly omega(x) = s(x)*lambda(x) (modulo
   * x**NROOTS). in index form. Also find deg(omega).
   */
  deg_omega = 0;
  for (i = 0; i < NROOTS;i++){
    tmp = 0;
    j = (deg_lambda < i) ? deg_lambda : i;
    for(;j >= 0; j--){
      if ((s[i - j] != A0) && (lambda[j] != A0))
        tmp ^= ALPHA_TO[MODNN(s[i - j] + lambda[j])];
    }
    if(tmp != 0)
      deg_omega = i;
    omega[i] = INDEX_OF[tmp];
  }
  omega[NROOTS] = A0;

  /*
   * Compute error values in poly-form. num1 = omega(inv(X(l))), num2 =
   * inv(X(l))**(FCR-1) and den = lambda_pr(inv(X(l))) all in poly-form
   */
  for (j = count-1; j >=0; j--) {
    num1 = 0;
    for (i = deg_omega; i >= 0; i--) {
      if (omega[i] != A0)
        num1  ^= ALPHA_TO[MODNN(omega[i] + i * root[j])];
    }
    num2 = ALPHA_TO[MODNN(root[j] * (FCR - 1) + NN)];
    den = 0;

    /* lambda[i+1] for i even is the formal derivative lambda_pr of lambda[i] */
    for (i = min(deg_lambda,NROOTS-1) & ~1; i >= 0; i -=2) {
      if(lambda[i+1] != A0)
        den ^= ALPHA_TO[MODNN(lambda[i+1] + i * root[j])];
    }
    if (den == 0) {
      count = -1;
      goto finish;
    }
    /* Apply error to data */
    if (num1 != 0) {
      data[loc[j]] ^= ALPHA_TO[MODNN(INDEX_OF[num1] + INDEX_OF[num2] + NN - INDEX_OF[den])];
    }
  }

  finish:
#ifdef DEBUG_FEC
     printf(" --------------------------------------------\n");
     printf("\nin decode_rs: count%d\n\n", count); fflush(stdout);
     printf(" --------------------------------------------\n");
#endif
     return count;
}


std::string
getBits(const unsigned char *data, int len, const int symSize)
{
  std::string orig_s;
  for(int i = 0; i < len; i++)
  {
      std::bitset<sizeof(unsigned char) * 8> s_bits(data[i]);
      std::string tmp_string = s_bits.to_string<char,std::char_traits<char>,std::allocator<char> >();
      orig_s.append(tmp_string.substr(8 - symSize, symSize));
  }
  return orig_s;
}

struct rs*
find_rs_handle(std::vector<struct rs*> rs_vec, int n, int k)
{
    //printf("finding n: %d, k: %d\n", n, k); fflush(stdout);
    int syms;
    int gfpoly;
    int nn, nroots;

    switch(n){
        case 3:
                syms = 2; gfpoly = 0x7; break;
        case 7:
                syms = 3; gfpoly = 0xb; break;
        case 15:
                syms = 4; gfpoly = 0x13; break;
        case 31:
                syms = 5; gfpoly = 0x25; break;
        case 63:
                syms = 6; gfpoly = 0x43; break;
        case 127:
                syms = 7; gfpoly = 0x89; break;
        case 255:
                syms = 8; gfpoly = 0x11d; break;
        default:
                printf("the length of the block is out of range, FEC cannot be initiated.");
                break;
    }

    std::vector<struct rs*>::iterator it = rs_vec.begin();
    struct rs *rs;

    while(it != rs_vec.end())
    {
        //printf("searching!\n"); fflush(stdout);
        rs = *it;
        if(rs->nn == n && (rs->nroots - rs->nn) == k)
        {
                return rs;
        }
        it++;
    }

    nroots = n - k;
    rs = digital_init_rs(syms, gfpoly, 1, 1, nroots);
    rs_vec.push_back(rs);
    return rs;
}

static struct rs*
digital_init_rs(unsigned int symsize,unsigned int gfpoly,unsigned fcr,unsigned prim, unsigned int nroots)
{
  struct rs *rs;
  int i, j, sr,root,iprim;

  //printf("coming in......\n");

  if(symsize > 8*sizeof(DTYPE)){
    printf("Need version with ints rather than chars\n");
    exit(0);
    //return NULL; /* Need version with ints rather than chars */
  }
  if(fcr >= (1<<symsize)){
    printf("The fcr value is abnormal.\n");
    exit(0);
    //return NULL;
  }
  if(prim == 0 || prim >= (1<<symsize)){
    printf("The prim value is abnormal...\n");
    exit(0);
    //return NULL;
  }
  if(nroots >= (1<<symsize)){
    printf("Can't have more roots than symbol values!\n");
    exit(0);
    //return NULL; /* Can't have more roots than symbol values! */
  }

  rs = (struct rs *)calloc(1,sizeof(struct rs));
  rs->mm = symsize;
  rs->nn = (1<<symsize)-1;

  rs->alpha_to = (DTYPE *)malloc(sizeof(DTYPE)*(rs->nn+1));
  if(rs->alpha_to == NULL){
    free(rs);
    printf("Cannot set the values of 'alpha_to[]' of reedsolomon...\n");
    exit(0);
    //return NULL;
  }

  rs->index_of = (DTYPE *)malloc(sizeof(DTYPE)*(rs->nn+1));
  if(rs->index_of == NULL){
    free(rs->alpha_to);
    free(rs);
    printf("Cannot set the value of index of reedsolomon...\n");
    exit(0);
    //return NULL;
  }

  /* Generate Galois field lookup tables */
  rs->index_of[0] = A0; /* log(zero) = -inf */
  rs->alpha_to[A0] = 0; /* alpha**-inf = 0 */
  sr = 1;
  for(i=0;i<rs->nn;i++){
    rs->index_of[sr] = i;
    rs->alpha_to[i] = sr;
    sr <<= 1;
    if(sr & (1<<symsize))
      sr ^= gfpoly;
    sr &= rs->nn;
  }
  if(sr != 1){
    /* field generator polynomial is not primitive! */
    free(rs->alpha_to);
    free(rs->index_of);
    free(rs);
    printf("field generator polynomial is not primitive!\n");
    exit(0);
    //return NULL;
  }

  /* Form RS code generator polynomial from its roots */
  rs->genpoly = (DTYPE *)malloc(sizeof(DTYPE)*(nroots+1));
  if(rs->genpoly == NULL){
    free(rs->alpha_to);
    free(rs->index_of);
    free(rs);
    printf("Cannot set the values of generation polynomial of reedsolomon...\n");
    exit(0);
    //return NULL;
  }
  rs->fcr = fcr;
  rs->prim = prim;
  rs->nroots = nroots;


  /* Find prim-th root of 1, used in decoding */
  for(iprim=1;(iprim % prim) != 0;iprim += rs->nn)
    ;
  rs->iprim = iprim / prim;

  rs->genpoly[0] = 1;
  for (i = 0,root=fcr*prim; i < nroots; i++,root += prim) {
    rs->genpoly[i+1] = 1;

    /* Multiply rs->genpoly[] by  @**(root + x) */
    for (j = i; j > 0; j--){
      if (rs->genpoly[j] != 0)
        rs->genpoly[j] = rs->genpoly[j-1] ^ rs->alpha_to[modnn(rs,rs->index_of[rs->genpoly[j]] + root)];
      else
        rs->genpoly[j] = rs->genpoly[j-1];
    }
    /* rs->genpoly[0] can never be zero */
    rs->genpoly[0] = rs->alpha_to[modnn(rs,rs->index_of[rs->genpoly[0]] + root)];
  }
  /* convert rs->genpoly[] to index form for quicker encoding */
  for (i = 0; i <= nroots; i++)
    rs->genpoly[i] = rs->index_of[rs->genpoly[i]];

  return rs;
}
