#!/bin/bash
find /usr/local -name "*gnuradio*" -exec rm -fr {} \;
echo "LIBS= -lltdl -larmadillo" >> gr-digital/lib/Makefile
make clean
make -j10
make install
ldconfig
