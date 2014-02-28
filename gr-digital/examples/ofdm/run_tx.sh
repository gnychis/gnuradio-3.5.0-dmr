#!/bin/bash

src=1
d_id=1
dst_id=3
flow=0
h_smart=1
freq=2480M
size=124
mod=qpsk

# tx-params
tx_gain=15
tx_ampl=0.35
mimo=1

cmd="sudo ./benchmark_tx.py -f $freq --src=$src --tx-gain=$tx_gain --tx-amplitude=$tx_ampl -s $size --id=$d_id -m $mod --dst=$dst_id --flow=$flow --h-smart=$h_smart --mimo=$mimo"

echo "-----------------"
echo " flow:    " $flow
echo " dst:     " $dst_id
echo " src:     " $src
echo " tx-gain: " $tx_gain
echo " tx-amp:  " $tx_ampl
echo "-----------------"

echo $cmd			# print the command
$cmd				# execute the command
