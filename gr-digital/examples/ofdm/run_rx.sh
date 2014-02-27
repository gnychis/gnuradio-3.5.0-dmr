#!/bin/bash

src=0
d_id=2
h_smart=0
freq=2480M
size=128
mod=qpsk

# tx-params
flow=0
tx_gain=15
tx_ampl=0.35
dst_id=2

# rx-params
thresh1=2e-4	# barcelona, olympic
thresh2=0 #3e-4
thresh3=0
thresh4=0

rx_gain=20

cmd="sudo ./benchmark.py -f $freq --src=$src --tx-gain=$tx_gain --tx-amplitude=$tx_ampl -s $size --id=$d_id -m $mod --threshold1=$thresh1 --rx-gain=$rx_gain --dst=$dst_id --flow=$flow --h-smart=$h_smart --threshold2=$thresh2 --threshold3=$thresh3 --threshold4=$thresh4"

echo $cmd			# print the command

echo "-----------------"
echo " flow:    " $flow
echo " dst:     " $dst_id
echo " src:     " $src
echo " tx-gain: " $tx_gain
echo " tx-amp:  " $tx_ampl
echo " rx-gain: " $rx_gain
echo "-----------------"


$cmd				# execute the command
