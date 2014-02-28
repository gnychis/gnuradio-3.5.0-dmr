#!/bin/bash

d_id=2
src=0
h_smart=1
freq=2480M
size=124
mod=qpsk

# tx-params
tx_gain=15
tx_ampl=0.35
fwd=0
hop_tx=0

# 1e-4 : barcelona
# 8e-5 : olympic

# rx-params
thresh1=1e-4 
thresh2=6e-5
thresh3=0
thresh4=0
rx_gain=20
hop_rx=0

# 0: 1-flow
# 1: 2-flows
# 2: 1-flow-joint
# 3: 2-flows-joint
threshold_type=1
threshold_gap=360000

cmd="sudo ./tunnel.py -f $freq --tx-gain=$tx_gain --tx-amplitude=$tx_ampl -s $size --id=$d_id -m $mod --rx-gain=$rx_gain --h-smart=$h_smart --threshold1=$thresh1 --threshold2=$thresh2 --threshold3=$thresh3 --threshold4=$thresh4 --threshold-type=$threshold_type --threshold-gap=$threshold_gap --fwd=$fwd"

echo $cmd                       # print the command

echo "-----------------"
echo " tx-gain: " $tx_gain
echo " tx-amp:  " $tx_ampl
echo " rx-gain: " $rx_gain
echo " fwd:     " $fwd
echo "-----------------"


$cmd                            # execute the command
