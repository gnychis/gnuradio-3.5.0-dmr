#!/bin/bash


#id
if [ ! -z $1 ]
   then
      id=$1
   else
      id=1
fi

#modulation
if [ ! -z $2 ]                         
   then
      mod=$2     
   else
      mod=qpsk
fi

#h-smart
if [ ! -z $3 ]                         
   then
      h_smart=$3
   else
      h_smart=1
fi

freq=2480M
pkt_size=104

# tx-parameters
amp=0.3
tx_gain=20

# rx-parameters
thresh=1e-4
rx_gain=10

# the command
cmd="sudo ./tunnel.py -f $freq --tx-gain=$tx_gain --tx-amplitude=$amp --threshold=$thresh --rx-gain=$rx_gain --id=$id -s $pkt_size -m $mod --h-smart=$h_smart"

echo $cmd			# print the command
$cmd				# execute the command
