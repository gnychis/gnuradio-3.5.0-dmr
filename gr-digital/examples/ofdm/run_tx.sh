#!/bin/bash

num_args=$#;

#id
if [ ! -z $1 ]
   then
      id=$1
   else
      id=1
fi

#dst
if [ ! -z $2 ]                         
   then
      dst=$2
   else
      dst=2
fi

#modulation
if [ ! -z $3 ]                         
   then
      mod=$3     
   else
      mod=qpsk
fi

#h-smart
if [ ! -z $4 ]                         
   then
      h_smart=$4     
   else
      h_smart=1
fi

freq=2480M
pkt_size=100

# tx-parameters
amp=0.4
tx_gain=20

# the command
cmd="sudo ./benchmark_tx.py -f $freq --tx-amplitude=$amp --tx-gain=$tx_gain --src=1 --id=$id --dst=$dst -s $pkt_size -m $mod --h-smart=$h_smart"

echo $cmd			# print the command
$cmd				# execute the command

