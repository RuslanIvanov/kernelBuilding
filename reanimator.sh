#!/bin/bash
#place in /root
cd /mnt/boot/
IP_ADDRESS=$1

echo `pwd`
sleep 3

scp  ./syslinux.cfg    root@$IP_ADDRESS:/mnt/boot/
scp  ./ldlinux.sys   root@$IP_ADDRESS:/mnt/boot/
scp  ./bzimage  root@$IP_ADDRESS:/mnt/boot/

ls -al 
