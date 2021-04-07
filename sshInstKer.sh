#!/bin/bash

D=`date +%d%m%Y_%H%M%S`
echo "Date [$D]"

#ssh-keygen -f "~/.ssh/known_hosts" -R $IP_ADDRESS

sleep 2

IP_ADDRESS=$1

if [ -z "$1" ]
then
  echo "No arguments in command string! Enter '$0 <IP_ADDRESS>'"
  exit 0
fi

if [ -z "$2" ]
then
  echo "No arguments in command string! Enter '$0 $1 <LOCAL FILE NAME>'"
  exit 0
fi

#KER_IMAGE=`echo "$2" | tr '[:upper:]' '[:lower:]'`
KER_IMAGE=$2
echo "image: '$KER_IMAGE'"

sleep 2

ssh root@$IP_ADDRESS cp /mnt/boot/bzimage /root/bzimage_old$D
scp  ./$KER_IMAGE root@$IP_ADDRESS:/mnt/boot/
ssh root@$IP_ADDRESS chmod 744 /mnt/boot/$KER_IMAGE
ssh root@$IP_ADDRESS ls -al /mnt/boot/


