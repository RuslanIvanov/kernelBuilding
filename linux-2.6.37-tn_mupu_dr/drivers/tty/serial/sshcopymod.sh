#!/bin/bash

D=`date +%d%m%Y_%H%M%S`
echo "Date [$D]"

echo "COPY TO server in '~/' or PATCH !"

echo "for all copy:    '$0 IP_ADR MOD [KER_NUM] [PATCH]'"
echo "for custom copy: '$0 IP_ADR MOD [KER_NUM] [PATCH]'"

sleep 5
IP_ADDRESS=$1
VAR2=$2
KER_NUM=$3
PATCH=$4

if [[ "$1" == "-h" || "$1" == "--help" || $# == "0" ]]
then
        echo "Exit..."
        exit 0;
fi

if [ -z "$1" ]
then
  echo "No arguments in command string! Enter '$0 <IP_ADDRESS>'"
  exit 0
fi


if [[ -z "$KER_NUM" || $KER_NUM == "NULL" || $KER_NUM == "0" ]]
then
	if [ -z "PATCH" ]
	then
        	PATCH="~/"
	fi
else
	 PATCH="/lib/modules/$KER_NUM/kernel/drivers/"
fi

FILEKEY="$HOME/.ssh/known_hosts"
echo "run: ssh-keygen -f $FILEKEY -R $IP_ADDRESS"
ssh-keygen -f $FILEKEY -R $IP_ADDRESS

sleep 2

echo "TRANSMIT on IP_ADDRESS $IP_ADDRESS ..."

if [[ "$VAR2" == "MOD" ]]
then

	echo "copy modules to '$PATCH'  [date: $D]"

	scp ./*.ko root@$IP_ADDRESS:$PATCH
#	ssh root@$IP_ADDRESS chmod 664 /usr/bin/readown
fi


echo "TX DONE!"
