#!/bin/bash

toolchain=~/builds/buildroot_tam3517_upum6/buildroot-2013.05/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabi-

if [ X${toolchain} = X ]
then
	echo "Toolchain not setup, please export CROSS_COMPILE to toolchain path and prefix, like \"export CROSS_COMPILE=/opt/arm-2010q1/bin/arm-none-linux-gnueabi-\""
	exit 1
fi

WORK_DIR=$PWD

if [ Y$1 = Y ]
then
        echo Argument missing. Should be patht to kernel sources
        exit 1
fi

kernel=$1

echo "clean $WORK_DIR/$kernel"
sleep 5

cd ${WORK_DIR}/$kernel
make ARCH=arm CROSS_COMPILE="${toolchain}" distclean
