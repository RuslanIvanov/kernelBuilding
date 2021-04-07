#!/bin/bash
D=`date +%d%m%Y_%H%M%S`
echo "Date [$D]"

if [ X$1 = X ]
then
        echo Argument missing. Should be one of "linux","..."
        exit 1
fi

product=$1
threads=`cat /proc/cpuinfo | grep processor | wc -l`
threads=`expr ${threads}`

WORK_DIR=$PWD
echo "WORK_DIR: '$WORK_DIR'"
sleep 5


if [ Y$2 = Y ]
then
        echo Argument missing. Should be patht to kernel sources
        exit 1
fi

kernel=$2
KERNEL_PATH=${WORK_DIR}/$kernel

echo "kernel: '$kernel'"
sleep 5

target_os=linux

echo "terget os: $target_os"
sleep 5

if [ ${target_os} = "linux" ]
then
		#toolchain=$CROSS_COMPILE
		toolchain=~/builds/buildroot_tam3517_upum6/buildroot-2013.05/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabi-
        if [ X${toolchain} = X ]
        then
                echo "Toolchain not setup, please export CROSS_COMPILE to toolchain path and prefix, like \"export CROSS_COMPILE=/opt/arm-2010q1/bin/arm-none-linux-gnueabi-\""
                exit 1
        fi

fi

if [ Z$3 = Z ]
then
        echo "Argument missing. Should be config file"
        exit 1
fi

#target=tam3517

target=$3

case "${target}" in
tam3517)
	defconfig=tam3517_defconfig
	echo "usage tam3517_defconfig"
	sleep 5
;;

tam3517_upum6)
	defconfig=tam3517_upum6_defconfig
	echo "usage tam3517_upum6_defconfig"
	sleep 5
;;

*)
	echo "Config file is unknown"
    exit
esac

CROSS_COMPILE=${toolchain}

cd ${KERNEL_PATH}

make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" ${defconfig}
make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" -j${threads} uImage modules

cd ${WORK_DIR}
