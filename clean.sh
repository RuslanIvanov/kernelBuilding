#!/bin/sh

#toolchain=$PWD/gb_afs/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
#toolchain=${CROSS_COMPILE}
toolchain=/home/russl1404/builds/buildroot_tam3517_psr/buildroot-2013.05/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabi-

if [ X${toolchain} = X ]
then
	echo "Toolchain not setup, please export CROSS_COMPILE to toolchain path and prefix, like \"export CROSS_COMPILE=/opt/arm-2010q1/bin/arm-none-linux-gnueabi-\""
	exit 1
fi

WORK_DIR=$PWD

CROSS_COMPILE=${toolchain}
KERNEL_CROSS_COMPILE=${toolchain}

cd ${WORK_DIR}/bootloader/x-loader
make ARCH=arm CROSS_COMPILE="${toolchain}" distclean

echo
echo Xloader Cleaned
echo 

cd ${WORK_DIR}/bootloader/u-boot
make ARCH=arm CROSS_COMPILE="${toolchain}" distclean
rm envdef-u-boot.bin

echo 
echo U-boot Cleaned
echo

cd ${WORK_DIR}/kernel
make ARCH=arm CROSS_COMPILE="${toolchain}" distclean

echo
echo Kernel Cleaned 
echo

exit 0

cd ${WORK_DIR}/gb_afs

rm -rf out

echo
echo Android Cleaned
echo

cd ${WORK_DIR}/gb_sgx
rm -rf data
rm -rf system
cd sgx
make BUILD=release OMAPES=3.x ANDROID_ROOT_DIR=${WORK_DIR}/gb_afs TARGET_PRODUCT=${target} KERNELDIR=${WORK_DIR}/kernel/linux-2.6.37-tn QUICK_SGX=${WORK_DIR}/gb_sgx clean
make BUILD=release OMAPES=5.x ANDROID_ROOT_DIR=${WORK_DIR}/gb_afs TARGET_PRODUCT=${target} KERNELDIR=${WORK_DIR}/kernel/linux-2.6.37-tn QUICK_SGX=${WORK_DIR}/gb_sgx clean

echo
echo SGX Cleaned
echo
echo All Cleaned
echo

cd ${WORK_DIR}
