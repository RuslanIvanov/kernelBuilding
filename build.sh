#!/bin/sh

if [ X$1 = X ]
then
	echo Argument missing. Should be one of "thunder", "tsunami", "tsunamixl", "blizzard", "blizzardxl", "twister", "twisterxl", "thb", "tao_prism", "tdm_prism", or "tam_prism"
	exit 1
fi

product=$1
threads=`cat /proc/cpuinfo | grep processor | wc -l`
threads=`expr ${threads}`

WORK_DIR=$PWD
UBOOT_PATH=${WORK_DIR}/bootloader/u-boot
XLOADER_PATH=${WORK_DIR}/bootloader/x-loader
KERNEL_PATH=${WORK_DIR}/kernel
#ANDROID_AFS_DIR=${WORK_DIR}/gb_afs
#SGX_DIR=${WORK_DIR}/gb_sgx
#NANDINSTALLER_PATH=${WORK_DIR}/nand_installer
#ZIP_TARBALL=yes

target_os=angstrom
os_subversion=2.6.37
#target_os=android
#os_subversion=2.3.4

if [ ${target_os} = "android" ]
then
	toolchain=${ANDROID_AFS_DIR}/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
fi

if [ ${target_os} = "angstrom" ]
then
#	toolchain=$CROSS_COMPILE
toolchain=/home/russl1404/builds/buildroot_tam3517_psr/buildroot-2013.05/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabi-
	if [ X${toolchain} = X ]
	then
		echo "Toolchain not setup, please export CROSS_COMPILE to toolchain path and prefix, like \"export CROSS_COMPILE=/opt/arm-2010q1/bin/arm-none-linux-gnueabi-\""
		exit 1
	fi

fi

ccache_exist=`which ccache`
CCACHE="ccache "
if [ X${ccache_exist} = X ]
then
	KERNEL_CROSS_COMPILE=${toolchain}
	CROSS_COMPILE=${toolchain}
else
	KERNEL_CROSS_COMPILE=${CCACHE}${toolchain}
	CROSS_COMPILE=${CCACHE}${toolchain}
fi

#################################################
#
# Preparation
#
#################################################

case "${product}" in
thunder)
        target=tao3530
        lcdsize=4_3
        ;;
tsunami)
        target=tao3530
        lcdsize=4_3
        ;;
tsunamixl)
        target=tao3530
        lcdsize=auto
        ;;
blizzard)
        target=tdm3730
        lcdsize=4_3
        ;;
blizzardxl)
        target=tdm3730
        lcdsize=auto
        ;;
twister)
        target=tam3517
        lcdsize=4_3
        ;;
twisterxl)
        target=tam3517
        lcdsize=auto
        ;;
thb)
        target=tam3517
        lcdsize=auto
        ;;
tao_prism)
        target=tao3530
        lcdsize=auto
        ;;
tdm_prism)
        target=tdm3730
        lcdsize=auto
        ;;
tam_prism)
        target=tam3517
        lcdsize=auto
        ;;
*)
        echo Target product is unknown!! Should be one of "thunder", "tsunami", "tsunamixl", "blizzard", "blizzardxl", "twister", "twisterxl", "thb", "tao_prism", "tdm_prism", or "tam_prism"
        exit
esac

if [ ${target_os} = "angstrom" ]
then

case "${target}" in
tam3517)
#        omapes=3.x
	defconfig=tam3517_defconfig
        ;;
tao3530)
#        omapes=3.x
	defconfig=taotdm_defconfig
        ;;
tdm3730)
#        omapes=5.x
	defconfig=taotdm_defconfig
        ;;
*)
        echo Target product is unknown
        exit
esac

fi

if [ ${target_os} = "android" ]
then

case "${target}" in
tam3517)
        omapes=3.x
	defconfig=tam3517_android_defconfig
        ;;
tao3530)
        omapes=3.x
	defconfig=taotdm_android_defconfig
        ;;
tdm3730)
        omapes=5.x
	defconfig=taotdm_android_defconfig
        ;;
*)
        echo Target product is unknown
        exit
esac

fi

##############################################
#
#  Make XUK 
#
##############################################

cd ${XLOADER_PATH}
#make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" distclean
make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" ${target}_config
make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" -j${threads}
cd ${WORK_DIR}

cd ${UBOOT_PATH}
#make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" distclean
make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" -j${threads}  ${target}_envdef
cp u-boot.bin envdef-u-boot
#cd ../

#cd u-boot
make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" distclean
make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" -j${threads}  ${target}
cd ${WORK_DIR}

cd ${KERNEL_PATH}
#make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" distclean
make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" ${defconfig}
make ARCH=arm CROSS_COMPILE="${CROSS_COMPILE}" -j${threads} uImage modules
cd ${WORK_DIR}

if [ ${target_os} = "android" ]
then

##############################################
#
# Make Android
#
##############################################

cd ${ANDROID_AFS_DIR}

#make TARGET_PRODUCT=${target} clean
make TARGET_PRODUCT=${target} -j${threads}
cd ${WORK_DIR}

#############################################
#
# Make PowerVR
#
#############################################

cd ${SGX_DIR}/
rm -rf data
rm -rf system
cd sgx
make BUILD=release OMAPES=${omapes} ANDROID_ROOT_DIR=${ANDROID_AFS_DIR} TARGET_PRODUCT=${target} KERNELDIR=${KERNEL_PATH} QUICK_SGX=${SGX_DIR} clean
make BUILD=release OMAPES=${omapes} ANDROID_ROOT_DIR=${ANDROID_AFS_DIR} TARGET_PRODUCT=${target} KERNELDIR=${KERNEL_PATH} QUICK_SGX=${SGX_DIR} 
make BUILD=release OMAPES=${omapes} ANDROID_ROOT_DIR=${ANDROID_AFS_DIR} TARGET_PRODUCT=${target} KERNELDIR=${KERNEL_PATH} QUICK_SGX=${SGX_DIR} install
cd ${WORK_DIR}

################################################
#
# Install Target(Android FS)
#
################################################

rm -rf build/${product}
mkdir -p build/${product}
mkdir -p build/${product}/boot
mkdir -p build/${product}/rootfs
cp -rf ${ANDROID_AFS_DIR}/out/target/product/${target}/root/* build/${product}/rootfs/
cp -rfa ${ANDROID_AFS_DIR}/out/target/product/${target}/system build/${product}/rootfs/
cp -rfa ${ANDROID_AFS_DIR}/out/target/product/${target}/data build/${product}/rootfs/
cp -rfa ${SGX_DIR}/system build/${product}/rootfs/
cp -rfa ${SGX_DIR}/data build/${product}/rootfs/

mkdir -p build/${product}/rootfs/system/bin/wifi
cp ${KERNEL_PATH}/drivers/net/wireless/libertas/libertas_sdio.ko build/${product}/rootfs/system/bin/wifi/

mkdir -p build/${product}/rootfs/system/usr/idc
ln -sf /data/data/org.openaos.tscalibrator/files/ADS7846\ Touchscreen.idc build/${product}/rootfs/system/usr/idc/ADS7846\ Touchscreen.idc
cp APK/*.apk build/${product}/rootfs/data/app/

fi

if [ ${target_os} = "angstrom" ]
then

################################################
#
# Install Target(Linux FS)
#
################################################

mkdir -p build/${product}
mkdir -p build/${product}/boot
mkdir -p build/${product}/rootfs
cp -rfa linux_fs/rootfs/* build/${product}/rootfs/

case "${target}" in
tam3517)
       cp -rfa linux_fs/tam3517/* build/${product}/rootfs/
        ;;
tao3530)
       cp -rfa linux_fs/tao3530/* build/${product}/rootfs/
        ;;
tdm3730)
       cp -rfa linux_fs/tdm3730/* build/${product}/rootfs/
        ;;
*)
        echo Target product is unknown
        exit
esac

find ${KERNEL_PATH} -name \*.ko -exec cp {} build/${product}/rootfs/boot/ \;

fi

################################################
#
# Install Target(boot)
#
################################################

cd ${WORK_DIR}

cp ${XLOADER_PATH}/MLO build/${product}/boot/ 
cp ${KERNEL_PATH}/arch/arm/boot/uImage build/${product}/boot/

case "${lcdsize}" in
4_3)
        cat ${UBOOT_PATH}/u-boot.bin ${UBOOT_PATH}/tnsplash/logofile ${UBOOT_PATH}/tnsplash/timing043 > build/${product}/boot/u-boot.bin
        cat ${UBOOT_PATH}/envdef-u-boot ${UBOOT_PATH}/tnsplash/timing043 > build/${product}/boot/u-boot.bin.envdef
        rm ${UBOOT_PATH}/envdef-u-boot
        ;;
auto)
	 cat ${UBOOT_PATH}/u-boot.bin ${UBOOT_PATH}/tnsplash/logofile > build/${product}/boot/u-boot.bin
        cat ${UBOOT_PATH}/envdef-u-boot > build/${product}/boot/u-boot.bin.envdef
        rm ${UBOOT_PATH}/envdef-u-boot
        ;;
*)
        cat ${UBOOT_PATH}/u-boot.bin ${UBOOT_PATH}/tnsplash/logofile > build/${product}/boot/u-boot.bin
        cat ${UBOOT_PATH}/envdef-u-boot > build/${product}/boot/u-boot.bin.envdef
        rm ${UBOOT_PATH}/envdef-u-boot
esac



################################################
# Build Target Version and Date
################################################

echo ${target_os}-${os_subversion}-${product}-`date +"%Y-%m-%d"` > build/${product}/version_date.txt

################################################
################################################



if [ X${NANDINSTALLER_PATH} != X ]
then

################################################
#
# Make Nand Installer Image
#
################################################

cd ${NANDINSTALLER_PATH}
rm *.img
tar xvf disk_img.tar.xz
umount /dev/loop*
losetup -d /dev/loop*
losetup /dev/loop0 boot.img
losetup /dev/loop1 rootfs.img
mount -t vfat /dev/loop0 boot
mount -t ext3 /dev/loop1 rootfs
cd ${WORK_DIR}
cp build/${product}/boot/MLO ${NANDINSTALLER_PATH}/boot/
cp build/${product}/boot/u-boot.bin ${NANDINSTALLER_PATH}/boot/NAND-u-boot.bin
cat build/${product}/boot/u-boot.bin.envdef ${NANDINSTALLER_PATH}/nanderase.logo > ${NANDINSTALLER_PATH}/boot/u-boot.bin
cp build/${product}/boot/uImage ${NANDINSTALLER_PATH}/boot/uImage
cp -rfa build/${product}/rootfs/* ${NANDINSTALLER_PATH}/rootfs/home/root/ubi/
sync
umount /dev/loop*
losetup -d /dev/loop*
cat ${NANDINSTALLER_PATH}/mbr.img ${NANDINSTALLER_PATH}/boot.img ${NANDINSTALLER_PATH}/rootfs.img > build/${product}/${target_os}-${product}-`date +"%Y-%m-%d"`-nand.img

if [ X${ZIP_TARBALL} != X ]
then
	zip ${WORK_DIR}/build/${target_os}-${product}-`date +"%Y-%m-%d"`-nand.zip ${WORK_DIR}/build/${product}/${target_os}-${product}-`date +"%Y-%m-%d"`-nand.img ${NANDINSTALLER_PATH}/version_date.txt ${NANDINSTALLER_PATH}/making-a-rescue-sd.pdf -j
fi

fi

echo
echo Product ${product} built.
echo
echo Files are in cd build/${product}/
echo 
echo
