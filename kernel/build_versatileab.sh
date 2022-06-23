#!/bin/bash

export CPUS=`grep -c processor /proc/cpuinfo`
export ARCH=arm
export CROSS_COMPILE=/opt/gcc-linaro-5.5.0-2017.10-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-
OUTPUT_PATH=output

do_build_debug()
{
	if [ $# -lt 1 ]; then
		echo "input few args ..."
		exit
	fi
	case $1 in
		-b)
			make  O=${OUTPUT_PATH} versatileab_defconfig
			make  O=${OUTPUT_PATH} -j${CPUS}
		;;
		-c)
			make clean
		;;
		-r)
			qemu-system-arm -M versatileab  \
			-kernel ${OUTPUT_PATH}/arch/arm/boot/zImage \
			-dtb ${OUTPUT_PATH}/arch/arm/boot/dts/arm/versatile-ab.dtb \
			-append "console=ttyAMA0,115200 root=/dev/mmcblk0 rw rootwait" \
			-sd rootfs_arm32.ext2 \
			-serial stdio  -show-cursor
		;;

		-d)
			qemu-system-arm -M versatileab \
			-kernel ${OUTPUT_PATH}/arch/arm/boot/zImage \
			-dtb ${OUTPUT_PATH}/arch/arm/boot/dts/arm/versatile-ab.dtb \
			-sd rootfs_arm32.ext2 \
			-append "console=ttyAMA0,115200 root=/dev/mmcblk0 rw rootwait" \
			-serial stdio  -show-cursor \
			-S -s
		;;
		*)
			;;
	esac
}

do_build_debug $1
