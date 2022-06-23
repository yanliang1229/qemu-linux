#!/bin/bash
export CPUS=`grep -c processor /proc/cpuinfo`

do_build_debug()
{
	if [ $# -lt 1 ]; then
		echo "input few args ..."
		exit
	fi
	case $1 in
		-b)
			make  qemu_aarch64_virt_defconfig
			make  -j${CPUS}
		;;
		-r)
			qemu/qemu-system-aarch64 -M virt-cortex-a53,gic_version=3  \
			-smp cpus=4 -m 4096M  \
			-kernel output/images/Image \
			-dtb output/images/virt-a53-gicv3.dtb \
			-append "root=/dev/mmcblk0 console=ttyAMA0 rw dsched_debug" \
			-net user,hostfwd=tcp::5555-:22 \
			-net nic \
			-serial stdio \
			-sd output/images/rootfs.ext2
		;;
		-d)
			qemu/qemu-system-aarch64 -M virt-cortex-a53,gic_version=2  \
			-smp cpus=4 -m 4096M  \
			-kernel output/images/boot/Image \
			-dtb output/images/virt-a53-gicv2.dtb \
			-append "root=/dev/mmcblk0 console=ttyAMA0 rw dsched_debug" \
			-serial stdio \
			-S -s \
			-sd output/images/rootfs.ext2
		;;
		*)
		;;
	esac
}


do_build_debug $1
