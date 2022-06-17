#!/bin/bash
export CPUS=`grep -c processor /proc/cpuinfo`
export ARCH=arm64
export CROSS_COMPILE=/opt/gcc-linaro-5.4.1-2017.01-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
OUTPUT_PATH=output

do_build_debug()
{
	if [ $# -lt 1 ]; then
		echo "input few args ..."
		exit
	fi
	case $1 in
		-b)
			make  O=output_arm64 qemu_defconfig
			make  O=output_arm64 -j${CPUS}
		;;
		-r)
			qemu-system-aarch64 -M virt-cortex-a53,gic_version=2  \
			-smp cpus=4 -m 4096M  \
			-kernel output_arm64/arch/arm64/boot/Image \
			-dtb output_arm64/arch/arm64/boot/dts/arm/virt-a53-gicv2.dtb \
			-append "root=/dev/mmcblk0 console=ttyAMA0 rootfstype=toy rw dsched_debug" \
			-net user,hostfwd=tcp::5555-:22 \
			-net nic \
			-serial stdio \
			-sd ../output/images/rootfs.ext2
		;;
		-d)
			qemu-system-aarch64 -M virt-cortex-a53,gic_version=2  \
			-smp cpus=4 -m 4096M  \
			-kernel output_arm64/arch/arm64/boot/Image \
			-dtb output_arm64/arch/arm64/boot/dts/arm/virt-a53-gicv2.dtb \
			-append "root=/dev/mmcblk0 console=ttyAMA0 rootfstype=toy rw dsched_debug" \
			-serial stdio \
			-S -s \
			-sd ../output/images/rootfs.ext2
		;;
		-f)
			echo "===== make rootfs ========="
			dd if=/dev/zero of=rootfs/rootfs.ext2 bs=1M count=4096
			mkfs.ext2 -i 4096 rootfs/rootfs.ext2
			sudo mount -t ext2 rootfs/rootfs.ext2 /mnt
			sudo cp -rfa rootfs/ubuntu-rootfs/* /mnt
			sudo umount /mnt
			echo "===== make rootfs done====="
		;;
		-c)
			make -C output_arm64 clean
		;;
		*)
		;;
	esac
}


do_build_debug $1
