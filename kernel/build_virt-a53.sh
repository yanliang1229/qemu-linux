#!/bin/bash
export CPUS=`grep -c processor /proc/cpuinfo`
export ARCH=arm64
export CROSS_COMPILE=/opt/gcc-linaro-5.4.1-2017.01-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
export KBUILD_BUILD_USER=linux
export KBUILD_BUILD_HOST=qemu

OUTPUT_PATH=output64

do_build_debug()
{
	if [ $# -lt 1 ]; then
		echo "input few args ..."
		exit
	fi
	case $1 in
		-b)
			make  O=${OUTPUT_PATH} qemu_defconfig
			make  O=${OUTPUT_PATH} -j${CPUS}
		;;
		-r)
			qemu-system-aarch64 -M virt-cortex-a53  \
			-smp cpus=4 -m 4096M  \
			-kernel ${OUTPUT_PATH}/arch/arm64/boot/Image \
                        -dtb ${OUTPUT_PATH}/arch/arm64/boot/dts/arm/virt-a53-gicv3.dtb \
			-append "root=/dev/mmcblk0 earlycon=pl011,0x9000000 console=ttyAMA0 rw dsched_debug" \
			-serial stdio \
			-show-cursor \
			-net user,hostfwd=tcp::5555-:22 \
			-net nic \
			-usb \
			-drive if=none,format=raw,id=disk1,file=udisk.img \
			-device usb-storage,drive=disk1 \
			-sd ../output/images/rootfs.ext2
		;;
		-d)
			qemu-system-aarch64 -M virt-cortex-a53 -nographic \
			-smp cpus=4 -m 4096M  \
			-kernel ${OUTPUT_PATH}/arch/arm64/boot/Image \
                        -dtb ${OUTPUT_PATH}/arch/arm64/boot/dts/arm/virt-a53-gicv3.dtb \
			-append "root=/dev/mmcblk0 earlycon=pl011,0x9000000 console=ttyAMA0  rw dsched_debug" \
			-s -S  \
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
			make -C output clean
		;;
		*)
		;;
	esac
}


do_build_debug $1
