#!/bin/bash
export CPUS=`grep -c processor /proc/cpuinfo`

function run_linux()
{
	qemu/qemu-system-aarch64 -M virt-cortex-a53,gic_version=3  \
	-smp cpus=4 -m 4096M  \
	-kernel kernel/output64/arch/arm64/boot/Image \
	-dtb kernel/output64/arch/arm64/boot/dts/arm/virt-a53-gicv3.dtb \
	-append "root=/dev/mmcblk0 console=ttyAMA0 rw dsched_debug" \
	-net user,hostfwd=tcp::5555-:22 \
	-net nic \
	-serial stdio \
	-sd buildroot/output/images/rootfs.ext2
}

function debug_linux()
{
	qemu/qemu-system-aarch64 -M virt-cortex-a53,gic_version=3  \
	-smp cpus=4 -m 4096M  \
	-kernel kernel/output64/arch/arm64/boot/Image \
	-dtb kernel/output64/arch/arm64/boot/dts/arm/virt-a53-gicv3.dtb \
	-append "root=/dev/mmcblk0 console=ttyAMA0 rw dsched_debug" \
	-net user,hostfwd=tcp::5555-:22 \
	-net nic \
	-serial stdio \
	-sd buildroot/output/images/rootfs.ext2 \
	-s \
	-S
}

function build_buildroot()
{
	cd buildroot
	./build.sh -b
	cd ..
}

function build_kernel()
{
	cd kernel
	./build_virt-a53.sh -b
	cd ..
}

function build_all()
{
	build_kernel
	build_buildroot
}

function usage()
{
	echo "Usage: build.sh [OPTIONS]"
	echo "Available options:"
	echo "kernel             -build kernel"
	echo "rootfs             -build buildroot rootfs"
	echo "all	 	 -build kernel & rootfs"
	echo "run		 - run qemu linux"
	echo "debug		 - debug qemu linux"
}

OPTIONS="${@:-all}"

for option in ${OPTIONS}; do
	echo "processing option: $option"
	case $option in
		kernel) build_kernel ;;
		all) build_all ;;
		rootfs) build_buildroot ;;
		run) run_linux;;
		debug) debug_linux;;
		*) usage ;;
	esac
done
