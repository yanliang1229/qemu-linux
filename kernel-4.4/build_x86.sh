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
			make x86_64_defconfig
			make -j${CPUS}
		;;
		-c)
			make clean 
		;;		
		-r)
			qemu-system-x86_64 -m 1024M -M q35 -cpu Nehalem -smp 8,cores=2,threads=2,sockets=2 \
			-numa node,mem=512M,cpus=0-3,nodeid=0 \
			-numa node,mem=512M,cpus=4-7,nodeid=1 \
			-kernel arch/x86_64/boot/bzImage \
			-drive format=raw,file=rootfs.ext2 \
			-append "console=ttyS0, 115200 init=/linuxrc root=/dev/sda" \
			--nographic
		;;
		-d)
			qemu-system-x86_64 -m 1024M -M q35 -cpu Nehalem -smp 8,cores=2,threads=2,sockets=2 \
			-numa node,mem=512M,cpus=0-3,nodeid=0 \
			-numa node,mem=512M,cpus=4-7,nodeid=1 \
			-kernel arch/x86_64/boot/bzImage \
			-drive format=raw,file=rootfs.ext2  \
			-append "console=ttyS0, 115200 init=/linuxrc root=/dev/sda" \
			--nographic -S -s
		;;

		*)
			;;
	esac
}

do_build_debug $1
