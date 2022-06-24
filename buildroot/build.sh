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
		*)
		;;
	esac
}


do_build_debug $1
