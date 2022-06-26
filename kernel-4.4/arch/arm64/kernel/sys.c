/*
 * AArch64-specific system calls implementation
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Catalin Marinas <catalin.marinas@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/compiler.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>

/**
 * @addr: 可选的线性地址, 指定MA_FIXED时，必须从他开始映射
 * @len: 映射长度
 * @prot: 映射线性区的访问权限(PROT_READ, PROT_WRITE, PROT_EXEC)
 * @flags: 映射标记
 *  MAP_SHARED: 用于多个进程共享对一个文件的访问
 *  MAP_PRIVATE: 用于创建一个与数据源分离的私有映射，对区域的写入操作不影响数据源文件中的内容
 *  MAP_FIXED: 用于在指定的目标线性地址创建一个映射，不允许调整到其他地址
 *  MAP_ANONYMOUS: 用于创建与文件无关的映射，或者说没有数据源的映射
 * @fd: 文件映射时的文件描述符
 * @offset: 文件映射的起始地址
 * @return: 返回新线性区中第一个单元位置的起始地址
 *
 * 用户空间创建匿名映射的方法:
 *   int *p = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS, -1, 0);
 */
asmlinkage long sys_mmap(unsigned long addr, unsigned long len,
			 unsigned long prot, unsigned long flags,
			 unsigned long fd, off_t off)
{
	/* off必须是页的整数倍 */
	if (offset_in_page(off) != 0)
		return -EINVAL;

	return sys_mmap_pgoff(addr, len, prot, flags, fd, off >> PAGE_SHIFT);
}

/*
 * Wrappers to pass the pt_regs argument.
 */
asmlinkage long sys_rt_sigreturn_wrapper(void);
#define sys_rt_sigreturn	sys_rt_sigreturn_wrapper

#undef __SYSCALL
#define __SYSCALL(nr, sym)	[nr] = sym,

/*
 * The sys_call_table array must be 4K aligned to be accessible from
 * kernel/entry.S.
 */
void * const sys_call_table[__NR_syscalls] __aligned(4096) = {
	[0 ... __NR_syscalls - 1] = sys_ni_syscall,
#include <asm/unistd.h>
};
