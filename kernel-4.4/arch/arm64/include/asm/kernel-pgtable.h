/*
 * Kernel page table mapping
 *
 * Copyright (C) 2015 ARM Ltd.
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

#ifndef __ASM_KERNEL_PGTABLE_H
#define __ASM_KERNEL_PGTABLE_H


/*
 * The linear mapping and the start of memory are both 2M aligned (per
 * the arm64 booting.txt requirements). Hence we can use section mapping
 * with 4K (section size = 2M) but not with 16K (section size = 32M) or
 * 64K (section size = 512M).
 * section map就是指使用2M的为单位进行映射,当然不是什么情况都是可以使用section map，
 * 对于kernel image，其起始地址是2M对齐的，因此block size是2M的情况下才OK，对于
 * PAGE SIZE是16K，其Block descriptor指向了一个32M的内存块，PAGE SIZE是64K的时候，
 * Block descriptor指向了一个512M的内存块. 因此，只有4K page size的情况下，才可以
 * 启用section map
 */
#ifdef CONFIG_ARM64_4K_PAGES
#define ARM64_SWAPPER_USES_SECTION_MAPS 1
#else
#define ARM64_SWAPPER_USES_SECTION_MAPS 0
#endif

/*
 * The idmap and swapper page tables need some space reserved in the kernel
 * image. Both require pgd, pud (4 levels only) and pmd tables to (section)
 * map the kernel. With the 64K page configuration, swapper and idmap need to
 * map to pte level. The swapper also maps the FDT (see __create_page_tables
 * for more information). Note that the number of ID map translation levels
 * could be increased on the fly if system RAM is out of reach for the default
 * VA range, so pages required to map highest possible PA are reserved in all
 * cases.
 */
#if ARM64_SWAPPER_USES_SECTION_MAPS
/**
 * 对于ARM64，由于虚拟地址空间变大了，因此我们需要更多的page来完成启动阶段的initial translation tables的构建
 * 我们仍然用VA是48 bit，page size是4K为例子进行说明。根据前面的描述，我们知道，内核空间的地址大小是256T，48 bit
 * 的地址被分成9 ＋ 9 ＋ 9 ＋ 9 ＋ 12，因此PGD（Level 0）、PUD（Level 1）、PMD（Level 2）、PT（Level 3）的
 * translation table中的entry都是512项，每个描述符是8个byte，因此这些translation table都是4KB，恰好是一个page size。
 * 根据链接脚本中的定义，idmap和swapper page tables （或者叫做translation table）分别保留了3个page的页面。3个page分别
 * 是3个level的translation table。等等，读者可能会问：上面不是说48 bit VA加上4K page size需要4阶translation table吗？
 * 这里怎么只有3个level？实际上，3级映射是PGD/PUM/PMD（每个table占据一个page），只不过PMD的内容不是下一级的table descriptor，
 * 而是基于2M block的mapping
 */
#define SWAPPER_PGTABLE_LEVELS	(CONFIG_PGTABLE_LEVELS - 1)
#define IDMAP_PGTABLE_LEVELS	(ARM64_HW_PGTABLE_LEVELS(PHYS_MASK_SHIFT) - 1)
#else
#define SWAPPER_PGTABLE_LEVELS	(CONFIG_PGTABLE_LEVELS)
#define IDMAP_PGTABLE_LEVELS	(ARM64_HW_PGTABLE_LEVELS(PHYS_MASK_SHIFT))
#endif

#define SWAPPER_DIR_SIZE	(SWAPPER_PGTABLE_LEVELS * PAGE_SIZE)
#define IDMAP_DIR_SIZE		(IDMAP_PGTABLE_LEVELS * PAGE_SIZE)

/* Initial memory map size */
#if ARM64_SWAPPER_USES_SECTION_MAPS
#define SWAPPER_BLOCK_SHIFT	SECTION_SHIFT
#define SWAPPER_BLOCK_SIZE	SECTION_SIZE
#define SWAPPER_TABLE_SHIFT	PUD_SHIFT
#else
#define SWAPPER_BLOCK_SHIFT	PAGE_SHIFT
#define SWAPPER_BLOCK_SIZE	PAGE_SIZE
#define SWAPPER_TABLE_SHIFT	PMD_SHIFT
#endif

/* The size of the initial kernel direct mapping */
#define SWAPPER_INIT_MAP_SIZE	(_AC(1, UL) << SWAPPER_TABLE_SHIFT)

/*
 * Initial memory map attributes.
 */
#define SWAPPER_PTE_FLAGS	(PTE_TYPE_PAGE | PTE_AF | PTE_SHARED)
#define SWAPPER_PMD_FLAGS	(PMD_TYPE_SECT | PMD_SECT_AF | PMD_SECT_S)

#if ARM64_SWAPPER_USES_SECTION_MAPS
/* section映射属性值 */
#define SWAPPER_MM_MMUFLAGS	(PMD_ATTRINDX(MT_NORMAL) | SWAPPER_PMD_FLAGS)
#else
#define SWAPPER_MM_MMUFLAGS	(PTE_ATTRINDX(MT_NORMAL) | SWAPPER_PTE_FLAGS)
#endif


#endif	/* __ASM_KERNEL_PGTABLE_H */
