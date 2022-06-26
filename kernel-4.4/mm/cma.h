#ifndef __MM_CMA_H__
#define __MM_CMA_H__

/*
 * CMA是保留的一块内存,用于分配连续的大块内存,当设备驱动不用时，内存
 * 管理系统将该区域用于分配和管理可移动类型的页面,当设备驱动使用时,
 * 此时已经分配的页面需要进行迁移,又用于连续内存的分配.
 * @order_per_bit: 如果等于0,表示按照一个一个页面来分配和释放. 如果等于
 * 1，表示安装2^1的页面分配和释放, 以此类推.
 */
struct cma {
	unsigned long   base_pfn;	/* CMA区域的起始物理页号 */
	unsigned long   count;		/* CMA区域总的页数 */
	unsigned long   *bitmap;	/* 位图，管理CMA页框的分配情况 */
	unsigned int order_per_bit; /* Order of pages represented by one bit */
	struct mutex    lock;
#ifdef CONFIG_CMA_DEBUGFS
	struct hlist_head mem_head;
	spinlock_t mem_head_lock;
#endif
};

extern struct cma cma_areas[MAX_CMA_AREAS];
extern unsigned cma_area_count;

static inline unsigned long cma_bitmap_maxno(struct cma *cma)
{
	return cma->count >> cma->order_per_bit;
}

#endif
