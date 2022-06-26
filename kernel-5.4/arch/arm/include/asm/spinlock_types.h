/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_SPINLOCK_TYPES_H
#define __ASM_SPINLOCK_TYPES_H

#ifndef __LINUX_SPINLOCK_TYPES_H
# error "please don't include this file directly"
#endif

#define TICKET_SHIFT	16

/**
 * 自旋锁的实现原理:
 * owner就是当前已经获得自旋锁的那个号码，next记录的是下一个要分发的号码。
 *
 * 例如:
 *   最开始的时候，slock被赋值为0，也就是说owner和next都是0，owner和next相等，
 *   表示unlocked。当thread 0调用spin_lock来申请lock的时候，owner和next相等，
 *   表示unlocked，这时候该thread 0持有该spin lock，并且执行next++，也就是将
 *   next设定为1, 也许该thread 0 执行很快,没有其他thread来竞争, 就调用spin_unlock了,
 *   这时候执行owner++，也就是将owner设定为1,下一个thread 1获得了直接进入的机会，
 *   next++之后等于2。thread 1号执行很慢（thread不能持有spin lock太久,但是存在)
 *   又来thread 3尝试获得lock, 分配当前next值的号码2，当然也会执行next++，以便下一个
 *   thread获得3的号码牌。持续来的thread就会分配3、4、5、6这些号码牌，next值不断的增加
 *   但是owner岿然不动，直到欠扁的 thread 1号离开临界区（调用spin_unlock），owner++之
 *   后等于2，表示持有2那个 thread的就可以获得自旋锁了.
 */
typedef struct {
	union {
		u32 slock;
		struct __raw_tickets {
#ifdef __ARMEB__
			u16 next;
			u16 owner;
#else
			u16 owner;
			u16 next;
#endif
		} tickets;
	};
} arch_spinlock_t;

#define __ARCH_SPIN_LOCK_UNLOCKED	{ { 0 } }

typedef struct {
	u32 lock;
} arch_rwlock_t;

#define __ARCH_RW_LOCK_UNLOCKED		{ 0 }

#endif
