/*
 *  Deadline i/o scheduler.
 *
 *  Copyright (C) 2002 Jens Axboe <axboe@kernel.dk>
 *
 * 引入原因:
 *  早期的I/O调度算法总是按照扇区顺序将请求插入到队列，从不检查驻留时间过长的请求，
 *  所以，它虽然能让寻址时间最短，但是却会带来请求饥饿的问题。对磁盘某一个局部区域
 *  的持续请求流，会造成较远位置的其他请求永远得不到运行机会，这就是所谓的饥饿现象。
 *  最后期限调度算法试图解决这个问题
 *
 * 原理:
 *  1. 在最后期限调度算法中，每个请求都有一个超时时间。默认情况下，读请求的超时时间是
 *     500毫秒，写请求的超时时间是5秒。最后期限I/O调度算法一方面以请求作用的扇区编号
 *     为次序来维护请求，组成排序队列。另一方面也按请求到达的时间为次序来维护请求，组
 *     成FIFO（先来先服务）队列;
 *  2. 一般情况下，请求从排序队列中取下，再推入到派发队列，这试图使物理磁盘的寻道操作次
 *     数和幅度最小。但是，如果FIFO队列头部有请求超时（也就是，当前时间超过了请求指定的
 *     超时时间），那么最后期限I/O调度程序便从FIFO队列中取出请求进行服务。依靠这种方法，
 *     最后期限I/O调度程序试图保证不会发生有请求在明显超时的情况下仍不能得到服务的现象
 */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/rbtree.h>

/*
 * See Documentation/block/deadline-iosched.txt
 */
static const int read_expire = HZ / 2;  /* max time before a read is submitted. */
static const int write_expire = 5 * HZ; /* ditto for writes, these limits are SOFT! */
static const int writes_starved = 2;    /* max times reads can starve a write */
static const int fifo_batch = 16;       /* # of sequential requests treated as one
				     by the above parameters. For throughput. */

struct deadline_data {
	/*
	 * run time data
	 */

	/*
	 * requests (deadline_rq s) are present on both sort_list and fifo_list
	 */
	struct rb_root sort_list[2];	/* 请求以磁盘扇区号为基准的排序队列  */
	struct list_head fifo_list[2];	/* 请求加入到队列时的管理队列 */

	/*
	 * next in sort order. read, write or both are NULL
	 */
	struct request *next_rq[2];	/* 指向按扇区编号增加方向下一个请求的指针，如果为NULL, 表示电梯已走到顶部 */
	unsigned int batching;		/* number of sequential requests made 连续提交请求的次数 */
	sector_t last_sector;		/* head position */
	unsigned int starved;		/* times reads have starved writes (提交读请求造成写请求饥饿的次数) */

	/*
	 * settings that change how the i/o scheduler behaves
	 */
	int fifo_expire[2];
	int fifo_batch;
	int writes_starved;
	int front_merges;		/* 是否允许向前合并 */
};

static void deadline_move_request(struct deadline_data *, struct request *);

static inline struct rb_root *
deadline_rb_root(struct deadline_data *dd, struct request *rq)
{
	return &dd->sort_list[rq_data_dir(rq)];
}

/*
 * get the request after `rq' in sector-sorted order
 */
static inline struct request *
deadline_latter_request(struct request *rq)
{
	struct rb_node *node = rb_next(&rq->rb_node);

	if (node)
		return rb_entry_rq(node);

	return NULL;
}

static void
deadline_add_rq_rb(struct deadline_data *dd, struct request *rq)
{
	struct rb_root *root = deadline_rb_root(dd, rq);

	elv_rb_add(root, rq);
}

static inline void
deadline_del_rq_rb(struct deadline_data *dd, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);

	if (dd->next_rq[data_dir] == rq)
		dd->next_rq[data_dir] = deadline_latter_request(rq);

	elv_rb_del(deadline_rb_root(dd, rq), rq);
}

/*
 * add rq to rbtree and fifo
 * 同一个请求被加入到两种管理结构中
 */
static void
deadline_add_request(struct request_queue *q, struct request *rq)
{
	struct deadline_data *dd = q->elevator->elevator_data;
	const int data_dir = rq_data_dir(rq);
	/* 加入以请求扇区编号为key的红黑树中 */
	deadline_add_rq_rb(dd, rq);

	/*
	 * set expire time and add to fifo list
	 * 设置请求的超时时间
	 */
	rq->fifo_time = jiffies + dd->fifo_expire[data_dir];
	/* 加入FIFO 链表队列中 */
	list_add_tail(&rq->queuelist, &dd->fifo_list[data_dir]);
}

/*
 * remove rq from rbtree and fifo.
 */
static void deadline_remove_request(struct request_queue *q, struct request *rq)
{
	struct deadline_data *dd = q->elevator->elevator_data;

	rq_fifo_clear(rq);
	deadline_del_rq_rb(dd, rq);
}

static int
deadline_merge(struct request_queue *q, struct request **req, struct bio *bio)
{
	struct deadline_data *dd = q->elevator->elevator_data;
	struct request *__rq;
	int ret;

	/*
	 * check for front merge
	 * 是否允许向前合并
	 */
	if (dd->front_merges) {
		sector_t sector = bio_end_sector(bio);

		__rq = elv_rb_find(&dd->sort_list[bio_data_dir(bio)], sector);
		if (__rq) {
			BUG_ON(sector != blk_rq_pos(__rq));

			if (elv_rq_merge_ok(__rq, bio)) {
				ret = ELEVATOR_FRONT_MERGE;
				goto out;
			}
		}
	}

	return ELEVATOR_NO_MERGE;
out:
	*req = __rq;
	return ret;
}

static void deadline_merged_request(struct request_queue *q,
				    struct request *req, int type)
{
	struct deadline_data *dd = q->elevator->elevator_data;

	/*
	 * if the merge was a front merge, we need to reposition request
	 * 合并操作，改变了req的值(bio被合并进来), req重新插入到红黑树中
	 */
	if (type == ELEVATOR_FRONT_MERGE) {
		elv_rb_del(deadline_rb_root(dd, req), req);
		deadline_add_rq_rb(dd, req);
	}
}

static void
deadline_merged_requests(struct request_queue *q, struct request *req,
			 struct request *next)
{
	/*
	 * if next expires before rq, assign its expire time to rq
	 * and move into next position (next will be deleted) in fifo
	 */
	if (!list_empty(&req->queuelist) && !list_empty(&next->queuelist)) {
		/**
		 * 如果next的期限时间小于req, 也就是说, next马上就要到期了, 需要先响应
		 * 并且将req放置到next在fifo的位置，因为next将要被删除
		 */
		if (time_before(next->fifo_time, req->fifo_time)) {
			/* req请求移动到next请求所在的队列 */
			list_move(&req->queuelist, &next->queuelist);
			/* 重新设置req请求的超时时间 */
			req->fifo_time = next->fifo_time;
		}
	}

	/*
	 * kill knowledge of next, this one is a goner
	 * 将next从链表和红黑树中删除
	 */
	deadline_remove_request(q, next);
}

/*
 * move request from sort list to dispatch queue.
 */
static inline void
deadline_move_to_dispatch(struct deadline_data *dd, struct request *rq)
{
	struct request_queue *q = rq->q;

	deadline_remove_request(q, rq);
	elv_dispatch_add_tail(q, rq);
}

/*
 * move an entry to dispatch queue
 */
static void
deadline_move_request(struct deadline_data *dd, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);

	dd->next_rq[READ] = NULL;
	dd->next_rq[WRITE] = NULL;
	dd->next_rq[data_dir] = deadline_latter_request(rq);

	dd->last_sector = rq_end_sector(rq);

	/*
	 * take it off the sort and fifo list, move
	 * to dispatch queue
	 */
	deadline_move_to_dispatch(dd, rq);
}

/*
 * deadline_check_fifo returns 0 if there are no expired requests on the fifo,
 * 1 otherwise. Requires !list_empty(&dd->fifo_list[data_dir])
 */
static inline int deadline_check_fifo(struct deadline_data *dd, int ddir)
{
	struct request *rq = rq_entry_fifo(dd->fifo_list[ddir].next);

	/*
	 * rq is expired!
	 * 检查是否存在被饿死的请求
	 */
	if (time_after_eq(jiffies, rq->fifo_time))
		return 1;

	return 0;
}

/*
 * deadline_dispatch_requests selects the best request according to
 * read/write expire, fifo_batch, etc
 * 选择最佳的请求给request_queue队列
 */
static int deadline_dispatch_requests(struct request_queue *q, int force)
{
	struct deadline_data *dd = q->elevator->elevator_data;
	const int reads = !list_empty(&dd->fifo_list[READ]);
	const int writes = !list_empty(&dd->fifo_list[WRITE]);
	struct request *rq;
	int data_dir;

	/*
	 * batches are currently reads XOR writes
	 */
	if (dd->next_rq[WRITE])
		rq = dd->next_rq[WRITE];
	else
		rq = dd->next_rq[READ];

	/* 如果批量请求处理存在，并且还没有达到批量请求处理的上限值，那么继续请求的批量处理 */
	if (rq && dd->batching < dd->fifo_batch)
		/* we have a next request are still entitled to batch */
		goto dispatch_request;

	/*
	 * at this point we are not running a batch. select the appropriate
	 * data direction (read / write)
	 * 先处理读队列
	 */
	if (reads) {
		BUG_ON(RB_EMPTY_ROOT(&dd->sort_list[READ]));
		/* 如果写请求队列存在饿死的现象，那么优先处理写请求队列 */
		if (writes && (dd->starved++ >= dd->writes_starved))
			goto dispatch_writes;

		data_dir = READ;

		goto dispatch_find_request;
	}

	/*
	 * there are either no reads or writes have been starved
	 * 没有读请求，或者写请求有饿死现象
	 */
	if (writes) {
dispatch_writes:
		BUG_ON(RB_EMPTY_ROOT(&dd->sort_list[WRITE]));

		dd->starved = 0;

		data_dir = WRITE;

		goto dispatch_find_request;
	}

	return 0;

dispatch_find_request:
	/*
	 * we are not running a batch, find best request for selected data_dir
	 * 如果请求队列中存在即将饿死的request，或者不存在需要批量处理的请求，那么从FIFO队列头获取一个request
	 */
	if (deadline_check_fifo(dd, data_dir) || !dd->next_rq[data_dir]) {
		/*
		 * A deadline has expired, the last request was in the other
		 * direction, or we have run out of higher-sectored requests.
		 * Start again from the request with the earliest expiry time.
		 */
		rq = rq_entry_fifo(dd->fifo_list[data_dir].next);
	} else {
		/*
		 * 继续批量处理，获取需要批量处理的下一个request
		 * The last req was the same dir and we have a next request in
		 * sort order. No expired requests so continue on from here.
		 */
		rq = dd->next_rq[data_dir];
	}

	dd->batching = 0;

dispatch_request:
	/*
	 * rq is the selected appropriate request.
	 * 将request从dd中删除，并派发到请求队列中
	 */
	dd->batching++;
	deadline_move_request(dd, rq);

	return 1;
}

static void deadline_exit_queue(struct elevator_queue *e)
{
	struct deadline_data *dd = e->elevator_data;

	BUG_ON(!list_empty(&dd->fifo_list[READ]));
	BUG_ON(!list_empty(&dd->fifo_list[WRITE]));

	kfree(dd);
}

/*
 * initialize elevator private data (deadline_data).
 */
static int deadline_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct deadline_data *dd;
	struct elevator_queue *eq;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	dd = kzalloc_node(sizeof(*dd), GFP_KERNEL, q->node);
	if (!dd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	/* 设置IO调度队列的私有数据 */
	eq->elevator_data = dd;

	INIT_LIST_HEAD(&dd->fifo_list[READ]);
	INIT_LIST_HEAD(&dd->fifo_list[WRITE]);
	dd->sort_list[READ] = RB_ROOT;
	dd->sort_list[WRITE] = RB_ROOT;
	/* 默认设置读写请求的超时时间 */
	dd->fifo_expire[READ] = read_expire;
	dd->fifo_expire[WRITE] = write_expire;
	dd->writes_starved = writes_starved;
	dd->front_merges = 1;
	dd->fifo_batch = fifo_batch;

	/* 请求队列，关联IO调度队列 */
	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

/*
 * sysfs parts below
 */
static ssize_t
deadline_var_show(int var, char *page)
{
	return sprintf(page, "%d\n", var);
}

static ssize_t
deadline_var_store(int *var, const char *page, size_t count)
{
	char *p = (char *) page;

	*var = simple_strtol(p, &p, 10);
	return count;
}

static ssize_t deadline_read_expire_show(struct elevator_queue *e, char *page)
{
	struct deadline_data *dd = e->elevator_data;
	int __data =  dd->fifo_expire[READ];
	__data = jiffies_to_msecs(__data);
	return deadline_var_show(__data, (page));
}


static ssize_t deadline_write_expire_show(struct elevator_queue *e, char *page)
{
	struct deadline_data *dd = e->elevator_data;
	int __data = dd->fifo_expire[WRITE];
	__data = jiffies_to_msecs(__data);
	return deadline_var_show(__data, (page));
}

static ssize_t deadline_writes_starved_show(struct elevator_queue *e, char *page)
{
	struct deadline_data *dd = e->elevator_data;
	int __data = dd->writes_starved;

	return deadline_var_show(__data, (page));
}

static ssize_t deadline_front_merges_show(struct elevator_queue *e, char *page)
{
	struct deadline_data *dd = e->elevator_data;
	int __data = dd->front_merges;

	return deadline_var_show(__data, (page));
}

static ssize_t deadline_fifo_batch_show(struct elevator_queue *e, char *page)
{
	struct deadline_data *dd = e->elevator_data;
	int __data = dd->fifo_batch;

	return deadline_var_show(__data, (page));
}

/* 修改读超时 */
static ssize_t deadline_read_expire_store(struct elevator_queue *e, const char *page, size_t count)
{
	struct deadline_data *dd = e->elevator_data;
	int __data;
	int ret = deadline_var_store(&__data, (page), count);
	if (__data < 0)
		__data = 0;
	else if (__data > INT_MAX)
		__data = INT_MAX;

	dd->fifo_expire[READ] = msecs_to_jiffies(__data);

	return ret;
}

/* 修改写超时 */
static ssize_t deadline_write_expire_store(struct elevator_queue *e, const char *page, size_t count)
{
	struct deadline_data *dd = e->elevator_data;
	int __data;
	int ret = deadline_var_store(&__data, (page), count);
	if (__data < 0)
		__data = 0;
	else if (__data > INT_MAX)
		__data = INT_MAX;

	dd->fifo_expire[WRITE] = msecs_to_jiffies(__data);
	return ret;
}

static ssize_t deadline_writes_starved_store(struct elevator_queue *e, const char *page, size_t count)
{
	struct deadline_data *dd = e->elevator_data;
	int __data;
	int ret = deadline_var_store(&__data, (page), count);
	if (__data < INT_MIN)
		__data = INT_MIN;
	else if (__data > INT_MAX)
		__data = INT_MAX;

	dd->writes_starved = __data;

	return ret;
}

static ssize_t deadline_front_merges_store(struct elevator_queue *e, const char *page, size_t count)
{
	struct deadline_data *dd = e->elevator_data;
	int __data;
	int ret = deadline_var_store(&__data, (page), count);
	if (__data < 0)
		__data = 0;
	else if (__data > 1)
		__data = 1;

	dd->front_merges = __data;
	return ret;
}

static ssize_t deadline_fifo_batch_store(struct elevator_queue *e, const char *page, size_t count)
{
	struct deadline_data *dd = e->elevator_data;
	int __data;
	int ret = deadline_var_store(&__data, (page), count);
	if (__data < 0)
		__data = 0;
	else if (__data > INT_MAX)
		__data = INT_MAX;

	dd->fifo_batch = __data;

	return ret;
}

static struct elv_fs_entry deadline_attrs[] = {
	__ATTR(read_expire, S_IRUGO|S_IWUSR, deadline_read_expire_show, \
				deadline_read_expire_store),
	__ATTR(write_expire, S_IRUGO|S_IWUSR, deadline_write_expire_show, \
				      deadline_write_expire_store),
	__ATTR(writes_starved, S_IRUGO|S_IWUSR, deadline_writes_starved_show, \
				      deadline_writes_starved_store),
	__ATTR(front_merges, S_IRUGO|S_IWUSR, deadline_front_merges_show, \
				      deadline_front_merges_store),
	__ATTR(fifo_batch, S_IRUGO|S_IWUSR, deadline_fifo_batch_show, \
				      deadline_fifo_batch_store),
	__ATTR_NULL
};

static struct elevator_type iosched_deadline = {
	.ops = {
		.elevator_merge_fn = 		deadline_merge,
		.elevator_merged_fn =		deadline_merged_request,
		.elevator_merge_req_fn =	deadline_merged_requests,
		.elevator_dispatch_fn =		deadline_dispatch_requests,
		.elevator_add_req_fn =		deadline_add_request,
		.elevator_former_req_fn =	elv_rb_former_request,
		.elevator_latter_req_fn =	elv_rb_latter_request,
		.elevator_init_fn =		deadline_init_queue,
		.elevator_exit_fn =		deadline_exit_queue,
	},

	.elevator_attrs = deadline_attrs,
	.elevator_name = "deadline",
	.elevator_owner = THIS_MODULE,
};

static int __init deadline_init(void)
{
	return elv_register(&iosched_deadline);
}

static void __exit deadline_exit(void)
{
	elv_unregister(&iosched_deadline);
}

module_init(deadline_init);
module_exit(deadline_exit);

MODULE_AUTHOR("Jens Axboe");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("deadline IO scheduler");
