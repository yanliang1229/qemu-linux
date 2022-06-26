#ifndef MIGRATE_MODE_H_INCLUDED
#define MIGRATE_MODE_H_INCLUDED
/*
 * MIGRATE_ASYNC means never block
 * MIGRATE_SYNC_LIGHT in the current implementation means to allow blocking
 *	on most operations but not ->writepage as the potential stall time
 *	is too significant
 * MIGRATE_SYNC will block when migrating pages
 */
enum migrate_mode {
	MIGRATE_ASYNC,		/* 异步迁移 */
	MIGRATE_SYNC_LIGHT,	/* 轻量级的同步迁移，允许大多数的操作阻塞，但是不允许阻塞在文件页回写(需要等很长时间) */
	MIGRATE_SYNC,		/* 完全同步阻塞迁移 */
};

#endif		/* MIGRATE_MODE_H_INCLUDED */
