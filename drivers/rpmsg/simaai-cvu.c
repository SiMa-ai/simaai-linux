// SPDX-License-Identifier: GPL-2.0-only
/*
 * SiMa.ai CVU/EV74 job submission driver.
 *
 * Exposes /dev/cvu. Userspace submits a job as a graph id, a per-graph
 * config descriptor (a dma-buf) and a list of buffer relocations. For each
 * relocation the driver resolves the dma-buf fd to the EV74 bus address
 * (through the EVXX STU), patches it into the descriptor at the requested
 * offset, then dispatches the job to the remote core over rpmsg. Userspace
 * never handles a physical or bus address; the kernel bounds-checks every
 * address it writes, so the EV74 can only reach buffers the caller
 * legitimately holds fds to.
 *
 * Multi-client model: the EV74 firmware exposes a single service endpoint
 * (RPMSG_SERV_NAME "simaai-ev74"). Each open() of /dev/cvu creates its own
 * rpmsg endpoint with a kernel-assigned address; that address is stamped into
 * the request's reserved field and the firmware routes the reply straight back
 * to it. So separate opens (separate processes, or threads that each open their
 * own fd) run concurrently, and replies demultiplex by endpoint with no shared
 * lock. A given fd has at most one request in flight.
 *
 * Submit is synchronous by default (CVU_IOC_SUBMIT blocks for the reply). With
 * CVU_SUBMIT_NONBLOCK it returns once dispatched; the caller then waits with
 * poll() and collects the outcome with read() as a struct cvu_completion.
 *
 * The driver binds the name-service channel the EV74 firmware announces, so
 * /dev/cvu appears only once the firmware is running.
 *
 * Copyright (c) 2025 SiMa.ai
 *
 * Author: Nileshkumar Raghuvanshi <nilesh.r@sima.ai>
 */

#include <linux/dma-buf.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/io.h>
#include <linux/iosys-map.h>
#include <linux/jiffies.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/rpmsg.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/simaai-stu.h>
#include <uapi/linux/simaai/simaai_cvu.h>

#define CVU_RPMSG_SERV_NAME		"simaai-ev74"
#define CVU_EVXX_RPROC_COMPATIBLE	"simaai,modalix-evxx-rproc"

/* Wire protocol shared with the EV74 firmware (platform/common/ipc_msg.h). */
#define SIMAAI_PROTO_UNIFIED		3
#define SIMA_IPC_FLAG_REPLY_ERROR	0x01

struct cvu_ipc_unified {
	u8  address_msg;
	u8  graph_id;
	u16 sequence_id;
	u64 addr;
	u8  reserved[4];	/* carries the client endpoint address (reply route) */
} __packed;

struct cvu_ipc_reply {
	u16 version;
	u16 sequence_id;
	struct {
		u8  ipc_class;
		u8  ipc_flags;
		u16 ipc_code;
	} message_type;
	u8  payload[8];
} __packed;

static unsigned int cvu_timeout_ms = 5000;
module_param(cvu_timeout_ms, uint, 0644);
MODULE_PARM_DESC(cvu_timeout_ms, "CVU job completion timeout in milliseconds (synchronous submit)");

/*
 * Per-fd cap on cached buffer imports. A lane binds a small fixed set of
 * pool buffers, so this only has to cover one pool's working set; entries
 * past it are evicted in LRU order and re-resolved on their next use.
 */
/*
 * Per-fd cap on cached buffer imports. A lane binds a small fixed set of pool
 * buffers, so this only has to cover one pool's working set; entries past it
 * are evicted in LRU order and re-resolved on their next use.
 *
 * The default is CVU_MAX_RELOCS, which means a single submit can never evict
 * the entries it created earlier in its own relocation loop. Lowering it below
 * CVU_MAX_RELOCS gives up that property and is for testing the miss path.
 */
static unsigned int cvu_cache_max = CVU_MAX_RELOCS;
module_param(cvu_cache_max, uint, 0644);
MODULE_PARM_DESC(cvu_cache_max, "Max cached buffer imports per open fd (0 disables the cache)");

/*
 * Second cap on the same cache, in bytes. An entry count alone bounds how many
 * buffers a lane pins but says nothing about how much memory that is: 64
 * entries is a few megabytes of 256x256 frames or a gigabyte of 16 MiB ones.
 *
 * Exceeding it evicts in LRU order like the entry cap does, so a value set too
 * low costs re-resolves rather than correctness. A buffer larger than the cap
 * on its own is still cached -- eviction stops at an empty cache and the insert
 * proceeds -- so the bound is "this many bytes, plus at most one buffer".
 *
 * The default is a placeholder: it is above any pool a lane realistically
 * cycles (a 30-deep 4K NV12 pool is ~376 MiB across all its buffers, and a
 * lane binds a handful per frame) and well below the 1 GiB the entry cap alone
 * would permit. Size it from real pool cardinalities when they are known.
 */
static unsigned int cvu_cache_max_mb = 256;
module_param(cvu_cache_max_mb, uint, 0644);
MODULE_PARM_DESC(cvu_cache_max_mb, "Max MiB of cached buffer imports per open fd (0 = no byte limit)");

/* Per rpmsg device (per firmware instance). */
struct cvu_dev {
	/*
	 * Clients outlive remove(): userspace can hold an fd open across a
	 * remoteproc stop. They keep using cdev for its locks, its STU handle
	 * and the device their cached buffers are attached to, so the device
	 * is reference counted rather than tied to the rpmsg device's own
	 * lifetime.
	 */
	struct kref		ref;
	struct rpmsg_device	*rpdev;
	struct miscdevice	mdev;
	struct simaai_stu	*stu;
	/*
	 * The device buffers are attached to. This is the misc device, given
	 * a 64-bit mask at probe: attaching is what pins an imported buffer's
	 * backing store for as long as the import is cached.
	 */
	struct device		*dma_dev;

	/*
	 * Every accepted job, across all clients. A job is on this list from
	 * the moment it is dispatched until its references are dropped, so
	 * remove() -- which is what a remoteproc stop reaches us as -- can
	 * find the jobs the remote will now never answer.
	 */
	struct list_head	jobs;
	struct mutex		jobs_lock;

	/* every open client, so the stats file can walk them */
	struct list_head	clients;
	struct mutex		clients_lock;
	struct dentry		*debugfs;
};

enum cvu_req_state {
	CVU_IDLE = 0,	/* nothing submitted */
	CVU_PENDING,	/* dispatched, awaiting reply */
	CVU_DONE,	/* reply received, not yet consumed */
	CVU_POISONED,	/* timed out; job retained, lane refuses new work */
};

/*
 * An accepted job: what the kernel owns on the caller's behalf from the moment
 * the request reaches the EV74 until the remote is known to be done with it.
 *
 * The EV74 was given the addresses of these buffers, so the kernel is the last
 * layer that can keep them allocated. Userspace can die, close its fds or free
 * its pool the instant the ioctl returns; none of that may recycle memory a
 * running graph is reading or writing. So the job holds a reference to the
 * descriptor and to every buffer it binds, and drops them only on the matching
 * reply or when the remote goes away.
 *
 * References, not attachments: a reference is what stops the pages being
 * freed. Pinning against relocation is the import cache's job for cached
 * buffers, and every exporter reaching the CVU today is static anyway.
 */
struct cvu_job {
	struct list_head	node;		/* cvu_dev.jobs */
	struct cvu_client	*cl;		/* holds a reference */
	struct dma_buf		*desc;
	struct dma_buf		**bufs;
	unsigned int		nbufs;
	size_t			retained;	/* bytes held, for reporting */
	u8			graph_id;
	u64			req_id;
};

#define CVU_CACHE_BITS		5	/* 32 buckets; sized for a pool, not a heap */

/*
 * One cached buffer import. Resolving a buffer costs a mapping of the whole
 * buffer, so the result is kept for as long as the fd keeps using it:
 * userspace recycles a small pool, so the same buffers come back every frame.
 *
 * Keyed on the dma_buf, never on the fd number -- fd numbers are recycled and
 * two fds in the same process routinely name the same buffer.
 *
 * @attach is held for the entry's lifetime. On a non-dynamic exporter that
 * pins the backing store, which is what makes @bus safe to remember at all.
 */
struct cvu_buf_import {
	struct hlist_node	hnode;
	struct list_head	lru;		/* cl->cache_lru, most recent first */
	struct dma_buf		*dmabuf;	/* reference held */
	struct dma_buf_attachment *attach;
	struct sg_table		*sgt;
	u64			bus;		/* EV74 bus address of byte 0 */
	size_t			len;
};

/*
 * Per open() context. Owns a dedicated rpmsg endpoint; a submit stamps this
 * endpoint's address into the request so the firmware replies to it. At most
 * one request is in flight per fd (@state), so a shared fd serializes while
 * different fds run in parallel.
 */
struct cvu_client {
	struct cvu_dev		*cdev;
	struct rpmsg_endpoint	*ept;
	/*
	 * The client outlives its fd whenever a job does. close() drops the
	 * file's reference; the accepted job holds the other, so the endpoint
	 * stays alive and a late reply still finds a home. The last put frees
	 * everything, always from process context.
	 */
	struct kref		ref;
	bool			orphaned;	/* fd closed, no consumer left */
	/* submitter gave up: discard the outcome and reopen the lane */
	bool			abandoned;
	spinlock_t		slock;		/* protects state and reply */
	wait_queue_head_t	wq;
	enum cvu_req_state	state;
	struct cvu_ipc_reply	reply;
	struct cvu_job		*job;		/* the accepted job, if any */
	struct timer_list	timer;		/* bounds an unanswered job */
	struct work_struct	release_work;	/* releases a job with no consumer */
	u64			req_id;		/* request_id of the in-flight job */
	u8			graph_id;	/* graph of the in-flight job */
	u16			seq;		/* sequence id stamped into it */
	u64			replies_stale;	/* replies that matched no request */

	/* buffer import cache; submit is the only user, always process context */
	struct mutex		cache_lock;
	DECLARE_HASHTABLE(cache, CVU_CACHE_BITS);
	struct list_head	cache_lru;
	unsigned int		cache_count;
	size_t			cache_bytes;	/* currently pinned by the cache */
	size_t			cache_bytes_peak;
	u64			cache_hits;
	u64			cache_misses;
	u64			cache_evictions;

	struct list_head	node;		/* cvu_dev.clients, for stats */
};

/*
 * The EV74 sees DRAM through the EVXX STU. Resolve the STU handle from the
 * rproc DT node the same way the (now removed) dma-buf GET_DESC ioctl did;
 * the STU is a built-in platform device that never goes away, so cache it.
 */
static struct simaai_stu *cvu_get_evxx_stu(void)
{
	static struct simaai_stu *stu;
	struct device_node *np;

	if (stu)
		return stu;

	np = of_find_compatible_node(NULL, NULL, CVU_EVXX_RPROC_COMPATIBLE);
	if (!np) {
		pr_err_once("simaai-cvu: evxx rproc node not found\n");
		return NULL;
	}

	stu = simaai_stu_get_by_phandle(np, "simaai,stu");
	of_node_put(np);
	if (IS_ERR(stu)) {
		stu = NULL;
		return NULL;
	}

	return stu;
}

/*
 * Translate a physical address into the address the EV74 uses to reach it.
 *
 * The EV74 has no view of DRAM except through the STU: a physical address it
 * has no window for is not a slower address, it is a different one. So a
 * failed translation is a failed submit. Passing the untranslated address
 * through instead -- which this used to do -- lets the firmware run to
 * completion against memory that is not the caller's buffer, which surfaces as
 * a graph that reports success and produces nothing.
 */
static int cvu_stu_bus(struct cvu_dev *c, phys_addr_t phys, u64 *bus)
{
	dma_addr_t b;
	int ret;

	ret = simaai_stu_get_bus_address(c->stu, phys, &b);
	if (ret) {
		dev_warn_once(&c->rpdev->dev,
			      "no EVXX STU window for %pap; the EV74 cannot reach this buffer\n",
			      &phys);
		return ret;
	}

	*bus = (u64)b;
	return 0;
}

/*
 * Physical address of a (contiguous) dma-buf, obtained the same way the
 * GET_DESC ioctl did: vmap and translate the mapping's first page. Only
 * contiguous CMA buffers reach the CVU, so the base page is enough.
 */
static int cvu_dmabuf_phys(struct dma_buf *dmabuf, phys_addr_t *phys)
{
	struct iosys_map map;
	struct page *page;
	int ret;

	ret = dma_buf_vmap_unlocked(dmabuf, &map);
	if (ret)
		return ret;

	page = vmalloc_to_page(map.vaddr);
	if (page)
		*phys = page_to_phys(page) + offset_in_page(map.vaddr);
	else
		ret = -EINVAL;

	dma_buf_vunmap_unlocked(dmabuf, &map);
	return ret;
}

/* Resolve a buffer from scratch: import it, pin it, translate its address. */
static struct cvu_buf_import *cvu_import_create(struct cvu_client *cl,
						struct dma_buf *dmabuf)
{
	struct cvu_dev *c = cl->cdev;
	struct cvu_buf_import *e;
	phys_addr_t phys;
	u64 b;
	int ret;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return ERR_PTR(-ENOMEM);

	e->attach = dma_buf_attach(dmabuf, c->dma_dev);
	if (IS_ERR(e->attach)) {
		ret = PTR_ERR(e->attach);
		goto err_free;
	}

	/*
	 * The mapping itself is not used -- the EV74 does not see DRAM
	 * through this device's DMA address space, it sees it through the
	 * STU. It is taken to pin the backing store for the lifetime of the
	 * entry, so the address resolved below stays true.
	 */
	e->sgt = dma_buf_map_attachment_unlocked(e->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(e->sgt)) {
		ret = PTR_ERR(e->sgt);
		goto err_detach;
	}

	ret = cvu_dmabuf_phys(dmabuf, &phys);
	if (ret)
		goto err_unmap;

	ret = cvu_stu_bus(c, phys, &b);
	if (ret)
		goto err_unmap;

	get_dma_buf(dmabuf);
	e->dmabuf = dmabuf;
	e->bus = b;
	e->len = dmabuf->size;
	return e;

err_unmap:
	dma_buf_unmap_attachment_unlocked(e->attach, e->sgt, DMA_BIDIRECTIONAL);
err_detach:
	dma_buf_detach(dmabuf, e->attach);
err_free:
	kfree(e);
	return ERR_PTR(ret);
}

static void cvu_import_destroy(struct cvu_buf_import *e)
{
	dma_buf_unmap_attachment_unlocked(e->attach, e->sgt, DMA_BIDIRECTIONAL);
	dma_buf_detach(e->dmabuf, e->attach);
	dma_buf_put(e->dmabuf);
	kfree(e);
}

/* Drop every import this client holds. Called from release, with no job left. */
static void cvu_cache_purge(struct cvu_client *cl)
{
	struct cvu_buf_import *e, *tmp;

	mutex_lock(&cl->cache_lock);
	list_for_each_entry_safe(e, tmp, &cl->cache_lru, lru) {
		hash_del(&e->hnode);
		list_del(&e->lru);
		cvu_import_destroy(e);
	}
	cl->cache_count = 0;
	cl->cache_bytes = 0;
	mutex_unlock(&cl->cache_lock);
}

/*
 * Bus address of @off bytes into @dmabuf, from the cache where possible.
 *
 * A miss costs what every submit used to cost; a hit costs a hash lookup.
 * Steady state is all hits, because a lane cycles through one pool.
 */
static int cvu_bus_addr(struct cvu_client *cl, struct dma_buf *dmabuf,
			u64 off, u64 *bus)
{
	struct cvu_dev *c = cl->cdev;
	struct cvu_buf_import *e, *victim;
	u32 key = hash_ptr(dmabuf, CVU_CACHE_BITS);
	size_t max_bytes;
	int ret = 0;

	if (!cvu_cache_max) {
		phys_addr_t phys;
		u64 b;

		ret = cvu_dmabuf_phys(dmabuf, &phys);
		if (ret)
			return ret;
		ret = cvu_stu_bus(c, phys, &b);
		if (ret)
			return ret;
		*bus = b + off;
		return 0;
	}

	mutex_lock(&cl->cache_lock);

	hlist_for_each_entry(e, &cl->cache[key], hnode) {
		if (e->dmabuf == dmabuf) {
			list_move(&e->lru, &cl->cache_lru);
			cl->cache_hits++;
			goto out;
		}
	}

	e = cvu_import_create(cl, dmabuf);
	if (IS_ERR(e)) {
		ret = PTR_ERR(e);
		goto unlock;
	}
	cl->cache_misses++;

	/*
	 * Evict before insert so the entry just resolved -- the one this
	 * submit is about to use -- can never be the victim. Both caps are
	 * enforced here; eviction stops at an empty cache, so a buffer bigger
	 * than the byte cap is still cached rather than re-resolved forever.
	 */
	max_bytes = (size_t)cvu_cache_max_mb << 20;
	while (!list_empty(&cl->cache_lru) &&
	       (cl->cache_count >= cvu_cache_max ||
		(max_bytes && cl->cache_bytes + e->len > max_bytes))) {
		victim = list_last_entry(&cl->cache_lru, struct cvu_buf_import,
					 lru);
		hash_del(&victim->hnode);
		list_del(&victim->lru);
		cl->cache_count--;
		cl->cache_bytes -= victim->len;
		cl->cache_evictions++;
		cvu_import_destroy(victim);
	}

	hlist_add_head(&e->hnode, &cl->cache[key]);
	list_add(&e->lru, &cl->cache_lru);
	cl->cache_count++;
	cl->cache_bytes += e->len;
	if (cl->cache_bytes > cl->cache_bytes_peak)
		cl->cache_bytes_peak = cl->cache_bytes;

out:
	if (off >= e->len)
		ret = -EINVAL;
	else
		*bus = e->bus + off;
unlock:
	mutex_unlock(&cl->cache_lock);
	return ret;
}

/*
 * ---------------------------------------------------------------------------
 * Accepted-job ownership
 *
 * Nothing below ever runs from the rpmsg receive callback or the timer: both
 * are atomic context, and releasing a job takes locks that sleep. Those two
 * only set state and, where there is no consumer left, queue release_work.
 * Every path that actually drops references lands in cvu_job_finish().
 * ---------------------------------------------------------------------------
 */

static void cvu_dev_free(struct kref *ref)
{
	struct cvu_dev *c = container_of(ref, struct cvu_dev, ref);

	put_device(c->dma_dev);		/* the misc device buffers attach to */
	put_device(&c->rpdev->dev);
	mutex_destroy(&c->clients_lock);
	mutex_destroy(&c->jobs_lock);
	kfree(c);
}

static void cvu_client_free(struct kref *ref)
{
	struct cvu_client *cl = container_of(ref, struct cvu_client, ref);

	mutex_lock(&cl->cdev->clients_lock);
	list_del(&cl->node);
	mutex_unlock(&cl->cdev->clients_lock);

	rpmsg_destroy_ept(cl->ept);
	cvu_cache_purge(cl);
	timer_delete_sync(&cl->timer);
	mutex_destroy(&cl->cache_lock);
	kref_put(&cl->cdev->ref, cvu_dev_free);
	kfree(cl);
}

/*
 * Drop a job's buffer references and free it. Nothing else: the caller decides
 * whether the job ever reached the device list or took a client reference.
 */
static void cvu_job_free(struct cvu_job *job)
{
	unsigned int i;

	for (i = 0; i < job->nbufs; i++)
		dma_buf_put(job->bufs[i]);
	kfree(job->bufs);
	if (job->desc)
		dma_buf_put(job->desc);
	kfree(job);
}

/* Release a committed job: off the device list, references dropped. */
static void cvu_job_destroy(struct cvu_job *job)
{
	struct cvu_client *cl = job->cl;

	mutex_lock(&cl->cdev->jobs_lock);
	list_del(&job->node);
	mutex_unlock(&cl->cdev->jobs_lock);

	cvu_job_free(job);
	kref_put(&cl->ref, cvu_client_free);
}

/* Detach the client's job, if it still has one. Safe to call repeatedly. */
static struct cvu_job *cvu_job_take(struct cvu_client *cl)
{
	struct cvu_job *job;
	unsigned long flags;

	spin_lock_irqsave(&cl->slock, flags);
	job = cl->job;
	cl->job = NULL;
	spin_unlock_irqrestore(&cl->slock, flags);

	return job;
}

/*
 * End a job: stop its timer, then release it. The timer is deleted
 * synchronously, which sleeps, so this is process context only.
 */
/*
 * Detach a job whose outcome nobody is going to collect, and reopen the lane
 * if the submitter merely gave up waiting. Setting CVU_IDLE here rather than
 * in the callback keeps it in the same locked region as the detach, so a
 * dispatch that follows cannot have its own job taken by this one.
 */
static struct cvu_job *cvu_job_take_reclaim(struct cvu_client *cl)
{
	struct cvu_job *job;
	unsigned long flags;

	spin_lock_irqsave(&cl->slock, flags);
	job = cl->job;
	cl->job = NULL;
	if (cl->abandoned && cl->state == CVU_DONE) {
		cl->state = CVU_IDLE;
		cl->abandoned = false;
	}
	spin_unlock_irqrestore(&cl->slock, flags);

	return job;
}

static void cvu_job_release(struct cvu_client *cl, struct cvu_job *job)
{
	if (!job)
		return;

	timer_delete_sync(&cl->timer);
	cvu_job_destroy(job);
}

/*
 * Detach and release in one step. Only for callers that are not already
 * holding slock and cannot race a dispatch -- anyone completing a job must
 * detach it in the same locked region that returns the lane to CVU_IDLE, or a
 * second thread on the same fd can dispatch into the gap and have its job
 * released underneath it.
 */
static void cvu_job_finish(struct cvu_client *cl)
{
	cvu_job_release(cl, cvu_job_take(cl));
}

static void cvu_release_work(struct work_struct *work)
{
	struct cvu_client *cl = container_of(work, struct cvu_client,
					     release_work);

	cvu_job_release(cl, cvu_job_take_reclaim(cl));
	kref_put(&cl->ref, cvu_client_free);	/* taken by cvu_queue_release() */
}

/*
 * Hand the job to release_work, keeping the client alive until it runs.
 *
 * Without a reference of its own the work can outlive everything that was
 * holding the client: remove() releases the job, that drops the last
 * reference, and the work then runs on freed memory. Cancelling from
 * cvu_client_free() is not an option -- the work is one of the callers that
 * can drop the last reference, so it would wait for itself.
 */
static void cvu_queue_release(struct cvu_client *cl)
{
	kref_get(&cl->ref);
	if (!queue_work(system_wq, &cl->release_work))
		/*
		 * Already pending, so that instance's reference covers it. It
		 * cannot be the last one, which is what makes this put safe
		 * from the rpmsg callback's atomic context.
		 */
		kref_put(&cl->ref, cvu_client_free);
}

/*
 * The remote did not answer within the timeout. That says the reply is late;
 * it does not say the EV74 stopped reading and writing the buffers, so the job
 * is kept and the lane refuses further work. The memory comes back when the
 * late reply arrives, or when the remote is reset -- which reaches this driver
 * as cvu_rpmsg_remove().
 */
static void cvu_job_timeout(struct timer_list *t)
{
	struct cvu_client *cl = timer_container_of(cl, t, timer);
	unsigned long flags;
	size_t retained;
	u64 req_id;
	u8 graph_id;

	spin_lock_irqsave(&cl->slock, flags);
	if (cl->state != CVU_PENDING || !cl->job) {
		spin_unlock_irqrestore(&cl->slock, flags);
		return;
	}
	cl->state = CVU_POISONED;
	/* copy what the message needs: the job may be released once unlocked */
	graph_id = cl->job->graph_id;
	req_id = cl->job->req_id;
	retained = cl->job->retained;
	spin_unlock_irqrestore(&cl->slock, flags);

	wake_up_interruptible(&cl->wq);

	dev_warn(&cl->cdev->rpdev->dev,
		 "graph %u req 0x%016llx: no reply after %u ms; lane poisoned, %zu KiB retained until the reply arrives or the remote is reset\n",
		 graph_id, req_id, cvu_timeout_ms, retained >> 10);
}

/*
 * Resolve the fds, patch their bus addresses into the config descriptor, and
 * build the job that will own them.
 *
 * The references taken here are kept, not dropped at the end of the loop: from
 * here on the kernel holds the descriptor and every buffer until the job ends.
 * On any failure the part-built job is released and nothing is retained.
 */
static int cvu_patch_descriptor(struct cvu_client *cl, struct cvu_submit *s,
				u64 *desc_bus, struct cvu_job **out)
{
	struct cvu_dev *c = cl->cdev;
	struct cvu_buf_reloc *relocs = NULL;
	struct cvu_job *job;
	struct dma_buf *desc;
	struct iosys_map map;
	phys_addr_t desc_phys;
	struct page *desc_page;
	void *desc_va;
	unsigned int i;
	int ret;

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return -ENOMEM;

	INIT_LIST_HEAD(&job->node);
	job->cl = cl;
	job->graph_id = s->graph_id;
	job->req_id = s->request_id;

	if (s->num_relocs) {
		relocs = kmalloc_array(s->num_relocs, sizeof(*relocs),
				       GFP_KERNEL);
		job->bufs = kcalloc(s->num_relocs, sizeof(*job->bufs),
				    GFP_KERNEL);
		if (!relocs || !job->bufs) {
			ret = -ENOMEM;
			goto free_job;
		}

		if (copy_from_user(relocs,
				   (void __user *)(uintptr_t)s->relocs,
				   s->num_relocs * sizeof(*relocs))) {
			ret = -EFAULT;
			goto free_job;
		}
	}

	desc = dma_buf_get(s->desc_fd);
	if (IS_ERR(desc)) {
		ret = PTR_ERR(desc);
		goto free_job;
	}
	job->desc = desc;			/* reference now owned by the job */
	job->retained = desc->size;

	ret = dma_buf_vmap_unlocked(desc, &map);
	if (ret)
		goto free_job;
	desc_va = map.vaddr;

	/* the descriptor's own bus address travels in the message */
	desc_page = vmalloc_to_page(desc_va);
	if (!desc_page) {
		ret = -EINVAL;
		goto vunmap;
	}
	desc_phys = page_to_phys(desc_page) + offset_in_page(desc_va);
	ret = cvu_stu_bus(c, desc_phys, desc_bus);
	if (ret)
		goto vunmap;

	dma_buf_begin_cpu_access(desc, DMA_TO_DEVICE);
	for (i = 0; i < s->num_relocs; i++) {
		struct dma_buf *buf;
		u64 bbus;

		if ((u64)relocs[i].desc_offset + sizeof(u64) > desc->size) {
			ret = -EINVAL;
			goto end_access;
		}

		buf = dma_buf_get(relocs[i].buf_fd);
		if (IS_ERR(buf)) {
			ret = PTR_ERR(buf);
			goto end_access;
		}
		if (relocs[i].buf_offset >= buf->size) {
			dma_buf_put(buf);
			ret = -EINVAL;
			goto end_access;
		}

		/* the job owns this reference from here, even on failure */
		job->bufs[job->nbufs++] = buf;
		job->retained += buf->size;

		ret = cvu_bus_addr(cl, buf, relocs[i].buf_offset, &bbus);
		if (ret)
			goto end_access;

		*(u64 *)(desc_va + relocs[i].desc_offset) = bbus;
	}
	ret = 0;

end_access:
	dma_buf_end_cpu_access(desc, DMA_TO_DEVICE);	/* clean for the EV74 */
vunmap:
	dma_buf_vunmap_unlocked(desc, &map);
free_job:
	if (ret)
		cvu_job_free(job);	/* never committed; nothing was retained */
	else
		*out = job;
	kfree(relocs);
	return ret;
}

/*
 * Dispatch a prepared request on this client's endpoint, and hand the job to
 * the kernel at the same moment.
 *
 * The job is committed, the client reference taken and the timer armed before
 * the message goes out, so a reply that lands immediately -- or a timeout --
 * always finds a fully formed job to act on.
 */
static int cvu_dispatch(struct cvu_client *cl, struct cvu_submit *s,
			u64 desc_bus, struct cvu_job *job)
{
	struct cvu_ipc_unified msg = {};
	u32 reply_addr;
	unsigned long flags;
	u16 seq;
	int ret;

	/* claim the single in-flight slot for this fd */
	spin_lock_irqsave(&cl->slock, flags);
	if (cl->state == CVU_POISONED) {
		spin_unlock_irqrestore(&cl->slock, flags);
		cvu_job_free(job);	/* refused before the remote saw it */
		return -ENOTRECOVERABLE;
	}
	if (cl->state != CVU_IDLE) {
		spin_unlock_irqrestore(&cl->slock, flags);
		cvu_job_free(job);
		return -EBUSY;
	}
	cl->state = CVU_PENDING;
	cl->graph_id = s->graph_id;
	cl->req_id = s->request_id;
	seq = ++cl->seq;		/* the value that identifies this job */
	cl->job = job;
	kref_get(&cl->ref);		/* the job keeps the client alive */
	spin_unlock_irqrestore(&cl->slock, flags);

	mutex_lock(&cl->cdev->jobs_lock);
	list_add_tail(&job->node, &cl->cdev->jobs);
	mutex_unlock(&cl->cdev->jobs_lock);

	mod_timer(&cl->timer, jiffies + msecs_to_jiffies(cvu_timeout_ms));

	msg.address_msg = SIMAAI_PROTO_UNIFIED;
	msg.graph_id	= s->graph_id;
	msg.sequence_id	= seq;
	msg.addr	= desc_bus;
	/*
	 * The firmware cannot read the rpmsg source address (the prebuilt
	 * libopen_amp leaves it 0), so it takes the reply route from the
	 * reserved field. Stamp this endpoint's address there.
	 */
	reply_addr = cl->ept->addr;
	memcpy(msg.reserved, &reply_addr, sizeof(reply_addr));

	ret = rpmsg_send(cl->ept, &msg, sizeof(msg));
	if (ret) {
		/* never reached the remote, so nothing needs retaining */
		spin_lock_irqsave(&cl->slock, flags);
		cl->state = CVU_IDLE;
		spin_unlock_irqrestore(&cl->slock, flags);
		cvu_job_finish(cl);
	}
	return ret;
}

/* Translate a received reply into a result code (called with state == DONE). */
/*
 * Does this reply belong to the request currently in flight?
 *
 * The firmware echoes the sequence id the driver stamped into the request, so
 * a reply can be tied to exactly one submit. Older firmware overwrote that
 * field with the graph id instead, which correlates nothing: every job of a
 * given graph looked alike, so a reply left over from a previous job would be
 * accepted as the current one's completion. Accept that form too while such
 * firmware is still in the field, but say so once, because the guarantee this
 * function exists to provide is absent when it is in use.
 *
 * Called from the rpmsg callback, so it must not sleep.
 */
static bool cvu_reply_matches(struct cvu_client *cl, u16 seq)
{
	if (seq == cl->seq)
		return true;

	if (seq == cl->graph_id) {
		dev_warn_once(&cl->cdev->rpdev->dev,
			      "firmware echoes the graph id instead of the sequence id; stale replies cannot be detected. Update cvu-sw\n");
		return true;
	}

	return false;
}

static int cvu_reply_result(struct cvu_client *cl)
{
	if (cl->reply.message_type.ipc_flags & SIMA_IPC_FLAG_REPLY_ERROR)
		return -EIO;
	return 0;
}

static int cvu_do_submit(struct cvu_client *cl, struct cvu_submit *s)
{
	struct cvu_job *job = NULL;
	unsigned long flags;
	u64 desc_bus;
	int ret;

	if ((s->flags & ~CVU_SUBMIT_NONBLOCK) || s->num_relocs > CVU_MAX_RELOCS)
		return -EINVAL;

	/*
	 * Checked before anything is resolved, not just at dispatch: after the
	 * remote has gone the resolve path would be reaching into a device
	 * that is being torn down.
	 */
	if (READ_ONCE(cl->state) == CVU_POISONED)
		return -ENOTRECOVERABLE;

	ret = cvu_patch_descriptor(cl, s, &desc_bus, &job);
	if (ret)
		return ret;

	ret = cvu_dispatch(cl, s, desc_bus, job);
	if (ret)
		return ret;

	if (s->flags & CVU_SUBMIT_NONBLOCK)
		return 0;	/* outcome retrieved via poll()/read() */

	/*
	 * No timeout of its own: the job's timer is the single authority on
	 * how long a submit may go unanswered, and it moves the lane to
	 * POISONED, which ends this wait.
	 */
	if (wait_event_interruptible(cl->wq,
				     READ_ONCE(cl->state) != CVU_PENDING)) {
		/*
		 * Interrupted. The job is still the kernel's, but nobody is
		 * going to collect its outcome: a synchronous caller has no
		 * reason to read() afterwards. Say so, so that whoever sees
		 * the reply releases the job and reopens the lane -- otherwise
		 * it sits in CVU_DONE and every later submit gets -EBUSY for
		 * the life of the fd.
		 */
		spin_lock_irqsave(&cl->slock, flags);
		if (cl->state == CVU_DONE) {
			cl->state = CVU_IDLE;	/* it arrived while we unwound */
			job = cl->job;
			cl->job = NULL;
		} else {
			cl->abandoned = true;
			job = NULL;
		}
		spin_unlock_irqrestore(&cl->slock, flags);

		cvu_job_release(cl, job);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&cl->slock, flags);
	if (cl->state != CVU_DONE) {
		spin_unlock_irqrestore(&cl->slock, flags);
		return -ETIMEDOUT;	/* poisoned; the job is retained */
	}
	ret = cvu_reply_result(cl);
	s->fw_status = cl->reply.message_type.ipc_flags;
	cl->state = CVU_IDLE;
	job = cl->job;			/* detached here, not after unlocking */
	cl->job = NULL;
	spin_unlock_irqrestore(&cl->slock, flags);

	cvu_job_release(cl, job);
	return ret;
}

/*
 * Per-endpoint reply callback: hand the reply to the waiting/ polling
 * submitter.
 *
 * Interrupt context, so this releases nothing. It records the reply and, when
 * there is no consumer left to do it -- the fd is closed, or the lane was
 * already poisoned and the reply is simply late -- queues the work that will.
 */
static int cvu_ept_cb(struct rpmsg_device *rpdev, void *data, int len,
		      void *priv, u32 src)
{
	const struct cvu_ipc_reply *r = data;
	struct cvu_client *cl = priv;
	unsigned long flags;
	bool release = false;
	bool stale = false;
	u16 seq;

	/* need at least the sequence id to say who this belongs to */
	if (len < offsetofend(struct cvu_ipc_reply, sequence_id)) {
		dev_warn_ratelimited(&rpdev->dev,
				     "runt reply (%d bytes) dropped\n", len);
		return 0;
	}
	seq = r->sequence_id;

	spin_lock_irqsave(&cl->slock, flags);
	if (cl->state == CVU_PENDING && cvu_reply_matches(cl, seq)) {
		if (len > sizeof(cl->reply))
			len = sizeof(cl->reply);
		memcpy(&cl->reply, data, len);
		cl->state = CVU_DONE;
		release = cl->orphaned || cl->abandoned;
		spin_unlock_irqrestore(&cl->slock, flags);
		wake_up_interruptible(&cl->wq);
	} else if (cl->state == CVU_POISONED && cl->job &&
		   cvu_reply_matches(cl, seq)) {
		/*
		 * A late reply for a poisoned lane: the remote has finished
		 * with the buffers after all, so this is what lets them go.
		 */
		release = true;
		spin_unlock_irqrestore(&cl->slock, flags);
	} else {
		/*
		 * Belongs to no request this fd is waiting on. Dropping it is
		 * the point: accepting it would complete a job the remote is
		 * still running, which is how a reply left over from a closed
		 * fd used to be handed to whoever inherited its endpoint.
		 */
		cl->replies_stale++;
		stale = true;
		spin_unlock_irqrestore(&cl->slock, flags);
	}

	if (stale)
		dev_warn_ratelimited(&rpdev->dev,
				     "dropped reply seq %u: no request in flight matches it\n",
				     seq);

	if (release)
		cvu_queue_release(cl);

	return 0;
}

static int cvu_open(struct inode *inode, struct file *file)
{
	struct cvu_dev *c = container_of(file->private_data,
					 struct cvu_dev, mdev);
	struct rpmsg_channel_info chinfo = {};
	struct cvu_client *cl;

	cl = kzalloc(sizeof(*cl), GFP_KERNEL);
	if (!cl)
		return -ENOMEM;

	cl->cdev = c;
	kref_get(&c->ref);		/* the client keeps the device alive */
	kref_init(&cl->ref);		/* the file's reference */
	spin_lock_init(&cl->slock);
	init_waitqueue_head(&cl->wq);
	cl->state = CVU_IDLE;
	mutex_init(&cl->cache_lock);
	hash_init(cl->cache);
	INIT_LIST_HEAD(&cl->cache_lru);
	timer_setup(&cl->timer, cvu_job_timeout, 0);
	INIT_WORK(&cl->release_work, cvu_release_work);
	INIT_LIST_HEAD(&cl->node);

	/* dedicated endpoint: kernel-assigned src, firmware service as dst */
	strscpy(chinfo.name, c->rpdev->id.name, sizeof(chinfo.name));
	chinfo.src = RPMSG_ADDR_ANY;
	chinfo.dst = c->rpdev->dst;

	cl->ept = rpmsg_create_ept(c->rpdev, cvu_ept_cb, cl, chinfo);
	if (!cl->ept) {
		mutex_destroy(&cl->cache_lock);
		kref_put(&c->ref, cvu_dev_free);
		kfree(cl);
		return -ENOMEM;
	}

	mutex_lock(&c->clients_lock);
	list_add_tail(&cl->node, &c->clients);
	mutex_unlock(&c->clients_lock);

	file->private_data = cl;
	return 0;
}

/*
 * close() never waits and never cancels a job. If one is in flight the kernel
 * keeps it, and the client with it, so the endpoint stays alive and a reply
 * still has somewhere to land; the caller is free to die at any point after
 * the ioctl returned.
 */
static int cvu_release(struct inode *inode, struct file *file)
{
	struct cvu_client *cl = file->private_data;
	unsigned long flags;
	bool finish;

	spin_lock_irqsave(&cl->slock, flags);
	cl->orphaned = true;
	/*
	 * Still running, or poisoned and waiting on a late reply: leave it to
	 * the reply or to remove(). Already answered: nobody will read() it
	 * now, so end it here.
	 */
	finish = cl->job && cl->state == CVU_DONE;
	spin_unlock_irqrestore(&cl->slock, flags);

	if (finish)
		cvu_job_finish(cl);

	kref_put(&cl->ref, cvu_client_free);
	return 0;
}

static long cvu_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct cvu_client *cl = file->private_data;
	struct cvu_submit s;
	int ret;

	if (cmd != CVU_IOC_SUBMIT)
		return -ENOTTY;

	if (copy_from_user(&s, (void __user *)arg, sizeof(s)))
		return -EFAULT;

	ret = cvu_do_submit(cl, &s);
	if (ret)
		return ret;

	if (copy_to_user((void __user *)arg, &s, sizeof(s)))
		return -EFAULT;

	return 0;
}

/* Collect the outcome of an asynchronous submit. */
static ssize_t cvu_read(struct file *file, char __user *ubuf, size_t count,
			loff_t *ppos)
{
	struct cvu_client *cl = file->private_data;
	struct cvu_completion comp = {};
	struct cvu_job *job;
	unsigned long flags;
	int ret;

	if (count < sizeof(comp))
		return -EINVAL;

	spin_lock_irqsave(&cl->slock, flags);
	while (cl->state != CVU_DONE) {
		if (cl->state == CVU_IDLE) {
			spin_unlock_irqrestore(&cl->slock, flags);
			return -ENODATA;	/* nothing in flight */
		}
		/* state == PENDING */
		spin_unlock_irqrestore(&cl->slock, flags);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(cl->wq,
					       READ_ONCE(cl->state) != CVU_PENDING);
		if (ret)
			return ret;
		spin_lock_irqsave(&cl->slock, flags);
	}

	comp.request_id = cl->req_id;
	comp.fw_status = cl->reply.message_type.ipc_flags;
	comp.result = cvu_reply_result(cl);
	cl->state = CVU_IDLE;
	job = cl->job;			/* detached here, not after unlocking */
	cl->job = NULL;
	spin_unlock_irqrestore(&cl->slock, flags);

	/* the outcome is consumed, so the remote is done with the buffers */
	cvu_job_release(cl, job);

	if (copy_to_user(ubuf, &comp, sizeof(comp)))
		return -EFAULT;

	return sizeof(comp);
}

static __poll_t cvu_poll(struct file *file, poll_table *wait)
{
	struct cvu_client *cl = file->private_data;

	poll_wait(file, &cl->wq, wait);

	if (READ_ONCE(cl->state) == CVU_DONE)
		return EPOLLIN | EPOLLRDNORM;

	return 0;
}

static const struct file_operations cvu_fops = {
	.owner		= THIS_MODULE,
	.open		= cvu_open,
	.release	= cvu_release,
	.unlocked_ioctl	= cvu_ioctl,
	.read		= cvu_read,
	.poll		= cvu_poll,
	.llseek		= noop_llseek,
};

/*
 * ---------------------------------------------------------------------------
 * Statistics
 *
 * Two pools of memory are held on userspace's behalf and they are not the same
 * thing. The import cache pins buffers so a resolved address stays true; it is
 * capped and evictable, and its size is a tuning question. An accepted job
 * retains its buffers because the EV74 is using them; that is not capped and
 * is released only on the reply, or on reset. A buffer bound by a running job
 * is usually in both, so the two byte counts overlap rather than add.
 * ---------------------------------------------------------------------------
 */

static const char *cvu_state_name(enum cvu_req_state st)
{
	switch (st) {
	case CVU_IDLE:		return "idle";
	case CVU_PENDING:	return "pending";
	case CVU_DONE:		return "done";
	case CVU_POISONED:	return "poisoned";
	}
	return "?";
}

static int cvu_stats_show(struct seq_file *m, void *v)
{
	struct cvu_dev *c = m->private;
	struct cvu_client *cl;
	size_t job_bytes_total = 0;
	unsigned int njobs = 0;

	seq_printf(m, "cache_max_entries %u\ncache_max_bytes %llu\n",
		   cvu_cache_max, (u64)cvu_cache_max_mb << 20);
	seq_puts(m,
		 "\nept    state     entries  cache_bytes   peak_bytes        hits      misses   evictions  job_bytes  stale\n");

	mutex_lock(&c->clients_lock);
	list_for_each_entry(cl, &c->clients, node) {
		size_t cache_bytes, peak, job_bytes = 0;
		unsigned int entries;
		u64 hits, misses, evict, stale;
		unsigned long flags;
		enum cvu_req_state st;

		mutex_lock(&cl->cache_lock);
		entries = cl->cache_count;
		cache_bytes = cl->cache_bytes;
		peak = cl->cache_bytes_peak;
		hits = cl->cache_hits;
		misses = cl->cache_misses;
		evict = cl->cache_evictions;
		mutex_unlock(&cl->cache_lock);

		spin_lock_irqsave(&cl->slock, flags);
		st = cl->state;
		stale = cl->replies_stale;
		if (cl->job) {
			job_bytes = cl->job->retained;
			njobs++;
		}
		spin_unlock_irqrestore(&cl->slock, flags);

		job_bytes_total += job_bytes;

		seq_printf(m, "%-6u %-9s %7u %12zu %12zu %11llu %11llu %11llu %10zu %6llu\n",
			   cl->ept->addr, cvu_state_name(st), entries,
			   cache_bytes, peak, hits, misses, evict, job_bytes,
			   stale);
	}
	mutex_unlock(&c->clients_lock);

	seq_printf(m, "\njobs_outstanding %u\njob_bytes_total %zu\n",
		   njobs, job_bytes_total);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cvu_stats);

static int cvu_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct cvu_dev *c;
	int ret;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	kref_init(&c->ref);		/* the driver's own reference */
	c->rpdev = rpdev;
	get_device(&rpdev->dev);
	INIT_LIST_HEAD(&c->jobs);
	mutex_init(&c->jobs_lock);
	INIT_LIST_HEAD(&c->clients);
	mutex_init(&c->clients_lock);
	/*
	 * The EV74 reaches DRAM only through the STU, so without one there is
	 * no address this driver could give it that means anything. Refuse to
	 * register rather than accept jobs that cannot work.
	 */
	c->stu = cvu_get_evxx_stu();
	if (!c->stu) {
		ret = dev_err_probe(&rpdev->dev, -ENODEV,
				    "no EVXX STU; cannot translate buffer addresses\n");
		goto err_put_dev;
	}

	c->mdev.minor	= MISC_DYNAMIC_MINOR;
	c->mdev.name	= "cvu";
	c->mdev.fops	= &cvu_fops;
	c->mdev.parent	= &rpdev->dev;

	dev_set_drvdata(&rpdev->dev, c);

	ret = misc_register(&c->mdev);
	if (ret) {
		dev_err(&rpdev->dev, "failed to register /dev/cvu: %d\n", ret);
		goto err_put_dev;
	}

	/*
	 * Buffers are attached to the misc device. It has no mask of its own,
	 * and the attachments exist to pin memory rather than to be mapped
	 * through, so give it the widest one.
	 */
	c->dma_dev = get_device(c->mdev.this_device);
	ret = dma_coerce_mask_and_coherent(c->dma_dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(&rpdev->dev, "failed to set the dma mask: %d\n", ret);
		misc_deregister(&c->mdev);
		goto err_put_dev;
	}

	c->debugfs = debugfs_create_dir("simaai-cvu", NULL);
	debugfs_create_file("stats", 0444, c->debugfs, c, &cvu_stats_fops);

	dev_info(&rpdev->dev, "CVU dispatch ready on /dev/%s (service dst %u)\n",
		 c->mdev.name, rpdev->dst);
	return 0;

err_put_dev:
	kref_put(&c->ref, cvu_dev_free);
	return ret;
}

/*
 * The remote is going away -- this is what a remoteproc stop reaches us as,
 * and it is the only point at which the EV74 is known to have stopped touching
 * DRAM. Any job still outstanding will never be answered, so this is where
 * what they retain is finally released.
 */
static void cvu_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct cvu_dev *c = dev_get_drvdata(&rpdev->dev);
	struct cvu_job *job, *tmp;
	struct cvu_client *cl;
	LIST_HEAD(orphans);

	debugfs_remove_recursive(c->debugfs);
	misc_deregister(&c->mdev);

	/*
	 * Poison every open lane, not only those with a job. Their fds stay
	 * usable after this returns, and without the remote there is nothing
	 * a submit could do but touch a device that is on its way out.
	 */
	mutex_lock(&c->clients_lock);
	list_for_each_entry(cl, &c->clients, node) {
		unsigned long flags;

		spin_lock_irqsave(&cl->slock, flags);
		cl->state = CVU_POISONED;
		spin_unlock_irqrestore(&cl->slock, flags);
		wake_up_interruptible(&cl->wq);
	}
	mutex_unlock(&c->clients_lock);

	mutex_lock(&c->jobs_lock);
	list_splice_init(&c->jobs, &orphans);
	mutex_unlock(&c->jobs_lock);

	list_for_each_entry_safe(job, tmp, &orphans, node) {
		unsigned long flags;

		cl = job->cl;

		dev_warn(&rpdev->dev,
			 "remote removed with graph %u req 0x%016llx outstanding; releasing %zu KiB\n",
			 job->graph_id, job->req_id, job->retained >> 10);

		spin_lock_irqsave(&cl->slock, flags);
		cl->job = NULL;
		cl->state = CVU_POISONED;
		spin_unlock_irqrestore(&cl->slock, flags);

		wake_up_interruptible(&cl->wq);

		timer_delete_sync(&cl->timer);
		list_del(&job->node);		/* off the local list */
		cvu_job_free(job);
		kref_put(&cl->ref, cvu_client_free);
	}

	kref_put(&c->ref, cvu_dev_free);	/* the driver's reference */
}

static struct rpmsg_device_id cvu_rpmsg_id_table[] = {
	{ .name = CVU_RPMSG_SERV_NAME },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, cvu_rpmsg_id_table);

static struct rpmsg_driver cvu_rpmsg_driver = {
	.drv.name	= "simaai-cvu",
	.id_table	= cvu_rpmsg_id_table,
	.probe		= cvu_rpmsg_probe,
	.remove		= cvu_rpmsg_remove,
};
module_rpmsg_driver(cvu_rpmsg_driver);

MODULE_AUTHOR("Nileshkumar Raghuvanshi <nilesh.r@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai CVU/EV74 job submission driver");
MODULE_IMPORT_NS("DMA_BUF");
MODULE_LICENSE("GPL");
