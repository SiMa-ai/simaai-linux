// SPDX-License-Identifier: GPL-2.0
/*
 * Selftest for the SiMa.ai carveout dma-buf heap (/dev/dma_heap/simaai,dms).
 *
 * Exercises the behavior specific to the carveout heap on top of what the
 * generic dmabuf-heap selftest covers: the single-sg-entry size limit,
 * physical address reporting via DMA_BUF_IOCTL_GET_DESC, buffer isolation
 * in the gen_pool allocator, allocator leak-freedom over alloc/free cycles,
 * cache sync ioctl availability, and buffer lifetime past fd close.
 *
 * Usage: carveout-heap [heap-name]     (default: simaai,dms)
 *
 * Note: full cache-coherency validation (that begin/end_cpu_access really
 * cleans/invalidates) requires a non-coherent DMA master reading DRAM and
 * cannot be done CPU-only; see the DMA round-trip test described in the
 * commit message.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

#include "../kselftest.h"

/* Fallback when building against headers without the SiMa.ai extension. */
#ifndef DMA_BUF_IOCTL_GET_DESC
struct dma_buf_desc {
	__u64 paddr;
	__u64 size;
	__u64 busAddr;
	__u32 fd;
};
#define DMA_BUF_IOCTL_GET_DESC _IOWR(DMA_BUF_BASE, 4, struct dma_buf_desc)
#endif

#define DEFAULT_HEAP	"simaai,dms"
#define NUM_TESTS	10

static int heap_fd = -1;
static const char *heap_name = DEFAULT_HEAP;

static int heap_alloc(size_t len, int *dmabuf_fd)
{
	struct dma_heap_allocation_data data = {
		.len = len,
		.fd_flags = O_RDWR | O_CLOEXEC,
	};
	int ret;

	ret = ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &data);
	if (ret < 0)
		return -errno;
	*dmabuf_fd = data.fd;
	return 0;
}

static int buf_sync(int fd, uint64_t start_stop, uint64_t rw)
{
	struct dma_buf_sync sync = { .flags = start_stop | rw };

	return ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) ? -errno : 0;
}

static int buf_get_desc(int fd, struct dma_buf_desc *desc)
{
	memset(desc, 0, sizeof(*desc));
	return ioctl(fd, DMA_BUF_IOCTL_GET_DESC, desc) ? -errno : 0;
}

static void test_alloc_basic(void)
{
	const size_t len = 1 << 20;
	off_t size;
	int fd;

	if (heap_alloc(len, &fd)) {
		ksft_test_result_fail("alloc 1M: %s\n", strerror(errno));
		return;
	}
	size = lseek(fd, 0, SEEK_END);
	ksft_test_result(size == (off_t)len,
			 "alloc 1M: fd valid, size %lld\n", (long long)size);
	close(fd);
}

static void test_alloc_zero_fails(void)
{
	int fd = -1, ret = heap_alloc(0, &fd);

	if (!ret)
		close(fd);
	ksft_test_result(ret == -EINVAL, "zero-length alloc fails (%d)\n", ret);
}

static void test_alloc_over_4g_fails(void)
{
	/* One sg entry length is a u32: the heap must reject > 4 GiB. */
	int fd = -1, ret = heap_alloc(5ULL << 30, &fd);

	if (!ret)
		close(fd);
	ksft_test_result(ret == -E2BIG || ret == -ENOMEM,
			 ">4GiB alloc rejected (%d)\n", ret);
}

static void test_mmap_pattern(void)
{
	const size_t len = 2 << 20;
	uint8_t *p;
	size_t i, bad = 0;
	int fd;

	if (heap_alloc(len, &fd)) {
		ksft_test_result_fail("mmap pattern: alloc failed\n");
		return;
	}
	p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED) {
		ksft_test_result_fail("mmap pattern: mmap: %s\n", strerror(errno));
		close(fd);
		return;
	}
	buf_sync(fd, DMA_BUF_SYNC_START, DMA_BUF_SYNC_WRITE);
	for (i = 0; i < len; i++)
		p[i] = (uint8_t)(i * 63 + 1);
	buf_sync(fd, DMA_BUF_SYNC_END, DMA_BUF_SYNC_WRITE);

	buf_sync(fd, DMA_BUF_SYNC_START, DMA_BUF_SYNC_READ);
	for (i = 0; i < len; i++)
		if (p[i] != (uint8_t)(i * 63 + 1))
			bad++;
	buf_sync(fd, DMA_BUF_SYNC_END, DMA_BUF_SYNC_READ);

	ksft_test_result(bad == 0, "mmap write/readback (%zu bad bytes)\n", bad);
	munmap(p, len);
	close(fd);
}

static void test_mmap_too_big_fails(void)
{
	const size_t len = 1 << 20;
	void *p;
	int fd;

	if (heap_alloc(len, &fd)) {
		ksft_test_result_fail("mmap-too-big: alloc failed\n");
		return;
	}
	p = mmap(NULL, 2 * len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (p != MAP_FAILED)
		munmap(p, 2 * len);
	ksft_test_result(p == MAP_FAILED, "mmap beyond buffer size fails\n");
	close(fd);
}

static void test_get_desc(void)
{
	const size_t len = 1 << 20;
	struct dma_buf_desc desc;
	int fd, ret;

	if (heap_alloc(len, &fd)) {
		ksft_test_result_fail("get_desc: alloc failed\n");
		return;
	}
	ret = buf_get_desc(fd, &desc);
	/*
	 * Carveout memory has no STU window: the accelerators address it
	 * directly, so the bus address must equal the physical address.
	 */
	ksft_test_result(!ret && desc.paddr != (uint64_t)-1 && desc.paddr &&
			 !(desc.paddr & 0xfff) && desc.size == len &&
			 (!strstr(heap_name, "dms") ||
			  desc.busAddr == desc.paddr),
			 "GET_DESC: paddr 0x%llx bus 0x%llx size %llu\n",
			 (unsigned long long)desc.paddr,
			 (unsigned long long)desc.busAddr,
			 (unsigned long long)desc.size);
	close(fd);
}

static void test_buffer_isolation(void)
{
	const size_t len = 1 << 20;
	struct dma_buf_desc d1, d2;
	uint8_t *p1, *p2;
	size_t i, bad = 0;
	int fd1, fd2;

	if (heap_alloc(len, &fd1) || heap_alloc(len, &fd2)) {
		ksft_test_result_fail("isolation: alloc failed\n");
		return;
	}
	if (buf_get_desc(fd1, &d1) || buf_get_desc(fd2, &d2)) {
		ksft_test_result_fail("isolation: GET_DESC failed\n");
		goto out_close;
	}
	/* Physical ranges must not overlap. */
	if (d1.paddr < d2.paddr + len && d2.paddr < d1.paddr + len) {
		ksft_test_result_fail("isolation: ranges overlap (0x%llx 0x%llx)\n",
				      (unsigned long long)d1.paddr,
				      (unsigned long long)d2.paddr);
		goto out_close;
	}
	p1 = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd1, 0);
	p2 = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd2, 0);
	if (p1 == MAP_FAILED || p2 == MAP_FAILED) {
		ksft_test_result_fail("isolation: mmap failed\n");
		goto out_close;
	}
	memset(p1, 0xAA, len);
	memset(p2, 0x55, len);
	for (i = 0; i < len; i++) {
		if (p1[i] != 0xAA)
			bad++;
		if (p2[i] != 0x55)
			bad++;
	}
	ksft_test_result(bad == 0, "two buffers isolated (%zu bad bytes)\n", bad);
	munmap(p1, len);
	munmap(p2, len);
out_close:
	close(fd1);
	close(fd2);
}

static void test_sync_ioctl(void)
{
	const size_t len = 1 << 20;
	int fd, ret = 0;

	if (heap_alloc(len, &fd)) {
		ksft_test_result_fail("sync: alloc failed\n");
		return;
	}
	ret |= buf_sync(fd, DMA_BUF_SYNC_START, DMA_BUF_SYNC_READ);
	ret |= buf_sync(fd, DMA_BUF_SYNC_END, DMA_BUF_SYNC_READ);
	ret |= buf_sync(fd, DMA_BUF_SYNC_START, DMA_BUF_SYNC_WRITE);
	ret |= buf_sync(fd, DMA_BUF_SYNC_END, DMA_BUF_SYNC_WRITE);
	ret |= buf_sync(fd, DMA_BUF_SYNC_START, DMA_BUF_SYNC_RW);
	ret |= buf_sync(fd, DMA_BUF_SYNC_END, DMA_BUF_SYNC_RW);
	ksft_test_result(ret == 0, "SYNC ioctl for READ/WRITE/RW (%d)\n", ret);
	close(fd);
}

static void test_alloc_free_cycles(void)
{
	const size_t len = 16 << 20;
	int i, fd = -1;

	/*
	 * If allocation or release leaked gen_pool space or mappings, a
	 * long alloc/free sequence would start failing.
	 */
	for (i = 0; i < 200; i++) {
		if (heap_alloc(len, &fd)) {
			ksft_test_result_fail("alloc/free cycle %d: %s\n",
					      i, strerror(errno));
			return;
		}
		close(fd);
	}
	ksft_test_result_pass("200 alloc/free cycles of 16M\n");
}

static void test_mmap_outlives_fd(void)
{
	const size_t len = 1 << 20;
	uint8_t *p;
	int fd;

	if (heap_alloc(len, &fd)) {
		ksft_test_result_fail("outlive: alloc failed\n");
		return;
	}
	p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED) {
		ksft_test_result_fail("outlive: mmap failed\n");
		close(fd);
		return;
	}
	p[0] = 0x5A;
	close(fd);
	/* The mapping holds a reference; the buffer must stay usable. */
	p[len - 1] = 0xA5;
	ksft_test_result(p[0] == 0x5A && p[len - 1] == 0xA5,
			 "mmap keeps buffer alive after close(fd)\n");
	munmap(p, len);
}

int main(int argc, char *argv[])
{
	char path[256];

	ksft_print_header();

	if (argc > 1)
		heap_name = argv[1];
	snprintf(path, sizeof(path), "/dev/dma_heap/%s", heap_name);
	heap_fd = open(path, O_RDONLY | O_CLOEXEC);
	if (heap_fd < 0)
		ksft_exit_skip("%s not present (%s)\n", path, strerror(errno));

	ksft_set_plan(NUM_TESTS);

	test_alloc_basic();
	test_alloc_zero_fails();
	test_alloc_over_4g_fails();
	test_mmap_pattern();
	test_mmap_too_big_fails();
	test_get_desc();
	test_buffer_isolation();
	test_sync_ioctl();
	test_alloc_free_cycles();
	test_mmap_outlives_fd();

	close(heap_fd);
	ksft_finished();
}
