// SPDX-License-Identifier: GPL-2.0
/*
 * Run on an idle Modalix board with reusable DMS; on legacy boards,
 * force fallback first. Fill DMS before passing sysram/cma to require
 * that tier. CPU readback checks isolation, not accelerator coherency.
 */
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

#include "../kselftest.h"

#ifndef DMA_BUF_IOCTL_GET_DESC
struct dma_buf_desc {
	__u64 paddr;
	__u64 size;
	__u64 busAddr;
	__u32 fd;
};

#define DMA_BUF_IOCTL_GET_DESC _IOWR(DMA_BUF_BASE, 4, struct dma_buf_desc)
#endif

#define BUFFER_SIZE (1024 * 1024)
#define WORKERS 4

static int heap_fd;

struct allocation {
	int fd;
	unsigned char *mapping;
	uint64_t physical;
};

static int allocate(void)
{
	struct dma_heap_allocation_data data = {
		.len = BUFFER_SIZE,
		.fd_flags = O_RDWR | O_CLOEXEC,
	};

	if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &data))
		return -errno;
	return data.fd;
}

static bool sync_buffer(int fd, uint64_t flags)
{
	struct dma_buf_sync sync = { .flags = flags };

	return !ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
}

static bool counter(const char *name, uint64_t *value)
{
	char key[64];
	unsigned long long result;
	FILE *file;
	bool ok = false;

	file = fopen("/sys/kernel/debug/mla_heap/stats", "r");
	if (!file)
		return false;
	while (fscanf(file, "%63s %llu", key, &result) == 2) {
		if (!strcmp(key, name)) {
			*value = result;
			ok = true;
			break;
		}
	}
	fclose(file);
	return ok;
}

static bool owned_bytes(uint64_t *value)
{
	uint64_t sysram, cma;

	if (!counter("sysram_bytes", &sysram) || !counter("cma_bytes", &cma))
		return false;
	*value = sysram + cma;
	return true;
}

static void free_allocation(struct allocation *a)
{
	if (a->mapping != MAP_FAILED)
		munmap(a->mapping, BUFFER_SIZE);
	if (a->fd >= 0)
		close(a->fd);
}

static bool create_allocation(struct allocation *a, unsigned char pattern)
{
	struct dma_buf_desc desc = {};

	*a = (struct allocation){ .fd = -1, .mapping = MAP_FAILED };
	a->fd = allocate();
	if (a->fd < 0 || ioctl(a->fd, DMA_BUF_IOCTL_GET_DESC, &desc))
		return false;
	if (!desc.paddr || desc.paddr > (1ULL << 40) - BUFFER_SIZE ||
	    desc.size != BUFFER_SIZE || (desc.paddr & (getpagesize() - 1)))
		return false;
	a->physical = desc.paddr;
	a->mapping = mmap(NULL, BUFFER_SIZE, PROT_READ | PROT_WRITE,
			  MAP_SHARED, a->fd, 0);
	if (a->mapping == MAP_FAILED ||
	    !sync_buffer(a->fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW))
		return false;
	memset(a->mapping, pattern, BUFFER_SIZE);
	return sync_buffer(a->fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW);
}

struct worker {
	struct allocation allocation;
	unsigned char pattern;
	bool ok;
};

static void *allocate_worker(void *arg)
{
	struct worker *worker = arg;

	worker->ok = create_allocation(&worker->allocation, worker->pattern);
	return NULL;
}

static void test_concurrent(const char *tier)
{
	struct worker workers[WORKERS] = {};
	pthread_t threads[WORKERS];
	unsigned int count = 0, i, j;
	uint64_t before = 0, during;
	bool have_tier = tier && counter(tier, &before);
	bool ok;

	for (i = 0; i < WORKERS; ++i) {
		workers[i].pattern = i + 1;
		if (pthread_create(&threads[i], NULL, allocate_worker, &workers[i]))
			break;
		count++;
	}
	for (i = 0; i < count; ++i)
		pthread_join(threads[i], NULL);
	ok = count == WORKERS;
	for (i = 0; i < count; ++i) {
		struct allocation *a = &workers[i].allocation;
		size_t offset;

		ok &= workers[i].ok;
		if (!workers[i].ok)
			continue;
		for (j = 0; j < i; ++j) {
			uint64_t other = workers[j].allocation.physical;

			ok &= a->physical + BUFFER_SIZE <= other ||
			      other + BUFFER_SIZE <= a->physical;
		}
		ok &= sync_buffer(a->fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
		for (offset = 0; offset < BUFFER_SIZE; ++offset)
			if (a->mapping[offset] != workers[i].pattern)
				ok = false;
		ok &= sync_buffer(a->fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
	}
	ksft_test_result(ok, "concurrent allocations are addressable and isolated\n");
	if (tier)
		ksft_test_result(have_tier && counter(tier, &during) &&
				 during >= before + WORKERS * BUFFER_SIZE,
				 "%s accounts for the held buffers\n", tier);
	else
		ksft_test_result_skip("pass sysram/cma with DMS full to require fallback\n");
	for (i = 0; i < count; ++i)
		free_allocation(&workers[i].allocation);
}

int main(int argc, char **argv)
{
	uint64_t before, after;
	const char *tier = NULL;
	bool have_bytes;

	ksft_print_header();
	if (argc > 2 ||
	    (argc == 2 && strcmp(argv[1], "sysram") && strcmp(argv[1], "cma")))
		ksft_exit_fail_msg("usage: %s [sysram|cma]\n", argv[0]);
	if (argc == 2)
		tier = !strcmp(argv[1], "sysram") ? "sysram_bytes" : "cma_bytes";
	heap_fd = open("/dev/dma_heap/simaai,mla", O_RDONLY | O_CLOEXEC);
	if (heap_fd < 0 && errno == ENOENT)
		ksft_exit_skip("simaai,mla heap is unavailable\n");
	if (heap_fd < 0)
		ksft_exit_fail_msg("open simaai,mla: %s\n", strerror(errno));
	ksft_set_plan(3);
	have_bytes = owned_bytes(&before);
	test_concurrent(tier);
	if (have_bytes && owned_bytes(&after))
		ksft_test_result(before == after, "fallback bytes return to their initial value\n");
	else
		ksft_test_result_skip("fallback byte counters unavailable\n");
	close(heap_fd);
	ksft_finished();
}
