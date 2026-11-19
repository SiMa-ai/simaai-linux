// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 SiMa.ai
 *
 * KUnit coverage for the retire-then-reuse interleaving that made timeout
 * recovery act on the wrong command.  The slot is a reused
 * resource, so a timeout marker is only meaningful together with the
 * generation that raised it.
 */

#include <kunit/test.h>
#include <linux/moduleparam.h>

#include "al_ip.h"
#include "al_cmd.h"

/* Built in when CONFIG_ALLEGRO_CODEC_KUNIT is set, but only run when asked
 * for: pass simaai_vidip_kunit on the kernel command line.
 */
static bool simaai_vidip_kunit;
core_param(simaai_vidip_kunit, simaai_vidip_kunit, bool, 0444);

static int al5r_kunit_init(struct kunit *test)
{
	if (!simaai_vidip_kunit)
		kunit_skip(test,
			   "boot with simaai_vidip_kunit to run the video IP tests");
	return 0;
}

/* An idle slot cannot own recovery: there is nothing to recover. */
static void al5_stale_idle_slot(struct kunit *test)
{
	struct al5_slot slot = { .running = NULL, .timeout_gen = 7 };

	KUNIT_EXPECT_TRUE(test, al5_recovery_is_stale(&slot));
}

/* The ordinary case: the command that timed out is still in the slot. */
static void al5_stale_same_generation(struct kunit *test)
{
	struct al5_pending_cmd a = { .gen = 7 };
	struct al5_slot slot = { .running = &a, .timeout_gen = 7 };

	KUNIT_EXPECT_FALSE(test, al5_recovery_is_stale(&slot));
}

/* The defect: A timed out, A retired, B took the slot.  Recovery raised
 * for A must not be applied to B.
 */
static void al5_stale_retire_then_reuse(struct kunit *test)
{
	struct al5_pending_cmd a = { .gen = 7 };
	struct al5_pending_cmd b = { .gen = 8 };
	struct al5_slot slot = { .running = &a, .timeout_gen = 7 };

	/* timer marks the slot while A runs */
	KUNIT_EXPECT_FALSE(test, al5_recovery_is_stale(&slot));

	/* A completes by IRQ and B is started in the vacated slot */
	slot.running = &b;

	/* recovery finally runs: it must now be discarded */
	KUNIT_EXPECT_TRUE(test, al5_recovery_is_stale(&slot));
}

/* Generations are never reused, so a wrapped-looking value is still a
 * mismatch rather than an accidental match.
 */
static void al5_stale_distinct_generations(struct kunit *test)
{
	struct al5_pending_cmd b = { .gen = U64_MAX };
	struct al5_slot slot = { .running = &b, .timeout_gen = 1 };

	KUNIT_EXPECT_TRUE(test, al5_recovery_is_stale(&slot));

	slot.timeout_gen = U64_MAX;
	KUNIT_EXPECT_FALSE(test, al5_recovery_is_stale(&slot));
}

static struct kunit_case al5r_recovery_cases[] = {
	KUNIT_CASE(al5_stale_idle_slot),
	KUNIT_CASE(al5_stale_same_generation),
	KUNIT_CASE(al5_stale_retire_then_reuse),
	KUNIT_CASE(al5_stale_distinct_generations),
	{}
};

static struct kunit_suite al5r_recovery_suite = {
	.name = "al5r_recovery",
	.init = al5r_kunit_init,
	.test_cases = al5r_recovery_cases,
};
kunit_test_suite(al5r_recovery_suite);

MODULE_DESCRIPTION("KUnit tests for the AL5R command arbiter recovery path");
MODULE_LICENSE("GPL");
