/*
 * Copyright (c) 2026 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * attach / reattach / detach lifecycle (i3c side).
 */

#include "common.h"

#include <zephyr/drivers/i3c.h>
#include <zephyr/ztest.h>

#define WAIT K_MSEC(500)

static void attach_before(void *fixture)
{
	ARG_UNUSED(fixture);
	zassert_ok(loopback_redo_daa());
	loopback_reset_state();
}

static int do_one_write(uint8_t payload)
{
	struct i3c_msg msg = {
		.buf = &payload, .len = 1, .flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};
	return i3c_transfer(loopback_desc, &msg, 1);
}

ZTEST(i3c_loopback_attach, test_attach_then_xfer_succeeds)
{
	/* The desc is already attached by SYS_INIT — verify a transfer works. */
	zassert_ok(do_one_write(0x10));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
}

ZTEST(i3c_loopback_attach, test_detach_then_xfer_fails)
{
	int ret;

	zassert_ok(i3c_detach_i3c_device(loopback_desc));

	ret = do_one_write(0x11);
	zassert_not_equal(0, ret, "transfer after detach should fail");

	/* Restore: re-attach and redo DAA so downstream tests work. */
	zassert_ok(i3c_attach_i3c_device(loopback_desc));
	zassert_ok(loopback_redo_daa());
}

ZTEST(i3c_loopback_attach, test_reattach_after_setnewda)
{
	uint8_t newda = (loopback_desc->dynamic_addr == 0x55) ? 0x56 : 0x55;

	zassert_ok(i3c_bus_setnewda(loopback_desc, newda));
	zassert_equal(newda, loopback_desc->dynamic_addr);

	zassert_ok(do_one_write(0x12));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));

	zassert_ok(loopback_redo_daa());
}

ZTEST(i3c_loopback_attach, test_detach_unknown_returns_status)
{
	struct i3c_device_desc bogus = *loopback_desc;
	int ret;

	bogus.controller_priv = NULL;
	ret = i3c_detach_i3c_device(&bogus);

	zassert_true(ret == 0 || ret == -EINVAL || ret == -ENODEV,
		     "detach of unknown desc should return 0/-EINVAL/-ENODEV, got %d", ret);
}

ZTEST(i3c_loopback_attach, test_reattach_keeps_transfer_working)
{
	uint8_t old_da = loopback_desc->dynamic_addr;

	/* No-op reattach (same DA) — should keep the slot mapping intact. */
	zassert_ok(i3c_reattach_i3c_device(loopback_desc, old_da));
	zassert_ok(do_one_write(0x13));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
}

ZTEST_SUITE(i3c_loopback_attach, NULL, NULL, attach_before, NULL, NULL);
