/*
 * Copyright (c) 2026 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * i3c_configure / i3c_config_get round-trip on both controller and
 * target sides, plus a transfer-at-new-speed sanity check.
 */

#include "common.h"

#include <string.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/ztest.h>

#define WAIT K_MSEC(500)

static struct i3c_config_controller saved_ctrl_cfg;

static void configure_before(void *fixture)
{
	ARG_UNUSED(fixture);
	zassert_ok(loopback_redo_daa());
	loopback_reset_state();
	zassert_ok(i3c_config_get_controller(loopback_controller, &saved_ctrl_cfg));
}

static void configure_after(void *fixture)
{
	ARG_UNUSED(fixture);
	/* Always restore the pre-test controller config so a slow-SCL test
	 * doesn't poison downstream suites.
	 */
	(void)i3c_configure_controller(loopback_controller, &saved_ctrl_cfg);
}

ZTEST(i3c_loopback_configure, test_get_controller_config)
{
	struct i3c_config_controller cfg = {0};

	zassert_ok(i3c_config_get_controller(loopback_controller, &cfg));
	zassert_true(cfg.scl.i3c > 0, "i3c SCL must be non-zero");
}

ZTEST(i3c_loopback_configure, test_get_target_config)
{
	struct i3c_config_target cfg = {0};

	zassert_ok(i3c_config_get_target(loopback_target, &cfg));
	zassert_not_equal(0, cfg.pid, "target should report a PID");
}

ZTEST(i3c_loopback_configure, test_set_controller_scl_then_roundtrip)
{
	struct i3c_config_controller cfg = saved_ctrl_cfg;
	struct i3c_config_controller readback = {0};

	cfg.scl.i3c = 1000000;
	zassert_ok(i3c_configure_controller(loopback_controller, &cfg));
	zassert_ok(i3c_config_get_controller(loopback_controller, &readback));

	/* Allow some implementations to round to nearest achievable. */
	zassert_true(readback.scl.i3c > 0 && readback.scl.i3c <= cfg.scl.i3c * 2,
		     "SCL readback %u out of range", readback.scl.i3c);
}

ZTEST(i3c_loopback_configure, test_set_controller_scl_then_transfer_succeeds)
{
	struct i3c_config_controller cfg = saved_ctrl_cfg;
	uint8_t b = 0x99;
	struct i3c_msg msg = {
		.buf = &b, .len = 1, .flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};

	cfg.scl.i3c = 1000000;
	zassert_ok(i3c_configure_controller(loopback_controller, &cfg));

	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
}

ZTEST(i3c_loopback_configure, test_set_controller_scl_restore)
{
	uint8_t b = 0xAA;
	struct i3c_msg msg = {
		.buf = &b, .len = 1, .flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};

	zassert_ok(i3c_configure_controller(loopback_controller, &saved_ctrl_cfg));
	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
}

ZTEST(i3c_loopback_configure, test_set_controller_invalid_type_rejects)
{
	int ret = i3c_configure(loopback_controller, I3C_CONFIG_CUSTOM, NULL);

	/* Accept -ENOTSUP, -EINVAL, or driver-specific negative. */
	zassert_not_equal(0, ret, "I3C_CONFIG_CUSTOM with NULL should not succeed");
}

ZTEST_SUITE(i3c_loopback_configure, NULL, NULL, configure_before, configure_after, NULL);
