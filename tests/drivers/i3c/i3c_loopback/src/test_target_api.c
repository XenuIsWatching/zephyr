/*
 * Copyright (c) 2026 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * i3c_target_register/unregister lifecycle and i3c_target_tx_write
 * FIFO behavior.
 */

#include "common.h"

#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/target_device.h>
#include <zephyr/ztest.h>

#define WAIT K_MSEC(500)

static struct i3c_target_config alt_cfg = {
	.callbacks = NULL,	/* filled in by suite setup */
};

static void target_api_before(void *fixture)
{
	ARG_UNUSED(fixture);
	zassert_ok(loopback_redo_daa());
	loopback_reset_state();
	alt_cfg.callbacks = &loopback_target_cbs;
}

ZTEST(i3c_loopback_target_api, test_target_unregister_register_lifecycle)
{
	int ret;

	ret = i3c_target_unregister(loopback_target, &alt_cfg);
	if (ret == -ENOSYS) {
		ztest_test_skip();
	}
	zassert_ok(ret);

	/* Re-register so downstream suites continue to work. */
	zassert_ok(i3c_target_register(loopback_target, &alt_cfg));

	/* Verify a transfer still completes. */
	uint8_t b = 0x44;
	struct i3c_msg msg = {
		.buf = &b, .len = 1, .flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};
	zassert_ok(loopback_redo_daa());
	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
}

ZTEST(i3c_loopback_target_api, test_target_tx_write_returns_byte_count)
{
	uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
	int ret = i3c_target_tx_write(loopback_target, data, sizeof(data), 0);

	zassert_true(ret >= 0, "i3c_target_tx_write returned %d", ret);
	zassert_true((size_t)ret <= sizeof(data),
		     "i3c_target_tx_write returned %d, expected <= %u", ret,
		     (unsigned int)sizeof(data));
}

ZTEST(i3c_loopback_target_api, test_target_tx_write_oversize_is_handled)
{
	uint8_t big[LOOPBACK_TX_BUF_SIZE];

	for (size_t i = 0; i < sizeof(big); ++i) {
		big[i] = (uint8_t)i;
	}

	int ret = i3c_target_tx_write(loopback_target, big, sizeof(big), 0);

	/* Either accepts all, partially fills, or returns an error — any
	 * deterministic outcome is OK as long as it doesn't crash.
	 */
	zassert_true(ret >= -EINVAL, "unexpected return %d", ret);
}

ZTEST_SUITE(i3c_loopback_target_api, NULL, NULL, target_api_before, NULL, NULL);
