/*
 * Copyright (c) 2026 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DAA flow coverage via i3c_bus_* wrappers (which keep the device
 * desc in sync, unlike the lower-level i3c_ccc_do_* calls).
 */

#include "common.h"

#include <zephyr/drivers/i3c.h>
#include <zephyr/ztest.h>

#define WAIT K_MSEC(500)

static uint8_t pre_da;

static void daa_before(void *fixture)
{
	ARG_UNUSED(fixture);
	zassert_ok(loopback_redo_daa());
	pre_da = loopback_desc->dynamic_addr;
	zassert_not_equal(0, pre_da, "DAA didn't assign a dynamic addr");
	loopback_reset_state();
}

ZTEST(i3c_loopback_daa, test_rstdaa_clears_address)
{
	uint8_t b = 0xAA;
	struct i3c_msg msg = {
		.buf = &b, .len = 1, .flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};

	zassert_ok(i3c_bus_rstdaa_all(loopback_controller));
	zassert_equal(0, loopback_desc->dynamic_addr,
		      "i3c_bus_rstdaa_all should clear desc->dynamic_addr");

	int ret = i3c_transfer(loopback_desc, &msg, 1);

	zassert_not_equal(0, ret, "transfer should fail after RSTDAA");

	/* Restore for next test. */
	zassert_ok(loopback_redo_daa());
}

ZTEST(i3c_loopback_daa, test_entdaa_assigns_address)
{
	uint8_t b = 0x5A;
	struct i3c_msg msg = {
		.buf = &b, .len = 1, .flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};

	/* loopback_redo_daa() already ran in `before`. Just verify. */
	zassert_not_equal(0, loopback_desc->dynamic_addr);
	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
}

ZTEST(i3c_loopback_daa, test_setnewda_changes_address)
{
	uint8_t newda_addr = (pre_da == 0x5F) ? 0x60 : 0x5F;
	uint8_t b = 0x12;
	struct i3c_msg msg = {
		.buf = &b, .len = 1, .flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};

	zassert_ok(i3c_bus_setnewda(loopback_desc, newda_addr));
	zassert_equal(newda_addr, loopback_desc->dynamic_addr,
		      "i3c_bus_setnewda should update desc->dynamic_addr");

	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));

	/* Restore. */
	zassert_ok(loopback_redo_daa());
}

ZTEST(i3c_loopback_daa, test_setdasa_with_static_addr)
{
	struct i3c_config_target tcfg;
	uint8_t b = 0x77;
	struct i3c_msg msg = {
		.buf = &b, .len = 1, .flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};

	zassert_ok(i3c_config_get_target(loopback_target, &tcfg));
	if (tcfg.static_addr == 0) {
		ztest_test_skip();
	}

	zassert_ok(i3c_bus_rstdaa_all(loopback_controller));
	zassert_ok(i3c_bus_setdasa(loopback_desc, tcfg.static_addr + 1));
	zassert_equal(tcfg.static_addr + 1, loopback_desc->dynamic_addr);

	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));

	zassert_ok(loopback_redo_daa());
}

ZTEST(i3c_loopback_daa, test_setaasa_with_static_addr)
{
	struct i3c_config_target tcfg;
	uint8_t b = 0x88;
	struct i3c_msg msg = {
		.buf = &b, .len = 1, .flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};

	zassert_ok(i3c_config_get_target(loopback_target, &tcfg));
	if (tcfg.static_addr == 0 ||
	    (loopback_desc->flags & I3C_SUPPORTS_SETAASA) == 0) {
		ztest_test_skip();
	}

	zassert_ok(i3c_bus_rstdaa_all(loopback_controller));
	zassert_ok(i3c_bus_setaasa(loopback_controller));
	zassert_equal(tcfg.static_addr, loopback_desc->dynamic_addr,
		      "SETAASA should set DA == static_addr (%u, got %u)",
		      tcfg.static_addr, loopback_desc->dynamic_addr);

	/* Verify a real transfer round-trips at the static-as-dynamic address. */
	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));

	zassert_ok(loopback_redo_daa());
}

ZTEST(i3c_loopback_daa, test_device_info_get_after_daa)
{
	struct i3c_config_target tcfg;

	zassert_ok(i3c_device_info_get(loopback_desc));
	zassert_ok(i3c_config_get_target(loopback_target, &tcfg));
	zassert_equal(tcfg.pid, loopback_desc->pid);
	zassert_equal(tcfg.bcr, loopback_desc->bcr);
	zassert_equal(tcfg.dcr, loopback_desc->dcr);
}

ZTEST_SUITE(i3c_loopback_daa, NULL, NULL, daa_before, NULL, NULL);
