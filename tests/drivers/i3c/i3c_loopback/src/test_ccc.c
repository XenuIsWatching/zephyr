/*
 * Copyright (c) 2026 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Common Command Code coverage. Expected target identity values are
 * queried at runtime from i3c_config_get_target() so the test never
 * duplicates DT properties.
 */

#include "common.h"

#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/ztest.h>

static struct i3c_config_target tcfg;

static void ccc_before(void *fixture)
{
	ARG_UNUSED(fixture);
	zassert_ok(loopback_redo_daa());
	loopback_reset_state();
	zassert_ok(i3c_config_get_target(loopback_target, &tcfg));
}

ZTEST(i3c_loopback_ccc, test_getbcr)
{
	struct i3c_ccc_getbcr p = { .bcr = 0 };

	zassert_ok(i3c_ccc_do_getbcr(loopback_desc, &p));
	zassert_equal(tcfg.bcr, p.bcr, "expected BCR 0x%02x got 0x%02x", tcfg.bcr, p.bcr);
}

ZTEST(i3c_loopback_ccc, test_getdcr)
{
	struct i3c_ccc_getdcr p = { .dcr = 0 };

	zassert_ok(i3c_ccc_do_getdcr(loopback_desc, &p));
	zassert_equal(tcfg.dcr, p.dcr);
}

ZTEST(i3c_loopback_ccc, test_getpid)
{
	struct i3c_ccc_getpid p;
	uint64_t got = 0;

	zassert_ok(i3c_ccc_do_getpid(loopback_desc, &p));
	for (int i = 0; i < 6; i++) {
		got = (got << 8) | p.pid[i];
	}
	zassert_equal(tcfg.pid, got, "expected PID 0x%012llx got 0x%012llx", tcfg.pid, got);
}

ZTEST(i3c_loopback_ccc, test_getstatus_fmt1)
{
	union i3c_ccc_getstatus status = {0};

	zassert_ok(i3c_ccc_do_getstatus_fmt1(loopback_desc, &status));
}

ZTEST(i3c_loopback_ccc, test_getcaps_fmt1)
{
	union i3c_ccc_getcaps caps = {0};

	/* GETCAPS is mandatory for I3C v1.1+ devices, and for v1.0 devices
	 * only when BCR.ADV_CAPABILITIES is set. Skip when neither applies.
	 */
	if ((loopback_desc->flags & I3C_V1P0_SUPPORT) &&
	    (tcfg.bcr & I3C_BCR_ADV_CAPABILITIES) == 0) {
		ztest_test_skip();
	}

	zassert_ok(i3c_ccc_do_getcaps_fmt1(loopback_desc, &caps));
}

ZTEST(i3c_loopback_ccc, test_getmxds)
{
	union i3c_ccc_getmxds mxds = {0};

	if ((tcfg.bcr & I3C_BCR_MAX_DATA_SPEED_LIMIT) == 0) {
		ztest_test_skip();
	}
	zassert_ok(i3c_ccc_do_getmxds_fmt1(loopback_desc, &mxds));
}

ZTEST(i3c_loopback_ccc, test_getmrl)
{
	struct i3c_ccc_mrl mrl = {0};

	zassert_ok(i3c_ccc_do_getmrl(loopback_desc, &mrl));
	zassert_equal(tcfg.max_read_len, mrl.len,
		      "expected MRL %u got %u", tcfg.max_read_len, mrl.len);
}

ZTEST(i3c_loopback_ccc, test_getmwl)
{
	struct i3c_ccc_mwl mwl = {0};

	zassert_ok(i3c_ccc_do_getmwl(loopback_desc, &mwl));
	zassert_equal(tcfg.max_write_len, mwl.len,
		      "expected MWL %u got %u", tcfg.max_write_len, mwl.len);
}

ZTEST(i3c_loopback_ccc, test_setmrl_then_getmrl_roundtrip)
{
	struct i3c_ccc_mrl set = { .len = 0x40 };
	struct i3c_ccc_mrl get = {0};

	zassert_ok(i3c_ccc_do_setmrl(loopback_desc, &set));
	zassert_ok(i3c_ccc_do_getmrl(loopback_desc, &get));
	zassert_equal(set.len, get.len, "MRL roundtrip: set %u got %u", set.len, get.len);
}

ZTEST(i3c_loopback_ccc, test_setmwl_then_getmwl_roundtrip)
{
	struct i3c_ccc_mwl set = { .len = 0x20 };
	struct i3c_ccc_mwl get = {0};

	zassert_ok(i3c_ccc_do_setmwl(loopback_desc, &set));
	zassert_ok(i3c_ccc_do_getmwl(loopback_desc, &get));
	zassert_equal(set.len, get.len, "MWL roundtrip: set %u got %u", set.len, get.len);
}

ZTEST(i3c_loopback_ccc, test_enec_disec_intr_bit)
{
	struct i3c_ccc_events events = { .events = I3C_CCC_ENEC_EVT_ENINTR };

	zassert_ok(i3c_ccc_do_events_set(loopback_desc, false, &events));
	zassert_ok(i3c_ccc_do_events_set(loopback_desc, true, &events));
}

ZTEST(i3c_loopback_ccc, test_rstact_resets_target)
{
	int ret = i3c_ccc_do_rstact_fmt2(loopback_desc, I3C_CCC_RSTACT_PERIPHERAL_ONLY);

	/* RSTACT support is target-defined; both ENOTSUP (target NACK'd it)
	 * and OK are acceptable outcomes.
	 */
	zassert_true(ret == 0 || ret == -ENOTSUP, "unexpected RSTACT ret %d", ret);
}

ZTEST_SUITE(i3c_loopback_ccc, NULL, NULL, ccc_before, NULL, NULL);
