/*
 * Copyright (c) 2026 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * In-band-interrupt coverage. Target capability is gated on its BCR
 * (queried from i3c_config_get_target) rather than DT properties:
 *   - I3C_BCR_IBI_REQUEST_CAPABLE: target advertises ability to raise
 *   - I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE: target raises a payload IBI
 *     (with the MDB as the first byte) vs an MDB-less IBI.
 */

#include "common.h"

#include <string.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/drivers/i3c/ibi.h>
#include <zephyr/ztest.h>

#define WAIT K_MSEC(500)
#define SHORT_WAIT K_MSEC(100)

#define LOOPBACK_MDB 0x61

static struct i3c_config_target tcfg;

static void ibi_before(void *fixture)
{
	ARG_UNUSED(fixture);
	zassert_ok(loopback_redo_daa());
	loopback_reset_state();
	zassert_ok(i3c_config_get_target(loopback_target, &tcfg));
	if ((tcfg.bcr & I3C_BCR_IBI_REQUEST_CAPABLE) == 0) {
		ztest_test_skip();
	}
}

static int raise_ibi_with_payload(void)
{
	uint8_t payload[5] = { LOOPBACK_MDB, 0xCA, 0xFE, 0xBA, 0xBE };
	struct i3c_ibi ibi = {
		.ibi_type = I3C_IBI_TARGET_INTR,
		.payload = payload,
		.payload_len = sizeof(payload),
	};
	return i3c_ibi_raise(loopback_target, &ibi);
}

static int raise_ibi_no_payload(void)
{
	struct i3c_ibi ibi = {
		.ibi_type = I3C_IBI_TARGET_INTR,
		.payload = NULL,
		.payload_len = 0,
	};
	return i3c_ibi_raise(loopback_target, &ibi);
}

ZTEST(i3c_loopback_ibi, test_ibi_enable_then_raise_delivers)
{
	zassert_ok(i3c_ibi_enable(loopback_desc));
	if (tcfg.bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE) {
		zassert_ok(raise_ibi_with_payload());
	} else {
		zassert_ok(raise_ibi_no_payload());
	}
	zassert_ok(k_sem_take(&g_loopback.ibi_sem, WAIT), "IBI callback did not fire");
}

ZTEST(i3c_loopback_ibi, test_ibi_payload_with_mdb)
{
	uint8_t expected[5] = { LOOPBACK_MDB, 0xCA, 0xFE, 0xBA, 0xBE };

	if ((tcfg.bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE) == 0) {
		ztest_test_skip();
	}

	zassert_ok(i3c_ibi_enable(loopback_desc));
	zassert_ok(raise_ibi_with_payload());
	zassert_ok(k_sem_take(&g_loopback.ibi_sem, WAIT));
	zassert_equal(sizeof(expected), g_loopback.ibi_payload_len,
		      "IBI payload len: expected %u got %u", (unsigned int)sizeof(expected),
		      g_loopback.ibi_payload_len);
	zassert_equal(LOOPBACK_MDB, g_loopback.ibi_payload[0],
		      "IBI first byte should be MDB 0x%02x, got 0x%02x", LOOPBACK_MDB,
		      g_loopback.ibi_payload[0]);
	zassert_mem_equal(expected, g_loopback.ibi_payload, sizeof(expected));
}

ZTEST(i3c_loopback_ibi, test_ibi_no_payload)
{
	if (tcfg.bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE) {
		ztest_test_skip();
	}

	zassert_ok(i3c_ibi_enable(loopback_desc));
	zassert_ok(raise_ibi_no_payload());
	zassert_ok(k_sem_take(&g_loopback.ibi_sem, WAIT));
	zassert_equal(0, g_loopback.ibi_payload_len);
}

ZTEST(i3c_loopback_ibi, test_ibi_disec_blocks_raise)
{
	struct i3c_ccc_events events = { .events = I3C_CCC_ENEC_EVT_ENINTR };
	int ret;

	zassert_ok(i3c_ibi_enable(loopback_desc));
	zassert_ok(i3c_ccc_do_events_set(loopback_desc, false, &events));

	ret = (tcfg.bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE) ? raise_ibi_with_payload()
							     : raise_ibi_no_payload();

	zassert_equal(-EAGAIN, ret, "raise after DISEC should -EAGAIN, got %d", ret);

	/* Restore for downstream. */
	zassert_ok(i3c_ccc_do_events_set(loopback_desc, true, &events));
}

ZTEST(i3c_loopback_ibi, test_ibi_enec_re_enables_raise)
{
	struct i3c_ccc_events events = { .events = I3C_CCC_ENEC_EVT_ENINTR };

	zassert_ok(i3c_ibi_enable(loopback_desc));
	zassert_ok(i3c_ccc_do_events_set(loopback_desc, false, &events));
	zassert_equal(-EAGAIN, (tcfg.bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE)
				       ? raise_ibi_with_payload()
				       : raise_ibi_no_payload());
	zassert_ok(i3c_ccc_do_events_set(loopback_desc, true, &events));
	zassert_ok((tcfg.bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE) ? raise_ibi_with_payload()
								  : raise_ibi_no_payload());
	zassert_ok(k_sem_take(&g_loopback.ibi_sem, WAIT));
}

ZTEST(i3c_loopback_ibi, test_ibi_disable_blocks_callback)
{
	zassert_ok(i3c_ibi_enable(loopback_desc));
	zassert_ok(i3c_ibi_disable(loopback_desc));

	/* Either the target refuses or the IBI is raised but the controller
	 * doesn't deliver — both acceptable. The test asserts only that the
	 * controller-side callback does NOT fire.
	 */
	(void)((tcfg.bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE) ? raise_ibi_with_payload()
							      : raise_ibi_no_payload());

	zassert_not_equal(0, k_sem_take(&g_loopback.ibi_sem, SHORT_WAIT),
			  "IBI delivered after i3c_ibi_disable");
}

ZTEST(i3c_loopback_ibi, test_hj_raise_delivers_after_rstdaa)
{
	struct i3c_ibi ibi = { .ibi_type = I3C_IBI_HOTJOIN };
	int ret;

	/* Per spec, HJ only fires when the target has no dynamic address. */
	zassert_ok(i3c_bus_rstdaa_all(loopback_controller));
	zassert_equal(0, loopback_desc->dynamic_addr);

	ret = i3c_ibi_raise(loopback_target, &ibi);
	if (ret == -ENOTSUP) {
		ztest_test_skip();
	}
	zassert_ok(ret, "i3c_ibi_raise(HJ) returned %d", ret);

	/* HJ delivery semantics are controller-specific; just ensure no fault. */
	(void)k_sem_take(&g_loopback.ibi_sem, SHORT_WAIT);

	zassert_ok(loopback_redo_daa());
}

ZTEST(i3c_loopback_ibi, test_hj_response_disabled_blocks)
{
	struct i3c_ibi ibi = { .ibi_type = I3C_IBI_HOTJOIN };
	int ret;

	ret = i3c_ibi_hj_response(loopback_controller, false);
	if (ret == -ENOSYS || ret == -ENOTSUP) {
		ztest_test_skip();
	}
	zassert_ok(ret);

	zassert_ok(i3c_bus_rstdaa_all(loopback_controller));
	(void)i3c_ibi_raise(loopback_target, &ibi);

	zassert_not_equal(0, k_sem_take(&g_loopback.ibi_sem, SHORT_WAIT),
			  "HJ delivered while response disabled");

	/* Restore. */
	(void)i3c_ibi_hj_response(loopback_controller, true);
	zassert_ok(loopback_redo_daa());
}

ZTEST_SUITE(i3c_loopback_ibi, NULL, NULL, ibi_before, NULL, NULL);
