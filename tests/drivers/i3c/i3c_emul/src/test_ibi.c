/*
 * Copyright (c) 2026 Ryan McClelland
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * M3 IBI tests for the I3C emulator: enable/disable propagation,
 * synchronous and workqueue-based IBI delivery, and HJ/CRR ack gating.
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/ibi.h>
#include <zephyr/drivers/i3c_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

#include "test_target_emul.h"

#define I3C_BUS DT_NODELABEL(i3c0)
#define TARGET_A DT_NODELABEL(test_target_a)

#define TARGET_A_PID  ((uint64_t)0x1234 << 32 | 0x12345678)

static const struct device *bus = DEVICE_DT_GET(I3C_BUS);
static const struct emul *target_a = EMUL_DT_GET(TARGET_A);

static struct {
	atomic_t calls;
	uint8_t last_payload[8];
	uint8_t last_payload_len;
	struct i3c_device_desc *last_target;
} g_ibi;

static int test_ibi_cb(struct i3c_device_desc *target, struct i3c_ibi_payload *payload)
{
	atomic_inc(&g_ibi.calls);
	g_ibi.last_target = target;
	if (payload != NULL) {
		g_ibi.last_payload_len = payload->payload_len;
		if (payload->payload_len <= sizeof(g_ibi.last_payload)) {
			memcpy(g_ibi.last_payload, payload->payload, payload->payload_len);
		}
	} else {
		g_ibi.last_payload_len = 0;
	}
	return 0;
}

static struct i3c_device_desc *find_desc(uint64_t pid)
{
	struct i3c_device_id id = I3C_DEVICE_ID(pid);

	return i3c_device_find(bus, &id);
}

static void *i3c_emul_ibi_setup(void)
{
	struct i3c_device_desc *desc = find_desc(TARGET_A_PID);

	zassert_not_null(desc, "target A desc");
	if (desc->dynamic_addr == 0U) {
		int rc = i3c_bus_setdasa(desc, desc->static_addr);

		zassert_ok(rc, "SETDASA failed: %d", rc);
	}
	desc->ibi_cb = test_ibi_cb;
	return NULL;
}

static void i3c_emul_ibi_before(void *fixture)
{
	ARG_UNUSED(fixture);
	atomic_set(&g_ibi.calls, 0);
	g_ibi.last_payload_len = 0;
	g_ibi.last_target = NULL;
	memset(g_ibi.last_payload, 0, sizeof(g_ibi.last_payload));
	(void)i3c_ibi_hj_response(bus, false);
}

ZTEST(i3c_emul_ibi, test_ibi_disabled_drops)
{
	uint8_t payload[] = {0xAA, 0xBB};
	int rc = test_target_trigger_ibi(target_a, payload, sizeof(payload));

	zassert_equal(rc, -ENOTCONN, "expected -ENOTCONN with IBI disabled, got %d", rc);
	zassert_equal(atomic_get(&g_ibi.calls), 0, "callback should not have fired");
}

ZTEST(i3c_emul_ibi, test_ibi_enabled_delivers_payload)
{
	struct i3c_device_desc *desc = find_desc(TARGET_A_PID);
	uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
	int rc;

	rc = i3c_ibi_enable(desc);
	zassert_ok(rc, "i3c_ibi_enable: %d", rc);
	zassert_true(test_target_ibi_was_enabled(target_a),
		     "peripheral should have observed enable");

	rc = test_target_trigger_ibi(target_a, payload, sizeof(payload));
	zassert_ok(rc, "trigger_ibi: %d", rc);

#ifdef CONFIG_I3C_IBI_WORKQUEUE
	k_msleep(50);
#endif

	zassert_equal(atomic_get(&g_ibi.calls), 1, "callback fired once");
	zassert_equal(g_ibi.last_target, desc, "callback target matches");
	zassert_equal(g_ibi.last_payload_len, sizeof(payload), "payload length");
	zassert_mem_equal(g_ibi.last_payload, payload, sizeof(payload), "payload bytes");

	(void)i3c_ibi_disable(desc);
}

ZTEST(i3c_emul_ibi, test_hj_nack)
{
	int rc = i3c_ibi_hj_response(bus, false);

	zassert_ok(rc, "hj_response false: %d", rc);

	rc = test_target_trigger_hj(target_a);
	zassert_equal(rc, -ENOTCONN, "expected -ENOTCONN when HJ NACKed, got %d", rc);
}

ZTEST(i3c_emul_ibi, test_hj_ack)
{
	int rc = i3c_ibi_hj_response(bus, true);

	zassert_ok(rc, "hj_response true: %d", rc);

	rc = test_target_trigger_hj(target_a);
	zassert_ok(rc, "trigger_hj after ACK: %d", rc);

	(void)i3c_ibi_hj_response(bus, false);
}

ZTEST_SUITE(i3c_emul_ibi, NULL, i3c_emul_ibi_setup, i3c_emul_ibi_before, NULL, NULL);
