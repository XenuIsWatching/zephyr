/*
 * Copyright (c) 2026 Ryan McClelland
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * M6 CCC coverage tests for the I3C emulator: SETMRL/SETMWL/GETMRL/GETMWL
 * round-trip, GETSTATUS, SETNEWDA, explicit RSTDAA, and DEFTGTS dispatch.
 *
 * The test target peripheral exposes the underlying state via its
 * backend API (test_target_get_mrl, test_target_get_deftgts, ...) so the
 * tests can verify both directions of each CCC.
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/drivers/i3c_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include "test_target_emul.h"

#define I3C_BUS DT_NODELABEL(i3c0)
#define TARGET_A DT_NODELABEL(test_target_a)
#define TARGET_B DT_NODELABEL(test_target_b)

#define TARGET_A_PID		TEST_TARGET_A_PID
#define TARGET_B_PID		TEST_TARGET_B_PID

static const struct device *bus = DEVICE_DT_GET(I3C_BUS);
static const struct emul *target_a = EMUL_DT_GET(TARGET_A);
static const struct emul *target_b = EMUL_DT_GET(TARGET_B);

static struct i3c_device_desc *find_desc(uint64_t pid)
{
	struct i3c_device_id id = { .pid = pid };

	return i3c_device_find(bus, &id);
}

static void *ccc_setup(void)
{
	int rc = test_target_bus_known_state(bus, TEST_TARGET_A_PID, TEST_TARGET_A_STATIC,
					     TEST_TARGET_B_PID, TEST_TARGET_B_INIT_DA);

	zassert_ok(rc, "test_target_bus_known_state: %d", rc);
	return NULL;
}

static void ccc_before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Re-establish the canonical address state so per-test mutations
	 * (RSTDAA, SETNEWDA, ...) don't leak between tests.
	 */
	(void)test_target_bus_known_state(bus, TEST_TARGET_A_PID, TEST_TARGET_A_STATIC,
					  TEST_TARGET_B_PID, TEST_TARGET_B_INIT_DA);
}

ZTEST(i3c_emul_ccc, test_setmwl_getmwl_round_trip)
{
	struct i3c_device_desc *desc = find_desc(TARGET_A_PID);
	struct i3c_ccc_mwl set = { .len = 0x0123 };
	struct i3c_ccc_mwl got = { 0 };
	int rc;

	rc = i3c_ccc_do_setmwl(desc, &set);
	zassert_ok(rc, "SETMWL: %d", rc);
	zassert_equal(test_target_get_mwl(target_a), 0x0123, "peripheral stored MWL");

	rc = i3c_ccc_do_getmwl(desc, &got);
	zassert_ok(rc, "GETMWL: %d", rc);
	zassert_equal(got.len, 0x0123, "GETMWL round-trip");
}

ZTEST(i3c_emul_ccc, test_setmrl_getmrl_round_trip)
{
	struct i3c_device_desc *desc = find_desc(TARGET_A_PID);
	struct i3c_ccc_mrl set = { .len = 0x4567, .ibi_len = 0x08 };
	struct i3c_ccc_mrl got = { 0 };
	int rc;

	/*
	 * GETMRL only requests the ibi_len byte when the peripheral's BCR
	 * advertises IBI payload data. Our DT BCR for target_a is 0x00, so
	 * the controller helper will issue a 2-byte GETMRL and we only
	 * round-trip the length here.
	 */
	rc = i3c_ccc_do_setmrl(desc, &set);
	zassert_ok(rc, "SETMRL: %d", rc);
	zassert_equal(test_target_get_mrl(target_a), 0x4567, "peripheral stored MRL");

	rc = i3c_ccc_do_getmrl(desc, &got);
	zassert_ok(rc, "GETMRL: %d", rc);
	zassert_equal(got.len, 0x4567, "GETMRL round-trip");
}

ZTEST(i3c_emul_ccc, test_getstatus_returns_poked_value)
{
	struct i3c_device_desc *desc = find_desc(TARGET_A_PID);
	union i3c_ccc_getstatus got = { 0 };
	int rc;

	test_target_set_status_fmt1(target_a, 0xCAFE);

	rc = i3c_ccc_do_getstatus_fmt1(desc, &got);
	zassert_ok(rc, "GETSTATUS: %d", rc);
	zassert_equal(got.fmt1.status, 0xCAFE, "status round-trip");

	test_target_set_status_fmt1(target_a, 0);
}

ZTEST(i3c_emul_ccc, test_setnewda_changes_dynamic_addr)
{
	struct i3c_device_desc *desc = find_desc(TARGET_A_PID);
	const uint8_t new_addr = 0x33;
	int rc;

	rc = i3c_bus_setnewda(desc, new_addr);
	zassert_ok(rc, "SETNEWDA: %d", rc);
	zassert_equal(desc->dynamic_addr, new_addr, "desc dyn addr updated");
	zassert_equal(test_target_get_dynamic_addr(target_a), new_addr,
		      "peripheral mirror updated");

	/* No restore here — the suite's per-test before-hook re-establishes
	 * the canonical address state for whichever test runs next.
	 */
}

ZTEST(i3c_emul_ccc, test_rstdaa_clears_addresses)
{
	struct i3c_device_desc *desc_a = find_desc(TARGET_A_PID);
	struct i3c_device_desc *desc_b = find_desc(TARGET_B_PID);
	int rc;

	zassert_not_equal(desc_a->dynamic_addr, 0U, "precondition: A has dyn addr");
	zassert_not_equal(desc_b->dynamic_addr, 0U, "precondition: B has dyn addr");

	rc = i3c_bus_rstdaa_all(bus);
	zassert_ok(rc, "RSTDAA: %d", rc);

	zassert_equal(desc_a->dynamic_addr, 0U, "A dyn addr cleared");
	zassert_equal(desc_b->dynamic_addr, 0U, "B dyn addr cleared");
	zassert_equal(test_target_get_dynamic_addr(target_a), 0U, "A peripheral mirror cleared");
	zassert_equal(test_target_get_dynamic_addr(target_b), 0U, "B peripheral mirror cleared");

	/* No restore here — the suite's per-test before-hook re-establishes
	 * the canonical address state for whichever test runs next.
	 */
}

ZTEST(i3c_emul_ccc, test_deftgts_broadcast_reaches_peripherals)
{
	int rc;
	uint8_t buf[64];
	size_t n;

	test_target_clear_deftgts(target_a);
	test_target_clear_deftgts(target_b);

	rc = i3c_bus_deftgts(bus);
	zassert_ok(rc, "i3c_bus_deftgts: %d", rc);

	zassert_true(test_target_deftgts_was_seen(target_a), "A saw DEFTGTS");
	zassert_true(test_target_deftgts_was_seen(target_b), "B saw DEFTGTS");

	n = test_target_get_deftgts(target_a, buf, sizeof(buf));
	zassert_true(n > 0U, "DEFTGTS payload non-empty (got %zu)", n);

	/*
	 * Wire layout (drivers/i3c/i3c_common.c::i3c_bus_deftgts builds it,
	 * i3c_ccc_do_deftgts_all sends it as ccc.data):
	 *   buf[0]                  = count of targets
	 *   buf[1..1+sizeof(active_controller)] = active controller descriptor
	 *   then count * sizeof(target descriptor)
	 *
	 * count must be the number of attached I3C + I2C devices on this
	 * bus, which for the test overlay is 2 (target_a + target_b).
	 */
	zassert_equal(buf[0], 2U, "DEFTGTS count = num attached, got %u", buf[0]);
}

ZTEST_SUITE(i3c_emul_ccc, NULL, ccc_setup, ccc_before, NULL, NULL);
