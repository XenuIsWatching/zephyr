/*
 * Copyright (c) 2026 Ryan McClelland
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * M5 target-mode tests for the I3C emulator.
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/target_device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

#include "test_target_emul.h"

#define I3C_BUS DT_NODELABEL(i3c0)
#define TARGET_MODE_ADDR 0x6A

#define TARGET_A_PID  ((uint64_t)0x1234 << 32 | 0x12345678)

static const struct device *bus = DEVICE_DT_GET(I3C_BUS);

static struct i3c_device_desc *find_desc(uint64_t pid)
{
	struct i3c_device_id id = I3C_DEVICE_ID(pid);

	return i3c_device_find(bus, &id);
}

static struct {
	atomic_t write_requested;
	atomic_t write_received_total;
	atomic_t read_requested;
	atomic_t read_processed;
	atomic_t stop;
	atomic_t handoff;
	uint8_t last_write[16];
	uint8_t last_write_idx;
	uint8_t read_seq;
} g;

static int cb_write_requested(struct i3c_target_config *cfg)
{
	ARG_UNUSED(cfg);
	atomic_inc(&g.write_requested);
	g.last_write_idx = 0;
	return 0;
}

static int cb_write_received(struct i3c_target_config *cfg, uint8_t val)
{
	ARG_UNUSED(cfg);
	atomic_inc(&g.write_received_total);
	if (g.last_write_idx < sizeof(g.last_write)) {
		g.last_write[g.last_write_idx++] = val;
	}
	return 0;
}

static int cb_read_requested(struct i3c_target_config *cfg, uint8_t *val)
{
	ARG_UNUSED(cfg);
	atomic_inc(&g.read_requested);
	*val = g.read_seq++;
	return 0;
}

static int cb_read_processed(struct i3c_target_config *cfg, uint8_t *val)
{
	ARG_UNUSED(cfg);
	atomic_inc(&g.read_processed);
	*val = g.read_seq++;
	return 0;
}

static int cb_stop(struct i3c_target_config *cfg)
{
	ARG_UNUSED(cfg);
	atomic_inc(&g.stop);
	return 0;
}

static int cb_handoff(struct i3c_target_config *cfg)
{
	ARG_UNUSED(cfg);
	atomic_inc(&g.handoff);
	return 0;
}

static const struct i3c_target_callbacks tcb = {
	.write_requested_cb = cb_write_requested,
	.write_received_cb = cb_write_received,
	.read_requested_cb = cb_read_requested,
	.read_processed_cb = cb_read_processed,
	.stop_cb = cb_stop,
	.controller_handoff_cb = cb_handoff,
};

static struct i3c_target_config tcfg = {
	.address = TARGET_MODE_ADDR,
	.callbacks = &tcb,
};

static struct i3c_device_desc shadow_desc;

static void *target_setup(void)
{
	shadow_desc.bus = bus;
	shadow_desc.dynamic_addr = TARGET_MODE_ADDR;
	return NULL;
}

static void target_before(void *fixture)
{
	ARG_UNUSED(fixture);
	atomic_set(&g.write_requested, 0);
	atomic_set(&g.write_received_total, 0);
	atomic_set(&g.read_requested, 0);
	atomic_set(&g.read_processed, 0);
	atomic_set(&g.stop, 0);
	atomic_set(&g.handoff, 0);
	g.last_write_idx = 0;
	g.read_seq = 0xA0;
	memset(g.last_write, 0, sizeof(g.last_write));
}

ZTEST(i3c_emul_target, test_register_then_xfer_invokes_callbacks)
{
	uint8_t out[3] = {0xAA, 0xBB, 0xCC};
	uint8_t in[2] = {0};
	struct i3c_msg msgs[2] = {
		{ .buf = out, .len = sizeof(out), .flags = I3C_MSG_WRITE | I3C_MSG_STOP },
		{ .buf = in, .len = sizeof(in), .flags = I3C_MSG_READ | I3C_MSG_STOP },
	};
	int rc;

	rc = i3c_target_register(bus, &tcfg);
	zassert_ok(rc, "target_register: %d", rc);

	rc = i3c_transfer(&shadow_desc, &msgs[0], 1);
	zassert_ok(rc, "write transfer: %d", rc);
	zassert_equal(atomic_get(&g.write_requested), 1, "write_requested fired");
	zassert_equal(atomic_get(&g.write_received_total), 3, "write_received per byte");
	zassert_mem_equal(g.last_write, out, sizeof(out), "bytes match");
	zassert_equal(atomic_get(&g.stop), 1, "stop fired");

	rc = i3c_transfer(&shadow_desc, &msgs[1], 1);
	zassert_ok(rc, "read transfer: %d", rc);
	zassert_equal(atomic_get(&g.read_requested), 1, "read_requested fired");
	zassert_equal(atomic_get(&g.read_processed), 1, "read_processed fired");
	zassert_equal(in[0], 0xA0, "first byte from cb");
	zassert_equal(in[1], 0xA1, "second byte from cb");

	rc = i3c_target_unregister(bus, &tcfg);
	zassert_ok(rc, "target_unregister: %d", rc);
}

ZTEST(i3c_emul_target, test_target_tx_write_fifo)
{
	uint8_t pushed[4] = {0x10, 0x20, 0x30, 0x40};
	uint8_t in[4] = {0};
	struct i3c_msg msg = {
		.buf = in, .len = sizeof(in), .flags = I3C_MSG_READ | I3C_MSG_STOP,
	};
	int rc;

	rc = i3c_target_register(bus, &tcfg);
	zassert_ok(rc, "target_register: %d", rc);

	rc = i3c_target_tx_write(bus, pushed, sizeof(pushed), 0);
	zassert_equal(rc, sizeof(pushed), "tx_write returned %d", rc);

	rc = i3c_transfer(&shadow_desc, &msg, 1);
	zassert_ok(rc, "read transfer: %d", rc);
	zassert_mem_equal(in, pushed, sizeof(pushed), "FIFO bytes consumed");
	zassert_equal(atomic_get(&g.read_requested), 0,
		      "read_requested should not fire when FIFO satisfies the whole msg");

	(void)i3c_target_unregister(bus, &tcfg);
}

ZTEST(i3c_emul_target, test_controller_handoff_records)
{
	int rc;

	rc = i3c_target_register(bus, &tcfg);
	zassert_ok(rc, "target_register: %d", rc);

	/*
	 * Accepting handoff invokes the controller_handoff_cb. Per the
	 * I3C spec, is_secondary is the boot-time role and does not flip
	 * here, so we don't assert on it.
	 */
	rc = i3c_target_controller_handoff(bus, true);
	zassert_ok(rc, "handoff(true): %d", rc);
	zassert_equal(atomic_get(&g.handoff), 1, "handoff cb fires on accept");

	/* NACK should not invoke the callback. */
	rc = i3c_target_controller_handoff(bus, false);
	zassert_ok(rc, "handoff(false): %d", rc);
	zassert_equal(atomic_get(&g.handoff), 1, "handoff cb does NOT fire on nack");

	(void)i3c_target_unregister(bus, &tcfg);
}

ZTEST(i3c_emul_target, test_controller_initiates_handoff_via_getacccr)
{
	struct i3c_device_desc *desc;
	int rc;

	/*
	 * Drive the *initiation* side of controller handoff: the bus
	 * emulator is the active controller; it issues the handoff CCC
	 * sequence (DISEC broadcast, then GETACCCR direct) targeting an
	 * attached peripheral. PLAN.md explicitly leaves the deeper
	 * "newly-active controller now drives the bus" handoff state
	 * machine out of scope (M5 non-goals); this covers the half
	 * the emulator is responsible for: dispatching the wire CCCs
	 * and round-tripping GETACCCR's parity-encoded reply.
	 *
	 * A 0 return from i3c_device_controller_handoff(requested=true)
	 * implies i3c_bus_getacccr's parity check passed against
	 * desc->dynamic_addr, which is only possible if (a) the bus
	 * emulator dispatched GETACCCR to the right peripheral, (b)
	 * the peripheral's do_ccc handled it, and (c) the reply byte
	 * was spec-correct. So the assertion below is sufficient on
	 * its own.
	 */
	desc = find_desc(TARGET_A_PID);
	zassert_not_null(desc, "target A desc");
	if (desc->dynamic_addr == 0U) {
		rc = i3c_bus_setdasa(desc, desc->static_addr);
		zassert_ok(rc, "SETDASA: %d", rc);
	}

	rc = i3c_device_controller_handoff(desc, true);
	zassert_ok(rc, "i3c_device_controller_handoff(requested=true): %d", rc);
}

ZTEST_SUITE(i3c_emul_target, NULL, target_setup, target_before, NULL, NULL);
