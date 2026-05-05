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

#define TARGET_A_PID		TEST_TARGET_A_PID

static const struct device *bus = DEVICE_DT_GET(I3C_BUS);

static struct i3c_device_desc *find_desc(uint64_t pid)
{
	struct i3c_device_id id = { .pid = pid };

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
	int rc = test_target_bus_known_state(bus, TEST_TARGET_A_PID, TEST_TARGET_A_STATIC,
					     TEST_TARGET_B_PID, TEST_TARGET_B_INIT_DA);

	zassert_ok(rc, "test_target_bus_known_state: %d", rc);

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

	/* Re-establish the canonical address state — handoff tests
	 * (RSTDAA, sec_handoffed re-attach) can leave the bus in a
	 * different shape.
	 */
	(void)test_target_bus_known_state(bus, TEST_TARGET_A_PID, TEST_TARGET_A_STATIC,
					  TEST_TARGET_B_PID, TEST_TARGET_B_INIT_DA);
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

ZTEST(i3c_emul_target, test_controller_handoff_accept_records)
{
	int rc;

	/*
	 * i3c_target_controller_handoff(true) only stores the application's
	 * willingness to accept a future handoff. It does NOT itself fire
	 * controller_handoff_cb (real drivers fire that callback only when
	 * the wire-level handoff actually completes — see i3c_dw.c and
	 * i3c_cdns.c). Without a controller-side GETACCCR, the cb stays
	 * silent.
	 */
	rc = i3c_target_register(bus, &tcfg);
	zassert_ok(rc, "target_register: %d", rc);

	rc = i3c_target_controller_handoff(bus, true);
	zassert_ok(rc, "handoff(true): %d", rc);
	zassert_equal(atomic_get(&g.handoff), 0,
		      "handoff cb must not fire from the API call alone");

	rc = i3c_target_controller_handoff(bus, false);
	zassert_ok(rc, "handoff(false): %d", rc);
	zassert_equal(atomic_get(&g.handoff), 0, "handoff cb still silent on nack");

	(void)i3c_target_unregister(bus, &tcfg);
}

ZTEST(i3c_emul_target, test_handoff_completes_on_getacccr_to_registered_target)
{
	struct i3c_target_config tcfg_at_a = {
		.address = 0,        /* filled in below from desc_a->dynamic_addr */
		.callbacks = &tcb,
	};
	struct i3c_config_target out_cfg = { 0 };
	struct i3c_device_desc *desc_a;
	int rc;

	/*
	 * Spec model:
	 * - Application brings the device's dynamic address up the proper
	 *   way (SETDASA / DAA), then registers as a target at that same
	 *   address. The bus emulator now routes CCCs at that address to
	 *   the application's target_cfg instead of the peripheral emul.
	 * - Application opts in via i3c_target_controller_handoff(true).
	 * - Active controller initiates handoff via
	 *   i3c_device_controller_handoff(desc, true), which issues
	 *   GETACCCR at the address.
	 * - The bus emulator detects "GETACCCR addressed to a registered
	 *   target_cfg with handoff_accept set", replies parity-correctly,
	 *   fires controller_handoff_cb, and flips
	 *   target_config.enabled = false (we are the active controller
	 *   now, not a target).
	 */
	desc_a = find_desc(TARGET_A_PID);
	zassert_not_null(desc_a, "target A desc");
	zassert_not_equal(desc_a->dynamic_addr, 0U,
			  "before-hook should have established target A's DA");

	tcfg_at_a.address = desc_a->dynamic_addr;
	rc = i3c_target_register(bus, &tcfg_at_a);
	zassert_ok(rc, "target_register at 0x%02x: %d", tcfg_at_a.address, rc);

	rc = i3c_config_get(bus, I3C_CONFIG_TARGET, &out_cfg);
	zassert_ok(rc, "config_get: %d", rc);
	zassert_true(out_cfg.enabled, "registering as target should set enabled");

	rc = i3c_target_controller_handoff(bus, true);
	zassert_ok(rc, "handoff(true): %d", rc);

	rc = i3c_device_controller_handoff(desc_a, true);
	zassert_ok(rc, "i3c_device_controller_handoff: %d", rc);

	zassert_equal(atomic_get(&g.handoff), 1,
		      "controller_handoff_cb fires on completed handoff");

	rc = i3c_config_get(bus, I3C_CONFIG_TARGET, &out_cfg);
	zassert_ok(rc, "config_get post-handoff: %d", rc);
	zassert_false(out_cfg.enabled,
		      "after handoff completes, the new active controller is no "
		      "longer behaving as a target");

	(void)i3c_target_unregister(bus, &tcfg_at_a);
}

ZTEST(i3c_emul_target, test_handoff_nacked_by_application_fails)
{
	struct i3c_target_config tcfg_at_a = {
		.address = 0,
		.callbacks = &tcb,
	};
	struct i3c_device_desc *desc_a;
	int rc;

	/* Same setup as the accept test, but the application explicitly
	 * NACKs the handoff offer. The controller-side initiation must
	 * fail and the callback must stay silent.
	 */
	desc_a = find_desc(TARGET_A_PID);
	zassert_not_null(desc_a, "target A desc");
	zassert_not_equal(desc_a->dynamic_addr, 0U,
			  "before-hook should have established target A's DA");

	tcfg_at_a.address = desc_a->dynamic_addr;
	rc = i3c_target_register(bus, &tcfg_at_a);
	zassert_ok(rc, "target_register: %d", rc);

	rc = i3c_target_controller_handoff(bus, false);
	zassert_ok(rc, "handoff(false): %d", rc);

	rc = i3c_device_controller_handoff(desc_a, true);
	zassert_not_equal(rc, 0,
			  "controller-side handoff must fail when the target NACKs");
	zassert_equal(atomic_get(&g.handoff), 0, "callback must stay silent on NACK");

	(void)i3c_target_unregister(bus, &tcfg_at_a);
}

ZTEST(i3c_emul_target, test_controller_handoff_to_peripheral_via_getacccr)
{
	struct i3c_device_desc *desc;
	int rc;

	/*
	 * No target_cfg registered at desc->dynamic_addr — the GETACCCR
	 * routes to the registered peripheral emul, which replies
	 * parity-correctly. This is the "controller initiates handoff
	 * to a secondary controller it knows about, but no application
	 * on this device is acting as that target" scenario; the bus
	 * emulator's CCC dispatch + the peripheral's GETACCCR responder
	 * are what get exercised.
	 */
	desc = find_desc(TARGET_A_PID);
	zassert_not_null(desc, "target A desc");
	zassert_not_equal(desc->dynamic_addr, 0U,
			  "before-hook should have established target A's DA");

	rc = i3c_device_controller_handoff(desc, true);
	zassert_ok(rc, "i3c_device_controller_handoff: %d", rc);
}

ZTEST(i3c_emul_target, test_handoff_completes_and_sec_handoffed_repopulates_bus)
{
#if !defined(CONFIG_I3C_USE_IBI) || !defined(CONFIG_I3C_IBI_WORKQUEUE)
	ztest_test_skip();
#else
	struct i3c_target_config tcfg_at_a = {
		.address = 0,
		.callbacks = &tcb,
	};
	struct i3c_device_desc *desc_a;
	struct i3c_device_desc *desc;
	int n_attached;
	int rc;

	/*
	 * End-to-end secondary-controller handoff:
	 *  1. As the active controller, broadcast DEFTGTS — peripherals
	 *     receive it AND the bus emulator stores a (de-shifted) deep
	 *     copy in data->common.deftgts because a target_cfg is
	 *     registered.
	 *  2. Active controller initiates handoff via GETACCCR direct CCC
	 *     at the registered target_cfg's address.
	 *  3. Bus emulator's GETACCCR-to-target_cfg interception replies
	 *     parity-correctly, fires controller_handoff_cb, flips
	 *     target_config.enabled = false, and enqueues
	 *     i3c_sec_handoffed on the IBI workqueue.
	 *  4. i3c_sec_handoffed runs: i3c_sec_bus_reset detaches all descs,
	 *     then for each entry in data->common.deftgts it calls
	 *     i3c_sec_get_basic_info, which attaches a temp_desc and
	 *     issues GETPID. The bus emulator's wire-level dynamic-address
	 *     fallback finds the peripheral, GETPID returns the right PID,
	 *     the framework looks up the static desc by PID and re-attaches
	 *     it, and this driver's PID-keyed attach hook re-links
	 *     desc->controller_priv.
	 *
	 * After the workqueue drains, every desc in DEFTGTS should be
	 * re-attached and have controller_priv set, proving the device-now-
	 * acting-as-active-controller can immediately drive the bus.
	 */
	desc_a = find_desc(TARGET_A_PID);
	zassert_not_null(desc_a, "target A desc");
	zassert_not_equal(desc_a->dynamic_addr, 0U,
			  "before-hook should have established target A's DA");

	tcfg_at_a.address = desc_a->dynamic_addr;
	rc = i3c_target_register(bus, &tcfg_at_a);
	zassert_ok(rc, "target_register: %d", rc);

	rc = i3c_target_controller_handoff(bus, true);
	zassert_ok(rc, "handoff(true): %d", rc);

	rc = i3c_bus_deftgts(bus);
	zassert_ok(rc, "i3c_bus_deftgts: %d", rc);

	rc = i3c_device_controller_handoff(desc_a, true);
	zassert_ok(rc, "i3c_device_controller_handoff: %d", rc);

	zassert_equal(atomic_get(&g.handoff), 1, "controller_handoff_cb fired");

	/*
	 * Wait for the IBI workqueue to run i3c_sec_handoffed and re-attach
	 * descs from DEFTGTS. Poll the bus list for the first desc whose
	 * controller_priv is non-NULL — that's the observable condition,
	 * no arbitrary sleep needed.
	 */
	bool any_relinked = false;

	zassert_true(WAIT_FOR(({
		any_relinked = false;
		I3C_BUS_FOR_EACH_I3CDEV(bus, desc) {
			if (desc->controller_priv != NULL) {
				any_relinked = true;
				break;
			}
		}
		any_relinked;
	}), USEC_PER_MSEC * 100, k_msleep(1)),
		     "i3c_sec_handoffed did not re-attach within 100ms");

	n_attached = 0;
	I3C_BUS_FOR_EACH_I3CDEV(bus, desc) {
		n_attached++;
		zassert_not_null((void *)desc->controller_priv,
				 "desc 0x%02x relinked after handoff", desc->dynamic_addr);
	}
	zassert_true(n_attached >= 1,
		     "expected at least one desc re-attached from DEFTGTS, got %d", n_attached);

	(void)i3c_target_unregister(bus, &tcfg_at_a);
#endif
}

ZTEST_SUITE(i3c_emul_target, NULL, target_setup, target_before, NULL, NULL);
