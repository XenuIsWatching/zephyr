/*
 * Copyright (c) 2026 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef I3C_LOOPBACK_COMMON_H_
#define I3C_LOOPBACK_COMMON_H_

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/target_device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <stdint.h>

/*
 * Devicetree wiring contract (see README.rst):
 *   - i3c-loopback-controller : controller-mode bus
 *   - i3c-loopback-target     : target-mode bus
 *   - on the controller bus, a child of compatible
 *     "zephyr,i3c-loopback-target" describes the loopback target's
 *     static address and PID. All other identity (BCR, DCR, MRL,
 *     MWL, supported HDR modes) is queried at runtime from the
 *     target-mode controller via i3c_config_get_target() so the
 *     test never duplicates the truth.
 */
#define LOOPBACK_CONTROLLER_NODE DT_ALIAS(i3c_loopback_controller)
#define LOOPBACK_TARGET_NODE	 DT_ALIAS(i3c_loopback_target)
#define LOOPBACK_TARGET_DESC_NODE                                                                  \
	DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_i3c_loopback_target)

BUILD_ASSERT(DT_NODE_HAS_STATUS(LOOPBACK_CONTROLLER_NODE, okay),
	     "missing i3c-loopback-controller alias or node not okay");
BUILD_ASSERT(DT_NODE_HAS_STATUS(LOOPBACK_TARGET_NODE, okay),
	     "missing i3c-loopback-target alias or node not okay");
BUILD_ASSERT(DT_NODE_EXISTS(LOOPBACK_TARGET_DESC_NODE),
	     "missing zephyr,i3c-loopback-target child node");

/* Static 64-bit PID derived from the target child node's reg cells. */
#define LOOPBACK_TARGET_PID                                                                        \
	((((uint64_t)DT_PROP_BY_IDX(LOOPBACK_TARGET_DESC_NODE, reg, 1)) << 32) |                   \
	 ((uint64_t)DT_PROP_BY_IDX(LOOPBACK_TARGET_DESC_NODE, reg, 2)))

/*
 * Per-test state shared between target-side callbacks and the test thread.
 * A single global so registration is one-time; each test's `before` hook
 * zeroes the fields it cares about.
 */
#define LOOPBACK_RX_BUF_SIZE 64
#define LOOPBACK_TX_BUF_SIZE 64

struct loopback_state {
	struct k_sem stop_sem;	   /* released from target's stop_cb */
	struct k_sem ibi_sem;	   /* released from controller's ibi_cb */
	uint8_t rx_buf[LOOPBACK_RX_BUF_SIZE];
	volatile size_t rx_cnt;
	uint8_t tx_buf[LOOPBACK_TX_BUF_SIZE];
	volatile size_t tx_cursor;
	size_t tx_len;
	uint8_t ibi_payload[CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE];
	uint8_t ibi_payload_len;
	volatile uint32_t stop_count;
	volatile uint32_t write_received_count;
	volatile uint32_t read_processed_count;
};

extern struct loopback_state g_loopback;

/*
 * Pre-built target-side callbacks used by suite setup. Each test's
 * `before` hook just zeroes g_loopback fields it cares about; the
 * callbacks themselves stay registered for the lifetime of the test
 * binary.
 */
extern const struct i3c_target_callbacks loopback_target_cbs;

/* Pointers populated by suite setup. */
extern const struct device *loopback_controller;
extern const struct device *loopback_target;
extern struct i3c_device_desc *loopback_desc;

/*
 * Helpers
 */

/*
 * Restore a known-DA state by RSTDAA-then-ENTDAA, leaving
 * loopback_desc->dynamic_addr populated. Tests that change the DA
 * (e.g. via i3c_bus_setnewda) call this in their `before` hook so
 * they don't poison downstream tests.
 */
int loopback_redo_daa(void);

/* Re-prime target TX FIFO with the given payload for the next read. */
int loopback_prime_target_tx(const uint8_t *data, size_t len, uint8_t hdr_mode);

/* Zero g_loopback counters/buffers without disturbing the semaphores. */
void loopback_reset_state(void);

#endif /* I3C_LOOPBACK_COMMON_H_ */
