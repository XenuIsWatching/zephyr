/*
 * Copyright (c) 2026 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HDR-DDR private xfer coverage. Suite-level skip if either side
 * doesn't advertise HDR-DDR in its supported_hdr bitmask via
 * i3c_config_get_*().
 */

#include "common.h"

#include <string.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/ztest.h>

#define WAIT K_MSEC(500)

static void hdr_ddr_before(void *fixture)
{
	struct i3c_config_controller ccfg;
	struct i3c_config_target tcfg;

	ARG_UNUSED(fixture);

	zassert_ok(i3c_config_get_controller(loopback_controller, &ccfg));
	if ((ccfg.supported_hdr & I3C_MSG_HDR_DDR) == 0) {
		ztest_test_skip();
	}

	zassert_ok(i3c_config_get_target(loopback_target, &tcfg));
	if ((tcfg.supported_hdr & I3C_MSG_HDR_DDR) == 0) {
		ztest_test_skip();
	}

	zassert_ok(loopback_redo_daa());
	loopback_reset_state();
}

ZTEST(i3c_loopback_hdr_ddr, test_hdr_ddr_write_single_word)
{
	uint8_t tx[2] = {0x55, 0xAA};
	struct i3c_msg msg = {
		.buf = tx,
		.len = sizeof(tx),
		.flags = I3C_MSG_WRITE | I3C_MSG_HDR,
		.hdr_mode = I3C_MSG_HDR_DDR,
	};

	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_mem_equal(tx, g_loopback.rx_buf, sizeof(tx));
}

ZTEST(i3c_loopback_hdr_ddr, test_hdr_ddr_write_multi_word)
{
	uint8_t tx[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
	struct i3c_msg msg = {
		.buf = tx,
		.len = sizeof(tx),
		.flags = I3C_MSG_WRITE | I3C_MSG_HDR,
		.hdr_mode = I3C_MSG_HDR_DDR,
	};

	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_mem_equal(tx, g_loopback.rx_buf, sizeof(tx));
}

ZTEST(i3c_loopback_hdr_ddr, test_hdr_ddr_read_single_word)
{
	uint8_t prime[2] = {0xDE, 0xAD};
	uint8_t rx[2] = {0};
	struct i3c_msg msg = {
		.buf = rx,
		.len = sizeof(rx),
		.flags = I3C_MSG_READ | I3C_MSG_HDR | I3C_MSG_STOP,
		.hdr_mode = I3C_MSG_HDR_DDR,
	};

	zassert_ok(loopback_prime_target_tx(prime, sizeof(prime), I3C_MSG_HDR_DDR));
	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_mem_equal(prime, rx, sizeof(prime));
}

ZTEST(i3c_loopback_hdr_ddr, test_hdr_ddr_read_multi_word)
{
	uint8_t prime[8] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7};
	uint8_t rx[8] = {0};
	struct i3c_msg msg = {
		.buf = rx,
		.len = sizeof(rx),
		.flags = I3C_MSG_READ | I3C_MSG_HDR | I3C_MSG_STOP,
		.hdr_mode = I3C_MSG_HDR_DDR,
	};

	zassert_ok(loopback_prime_target_tx(prime, sizeof(prime), I3C_MSG_HDR_DDR));
	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_mem_equal(prime, rx, sizeof(prime));
}

ZTEST_SUITE(i3c_loopback_hdr_ddr, NULL, NULL, hdr_ddr_before, NULL, NULL);
