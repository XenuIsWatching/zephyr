/*
 * Copyright (c) 2026 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private SDR transfer coverage: 1-byte, multi-byte, multi-msg with
 * STOP and RESTART, NBCH, num_xfer reporting; and the i3c_*write,
 * i3c_*read, i3c_write_read, i3c_burst_*, i3c_reg_* helper inlines.
 */

#include "common.h"

#include <string.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/ztest.h>

#define WAIT K_MSEC(500)

static void xfer_before(void *fixture)
{
	ARG_UNUSED(fixture);
	loopback_reset_state();
}

ZTEST(i3c_loopback_xfer, test_private_write_single_byte)
{
	uint8_t tx[1] = {0x5a};
	struct i3c_msg msg = {
		.buf = tx,
		.len = sizeof(tx),
		.flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};

	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT), "no stop");
	zassert_equal(1, g_loopback.rx_cnt);
	zassert_mem_equal(tx, g_loopback.rx_buf, sizeof(tx));
}

ZTEST(i3c_loopback_xfer, test_private_write_multi_byte)
{
	uint8_t tx[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
	struct i3c_msg msg = {
		.buf = tx,
		.len = sizeof(tx),
		.flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};

	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(sizeof(tx), g_loopback.rx_cnt);
	zassert_mem_equal(tx, g_loopback.rx_buf, sizeof(tx));
}

ZTEST(i3c_loopback_xfer, test_private_write_max_size)
{
	uint8_t tx[32];
	struct i3c_msg msg = {
		.buf = tx,
		.len = sizeof(tx),
		.flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};

	for (size_t i = 0; i < sizeof(tx); ++i) {
		tx[i] = (uint8_t)(0xa0 + i);
	}

	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(sizeof(tx), g_loopback.rx_cnt);
	zassert_mem_equal(tx, g_loopback.rx_buf, sizeof(tx));
}

ZTEST(i3c_loopback_xfer, test_private_read_single_byte)
{
	uint8_t tx[1] = {0x42};
	uint8_t rx[1] = {0};
	struct i3c_msg msg = {
		.buf = rx,
		.len = sizeof(rx),
		.flags = I3C_MSG_READ | I3C_MSG_STOP,
	};

	zassert_ok(loopback_prime_target_tx(tx, sizeof(tx), 0));
	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(tx[0], rx[0]);
}

ZTEST(i3c_loopback_xfer, test_private_read_multi_byte)
{
	uint8_t tx[16];
	uint8_t rx[16] = {0};
	struct i3c_msg msg = {
		.buf = rx,
		.len = sizeof(rx),
		.flags = I3C_MSG_READ | I3C_MSG_STOP,
	};

	for (size_t i = 0; i < sizeof(tx); ++i) {
		tx[i] = (uint8_t)(0xb0 + i);
	}

	zassert_ok(loopback_prime_target_tx(tx, sizeof(tx), 0));
	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_mem_equal(tx, rx, sizeof(tx));
}

ZTEST(i3c_loopback_xfer, test_private_read_num_xfer_reporting)
{
	uint8_t tx[4] = {0xC0, 0xC1, 0xC2, 0xC3};
	uint8_t rx[4] = {0};
	struct i3c_msg msg = {
		.buf = rx,
		.len = sizeof(rx),
		.flags = I3C_MSG_READ | I3C_MSG_STOP,
	};

	zassert_ok(loopback_prime_target_tx(tx, sizeof(tx), 0));
	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(sizeof(tx), msg.num_xfer, "num_xfer=%u expected %u", msg.num_xfer,
		      (unsigned int)sizeof(tx));
}

ZTEST(i3c_loopback_xfer, test_msg_pair_write_then_read_with_restart)
{
	uint8_t reg_addr[1] = {0x10};
	uint8_t prime[2] = {0xDE, 0xAD};
	uint8_t rx[2] = {0};
	struct i3c_msg msgs[2] = {
		{
			.buf = reg_addr,
			.len = sizeof(reg_addr),
			.flags = I3C_MSG_WRITE,
		},
		{
			.buf = rx,
			.len = sizeof(rx),
			.flags = I3C_MSG_RESTART | I3C_MSG_READ | I3C_MSG_STOP,
		},
	};

	zassert_ok(loopback_prime_target_tx(prime, sizeof(prime), 0));
	zassert_ok(i3c_transfer(loopback_desc, msgs, 2));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(reg_addr[0], g_loopback.rx_buf[0]);
	zassert_mem_equal(prime, rx, sizeof(prime));
}

ZTEST(i3c_loopback_xfer, test_msg_pair_write_then_read_with_stop)
{
	uint8_t tx[2] = {0x11, 0x22};
	uint8_t prime[2] = {0xBE, 0xEF};
	uint8_t rx[2] = {0};
	struct i3c_msg msgs_w[1] = {
		{ .buf = tx, .len = sizeof(tx), .flags = I3C_MSG_WRITE | I3C_MSG_STOP },
	};
	struct i3c_msg msgs_r[1] = {
		{ .buf = rx, .len = sizeof(rx), .flags = I3C_MSG_READ | I3C_MSG_STOP },
	};

	zassert_ok(i3c_transfer(loopback_desc, msgs_w, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_mem_equal(tx, g_loopback.rx_buf, sizeof(tx));

	loopback_reset_state();
	zassert_ok(loopback_prime_target_tx(prime, sizeof(prime), 0));
	zassert_ok(i3c_transfer(loopback_desc, msgs_r, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_mem_equal(prime, rx, sizeof(prime));
}

ZTEST(i3c_loopback_xfer, test_msg_three_write_chain_with_restart)
{
	uint8_t a[2] = {0xAA, 0xAB};
	uint8_t b[2] = {0xBB, 0xBC};
	uint8_t c[2] = {0xCC, 0xCD};
	struct i3c_msg msgs[3] = {
		{ .buf = a, .len = sizeof(a), .flags = I3C_MSG_WRITE },
		{ .buf = b, .len = sizeof(b), .flags = I3C_MSG_RESTART | I3C_MSG_WRITE },
		{ .buf = c, .len = sizeof(c),
		  .flags = I3C_MSG_RESTART | I3C_MSG_WRITE | I3C_MSG_STOP },
	};
	uint8_t expected[6];

	memcpy(&expected[0], a, sizeof(a));
	memcpy(&expected[2], b, sizeof(b));
	memcpy(&expected[4], c, sizeof(c));

	zassert_ok(i3c_transfer(loopback_desc, msgs, 3));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(sizeof(expected), g_loopback.rx_cnt);
	zassert_mem_equal(expected, g_loopback.rx_buf, sizeof(expected));
}

ZTEST(i3c_loopback_xfer, test_msg_chain_mixed_lengths)
{
	uint8_t a[1] = {0x77};
	uint8_t b[3] = {0x88, 0x89, 0x8A};
	uint8_t c[5] = {0x91, 0x92, 0x93, 0x94, 0x95};
	struct i3c_msg msgs[3] = {
		{ .buf = a, .len = sizeof(a), .flags = I3C_MSG_WRITE },
		{ .buf = b, .len = sizeof(b), .flags = I3C_MSG_RESTART | I3C_MSG_WRITE },
		{ .buf = c, .len = sizeof(c),
		  .flags = I3C_MSG_RESTART | I3C_MSG_WRITE | I3C_MSG_STOP },
	};
	uint8_t expected[9];

	memcpy(&expected[0], a, sizeof(a));
	memcpy(&expected[1], b, sizeof(b));
	memcpy(&expected[4], c, sizeof(c));

	zassert_ok(i3c_transfer(loopback_desc, msgs, 3));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(sizeof(expected), g_loopback.rx_cnt);
	zassert_mem_equal(expected, g_loopback.rx_buf, sizeof(expected));
}

ZTEST(i3c_loopback_xfer, test_nbch_write_skips_broadcast)
{
	uint8_t tx[2] = {0x33, 0x44};
	struct i3c_msg msg = {
		.buf = tx,
		.len = sizeof(tx),
		.flags = I3C_MSG_WRITE | I3C_MSG_NBCH | I3C_MSG_STOP,
	};

	zassert_ok(i3c_transfer(loopback_desc, &msg, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_mem_equal(tx, g_loopback.rx_buf, sizeof(tx));
}

ZTEST(i3c_loopback_xfer, test_i3c_write_helper)
{
	uint8_t tx[3] = {0x55, 0x66, 0x77};

	zassert_ok(i3c_write(loopback_desc, tx, sizeof(tx)));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_mem_equal(tx, g_loopback.rx_buf, sizeof(tx));
}

ZTEST(i3c_loopback_xfer, test_i3c_read_helper)
{
	uint8_t prime[3] = {0xA1, 0xA2, 0xA3};
	uint8_t rx[3] = {0};

	zassert_ok(loopback_prime_target_tx(prime, sizeof(prime), 0));
	zassert_ok(i3c_read(loopback_desc, rx, sizeof(rx)));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_mem_equal(prime, rx, sizeof(prime));
}

ZTEST(i3c_loopback_xfer, test_i3c_write_read_helper)
{
	uint8_t cmd[2] = {0x20, 0x21};
	uint8_t prime[4] = {0xE0, 0xE1, 0xE2, 0xE3};
	uint8_t rx[4] = {0};

	zassert_ok(loopback_prime_target_tx(prime, sizeof(prime), 0));
	zassert_ok(i3c_write_read(loopback_desc, cmd, sizeof(cmd), rx, sizeof(rx)));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(cmd[0], g_loopback.rx_buf[0]);
	zassert_equal(cmd[1], g_loopback.rx_buf[1]);
	zassert_mem_equal(prime, rx, sizeof(prime));
}

ZTEST(i3c_loopback_xfer, test_i3c_burst_write_and_burst_read)
{
	uint8_t start = 0x40;
	uint8_t tx[3] = {0xD0, 0xD1, 0xD2};
	uint8_t prime[3] = {0xF0, 0xF1, 0xF2};
	uint8_t rx[3] = {0};

	zassert_ok(i3c_burst_write(loopback_desc, start, tx, sizeof(tx)));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(start, g_loopback.rx_buf[0]);
	zassert_mem_equal(tx, &g_loopback.rx_buf[1], sizeof(tx));

	loopback_reset_state();
	zassert_ok(loopback_prime_target_tx(prime, sizeof(prime), 0));
	zassert_ok(i3c_burst_read(loopback_desc, start, rx, sizeof(rx)));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(start, g_loopback.rx_buf[0]);
	zassert_mem_equal(prime, rx, sizeof(prime));
}

ZTEST(i3c_loopback_xfer, test_i3c_reg_byte_helpers)
{
	uint8_t prime[1] = {0x5A};
	uint8_t val = 0;

	/* reg_write_byte: payload is {reg, val} */
	zassert_ok(i3c_reg_write_byte(loopback_desc, 0x08, 0xC3));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(0x08, g_loopback.rx_buf[0]);
	zassert_equal(0xC3, g_loopback.rx_buf[1]);

	loopback_reset_state();
	/* reg_read_byte: writes reg then reads 1 byte */
	zassert_ok(loopback_prime_target_tx(prime, sizeof(prime), 0));
	zassert_ok(i3c_reg_read_byte(loopback_desc, 0x08, &val));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(prime[0], val);
}

ZTEST(i3c_loopback_xfer, test_setmwl_limit_enforced)
{
	struct i3c_ccc_mwl mwl_set = { .len = 4 };
	struct i3c_ccc_mwl mwl_orig;
	uint8_t tx_under[4] = {0x01, 0x02, 0x03, 0x04};
	uint8_t tx_over[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
	struct i3c_msg msg_under = {
		.buf = tx_under, .len = sizeof(tx_under),
		.flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};
	struct i3c_msg msg_over = {
		.buf = tx_over, .len = sizeof(tx_over),
		.flags = I3C_MSG_WRITE | I3C_MSG_STOP,
	};
	int ret;

	/* Snapshot so we can restore. */
	zassert_ok(i3c_ccc_do_getmwl(loopback_desc, &mwl_orig));

	/* Cap MWL at 4 bytes. */
	zassert_ok(i3c_ccc_do_setmwl(loopback_desc, &mwl_set));

	/* Write at the limit must succeed and deliver all bytes. */
	zassert_ok(i3c_transfer(loopback_desc, &msg_under, 1));
	zassert_ok(k_sem_take(&g_loopback.stop_sem, WAIT));
	zassert_equal(sizeof(tx_under), g_loopback.rx_cnt);
	zassert_mem_equal(tx_under, g_loopback.rx_buf, sizeof(tx_under));

	/* Write over the limit: per spec, target silently drops bytes past
	 * MWL — so the transfer call may succeed but the target receives
	 * <= MWL bytes. Accept either: an error from the transfer, OR
	 * delivery truncated to <= MWL bytes.
	 */
	loopback_reset_state();
	ret = i3c_transfer(loopback_desc, &msg_over, 1);
	(void)k_sem_take(&g_loopback.stop_sem, WAIT);
	if (ret == 0) {
		zassert_true(g_loopback.rx_cnt <= mwl_set.len,
			     "MWL ignored: target got %u bytes, MWL=%u",
			     (unsigned int)g_loopback.rx_cnt, mwl_set.len);
	}

	/* Restore. */
	(void)i3c_ccc_do_setmwl(loopback_desc, &mwl_orig);
}

ZTEST(i3c_loopback_xfer, test_setmrl_limit_enforced)
{
	struct i3c_ccc_mrl mrl_set = { .len = 4 };
	struct i3c_ccc_mrl mrl_orig;
	uint8_t prime[8] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7};
	uint8_t rx[8] = {0};
	struct i3c_msg msg = {
		.buf = rx, .len = sizeof(rx),
		.flags = I3C_MSG_READ | I3C_MSG_STOP,
	};
	int ret;

	zassert_ok(i3c_ccc_do_getmrl(loopback_desc, &mrl_orig));
	zassert_ok(i3c_ccc_do_setmrl(loopback_desc, &mrl_set));

	zassert_ok(loopback_prime_target_tx(prime, sizeof(prime), 0));

	/* Per spec, target's TX is capped at MRL — the controller's read
	 * either truncates or errors. Either is acceptable. msg.num_xfer
	 * must be <= MRL.
	 */
	ret = i3c_transfer(loopback_desc, &msg, 1);
	(void)k_sem_take(&g_loopback.stop_sem, WAIT);
	if (ret == 0) {
		zassert_true(msg.num_xfer <= mrl_set.len,
			     "MRL ignored: read got %u bytes, MRL=%u", msg.num_xfer,
			     mrl_set.len);
	}

	(void)i3c_ccc_do_setmrl(loopback_desc, &mrl_orig);
}

ZTEST_SUITE(i3c_loopback_xfer, NULL, NULL, xfer_before, NULL, NULL);
