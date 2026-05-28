/*
 * Copyright (c) 2026 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "common.h"

#include <string.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i3c_loopback, LOG_LEVEL_INF);

const struct device *loopback_controller = DEVICE_DT_GET(LOOPBACK_CONTROLLER_NODE);
const struct device *loopback_target = DEVICE_DT_GET(LOOPBACK_TARGET_NODE);
struct i3c_device_desc *loopback_desc;

struct loopback_state g_loopback;

/* --- target-side callbacks --- */

static int loopback_target_write_received(struct i3c_target_config *cfg, uint8_t b)
{
	ARG_UNUSED(cfg);

	if (g_loopback.rx_cnt < LOOPBACK_RX_BUF_SIZE) {
		g_loopback.rx_buf[g_loopback.rx_cnt++] = b;
	}
	g_loopback.write_received_count++;
	return 0;
}

static int loopback_target_read_processed(struct i3c_target_config *cfg, uint8_t *b)
{
	ARG_UNUSED(cfg);

	/* TH callback continues a transfer past the controller's TX FIFO size.
	 * Only feeds bytes that were primed before the read began.
	 */
	if ((g_loopback.tx_cursor != 0) && (g_loopback.tx_cursor < g_loopback.tx_len)) {
		*b = g_loopback.tx_buf[g_loopback.tx_cursor++];
		g_loopback.read_processed_count++;
		return 0;
	}

	return -1;
}

static int loopback_target_stop(struct i3c_target_config *cfg)
{
	ARG_UNUSED(cfg);
	g_loopback.stop_count++;
	k_sem_give(&g_loopback.stop_sem);
	return 0;
}

const struct i3c_target_callbacks loopback_target_cbs = {
	.write_requested_cb = NULL,
	.write_received_cb = loopback_target_write_received,
	.read_requested_cb = NULL,
	.read_processed_cb = loopback_target_read_processed,
	.stop_cb = loopback_target_stop,
};

static struct i3c_target_config loopback_target_cfg = {
	.callbacks = &loopback_target_cbs,
};

/* --- controller-side IBI callback --- */

static int loopback_ibi_cb(struct i3c_device_desc *desc, struct i3c_ibi_payload *payload)
{
	ARG_UNUSED(desc);

	g_loopback.ibi_payload_len = MIN(payload->payload_len, CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE);
	memcpy(g_loopback.ibi_payload, payload->payload, g_loopback.ibi_payload_len);
	k_sem_give(&g_loopback.ibi_sem);
	return 0;
}

/* --- helpers --- */

void loopback_reset_state(void)
{
	memset(g_loopback.rx_buf, 0, sizeof(g_loopback.rx_buf));
	memset(g_loopback.tx_buf, 0, sizeof(g_loopback.tx_buf));
	memset(g_loopback.ibi_payload, 0, sizeof(g_loopback.ibi_payload));
	g_loopback.rx_cnt = 0;
	g_loopback.tx_cursor = 0;
	g_loopback.tx_len = 0;
	g_loopback.ibi_payload_len = 0;
	g_loopback.stop_count = 0;
	g_loopback.write_received_count = 0;
	g_loopback.read_processed_count = 0;
	k_sem_reset(&g_loopback.stop_sem);
	k_sem_reset(&g_loopback.ibi_sem);
}

int loopback_redo_daa(void)
{
	int ret;

	ret = i3c_bus_rstdaa_all(loopback_controller);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_do_daa(loopback_controller);
	if (ret != 0) {
		return ret;
	}

	if (loopback_desc != NULL && loopback_desc->dynamic_addr == 0) {
		return -ENODEV;
	}

	return 0;
}

int loopback_prime_target_tx(const uint8_t *data, size_t len, uint8_t hdr_mode)
{
	int ret;

	if (len > LOOPBACK_TX_BUF_SIZE) {
		return -EINVAL;
	}
	memcpy(g_loopback.tx_buf, data, len);
	g_loopback.tx_len = len;
	ret = i3c_target_tx_write(loopback_target, g_loopback.tx_buf, len, hdr_mode);
	if (ret < 0) {
		return ret;
	}
	g_loopback.tx_cursor = ret;
	return 0;
}

/* --- one-shot init: register target, do initial DAA, cache the desc --- */

static int loopback_global_init(void)
{
	int ret;
	const struct i3c_device_id id = I3C_DEVICE_ID_DT(LOOPBACK_TARGET_DESC_NODE);

	k_sem_init(&g_loopback.stop_sem, 0, 1);
	k_sem_init(&g_loopback.ibi_sem, 0, 1);

	if (!device_is_ready(loopback_controller)) {
		LOG_ERR("controller not ready");
		return -ENODEV;
	}
	if (!device_is_ready(loopback_target)) {
		LOG_ERR("target not ready");
		return -ENODEV;
	}

	ret = i3c_target_register(loopback_target, &loopback_target_cfg);
	if (ret != 0) {
		LOG_ERR("i3c_target_register: %d", ret);
		return ret;
	}

	loopback_desc = i3c_device_find(loopback_controller, &id);
	if (loopback_desc == NULL) {
		LOG_ERR("i3c_device_find for loopback target PID returned NULL");
		return -ENODEV;
	}
	loopback_desc->ibi_cb = loopback_ibi_cb;

	/* The controller's bus init normally runs DAA at boot; redo here so
	 * the desc has a DA known to be in sync after target_register
	 * (target_register may have changed the target side's DA state).
	 */
	ret = loopback_redo_daa();
	if (ret != 0) {
		LOG_ERR("initial DAA failed: %d", ret);
		return ret;
	}

	return 0;
}

SYS_INIT(loopback_global_init, APPLICATION, 90);

/*
 * Some I3C controller drivers enumerate child nodes at build time into a
 * per-controller device array; each child needs a backing DEVICE_DT_DEFINE
 * even when the actual device is just a marker for the loopback target.
 * This satisfies the linker without adding any runtime behavior.
 */
static int loopback_target_stub_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}
I3C_DEVICE_DT_DEFINE(LOOPBACK_TARGET_DESC_NODE, loopback_target_stub_init, NULL, NULL, NULL,
		     POST_KERNEL, 80, NULL);
