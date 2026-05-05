/*
 * Copyright (c) 2026 Ryan McClelland
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tiny synthetic I3C peripheral emulator used by the M2 i3c_emul ztest. Will
 * be replaced (or supplemented) by the full reference peripheral in M6.
 */

#define DT_DRV_COMPAT zephyr_i3c_emul_test_target

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/drivers/i3c_emul.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <string.h>

#include "test_target_emul.h"

#define TEST_TARGET_REG_SIZE 32U

struct test_target_cfg {
	uint64_t pid;
	uint8_t bcr;
	uint8_t dcr;
};

struct test_target_data {
	uint8_t regs[TEST_TARGET_REG_SIZE];
	uint8_t cursor;
	uint8_t dyn_addr;
};

static int test_target_xfers(const struct emul *target, struct i3c_msg *msgs, uint8_t num_msgs)
{
	struct test_target_data *data = target->data;

	for (uint8_t i = 0; i < num_msgs; i++) {
		struct i3c_msg *m = &msgs[i];

		if ((m->flags & I3C_MSG_RW_MASK) == I3C_MSG_WRITE) {
			if (m->len == 0U) {
				continue;
			}
			data->cursor = m->buf[0] % TEST_TARGET_REG_SIZE;
			for (uint32_t j = 1; j < m->len; j++) {
				data->regs[data->cursor] = m->buf[j];
				data->cursor = (data->cursor + 1U) % TEST_TARGET_REG_SIZE;
			}
			m->num_xfer = m->len;
		} else {
			for (uint32_t j = 0; j < m->len; j++) {
				m->buf[j] = data->regs[data->cursor];
				data->cursor = (data->cursor + 1U) % TEST_TARGET_REG_SIZE;
			}
			m->num_xfer = m->len;
		}
	}

	return 0;
}

static int test_target_do_ccc(const struct emul *target, struct i3c_ccc_payload *payload,
			      bool is_broadcast)
{
	struct test_target_data *data = target->data;
	const struct test_target_cfg *cfg = target->cfg;
	struct i3c_ccc_target_payload *tp = NULL;

	if (!is_broadcast) {
		for (size_t i = 0; i < payload->targets.num_targets; i++) {
			if (payload->targets.payloads[i].addr == data->dyn_addr) {
				tp = &payload->targets.payloads[i];
				break;
			}
		}
		if (tp == NULL) {
			return 0;
		}
	}

	switch (payload->ccc.id) {
	case I3C_CCC_GETPID:
		if (tp != NULL && tp->data != NULL && tp->data_len >= 6U) {
			sys_put_be48(cfg->pid, tp->data);
			tp->num_xfer = 6U;
		}
		return 0;
	case I3C_CCC_GETBCR:
		if (tp != NULL && tp->data != NULL && tp->data_len >= 1U) {
			tp->data[0] = cfg->bcr;
			tp->num_xfer = 1U;
		}
		return 0;
	case I3C_CCC_GETDCR:
		if (tp != NULL && tp->data != NULL && tp->data_len >= 1U) {
			tp->data[0] = cfg->dcr;
			tp->num_xfer = 1U;
		}
		return 0;
	case I3C_CCC_SETDASA:
	case I3C_CCC_SETNEWDA:
	case I3C_CCC_RSTDAA:
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int test_target_set_dynamic_addr(const struct emul *target, uint8_t dynamic_addr)
{
	struct test_target_data *data = target->data;

	data->dyn_addr = dynamic_addr;
	return 0;
}

static const struct i3c_emul_api test_target_api = {
	.xfers = test_target_xfers,
	.do_ccc = test_target_do_ccc,
	.set_dynamic_addr = test_target_set_dynamic_addr,
};

uint8_t test_target_get_reg(const struct emul *target, uint8_t idx)
{
	struct test_target_data *data = target->data;

	return data->regs[idx % TEST_TARGET_REG_SIZE];
}

void test_target_set_reg(const struct emul *target, uint8_t idx, uint8_t value)
{
	struct test_target_data *data = target->data;

	data->regs[idx % TEST_TARGET_REG_SIZE] = value;
}

uint8_t test_target_get_dynamic_addr(const struct emul *target)
{
	struct test_target_data *data = target->data;

	return data->dyn_addr;
}

void test_target_install_mock(const struct emul *target, struct i3c_emul_api *mock)
{
	target->bus.i3c->mock_api = mock;
}

static const struct test_target_backend_api test_target_backend = {
	.get_reg = test_target_get_reg,
	.set_reg = test_target_set_reg,
	.get_dynamic_addr = test_target_get_dynamic_addr,
	.install_mock = test_target_install_mock,
};

static int test_target_init(const struct emul *target, const struct device *parent)
{
	const struct test_target_cfg *cfg = target->cfg;

	ARG_UNUSED(parent);

	target->bus.i3c->pid = cfg->pid;
	target->bus.i3c->bcr = cfg->bcr;
	target->bus.i3c->dcr = cfg->dcr;
	return 0;
}

#define TEST_TARGET_INIT(n)                                                                        \
	static struct test_target_data test_target_data_##n;                                       \
	static const struct test_target_cfg test_target_cfg_##n = {                                \
		.pid = ((uint64_t)DT_INST_PROP_BY_IDX(n, reg, 1) << 32) |                          \
		       DT_INST_PROP_BY_IDX(n, reg, 2),                                             \
		.bcr = DT_INST_PROP(n, bcr),                                                       \
		.dcr = DT_INST_PROP(n, dcr),                                                       \
	};                                                                                         \
	I3C_DEVICE_DT_INST_DEFINE(n, NULL, NULL, &test_target_data_##n, &test_target_cfg_##n,      \
				  POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, NULL);            \
	EMUL_DT_INST_DEFINE(n, test_target_init, &test_target_data_##n, &test_target_cfg_##n,      \
			    &test_target_api, &test_target_backend);

DT_INST_FOREACH_STATUS_OKAY(TEST_TARGET_INIT)
