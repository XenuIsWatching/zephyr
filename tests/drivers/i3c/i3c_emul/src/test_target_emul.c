/*
 * Copyright (c) 2026 Ryan McClelland
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Synthetic I3C peripheral emulator used by the i3c_emul ztest. Implements
 * the full struct i3c_emul_api surface: private xfers, direct/broadcast CCC
 * responders (GETPID/GETBCR/GETDCR/GETSTATUS/GETACCCR, SETMRL/GETMRL/
 * SETMWL/GETMWL, DEFTGTS capture, SETDASA/SETNEWDA/RSTDAA), and a backend
 * trigger API for tests.
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

/* Capacity for a captured DEFTGTS payload: 1 byte count + active-controller
 * descriptor + a handful of target descriptors. The real subsys CCC bytes
 * are tightly packed, this just needs to be a generous upper bound for
 * reasonable test bus topologies.
 */
#define TEST_TARGET_DEFTGTS_BUF_SIZE 128U

struct test_target_cfg {
	uint64_t pid;
	uint8_t bcr;
	uint8_t dcr;
};

struct test_target_data {
	uint8_t regs[TEST_TARGET_REG_SIZE];
	uint8_t cursor;
	bool ibi_enabled_seen;

	/* MRL / MWL state set via SETMRL / SETMWL, returned via GETMRL / GETMWL. */
	uint16_t mrl;
	uint16_t mwl;
	uint8_t mrl_ibi_len;

	/* GETSTATUS Format 1 value (test-poke-able for status tests). */
	uint16_t status_fmt1;

	/* Last DEFTGTS broadcast we observed. Captured for test inspection. */
	uint8_t deftgts_buf[TEST_TARGET_DEFTGTS_BUF_SIZE];
	size_t deftgts_len;
	bool deftgts_seen;
};

/*
 * The peripheral's current dynamic address lives in the shared
 * struct i3c_emul (target->bus.i3c->dynamic_addr) — that's the field
 * the bus emulator dispatches CCCs and IBIs against, and matching
 * real silicon there is exactly what makes "the peripheral owns its
 * identity" true. Helpers below read/write through it directly.
 */
static inline uint8_t target_dyn_addr(const struct emul *target)
{
	return target->bus.i3c->dynamic_addr;
}

static inline void target_set_dyn_addr(const struct emul *target, uint8_t addr)
{
	target->bus.i3c->dynamic_addr = addr;
}

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
		uint8_t my_dyn = target_dyn_addr(target);
		uint8_t my_static = target->bus.i3c->static_addr;

		for (size_t i = 0; i < payload->targets.num_targets; i++) {
			uint8_t addr = payload->targets.payloads[i].addr;

			/*
			 * SETDASA addresses the peripheral by its static_addr
			 * (no dynamic_addr yet). Every other direct CCC uses
			 * dynamic_addr. Match on either so the peripheral
			 * intercepts all of its own direct CCCs regardless of
			 * which lookup mode the bus emulator used.
			 */
			if (addr == my_dyn || (my_static != 0U && addr == my_static)) {
				tp = &payload->targets.payloads[i];
				break;
			}
		}
		if (tp == NULL) {
			return 0;
		}
	}

	switch (payload->ccc.id) {
	/* --- Direct GET CCCs --- */
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
	case I3C_CCC_GETMRL:
		if (tp != NULL && tp->data != NULL && tp->data_len >= 2U) {
			sys_put_be16(data->mrl, tp->data);
			tp->num_xfer = 2U;
			if (tp->data_len >= 3U) {
				tp->data[2] = data->mrl_ibi_len;
				tp->num_xfer = 3U;
			}
		}
		return 0;
	case I3C_CCC_GETMWL:
		if (tp != NULL && tp->data != NULL && tp->data_len >= 2U) {
			sys_put_be16(data->mwl, tp->data);
			tp->num_xfer = 2U;
		}
		return 0;
	case I3C_CCC_GETSTATUS:
		if (tp != NULL && tp->data != NULL && tp->data_len >= 2U) {
			sys_put_be16(data->status_fmt1, tp->data);
			tp->num_xfer = 2U;
		}
		return 0;
	case I3C_CCC_GETACCCR:
		/*
		 * GETACCCR response is one byte: dyn_addr left-shifted by 1
		 * with bit[0] set to the odd parity of the address. The
		 * controller verifies both. See drivers/i3c/i3c_common.c
		 * i3c_bus_getacccr().
		 */
		if (tp != NULL && tp->data != NULL && tp->data_len >= 1U) {
			uint8_t dyn = target_dyn_addr(target);

			tp->data[0] = (dyn << 1) | i3c_odd_parity(dyn);
			tp->num_xfer = 1U;
		}
		return 0;

	/* --- Direct SET CCCs --- */
	case I3C_CCC_SETMRL(false):
		if (tp != NULL && tp->data != NULL && tp->data_len >= 2U) {
			data->mrl = sys_get_be16(tp->data);
			if (tp->data_len >= 3U) {
				data->mrl_ibi_len = tp->data[2];
			}
			tp->num_xfer = tp->data_len;
		}
		return 0;
	case I3C_CCC_SETMWL(false):
		if (tp != NULL && tp->data != NULL && tp->data_len >= 2U) {
			data->mwl = sys_get_be16(tp->data);
			tp->num_xfer = 2U;
		}
		return 0;

	/* --- Broadcast SETMRL/SETMWL apply to every target --- */
	case I3C_CCC_SETMRL(true):
		if (payload->ccc.data != NULL && payload->ccc.data_len >= 2U) {
			data->mrl = sys_get_be16(payload->ccc.data);
			if (payload->ccc.data_len >= 3U) {
				data->mrl_ibi_len = payload->ccc.data[2];
			}
		}
		return 0;
	case I3C_CCC_SETMWL(true):
		if (payload->ccc.data != NULL && payload->ccc.data_len >= 2U) {
			data->mwl = sys_get_be16(payload->ccc.data);
		}
		return 0;

	/* --- DEFTGTS broadcast: capture payload bytes for test inspection. --- */
	case I3C_CCC_DEFTGTS:
		if (payload->ccc.data != NULL && payload->ccc.data_len > 0U) {
			size_t copy = MIN(payload->ccc.data_len, sizeof(data->deftgts_buf));

			memcpy(data->deftgts_buf, payload->ccc.data, copy);
			data->deftgts_len = copy;
			data->deftgts_seen = true;
		}
		return 0;

	/*
	 * Wire-level address-mutating CCCs: the peripheral updates its
	 * own dyn_addr from the wire payload, the same way real silicon
	 * would. The bus emulator separately keeps its per-emul mirror
	 * in sync for its CCC-dispatch lookup helper.
	 */
	/*
	 * Wire-level address-mutating CCCs. SETDASA assigns when the
	 * peripheral has no DA; SETNEWDA changes an existing DA. RSTDAA
	 * clears it. Per spec, mis-use is NACK'd (-EACCES).
	 */
	case I3C_CCC_SETDASA:
	case I3C_CCC_SETNEWDA: {
		bool require_da = (payload->ccc.id == I3C_CCC_SETNEWDA);

		if (require_da == (target_dyn_addr(target) == 0U)) {
			return -EACCES;
		}
		if (tp != NULL && tp->data != NULL && tp->data_len >= 1U) {
			target_set_dyn_addr(target, tp->data[0] >> 1);
		}
		return 0;
	}
	case I3C_CCC_RSTDAA:
		target_set_dyn_addr(target, 0);
#ifdef CONFIG_I3C_USE_IBI
		target->bus.i3c->ibi_enabled = false;
#endif
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int test_target_ibi_enable(const struct emul *target)
{
	struct test_target_data *data = target->data;

	data->ibi_enabled_seen = true;
	return 0;
}

static const struct i3c_emul_api test_target_api = {
	.xfers = test_target_xfers,
	.do_ccc = test_target_do_ccc,
	.ibi_enable = test_target_ibi_enable,
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
	return target_dyn_addr(target);
}

void test_target_install_mock(const struct emul *target, struct i3c_emul_api *mock)
{
	target->bus.i3c->mock_api = mock;
}

int test_target_trigger_ibi(const struct emul *target, uint8_t *payload, uint8_t len)
{
	return i3c_emul_target_raise_ibi(target, payload, len);
}

int test_target_trigger_hj(const struct emul *target)
{
	return i3c_emul_target_raise_hj(target);
}

int test_target_trigger_crr(const struct emul *target)
{
	return i3c_emul_target_raise_crr(target);
}

bool test_target_ibi_was_enabled(const struct emul *target)
{
	struct test_target_data *data = target->data;

	return data->ibi_enabled_seen;
}

void test_target_set_status_fmt1(const struct emul *target, uint16_t status)
{
	struct test_target_data *data = target->data;

	data->status_fmt1 = status;
}

uint16_t test_target_get_mrl(const struct emul *target)
{
	struct test_target_data *data = target->data;

	return data->mrl;
}

uint16_t test_target_get_mwl(const struct emul *target)
{
	struct test_target_data *data = target->data;

	return data->mwl;
}

bool test_target_deftgts_was_seen(const struct emul *target)
{
	struct test_target_data *data = target->data;

	return data->deftgts_seen;
}

size_t test_target_get_deftgts(const struct emul *target, uint8_t *out, size_t out_len)
{
	struct test_target_data *data = target->data;
	size_t copy = MIN(out_len, data->deftgts_len);

	if (copy > 0U) {
		memcpy(out, data->deftgts_buf, copy);
	}
	return copy;
}

void test_target_clear_deftgts(const struct emul *target)
{
	struct test_target_data *data = target->data;

	data->deftgts_len = 0;
	data->deftgts_seen = false;
}

static const struct test_target_backend_api test_target_backend = {
	.get_reg = test_target_get_reg,
	.set_reg = test_target_set_reg,
	.get_dynamic_addr = test_target_get_dynamic_addr,
	.install_mock = test_target_install_mock,
	.trigger_ibi = test_target_trigger_ibi,
	.trigger_hj = test_target_trigger_hj,
	.trigger_crr = test_target_trigger_crr,
	.ibi_was_enabled = test_target_ibi_was_enabled,
	.set_status_fmt1 = test_target_set_status_fmt1,
	.get_mrl = test_target_get_mrl,
	.get_mwl = test_target_get_mwl,
	.deftgts_was_seen = test_target_deftgts_was_seen,
	.get_deftgts = test_target_get_deftgts,
	.clear_deftgts = test_target_clear_deftgts,
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

int test_target_bus_reset_to_default(const struct device *bus)
{
	struct i3c_device_id id_a = { .pid = TEST_TARGET_A_PID };
	struct i3c_device_id id_b = { .pid = TEST_TARGET_B_PID };
	struct i3c_device_desc *desc_a = i3c_device_find(bus, &id_a);
	struct i3c_device_desc *desc_b = i3c_device_find(bus, &id_b);
	int rc;

	if (desc_a == NULL || desc_b == NULL) {
		return -ENODEV;
	}

	if (desc_a->dynamic_addr == TEST_TARGET_A_STATIC &&
	    desc_b->dynamic_addr == TEST_TARGET_B_INIT_DA) {
		return 0;
	}

	rc = i3c_bus_rstdaa_all(bus);
	if (rc != 0 && rc != -ENOTSUP) {
		return rc;
	}

	rc = i3c_bus_setdasa(desc_a, TEST_TARGET_A_STATIC);
	if (rc != 0) {
		return rc;
	}

	rc = i3c_do_daa(bus);
	if (rc != 0) {
		return rc;
	}

	if (desc_a->dynamic_addr != TEST_TARGET_A_STATIC ||
	    desc_b->dynamic_addr != TEST_TARGET_B_INIT_DA) {
		return -EIO;
	}

	return 0;
}
