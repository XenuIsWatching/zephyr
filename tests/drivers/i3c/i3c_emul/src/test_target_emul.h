/*
 * Copyright (c) 2026 Ryan McClelland
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef TEST_DRIVERS_I3C_I3C_EMUL_TEST_TARGET_EMUL_H_
#define TEST_DRIVERS_I3C_I3C_EMUL_TEST_TARGET_EMUL_H_

#include <stdint.h>

#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i3c_emul.h>

struct test_target_backend_api {
	uint8_t (*get_reg)(const struct emul *target, uint8_t idx);
	void (*set_reg)(const struct emul *target, uint8_t idx, uint8_t value);
	uint8_t (*get_dynamic_addr)(const struct emul *target);
	void (*install_mock)(const struct emul *target, struct i3c_emul_api *mock);
	int (*trigger_ibi)(const struct emul *target, uint8_t *payload, uint8_t len);
	int (*trigger_hj)(const struct emul *target);
	int (*trigger_crr)(const struct emul *target);
	bool (*ibi_was_enabled)(const struct emul *target);
	void (*set_status_fmt1)(const struct emul *target, uint16_t status);
	uint16_t (*get_mrl)(const struct emul *target);
	uint16_t (*get_mwl)(const struct emul *target);
	bool (*deftgts_was_seen)(const struct emul *target);
	size_t (*get_deftgts)(const struct emul *target, uint8_t *out, size_t out_len);
	void (*clear_deftgts)(const struct emul *target);
};

uint8_t test_target_get_reg(const struct emul *target, uint8_t idx);
void test_target_set_reg(const struct emul *target, uint8_t idx, uint8_t value);
uint8_t test_target_get_dynamic_addr(const struct emul *target);
void test_target_install_mock(const struct emul *target, struct i3c_emul_api *mock);
int test_target_trigger_ibi(const struct emul *target, uint8_t *payload, uint8_t len);
int test_target_trigger_hj(const struct emul *target);
int test_target_trigger_crr(const struct emul *target);
bool test_target_ibi_was_enabled(const struct emul *target);
void test_target_set_status_fmt1(const struct emul *target, uint16_t status);
uint16_t test_target_get_mrl(const struct emul *target);
uint16_t test_target_get_mwl(const struct emul *target);
bool test_target_deftgts_was_seen(const struct emul *target);
size_t test_target_get_deftgts(const struct emul *target, uint8_t *out, size_t out_len);
void test_target_clear_deftgts(const struct emul *target);

#endif /* TEST_DRIVERS_I3C_I3C_EMUL_TEST_TARGET_EMUL_H_ */
