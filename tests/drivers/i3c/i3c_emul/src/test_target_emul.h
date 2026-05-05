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
};

uint8_t test_target_get_reg(const struct emul *target, uint8_t idx);
void test_target_set_reg(const struct emul *target, uint8_t idx, uint8_t value);
uint8_t test_target_get_dynamic_addr(const struct emul *target);
void test_target_install_mock(const struct emul *target, struct i3c_emul_api *mock);

#endif /* TEST_DRIVERS_I3C_I3C_EMUL_TEST_TARGET_EMUL_H_ */
