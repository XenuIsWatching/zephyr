/*
 * Copyright 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I3C_ADDRESSES_H_
#define ZEPHYR_INCLUDE_DRIVERS_I3C_ADDRESSES_H_

/**
 * @brief I3C Address-related Helper Code
 * @defgroup i3c_addresses I3C Address-related Helper Code
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I3C_BROADCAST_ADDR			0x7E
#define I3C_MAX_ADDR				0x7F

enum i3c_addr_slot_status {
	I3C_ADDR_SLOT_STATUS_FREE = 0U,
	I3C_ADDR_SLOT_STATUS_RSVD,
	I3C_ADDR_SLOT_STATUS_I3C_DEV,
	I3C_ADDR_SLOT_STATUS_I2C_DEV,
	I3C_ADDR_SLOT_STATUS_MASK = 0x03U,
};

/**
 * @brief Structure to keep track of addresses on I3C bus.
 */
struct i3c_addr_slots {
	/* 2 bits per slot */
	unsigned long	slots[((I3C_MAX_ADDR + 1) * 2) / BITS_PER_LONG];
};

/**
 * @brief Initialize the I3C address slots struct.
 *
 * This clears out the assigned address bits, and set the reserved
 * address bits according to the I3C specification.
 */
void i3c_addr_slots_init(struct i3c_addr_slots *slots);

/**
 * @brief Set the address status of a device.
 *
 * @param slots Pointer to the address slots structure.
 * @param dev_addr Device address.
 * @param status New status for the address @p dev_addr.
 */
void i3c_addr_slots_status_set(struct i3c_addr_slots *slots,
			       uint8_t dev_addr,
			       enum i3c_addr_slot_status status);

/**
 * @brief Get the address status of a device.
 *
 * @param slots Pointer to the address slots structure.
 * @param dev_addr Device address.
 *
 * @return Address status for the address @p dev_addr.
 */
enum i3c_addr_slot_status i3c_addr_slots_status_get(struct i3c_addr_slots *slots,
						    uint8_t dev_addr);

/**
 * @brief Check if the address is free.
 *
 * @param slots Pointer to the address slots structure.
 * @param dev_addr Device address.
 *
 * @retval true if address is free.
 * @retval false if address is not free.
 */
bool i3c_addr_slots_status_is_free(struct i3c_addr_slots *slots,
				   uint8_t dev_addr);

/**
 * @brief Find the next free address.
 *
 * This can be used to find the next free address that can be
 * assigned to a new device.
 *
 * @param slots Pointer to the address slots structure.
 *
 * @return The next free address, or 0 if none found.
 */
uint8_t i3c_addr_slots_status_next_free_find(struct i3c_addr_slots *slots);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_I3C_ADDRESSES_H_ */
