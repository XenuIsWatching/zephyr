/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LSM6DSV16X_DECODER_H_
#define ZEPHYR_DRIVERS_SENSOR_LSM6DSV16X_DECODER_H_

#include <stdint.h>
#include <zephyr/drivers/sensor.h>

struct lsm6dsv16x_decoder_header {
	uint64_t timestamp;
	uint16_t is_fifo: 1;
	uint16_t gyro_fs: 4;
	uint16_t accel_fs: 4;
	uint16_t reserved: 7;
} __attribute__((__packed__));

struct lsm6dsv16x_fifo_data {
	struct lsm6dsv16x_decoder_header header;
	uint16_t fifo_count: 9;
	uint16_t fifo_status: 7;
	uint16_t gyro_odr: 5;
	uint16_t accel_odr: 5;
	uint16_t reserved: 6;
} __attribute__((__packed__));

struct lsm6dsv16x_encoded_data {
	struct lsm6dsv16x_decoder_header header;
	struct {
		uint8_t has_gyro: 1;
		uint8_t has_accel: 1;
		uint8_t has_temp: 1;
	}  __attribute__((__packed__));
	int16_t temp;
	int16_t gyro_xyz[3];
	int16_t accel_xyz[3];
};

int lsm6dsv16x_encode(const struct device *dev, const struct sensor_chan_spec *const channels,
		    const size_t num_channels, uint8_t *buf);

int lsm6dsv16x_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder);

#endif /* ZEPHYR_DRIVERS_SENSOR_LSM6DSV16X_DECODER_H_ */
