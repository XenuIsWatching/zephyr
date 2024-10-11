/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LSM6DSV16X_RTIO_H_
#define ZEPHYR_DRIVERS_SENSOR_LSM6DSV16X_RTIO_H_

#include <zephyr/device.h>
#include <zephyr/rtio/rtio.h>

void lsm6dsv16x_submit(const struct device *sensor, struct rtio_iodev_sqe *iodev_sqe);

void lsm6dsv16x_submit_stream(const struct device *sensor, struct rtio_iodev_sqe *iodev_sqe);

void lsm6dsv16x_fifo_event(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SENSOR_LSM6DSV16X_RTIO_H_ */