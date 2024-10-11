/*
 * Copyright (c) 2023 Google LLC
 * Copyright (c) 2024 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/work.h>
#include "lsm6dsv16x.h"
#include "lsm6dsv16x_decoder.h"
#include "lsm6dsv16x_reg.h"
#include "lsm6dsv16x_rtio.h"
#include "lsm6dsv16x_spi.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LSM6DSV16X_RTIO, CONFIG_SENSOR_LOG_LEVEL);

static void lsm6dsv16x_submit_one_shot(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct sensor_chan_spec *const channels = cfg->channels;
	const size_t num_channels = cfg->count;
	uint32_t min_buf_len = sizeof(struct lsm6dsv16x_encoded_data);
	int rc;
	uint8_t *buf;
	uint32_t buf_len;
	struct lsm6dsv16x_encoded_data *edata;

	/* Get the buffer for the frame, it may be allocated dynamically by the rtio context */
	rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (rc != 0) {
		LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	edata = (struct lsm6dsv16x_encoded_data *)buf;

	lsm6dsv16x_encode(dev, channels, num_channels, buf);

	/* Determine what channels we need to fetch */
	for (int i = 0; i < num_channels; i++) {
        if (channels[i].chan_idx != 0) {
			LOG_ERR("Only channel index 0 supported");
			rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
			return;
		}
		switch (channels[i].chan_type) {
		case SENSOR_CHAN_ALL:
            edata->has_accel = 1;
            edata->has_gyro = 1;
#if defined(CONFIG_LSM6DSV16X_ENABLE_TEMP)
            edata->has_temp = 1;
#endif
        case SENSOR_CHAN_ACCEL_X:
		case SENSOR_CHAN_ACCEL_Y:
		case SENSOR_CHAN_ACCEL_Z:
		case SENSOR_CHAN_ACCEL_XYZ:
			edata->has_accel = 1;
			break;
        case SENSOR_CHAN_GYRO_X:
		case SENSOR_CHAN_GYRO_Y:
		case SENSOR_CHAN_GYRO_Z:
		case SENSOR_CHAN_GYRO_XYZ:
			edata->has_gyro = 1;
			break;
#if defined(CONFIG_LSM6DSV16X_ENABLE_TEMP)
        case SENSOR_CHAN_DIE_TEMP:
            edata->has_temp = 1;
            break;
#endif
        default:
			LOG_ERR("Requested unsupported channel type %d, idx %d",
				channels[i].chan_type, channels[i].chan_idx);
			rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
			return;
		}
    }
#if defined(CONFIG_LSM6DSV16X_ENABLE_TEMP)
	if (edata->has_temp) {
        if (lsm6dsv16x_temperature_raw_get(ctx, &edata->temp) < 0) {
            LOG_ERR("Failed to read sample");
            rtio_iodev_sqe_err(iodev_sqe, -EIO);
            return;
        }
    }
#endif
    if (edata->has_accel) {
        if (lsm6dsv16x_acceleration_raw_get(ctx, &edata->accel_xyz) < 0) {
            LOG_ERR("Failed to read sample");
            rtio_iodev_sqe_err(iodev_sqe, -EIO);
            return;
        }
    }

    if (edata->has_gyro) {
        if (lsm6dsv16x_angular_rate_raw_get(ctx, &edata->gyro_xyz) < 0) {
            LOG_ERR("Failed to read sample");
            rtio_iodev_sqe_err(iodev_sqe, -EIO);
            return;
        }
    }

	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

void lsm6dsv16x_submit_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = cfg->sensor;

	if (!cfg->is_streaming) {
		lsm6dsv16x_submit_one_shot(dev, iodev_sqe);
	} else if (IS_ENABLED(CONFIG_LSM6DSV16X_STREAM)) {
		lsm6dsv16x_submit_stream(dev, iodev_sqe);
	} else {
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
	}
}

void lsm6dsv16x_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_work_req *req = rtio_work_req_alloc();

	__ASSERT_NO_MSG(req);

	rtio_work_req_submit(req, iodev_sqe, lsm6dsv16x_submit_sync);
}

BUILD_ASSERT(sizeof(struct lsm6dsv16x_decoder_header) == 9);
