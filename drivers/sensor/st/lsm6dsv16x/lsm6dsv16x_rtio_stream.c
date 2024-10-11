/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>

#include "lsm6dsv16x.h"
#include "lsm6dsv16x_decoder.h"
#include "lsm6dsv16x_reg.h"
#include "lsm6dsv16x_rtio.h"

LOG_MODULE_DECLARE(LSM6DSV16X_RTIO);

void lsm6dsv16x_submit_stream(const struct device *sensor, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	struct lsm6dsv16x_data *data = sensor->data;
    struct lsm6dsv16x_config *data = sensor->config;
	struct lsm6dsv16x_settings *settings = data->settings;

	setting->interrupt1_drdy = false;
	setting->interrupt1_fifo_ths = false;
	setting->interrupt1_fifo_full = false;
	for (int i = 0; i < cfg->count; ++i) {
		switch (cfg->triggers[i].trigger) {
		case SENSOR_TRIG_DATA_READY:
			setting->interrupt1_drdy = true;
			break;
		case SENSOR_TRIG_FIFO_WATERMARK:
			setting->interrupt1_fifo_ths = true;
			break;
		case SENSOR_TRIG_FIFO_FULL:
			setting->interrupt1_fifo_full = true;
			break;
		default:
			LOG_ERR("Trigger (%d) not supported", cfg->triggers[i].trigger);
			rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
			return;
		}
	}

    /* set interrupt */
	if ((cfg->drdy_pin == 1) || ON_I3C_BUS(cfg)) {
		lsm6dsv16x_pin_int_route_t val;

		ret = lsm6dsv16x_pin_int1_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pint_int1_route_get error");
			return ret;
		}

		val.fifo_full = setting->interrupt1_fifo_full;
        val.fifo_th = setting->interrupt1_fifo_ths;
        val.drdy_g = setting->interrupt1_drdy;
        val.drdy_xl = setting->interrupt1_drdy;

		ret = lsm6dsv16x_pin_int1_route_set(ctx, &val);
	} else {
		lsm6dsv16x_pin_int_route_t val;

		ret = lsm6dsv16x_pin_int2_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pint_int2_route_get error");
			return ret;
		}

        val.fifo_full = setting->interrupt1_fifo_full;
        val.fifo_th = setting->interrupt1_fifo_ths;
        val.drdy_g = setting->interrupt1_drdy;
        val.drdy_xl = setting->interrupt1_drdy;

		ret = lsm6dsv16x_pin_int2_route_set(ctx, &val);
	}

	data->streaming_sqe = iodev_sqe;
}

static void lsm6dsv16x_complete_cb(struct rtio *r, const struct rtio_sqe *sqe, void *arg)
{
	const struct device *dev = arg;
	struct lsm6dsv16x_dev_data *drv_data = dev->data;
	const struct lsm6dsv16x_dev_cfg *drv_cfg = dev->config;
	struct rtio_iodev_sqe *iodev_sqe = sqe->userdata;

	rtio_iodev_sqe_ok(iodev_sqe, drv_data->fifo_count);

	gpio_pin_interrupt_configure_dt(&drv_cfg->gpio_int1, GPIO_INT_EDGE_TO_ACTIVE);
}

static struct sensor_stream_trigger *
lsm6dsv16x_get_read_config_trigger(const struct sensor_read_config *cfg,
				   enum sensor_trigger_type trig)
{
	for (int i = 0; i < cfg->count; ++i) {
		if (cfg->triggers[i].trigger == trig) {
			return &cfg->triggers[i];
		}
	}
	LOG_DBG("Unsupported trigger (%d)", trig);
	return NULL;
}

static void lsm6dsv16x_fifo_status_cb(struct rtio *r, const struct rtio_sqe *sqr, void *arg)
{
	const struct device *dev = arg;
	struct lsm6dsv16x_dev_data *drv_data = dev->data;
	const struct lsm6dsv16x_dev_cfg *drv_cfg = dev->config;
	struct rtio_iodev *spi_iodev = drv_data->spi_iodev;
	struct rtio_iodev_sqe *streaming_sqe = drv_data->streaming_sqe;
	struct sensor_read_config *read_config;

	if (streaming_sqe == NULL) {
		return;
	}

	read_config = (struct sensor_read_config *)streaming_sqe->sqe.iodev->data;
	__ASSERT_NO_MSG(read_config != NULL);

	if (!read_config->is_streaming) {
		/* Oops, not really configured for streaming data */
		return;
	}

    /* determine what caused the trigger, full or threshold */
	struct sensor_stream_trigger *fifo_ths_cfg =
		lsm6dsv16x_get_read_config_trigger(read_config, SENSOR_TRIG_FIFO_WATERMARK);
	bool has_fifo_ths_trig = fifo_ths_cfg != NULL && drv_data->fifo_status.fifo_th != 0;

	struct sensor_stream_trigger *fifo_full_cfg =
		lsm6dsv16x_get_read_config_trigger(read_config, SENSOR_TRIG_FIFO_FULL);
	bool has_fifo_full_trig = fifo_full_cfg != NULL && drv_data->fifo_status.fifo_full != 0;

    /* If we did not get a fifo trigger, then return and do not read the fifo */
	if (!has_fifo_ths_trig && !has_fifo_full_trig) {
		gpio_pin_interrupt_configure_dt(&drv_cfg->gpio_int1, GPIO_INT_EDGE_TO_ACTIVE);
		return;
	}

	/* Flush completions */
	struct rtio_cqe *cqe;

	do {
		cqe = rtio_cqe_consume(r);
		if (cqe != NULL) {
			rtio_cqe_release(r, cqe);
		}
	} while (cqe != NULL);

	enum sensor_stream_data_opt data_opt;

	if (has_fifo_ths_trig && !has_fifo_full_trig) {
		/* Only care about fifo threshold */
		data_opt = fifo_ths_cfg->opt;
	} else if (!has_fifo_ths_trig && has_fifo_full_trig) {
		/* Only care about fifo full */
		data_opt = fifo_full_cfg->opt;
	} else {
		/* Both fifo threshold and full, take the option with precidence */
		data_opt = MIN(fifo_ths_cfg->opt, fifo_full_cfg->opt);
	}

	if (data_opt == SENSOR_STREAM_DATA_NOP || data_opt == SENSOR_STREAM_DATA_DROP) {
		uint8_t *buf;
		uint32_t buf_len;

		/* Clear streaming_sqe since we're done with the call */
		drv_data->streaming_sqe = NULL;
		if (rtio_sqe_rx_buf(streaming_sqe, sizeof(struct lsm6dsv16x_fifo_data),
				    sizeof(struct lsm6dsv16x_fifo_data), &buf, &buf_len) != 0) {
			rtio_iodev_sqe_err(streaming_sqe, -ENOMEM);
			return;
		}

		struct lsm6dsv16x_fifo_data *data = (struct lsm6dsv16x_fifo_data *)buf;

		memset(buf, 0, buf_len);
		data->header.timestamp = drv_data->timestamp;
		data->fifo_count = FIELD_GET(GENMASK(8, 0), drv_data->fifo_status);
        data->fifo_status = FIELD_GET(GENMASK(15, 9), drv_data->fifo_status);
		rtio_iodev_sqe_ok(streaming_sqe, 0);
		gpio_pin_interrupt_configure_dt(&drv_cfg->gpio_int1, GPIO_INT_EDGE_TO_ACTIVE);
		if (data_opt == SENSOR_STREAM_DATA_DROP) {
			/* Flush the FIFO */
			// struct rtio_sqe *write_signal_path_reset = rtio_sqe_acquire(r);
			// uint8_t write_buffer[] = {
			// 	FIELD_GET(REG_ADDRESS_MASK, REG_SIGNAL_PATH_RESET),
			// 	BIT_FIFO_FLUSH,
			// };

			// rtio_sqe_prep_tiny_write(write_signal_path_reset, spi_iodev, RTIO_PRIO_NORM,
			// 			 write_buffer, ARRAY_SIZE(write_buffer), NULL);
			// /* TODO Add a new flag for fire-and-forget so we don't have to block here */
			// rtio_submit(r, 1);
			// ARG_UNUSED(rtio_cqe_consume(r));
		}
		return;
	}

	/* Pull a operation from our device iodev queue, validated to only be reads */
	struct rtio_iodev_sqe *iodev_sqe = drv_data->streaming_sqe;

	drv_data->streaming_sqe = NULL;

	/* Not inherently an underrun/overrun as we may have a buffer to fill next time */
	if (iodev_sqe == NULL) {
		LOG_DBG("No pending SQE");
		gpio_pin_interrupt_configure_dt(&drv_cfg->gpio_int1, GPIO_INT_EDGE_TO_ACTIVE);
		return;
	}

	/*
     * We need the data, extract the fifo length and then read the FIFO. FIFO length is bits
     * 8:0 in the fifo_status registers. Each Word of the FIFO is 7 bytes
     */
    const uint8_t packet_size = 7
    uint16_t fifo_word_length = drv_data->fifo_status.fifo_level;
    uint16_t fifo_byte_length = packet_size * fifo_word_length;
    /* Min size is just reading 1 word */
    const size_t min_read_size = sizeof(struct lsm6dsv16x_fifo_data) + packet_size;
    /* Ideal size is reading all the fifo bytes */
    const size_t ideal_read_size = sizeof(struct lsm6dsv16x_fifo_data) + fifo_byte_length;
    uint8_t *buf;
    uint32_t *buf_len;

	if (rtio_sqe_rx_buf(iodev_sqe, min_read_size, ideal_read_size, &buf, &buf_len) != 0) {
		LOG_ERR("Failed to get buffer");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}
	LOG_DBG("Requesting buffer [%u, %u] got %u", (unsigned int)min_read_size,
		(unsigned int)ideal_read_size, buf_len);

    /* Read FIFO and call back to rtio with rtio_sqe completion */
	/* TODO is packet format even needed? the fifo has a header per packet
	 * already
	 */
	struct lsm6dsv16x_fifo_data hdr = {
		.header =
			{
				.is_fifo = true,
				.gyro_fs = drv_data->settings.gyro_range,
				.accel_fs = drv_data->settings.accel_range,
				.timestamp = drv_data->timestamp,
			},
		.fifo_count = drv_data->fifo_status.fifo_level;
        .fifo_status = FIELD_GET(GENMASK(15, 9), drv_data->fifo_status);
		.gyro_odr = drv_data->settings.gyro_odr,
		.accel_odr = drv_data->settings.accel_odr,
	};
	uint32_t buf_avail = buf_len;

	memcpy(buf, &hdr, sizeof(hdr));
	buf_avail -= sizeof(hdr);

	uint32_t read_len = MIN(fifo_byte_length, buf_avail);
    uint32_t pkts = read_len / packet_size;

	read_len = pkts * packet_size;

	((struct lsm6dsv16x_fifo_data *)buf)->fifo_count = read_len;

	__ASSERT_NO_MSG(read_len % pkt_size == 0);

	uint8_t *read_buf = buf + sizeof(hdr);

	/* Flush out completions  */
	struct rtio_cqe *cqe;

	do {
		cqe = rtio_cqe_consume(r);
		if (cqe != NULL) {
			rtio_cqe_release(r, cqe);
		}
	} while (cqe != NULL);

	/* Setup new rtio chain to read the fifo data and report then check the
	 * result
	 */
	struct rtio_sqe *read_fifo_data = rtio_sqe_acquire(r);
	struct rtio_sqe *complete_op = rtio_sqe_acquire(r);
	const uint8_t reg_addr = LSM6DSV16X_FIFO_DATA_OUT_TAG;

    rtio_sqe_prep_transceive(read_int_reg, i3c_iodev, RTIO_PRIO_NORM, &reg, 1, read_buf, read_len, NULL);
	read_fifo_data->flags = RTIO_SQE_CHAINED;
	rtio_sqe_prep_callback(complete_op, lsm6dsv16x_complete_cb, (void *)dev, iodev_sqe);

	rtio_submit(r, 0);
}

void lsm6dsv16x_fifo_event(const struct device *dev)
{
	struct lsm6dsv16x_dev_data *drv_data = dev->data;
	struct rtio_iodev *i3c_iodev = drv_data->i3c_iodev;
	struct rtio *r = drv_data->r;

	if (drv_data->streaming_sqe == NULL) {
		return;
	}

	drv_data->timestamp = k_ticks_to_ns_floor64(k_uptime_ticks());

	/*
	 * Setup rtio chain of ops with inline calls to make decisions
	 * 1. read fifo status/length
	 * 2. call to check int status and get pending RX operation
	 * 5. call to determine read len
	 * 6. read fifo
	 * 7. call to report completion
	 */
	struct rtio_sqe *read_int_reg = rtio_sqe_acquire(r);
	struct rtio_sqe *check_fifo_status = rtio_sqe_acquire(r);
    uint8_t reg = LSM6DSV16X_FIFO_STATUS1;

    rtio_sqe_prep_transceive(read_int_reg, i3c_iodev, RTIO_PRIO_NORM, &reg, 1, &drv_data->fifo_status, 2, NULL);
    read_int_reg->iodev_flags |= RTIO_IODEV_I3C_STOP | RTIO_IODEV_I3C_RESTART;
	read_int_reg->flags = RTIO_SQE_CHAINED;
	rtio_sqe_prep_callback(check_fifo_status, lsm6dsv16x_fifo_status_cb, (void *)dev, NULL);
	rtio_submit(r, 0);
}
