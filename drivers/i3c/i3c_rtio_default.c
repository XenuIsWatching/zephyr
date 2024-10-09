/*
 * Copyright (c) 2024 Google LLC
 * Copyright (c) 2024 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/rtio.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/rtio/work.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(i3c_rtio, CONFIG_I3C_LOG_LEVEL);

static int i3c_iodev_submit_rx(struct rtio_iodev_sqe *iodev_sqe, struct i3c_msg msgs[2],
			       uint8_t *num_msgs)
{
	__ASSERT_NO_MSG(iodev_sqe->sqe.op == RTIO_OP_RX);

	msgs[0].buf = iodev_sqe->sqe.rx.buf;
	msgs[0].len = iodev_sqe->sqe.rx.buf_len;
	msgs[0].flags =
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_STOP) ? I3C_MSG_STOP : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_RESTART) ? I3C_MSG_RESTART : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_HDR) ? I3C_MSG_HDR : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_NBCH) ? I3C_MSG_NBCH : 0) |
		I3C_MSG_READ;
	msgs[0].hdr_mode = RTIO_IODEV_I3C_HDR_MODE_GET(iodev_sqe->sqe.iodev_flags);
	msgs[0].hdr_cmd_code = RTIO_IODEV_I3C_HDR_CMD_CODE_GET(iodev_sqe->sqe.iodev_flags);
	*num_msgs = 1;
	return 0;
}

static int i3c_iodev_submit_tx(struct rtio_iodev_sqe *iodev_sqe, struct i3c_msg msgs[2],
			       uint8_t *num_msgs)
{
	__ASSERT_NO_MSG(iodev_sqe->sqe.op == RTIO_OP_TX);

	msgs[0].buf = (uint8_t *)iodev_sqe->sqe.tx.buf;
	msgs[0].len = iodev_sqe->sqe.tx.buf_len;
	msgs[0].flags =
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_STOP) ? I3C_MSG_STOP : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_RESTART) ? I3C_MSG_RESTART : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_HDR) ? I3C_MSG_HDR : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_NBCH) ? I3C_MSG_NBCH : 0) |
		I3C_MSG_WRITE;
	msgs[0].hdr_mode = RTIO_IODEV_I3C_HDR_MODE_GET(iodev_sqe->sqe.iodev_flags);
	msgs[0].hdr_cmd_code = RTIO_IODEV_I3C_HDR_CMD_CODE_GET(iodev_sqe->sqe.iodev_flags);
	*num_msgs = 1;
	return 0;
}

static int i3c_iodev_submit_tiny_tx(struct rtio_iodev_sqe *iodev_sqe, struct i3c_msg msgs[2],
				    uint8_t *num_msgs)
{
	__ASSERT_NO_MSG(iodev_sqe->sqe.op == RTIO_OP_TINY_TX);

	msgs[0].buf = (uint8_t *)iodev_sqe->sqe.tiny_tx.buf;
	msgs[0].len = iodev_sqe->sqe.tiny_tx.buf_len;
	msgs[0].flags =
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_STOP) ? I3C_MSG_STOP : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_RESTART) ? I3C_MSG_RESTART : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_HDR) ? I3C_MSG_HDR : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_NBCH) ? I3C_MSG_NBCH : 0) |
		I3C_MSG_WRITE;
	msgs[0].hdr_mode = RTIO_IODEV_I3C_HDR_MODE_GET(iodev_sqe->sqe.iodev_flags);
	msgs[0].hdr_cmd_code = RTIO_IODEV_I3C_HDR_CMD_CODE_GET(iodev_sqe->sqe.iodev_flags);
	*num_msgs = 1;
	return 0;
}

static int i3c_iodev_submit_txrx(struct rtio_iodev_sqe *iodev_sqe, struct i3c_msg msgs[2],
				 uint8_t *num_msgs)
{
	__ASSERT_NO_MSG(iodev_sqe->sqe.op == RTIO_OP_TXRX);

	msgs[0].buf = (uint8_t *)iodev_sqe->sqe.txrx.tx_buf;
	msgs[0].len = iodev_sqe->sqe.txrx.buf_len;
	msgs[0].flags = ((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_HDR) ? I3C_MSG_HDR : 0) |
			((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_NBCH) ? I3C_MSG_NBCH : 0) |
			I3C_MSG_WRITE;
	msgs[0].hdr_mode = RTIO_IODEV_I3C_HDR_MODE_GET(iodev_sqe->sqe.iodev_flags);
	msgs[0].hdr_cmd_code = RTIO_IODEV_I3C_HDR_CMD_CODE_GET(iodev_sqe->sqe.iodev_flags);
	msgs[1].buf = iodev_sqe->sqe.txrx.rx_buf;
	msgs[1].len = iodev_sqe->sqe.txrx.buf_len;
	msgs[1].flags =
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_STOP) ? I3C_MSG_STOP : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_RESTART) ? I3C_MSG_RESTART : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_HDR) ? I3C_MSG_HDR : 0) |
		((iodev_sqe->sqe.iodev_flags & RTIO_IODEV_I3C_NBCH) ? I3C_MSG_NBCH : 0) |
		I3C_MSG_READ;
	msgs[1].hdr_mode = RTIO_IODEV_I3C_HDR_MODE_GET(iodev_sqe->sqe.iodev_flags);
	msgs[1].hdr_cmd_code = RTIO_IODEV_I3C_HDR_CMD_CODE_GET(iodev_sqe->sqe.iodev_flags);
	*num_msgs = 2;
	return 0;
}

static int i3c_iodev_submit_ccc(struct rtio_iodev_sqe *iodev_sqe, struct i3c_ccc_payload *payload)
{
	__ASSERT_NO_MSG(iodev_sqe->sqe.op == RTIO_OP_I3C_CCC);

	payload = iodev_sqe->sqe.ccc_payload;

	return 0;
}

void i3c_iodev_submit_work_handler(struct rtio_iodev_sqe *iodev_sqe)
{
	const struct i3c_iodev_data *data =
		(const struct i3c_iodev_data *)iodev_sqe->sqe.iodev->data;
	struct i3c_device_desc *desc;

	LOG_DBG("Sync RTIO work item for: %p", (void *)iodev_sqe);

	struct rtio_iodev_sqe *transaction_current = iodev_sqe;
	struct i3c_msg msgs[2];
	uint8_t num_msgs = 0;
	struct i3c_ccc_payload *payload = NULL;
	int rc = 0;

	/* TODO: there really needs to be a compile time way to get the i3c_device_desc */
	desc = i3c_device_find(data->bus, &data->dev_id);
	if (!desc) {
		LOG_ERR("Cannot find I3C device descriptor");
		rc = -ENODEV;
		goto error;
	}

	do {
		/* Convert the iodev_sqe back to an i3c_msg */
		switch (transaction_current->sqe.op) {
		case RTIO_OP_RX:
			rc = i3c_iodev_submit_rx(transaction_current, msgs, &num_msgs);
			break;
		case RTIO_OP_TX:
			rc = i3c_iodev_submit_tx(transaction_current, msgs, &num_msgs);
			break;
		case RTIO_OP_TINY_TX:
			rc = i3c_iodev_submit_tiny_tx(transaction_current, msgs, &num_msgs);
			break;
		case RTIO_OP_TXRX:
			rc = i3c_iodev_submit_txrx(transaction_current, msgs, &num_msgs);
			break;
		case RTIO_OP_I3C_CCC:
			rc = i3c_iodev_submit_ccc(transaction_current, payload);
			break;
		default:
			LOG_ERR("Invalid op code %d for submission %p", transaction_current->sqe.op,
				(void *)&transaction_current->sqe);
			rc = -EIO;
			break;
		}

		if (rc == 0) {
			__ASSERT_NO_MSG(num_msgs > 0);

			if (transaction_current->sqe.op == RTIO_OP_I3C_CCC) {
				rc = i3c_do_ccc(desc->bus, payload);
			} else {
				rc = i3c_transfer(desc, msgs, num_msgs);
			}
			transaction_current = rtio_txn_next(transaction_current);
		}
	} while (rc == 0 && transaction_current != NULL);

error:
	if (rc != 0) {
		rtio_iodev_sqe_err(iodev_sqe, rc);
	} else {
		rtio_iodev_sqe_ok(iodev_sqe, 0);
	}
}

void i3c_iodev_submit_fallback(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	LOG_DBG("Executing fallback for dev: %p, sqe: %p", (void *)dev, (void *)iodev_sqe);

	struct rtio_work_req *req = rtio_work_req_alloc();

	if (req == NULL) {
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	rtio_work_req_submit(req, iodev_sqe, i3c_iodev_submit_work_handler);
}
