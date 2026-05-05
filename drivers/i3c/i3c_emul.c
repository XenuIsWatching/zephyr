/*
 * Copyright (c) 2026 Ryan McClelland
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Emulated I3C bus controller. Mirrors the role of drivers/i2c/i2c_emul.c but
 * covers the I3C controller API: private SDR transfers, broadcast/direct CCCs,
 * dynamic-address assignment, and (in later milestones) IBI, async, RTIO,
 * target-mode and controller-handoff.
 */

#define DT_DRV_COMPAT zephyr_i3c_emul_controller

#define LOG_LEVEL CONFIG_I3C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i3c_emul_ctlr);

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/addresses.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/drivers/i3c/ibi.h>
#include <zephyr/drivers/i3c/target_device.h>
#include <zephyr/drivers/i3c_emul.h>

#include <string.h>

struct i3c_emul_data {
	struct i3c_driver_data common;
#ifdef CONFIG_I3C_USE_IBI
	bool ibi_hj_ack;
	bool ibi_crr_ack;
#endif
#ifdef CONFIG_I3C_TARGET
	struct i3c_target_config *target_cfg;
	bool handoff_accept;
#if CONFIG_I3C_EMUL_TARGET_TX_FIFO_SIZE > 0
	uint8_t tx_fifo[CONFIG_I3C_EMUL_TARGET_TX_FIFO_SIZE];
	size_t tx_fifo_len;
	uint8_t tx_fifo_hdr_mode;
#endif
#endif
};

struct i3c_emul_config {
	struct i3c_driver_config common;
	struct emul_list_for_bus emul_list;
	uint32_t i3c_scl_hz;
	uint32_t i2c_scl_hz;
};

static struct i3c_emul *i3c_emul_for_desc(const struct i3c_device_desc *desc)
{
	if (desc == NULL) {
		return NULL;
	}
	return (struct i3c_emul *)desc->controller_priv;
}

static struct i3c_emul *i3c_emul_find_by_addr(const struct device *dev, uint8_t addr,
					      bool is_setdasa)
{
	struct i3c_device_desc *desc;

	if (addr == 0U) {
		return NULL;
	}

	desc = is_setdasa ? i3c_dev_list_i3c_static_addr_find(dev, addr)
			  : i3c_dev_list_i3c_addr_find(dev, addr);
	return i3c_emul_for_desc(desc);
}

static int i3c_emul_configure(const struct device *dev, enum i3c_config_type type, void *config)
{
	struct i3c_emul_data *data = dev->data;

	if (type != I3C_CONFIG_CONTROLLER) {
		return -ENOTSUP;
	}

	(void)memcpy(&data->common.ctrl_config, config, sizeof(data->common.ctrl_config));

	return 0;
}

static int i3c_emul_config_get(const struct device *dev, enum i3c_config_type type, void *config)
{
	struct i3c_emul_data *data = dev->data;

	if (type != I3C_CONFIG_CONTROLLER) {
		return -ENOTSUP;
	}

	(void)memcpy(config, &data->common.ctrl_config, sizeof(data->common.ctrl_config));

	return 0;
}

static int i3c_emul_recover_bus(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int i3c_emul_reattach_i3c_device(const struct device *dev, struct i3c_device_desc *target,
					uint8_t old_dyn_addr)
{
	struct i3c_emul *emul = i3c_emul_for_desc(target);

	ARG_UNUSED(dev);
	ARG_UNUSED(old_dyn_addr);

	if (emul == NULL) {
		return 0;
	}

	emul->dynamic_addr = target->dynamic_addr;

	if (emul->api != NULL && emul->api->set_dynamic_addr != NULL) {
		return emul->api->set_dynamic_addr(emul->target, emul->dynamic_addr);
	}

	return 0;
}

static int i3c_emul_detach_i3c_device(const struct device *dev, struct i3c_device_desc *target)
{
	struct i3c_emul *emul = i3c_emul_for_desc(target);

	ARG_UNUSED(dev);

	if (emul == NULL) {
		return 0;
	}

	emul->dynamic_addr = 0U;

	if (emul->api != NULL && emul->api->set_dynamic_addr != NULL) {
		(void)emul->api->set_dynamic_addr(emul->target, 0U);
	}

	return 0;
}

static int i3c_emul_attach_i2c_device(const struct device *dev,
				      struct i3c_i2c_device_desc *target)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(target);
	return 0;
}

static int i3c_emul_detach_i2c_device(const struct device *dev,
				      struct i3c_i2c_device_desc *target)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(target);
	return 0;
}

static int i3c_emul_assign_dynamic_addr(const struct device *dev, struct i3c_device_desc *desc,
					struct i3c_emul *emul)
{
	struct i3c_emul_data *data = dev->data;
	struct i3c_addr_slots *slots = &data->common.attached_dev.addr_slots;
	uint8_t addr;

	if (desc->dynamic_addr != 0U) {
		return 0;
	}

	addr = desc->init_dynamic_addr;
	if (addr != 0U && !i3c_addr_slots_is_free(slots, addr)) {
		LOG_WRN("%s: requested dynamic addr 0x%02x for %s not free", dev->name, addr,
			desc->dev->name);
		addr = 0U;
	}

	if (addr == 0U) {
		addr = i3c_addr_slots_next_free_find(slots, 0U);
		if (addr == 0U) {
			LOG_ERR("%s: no free dynamic addresses", dev->name);
			return -ENOSPC;
		}
	}

	desc->dynamic_addr = addr;
	emul->dynamic_addr = addr;
	i3c_addr_slots_mark_i3c(slots, addr);

	if (emul->api != NULL && emul->api->set_dynamic_addr != NULL) {
		int rc = emul->api->set_dynamic_addr(emul->target, addr);

		if (rc != 0) {
			LOG_ERR("%s: %s rejected dynamic addr 0x%02x (%d)", dev->name,
				desc->dev->name, addr, rc);
			i3c_addr_slots_mark_free(slots, addr);
			desc->dynamic_addr = 0U;
			emul->dynamic_addr = 0U;
			return rc;
		}
	}

	LOG_DBG("%s: assigned dynamic addr 0x%02x to %s", dev->name, addr, desc->dev->name);
	return 0;
}

static int i3c_emul_do_daa(const struct device *dev)
{
	struct i3c_device_desc *desc;
	int ret = 0;

	I3C_BUS_FOR_EACH_I3CDEV(dev, desc) {
		struct i3c_emul *emul;

		if (desc->dynamic_addr != 0U) {
			continue;
		}

		emul = i3c_emul_for_desc(desc);
		if (emul == NULL) {
			continue;
		}

		ret = i3c_emul_assign_dynamic_addr(dev, desc, emul);
		if (ret != 0) {
			break;
		}
	}

	return ret;
}

static int i3c_emul_do_ccc_one(struct i3c_emul *emul, struct i3c_ccc_payload *payload,
			       bool is_broadcast)
{
	if (emul->mock_api != NULL && emul->mock_api->do_ccc != NULL) {
		int rc = emul->mock_api->do_ccc(emul->target, payload, is_broadcast);

		if (rc != -ENOSYS) {
			return rc;
		}
	}

	if (emul->api == NULL || emul->api->do_ccc == NULL) {
		return -ENOTSUP;
	}

	return emul->api->do_ccc(emul->target, payload, is_broadcast);
}

static void i3c_emul_post_ccc_update(struct i3c_ccc_payload *payload,
				     struct i3c_ccc_target_payload *tp, struct i3c_emul *emul)
{
	uint8_t new_addr;

	switch (payload->ccc.id) {
	case I3C_CCC_SETDASA:
	case I3C_CCC_SETNEWDA:
		if (tp->data == NULL || tp->data_len < 1U) {
			return;
		}
		new_addr = tp->data[0] >> 1;
		emul->dynamic_addr = new_addr;
		if (emul->api->set_dynamic_addr != NULL) {
			(void)emul->api->set_dynamic_addr(emul->target, new_addr);
		}
		break;
	default:
		break;
	}
}

static int i3c_emul_do_ccc_broadcast(const struct device *dev, struct i3c_ccc_payload *payload)
{
	struct i3c_device_desc *desc;
	int ret = 0;

	I3C_BUS_FOR_EACH_I3CDEV(dev, desc) {
		struct i3c_emul *emul = i3c_emul_for_desc(desc);
		int rc;

		if (emul == NULL || emul->api == NULL) {
			continue;
		}

		rc = i3c_emul_do_ccc_one(emul, payload, true);
		if (rc != 0 && rc != -ENOTSUP && ret == 0) {
			ret = rc;
		}
	}

	if (payload->ccc.id == I3C_CCC_RSTDAA) {
		/*
		 * Per-target peripheral mirror cleanup. Controller-side state
		 * (desc->dynamic_addr, address-slot map) is reset by
		 * i3c_bus_rstdaa_all() in drivers/i3c/i3c_common.c.
		 */
		I3C_BUS_FOR_EACH_I3CDEV(dev, desc) {
			struct i3c_emul *emul = i3c_emul_for_desc(desc);

			if (emul == NULL) {
				continue;
			}

			emul->dynamic_addr = 0U;
#ifdef CONFIG_I3C_USE_IBI
			emul->ibi_enabled = false;
#endif
			if (emul->api != NULL && emul->api->set_dynamic_addr != NULL) {
				(void)emul->api->set_dynamic_addr(emul->target, 0U);
			}
		}
	}

	return ret;
}

static int i3c_emul_do_ccc_direct(const struct device *dev, struct i3c_ccc_payload *payload)
{
	const bool is_setdasa = (payload->ccc.id == I3C_CCC_SETDASA);
	int ret = 0;

	for (size_t i = 0; i < payload->targets.num_targets; i++) {
		struct i3c_ccc_target_payload *tp = &payload->targets.payloads[i];
		struct i3c_emul *emul = i3c_emul_find_by_addr(dev, tp->addr, is_setdasa);
		int rc;

		if (emul == NULL) {
			ret = ret ? ret : -ENODEV;
			continue;
		}

		/* Default tp->num_xfer to 0 so callers don't read uninitialized
		 * memory if the peripheral does not touch it.
		 */
		tp->num_xfer = 0;

		rc = i3c_emul_do_ccc_one(emul, payload, false);
		if (rc == 0) {
			i3c_emul_post_ccc_update(payload, tp, emul);
		} else if (ret == 0) {
			ret = rc;
		}
	}

	return ret;
}

static int i3c_emul_do_ccc(const struct device *dev, struct i3c_ccc_payload *payload)
{
	if (payload == NULL) {
		return -EINVAL;
	}

	if (i3c_ccc_is_payload_broadcast(payload)) {
		return i3c_emul_do_ccc_broadcast(dev, payload);
	}

	return i3c_emul_do_ccc_direct(dev, payload);
}

#ifdef CONFIG_I3C_TARGET
static int i3c_emul_target_xfer(struct i3c_emul_data *data, struct i3c_msg *msgs, uint8_t num_msgs)
{
	const struct i3c_target_callbacks *cb = data->target_cfg->callbacks;
	int rc;

	for (uint8_t i = 0; i < num_msgs; i++) {
		struct i3c_msg *m = &msgs[i];

		if ((m->flags & I3C_MSG_RW_MASK) == I3C_MSG_READ) {
			uint32_t off = 0;

#if CONFIG_I3C_EMUL_TARGET_TX_FIFO_SIZE > 0
			if (data->tx_fifo_len > 0U) {
				size_t take = MIN(data->tx_fifo_len, m->len);

				memcpy(m->buf, data->tx_fifo, take);
				if (take < data->tx_fifo_len) {
					memmove(data->tx_fifo, &data->tx_fifo[take],
						data->tx_fifo_len - take);
				}
				data->tx_fifo_len -= take;
				off = take;
			}
#endif
			for (; off < m->len; off++) {
				rc = (off == 0)
					     ? (cb->read_requested_cb != NULL
							? cb->read_requested_cb(data->target_cfg,
										&m->buf[off])
							: -ENOSYS)
					     : (cb->read_processed_cb != NULL
							? cb->read_processed_cb(data->target_cfg,
										&m->buf[off])
							: -ENOSYS);
				if (rc != 0) {
					return rc;
				}
			}
			m->num_xfer = m->len;
		} else {
			for (uint32_t j = 0; j < m->len; j++) {
				if (j == 0 && cb->write_requested_cb != NULL) {
					rc = cb->write_requested_cb(data->target_cfg);
					if (rc != 0) {
						return rc;
					}
				}
				if (cb->write_received_cb != NULL) {
					rc = cb->write_received_cb(data->target_cfg, m->buf[j]);
					if (rc != 0) {
						return rc;
					}
				}
			}
			m->num_xfer = m->len;
		}

		if ((m->flags & I3C_MSG_STOP) && cb->stop_cb != NULL) {
			(void)cb->stop_cb(data->target_cfg);
		}
	}

	return 0;
}
#endif /* CONFIG_I3C_TARGET */

static int i3c_emul_xfers(const struct device *dev, struct i3c_device_desc *target,
			  struct i3c_msg *msgs, uint8_t num_msgs)
{
	struct i3c_emul *emul;

	if (target == NULL || msgs == NULL) {
		return -EINVAL;
	}

#ifdef CONFIG_I3C_TARGET
	{
		struct i3c_emul_data *data = dev->data;

		if (data->target_cfg != NULL && data->target_cfg->address == target->dynamic_addr) {
			return i3c_emul_target_xfer(data, msgs, num_msgs);
		}
	}
#endif

	emul = i3c_emul_find_by_addr(dev, target->dynamic_addr, false);
	if (emul == NULL) {
		LOG_DBG("%s: no emul for dynamic addr 0x%02x", dev->name, target->dynamic_addr);
		return -ENODEV;
	}

	if (emul->mock_api != NULL && emul->mock_api->xfers != NULL) {
		int rc = emul->mock_api->xfers(emul->target, msgs, num_msgs);

		if (rc != -ENOSYS) {
			return rc;
		}
	}

	if (emul->api == NULL || emul->api->xfers == NULL) {
		return -ENOSYS;
	}

	return emul->api->xfers(emul->target, msgs, num_msgs);
}

static struct i3c_device_desc *i3c_emul_device_find(const struct device *dev,
						    const struct i3c_device_id *id)
{
	const struct i3c_emul_config *cfg = dev->config;

	return i3c_dev_list_find(&cfg->common.dev_list, id);
}

#ifdef CONFIG_I3C_CALLBACK
static int i3c_emul_do_ccc_cb(const struct device *dev, struct i3c_ccc_payload *payload,
			      i3c_callback_t cb, void *userdata)
{
	int ret = i3c_emul_do_ccc(dev, payload);

	if (cb != NULL) {
		cb(dev, ret, userdata);
	}

	return ret;
}

static int i3c_emul_xfers_cb(const struct device *dev, struct i3c_device_desc *target,
			     struct i3c_msg *msgs, uint8_t num_msgs, i3c_callback_t cb,
			     void *userdata)
{
	int ret = i3c_emul_xfers(dev, target, msgs, num_msgs);

	if (cb != NULL) {
		cb(dev, ret, userdata);
	}

	return ret;
}
#endif /* CONFIG_I3C_CALLBACK */

#ifdef CONFIG_I3C_USE_IBI
static int i3c_emul_ibi_enable(const struct device *dev, struct i3c_device_desc *target)
{
	struct i3c_emul *emul;

	if (target == NULL) {
		return -EINVAL;
	}

	emul = i3c_emul_find_by_addr(dev, target->dynamic_addr, false);
	if (emul == NULL) {
		return -ENODEV;
	}

	emul->ibi_enabled = true;

	if (emul->api != NULL && emul->api->ibi_enable != NULL) {
		return emul->api->ibi_enable(emul->target);
	}

	return 0;
}

static int i3c_emul_ibi_disable(const struct device *dev, struct i3c_device_desc *target)
{
	struct i3c_emul *emul;

	if (target == NULL) {
		return -EINVAL;
	}

	emul = i3c_emul_find_by_addr(dev, target->dynamic_addr, false);
	if (emul == NULL) {
		return -ENODEV;
	}

	emul->ibi_enabled = false;

	if (emul->api != NULL && emul->api->ibi_disable != NULL) {
		return emul->api->ibi_disable(emul->target);
	}

	return 0;
}

static int i3c_emul_ibi_hj_response(const struct device *dev, bool ack)
{
	struct i3c_emul_data *data = dev->data;

	data->ibi_hj_ack = ack;
	return 0;
}

static int i3c_emul_ibi_crr_response(struct i3c_device_desc *target, bool ack)
{
	struct i3c_emul_data *data;

	if (target == NULL || target->bus == NULL) {
		return -EINVAL;
	}

	data = target->bus->data;
	data->ibi_crr_ack = ack;
	return 0;
}
#endif /* CONFIG_I3C_USE_IBI */

#ifdef CONFIG_I3C_TARGET
static int i3c_emul_target_register(const struct device *dev, struct i3c_target_config *cfg)
{
	struct i3c_emul_data *data = dev->data;

	if (cfg == NULL) {
		return -EINVAL;
	}

	data->target_cfg = cfg;
#if CONFIG_I3C_EMUL_TARGET_TX_FIFO_SIZE > 0
	data->tx_fifo_len = 0;
	data->tx_fifo_hdr_mode = 0;
#endif
	return 0;
}

static int i3c_emul_target_unregister(const struct device *dev, struct i3c_target_config *cfg)
{
	struct i3c_emul_data *data = dev->data;

	if (data->target_cfg != cfg) {
		return -EINVAL;
	}

	data->target_cfg = NULL;
#if CONFIG_I3C_EMUL_TARGET_TX_FIFO_SIZE > 0
	data->tx_fifo_len = 0;
	data->tx_fifo_hdr_mode = 0;
#endif
	return 0;
}

static int i3c_emul_target_tx_write(const struct device *dev, uint8_t *buf, uint16_t len,
				    uint8_t hdr_mode)
{
#if CONFIG_I3C_EMUL_TARGET_TX_FIFO_SIZE > 0
	struct i3c_emul_data *data = dev->data;
	size_t free_space = sizeof(data->tx_fifo) - data->tx_fifo_len;
	size_t take = MIN(free_space, (size_t)len);

	if (data->target_cfg == NULL) {
		return -ENOTSUP;
	}
	if (take == 0U && len > 0U) {
		return -ENOSPC;
	}

	memcpy(&data->tx_fifo[data->tx_fifo_len], buf, take);
	data->tx_fifo_len += take;
	data->tx_fifo_hdr_mode = hdr_mode;

	return (int)take;
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	ARG_UNUSED(hdr_mode);
	return -ENOTSUP;
#endif
}

static int i3c_emul_target_controller_handoff(const struct device *dev, bool accept)
{
	struct i3c_emul_data *data = dev->data;

	/*
	 * "Accept" here means: when the active controller offers controller
	 * role to this device (via GETACCCR), the application is willing
	 * to take it. is_secondary is the boot-time role (Primary vs
	 * Secondary controller) and stays as initialized; it is NOT a
	 * "currently active" flag.
	 */
	data->handoff_accept = accept;

	if (accept && data->target_cfg != NULL &&
	    data->target_cfg->callbacks != NULL &&
	    data->target_cfg->callbacks->controller_handoff_cb != NULL) {
		(void)data->target_cfg->callbacks->controller_handoff_cb(data->target_cfg);
	}

	return 0;
}
#endif /* CONFIG_I3C_TARGET */

static int i3c_emul_i2c_api_configure(const struct device *dev, uint32_t dev_config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(dev_config);
	return 0;
}

static int i3c_emul_i2c_api_transfer(const struct device *dev, struct i2c_msg *msgs,
				     uint8_t num_msgs, uint16_t addr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(msgs);
	ARG_UNUSED(num_msgs);
	ARG_UNUSED(addr);
	return -ENOSYS;
}

int i3c_emul_register(const struct device *dev, struct i3c_emul *emul)
{
	struct i3c_device_desc *desc;

	emul->bus = dev;

	/*
	 * Link the peripheral emulator to the matching i3c_device_desc via
	 * controller_priv so the bus emulator can dispatch using the standard
	 * I3C_BUS_FOR_EACH_I3CDEV iteration without keeping its own list.
	 */
	I3C_BUS_FOR_EACH_I3CDEV(dev, desc) {
		if (desc->dev == emul->target->dev) {
			desc->controller_priv = emul;
			break;
		}
	}

	LOG_INF("%s: register I3C emul '%s' (sa=0x%02x, pid=0x%012llx)", dev->name,
		emul->target->dev->name, emul->static_addr, (unsigned long long)emul->pid);

	return 0;
}

int i3c_emul_i2c_register(const struct device *dev, struct i3c_i2c_emul *emul)
{
	struct i3c_i2c_device_desc *desc;

	I3C_BUS_FOR_EACH_I2CDEV(dev, desc) {
		if (desc->addr == emul->addr) {
			desc->controller_priv = emul;
			break;
		}
	}

	LOG_INF("%s: register I2C-on-I3C emul '%s' (addr=0x%02x)", dev->name,
		emul->target->dev->name, emul->addr);

	return 0;
}

int i3c_emul_target_raise_ibi(const struct emul *target, uint8_t *payload, uint8_t payload_len)
{
#ifdef CONFIG_I3C_USE_IBI
	const struct device *bus;
	struct i3c_emul *emul;
	struct i3c_device_desc *desc;

	if (target == NULL || target->bus_type != EMUL_BUS_TYPE_I3C) {
		return -EINVAL;
	}

	emul = target->bus.i3c;
	if (emul == NULL || emul->bus == NULL || emul->dynamic_addr == 0U) {
		return -EINVAL;
	}

	if (!emul->ibi_enabled) {
		return -ENOTCONN;
	}

	bus = emul->bus;
	desc = i3c_dev_list_i3c_addr_find(bus, emul->dynamic_addr);
	if (desc == NULL || desc->ibi_cb == NULL) {
		return -EINVAL;
	}

#ifdef CONFIG_I3C_IBI_WORKQUEUE
	return i3c_ibi_work_enqueue_target_irq(desc, payload, payload_len);
#else
	{
		struct i3c_ibi_payload ibi_payload = {0};

		if (payload_len > sizeof(ibi_payload.payload)) {
			payload_len = sizeof(ibi_payload.payload);
		}
		if ((payload != NULL) && (payload_len > 0U)) {
			(void)memcpy(ibi_payload.payload, payload, payload_len);
		}
		ibi_payload.payload_len = payload_len;

		return desc->ibi_cb(desc, payload_len > 0U ? &ibi_payload : NULL);
	}
#endif /* CONFIG_I3C_IBI_WORKQUEUE */
#else
	ARG_UNUSED(target);
	ARG_UNUSED(payload);
	ARG_UNUSED(payload_len);
	return -ENOSYS;
#endif /* CONFIG_I3C_USE_IBI */
}

int i3c_emul_target_raise_hj(const struct emul *target)
{
#ifdef CONFIG_I3C_USE_IBI
	struct i3c_emul *emul;
	struct i3c_emul_data *data;

	if (target == NULL || target->bus_type != EMUL_BUS_TYPE_I3C) {
		return -EINVAL;
	}

	emul = target->bus.i3c;
	if (emul == NULL || emul->bus == NULL) {
		return -EINVAL;
	}

	data = emul->bus->data;
	if (!data->ibi_hj_ack) {
		return -ENOTCONN;
	}

#ifdef CONFIG_I3C_IBI_WORKQUEUE
	return i3c_ibi_work_enqueue_hotjoin(emul->bus);
#else
	return 0;
#endif
#else
	ARG_UNUSED(target);
	return -ENOSYS;
#endif
}

int i3c_emul_target_raise_crr(const struct emul *target)
{
#ifdef CONFIG_I3C_USE_IBI
	struct i3c_emul_data *data;
	struct i3c_emul *emul;
	struct i3c_device_desc *desc;

	if (target == NULL || target->bus_type != EMUL_BUS_TYPE_I3C) {
		return -EINVAL;
	}

	emul = target->bus.i3c;
	if (emul == NULL || emul->bus == NULL || emul->dynamic_addr == 0U) {
		return -EINVAL;
	}

	data = emul->bus->data;
	if (!data->ibi_crr_ack) {
		return -ENOTCONN;
	}

	desc = i3c_dev_list_i3c_addr_find(emul->bus, emul->dynamic_addr);
	if (desc == NULL) {
		return -EINVAL;
	}

#ifdef CONFIG_I3C_IBI_WORKQUEUE
	return i3c_ibi_work_enqueue_controller_request(desc);
#else
	return 0;
#endif
#else
	ARG_UNUSED(target);
	return -ENOSYS;
#endif
}

static int i3c_emul_init(const struct device *dev)
{
	const struct i3c_emul_config *cfg = dev->config;
	struct i3c_emul_data *data = dev->data;
	int ret;

	data->common.ctrl_config.scl.i3c = cfg->i3c_scl_hz;
	data->common.ctrl_config.scl.i2c = cfg->i2c_scl_hz;

	/*
	 * Attach all DT-listed targets first so the standard I3C device list
	 * is populated, then walk EMUL_DT_DEFINE peripherals so each
	 * i3c_emul_register can link itself into the matching desc via
	 * desc->controller_priv.
	 */
	ret = i3c_addr_slots_init(dev);
	if (ret != 0) {
		return ret;
	}

	ret = emul_init_for_bus_from_list(dev, &cfg->emul_list);
	if (ret != 0) {
		return ret;
	}

	if ((cfg->common.flags & I3C_CONTROLLER_FLAG_DISABLE_BUS_INIT) == 0U) {
		ret = i3c_bus_init(dev, &cfg->common.dev_list);
		if (ret != 0) {
			LOG_WRN("%s: i3c_bus_init returned %d", dev->name, ret);
			ret = 0;
		}
	}

	return ret;
}

static DEVICE_API(i3c, i3c_emul_api) = {
	.i2c_api = {
		.configure = i3c_emul_i2c_api_configure,
		.transfer = i3c_emul_i2c_api_transfer,
	},
	.configure = i3c_emul_configure,
	.config_get = i3c_emul_config_get,
	.recover_bus = i3c_emul_recover_bus,
	.reattach_i3c_device = i3c_emul_reattach_i3c_device,
	.detach_i3c_device = i3c_emul_detach_i3c_device,
	.attach_i2c_device = i3c_emul_attach_i2c_device,
	.detach_i2c_device = i3c_emul_detach_i2c_device,
	.do_daa = i3c_emul_do_daa,
	.do_ccc = i3c_emul_do_ccc,
	.i3c_xfers = i3c_emul_xfers,
	.i3c_device_find = i3c_emul_device_find,
#ifdef CONFIG_I3C_CALLBACK
	.do_ccc_cb = i3c_emul_do_ccc_cb,
	.i3c_xfers_cb = i3c_emul_xfers_cb,
#endif
#ifdef CONFIG_I3C_USE_IBI
	.ibi_enable = i3c_emul_ibi_enable,
	.ibi_disable = i3c_emul_ibi_disable,
	.ibi_hj_response = i3c_emul_ibi_hj_response,
	.ibi_crr_response = i3c_emul_ibi_crr_response,
#endif
#ifdef CONFIG_I3C_TARGET
	.target_register = i3c_emul_target_register,
	.target_unregister = i3c_emul_target_unregister,
	.target_tx_write = i3c_emul_target_tx_write,
	.target_controller_handoff = i3c_emul_target_controller_handoff,
#endif
#ifdef CONFIG_I3C_RTIO
	.iodev_submit = i3c_iodev_submit_fallback,
#endif
};

#define EMUL_LINK_AND_COMMA(node_id)                                                               \
	{                                                                                          \
		.dev = DEVICE_DT_GET(node_id),                                                     \
	},

#define I3C_EMUL_INIT(n)                                                                           \
	static struct i3c_device_desc i3c_emul_i3c_dev_arr_##n[] = I3C_DEVICE_ARRAY_DT_INST(n);    \
	static struct i3c_i2c_device_desc i3c_emul_i2c_dev_arr_##n[] =                             \
		I3C_I2C_DEVICE_ARRAY_DT_INST(n);                                                   \
	static const struct emul_link_for_bus i3c_emul_children_##n[] = {                          \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(n, EMUL_LINK_AND_COMMA)};                        \
	static const struct i3c_emul_config i3c_emul_cfg_##n = {                                   \
		.common =                                                                          \
			{                                                                          \
				.dev_list =                                                        \
					{                                                          \
						.i3c = i3c_emul_i3c_dev_arr_##n,                   \
						.num_i3c = ARRAY_SIZE(i3c_emul_i3c_dev_arr_##n),   \
						.i2c = i3c_emul_i2c_dev_arr_##n,                   \
						.num_i2c = ARRAY_SIZE(i3c_emul_i2c_dev_arr_##n),   \
					},                                                         \
				.flags = I3C_CONTROLLER_CONFIG_FLAGS_DT_INST(n),                   \
			},                                                                         \
		.emul_list =                                                                       \
			{                                                                          \
				.children = i3c_emul_children_##n,                                 \
				.num_children = ARRAY_SIZE(i3c_emul_children_##n),                 \
			},                                                                         \
		.i3c_scl_hz = DT_INST_PROP_OR(n, i3c_scl_hz, 0),                                   \
		.i2c_scl_hz = DT_INST_PROP_OR(n, i2c_scl_hz, 0),                                   \
	};                                                                                         \
	static struct i3c_emul_data i3c_emul_data_##n;                                             \
	DEVICE_DT_INST_DEFINE(n, i3c_emul_init, NULL, &i3c_emul_data_##n, &i3c_emul_cfg_##n,       \
			      POST_KERNEL, CONFIG_I3C_CONTROLLER_INIT_PRIORITY, &i3c_emul_api);

DT_INST_FOREACH_STATUS_OKAY(I3C_EMUL_INIT)
