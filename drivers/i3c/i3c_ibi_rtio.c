/*
 * Copyright (c) 2025 Zephyr contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * RTIO IBI backend.
 *
 * Target interrupt IBIs are dispatched directly in the caller's context
 * (typically the controller ISR). This lets an RTIO based target start its
 * read pipeline straight from the interrupt without a workqueue hop, which is
 * the whole point of RTIO. The registered i3c_target_ibi_cb_t therefore runs
 * in interrupt context and must be ISR safe and non blocking (it cannot use
 * the -EBUSY retry behaviour offered by the workqueue backend).
 *
 * Hot-join Dynamic Address Assignment is a bus operation, so it is issued as an
 * RTIO_OP_I3C_DAA submission bound to the controller. Like any i3c RTIO
 * submission it flows through i3c_iodev_submit() to the controller's
 * iodev_submit handler, which executes it natively or via the default
 * (work-queue) fallback. A chained callback refreshes DEFTGTS once the DAA
 * completes.
 *
 * TODO: controllership request (i3c_device_controller_handoff) and the generic
 * controller callback (i3c_ibi_submit_cb) are not i3c bus operations, so they
 * do not map onto an iodev submission. For now they are run from the system
 * work queue; how they should be expressed in the RTIO model is still open.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/rtio/rtio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(i3c, CONFIG_I3C_LOG_LEVEL);

/*
 * Per hot-join submission state. Holds the i3c iodev bound to the requesting
 * controller so the DAA submission routes through that controller's
 * iodev_submit. Freed by the chained completion callback.
 */
struct i3c_ibi_rtio_hotjoin {
	sys_snode_t node;
	struct rtio_iodev iodev;
	struct i3c_iodev_data iodev_data;
};

/* Deferred non-bus control work (controllership request / controller cb). */
struct i3c_ibi_rtio_ctrl {
	sys_snode_t node;
	struct k_work work;
	const struct device *controller;
	struct i3c_device_desc *target;
	i3c_ibi_cb_t cb;
};

static struct i3c_ibi_rtio_hotjoin i3c_ibi_rtio_hotjoins[CONFIG_I3C_IBI_RTIO_LENGTH];
static struct i3c_ibi_rtio_ctrl i3c_ibi_rtio_ctrls[CONFIG_I3C_IBI_RTIO_LENGTH];
static sys_slist_t i3c_ibi_rtio_hotjoins_free;
static sys_slist_t i3c_ibi_rtio_ctrls_free;
static struct k_spinlock i3c_ibi_rtio_lock;

/* DAA needs two SQEs (the op plus its chained completion callback). */
RTIO_DEFINE(i3c_ibi_rtio_ctx, 2 * CONFIG_I3C_IBI_RTIO_LENGTH, CONFIG_I3C_IBI_RTIO_LENGTH);

static void *i3c_ibi_rtio_node_get(sys_slist_t *list)
{
	k_spinlock_key_t key = k_spin_lock(&i3c_ibi_rtio_lock);
	sys_snode_t *node = sys_slist_get(list);

	k_spin_unlock(&i3c_ibi_rtio_lock, key);

	return node;
}

static void i3c_ibi_rtio_node_put(sys_slist_t *list, sys_snode_t *node)
{
	k_spinlock_key_t key = k_spin_lock(&i3c_ibi_rtio_lock);

	sys_slist_append(list, node);
	k_spin_unlock(&i3c_ibi_rtio_lock, key);
}

/*
 * Chained after the hot-join DAA. Runs in the DAA completion context (the
 * controller's, e.g. the RTIO work queue for the default fallback) and always
 * runs, even if the DAA errored, so it can release the submission state.
 */
static void i3c_ibi_rtio_hotjoin_done(struct rtio *r, const struct rtio_sqe *sqe,
				      int result, void *arg0)
{
	struct i3c_ibi_rtio_hotjoin *hj = arg0;
	const struct device *dev = hj->iodev_data.bus;

	ARG_UNUSED(r);
	ARG_UNUSED(sqe);

	if (result < 0) {
		LOG_ERR("hot-join DAA failed: %d", result);
	}
#ifdef CONFIG_I3C_TARGET
	else if (i3c_bus_has_sec_controller(dev) && (i3c_bus_deftgts(dev) != 0)) {
		LOG_ERR("Error sending DEFTGTS");
	}
#endif /* CONFIG_I3C_TARGET */

	i3c_ibi_rtio_node_put(&i3c_ibi_rtio_hotjoins_free, &hj->node);
}

/* System work queue handler for the non-bus control work. */
static void i3c_ibi_rtio_ctrl_handler(struct k_work *work)
{
	struct i3c_ibi_rtio_ctrl *ctrl = CONTAINER_OF(work, struct i3c_ibi_rtio_ctrl, work);

	if (ctrl->cb != NULL) {
		ctrl->cb(ctrl->controller);
	}
#ifdef CONFIG_I3C_TARGET
	else if (ctrl->target != NULL) {
		if (i3c_device_controller_handoff(ctrl->target, true) != 0) {
			LOG_ERR("i3c_device_controller_handoff failed");
		}
	}
#endif /* CONFIG_I3C_TARGET */

	i3c_ibi_rtio_node_put(&i3c_ibi_rtio_ctrls_free, &ctrl->node);
}

int i3c_ibi_submit_target_irq(struct i3c_device_desc *target,
			      uint8_t *payload, size_t payload_len)
{
	struct i3c_ibi_payload pl;

	if (target->ibi_cb == NULL) {
		LOG_ERR("No IBI callback for target %s", target->dev->name);
		return -ENODEV;
	}

	if ((payload == NULL) || (payload_len == 0U)) {
		return target->ibi_cb(target, NULL);
	}

	if (payload_len > sizeof(pl.payload)) {
		payload_len = sizeof(pl.payload);
	}

	pl.payload_len = payload_len;
	(void)memcpy(&pl.payload[0], payload, payload_len);

	return target->ibi_cb(target, &pl);
}

int i3c_ibi_submit_hotjoin(const struct device *dev)
{
	struct i3c_ibi_rtio_hotjoin *hj;
	struct rtio_sqe *daa;
	struct rtio_sqe *done;

	hj = i3c_ibi_rtio_node_get(&i3c_ibi_rtio_hotjoins_free);
	if (hj == NULL) {
		return -ENOMEM;
	}

	/* Bind an i3c iodev to the requesting controller for this submission. */
	hj->iodev_data.bus = dev;
	hj->iodev.api = &i3c_iodev_api;
	hj->iodev.data = &hj->iodev_data;

	daa = rtio_sqe_acquire(&i3c_ibi_rtio_ctx);
	done = rtio_sqe_acquire(&i3c_ibi_rtio_ctx);
	if ((daa == NULL) || (done == NULL)) {
		rtio_sqe_drop_all(&i3c_ibi_rtio_ctx);
		i3c_ibi_rtio_node_put(&i3c_ibi_rtio_hotjoins_free, &hj->node);
		return -ENOMEM;
	}

	/*
	 * Run the DAA through the controller's iodev_submit, then refresh
	 * DEFTGTS from the chained callback once it completes.
	 */
	daa->op = RTIO_OP_I3C_DAA;
	daa->prio = 0;
	daa->flags = RTIO_SQE_CHAINED | RTIO_SQE_NO_RESPONSE;
	daa->iodev = &hj->iodev;
	daa->userdata = NULL;

	rtio_sqe_prep_callback_no_cqe(done, i3c_ibi_rtio_hotjoin_done, hj, NULL);

	rtio_submit(&i3c_ibi_rtio_ctx, 0);

	return 0;
}

int i3c_ibi_submit_controller_request(struct i3c_device_desc *target)
{
	struct i3c_ibi_rtio_ctrl *ctrl = i3c_ibi_rtio_node_get(&i3c_ibi_rtio_ctrls_free);

	if (ctrl == NULL) {
		return -ENOMEM;
	}

	ctrl->controller = NULL;
	ctrl->target = target;
	ctrl->cb = NULL;
	k_work_submit(&ctrl->work);

	return 0;
}

int i3c_ibi_submit_cb(const struct device *dev, i3c_ibi_cb_t cb)
{
	struct i3c_ibi_rtio_ctrl *ctrl = i3c_ibi_rtio_node_get(&i3c_ibi_rtio_ctrls_free);

	if (ctrl == NULL) {
		return -ENOMEM;
	}

	ctrl->controller = dev;
	ctrl->target = NULL;
	ctrl->cb = cb;
	k_work_submit(&ctrl->work);

	return 0;
}

static int i3c_ibi_rtio_init(void)
{
	sys_slist_init(&i3c_ibi_rtio_hotjoins_free);
	sys_slist_init(&i3c_ibi_rtio_ctrls_free);

	for (int i = 0; i < ARRAY_SIZE(i3c_ibi_rtio_hotjoins); i++) {
		sys_slist_append(&i3c_ibi_rtio_hotjoins_free, &i3c_ibi_rtio_hotjoins[i].node);
	}

	for (int i = 0; i < ARRAY_SIZE(i3c_ibi_rtio_ctrls); i++) {
		i3c_ibi_rtio_ctrls[i].work.handler = i3c_ibi_rtio_ctrl_handler;
		sys_slist_append(&i3c_ibi_rtio_ctrls_free, &i3c_ibi_rtio_ctrls[i].node);
	}

	return 0;
}

SYS_INIT(i3c_ibi_rtio_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
