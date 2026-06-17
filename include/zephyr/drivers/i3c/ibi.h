/*
 * Copyright 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I3C_IBI_H_
#define ZEPHYR_INCLUDE_DRIVERS_I3C_IBI_H_

/**
 * @brief I3C In-Band Interrupts
 * @defgroup i3c_ibi I3C In-Band Interrupts
 * @ingroup i3c_interface
 * @{
 */

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/slist.h>

#ifndef CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE
#define CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct i3c_device_desc;

/**
 * @brief IBI Types.
 */
enum i3c_ibi_type {
	/** Target interrupt */
	I3C_IBI_TARGET_INTR,

	/** Controller Role Request */
	I3C_IBI_CONTROLLER_ROLE_REQUEST,

	/** Hot Join Request */
	I3C_IBI_HOTJOIN,

	I3C_IBI_TYPE_MAX = I3C_IBI_HOTJOIN,

	/**
	 * Not an actual IBI type, but simply used by
	 * the IBI workq for generic callbacks.
	 */
	I3C_IBI_WORKQUEUE_CB,
};

/**
 * @brief Struct for IBI request.
 */
struct i3c_ibi {
	/** Type of IBI. */
	enum i3c_ibi_type	ibi_type;

	/** Pointer to payload of IBI. */
	uint8_t			*payload;

	/** Length in bytes of the IBI payload. */
	uint8_t			payload_len;
};

/**
 * @brief Structure of payload buffer for IBI.
 *
 * This is used for the IBI callback.
 */
struct i3c_ibi_payload {
	/**
	 * Length of available data in the payload buffer.
	 */
	uint8_t payload_len;

	/**
	 * Pointer to byte array as payload buffer.
	 */
	uint8_t payload[CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE];
};

/**
 * @brief Function called when In-Band Interrupt received from target device.
 *
 * This function is invoked by the controller when the controller
 * receives an In-Band Interrupt from the target device.
 *
 * A success return shall cause the controller to ACK the next byte
 * received.  An error return shall cause the controller to NACK the
 * next byte received.
 *
 * @param target the device description structure associated with the
 *               device to which the operation is addressed.
 * @param payload Payload associated with the IBI. NULL if there is
 *                no payload.
 *
 * @return 0 if the IBI is accepted, or a negative error code.
 */
typedef int (*i3c_target_ibi_cb_t)(struct i3c_device_desc *target,
				   struct i3c_ibi_payload *payload);

/**
 * @brief Controller callback used to defer driver-specific IBI handling.
 *
 * Used with i3c_ibi_submit_cb() when a controller needs to run driver-specific
 * processing for an IBI (for example reading the IBI payload from hardware or
 * performing a controllership handoff).
 *
 * @param dev Controller device driver instance.
 */
typedef void (*i3c_ibi_cb_t)(const struct device *dev);


/**
 * @brief Submit a received target interrupt IBI.
 *
 * Called by a controller driver, typically from its ISR, when a target
 * interrupt IBI (and any payload it carried) has been received. The IBI is
 * dispatched to the target's registered @ref i3c_target_ibi_cb_t.
 *
 * The dispatch context depends on the selected IBI backend:
 * - @kconfig{CONFIG_I3C_IBI_WORKQUEUE}: the payload is copied and the callback
 *   is deferred to the IBI workqueue thread, where it may block.
 * - @kconfig{CONFIG_I3C_IBI_RTIO}: the callback is invoked directly in the
 *   caller's context (e.g. the controller ISR). It must be ISR safe and
 *   non blocking.
 *
 * @param target Pointer to target device descriptor.
 * @param payload Pointer to IBI payload byte array, or NULL if there is none.
 * @param payload_len Length of payload byte array.
 *
 * @retval 0 If the IBI was successfully queued (workqueue backend) or the
 *           callback accepted it (RTIO backend).
 * @retval -ENOMEM If there is no free internal node to store the IBI
 *                 (workqueue backend).
 * @retval -ENODEV If the target has no registered callback (RTIO backend).
 * @retval Others Value returned by @ref k_work_submit_to_queue or by the
 *                target callback.
 */
int i3c_ibi_submit_target_irq(struct i3c_device_desc *target,
				    uint8_t *payload, size_t payload_len);

/**
 * @brief Submit a received controllership request IBI.
 *
 * Called by a controller driver when a target requests the controller role.
 * The handoff (@ref i3c_device_controller_handoff) performs blocking bus
 * operations and is therefore always run in thread context: the IBI workqueue
 * thread (@kconfig{CONFIG_I3C_IBI_WORKQUEUE}) or the RTIO work queue
 * (@kconfig{CONFIG_I3C_IBI_RTIO}).
 *
 * @param target Pointer to target device descriptor.
 *
 * @retval 0 If the request was successfully submitted.
 * @retval -ENOMEM If there is no free internal node to store the IBI.
 * @retval Others @see k_work_submit_to_queue
 */
int i3c_ibi_submit_controller_request(struct i3c_device_desc *target);

/**
 * @brief Submit a received hot join IBI.
 *
 * Called by a controller driver when a hot join request is received. The
 * resulting Dynamic Address Assignment (and, on a bus with a secondary
 * controller, the DEFTGTS refresh) performs blocking bus operations and is run
 * in thread context: the IBI workqueue thread
 * (@kconfig{CONFIG_I3C_IBI_WORKQUEUE}) or the RTIO work queue
 * (@kconfig{CONFIG_I3C_IBI_RTIO}).
 *
 * @param dev Pointer to controller device driver instance.
 *
 * @retval 0 If the hot join was successfully submitted.
 * @retval -ENOMEM If there is no free internal node to store the IBI.
 * @retval Others @see k_work_submit_to_queue
 */
int i3c_ibi_submit_hotjoin(const struct device *dev);

/**
 * @brief Submit a controller callback for deferred processing.
 *
 * Used by controller drivers to run driver specific IBI handling that needs
 * thread context, such as reading the IBI payload from hardware or completing
 * a controllership handoff. The callback is invoked with @p dev from the IBI
 * workqueue thread (@kconfig{CONFIG_I3C_IBI_WORKQUEUE}) or the RTIO work queue
 * (@kconfig{CONFIG_I3C_IBI_RTIO}).
 *
 * @param dev Pointer to controller device driver instance.
 * @param cb Callback function invoked with @p dev.
 *
 * @retval 0 If the callback was successfully submitted.
 * @retval -ENOMEM If there is no free internal node to store the IBI.
 * @retval Others @see k_work_submit_to_queue
 */
int i3c_ibi_submit_cb(const struct device *dev,
			    i3c_ibi_cb_t cb);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_I3C_IBI_H_ */
