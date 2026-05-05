/**
 * @file
 *
 * @brief Public APIs for the I3C emulation drivers.
 */

/*
 * Copyright (c) 2026 Ryan McClelland
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_I3C_EMUL_H_
#define ZEPHYR_INCLUDE_DRIVERS_I3C_EMUL_H_

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/types.h>

/**
 * @brief I3C Emulation Interface
 * @defgroup i3c_emul_interface I3C Emulation Interface
 * @ingroup io_emulators
 * @ingroup i3c_interface
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

struct i3c_emul_api;

/**
 * Node in a linked list of emulators for I3C target devices on an emulated bus.
 *
 * The peripheral emulator typically embeds this struct in its own data
 * structure, fills in @ref static_addr / @ref pid / @ref bcr / @ref dcr from
 * its devicetree node, then attaches via @ref i3c_emul_register.
 */
struct i3c_emul {
	/** Target emulator backing this bus node. REQUIRED. */
	const struct emul *target;

	/** API provided by the peripheral emulator. */
	const struct i3c_emul_api *api;

	/**
	 * Optional mock API. If non-NULL, the bus emulator routes operations
	 * through these callbacks before falling back to @ref api.  Returning
	 * @c -ENOSYS from a mock callback delegates to the real @ref api.
	 */
	struct i3c_emul_api *mock_api;

	/**
	 * Bus emulator controller device this peripheral is attached to.
	 * Populated by @ref i3c_emul_register so that the peripheral side
	 * (e.g. @ref i3c_emul_target_raise_ibi) can locate the bus without
	 * external state.
	 */
	const struct device *bus;

	/**
	 * Static address from devicetree (first cell of @c reg). May be 0 when
	 * the device only obtains an address through DAA.
	 */
	uint8_t static_addr;

	/**
	 * Dynamic address currently assigned by the controller. Set by the bus
	 * emulator during DAA / SETDASA / SETNEWDA, cleared by RSTDAA.
	 */
	uint8_t dynamic_addr;

	/** 48-bit Provisioned ID. */
	uint64_t pid;

	/** Bus Characteristic Register (initial value). */
	uint8_t bcr;

	/** Device Characteristic Register. */
	uint8_t dcr;

	/**
	 * IBI ACK state, managed by the bus emulator.  Set when the controller
	 * calls @c ibi_enable for this target, cleared by @c ibi_disable or
	 * RSTDAA.  When @c false, IBIs raised via
	 * @ref i3c_emul_target_raise_ibi are NACKed.
	 */
	bool ibi_enabled;
};

/**
 * Node in a linked list of legacy I2C target emulators attached to an
 * emulated I3C bus (i.e. mixed-bus operation).
 */
struct i3c_i2c_emul {
	/** Target emulator backing this bus node. REQUIRED. */
	const struct emul *target;

	/** API provided by the peripheral emulator. */
	const struct i3c_emul_api *api;

	/** 7-bit I2C address. */
	uint16_t addr;

	/** Legacy Virtual Register (LVR). */
	uint8_t lvr;
};

/**
 * Issue private SDR (or HDR) read/write transfers to a target emulator.
 *
 * @param target  The peripheral emulator instance.
 * @param msgs    Array of I3C messages.  For read messages, the callback fills
 *                @c msgs[i].buf and updates @c msgs[i].num_xfer.
 * @param num_msgs Number of messages in the array.
 *
 * @retval 0 on success.
 * @retval -EIO on bus error.
 */
typedef int (*i3c_emul_xfers_t)(const struct emul *target, struct i3c_msg *msgs, uint8_t num_msgs);

/**
 * Deliver a Common Command Code (CCC) to a target emulator.
 *
 * The bus emulator dispatches broadcast CCCs to every attached target with
 * @c is_broadcast = true. For direct CCCs the bus emulator walks
 * @c payload->targets.payloads[] and invokes this callback once per addressed
 * target with @c is_broadcast = false; the per-target payload to act on is the
 * matching entry in @c payload->targets.payloads[].
 *
 * @param target       The peripheral emulator instance.
 * @param payload      The CCC payload (id, optional defining byte, target list).
 * @param is_broadcast True if invoked as part of a broadcast CCC dispatch.
 *
 * @retval 0 on success.
 * @retval -ENOTSUP if the target does not implement this CCC.
 * @retval -EIO on protocol error.
 */
typedef int (*i3c_emul_do_ccc_t)(const struct emul *target, struct i3c_ccc_payload *payload,
				 bool is_broadcast);

/**
 * Inform a target emulator that the controller has assigned (or cleared) its
 * dynamic address.  Called by the bus emulator during DAA, SETDASA, SETNEWDA,
 * and RSTDAA processing so the peripheral can keep its internal mirror in sync.
 *
 * @param target      The peripheral emulator instance.
 * @param dynamic_addr New dynamic address.  Zero indicates the address was
 *                    revoked (RSTDAA).
 *
 * @retval 0 on success.
 * @retval -EINVAL if the address is rejected by the peripheral.
 */
typedef int (*i3c_emul_set_dynamic_addr_t)(const struct emul *target, uint8_t dynamic_addr);

/** IBI enable/disable notification to a target emulator. */
typedef int (*i3c_emul_ibi_enable_t)(const struct emul *target);
typedef int (*i3c_emul_ibi_disable_t)(const struct emul *target);

/**
 * Per-target API exposed by an I3C peripheral emulator to the bus emulator.
 *
 * Mandatory members: @ref xfers and @ref do_ccc. All other callbacks may be
 * @c NULL and the bus emulator will treat them as no-ops returning 0
 * (or @c -ENOTSUP for IBI when the target is not IBI-capable).
 */
struct i3c_emul_api {
	i3c_emul_xfers_t xfers;
	i3c_emul_do_ccc_t do_ccc;
	i3c_emul_set_dynamic_addr_t set_dynamic_addr;
	i3c_emul_ibi_enable_t ibi_enable;
	i3c_emul_ibi_disable_t ibi_disable;
	/** Optional. If NULL, bus emulator routes HDR-DDR through @ref xfers. */
	i3c_emul_xfers_t hdr_ddr_xfers;
};

/**
 * Register an I3C target peripheral emulator with the bus emulator.
 *
 * Typically invoked from the peripheral emulator's @ref emul_init_t callback
 * via @c emul_init_for_bus.
 *
 * @param dev  The bus emulator device.
 * @param emul The peripheral's @ref i3c_emul node.
 *
 * @retval 0 always.
 */
int i3c_emul_register(const struct device *dev, struct i3c_emul *emul);

/**
 * Register a legacy I2C target on the emulated I3C bus (mixed-bus operation).
 *
 * @param dev  The bus emulator device.
 * @param emul The peripheral's @ref i3c_i2c_emul node.
 *
 * @retval 0 always.
 */
int i3c_emul_i2c_register(const struct device *dev, struct i3c_i2c_emul *emul);

/**
 * Inject an in-band interrupt (IBI) from a peripheral emulator into the bus.
 *
 * The bus emulator ACKs the IBI only if the controller has previously enabled
 * IBI for this target.  On ACK the corresponding @c i3c_device_desc->ibi_cb
 * is invoked (directly, or via the IBI workqueue when
 * @kconfig{CONFIG_I3C_IBI_WORKQUEUE} is enabled).
 *
 * @param target      The peripheral emulator instance.
 * @param payload     IBI payload bytes (may be NULL when @c payload_len == 0).
 * @param payload_len Length of the payload in bytes.
 *
 * @retval 0 on ACK + delivery.
 * @retval -ENOTCONN if IBI is disabled for this target.
 * @retval -EINVAL if the target is not attached to a controller.
 */
int i3c_emul_target_raise_ibi(const struct emul *target, uint8_t *payload, uint8_t payload_len);

/**
 * Inject a hot-join request from a peripheral emulator.
 *
 * The bus emulator ACKs the request iff the controller previously called
 * @c ibi_hj_response(true).  On ACK, application code may then trigger a fresh
 * @c i3c_do_daa() to attach the device.
 *
 * @retval 0 on ACK.
 * @retval -ENOTCONN if hot-join is currently NACKed by the controller.
 */
int i3c_emul_target_raise_hj(const struct emul *target);

/**
 * Inject a controller-role-request (CRR) from a secondary-controller-capable
 * target emulator.
 *
 * @retval 0 on ACK.
 * @retval -ENOTCONN if CRR is currently NACKed by the controller.
 */
int i3c_emul_target_raise_crr(const struct emul *target);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_I3C_EMUL_H_ */
