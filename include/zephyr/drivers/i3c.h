/*
 * Copyright 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I3C_H_
#define ZEPHYR_INCLUDE_DRIVERS_I3C_H_

/**
 * @brief I3C Controller Interface
 * @defgroup i3c_controller_interface I3C Controller Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <zephyr/device.h>

#include <zephyr/drivers/i3c/addresses.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE
#define CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE 0
#endif

/*
 * Bus Characteristic Register (BCR)
 * - BCR[7:6]: Device Role
 *   - 0: I3C Target
 *   - 1: I3C Controller capable
 *   - 2: Reserved
 *   - 3: Reserved
 * - BCR[5]: Advanced Capabilities
 *   - 0: Does not support optional advanced capabilities.
 *   - 1: Supports optional advanced capabilities which
 *        can be viewed via GETCAPS CCC.
 * - BCR[4}: Virtual Target Support
 *   - 0: Is not a virtual target.
 *   - 1: Is a virtual target.
 * - BCR[3]: Offline Capable
 *   - 0: Will always response to I3C commands.
 *   - 1: Will not always response to I3C commands.
 * - BCR[2]: IBI Payload
 *   - 0: No data bytes following the accepted IBI.
 *   - 1: One data byte (MDB, Mandatory Data Byte) follows
 *        the accepted IBI. Additional data bytes may also
 *        follows.
 * - BCR[1]: IBI Request Capable
 *   - 0: Not capable
 *   - 1: Capable
 * - BCR[0]: Max Data Speed Limitation
 *   - 0: No Limitation
 *   - 1: Limitation obtained via GETMXDS CCC.
 */
#define I3C_BCR_MAX_DATA_SPEED_LIMIT			BIT(0)
#define I3C_BCR_IBI_REQUEST_CAPABLE			BIT(1)
#define I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE		BIT(2)
#define I3C_BCR_OFFLINE_CAPABLE				BIT(3)
#define I3C_BCR_VIRTUAL_TARGET				BIT(4)
#define I3C_BCR_ADV_CAPABILITIES			BIT(5)

#define I3C_BCR_DEVICE_ROLE_I3C_TARGET			0U
#define I3C_BCR_DEVICE_ROLE_I3C_CONTROLLER_CAPABLE	1U

#define I3C_BCR_DEVICE_ROLE_SHIFT			6U
#define I3C_BCR_DEVICE_ROLE_MASK			(0x03U << I3C_BCR_DEVICE_ROLE_SHIFT)

#define I3C_BCR_DEVICE_ROLE(bcr)			\
	(((bcr) & I3C_BCR_DEVICE_ROLE_MASK) >> I3C_BCR_DEVICE_ROLE_SHIFT)

/*
 * Legacy Virtual Register (LVR)
 * - LVR[31:8]: Unused.
 * - LVR[7:5]: I2C device index:
 *   - 0: I2C device has a 50 ns spike filter where
 *        it is not affected by high frequency on SCL.
 *   - 1: I2C device does not have a 50 ns spike filter
 *        but can work with high frequency on SCL.
 *   - 2: I2C device does not have a 50 ns spike filter
 *        and cannot work with high frequency on SCL.
 * - LVR[4]: I2C mode indicator:
 *   - 0: FM+ mode
 *   - 1: FM mode
 * - LVR[3:0]: Reserved.
 */
#define I3C_DCR_I2C_FM_PLUS_MODE			0
#define I3C_DCR_I2C_FM_MODE				1

#define I3C_DCR_I2C_MODE_SHIFT				4
#define I3C_DCR_I2C_MODE_MASK				BIT(4)

#define I3C_DCR_I2C_MODE(dcr)				\
	(((mode) & I3C_DCR_I2C_MODE_MASK) >> I3C_DCR_I2C_MODE_SHIFT)

#define I3C_DCR_I2C_DEV_IDX_0				0
#define I3C_DCR_I2C_DEV_IDX_1				1
#define I3C_DCR_I2C_DEV_IDX_2				2

#define I3C_DCR_I2C_DEV_IDX_SHIFT			5
#define I3C_DCR_I2C_DEV_IDX_MASK			(0x07U << I3C_DCR_I2C_DEV_IDX_SHIFT)

#define I3C_DCR_I2C_DEV_IDX(dcr)			\
	(((dcr) & I3C_DCR_I2C_DEV_IDX_MASK) >> I3C_DCR_I2C_DEV_IDX_SHIFT)

/**
 * @brief I3C bus mode
 */
enum i3c_bus_mode {
	/** Only I3C devices are on the bus. */
	I3C_BUS_MODE_PURE,

	/**
	 * Both I3C and legacy I2C devices are on the bus.
	 * The I2C devices have 50ns spike filter on SCL.
	 */
	I3C_BUS_MODE_MIXED_FAST,

	/**
	 * Both I3C and legacy I2C devices are on the bus.
	 * The I2C devices do not have 50ns spike filter on SCL
	 * and can tolerate maximum SDR SCL clock frequency.
	 */
	I3C_BUS_MODE_MIXED_LIMITED,

	/**
	 * Both I3C and legacy I2C devices are on the bus.
	 * The I2C devices do not have 50ns spike filter on SCL
	 * but cannot tolerate maximum SDR SCL clock frequency.
	 */
	I3C_BUS_MODE_MIXED_SLOW,

	I3C_BUS_MODE_MAX = I3C_BUS_MODE_MIXED_SLOW,
	I3C_BUS_MODE_INVALID,
};

/**
 * @brief I2C bus speed under I3C bus.
 *
 * Only FM and FM+ modes are supported for I2C devices under I3C bus.
 */
enum i3c_i2c_speed_type {
	/** I2C FM mode */
	I3C_I2C_SPEED_FM,

	/** I2C FM+ mode */
	I3C_I2C_SPEED_FMPLUS,

	I3C_I2C_SPEED_MAX = I3C_I2C_SPEED_FMPLUS,
	I3C_I2C_SPEED_INVALID,
};

/**
 * @brief I3C data rate
 *
 * I3C data transfer rate defined by the I3C specification.
 */
enum i3c_data_rate {
	/** Single Data Rate messaging */
	I3C_DATA_RATE_SDR,

	/** High Data Rate - Double Data Rate messaging */
	I3C_DATA_RATE_HDR_DDR,

	/** High Data Rate - Ternary Symbol Legacy-inclusive-Bus */
	I3C_DATA_RATE_HDR_TSL,

	/** High Data Rate - Ternary Symbol for Pure Bus */
	I3C_DATA_RATE_HDR_TSP,

	/** High Data Rate - Bulk Transport */
	I3C_DATA_RATE_HDR_BT,

	I3C_DATA_RATE_MAX = I3C_DATA_RATE_HDR_BT,
	I3C_DATA_RATE_INVALID,
};

/**
 * @brief I3C SDR Controller Error Codes
 *
 * These are error codes defined by the I3C specification.
 *
 * @c I3C_ERROR_CE_UNKNOWN and @c I3C_ERROR_CE_NONE are not
 * official error codes according to the specification.
 * These are there simply to aid in error handling during
 * interactions with the I3C drivers and subsystem.
 */
enum i3c_sdr_controller_error_codes {
	/** Transaction after sending CCC */
	I3C_ERROR_CE0,

	/** Monitoring Error */
	I3C_ERROR_CE1,

	/** No response to broadcast address (0x7E) */
	I3C_ERROR_CE2,

	/** Failed Controller Handoff */
	I3C_ERROR_CE3,

	/** Unknown error (not official error code) */
	I3C_ERROR_CE_UNKNOWN,

	/** No error (not official error code) */
	I3C_ERROR_CE_NONE,

	I3C_ERROR_CE_MAX = I3C_ERROR_CE_UNKNOWN,
	I3C_ERROR_CE_INVALID,
};

/**
 * @brief I3C SDR Target Error Codes
 *
 * These are error codes defined by the I3C specification.
 *
 * @c I3C_ERROR_TE_UNKNOWN and @c I3C_ERROR_TE_NONE are not
 * official error codes according to the specification.
 * These are there simply to aid in error handling during
 * interactions with the I3C drivers and subsystem.
 */
enum i3c_sdr_target_error_codes {
	/**
	 * Invalid Broadcast Address or
	 * Dynamic Address after DA assignment
	 */
	I3C_ERROR_TE0,

	/** CCC Code */
	I3C_ERROR_TE1,

	/** Write Data */
	I3C_ERROR_TE2,

	/** Assigned Address during Dynamic Address Arbitration */
	I3C_ERROR_TE3,

	/** 0x7E/R missing after RESTART during Dynamic Address Arbitration */
	I3C_ERROR_TE4,

	/** Transaction after detecting CCC */
	I3C_ERROR_TE5,

	/** Monitoring Error */
	I3C_ERROR_TE6,

	/** Dead Bus Recovery */
	I3C_ERROR_DBR,

	/** Unknown error (not official error code) */
	I3C_ERROR_TE_UNKNOWN,

	/** No error (not official error code) */
	I3C_ERROR_TE_NONE,

	I3C_ERROR_TE_MAX = I3C_ERROR_TE_UNKNOWN,
	I3C_ERROR_TE_INVALID,
};

/*
 * I3C_MSG_* are I3C Message flags.
 */

/** Write message to I3C bus. */
#define I3C_MSG_WRITE			(0U << 0U)

/** Read message from I2C bus. */
#define I3C_MSG_READ			BIT(0)

/** @cond INTERNAL_HIDDEN */
#define I3C_MSG_RW_MASK			BIT(0)
/** @endcond  */

/** Send STOP after this message. */
#define I3C_MSG_STOP			BIT(1)

/** RESTART I3C transaction for this message.
 *
 * @note Not all I3C drivers have or require explicit support for this
 * feature. Some drivers require this be present on a read message
 * that follows a write, or vice-versa.  Some drivers will merge
 * adjacent fragments into a single transaction using this flag; some
 * will not.
 */
#define I3C_MSG_RESTART			BIT(2)

/** Transfer use HDR mode */
#define I3C_MSG_HDR			BIT(3)

/** I3C HDR Mode 0 */
#define I3C_MSG_HDR_MODE0		BIT(0)

/** I3C HDR Mode 1 */
#define I3C_MSG_HDR_MODE1		BIT(1)

/** I3C HDR Mode 2 */
#define I3C_MSG_HDR_MODE2		BIT(2)

/** I3C HDR Mode 3 */
#define I3C_MSG_HDR_MODE3		BIT(3)

/** I3C HDR Mode 4 */
#define I3C_MSG_HDR_MODE4		BIT(4)

/** I3C HDR Mode 5 */
#define I3C_MSG_HDR_MODE5		BIT(5)

/** I3C HDR Mode 6 */
#define I3C_MSG_HDR_MODE6		BIT(6)

/** I3C HDR Mode 7 */
#define I3C_MSG_HDR_MODE7		BIT(7)

/** I3C HDR-DDR (Double Data Rate) */
#define I3C_MSG_HDR_DDR			I3C_MSG_HDR_MODE0

/** I3C HDR-TSP (Ternary Symbol Pure-bus) */
#define I3C_MSG_HDR_TSP			I3C_MSG_HDR_MODE1

/** I3C HDR-TSL (Ternary Symbol Legacy-inclusive-bus) */
#define I3C_MSG_HDR_TSL			I3C_MSG_HDR_MODE2

/** I3C HDR-BT (Bulk Transport) */
#define I3C_MSG_HDR_BT			I3C_MSG_HDR_MODE3

/**
 * @brief One I3C Message.
 *
 * This defines one I3C message to transact on the I3C bus.
 *
 * @note Some of the configurations supported by this API may not be
 * supported by specific SoC I3C hardware implementations, in
 * particular features related to bus transactions intended to read or
 * write data from different buffers within a single transaction.
 * Invocations of i3c_transfer() may not indicate an error when an
 * unsupported configuration is encountered.  In some cases drivers
 * will generate separate transactions for each message fragment, with
 * or without presence of @ref I3C_MSG_RESTART in #flags.
 */
struct i3c_msg {
	/** Data buffer in bytes */
	uint8_t			*buf;

	/** Length of buffer in bytes */
	uint32_t		len;

	/** Flags for this message */
	uint8_t			flags;

	/**
	 * HDR mode (@c I3C_MSG_HDR_MODE*) for transfer
	 * if @c I3C_MSG_HDR is set in @c flags.
	 */
	uint8_t			hdr_mode;
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * These are for internal use only, so skip these in
 * public documentation.
 */
struct i3c_device_desc;
struct i3c_i2c_device_desc;

__subsystem struct i3c_driver_api {
	int (*do_daa)(const struct device *dev);
	int (*do_ccc)(const struct device *dev,
		      struct i3c_ccc_payload *payload);

	int (*i3c_xfers)(const struct device *dev,
			 struct i3c_device_desc *target,
			 struct i3c_msg *msgs,
			 uint8_t num_msgs);
	int (*i3c_device_register)(const struct device *dev,
				   struct i3c_device_desc *desc);

	int (*i3c_i2c_xfers)(const struct device *dev,
			     struct i3c_i2c_device_desc *i2c_dev,
			     struct i2c_msg *msgs,
			     uint8_t num_msgs);
	int (*i3c_i2c_device_register)(const struct device *dev,
				       struct i3c_i2c_device_desc *desc);

	int (*i3c_ibi_slot_request)(const struct device *dev,
				    struct i3c_device_desc *target);
	int (*i3c_ibi_slot_free)(const struct device *dev,
				 struct i3c_device_desc *target);
	int (*i3c_ibi_enable)(const struct device *dev,
			      struct i3c_device_desc *target);
	int (*i3c_ibi_disable)(const struct device *dev,
			       struct i3c_device_desc *target);
};

/**
 * @endcond
 */

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
 * @brief Structure describing a I3C target device.
 *
 * Instances of this are passed to the I3C controller device APIs,
 * for example:
 * - i3c_device_register() to tell the controller of a target device.
 * - i3c_transfers() to initiate data transfers between controller and
 *   target device.
 *
 * Fields @c bus, @c pid and @c static_addr must be initialized by
 * the module that implements the target device behavior prior to
 * passing the object reference to I3C controller device APIs.
 * @c static_addr can be zero if target device does not have static
 * address.
 *
 * Field @c node should not be initialized or modified manually.
 */
struct i3c_device_desc {
	/** Private, do not modify */
	sys_snode_t node;

	/** I3C bus to which this target device is attached */
	const struct device *bus;

	/** Device Provisioned ID */
	const uint64_t pid;

	/**
	 * Static address for this target device.
	 *
	 * 0 if static address is not being used, and only dynamic
	 * address is used. This means that the target device must
	 * go through ENTDAA (Dynamic Address Assignment) to get
	 * a dynamic address before it can communicate with
	 * the controller. This means SETAASA and SETDASA CCC
	 * cannot be used to set dynamic address on the target
	 * device (as both are to tell target device to use static
	 * address as dynamic address).
	 */
	const uint8_t static_addr;

	/**
	 * Initial dynamic address.
	 *
	 * This is specified in the device tree property "assigned-address"
	 * to indicate the desired dynamic address during address
	 * assignment (SETDASA and ENTDAA).
	 *
	 * 0 if there is no preference.
	 */
	const uint8_t init_dynamic_addr;

	/**
	 * Dynamic Address for this target device used for communication.
	 *
	 * This is to be set by the controller driver in one of
	 * the following situations:
	 * - During Dynamic Address Assignment (during ENTDAA)
	 * - Reset Dynamic Address Assignment (RSTDAA)
	 * - Set All Addresses to Static Addresses (SETAASA)
	 * - Set New Dynamic Address (SETNEWDA)
	 * - Set Dynamic Address from Static Address (SETDASA)
	 *
	 * 0 if address has not been assigned.
	 */
	uint8_t dynamic_addr;

#ifdef CONFIG_I3C_USE_GROUP_ADDR
	/**
	 * Group address for this target device. Set during:
	 * - Reset Group Address(es) (RSTGRPA)
	 * - Set Group Address (SETGRPA)
	 *
	 * 0 if group address has not been assigned.
	 */
	uint8_t group_addr;
#endif /* CONFIG_I3C_USE_GROUP_ADDR */

	/**
	 * Bus Characteristic Register (BCR)
	 * - BCR[7:6]: Device Role
	 *   - 0: I3C Target
	 *   - 1: I3C Controller capable
	 *   - 2: Reserved
	 *   - 3: Reserved
	 * - BCR[5]: Advanced Capabilities
	 *   - 0: Does not support optional advanced capabilities.
	 *   - 1: Supports optional advanced capabilities which
	 *        can be viewed via GETCAPS CCC.
	 * - BCR[4}: Virtual Target Support
	 *   - 0: Is not a virtual target.
	 *   - 1: Is a virtual target.
	 * - BCR[3]: Offline Capable
	 *   - 0: Will always response to I3C commands.
	 *   - 1: Will not always response to I3C commands.
	 * - BCR[2]: IBI Payload
	 *   - 0: No data bytes following the accepted IBI.
	 *   - 1: One data byte (MDB, Mandatory Data Byte) follows
	 *        the accepted IBI. Additional data bytes may also
	 *        follows.
	 * - BCR[1]: IBI Request Capable
	 *   - 0: Not capable
	 *   - 1: Capable
	 * - BCR[0]: Max Data Speed Limitation
	 *   - 0: No Limitation
	 *   - 1: Limitation obtained via GETMXDS CCC.
	 */
	uint8_t bcr;

	/**
	 * Device Characteristic Register (DCR)
	 *
	 * Describes the type of device. Refer to official documentation
	 * on what this number means.
	 */
	uint8_t dcr;

	struct {
		/** Maximum Read Speed */
		uint8_t maxrd;

		/** Maximum Write Speed */
		uint8_t maxwr;

		/** Maximum Read turnaround time in microseconds. */
		uint32_t max_read_turnaround;
	} data_speed;

	struct {
		/** Maximum Read Length */
		uint16_t mrl;

		/** Maximum Write Length */
		uint16_t mwl;

		/** Maximum IBI Payload Size. Valid only if BCR[2] is 1. */
		uint8_t max_ibi;
	} data_length;

	/** Private data by the controller to aid in transactions. Do not modify. */
	void *controller_priv;

#ifdef CONFIG_I3C_USE_IBI
	/**
	 * In-Band Interrupt (IBI) callback.
	 */
	i3c_target_ibi_cb_t ibi_cb;
#endif /* CONFIG_I3C_USE_IBI */
};

/**
 * @brief Structure initializer for i3c_device_desc from devicetree
 *
 * This helper macro expands to a static initializer for a <tt>struct
 * i3c_device_desc</tt> by reading the relevant bus and device data
 * from the devicetree, and an associated IBI callback.
 *
 * @param node_id Devicetree node identifier for the I3C device whose
 *                struct i3c_device_desc to create an initializer for.
 * @param ibi_cb Pointer to IBI callback.
 */
#define I3C_DEVICE_DESC_DT_WITH_IBI_CB(node_id, ibi_cb)			\
	{								\
		.bus = DEVICE_DT_GET(DT_BUS(node_id)),			\
		.static_addr = DT_PROP_BY_IDX(node_id, reg, 0),		\
		.pid = ((uint64_t)DT_PROP_BY_IDX(node_id, reg, 1) << 32)\
		       | DT_PROP_BY_IDX(node_id, reg, 2),		\
		.ibi_cb = ibi_cb,					\
		.init_dynamic_addr =					\
			DT_PROP_OR(node_id, assigned_address, 0),	\
	}

/**
 * @brief Structure initializer for i3c_device_desc from devicetree
 *
 * This helper macro expands to a static initializer for a <tt>struct
 * i3c_device_desc</tt> by reading the relevant bus and device data
 * from the devicetree.
 *
 * @param node_id Devicetree node identifier for the I3C device whose
 *                struct i3c_device_desc to create an initializer for
 */
#define I3C_DEVICE_DESC_DT(node_id)					\
	{								\
		.bus = DEVICE_DT_GET(DT_BUS(node_id)),			\
		.static_addr = DT_PROP_BY_IDX(node_id, reg, 0),		\
		.pid = ((uint64_t)DT_PROP_BY_IDX(node_id, reg, 1) << 32)\
		       | DT_PROP_BY_IDX(node_id, reg, 2),		\
		.init_dynamic_addr =					\
			DT_PROP_OR(node_id, assigned_address, 0),	\
	}

/**
 * @brief Structure initializer for i3c_device_desc from devicetree instance
 *
 * This is equivalent to
 * <tt>I3C_DEVICE_DESC_DT_WITH_IBI_PAYLOAD(DT_DRV_INST(inst), ibi_cb)</tt>.
 *
 * @param inst Devicetree instance number
 * @param ibi_cb Pointer to IBI callback.
 */
#define I3C_DEVICE_DESC_DT_INST_WITH_IBI_CB(inst, ibi_cb)		\
	I3C_DEVICE_DESC_DT_WITH_IBI_CB(DT_DRV_INST(inst, ibi_cb))

/**
 * @brief Structure initializer for i3c_device_desc from devicetree instance
 *
 * This is equivalent to
 * <tt>I3C_DEVICE_DESC_DT(DT_DRV_INST(inst))</tt>.
 *
 * @param inst Devicetree instance number
 */
#define I3C_DEVICE_DESC_DT_INST(inst) \
	I3C_DEVICE_DESC_DT(DT_DRV_INST(inst))

/**
 * @brief Like DEVICE_DT_DEFINE() with I3C target device specifics.
 *
 * Defines a I3C target device which implements the I3C target device API.
 *
 * @param node_id The devicetree node identifier.
 *
 * @param init_fn Name of the init function of the driver.
 *
 * @param pm_device PM device resources reference (NULL if device does not use PM).
 *
 * @param data_ptr Pointer to the device's private data.
 *
 * @param cfg_ptr The address to the structure containing the
 *                configuration information for this instance of the driver.
 *
 * @param level The initialization level. See SYS_INIT() for
 *              details.
 *
 * @param prio Priority within the selected initialization level. See
 *             SYS_INIT() for details.
 *
 * @param api_ptr Provides an initial pointer to the API function struct
 *                used by the driver. Can be NULL.
 */
#define I3C_DEVICE_DT_DEFINE(node_id, init_fn, pm_device,		\
			     data_ptr, cfg_ptr, level, prio,		\
			     api_ptr, ...)				\
	DEVICE_DT_DEFINE(node_id, init_fn, pm_device,			\
			 data_ptr, cfg_ptr, level, prio,		\
			 api_ptr, __VA_ARGS__)

/**
 * @brief Like I3C_TARGET_DT_DEFINE() for an instance of a DT_DRV_COMPAT compatible
 *
 * @param inst instance number. This is replaced by
 * <tt>DT_DRV_COMPAT(inst)</tt> in the call to I3C_TARGET_DT_DEFINE().
 *
 * @param ... other parameters as expected by I3C_TARGET_DT_DEFINE().
 */
#define I3C_DEVICE_DT_INST_DEFINE(inst, ...)				\
	I3C_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

/**
 * @brief Structure describing a I2C device on I3C bus.
 *
 * Instances of this are passed to the I3C controller device APIs,
 * for example:
 * () i3c_i2c_device_register() to tell the controller of an I2C device.
 * () i3c_i2c_transfers() to initiate data transfers between controller
 *    and I2C device.
 *
 * Fields other than @c node must be initialized by the module that
 * implements the device behavior prior to passing the object
 * reference to I3C controller device APIs.
 */
struct i3c_i2c_device_desc {
	/** Private, do not modify */
	sys_snode_t node;

	/** I3C bus to which this I2C device is attached */
	const struct device *bus;

	/** Static address for this I2C device. */
	const uint8_t addr;

	/**
	 * Legacy Virtual Register (LVR)
	 * - LVR[31:8]: Unused.
	 * - LVR[7:5]: I2C device index:
	 *   - 0: I2C device has a 50 ns spike filter where
	 *        it is not affected by high frequency on SCL.
	 *   - 1: I2C device does not have a 50 ns spike filter
	 *        but can work with high frequency on SCL.
	 *   - 2: I2C device does not have a 50 ns spike filter
	 *        and cannot work with high frequency on SCL.
	 * - LVR[4]: I2C mode indicator:
	 *   - 0: FM+ mode
	 *   - 1: FM mode
	 * - LVR[3:0]: Reserved.
	 */
	const uint32_t lvr;

	/** Private data by the controller to aid in transactions. Do not modify. */
	void *controller_priv;
};

/**
 * @brief Structure initializer for i3c_i2c_device_desc from devicetree
 *
 * This helper macro expands to a static initializer for a <tt>struct
 * i3c_i2c_device_desc</tt> by reading the relevant bus and device data
 * from the devicetree.
 *
 * @param node_id Devicetree node identifier for the I3C device whose
 *                struct i3c_i2c_device_desc to create an initializer for
 */
#define I3C_I2C_DEVICE_DESC_DT(node_id)					\
	{								\
		.bus = DEVICE_DT_GET(DT_BUS(node_id)),			\
		.addr = DT_PROP_BY_IDX(node_id, reg, 0),		\
		.lvr = DT_PROP_BY_IDX(node_id, reg, 2),			\
	}

/**
 * @brief Structure initializer for i3c_device_desc from devicetree instance
 *
 * This is equivalent to
 * <tt>I3C_I2C_DEVICE_DESC_DT(DT_DRV_INST(inst))</tt>.
 *
 * @param inst Devicetree instance number
 */
#define I3C_I2C_DEVICE_DESC_DT_INST(inst) \
	I3C_I2C_DEVICE_DESC_DT(DT_DRV_INST(inst))

/**
 * @brief Run an init function to perform I3C bus initialization
 *
 * This calls @p _init_fn to perform I3C bus initialization at init
 * level @c POST_KERNEL and priority @c CONFIG_I3C_BUS_INIT_PRIORITY.
 *
 * This needs to be called after the device driver instances of
 * attached I2C and I3C devices have registered themselves with
 * the controller device driver instance. Then, @p _init_fn performs
 * necessary steps to setup the controller hardware and to assign
 * addresses to attached devices (with @c SETAASA, @c SETDASA, and/or
 * @c ENTDAA). Only the devices that have called i3c_device_register()
 * will have dynamic addresses set or assigned. If there are I2C
 * devices on the bus, i3c_i2c_device_register() must be called to
 * tell the controller which addresses are reserved for I2C and
 * will not be assigned as dynamic addresses.
 *
 * @note Only the primary controller of the bus should initialize
 * the bus.
 *
 * @param _init_fn Pointer to the bus init function to run
 * @param _dev Pointer to the I3C controller device driver instance
 */
#define I3C_BUS_INIT(_init_fn, _dev) \
	Z_INIT_ENTRY_DEFINE(Z_SYS_NAME(_init_fn), _init_fn, _dev, \
			    POST_KERNEL, CONFIG_I3C_BUS_INIT_PRIORITY)

/**
 * @brief Run an init function to register devices to I3C controller
 *
 * This calls @p _init_fn to allow devices to register themselves
 * to the I3C controller at init level @c POST_KERNEL and
 * priority @c CONFIG_I3C_DEV_REGISTER_INIT_PRIORITY.
 *
 * This needs to be called after the I3C controller has initialized
 * its internal data structures to store device information. Also,
 * this must be called before bus initialization. The device
 * registration is to tell the controller the connected I2C and I3C
 * devices so the controller can assign dynamic addresses to I3C
 * devices, and reserve the addresses used by I2C (so they are not
 * being assigned to I3C devices).
 *
 * @param _init_fn Pointer to the bus init function to run
 * @param _dev Pointer to the I3C controller device driver instance
 */
#define I3C_DEVICE_REGISTER_INIT(_init_fn, _dev) \
	Z_INIT_ENTRY_DEFINE(Z_SYS_NAME(_init_fn), _init_fn, _dev, \
			    POST_KERNEL, CONFIG_I3C_DEV_REGISTER_INIT_PRIORITY)

/**
 * @brief Structure for describing attached devices for a controller.
 *
 * This contains linked lists describing attached I3C and I2C devices.
 * This also has an array of address slots which describes the status
 * of addresses (e.g. free or in use).
 *
 * This is a helper struct that can be used by controller device
 * driver to aid in device management.
 */
struct i3c_dev_list {
	/**
	 * Address slots:
	 * - Aid in dynamic address assignment.
	 * - Quick way to find out if a target address is
	 *   a I3C or I2C device.
	 */
	struct i3c_addr_slots addr_slots;

	struct {
		/**
		 * Linked list of attached I3C devices.
		 */
		sys_slist_t i3c;

		/**
		 * Linked list of attached I2C devices.
		 */
		sys_slist_t i2c;
	} devices;
};

/**
 * @brief Initialize the device list struct.
 *
 * This initializes the address slots and the linked lists of
 * attached devices.
 *
 * @param dev_list Pointer to the device list struct.
 */
void i3c_dev_list_init(struct i3c_dev_list *dev_list);

/**
 * @brief Add an I3C target device to the devce list.
 *
 * This adds an I3C target device to the device list, using
 * the I3C target descriptor.
 *
 * @param dev_list Pointer to the device list struct.
 * @param target Pointer to the I3C target device descriptor.
 */
void i3c_dev_list_target_add(struct i3c_dev_list *dev_list,
			     struct i3c_device_desc *target);

/**
 * @brief Add an I2C device to the devce list.
 *
 * This adds an I2C device to the device list, using
 * the I2C device descriptor.
 *
 * @param dev_list Pointer to the device list struct.
 * @param i2c_dev Pointer to the I2C device descriptor.
 */
void i3c_dev_list_i2c_devce_add(struct i3c_dev_list *dev_list,
				struct i3c_i2c_device_desc *i2c_dev);

/**
 * @brief Find a I3C target device descriptor by Provisioned ID.
 *
 * This finds the I3C target device descriptor in the device list
 * matching the provided Provisioned ID (@p pid).
 *
 * @param dev_list Pointer to the device list struct.
 * @param pid Provisioned ID to be matched.
 *
 * @return Pointer the the I3C target device descriptor, or
 *         NULL if none is found.
 */
struct i3c_device_desc *i3c_dev_list_find_pid(struct i3c_dev_list *dev_list,
					      uint64_t pid);

/**
 * @brief Perform Dynamic Address Assignment on the I3C bus.
 *
 * This routine asks the controller to perform dynamic address assignment
 * where the controller belongs. Only the active controller of the bus
 * should do this.
 *
 * @note For controller driver implementation, the controller should perform
 * SETAASA to allow static addresses to be the dynamic addresses before
 * actually doing ENTDAA.
 *
 * @param dev Pointer to the device structure for the controller driver
 *            instance.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error, failed to initialize bus.
 * @retval -ENODEV If a provisioned ID does not match to any target devices
 *                 in the registered device list.
 * @retval -ENOSPC No more free addresses can be assigned to target.
 * @retval -ENOSYS Dynamic address assignment is not supported by
 *                 the controller driver.
 */
static inline int i3c_do_daa(const struct device *dev)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)dev->api;

	if (api->do_daa == NULL) {
		return -ENOSYS;
	}

	return api->do_daa(dev);
}

/**
 * @brief Send CCC to the bus.
 *
 * @param dev Pointer to the device structure for the controller driver
 *            instance.
 * @param payload Pointer to the structure describing the CCC payload.
 *
 * @retval 0 If successful.
 * @retval -EIO General Input / output error.
 * @retval -EINVAL Invalid valid set in the payload structure.
 */
__syscall int i3c_do_ccc(const struct device *dev,
			 struct i3c_ccc_payload *payload);

static inline int z_impl_i3c_do_ccc(const struct device *dev,
				    struct i3c_ccc_payload *payload)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)dev->api;

	return api->do_ccc(dev, payload);
}

/**
 * @brief Perform data transfer from the controller to a I3C target device.
 *
 * This routine provides a generic interface to perform data transfer
 * to a target device synchronously. Use i3c_read()/i3c_write()
 * for simple read or write.
 *
 * The array of message @a msgs must not be NULL.  The number of
 * message @a num_msgs may be zero, in which case no transfer occurs.
 *
 * @note Not all scatter/gather transactions can be supported by all
 * drivers.  As an example, a gather write (multiple consecutive
 * `i3c_msg` buffers all configured for `I3C_MSG_WRITE`) may be packed
 * into a single transaction by some drivers, but others may emit each
 * fragment as a distinct write transaction, which will not produce
 * the same behavior.  See the documentation of `struct i3c_msg` for
 * limitations on support for multi-message bus transactions.
 *
 * @param target I3C target device descriptor.
 * @param msgs Array of messages to transfer.
 * @param num_msgs Number of messages to transfer.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
__syscall int i3c_transfer(struct i3c_device_desc *target,
			   struct i3c_msg *msgs, uint8_t num_msgs);

static inline int z_impl_i3c_transfer(struct i3c_device_desc *target,
				      struct i3c_msg *msgs, uint8_t num_msgs)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)target->bus->api;

	return api->i3c_xfers(target->bus, target, msgs, num_msgs);
}

/**
 * @brief Register a target device with the controller driver instance.
 *
 * This makes the controller driver instance aware of a new target device,
 * and is inserted into the controller's connected device list.
 *
 * @note This only tells the controller about this target device.
 * Dynamic address of the device is not assigned yet.
 *
 * @param target I3C target device descriptor.
 *
 * @retval 0 if registration is successful.
 * @retval -EINVAL device static address is reserved or already assigned.
 */
static inline int i3c_device_register(struct i3c_device_desc *target)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)target->bus->api;

	return api->i3c_device_register(target->bus, target);
}

/**
 * @brief Request/Allocate an IBI slot for a target device.
 *
 * There are usually limited IBI slots available on the controller.
 * Each slot must be programmed accordingly for the controller to
 * recognize IBI from target devices. This function is to request
 * an IBI slot and to program the slot.
 *
 * @param target I3C target device descriptor.
 *
 * @retval 0 If successful.
 * @retval -EIO General Input / output error.
 * @retval -ENOSPC No free IBI slots available.
 */
static inline int i3c_ibi_slot_request(struct i3c_device_desc *target)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)target->bus->api;

	if (api->i3c_ibi_slot_request == NULL) {
		return -ENOSYS;
	}

	return api->i3c_ibi_slot_request(target->bus, target);
}

/**
 * @brief Free/Deallocate an IBI slot for a target device.
 *
 * This frees a previous allocated IBI slot for a target device.
 *
 * @param target I3C target device descriptor.
 *
 * @retval 0 If successful.
 * @retval -EINVAL If there is no allocated IBI slot for the target device.
 * @retval -EIO General Input / output error.
 */
static inline int i3c_ibi_slot_free(struct i3c_device_desc *target)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)target->bus->api;

	if (api->i3c_ibi_slot_free == NULL) {
		return -ENOSYS;
	}

	return api->i3c_ibi_slot_free(target->bus, target);
}

/**
 * @brief Enable IBI of a target device.
 *
 * This enables IBI of a target device where the IBI has already been
 * request.
 *
 * @param target I3C target device descriptor.
 *
 * @retval 0 If successful.
 * @retval -EIO General Input / output error.
 */
static inline int i3c_ibi_enable(struct i3c_device_desc *target)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)target->bus->api;

	if (api->i3c_ibi_enable == NULL) {
		return -ENOSYS;
	}

	return api->i3c_ibi_enable(target->bus, target);
}

/**
 * @brief Disable IBI of a target device.
 *
 * This enables IBI of a target device where the IBI has already been
 * request.
 *
 * @param target I3C target device descriptor.
 *
 * @retval 0 If successful.
 * @retval -EIO General Input / output error.
 */
static inline int i3c_ibi_disable(struct i3c_device_desc *target)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)target->bus->api;

	if (api->i3c_ibi_disable == NULL) {
		return -ENOSYS;
	}

	return api->i3c_ibi_disable(target->bus, target);
}

/*
 * Derived I3C APIs -- all implemented in terms of i3c_transfer()
 */

/**
 * @brief Write a set amount of data to an I3C target device.
 *
 * This routine writes a set amount of data synchronously.
 *
 * @param target I3C target device descriptor.
 * @param buf Memory pool from which the data is transferred.
 * @param num_bytes Number of bytes to write.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_write(struct i3c_device_desc *target,
			    const uint8_t *buf, uint32_t num_bytes)
{
	struct i3c_msg msg;

	msg.buf = (uint8_t *)buf;
	msg.len = num_bytes;
	msg.flags = I3C_MSG_WRITE | I3C_MSG_STOP;

	return i3c_transfer(target, &msg, 1);
}

/**
 * @brief Read a set amount of data from an I3C target device.
 *
 * This routine reads a set amount of data synchronously.
 *
 * @param target I3C target device descriptor.
 * @param buf Memory pool that stores the retrieved data.
 * @param num_bytes Number of bytes to read.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_read(struct i3c_device_desc *target,
			   uint8_t *buf, uint32_t num_bytes)
{
	struct i3c_msg msg;

	msg.buf = buf;
	msg.len = num_bytes;
	msg.flags = I3C_MSG_READ | I3C_MSG_STOP;

	return i3c_transfer(target, &msg, 1);
}

/**
 * @brief Write then read data from an I3C target device.
 *
 * This supports the common operation "this is what I want", "now give
 * it to me" transaction pair through a combined write-then-read bus
 * transaction.
 *
 * @param target I3C target device descriptor.
 * @param write_buf Pointer to the data to be written
 * @param num_write Number of bytes to write
 * @param read_buf Pointer to storage for read data
 * @param num_read Number of bytes to read
 *
 * @retval 0 if successful
 * @retval negative on error.
 */
static inline int i3c_write_read(struct i3c_device_desc *target,
				 const void *write_buf, size_t num_write,
				 void *read_buf, size_t num_read)
{
	struct i3c_msg msg[2];

	msg[0].buf = (uint8_t *)write_buf;
	msg[0].len = num_write;
	msg[0].flags = I3C_MSG_WRITE;

	msg[1].buf = (uint8_t *)read_buf;
	msg[1].len = num_read;
	msg[1].flags = I3C_MSG_RESTART | I3C_MSG_READ | I3C_MSG_STOP;

	return i3c_transfer(target, msg, 2);
}

/**
 * @brief Read multiple bytes from an internal address of an I3C target device.
 *
 * This routine reads multiple bytes from an internal address of an
 * I3C target device synchronously.
 *
 * Instances of this may be replaced by i3c_write_read().
 *
 * @param target I3C target device descriptor,
 * @param start_addr Internal address from which the data is being read.
 * @param buf Memory pool that stores the retrieved data.
 * @param num_bytes Number of bytes being read.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_burst_read(struct i3c_device_desc *target,
				 uint8_t start_addr,
				 uint8_t *buf,
				 uint32_t num_bytes)
{
	return i3c_write_read(target,
			      &start_addr, sizeof(start_addr),
			      buf, num_bytes);
}

/**
 * @brief Write multiple bytes to an internal address of an I3C target device.
 *
 * This routine writes multiple bytes to an internal address of an
 * I3C target device synchronously.
 *
 * @warning The combined write synthesized by this API may not be
 * supported on all I3C devices.  Uses of this API may be made more
 * portable by replacing them with calls to i3c_write() passing a
 * buffer containing the combined address and data.
 *
 * @param target I3C target device descriptor.
 * @param start_addr Internal address to which the data is being written.
 * @param buf Memory pool from which the data is transferred.
 * @param num_bytes Number of bytes being written.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_burst_write(struct i3c_device_desc *target,
				  uint8_t start_addr,
				  const uint8_t *buf,
				  uint32_t num_bytes)
{
	struct i3c_msg msg[2];

	msg[0].buf = &start_addr;
	msg[0].len = 1U;
	msg[0].flags = I3C_MSG_WRITE;

	msg[1].buf = (uint8_t *)buf;
	msg[1].len = num_bytes;
	msg[1].flags = I3C_MSG_RESTART | I3C_MSG_WRITE | I3C_MSG_STOP;

	return i3c_transfer(target, msg, 2);
}

/**
 * @brief Read internal register of an I3C target device.
 *
 * This routine reads the value of an 8-bit internal register of an I3C target
 * device synchronously.
 *
 * @param target I3C target device descriptor.
 * @param reg_addr Address of the internal register being read.
 * @param value Memory pool that stores the retrieved register value.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_reg_read_byte(struct i3c_device_desc *target,
				    uint8_t reg_addr, uint8_t *value)
{
	return i3c_write_read(target,
			      &reg_addr, sizeof(reg_addr),
			      value, sizeof(*value));
}

/**
 * @brief Write internal register of an I3C target device.
 *
 * This routine writes a value to an 8-bit internal register of an I3C target
 * device synchronously.
 *
 * @note This function internally combines the register and value into
 * a single bus transaction.
 *
 * @param target I3C target device descriptor.
 * @param reg_addr Address of the internal register being written.
 * @param value Value to be written to internal register.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_reg_write_byte(struct i3c_device_desc *target,
				     uint8_t reg_addr, uint8_t value)
{
	uint8_t tx_buf[2] = {reg_addr, value};

	return i3c_write(target, tx_buf, 3);
}

/**
 * @brief Update internal register of an I3C target device.
 *
 * This routine updates the value of a set of bits from an 8-bit internal
 * register of an I3C target device synchronously.
 *
 * @note If the calculated new register value matches the value that
 * was read this function will not generate a write operation.
 *
 * @param target I3C target device descriptor.
 * @param reg_addr Address of the internal register being updated.
 * @param mask Bitmask for updating internal register.
 * @param value Value for updating internal register.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_reg_update_byte(struct i3c_device_desc *target,
				      uint8_t reg_addr, uint8_t mask,
				      uint8_t value)
{
	uint8_t old_value, new_value;
	int rc;

	rc = i3c_reg_read_byte(target, reg_addr, &old_value);
	if (rc != 0) {
		return rc;
	}

	new_value = (old_value & ~mask) | (value & mask);
	if (new_value == old_value) {
		return 0;
	}

	return i3c_reg_write_byte(target, reg_addr, new_value);
}

/**
 * @brief Perform data transfer from the I3C controller to an I2C device.
 *
 * This routine provides a generic interface to perform data transfer
 * to a target device synchronously. Use i3c_i2c_read()/i3c_i2c_write()
 * for simple read or write.
 *
 * The array of message @a msgs must not be NULL.  The number of
 * message @a num_msgs may be zero, in which case no transfer occurs.
 *
 * @note Not all scatter/gather transactions can be supported by all
 * drivers.  As an example, a gather write (multiple consecutive
 * `i2c_msg` buffers all configured for `I2C_MSG_WRITE`) may be packed
 * into a single transaction by some drivers, but others may emit each
 * fragment as a distinct write transaction, which will not produce
 * the same behavior.  See the documentation of `struct i3c_msg` for
 * limitations on support for multi-message bus transactions.
 *
 * @param i2c_dev I2C device descriptor.
 * @param msgs Array of messages to transfer.
 * @param num_msgs Number of messages to transfer.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 * @retval -ENOSYS If I2C transfers are not supported by the controller.
 */
__syscall int i3c_i2c_transfer(struct i3c_i2c_device_desc *i2c_dev,
			       struct i2c_msg *msgs, uint8_t num_msgs);

static inline int z_impl_i3c_i2c_transfer(struct i3c_i2c_device_desc *i2c_dev,
					  struct i2c_msg *msgs, uint8_t num_msgs)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)i2c_dev->bus->api;

	if (api->i3c_i2c_xfers == NULL) {
		return -ENOSYS;
	}

	return api->i3c_i2c_xfers(i2c_dev->bus, i2c_dev, msgs, num_msgs);
}

/**
 * @brief Register an I2C device with the I3C controller driver instance.
 *
 * This makes the controller driver instance aware of a new I2C device,
 * and is inserted into the controller's connected device list.
 *
 * @param i2c_dev I2C target device descriptor.
 *
 * @retval 0 If successful.
 * @retval -EINVAL Device address is reserved or already assigned.
 * @retval -EIO General input / output error.
 * @retval -ENOSYS If the controller does not allow I2C devices on bus.
 */
static inline int i3c_i2c_device_register(struct i3c_i2c_device_desc *i2c_dev)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)i2c_dev->bus->api;

	if (api->i3c_i2c_device_register == NULL) {
		return -ENOSYS;
	}

	return api->i3c_i2c_device_register(i2c_dev->bus, i2c_dev);
}

/*
 * Derived I2C APIs with I3C controller
 * -- all implemented in terms of i3c_i2c_transfer()
 */

/**
 * @brief Write a set amount of data to an I2C device.
 *
 * This routine writes a set amount of data synchronously.
 *
 * @param i2c_dev I2C device descriptor.
 * @param buf Memory pool from which the data is transferred.
 * @param num_bytes Number of bytes to write.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_write(struct i3c_i2c_device_desc *i2c_dev,
				const uint8_t *buf, uint32_t num_bytes)
{
	struct i2c_msg msg;

	msg.buf = (uint8_t *)buf;
	msg.len = num_bytes;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i3c_i2c_transfer(i2c_dev, &msg, 1);
}

/**
 * @brief Read a set amount of data from an I2C device.
 *
 * This routine reads a set amount of data synchronously.
 *
 * @param i2c_dev I2C device descriptor.
 * @param buf Memory pool that stores the retrieved data.
 * @param num_bytes Number of bytes to read.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_read(struct i3c_i2c_device_desc *i2c_dev,
			       uint8_t *buf, uint32_t num_bytes)
{
	struct i2c_msg msg;

	msg.buf = buf;
	msg.len = num_bytes;
	msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i3c_i2c_transfer(i2c_dev, &msg, 1);
}

/**
 * @brief Write then read data from an I2C device.
 *
 * This supports the common operation "this is what I want", "now give
 * it to me" transaction pair through a combined write-then-read bus
 * transaction.
 *
 * @param i2c_dev I2C device descriptor.
 * @param write_buf Pointer to the data to be written
 * @param num_write Number of bytes to write
 * @param read_buf Pointer to storage for read data
 * @param num_read Number of bytes to read
 *
 * @retval 0 if successful
 * @retval negative on error.
 */
static inline int i3c_i2c_write_read(struct i3c_i2c_device_desc *i2c_dev,
				     const void *write_buf, size_t num_write,
				     void *read_buf, size_t num_read)
{
	struct i2c_msg msg[2];

	msg[0].buf = (uint8_t *)write_buf;
	msg[0].len = num_write;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)read_buf;
	msg[1].len = num_read;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return i3c_i2c_transfer(i2c_dev, msg, 2);
}

/**
 * @brief Read multiple bytes from an internal address of an I2C device.
 *
 * This routine reads multiple bytes from an internal address of an
 * I2C device synchronously.
 *
 * Instances of this may be replaced by i2c_write_read().
 *
 * @param i2c_dev I2C target device descriptor,
 * @param start_addr Internal address from which the data is being read.
 * @param buf Memory pool that stores the retrieved data.
 * @param num_bytes Number of bytes being read.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_burst_read(struct i3c_i2c_device_desc *i2c_dev,
				     uint8_t start_addr,
				     uint8_t *buf,
				     uint32_t num_bytes)
{
	return i3c_i2c_write_read(i2c_dev,
				  &start_addr, sizeof(start_addr),
				  buf, num_bytes);
}

/**
 * @brief Write multiple bytes to an internal address of an I2C device.
 *
 * This routine writes multiple bytes to an internal address of an
 * I2C device synchronously.
 *
 * @warning The combined write synthesized by this API may not be
 * supported on all I2C devices.  Uses of this API may be made more
 * portable by replacing them with calls to i2c_write() passing a
 * buffer containing the combined address and data.
 *
 * @param i2c_dev I2C device descriptor.
 * @param start_addr Internal address to which the data is being written.
 * @param buf Memory pool from which the data is transferred.
 * @param num_bytes Number of bytes being written.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_burst_write(struct i3c_i2c_device_desc *i2c_dev,
				      uint8_t start_addr,
				      const uint8_t *buf,
				      uint32_t num_bytes)
{
	struct i2c_msg msg[2];

	msg[0].buf = &start_addr;
	msg[0].len = 1U;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_WRITE | I2C_MSG_STOP;

	return i3c_i2c_transfer(i2c_dev, msg, 2);
}

/**
 * @brief Read internal register of an I2C device.
 *
 * This routine reads the value of an 8-bit internal register of an I2C
 * device synchronously.
 *
 * @param i2c_dev I2C device descriptor.
 * @param reg_addr Address of the internal register being read.
 * @param value Memory pool that stores the retrieved register value.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_reg_read_byte(struct i3c_i2c_device_desc *i2c_dev,
					uint8_t reg_addr, uint8_t *value)
{
	return i3c_i2c_write_read(i2c_dev,
				  &reg_addr, sizeof(reg_addr),
				  value, sizeof(*value));
}

/**
 * @brief Write internal register of an I2C device.
 *
 * This routine writes a value to an 8-bit internal register of an I2C
 * device synchronously.
 *
 * @note This function internally combines the register and value into
 * a single bus transaction.
 *
 * @param i2c_dev I2C device descriptor.
 * @param reg_addr Address of the internal register being written.
 * @param value Value to be written to internal register.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_reg_write_byte(struct i3c_i2c_device_desc *i2c_dev,
					 uint8_t reg_addr, uint8_t value)
{
	uint8_t tx_buf[2] = {reg_addr, value};

	return i3c_i2c_write(i2c_dev, tx_buf, 3);
}

/**
 * @brief Update internal register of an I2C device.
 *
 * This routine updates the value of a set of bits from an 8-bit internal
 * register of an I2C device synchronously.
 *
 * @note If the calculated new register value matches the value that
 * was read this function will not generate a write operation.
 *
 * @param i2c_dev I2C device descriptor.
 * @param reg_addr Address of the internal register being updated.
 * @param mask Bitmask for updating internal register.
 * @param value Value for updating internal register.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_reg_update_byte(struct i3c_i2c_device_desc *i2c_dev,
					  uint8_t reg_addr, uint8_t mask,
					  uint8_t value)
{
	uint8_t old_value, new_value;
	int rc;

	rc = i3c_i2c_reg_read_byte(i2c_dev, reg_addr, &old_value);
	if (rc != 0) {
		return rc;
	}

	new_value = (old_value & ~mask) | (value & mask);
	if (new_value == old_value) {
		return 0;
	}

	return i3c_i2c_reg_write_byte(i2c_dev, reg_addr, new_value);
}


/**
 * @brief Dump out an I3C message
 *
 * Dumps out a list of I3C messages. For any that are writes (W), the data is
 * displayed in hex.
 *
 * It looks something like this (with name "testing"):
 *
 * D: I3C msg: testing, addr=56
 * D:    W len=01:
 * D: contents:
 * D: 06                      |.
 * D:    W len=0e:
 * D: contents:
 * D: 00 01 02 03 04 05 06 07 |........
 * D: 08 09 0a 0b 0c 0d       |......
 *
 * @param name Name of this dump, displayed at the top.
 * @param msgs Array of messages to dump.
 * @param num_msgs Number of messages to dump.
 * @param target I3C target device descriptor.
 */
void i3c_dump_msgs(const char *name, const struct i3c_msg *msgs,
		   uint8_t num_msgs, struct i3c_device_desc *target);

/**
 * @brief Dump out an I2C message
 *
 * Dumps out a list of I2C messages. For any that are writes (W), the data is
 * displayed in hex.
 *
 * It looks something like this (with name "testing"):
 *
 * D: I2C msg: testing, addr=56
 * D:    W len=01:
 * D: contents:
 * D: 06                      |.
 * D:    W len=0e:
 * D: contents:
 * D: 00 01 02 03 04 05 06 07 |........
 * D: 08 09 0a 0b 0c 0d       |......
 *
 * @param name Name of this dump, displayed at the top.
 * @param msgs Array of messages to dump.
 * @param num_msgs Number of messages to dump.
 * @param i2c_dev I2C device descriptor.
 */
static inline void i3c_i2c_dump_msgs(const char *name,
				     const struct i2c_msg *msgs,
				     uint8_t num_msgs,
				     struct i3c_i2c_device_desc *i2c_dev)
{
	i2c_dump_msgs(name, msgs, num_msgs, i2c_dev->addr);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/i3c.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_I3C_H_ */
