/*
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright (c) 2019 NXP
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_mcux_i3c

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i3c.h>

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

#include <fsl_i3c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i3c_mcux, CONFIG_I3C_MCUX_LOG_LEVEL);

struct mcux_i3c_config {
	I3C_Type *base;

	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;

	uint32_t i3c_pp_scl_hz;
	uint32_t i3c_od_scl_hz;
	uint32_t i2c_scl_hz;

	uint8_t clk_div;
	uint8_t clk_div_slow;
	uint8_t clk_div_tc;

#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pincfg;
#endif

	void (*irq_config_func)(const struct device *dev);
};

struct mcux_i3c_data {
	i3c_master_handle_t handle;
	struct k_sem device_sync_sem;
	status_t callback_status;

	struct i3c_dev_list device_list;
};

static int mcux_i3c_dev_register(const struct device *dev,
				 struct i3c_device_desc *desc)
{
	struct mcux_i3c_data *data = dev->data;
	struct i3c_dev_list *dev_list = &data->device_list;
	struct i3c_addr_slots *slots = &data->device_list.addr_slots;

	/*
	 * If there is a static address for the I3C devices, check
	 * if this address is free, and there is no other devices of
	 * the same (pre-assigned) address on the bus.
	 */
	if (desc->static_addr != 0U) {
		if (i3c_addr_slots_status_is_free(slots, desc->static_addr)) {
			/* Mark address slot as I3C device */
			i3c_addr_slots_status_set(slots, desc->static_addr,
						  I3C_ADDR_SLOT_STATUS_I3C_DEV);
		} else {
			/* Address slot is not free */
			__ASSERT_NO_MSG(false);
			return -EINVAL;
		}
	}

	i3c_dev_list_target_add(dev_list, desc);

	LOG_DBG("I3C device registered: %p", desc);

	return 0;
}

static int mcux_i3c_i2c_dev_register(const struct device *dev,
				     struct i3c_i2c_device_desc *desc)
{
	struct mcux_i3c_data *data = dev->data;
	struct i3c_dev_list *dev_list = &data->device_list;
	struct i3c_addr_slots *slots = &data->device_list.addr_slots;

	/*
	 * Check if the I2C device address is free, and there is
	 * no other devices of the same (pre-assigned) address
	 * on the bus.
	 */
	if (!i3c_addr_slots_status_is_free(slots, desc->addr)) {
		__ASSERT_NO_MSG(false);
		return -EINVAL;
	}

	/* Mark address slot as I2C device */
	i3c_addr_slots_status_set(slots, desc->addr,
				  I3C_ADDR_SLOT_STATUS_I2C_DEV);

	i3c_dev_list_i2c_devce_add(dev_list, desc);

	LOG_DBG("I2C device registered: %p", desc);

	return 0;
}

static void mcux_i3c_isr(const struct device *dev)
{
	const struct mcux_i3c_config *config = dev->config;
	struct mcux_i3c_data *data = dev->data;
	I3C_Type *base = config->base;

	I3C_MasterTransferHandleIRQ(base, &data->handle);
}

static void mcux_i3c_ctrl_transfer_callback(I3C_Type *base,
					    i3c_master_handle_t *handle,
					    status_t status, void *userData)
{
	struct mcux_i3c_data *data = userData;

	ARG_UNUSED(handle);
	ARG_UNUSED(base);

	data->callback_status = status;
	k_sem_give(&data->device_sync_sem);
}

/**
 * @brief Convert status to printable string.
 *
 * @param status Status value
 *
 * @return String associated with the @p status.
 */
static const char *mcux_i3c_cb_status_str_get(status_t status)
{
	switch (status) {
	case kStatus_I3C_Busy:
		return "Controller is busy";

	case kStatus_I3C_Nak:
		return "Target NACK";

	case kStatus_I3C_WriteAbort:
		return "Write abort";

	case kStatus_I3C_Term:
		return "Controller terminates read";

	case kStatus_I3C_HdrParityError:
		return "Parity error from DDR read";

	case kStatus_I3C_CrcError:
		return "CRC error from DDR read.";

	case kStatus_I3C_ReadFifoError:
		return "Read while RX FIFO is empty.";

	case kStatus_I3C_WriteFifoError:
		return "Write while TX FIFO is full.";

	case kStatus_I3C_MsgError:
		return "Read/write message in wrong state.";

	case kStatus_I3C_InvalidReq:
		return "Invalid request.";

	case kStatus_I3C_Timeout:
		return "Timeout.";

	case kStatus_I3C_SlaveCountExceed:
		/* >= I3C_MAX_DEVCNT */
		return "Exceed maximum device count.";

	case kStatus_I3C_IBIWon:
		return "IBI event won arbitration.";

	case kStatus_I3C_OverrunError:
		return "Buffer/FIFO Overrun.";

	case kStatus_I3C_UnderrunError:
		return "Buffer/FIFO Underrun.";

	case kStatus_I3C_UnderrunNak:
		return "Buffer/FIFO Underrun and NACK.";

	case kStatus_I3C_InvalidStart:
		return "Invalid start flag.";

	case kStatus_I3C_SdrParityError:
		return "SDR parity error.";

	case kStatus_I3C_S0S1Error:
		return "S0 or S1 error.";

	default:
		return "Unknown status.";
	}
}

static int mcux_i3c_do_one_transfer(I3C_Type *base,
				    struct mcux_i3c_data *data,
				    i3c_master_transfer_t *transfer)
{
	status_t status;
	uint32_t errStatus;

	/* Start the transfer */
	status = I3C_MasterTransferNonBlocking(base, &data->handle, transfer);

	/* Return an error if the transfer didn't start successfully
	 * e.g., if the bus was busy
	 */
	if (status != kStatus_Success) {
		errStatus = I3C_MasterGetErrorStatusFlags(base);
		LOG_ERR("xfer error: %s (0x%x)",
			mcux_i3c_cb_status_str_get(status), errStatus);

		goto io_err_out;
	}

	/* Wait for the transfer to complete */
	k_sem_take(&data->device_sync_sem, K_FOREVER);

	/* Return an error if the transfer didn't complete
	 * successfully. e.g., nak, timeout, lost arbitration
	 */
	if (data->callback_status != kStatus_Success) {
		LOG_ERR("xfer cb error: %s",
			mcux_i3c_cb_status_str_get(data->callback_status));

		goto io_err_out;
	}

	return 0;

io_err_out:
	I3C_MasterTransferAbort(base, &data->handle);
	return -EIO;
}

static uint32_t mcux_i3c_convert_flags(int msg_flags)
{
	uint32_t flags = 0U;

	if ((msg_flags & I3C_MSG_STOP) != I3C_MSG_STOP) {
		flags |= kI3C_TransferNoStopFlag;
	}

	if ((msg_flags & I3C_MSG_RESTART) == I3C_MSG_RESTART) {
		flags |= kI3C_TransferRepeatedStartFlag;
	}

	return flags;
}

static int mcux_i3c_transfer(const struct device *dev,
			     struct i3c_device_desc *target,
			     struct i3c_msg *msgs,
			     uint8_t num_msgs)
{
	const struct mcux_i3c_config *config = dev->config;
	struct mcux_i3c_data *data = dev->data;
	I3C_Type *base = config->base;
	i3c_master_transfer_t transfer;
	int ret;

	if (target->dynamic_addr == 0U) {
		return -EINVAL;
	}

	/* Iterate over all the messages */
	for (int i = 0; i < num_msgs; i++) {
		/* Initialize the transfer descriptor */
		transfer.flags = mcux_i3c_convert_flags(msgs->flags);

		/* Prevent the controller to send a start condition between
		 * messages, except if explicitly requested.
		 */
		if (i != 0 &&
		    ((msgs->flags & I3C_MSG_RESTART) != I3C_MSG_RESTART)) {
			transfer.flags |= kI3C_TransferNoStartFlag;
		}

		transfer.slaveAddress = target->dynamic_addr;
		transfer.direction = (msgs->flags & I3C_MSG_READ)
			? kI3C_Read : kI3C_Write;
		transfer.subaddress = 0;
		transfer.subaddressSize = 0;
		transfer.data = msgs->buf;
		transfer.dataSize = msgs->len;

		/* I3C SDR transfer */
		transfer.busType = kI3C_TypeI3CSdr;

		ret = mcux_i3c_do_one_transfer(base, data, &transfer);
		if (ret != 0U) {
			break;
		}

		/* Move to the next message */
		msgs++;
	}

	if (ret != 0U) {
		status_t status;

		LOG_ERR("sending STOP due error");
		status = I3C_MasterStop(base);
		if (status != kStatus_Success) {
			LOG_ERR("emit STOP error: %s",
				mcux_i3c_cb_status_str_get(status));
		}
	}

	return ret;
}

static uint32_t mcux_i3c_i2c_convert_flags(int msg_flags)
{
	uint32_t flags = 0U;

	/*
	 * Note that although this is a I2C transfer,
	 * the HAL API still takes I3C transfer flags.
	 */

	if (!(msg_flags & I2C_MSG_STOP)) {
		flags |= kI3C_TransferNoStopFlag;
	}

	if (msg_flags & I2C_MSG_RESTART) {
		flags |= kI3C_TransferRepeatedStartFlag;
	}

	return flags;
}

static int mcux_i3c_i2c_transfer(const struct device *dev,
				 struct i3c_i2c_device_desc *i2c_dev,
				 struct i2c_msg *msgs,
				 uint8_t num_msgs)
{
	const struct mcux_i3c_config *config = dev->config;
	struct mcux_i3c_data *data = dev->data;
	I3C_Type *base = config->base;
	i3c_master_transfer_t transfer;
	int ret;

	/* Iterate over all the messages */
	for (int i = 0; i < num_msgs; i++) {
		/* Initialize the transfer descriptor */
		transfer.flags = mcux_i3c_i2c_convert_flags(msgs->flags);

		/* Prevent the controller to send a start condition between
		 * messages, except if explicitly requested.
		 */
		if (i != 0 &&
		    ((msgs->flags & I2C_MSG_RESTART) != I2C_MSG_RESTART)) {
			transfer.flags |= kI3C_TransferNoStartFlag;
		}

		transfer.slaveAddress = i2c_dev->addr;
		transfer.direction = (msgs->flags & I2C_MSG_READ)
			? kI3C_Read : kI3C_Write;
		transfer.subaddress = 0;
		transfer.subaddressSize = 0;
		transfer.data = msgs->buf;
		transfer.dataSize = msgs->len;

		/* I2C transfer */
		transfer.busType = kI3C_TypeI2C;

		ret = mcux_i3c_do_one_transfer(base, data, &transfer);

		/* Move to the next message */
		msgs++;
	}

	return 0;
}

static int mcux_i3c_do_daa(const struct device *dev)
{
	const struct mcux_i3c_config *config = dev->config;
	struct mcux_i3c_data *data = dev->data;
	I3C_Type *base = config->base;
	int ret = 0;
	status_t result = kStatus_Success;
	uint32_t status;
	uint32_t errStatus;
	size_t rxCount;
	uint8_t rxBuffer[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
	uint8_t rxSize = 0;
	uint32_t enabledInts;

	/* Return an error if the bus is already in use not by us. */
	result = I3C_CheckForBusyBus(base);
	if (kStatus_Success != result) {
		return result;
	}

	/* Clear all flags. */
	I3C_MasterClearErrorStatusFlags(base, kI3C_MasterAllErrorFlags);
	I3C_MasterClearStatusFlags(base, kI3C_MasterClearFlags);

	/* Disable I3C IRQ sources while we configure stuff. */
	enabledInts = I3C_MasterGetEnabledInterrupts(base);
	I3C_MasterDisableInterrupts(base, enabledInts);

	/* Emit process DAA */
	I3C_MasterEmitRequest(base, kI3C_RequestProcessDAA);

	/* Loop until no more responses from devices */
	do {
		/* Loop to grab data from devices (Provisioned ID, BCR and DCR) */
		do {
			status = I3C_MasterGetStatusFlags(base);
			I3C_MasterGetFifoCounts(base, &rxCount, NULL);

			/* Check for error flags. */
			errStatus = I3C_MasterGetErrorStatusFlags(base);

			result = I3C_MasterCheckAndClearError(base, errStatus);
			if (result != kStatus_Success) {
				LOG_ERR("DAA recv error: %s",
					mcux_i3c_cb_status_str_get(result));
				ret = -EIO;
				goto daa_out;
			}

			while (rxCount > 0U) {
				if ((status & (uint32_t)kI3C_MasterRxReadyFlag) != 0U) {
					rxBuffer[rxSize] = (uint8_t)(base->MRDATAB &
								     I3C_MRDATAB_VALUE_MASK);
					rxSize++;
					rxCount--;
				}
			}
		} while ((status & (uint32_t)kI3C_MasterControlDoneFlag) !=
			 (uint32_t)kI3C_MasterControlDoneFlag);

		I3C_MasterClearStatusFlags(base, (uint32_t)kI3C_MasterControlDoneFlag);

		/* Figure out what address to assign to device */
		if ((I3C_MasterGetState(base) == kI3C_MasterStateDaa) &&
		    ((I3C_MasterGetStatusFlags(base) & (uint32_t)kI3C_MasterBetweenFlag) != 0U)) {
			struct i3c_device_desc *target;
			uint32_t vendor_id, part_no;
			uint64_t pid;
			uint8_t dyn_addr;

			rxSize = 0;

			/* Vendor ID portion of Provisioned ID */
			vendor_id = (((uint16_t)rxBuffer[0] << 8U) | (uint16_t)rxBuffer[1]) &
				    0xFFFEU;

			/* Part Number portion of Provisioned ID */
			part_no = (uint32_t)rxBuffer[2] << 24U | (uint32_t)rxBuffer[3] << 16U |
				  (uint32_t)rxBuffer[4] << 8U | (uint32_t)rxBuffer[5];

			/* ... Combined into one Provisioned ID */
			pid = (uint64_t)vendor_id << 32U | (uint64_t)part_no;

			target = i3c_dev_list_find_pid(&data->device_list, pid);
			if (target == NULL) {
				/*
				 * Provisioned ID does not match any devices on
				 * the registered device list.
				 */
				ret = -ENODEV;

				LOG_DBG("PID 0x%llx is not in registered device list", pid);

				goto daa_out;
			}

			if (target->static_addr != 0U) {
				/* Assign the static address as dynamic address */
				dyn_addr = target->static_addr;
			} else {
				/*
				 * No pre-defined static address: find the next
				 * available address
				 */
				dyn_addr = i3c_addr_slots_status_next_free_find(
						&data->device_list.addr_slots);

				if (dyn_addr == 0U) {
					/* No free addresses available */
					ret = -ENOSPC;
					goto daa_out;
				}
			}

			target->dynamic_addr = dyn_addr;
			target->bcr = rxBuffer[6];
			target->dcr = rxBuffer[7];

			/* Emit process DAA again to send the address to the device */
			base->MWDATAB = dyn_addr << 1;
			I3C_MasterEmitRequest(base, kI3C_RequestProcessDAA);
		}

	} while ((status & (uint32_t)kI3C_MasterCompleteFlag) != (uint32_t)kI3C_MasterCompleteFlag);

daa_out:
	/* Clear all flags. */
	I3C_MasterClearErrorStatusFlags(base, kI3C_MasterAllErrorFlags);
	I3C_MasterClearStatusFlags(base, kI3C_MasterClearFlags);

	/* Re-Enable I3C IRQ sources. */
	I3C_MasterEnableInterrupts(base, enabledInts);

	return ret;
}

static int mcux_i3c_do_ccc(const struct device *dev,
			   struct i3c_ccc_payload *payload)
{
	const struct mcux_i3c_config *config = dev->config;
	struct mcux_i3c_data *data = dev->data;
	I3C_Type *base = config->base;
	i3c_master_transfer_t xfer;
	status_t status;
	int ret = 0;

	__ASSERT_NO_MSG(payload != NULL);

	LOG_DBG("CCC[0x%02x]", payload->ccc.id);

	memset(&xfer, 0, sizeof(xfer));

	/* Set once for all subsequent transfers */
	xfer.busType = kI3C_TypeI3CSdr;

	if (i3c_ccc_is_payload_broadcast(payload)) {
		xfer.slaveAddress = I3C_BROADCAST_ADDR;
		xfer.subaddress = payload->ccc.id;
		xfer.subaddressSize = 1U;
		xfer.data = payload->ccc.data;
		xfer.dataSize = payload->ccc.data_len;
		xfer.direction = kI3C_Write;
		xfer.flags = (uint32_t)kI3C_TransferDefaultFlag;

		ret = mcux_i3c_do_one_transfer(base, data, &xfer);
		if (ret != 0U) {
			LOG_ERR("CCC[0x%02x] broadcast command error (%d)",
				payload->ccc.id, ret);
		}
	} else {
		int idx;

		/* Send the CCC command first */
		xfer.slaveAddress = I3C_BROADCAST_ADDR;
		xfer.subaddress = payload->ccc.id;
		xfer.subaddressSize = 1U;
		xfer.data = payload->ccc.data;
		xfer.dataSize = payload->ccc.data_len;
		xfer.direction = kI3C_Write;
		xfer.flags = (uint32_t)kI3C_TransferNoStopFlag;

		ret = mcux_i3c_do_one_transfer(base, data, &xfer);
		if (ret != 0U) {
			LOG_ERR("CCC[0x%02x] direct command error (%d)",
				payload->ccc.id, ret);
		}

		/* Clear this as we are not sending the CCC code anymore */
		xfer.subaddress = 0U;
		xfer.subaddressSize = 0U;

		/* Continue if command is sent successfully. */
		if (ret == 0U) {
			/* Then RESTART and the payload for each target */
			for (idx = 0; idx < payload->targets.num_targets; idx++) {
				struct i3c_ccc_target_payload *tgt_payload =
					&payload->targets.payloads[idx];

				xfer.slaveAddress = tgt_payload->addr;
				xfer.data = tgt_payload->data;
				xfer.dataSize = tgt_payload->data_len;
				xfer.direction = (tgt_payload->rnw == 1U) ? kI3C_Read : kI3C_Write;
				xfer.flags = (uint32_t)kI3C_TransferRepeatedStartFlag;

				if (idx < (payload->targets.num_targets - 1)) {
					/* Do not send STOP in between targets */
					xfer.flags |= (uint32_t)kI3C_TransferNoStopFlag;
				}

				ret = mcux_i3c_do_one_transfer(base, data, &xfer);
				if (ret != 0U) {
					LOG_ERR("CCC[0x%02x] target error (%d)",
						payload->ccc.id, ret);

					/*
					 * The HAL code emits STOP when error.
					 * So we don't have to.
					 */
					break;
				}
			}
		}
	}

	if (ret != 0U) {
		LOG_ERR("CCC[0x%02x] sending STOP due error", payload->ccc.id);
		status = I3C_MasterStop(base);
		if (status != kStatus_Success) {
			LOG_ERR("CCC[0x%02x]: emit STOP error: %s",
				payload->ccc.id,
				mcux_i3c_cb_status_str_get(status));
		}
	}

	return ret;
}

#ifdef CONFIG_I3C_USE_IBI
static void mcux_i3c_ibi_cb(I3C_Type *base,
			       i3c_master_handle_t *handle,
			       i3c_ibi_type_t ibiType,
			       i3c_ibi_state_t ibiState)
{
}
#endif

static i3c_master_transfer_callback_t mcux_i3c_ctrl_xfer_cb = {
	.ibiCallback = COND_CODE_1(CONFIG_I3C_USE_IBI,
				   (mcux_i3c_ibi_cb),
				   (NULL)),
	.transferComplete = mcux_i3c_ctrl_transfer_callback,
};

/**
 * @brief Do RSTDAA to reset dynamic addresses of all devices on bus.
 *
 * @param dev Pointer to the device driver instance.
 */
static void mcux_i3c_do_rstact_peripheral_all(const struct device *dev)
{
	const struct mcux_i3c_config *config = dev->config;
	I3C_Type *base = config->base;

	LOG_DBG("Broadcast RSTACT");

	/*
	 * Note that we ignore error from RSTACT as there may not be
	 * any connected devices responding to this CCC. So we clear
	 * any error bits anyway.
	 */
	if (i3c_ccc_do_rstact_peripheral_all(dev) != 0U) {
		LOG_DBG("Broadcast RSTACT was NACK.");
	}

	I3C_MasterClearErrorStatusFlags(base, kI3C_MasterAllErrorFlags);
}

/**
 * @brief Do RSTDAA to reset dynamic addresses of all devices on bus.
 *
 * @param dev Pointer to the device driver instance.
 */
static void mcux_i3c_do_rstdaa_all(const struct device *dev)
{
	const struct mcux_i3c_config *config = dev->config;
	I3C_Type *base = config->base;

	LOG_DBG("Broadcast RSTDAA");

	/*
	 * Note that we ignore error from RSTDAA as there may not be
	 * any connected devices responding to this CCC (e.g. no
	 * device has already been assigned an address, so no need
	 * to reset). So we clear any error bits anyway.
	 */
	if (i3c_ccc_do_rstdaa_all(dev) != 0U) {
		LOG_DBG("Broadcast RSTDAA was NACK.");
	}

	I3C_MasterClearErrorStatusFlags(base, kI3C_MasterAllErrorFlags);
}

/**
 * @brief Do SETDASA to set static address as dynamic address.
 *
 * @param dev Pointer to the device driver instance.
 * @param[out] True if DAA is still needed. False if all registered
 *             devices have static addresses.
 *
 * @retval 0 if successful.
 */
static int mcux_i3c_do_setdasa(const struct device *dev, bool *need_daa)
{
	const struct mcux_i3c_config *config = dev->config;
	struct mcux_i3c_data *data = dev->data;
	struct i3c_dev_list *dev_list = &data->device_list;
	I3C_Type *base = config->base;

	struct i3c_ccc_payload ccc_payload;
	struct i3c_ccc_target_payload ccc_tgt_payload;
	uint8_t dyn_addr;
	sys_snode_t *node;
	int ret;

	*need_daa = false;

	/* Set up common fields for CCC structs */
	ccc_tgt_payload.rnw = 0;
	ccc_tgt_payload.data = &dyn_addr;
	ccc_tgt_payload.data_len = 1;

	memset(&ccc_payload, 0, sizeof(ccc_payload));
	ccc_payload.ccc.id = I3C_CCC_SETDASA;
	ccc_payload.targets.payloads = &ccc_tgt_payload;
	ccc_payload.targets.num_targets = 1;

	/* Loop through the registered I3C devices */
	SYS_SLIST_FOR_EACH_NODE(&dev_list->devices.i3c, node) {
		struct i3c_device_desc *desc = (void *)node;
		struct i3c_ccc_getbcr bcr;
		struct i3c_ccc_getdcr dcr;

		/*
		 * A device without static address => need to do
		 * dynamic address assignment.
		 */
		if (desc->static_addr == 0U) {
			*need_daa = true;
			continue;
		}

		/*
		 * Note that the 7-bit address needs to start at
		 * bit 0. So shift left by 1 bit.
		 */
		dyn_addr = desc->static_addr << 1;
		ccc_tgt_payload.addr = desc->static_addr;

		LOG_DBG("SETDASA for 0x%x", desc->static_addr);

		ret = mcux_i3c_do_ccc(dev, &ccc_payload);
		if (ret == 0U) {
			/* If successful, set the dynamic address */
			desc->dynamic_addr = desc->static_addr;
		} else {
			LOG_ERR("SETDASA error on address 0x%x (%d)",
				desc->static_addr, ret);
			continue;
		}

		ret = i3c_ccc_do_getbcr(desc, &bcr);
		if (ret == 0U) {
			/* If successful, set the dynamic address */
			desc->bcr = bcr.bcr;

			LOG_DBG("BCR for 0x%x == 0x%x",
				desc->static_addr, bcr.bcr);
		} else {
			LOG_ERR("GETBCR error on address 0x%x (%d)",
				desc->static_addr, ret);
			continue;
		}

		ret = i3c_ccc_do_getdcr(desc, &dcr);
		if (ret == 0U) {
			/* If successful, set the dynamic address */
			desc->dcr = dcr.dcr;

			LOG_DBG("DCR for 0x%x == 0x%x",
				desc->static_addr, dcr.dcr);
		} else {
			LOG_ERR("GETDCR error on address 0x%x (%d)",
				desc->static_addr, ret);
			continue;
		}
	}

	I3C_MasterClearErrorStatusFlags(base, kI3C_MasterAllErrorFlags);

	return 0;
}

static int mcux_i3c_bus_init(const struct device *dev)
{
	const struct mcux_i3c_config *config = dev->config;
	struct mcux_i3c_data *data = dev->data;
	I3C_Type *base;
	i3c_master_config_t ctrl_config;
	uint32_t clock_freq;
	int error;
	int ret = 0;
	bool need_daa = true;

	if (dev == NULL) {
		ret = -ENODEV;
		goto init_out;
	}

	base = config->base;

	CLOCK_SetClkDiv(kCLOCK_DivI3cClk, config->clk_div);
	CLOCK_SetClkDiv(kCLOCK_DivI3cSlowClk, config->clk_div_slow);
	CLOCK_SetClkDiv(kCLOCK_DivI3cTcClk, config->clk_div_tc);

#ifdef CONFIG_PINCTRL
	error = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (error) {
		ret = error;
		goto init_out;
	}
#endif

	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);

	/* Get the clock frequency */
	if (clock_control_get_rate(config->clock_dev, config->clock_subsys,
				   &clock_freq)) {
		ret = -EINVAL;
		goto init_out;
	}

	/*
	 * Default controller configuration to act as the primary
	 * and active controller.
	 */
	I3C_MasterGetDefaultConfig(&ctrl_config);

	/* Set SCL clock rate (in Hz) */
	if (config->i2c_scl_hz != 0) {
		ctrl_config.baudRate_Hz.i2cBaud = config->i2c_scl_hz;
	}

	if (config->i3c_pp_scl_hz != 0) {
		ctrl_config.baudRate_Hz.i3cPushPullBaud = config->i3c_pp_scl_hz;
	}

	if (config->i3c_od_scl_hz != 0) {
		ctrl_config.baudRate_Hz.i3cOpenDrainBaud = config->i3c_od_scl_hz;
	}

	/* Initialize hardware */
	I3C_MasterInit(base, &ctrl_config, clock_freq);

	/* Setup transfer callback */
	I3C_MasterTransferCreateHandle(base, &data->handle,
				       &mcux_i3c_ctrl_xfer_cb,
				       data);

	/*
	 * Reset all connected targets.
	 */
	mcux_i3c_do_rstact_peripheral_all(dev);

	/*
	 * Reset Dynamic Addresses for all devices.
	 * As we have no idea what dynamic addresses the connected
	 * devices have (e.g. assigned during previous power cycle).
	 */
	mcux_i3c_do_rstdaa_all(dev);

	/*
	 * Set static addresses as dynamic addresses.
	 */
	ret = mcux_i3c_do_setdasa(dev, &need_daa);
	if (ret != 0U) {
		goto init_out;
	}

	if (need_daa) {
		/*
		 * Perform Dynamic Address Assignment.
		 */
		ret = mcux_i3c_do_daa(dev);
		if (ret != 0U) {
			goto init_out;
		}
	}

	I3C_MasterClearErrorStatusFlags(base, kI3C_MasterAllErrorFlags);

	/*
	 * Enable IRQ only after any potential operations that might
	 * trigger interrupts are done and flags cleared.
	 */
	config->irq_config_func(dev);

init_out:
	return ret;
}

static int mcux_i3c_init(const struct device *dev)
{
	struct mcux_i3c_data *data = dev->data;

	i3c_dev_list_init(&data->device_list);

	return 0;
}

static const struct i3c_driver_api mcux_i3c_driver_api = {
	.do_daa = mcux_i3c_do_daa,
	.do_ccc = mcux_i3c_do_ccc,

	.i3c_device_register = mcux_i3c_dev_register,
	.i3c_i2c_device_register = mcux_i3c_i2c_dev_register,

	.i3c_xfers = mcux_i3c_transfer,
	.i3c_i2c_xfers = mcux_i3c_i2c_transfer,
};

#ifdef CONFIG_PINCTRL
#define I3C_MCUX_PINCTRL_DEFINE(n) PINCTRL_DT_INST_DEFINE(n);
#define I3C_MCUX_PINCTRL_INIT(n) .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),
#else
#define I3C_MCUX_PINCTRL_DEFINE(n)
#define I3C_MCUX_PINCTRL_INIT(n)
#endif

#define I3C_MCUX_DEVICE(id)							\
	I3C_MCUX_PINCTRL_DEFINE(id)						\
	static void mcux_i3c_config_func_##id(const struct device *dev);	\
	static const struct mcux_i3c_config mcux_i3c_config_##id = {		\
		.base = (I3C_Type *) DT_INST_REG_ADDR(id),			\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(id)),		\
		.clock_subsys =							\
			(clock_control_subsys_t)DT_INST_CLOCKS_CELL(id, name),	\
		.irq_config_func = mcux_i3c_config_func_##id,			\
		.i3c_pp_scl_hz = COND_CODE_1(					\
				DT_INST_NODE_HAS_PROP(id, i3c_scl_hz),		\
				(DT_INST_PROP(id, i3c_scl_hz)),			\
				(0)),						\
		.i3c_od_scl_hz = COND_CODE_1(					\
				DT_INST_NODE_HAS_PROP(id, i3c_od_scl_hz),	\
				(DT_INST_PROP(id, i3c_od_scl_hz)),		\
				(0)),						\
		.i2c_scl_hz = COND_CODE_1(					\
				DT_INST_NODE_HAS_PROP(id, i2c_scl_hz),		\
				(DT_INST_PROP(id, i2c_scl_hz)),			\
				(0)),						\
		.clk_div = DT_INST_PROP(id, clk_divider),			\
		.clk_div_slow = DT_INST_PROP(id, clk_divider_slow),		\
		.clk_div_tc = DT_INST_PROP(id, clk_divider_tc),			\
		I3C_MCUX_PINCTRL_INIT(id)					\
	};									\
	static struct mcux_i3c_data mcux_i3c_data_##id;				\
	I3C_DEVICE_DT_INST_DEFINE(id,						\
				  mcux_i3c_init,				\
				  NULL,						\
				  &mcux_i3c_data_##id,				\
				  &mcux_i3c_config_##id,			\
				  POST_KERNEL,					\
				  CONFIG_I3C_CONTROLLER_INIT_PRIORITY,		\
				  &mcux_i3c_driver_api);			\
	static void mcux_i3c_config_func_##id(const struct device *dev)		\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(id),					\
			    DT_INST_IRQ(id, priority),				\
			    mcux_i3c_isr,					\
			    DEVICE_DT_INST_GET(id),				\
			    0);							\
		irq_enable(DT_INST_IRQN(id));					\
	};									\
	I3C_BUS_INIT(mcux_i3c_bus_init, DEVICE_DT_INST_GET(id));

DT_INST_FOREACH_STATUS_OKAY(I3C_MCUX_DEVICE)
