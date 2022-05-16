/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/slist.h>

#include <zephyr/drivers/i3c.h>

#define LOG_LEVEL CONFIG_I3C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i3c);

void i3c_dump_msgs(const char *name, const struct i3c_msg *msgs,
		   uint8_t num_msgs, struct i3c_device_desc *target)
{
	LOG_DBG("I3C msg: %s, addr=%x", name, target->dynamic_addr);
	for (unsigned int i = 0; i < num_msgs; i++) {
		const struct i3c_msg *msg = &msgs[i];

		LOG_DBG("   %c len=%02x: ",
			msg->flags & I3C_MSG_READ ? 'R' : 'W', msg->len);
		if (!(msg->flags & I3C_MSG_READ)) {
			LOG_HEXDUMP_DBG(msg->buf, msg->len, "contents:");
		}
	}
}

void i3c_addr_slots_status_set(struct i3c_addr_slots *slots,
			       uint8_t dev_addr,
			       enum i3c_addr_slot_status status)
{
	int bitpos;
	int idx;

	__ASSERT_NO_MSG(slots != NULL);

	if (dev_addr > I3C_MAX_ADDR) {
		/* Invalid address. Do nothing. */
		return;
	}

	bitpos = dev_addr * 2;
	idx = bitpos / BITS_PER_LONG;

	slots->slots[idx] &= ~((unsigned long)I3C_ADDR_SLOT_STATUS_MASK <<
			       (bitpos % BITS_PER_LONG));
	slots->slots[idx] |= status << (bitpos % BITS_PER_LONG);
}

enum i3c_addr_slot_status
i3c_addr_slots_status_get(struct i3c_addr_slots *slots,
			  uint8_t dev_addr)
{
	unsigned long status;
	int bitpos;
	int idx;

	__ASSERT_NO_MSG(slots != NULL);

	if (dev_addr > I3C_MAX_ADDR) {
		/* Invalid address.
		 * Simply says it's reserved so it will not be
		 * used for anything.
		 */
		return I3C_ADDR_SLOT_STATUS_RSVD;
	}

	bitpos = dev_addr * 2;
	idx = bitpos / BITS_PER_LONG;

	status = slots->slots[idx] >> (bitpos % BITS_PER_LONG);
	status &= I3C_ADDR_SLOT_STATUS_MASK;

	return status;
}


void i3c_addr_slots_init(struct i3c_addr_slots *slots)
{
	int i;

	__ASSERT_NO_MSG(slots != NULL);

	(void)memset(slots, 0, sizeof(*slots));

	for (i = 0; i <= 7; i++) {
		/* Addresses 0 to 7 are reserved */
		i3c_addr_slots_status_set(slots, i, I3C_ADDR_SLOT_STATUS_RSVD);

		/*
		 * Addresses within a single bit error of broadcast address
		 * are also reserved.
		 */
		i3c_addr_slots_status_set(slots, I3C_BROADCAST_ADDR ^ BIT(i),
					  I3C_ADDR_SLOT_STATUS_RSVD);

	}

	/* The broadcast address is reserved */
	i3c_addr_slots_status_set(slots, I3C_BROADCAST_ADDR,
				  I3C_ADDR_SLOT_STATUS_RSVD);
}

bool i3c_addr_slots_status_is_free(struct i3c_addr_slots *slots,
				   uint8_t dev_addr)
{
	enum i3c_addr_slot_status status;

	__ASSERT_NO_MSG(slots != NULL);

	status = i3c_addr_slots_status_get(slots, dev_addr);

	return (status == I3C_ADDR_SLOT_STATUS_FREE);
}

uint8_t i3c_addr_slots_status_next_free_find(struct i3c_addr_slots *slots)
{
	uint8_t addr;
	enum i3c_addr_slot_status status;

	/* Addresses 0 to 7 are reserved. So start at 8. */
	for (addr = 8; addr < I3C_MAX_ADDR; addr++) {
		status = i3c_addr_slots_status_get(slots, addr);
		if (status == I3C_ADDR_SLOT_STATUS_FREE) {
			return addr;
		}
	}

	return 0;
}

void i3c_dev_list_init(struct i3c_dev_list *dev_list)
{
	__ASSERT_NO_MSG(dev_list != NULL);

	i3c_addr_slots_init(&dev_list->addr_slots);
	sys_slist_init(&dev_list->devices.i3c);
	sys_slist_init(&dev_list->devices.i2c);
}

void i3c_dev_list_target_add(struct i3c_dev_list *dev_list,
			     struct i3c_device_desc *target)
{
	sys_snode_t *node;

	__ASSERT_NO_MSG(dev_list != NULL);
	__ASSERT_NO_MSG(target != NULL);

	node = (sys_snode_t *)&target->node;

	sys_slist_append(&dev_list->devices.i3c, node);
}

void i3c_dev_list_i2c_devce_add(struct i3c_dev_list *dev_list,
				struct i3c_i2c_device_desc *i2c_dev)
{
	sys_snode_t *node;

	__ASSERT_NO_MSG(dev_list != NULL);
	__ASSERT_NO_MSG(i2c_dev != NULL);

	node = (sys_snode_t *)&i2c_dev->node;

	sys_slist_append(&dev_list->devices.i2c, node);
}

struct i3c_device_desc *i3c_dev_list_find_pid(struct i3c_dev_list *dev_list,
					      uint64_t pid)
{
	sys_snode_t *node;

	__ASSERT_NO_MSG(dev_list != NULL);

	SYS_SLIST_FOR_EACH_NODE(&dev_list->devices.i3c, node) {
		struct i3c_device_desc *desc = (void *)node;

		if (desc->pid == pid) {
			return desc;
		}
	}

	return NULL;
}

int i3c_ccc_do_getbcr(struct i3c_device_desc *target,
		      struct i3c_ccc_getbcr *bcr)
{
	struct i3c_ccc_payload ccc_payload;
	struct i3c_ccc_target_payload ccc_tgt_payload;

	__ASSERT_NO_MSG(target != NULL);
	__ASSERT_NO_MSG(target->bus != NULL);
	__ASSERT_NO_MSG(bcr != NULL);

	ccc_tgt_payload.rnw = 1;
	ccc_tgt_payload.data = &bcr->bcr;
	ccc_tgt_payload.data_len = sizeof(bcr->bcr);

	memset(&ccc_payload, 0, sizeof(ccc_payload));
	ccc_payload.ccc.id = I3C_CCC_GETBCR;
	ccc_payload.targets.payloads = &ccc_tgt_payload;
	ccc_payload.targets.num_targets = 1;

	ccc_tgt_payload.addr = target->dynamic_addr;

	return i3c_do_ccc(target->bus, &ccc_payload);
}

int i3c_ccc_do_getdcr(struct i3c_device_desc *target,
		      struct i3c_ccc_getdcr *dcr)
{
	struct i3c_ccc_payload ccc_payload;
	struct i3c_ccc_target_payload ccc_tgt_payload;

	__ASSERT_NO_MSG(target != NULL);
	__ASSERT_NO_MSG(target->bus != NULL);
	__ASSERT_NO_MSG(dcr != NULL);

	/* GETBCR */
	ccc_tgt_payload.rnw = 1;
	ccc_tgt_payload.data = &dcr->dcr;
	ccc_tgt_payload.data_len = sizeof(dcr->dcr);

	memset(&ccc_payload, 0, sizeof(ccc_payload));
	ccc_payload.ccc.id = I3C_CCC_GETDCR;
	ccc_payload.targets.payloads = &ccc_tgt_payload;
	ccc_payload.targets.num_targets = 1;

	ccc_tgt_payload.addr = target->dynamic_addr;

	return i3c_do_ccc(target->bus, &ccc_payload);
}

int i3c_ccc_do_getpid(struct i3c_device_desc *target,
		      struct i3c_ccc_getpid *pid)
{
	struct i3c_ccc_payload ccc_payload;
	struct i3c_ccc_target_payload ccc_tgt_payload;

	__ASSERT_NO_MSG(target != NULL);
	__ASSERT_NO_MSG(target->bus != NULL);
	__ASSERT_NO_MSG(pid != NULL);

	ccc_tgt_payload.rnw = 1;
	ccc_tgt_payload.data = &pid->pid[0];
	ccc_tgt_payload.data_len = sizeof(pid->pid);

	memset(&ccc_payload, 0, sizeof(ccc_payload));
	ccc_payload.ccc.id = I3C_CCC_GETPID;
	ccc_payload.targets.payloads = &ccc_tgt_payload;
	ccc_payload.targets.num_targets = 1;

	ccc_tgt_payload.addr = target->dynamic_addr;

	return i3c_do_ccc(target->bus, &ccc_payload);
}

int i3c_ccc_do_rstdaa_all(const struct device *controller)
{
	struct i3c_ccc_payload ccc_payload;

	__ASSERT_NO_MSG(controller != NULL);

	memset(&ccc_payload, 0, sizeof(ccc_payload));
	ccc_payload.ccc.id = I3C_CCC_RSTDAA;

	return i3c_do_ccc(controller, &ccc_payload);
}

int i3c_ccc_do_rstact_peripheral_all(const struct device *controller)
{
	struct i3c_ccc_payload ccc_payload;
	uint8_t def_byte;

	__ASSERT_NO_MSG(controller != NULL);

	memset(&ccc_payload, 0, sizeof(ccc_payload));
	ccc_payload.ccc.id = I3C_CCC_RSTACT(true);

	def_byte = 0x01U;
	ccc_payload.ccc.data = &def_byte;
	ccc_payload.ccc.data_len = 1U;

	return i3c_do_ccc(controller, &ccc_payload);
}

int i3c_ccc_events_broadcast_set(const struct device *controller,
				 bool enable, struct i3c_ccc_events *events)
{
	struct i3c_ccc_payload ccc_payload;

	__ASSERT_NO_MSG(controller != NULL);

	memset(&ccc_payload, 0, sizeof(ccc_payload));

	ccc_payload.ccc.id = enable ? I3C_CCC_ENEC(true) : I3C_CCC_DISEC(true);

	ccc_payload.ccc.data = &events->events;
	ccc_payload.ccc.data_len = sizeof(events->events);

	return i3c_do_ccc(controller, &ccc_payload);
}
