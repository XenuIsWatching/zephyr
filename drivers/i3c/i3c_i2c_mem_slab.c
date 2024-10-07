/*
 * Copyright (c) 2024 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>

#include <zephyr/drivers/i3c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(i3c, CONFIG_I3C_LOG_LEVEL);

K_MEM_SLAB_DEFINE(i3c_i2c_device_desc_pool, sizeof(struct i3c_i2c_device_desc),
	CONFIG_I3C_I2C_NUM_OF_DESC_MEM_SLABS, 4);

struct i3c_i2c_device_desc *i3c_i2c_device_desc_alloc(void)
{
	struct i3c_i2c_device_desc *desc;

	if (k_mem_slab_alloc(&i3c_i2c_device_desc_pool, (void **)&desc, K_NO_WAIT) == 0) {
		memset(desc, 0, sizeof(struct i3c_i2c_device_desc));
		LOG_INF("I2C Device Desc allocated - %d free",
			k_mem_slab_num_free_get(&i3c_i2c_device_desc_pool));
	} else {
		LOG_WRN("No memory left for I2C descriptors");
	}

	return desc;
}

void i3c_i2c_device_desc_free(struct i3c_i2c_device_desc *desc)
{
	k_mem_slab_free(&i3c_i2c_device_desc_pool, (void *)desc);
	LOG_DBG("I2C Device Desc freed");
}

bool i3c_i2c_device_desc_in_pool(struct i3c_i2c_device_desc *desc)
{
	const char *p =  (const char *)desc;
	ptrdiff_t offset = p - i3c_i2c_device_desc_pool.buffer;

	return (offset >= 0) &&
	       (offset < (i3c_i2c_device_desc_pool.info.block_size *
		   i3c_i2c_device_desc_pool.info.num_blocks)) &&
	       ((offset % i3c_i2c_device_desc_pool.info.block_size) == 0);
}
