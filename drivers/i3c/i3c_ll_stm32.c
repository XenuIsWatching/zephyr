#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <stm32_ll_i3c.h>
#include <stm32_ll_rcc.h>
#include <errno.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/pinctrl.h>

/* Driver config */
struct cdns_i3c_config {
	struct i3c_driver_config common;
	/** base address of the controller */
	I3C_TypeDef *base;
	/** Interrupt configuration function. */
	void (*irq_config_func)(const struct device *dev);
};

/* Driver instance data */
struct cdns_i3c_data {
	struct i3c_driver_data common;
};

/**
 * @brief Initialize the hardware.
 *
 * @param dev Pointer to controller device driver instance.
 */
static int stm32_i3c_bus_init(const struct device *dev)
{
	return 0;
}

static struct i3c_driver_api api = {
	// .i2c_api.configure = stm32_i3c_i2c_api_configure,
	// .i2c_api.transfer = stm32_i3c_i2c_api_transfer,

	// .configure = stm32_i3c_configure,
	// .config_get = stm32_i3c_config_get,

	// .do_daa = stm32_i3c_do_daa,
	// .do_ccc = stm32_i3c_do_ccc,

	// .i3c_device_find = stm32_i3c_device_find,

	// .i3c_xfers = stm32_i3c_transfer,

#ifdef CONFIG_I3C_USE_IBI
	// .ibi_enable = stm32_i3c_controller_ibi_enable,
	// .ibi_disable = stm32_i3c_controller_ibi_disable,
#endif
};

#define ST_STM32_I3C_INSTANTIATE(n)                                                                \
	static const struct stm32_pclken pclken_##index[] = STM32_DT_INST_CLOCKS(index);           \
	static void stm32_i3c_config_func_##n(const struct device *dev);                           \
	static struct i3c_device_desc stm32_i3c_device_array_##n[] = I3C_DEVICE_ARRAY_DT_INST(n);  \
	static struct i3c_i2c_device_desc stm32_i3c_i2c_device_array_##n[] =                       \
		I3C_I2C_DEVICE_ARRAY_DT_INST(n);                                                   \
	static const struct stm32_i3c_config i3c_config_##n = {                                    \
		.base = (I3C_TypeDef *)DT_INST_REG_ADDR(n),                                        \
		.pclken = pclken_##index,                                                          \
		.pclk_len = DT_INST_NUM_CLOCKS(index),                                             \
		.irq_config_func = stm32_i3c_config_func_##n,                                      \
		.common.dev_list.i3c = stm32_i3c_device_array_##n,                                 \
		.common.dev_list.num_i3c = ARRAY_SIZE(stm32_i3c_device_array_##n),                 \
		.common.dev_list.i2c = stm32_i3c_i2c_device_array_##n,                             \
		.common.dev_list.num_i2c = ARRAY_SIZE(stm32_i3c_i2c_device_array_##n),             \
	};                                                                                         \
	static struct stm32_i3c_data i3c_data_##n = {                                              \
		.common.ctrl_config.scl.i3c = DT_INST_PROP_OR(n, i3c_scl_hz, 0),                   \
		.common.ctrl_config.scl.i2c = DT_INST_PROP_OR(n, i2c_scl_hz, 0),                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, stm32_i3c_bus_init, NULL, &i3c_data_##n, &i3c_config_##n,         \
			      POST_KERNEL, CONFIG_I3C_CONTROLLER_INIT_PRIORITY, &api);             \
	static void stm32_i3c_config_func_##n(const struct device *dev)                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), stm32_i3c_irq_handler,      \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};

#define DT_DRV_COMPAT st, stm32_i3c
DT_INST_FOREACH_STATUS_OKAY(ST_STM32_I3C_INSTANTIATE)