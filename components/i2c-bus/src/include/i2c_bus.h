// Copyright 2019-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef _I2C_BUS_H_
#define _I2C_BUS_H_
#include "driver/i2c.h"

typedef void *i2c_bus_handle_t; /*!< i2c bus handle */
typedef void *i2c_bus_device_handle_t; /*!< i2c device handle */

#ifdef __cplusplus
extern "C"
{
#endif

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
#define gpio_pad_select_gpio esp_rom_gpio_pad_select_gpio
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

/**************************************** Public Functions (Application level)*********************************************/

esp_err_t i2c_write(i2c_bus_device_handle_t dev_handle, const uint8_t* write_buf, size_t write_size);

esp_err_t i2c_read(i2c_bus_device_handle_t dev_handle, const uint8_t* write_buf, uint8_t* read_buf, size_t read_size);

/**
 * @brief Create an I2C bus instance then return a handle if created successfully. Each I2C bus works in a singleton mode,
 * which means for an i2c port only one group parameter works. When i2c_bus_create is called more than one time for the
 * same i2c port, following parameter will override the previous one.
 *
 * @param port I2C port number
 * @param conf Pointer to I2C bus configuration
 * @return i2c_bus_handle_t Return the I2C bus handle if created successfully, return NULL if failed.
 */
i2c_bus_handle_t i2c_bus_create(i2c_port_t port, const i2c_config_t *conf);

/**
 * @brief Delete and release the I2C bus resource.
 *
 * @param p_bus_handle Point to the I2C bus handle, if delete succeed handle will set to NULL.
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t i2c_bus_delete(i2c_bus_handle_t *p_bus_handle);

/**
 * @brief Get current active clock speed.
 * 
 * @param bus_handle I2C bus handle
 * @return uint32_t current clock speed
 */
uint32_t i2c_bus_get_current_clk_speed(i2c_bus_handle_t bus_handle);

/**
 * @brief Create an I2C device on specific bus.
 *        Dynamic configuration must be enable to achieve multiple devices with different configs on a single bus.
 *        menuconfig:Bus Options->I2C Bus Options->enable dynamic configuration
 * 
 * @param bus_handle Point to the I2C bus handle
 * @param dev_addr i2c device address
 * @param clk_speed device specified clock frequency the i2c_bus will switch to during each transfer. 0 if use current bus speed.
 * @return i2c_bus_device_handle_t return a device handle if created successfully, return NULL if failed.
 */
i2c_bus_device_handle_t i2c_bus_device_create(i2c_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed);

/**
 * @brief Delete and release the I2C device resource, i2c_bus_device_delete should be used in pairs with i2c_bus_device_create.
 *
 * @param p_dev_handle Point to the I2C device handle, if delete succeed handle will set to NULL.
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t i2c_bus_device_delete(i2c_bus_device_handle_t *p_dev_handle);

#ifdef __cplusplus
}
#endif

#endif
