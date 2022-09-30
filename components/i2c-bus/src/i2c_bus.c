// Copyright 2020-2021 Espressif Systems (Shanghai) PTE LTD
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
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "i2c_bus.h"

#define I2C_BUS_FLG_DEFAULT (0)
#define I2C_BUS_MASTER_BUF_LEN (0)
#define I2C_BUS_MS_TO_WAIT (200)
#define I2C_BUS_TICKS_TO_WAIT (I2C_BUS_MS_TO_WAIT/portTICK_RATE_MS)
#define I2C_BUS_MUTEX_TICKS_TO_WAIT (I2C_BUS_MS_TO_WAIT/portTICK_RATE_MS)

typedef struct {
    i2c_port_t i2c_port;    /*!<I2C port number */
    bool is_init;   /*if bus is initialized*/
    i2c_config_t conf_active;    /*!<I2C active configuration */
    SemaphoreHandle_t mutex;    /* mutex to achive thread-safe*/
    int32_t ref_counter;    /*reference count*/
} i2c_bus_t;

typedef struct {
    uint8_t dev_addr;   /*device address*/
    i2c_config_t conf;    /*!<I2C active configuration */
    i2c_bus_t *i2c_bus;    /*!<I2C bus*/
} i2c_bus_device_t;

static const char *TAG = "i2c_bus";
static i2c_bus_t s_i2c_bus[I2C_NUM_MAX];

#define I2C_BUS_CHECK(a, str, ret) if(!(a)) { \
        ESP_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
        return (ret); \
    }

#define I2C_BUS_INIT_CHECK(is_init, ret) if(!is_init) { \
        ESP_LOGE(TAG,"%s:%d (%s):i2c_bus has not inited", __FILE__, __LINE__, __FUNCTION__); \
        return (ret); \
    }

#define I2C_BUS_MUTEX_TAKE(mutex, ret) if (!xSemaphoreTake(mutex, I2C_BUS_MUTEX_TICKS_TO_WAIT)) { \
        ESP_LOGE(TAG, "i2c_bus take mutex timeout, max wait = %d ms", I2C_BUS_MUTEX_TICKS_TO_WAIT); \
        return (ret); \
    }

#define I2C_BUS_MUTEX_TAKE_MAX_DELAY(mutex, ret) if (!xSemaphoreTake(mutex, portMAX_DELAY)) { \
        ESP_LOGE(TAG, "i2c_bus take mutex timeout, max wait = %d ms", portMAX_DELAY); \
        return (ret); \
    }

#define I2C_BUS_MUTEX_GIVE(mutex, ret) if (!xSemaphoreGive(mutex)) { \
        ESP_LOGE(TAG, "i2c_bus give mutex failed"); \
        return (ret); \
    }

static esp_err_t i2c_driver_reinit(i2c_port_t port, const i2c_config_t *conf);
static esp_err_t i2c_driver_deinit(i2c_port_t port);
inline static bool i2c_config_compare(i2c_port_t port, const i2c_config_t *conf);
/**************************************** Public Functions (Application level)*********************************************/

esp_err_t i2c_write(i2c_bus_device_handle_t dev_handle, const uint8_t* write_buf, size_t write_size)
{
    I2C_BUS_CHECK(dev_handle != NULL, "device handle error", ESP_ERR_INVALID_ARG);
    I2C_BUS_CHECK(write_buf != NULL, "data pointer error", ESP_ERR_INVALID_ARG);
    i2c_bus_device_t *i2c_device = (i2c_bus_device_t *)dev_handle;
    I2C_BUS_INIT_CHECK(i2c_device->i2c_bus->is_init, ESP_ERR_INVALID_STATE);
    I2C_BUS_MUTEX_TAKE(i2c_device->i2c_bus->mutex, ESP_ERR_TIMEOUT);
    esp_err_t ret = i2c_master_write_to_device(i2c_device->i2c_bus->i2c_port, i2c_device->dev_addr, 
                                                write_buf, write_size, I2C_BUS_TICKS_TO_WAIT);
    I2C_BUS_MUTEX_GIVE(i2c_device->i2c_bus->mutex, ESP_FAIL);
    return ret;
}

esp_err_t i2c_read(i2c_bus_device_handle_t dev_handle, const uint8_t* write_buf, uint8_t* read_buf, size_t read_size)
{
    I2C_BUS_CHECK(dev_handle != NULL, "device handle error", ESP_ERR_INVALID_ARG);
    I2C_BUS_CHECK(write_buf != NULL, "data pointer error", ESP_ERR_INVALID_ARG);
    i2c_bus_device_t *i2c_device = (i2c_bus_device_t *)dev_handle;
    I2C_BUS_INIT_CHECK(i2c_device->i2c_bus->is_init, ESP_ERR_INVALID_STATE);
    I2C_BUS_MUTEX_TAKE(i2c_device->i2c_bus->mutex, ESP_ERR_TIMEOUT);    
    esp_err_t ret = i2c_master_write_read_device(i2c_device->i2c_bus->i2c_port, i2c_device->dev_addr, 
                                                    write_buf, 1, read_buf, read_size, I2C_BUS_TICKS_TO_WAIT);
    I2C_BUS_MUTEX_GIVE(i2c_device->i2c_bus->mutex, ESP_FAIL);
    return ret;
}

i2c_bus_handle_t i2c_bus_create(i2c_port_t port, const i2c_config_t *conf)
{
    I2C_BUS_CHECK(port < I2C_NUM_MAX, "I2C port error", NULL);
    I2C_BUS_CHECK(conf != NULL, "pointer = NULL error", NULL);
    I2C_BUS_CHECK(conf->mode == I2C_MODE_MASTER, "i2c_bus only supports master mode", NULL);

    if (s_i2c_bus[port].is_init) {
        /**if i2c_bus has been inited and configs not changed, return the handle directly**/
        if (i2c_config_compare(port, conf)) {
            ESP_LOGW(TAG, "i2c%d has been inited, return handle directly, ref_counter=%d", port, s_i2c_bus[port].ref_counter);
            return (i2c_bus_handle_t)&s_i2c_bus[port];
        }
    } else {
        s_i2c_bus[port].mutex = xSemaphoreCreateMutex();
        I2C_BUS_CHECK(s_i2c_bus[port].mutex != NULL, "i2c_bus xSemaphoreCreateMutex failed", NULL);
        s_i2c_bus[port].ref_counter = 0;
    }

    esp_err_t ret = i2c_driver_reinit(port, conf);
    I2C_BUS_CHECK(ret == ESP_OK, "init error", NULL);
    s_i2c_bus[port].conf_active = *conf;
    s_i2c_bus[port].i2c_port = port;
    return (i2c_bus_handle_t)&s_i2c_bus[port];
}

esp_err_t i2c_bus_delete(i2c_bus_handle_t *p_bus)
{
    I2C_BUS_CHECK(p_bus != NULL && *p_bus != NULL, "pointer = NULL error", ESP_ERR_INVALID_ARG);
    i2c_bus_t *i2c_bus = (i2c_bus_t *)(*p_bus);
    I2C_BUS_INIT_CHECK(i2c_bus->is_init, ESP_FAIL);
    I2C_BUS_MUTEX_TAKE_MAX_DELAY(i2c_bus->mutex, ESP_ERR_TIMEOUT);

    /** if ref_counter == 0, de-init the bus**/
    if ((i2c_bus->ref_counter) > 0) {
        ESP_LOGW(TAG, "i2c%d is also handled by others ref_counter=%u, won't be de-inited", i2c_bus->i2c_port, i2c_bus->ref_counter);
        return ESP_OK;
    }

    esp_err_t ret = i2c_driver_deinit(i2c_bus->i2c_port);
    I2C_BUS_CHECK(ret == ESP_OK, "deinit error", ret);
    vSemaphoreDelete(i2c_bus->mutex);
    *p_bus = NULL;
    return ESP_OK;
}

uint32_t i2c_bus_get_current_clk_speed(i2c_bus_handle_t bus_handle)
{
    I2C_BUS_CHECK(bus_handle != NULL, "Null Bus Handle", 0);
    i2c_bus_t *i2c_bus = (i2c_bus_t *)bus_handle;
    I2C_BUS_INIT_CHECK(i2c_bus->is_init, 0);
    return i2c_bus->conf_active.master.clk_speed;
}

i2c_bus_device_handle_t i2c_bus_device_create(i2c_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed)
{
    I2C_BUS_CHECK(bus_handle != NULL, "Null Bus Handle", NULL);
    I2C_BUS_CHECK(clk_speed <= 400000, "clk_speed must <= 400000", NULL);
    i2c_bus_t *i2c_bus = (i2c_bus_t *)bus_handle;
    I2C_BUS_INIT_CHECK(i2c_bus->is_init, NULL);
    i2c_bus_device_t *i2c_device = calloc(1, sizeof(i2c_bus_device_t));
    I2C_BUS_CHECK(i2c_device != NULL, "calloc memory failed", NULL);
    I2C_BUS_MUTEX_TAKE_MAX_DELAY(i2c_bus->mutex, NULL);
    i2c_device->dev_addr = dev_addr;
    i2c_device->conf = i2c_bus->conf_active;

    /*if clk_speed == 0, current active clock speed will be used, else set a specified value*/
    if (clk_speed != 0) {
        i2c_device->conf.master.clk_speed = clk_speed;
    }

    i2c_device->i2c_bus = i2c_bus;
    i2c_bus->ref_counter++;
    I2C_BUS_MUTEX_GIVE(i2c_bus->mutex, NULL);
    return (i2c_bus_device_handle_t)i2c_device;
}

esp_err_t i2c_bus_device_delete(i2c_bus_device_handle_t *p_dev_handle)
{
    I2C_BUS_CHECK(p_dev_handle != NULL && *p_dev_handle != NULL, "Null Device Handle", ESP_ERR_INVALID_ARG);
    i2c_bus_device_t *i2c_device = (i2c_bus_device_t *)(*p_dev_handle);
    I2C_BUS_MUTEX_TAKE_MAX_DELAY(i2c_device->i2c_bus->mutex, ESP_ERR_TIMEOUT);
    i2c_device->i2c_bus->ref_counter--;
    I2C_BUS_MUTEX_GIVE(i2c_device->i2c_bus->mutex, ESP_FAIL);
    free(i2c_device);
    *p_dev_handle = NULL;
    return ESP_OK;
}

/**************************************** Private Functions*********************************************/
static esp_err_t i2c_driver_reinit(i2c_port_t port, const i2c_config_t *conf)
{
    I2C_BUS_CHECK(port < I2C_NUM_MAX, "i2c port error", ESP_ERR_INVALID_ARG);
    I2C_BUS_CHECK(conf != NULL, "pointer = NULL error", ESP_ERR_INVALID_ARG);

    if (s_i2c_bus[port].is_init) {
        i2c_driver_delete(port);
        s_i2c_bus[port].is_init = false;
        ESP_LOGI(TAG, "i2c%d bus deinited", port);
    }

    esp_err_t ret = i2c_param_config(port, conf);
    I2C_BUS_CHECK(ret == ESP_OK, "i2c param config failed", ret);
    ret = i2c_driver_install(port, conf->mode, I2C_BUS_MASTER_BUF_LEN, I2C_BUS_MASTER_BUF_LEN, I2C_BUS_FLG_DEFAULT);
    I2C_BUS_CHECK(ret == ESP_OK, "i2c driver install failed", ret);
    s_i2c_bus[port].is_init = true;
    ESP_LOGI(TAG, "i2c%d bus inited", port);
    return ESP_OK;
}

static esp_err_t i2c_driver_deinit(i2c_port_t port)
{
    I2C_BUS_CHECK(port < I2C_NUM_MAX, "i2c port error", ESP_ERR_INVALID_ARG);
    I2C_BUS_CHECK(s_i2c_bus[port].is_init == true, "i2c not inited", ESP_ERR_INVALID_STATE);
    i2c_driver_delete(port); //always return ESP_OK
    s_i2c_bus[port].is_init = false;
    ESP_LOGI(TAG,"i2c%d bus deinited",port);
    return ESP_OK;
}

/**
 * @brief compare with active i2c_bus configuration
 *
 * @param port choose which i2c_port's configuration will be compared
 * @param conf new configuration
 * @return true new configuration is equal to active configuration
 * @return false new configuration is not equal to active configuration
 */
inline static bool i2c_config_compare(i2c_port_t port, const i2c_config_t *conf)
{
    if (s_i2c_bus[port].conf_active.master.clk_speed == conf->master.clk_speed
            && s_i2c_bus[port].conf_active.sda_io_num == conf->sda_io_num
            && s_i2c_bus[port].conf_active.scl_io_num == conf->scl_io_num
            && s_i2c_bus[port].conf_active.scl_pullup_en == conf->scl_pullup_en
            && s_i2c_bus[port].conf_active.sda_pullup_en == conf->sda_pullup_en) {
        return true;
    }

    return false;
}
