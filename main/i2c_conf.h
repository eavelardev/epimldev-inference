#ifndef _I2C_CONF_H_
#define _I2C_CONF_H_

#include "i2c_bus.h"

#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_NUM          I2C_NUM_0

static i2c_bus_handle_t i2c_bus = NULL;

i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master = {
        .clk_speed = I2C_MASTER_FREQ_HZ
    },
    .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
}; 

#endif
