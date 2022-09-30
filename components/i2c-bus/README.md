# i2c-bus component

## Introduction 

Libray based on `bus` component from [esp-iot-solution](https://github.com/espressif/esp-iot-solution) repo.

Differences:
* Implements `i2c_master_write_to_device` for write operations.
* Implements `i2c_master_write_read_device` for write-read operations.
* Can be added to the project components cloning the repo.
* Remove old implementations of write and read operations.

To do:
* Update the code to c++ to implement an object-oriented aproach

## How to install 

Get `i2c-bus` component in your project `components` folder.
```
git clone https://github.com/eavelardev/esp-comp-i2c-bus.git i2c-bus
```

## Usage

Configuration code can be set in a header file.

```c++
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
```

This hedaer can be added to your main app

```c++
#include "i2c_conf.h"
#include "<my_sensor>.h"

extern "C" void app_main()
{
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);

    <my_sensor_obj>.begin(i2c_bus, <my_sensor_addr>)
}
```

The sensor library has to support `i2c-bus` implementation

```c++
void <My_Sensor_Class>::begin(i2c_bus_handle_t i2c_bus, uint8_t dev_addr)
{
    i2c_dev = i2c_bus_device_create(i2c_bus, dev_addr, i2c_bus_get_current_clk_speed(i2c_bus));
}

void <My_Sensor_Class>::writeRegister(uint8_t reg, uint8_t val)
{   
    uint8_t write_buf[2] = {reg, val};
    i2c_write(i2c_dev, write_buf, sizeof(write_buf));
}

void <My_Sensor_Class>::readRegister(uint8_t *outputPointer, uint8_t reg, uint8_t length)
{
    reg |= 0x80; //turn auto-increment bit on, bit 7 for I2C
    i2c_read(i2c_dev, &reg, outputPointer, length);
}
```

You can implement methods to delete the device and i2c_bus

```c++
i2c_bus_device_delete(&i2c_dev);
i2c_bus_delete(&i2c_bus);
```

An example of a library for the `LIS3DH` IMU sensor implementing `i2c-bus` adapted from [AIWintermuteAI/LIS3DHTR_ESP-IDF](https://github.com/AIWintermuteAI/LIS3DHTR_ESP-IDF).
* [eavelardev/esp-comp-lis3dh](https://github.com/eavelardev/esp-comp-lis3dh)
* [eavelardev/esp-app-lis3dh](https://github.com/eavelardev/esp-app-lis3dh)

Now the communication with your sensor is `thread-safe`.
