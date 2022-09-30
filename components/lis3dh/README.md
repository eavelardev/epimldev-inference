# LIS3DH IMU component

## Introduction

Library based on [AIWintermuteAI/LIS3DH_ESP-IDF](https://github.com/AIWintermuteAI/LIS3DH_ESP-IDF) repo.

Differences:
* Implements an I2C bus handler for `thread-safe` communication
* I2C bus config is set in main app.

## How to install 

Get `lis3dh` component in your project `components` folder.
```
git clone https://github.com/eavelardev/esp-comp-lis3dh.git lis3dh
```

Get `i2c-bus` component dependency.
```
git clone https://github.com/eavelardev/esp-comp-i2c-bus.git i2c-bus
```

## Usage

Set the I2C config, you can look in these repos
* [eavelardev/esp-comp-i2c-bus](https://github.com/eavelardev/esp-comp-i2c-bus)
* [eavelardev/esp-app-lis3dh](https://github.com/eavelardev/esp-app-lis3dh)

```c++
#include <stdio.h>
#include "i2c_conf.h"
#include "lis3dh.h"

#define IMU_AXIS_SAMPLED    3
#define CONVERT_G_TO_MS2    9.80665f

LIS3DH lis3dh;
static float imu_data[IMU_AXIS_SAMPLED];

extern "C" void app_main()
{
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);

    lis3dh.begin(i2c_bus, LIS3DHTR_DEFAULT_ADDRESS);

    if(lis3dh.isConnection() == false) {
        printf("ERR: failed to connect to LIS3DH sensor!\n");
        return;
    }

    vTaskDelay(100 / portTICK_RATE_MS);
    lis3dh.setFullScaleRange(LIS3DH_RANGE_2G);
    lis3dh.setOutputDataRate(LIS3DH_DATARATE_100HZ);

    while (true)
    {
        lis3dh.getAcceleration(&imu_data[0], &imu_data[1], &imu_data[2]);
        imu_data[0] *= CONVERT_G_TO_MS2;
        imu_data[1] *= CONVERT_G_TO_MS2;
        imu_data[2] *= CONVERT_G_TO_MS2;

        printf("accX: %f\t accY: %f\t accZ: %f\n", imu_data[0], imu_data[1], imu_data[2]);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
```

----
## License
This software is written by Seeed studio<br>
and is licensed under [The MIT License](http://opensource.org/licenses/mit-license.php). Check License.txt for more information.<br>

Modified by Eduardo Avelar for EdgeImpulse Inc.

Contributing to this software is warmly welcomed. You can do this basically by<br>
[forking](https://help.github.com/articles/fork-a-repo), committing modifications and then [pulling requests](https://help.github.com/articles/using-pull-requests) (follow the links above<br>
for operating guide). Adding change log and your contact into file header is encouraged.<br>
Thanks for your contribution.

Seeed Studio is an open hardware facilitation company based in Shenzhen, China. <br>
Benefiting from local manufacture power and convenient global logistic system, <br>
we integrate resources to serve new era of innovation. Seeed also works with <br>
global distributors and partners to push open hardware movement.<br>
