# nrf51-vl53l0x-driver

In your sdk_config.h file, find the following lines and set them as follows:
```c
#define TWI_ENABLED 1
#define TWI0_ENABLED 1
```

Most of this driver is copied straight from ST, with the exception of `platform/src/vl53l0x_i2c_platform.c`, which implements the i2c driver on the nrf51, and contains the pin definitions for the i2c bus. Change those pin definitions to match your board layout.

`main.c` is an example of how to use the driver, but is not tested and may not compile as written. Please submit a pull request with any necessary changes to make it work out of the box.
