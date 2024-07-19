.. _nordic_pca63566:

Nordic PCA63566
###############

Overview
********

The PCA63566 is a shield that holds a series of sensors connected to I3C and SPI
buses:

- LPS22HH pressure sensor
- BME688 environmental sensor
- BMI270 IMU
- ADXL362 accelerometer

The shield also has an isolated CAN bus transceiver ready to be used with CAN
P9.4/5 pins (the default configuration). However, the transceiver does not
require additional setup, so the shield is not required for this use case.

Requirements
************

The Nordic PCA63566 shield is designed to fit into the
:ref:`nrf54h20dk_nrf54h20`.

Usage
*****

The shield can be used in any application by setting
``--shield nordic_pca63566`` when invoking ``west build``.

References
**********

TODO: Link to schematic
