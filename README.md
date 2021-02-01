# STM32 SensortTile.Box
STM32 is a family of 32-bit microcontroller integrated circuits by STMicroelectronics. Based on ARM® Cortex®-M, STM32 is suitable for low power, real time applications. 

The [STEVAL-MKSBOX1V1](https://www.st.com/en/evaluation-tools/steval-mksbox1v1.html) (SensorTile.box) is a ready-to-use box kit with wireless IoT and wearable sensor platform to help you use and develop apps based on remote motion and environmental sensor data, regardless of your level of expertise.
In this repository you will find explanations and step by step for the development using the Sensortile.box development board, this boards has many builtin sensors such as accelerometers, magnetometer and gyroscope. In addition it has environmental sensors for measuring pressure, temperature and relative humidity.

## On board sensors

Sensortile.box has some ST **MEMS** (Micro Electro-Mechanical Systems) and sensors on board such as:

* Digital temperature sensor (STTS751)
* 6-axis inertial measurement unit (LSM6DSOX)
* 3-axis accelerometers (LIS2DW12 and LIS3DHH)
* 3-axis magnetometer (LIS2MDL)
* Altimeter / pressure sensor (LPS22HH)
* Microphone / audio sensor (MP23ABS1)
* Humidity sensor (HTS221)

Thanks to the lithium battery and the plastic box it is ready to collect the data in any enviornment.

# Available connectivity

* [BLE-sender](https://github.com/iot2tangle/STM32_SensorTile.box/tree/main/BLE-sender) -- SensorTile.Box will send the sensors data using *Bluetooth Low Energy (BLE)* to [I2T BLE Gateway](https://github.com/iot2tangle/Streams-ble-gateway)
