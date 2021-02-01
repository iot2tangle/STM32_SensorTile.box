# STM32 SensortTile.Box -- Bluetooth Low Energy - BLE
STM32 is a family of 32-bit microcontroller integrated circuits by STMicroelectronics. Based on ARM® Cortex®-M, STM32 is suitable for low power, real time applications. 

The STEVAL-MKSBOX1V1 (SensorTile.box) is a ready-to-use box kit with wireless IoT and wearable sensor platform to help you use and develop apps based on remote motion and environmental sensor data, regardless of your level of expertise.
In this repository you will find explanations and step by step for the development using the Sensortile.box development board, this boards has many builtin sensors such as accelerometers, magnetometer and gyroscope. In addition it has environmental sensors for measuring pressure, temperature and relative humidity.

This Repository contains the source code and the steps to follow to be able to make STM32 read sensor data and send it, in an organized way, to the Tangle (DLT) of the IOTA Network through the Streams layer.

## Sensors
The following sensors will communicate with STM32 using SPI protocol or I2C, they will be initializated and the data will be collected each 500ms.

* Digital temperature sensor (STTS751)
* 6-axis inertial measurement unit (LSM6DSOX)
* 3-axis accelerometers (LIS2DW12 and LIS3DHH)
* 3-axis magnetometer (LIS2MDL)
* Altimeter / pressure sensor (LPS22HH)
* Microphone / audio sensor (MP23ABS1)
* Humidity and temperature sensor (HTS221)

## Download the firmware on the board

The source code can be built and flashed using the official IDE of St-microelectronics, supported IDEs are:

* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) available for Linux, Mac and Windows
* [SW4STM32](https://www.st.com/en/development-tools/sw4stm32.html) available for Linux, Mac and Windows

Once the installation of the IDE is complete click on *.cproject* file if you installed SW4STM32, the IDE will launch importing the project.

If you installed STM32CubeIDE click on the *.project* file and the IDE will import the project.

## Programmer


One of the following hardware programmers will be needed to launch the firmware on the board:

* [ST-link-v2](https://www.st.com/en/development-tools/st-link-v2.html) Good for programming and debugging the code
* [ST-link-v3](https://www.st.com/en/development-tools/stlink-v3set.html) The last version of the programmer

## Flash the code 

Connect the board to the programmer and connect the programmer to your pc, click on *run* on the IDE and the project will be compiled and flashed on the board

## Name of the device

The name of the device can be edited in the file *enable.c* the address instead will be fixed.

## Read BLE Services and Characteristics
You may also want to read the data directly from the *BLE Server*. For this there are Free OpenSource software.

We recommend ***nRF Connect*** (free OpenSource software) of *NordicSemiconductor* available in [Desktop](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Connect-for-desktop): on Windows, macOS and Linux. And in [Mobile](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Connect-for-mobile): on Android and iOS (The mobile version is very simple and more comfortable to debug).
It only needs to be connected to the *STM32ToTangle* and it will be able to read the data from the sensors found in the Characteristics Values

Note that when you have a device connected to STM32ToTangle the Gateway will not be able to read the data.

# Setting up the Streams BLE Gateway

## Preparation

Install Rust if you don't have it already. More info about Rust here https://www.rust-lang.org/tools/install

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Make sure you also have the build dependencies installed, if not run:

```
sudo apt update
sudo apt install build-essential pkg-config libssl-dev libdbus-glib-1-dev
```

## Installing the Streams BLE Gateway
### Clone the repository
Get the Streams BLE Gateway repository
https://github.com/iot2tangle/Streams-BLE-gateway
```
git clone https://github.com/iot2tangle/Streams-BLE-gateway
cd Streams-BLE-gateway
```
## Scan the BLE Server (STM32 BLE Server in our case)
Here you will get the address of your *STM32BLE device* (similar to the MAC address). You should make a note of it, as we'll put that address in the *config.json* file later in the next step.

Run the following to scan and get the address of your STM32
```
cargo run --release --bin scan
```

## Edit *config.json* file
Navigate to the root of **Streams-BLE-gateway** directory and edit the **config.json** and copy the *address* obtained in the previous step.

Here you can also configure the time interval with which the GW will read the data to the BLE Devices and send the data to the Tangle, also the node, amoung other settings.
```
{
    "device_ids": [
        "XX:XX:XX:XX:XX:XX"
    ],
    "reading_interval": 30,
    "node": "https://nodes.iota.cafe:443",
    "mwm": 14,
    "local_pow": false
}
```
## Start the Streams BLE Server

### Sending messages to the Tangle

Run the Streams BLE Gateway:

```
cargo run --release --bin ble-gateway
```

This will compile and start the *Streams BLE Gateway*. Note that the compilation process may take from 3 to 25 minutes (Pi3 took us around 15/25 mins, Pi4 8 mins and VPS or desktop machines will generally compile under the 5 mins) depending on the device you are using as host.
You will only go through the compilation process once and any restart done later will take a few seconds to have the Gateway working.

Once started, the ***Channel Id*** will be displayed, and the gateway will be open waiting for data to send to the Tangle.


### Reading messages from the Tangle

You can read the received messages directly from the **I2T Explorer**: https://explorer.iot2tangle.io/ using the Channel Id printed by the Gateway in shell.   

![I2T Explorer](https://i.postimg.cc/wTNf7dgp/Screenshot-from-2020-10-16-11-46-16.png)


*For inquiries, improvements or errors detected, please start an issue in this repository.*
