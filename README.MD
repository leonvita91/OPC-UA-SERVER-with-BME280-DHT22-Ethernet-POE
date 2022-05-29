## Table of contents
* [Introduction ](#introduction)
* [Hardware](#Hardware)
* [Photos From My Project](#Photos)
* [Technologies](#technologies)
* [Setup](#setup)
* [Extra setups](#extra_setups)
* [Problems We Had Solved](#problems_we_had_solved)
* [Reference](#reference)


## Introduction 
_An OPC UA running server on ESP32-ISO development board with Ethernet  and BME280 Sensor,DHT22 Sensor. Also with a temperature,pressure,humidity nodes reading over GPIO 4 and I2C from a DHT-22 sensor and BME280. The Project Build on ESP-IDF Environment to make it compatible with FreeRTOS and xtensa-toolchain._


## Hardware Setup
| BME280 | GPIO ESP32 |  
| -----| ------ |
| VCC  | 3V |
| GND  | GND |
| SDA  |GPIO13 |
| SCL  |GPIO16 |
| SDO  | 3V  |

**NOTE**:
BME280 has 2 mode SPI and I2C and when connecting SDO to 3V this will mapping to I2C


DHT22 
---------------------------
| DHT22 | GPIO ESP32 |
| ---  |----|
| VCC  | 3V |
| GND  | GND |
| DATA OUT | GPIO 4 |

## Photos 
* Photo of ESP32-ISO https://www.olimex.com/Products/IoT/ESP32/ESP32-POE-ISO/open-source-hardware
* Photo of GPIO Map  https://www.olimex.com/Products/IoT/ESP32/ESP32-POE-ISO/resources/ESP32-POE-ISO-GPIO.png
* Photo of BME280    https://esphome.io/components/sensor/bme280.html 
* Photo of DHT22     https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf
* Photo of 3D Case   https://www.thingiverse.com/thing:4860626

	
## Technologies
Project is created in 2021:
* TCP-IP 
* OPC-UA Protocol 
* Power over Ethernet DC
	
## Setup
**These Points you need to follow :**
```
* Building ESP-IDF Environment
* Clone OPC-UA Project & Building
* Flashing and Monitoring 
* Connect With OPC-UA Client 
```

* **Building IDF Environment:**
-------------------------------------
* Ubuntu and Debian:

*  `sudo apt-get install git wget flex bison gperf python3 python3-pip python3-setuptools `
*  `sudo apt-get install cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 `
* `mkdir -p ~/esp`
* `cd ~/esp`
* `git clone --recursive https://github.com/espressif/esp-idf.git`
* `cd ~/esp/esp-idf`
* `./install.sh`
* `. ./export.sh`

**ENDING Build ESP32-IDF Environment**

----------------------------
**Clone OPC-UA Project & Building**

* `cd ~/esp/esp-idf/examples/`
* `git clone https://github.com/leonvita91/OPC-UA-SERVER-with-BME280-DHT22-Ethernet-POE.git`
*  `cd opc-ua-with-bme280/`
*  `idf.py build`
*  `idf.py flash && idf.py monitor`

**NOTE:**
Connect your ESP32 with USB and Ethernet Cable to get (IP,MAC) 

 ## {- CAUTION !! -}

{-Dont Connect the ESP32 Board With USB and POE Ethernet in the same Time this will cause reverse electricity to your PC-} 
  
 {+ Please Connect Usb with cable without POE when you doing Monitoring OR Connecting POE Ethernet without Usb   +}
 


**ENDING Build OPC-UA With Flashing ESP32 Board**

## Extra_setups
The Extra Setup include :

* add new nodes.
* editing OPC-UA buffer.
* editing idf.py menuconfig settings.

 **Please Check the link below for extra_setup.md**

* https://gitlab.hzdr.de/alqase03/opc-ua-with-bme280/-/blob/master/Extra_Setup.md

## Problems We Had Solved 

* Ethernet Driver https://github.com/cmbahadir/opcua-esp32/issues/37

* emac no mem for receive buffer  https://github.com/cmbahadir/opcua-esp32/issues/42

## Reference

* The Orginial Code https://github.com/cmbahadir/opcua-esp32
* ESP-IDF HomePage  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/
* BME280 sensor     https://github.com/yanbe/bme280-esp-idf-i2c
* Ethernet Driver   https://github.com/OLIMEX/ESP32-POE
* Athene Wiki       https://athene.fz-rossendorf.de/fwk/Datentechnik:OPC-UA/ESP32_POE 

-----------------------------------------------------------------------------------------------------------
## {+Inspiration and Thanks!+} 

* Paul Petring,
I appreciate your consideration/guidance/help/time.
With your guidance and support,Thank you for sharing your wisdom and knowledge with me when I need it.
<p.petring@hzdr.de>

* Paul schwanse,
I'm humbled and grateful for all your support,Thank you for your assistance/consideration/encouragement/guidance/support/thoughtfulness/time.
<p.schwanse@hzdr.de>

* Finally,
Many thanks for Helmholtz-Zentrum Dresden-Rossendorf giving me this opportunity.
--------------------------------------------------------------------------------------------------------------------------------------
