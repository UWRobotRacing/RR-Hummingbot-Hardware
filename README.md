# RobotRacing2019-Hardware
This repo contains hardware includes mech &amp; electrical. Firmware code will also locate here.

## Code Style
1. Use `space: 2` for tabs
2. Preferrably to put all private global variables within a local struct called `FileName_data_S`, and declare a static global variable `FileName_data_S fileName_data;` & initialized with a `memset(&fileName_data, 0, sizeof(fileName_data));` 
    - Reason: to keep global vars more organized and trackable
3. use **cameralCase** preferrably for the variable names and function names
4. For libraries, we shall append a unique space name in CAP before each public functions such as `uint8_t RF24_read(void)`
5. All private data & functions should be declared with a `static`, so it clearly indicate the function/data is private.

## Key Notes
1. For Mac users to use uart for debugging, you will be required to install [FT232R driver](https://www.ftdichip.com/Drivers/VCP.htm), please see instructions here: - [MacOS High Sierra 10.13.3 FTDI Driver Fix](https://superuser.com/questions/1135730/how-to-release-reset-serial-port-ftdi-devices-mac-osx)
2. NXP mcuxpresso API for our chip: [link](https://mcuxpresso.nxp.com/api_doc/dev/1008/group__lpspi__driver.html#ae03069cfdcf680ee5fd81e077b81bc18), and you might be required to login, please register an account if necessary.
    - you can also download the API, which contains chip specific examples \[SDK_2.5.0_MKE14F512xxx16\]
3. Our M4 chip: MKE14F512xxx16
  
# Hardware Resources
## Reedy Sonic 540 Sensor 6Pins

IMHO the connector is "EFRA standard" (RC cars)
see here page 25
[http://mgm-compro.com/manuals/en-man...-071011-a1.pdf](http://mgm-compro.com/manuals/en-manual-x-series-071011-a1.pdf)
 - Pin #1 – black wire, ground potential (minus)
 - Pin #2 – orange wire, sensor phase C
 - Pin #3 – white wire, sensor phase B
 - Pin #4 – green wire, sensor phase A
 - Pin #5 – blue wire, motor temperature sensing, 10 k NTC
 - (other end of sensor is on ground potential, pin #1)
 - Pin #6 – red wire, sensors feeding, +5.0 V ± 10%.
(supply voltage for sensors provide controller, don´t connect external voltage !)

### interfacing hall sensors:
https://tutorial.cytron.io/2018/07/04/measuring-dc-motor-rpm-through-built-in-hall-sensor-encoder/
https://www.nxp.com/docs/en/application-note/AN2892.pdf

## Steering_Encoder:
https://www.digikey.ca/product-detail/en/cui-inc/AMT203-V/102-2050-ND/2278846

## *ESC Arduino* 
https://www.instructables.com/id/ESC-Programming-on-Arduino-Hobbyking-ESC/ 

## nRF24l01


### **nRF24L01** ###
1. [Intro](https://components101.com/wireless/nrf24l01-pinout-features-datasheet)
2. [DataSheet](https://components101.com/sites/default/files/component_datasheet/nRF24L01%20Datasheet.pdf)
3. [Enhancing](https://www.instructables.com/id/Enhanced-NRF24L01/)
4. [Detailed Explaination](http://www.diyembedded.com/tutorials/nrf24l01_0/nrf24l01_tutorial_0.pdf)
5. [Colorful InDepth Descriptions](



