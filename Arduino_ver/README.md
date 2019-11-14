## MAPPING

*---------------- NRF24L01 ----*

**MEGA 2560:**

   *MISO -> 50*

   *MOSI -> 51*

   *SCK -> 52*

   *CE -> 7*

   *CSN -> 8*

   *GND -> GND*

   *VCC -> 3.3v*

*---------------- VC ----*

**MEGA 2560:**

   *ESC -> 5*

   *SERVO -> 4*



## UART

- use **9600** baud rate

- Please add a capacitor across **reset & 5v**

  - > Purpose: is to **prevent** arduino to reset on usb plugin/off, open/closing uart comm.
    >
    > **NOTE:** please **remove** this cap before **flashing firmware**, or you will not be able to do it

- checkout `UnitTest` Folder:

  - `uart_test` is a python hoster script to test the struct and validate protocol

  - `Serial2Serial` is used to obtain the usb serial input from Jetson, and forward to **Another Serial**,

    and `Serial2SerialRead` will accept this uart message, and parsing it to jetson-m4 protocol, and print on USB serial, so you can view live data with arduino IDE monitor.

  - You can enable `DEBUG_UART_SERIAL_COMM_FROM_ANOTHER_MEGA` to forward message to **serial1**, and monitor the income message with another mega with `Serial2SerialRead` .

- For Serial Adapter/Forwarding testing tool, make sure, one **RX/TX** is connected to the other **TX/RX** respectively



## nRF24L01

1. [Intro](https://components101.com/wireless/nrf24l01-pinout-features-datasheet)
2. [DataSheet](https://components101.com/sites/default/files/component_datasheet/nRF24L01 Datasheet.pdf)
3. [Enhancing](https://www.instructables.com/id/Enhanced-NRF24L01/)
4. [Detailed Explaination](http://www.diyembedded.com/tutorials/nrf24l01_0/nrf24l01_tutorial_0.pdf)
5. [Colorful InDepth Descriptions](https://lastminuteengineers.com/nrf24l01-arduino-wireless-communication/)