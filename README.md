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
  
