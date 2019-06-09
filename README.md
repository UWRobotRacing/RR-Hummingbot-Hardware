# RobotRacing2019-Hardware
This repo contains hardware includes mech &amp; electrical. Firmware code will also locate here.

## Code Style
1. Use `space: 2` for tabs
2. Preferrably to put all private global variables within a local struct called `FileName_data_S`, and declare a static global variable `FileName_data_S fileName_data;` & initialized with a `memset(&fileName_data, 0, sizeof(fileName_data));` 
    - Reason: to keep global vars more organized and trackable
3. use **cameralCase** preferrably for the variable names and function names
4. For libraries, we shall append a unique space name in CAP before each public functions such as `uint8_t RF24_read(void)`
5. All private data & functions should be declared with a `static`, so it clearly indicate the function/data is private.
