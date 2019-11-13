import serial
import numpy as np

ser = serial.Serial('/dev/tty.usbmodem1441301', 115200, timeout=5)
# x = ser.read()          # read one byte
# s = ser.read(10)        # read up to ten bytes (timeout)
# line = ser.readline()   # read a '\n' terminated line
# print(line)

# class Payload(Structure):
#     _fields_ = [("jetson_ang", c_int16_t),
#                 ("jetson_spd", c_int16_t),
#                 ("jetson_flag", c_uint16_t)]
jetson_ang = np.int16(20)
jetson_spd = np.int16(-120)
jetson_flag = np.uint16(23)

for x in range(0, 10000):
    ser.write(b'abcdefgn')
    # ser.write(b'\023\0\026\0\0\0\0')
    line = ser.readline()   # read a '\n' terminated line
    print(line)
ser.close()

