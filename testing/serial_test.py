import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)
cmd = "#0P500S400\r" #1 P1722 S400 #2 P2500 S400 #3 P0833 S400\r"


ser.write(bytes(cmd, 'ascii'))