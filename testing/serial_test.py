import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)
val = (2500.0 - 500.0)/2.0 + 500.0
cmd = f"#1P{int(val)}S400\r" #1 P1722 S400 #2 P2500 S400 #3 P0833 S400\r"

print(val)

ser.write(bytes(cmd, 'ascii'))