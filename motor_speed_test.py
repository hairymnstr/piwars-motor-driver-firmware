import struct
import serial
import time
import sys

CMD_SET_MOTOR_SPEED = 10
CMD_SET_SERVO_ANGLE = 11

s = serial.Serial(sys.argv[1], baudrate=115200)

s.parity = serial.PARITY_MARK
s.write(b'\x05')

time.sleep(0.1)
msg = struct.pack('<Bb', CMD_SET_MOTOR_SPEED, int(sys.argv[2]))
cks = 0
for b in msg:
    cks ^= b
msg += bytes([cks])
print(msg)
s.parity = serial.PARITY_SPACE
s.write(msg)

time.sleep(0.1)
msg = b''
while(s.in_waiting):
    msg += s.read()
    
print(msg)
