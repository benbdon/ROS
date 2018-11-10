import serial
import time

ser = serial.Serial('/dev/ttyACM0')  # open serial port
ser.baudrate = 9600
print(ser.name)         # check which port was really used


def convert_position(dec):
    a=bin(dec*4)
    b='0'+a[2:]
    # print(b)
    # print('01011101110000')
    # print(b[7:])
    c='0'+b[7:]
    d='0'+b[0:7]
    e=int (c,2)
    f=int(d,2)
    print(e, f)
    return chr(e)+chr(f)


tilt = 1000
pan = 1000

while(1):
    while tilt < 1950:
        time.sleep(.3)

        ser.write(chr(0x84) + chr(0x00) + convert_position(pan))
        ser.write(chr(0x84) + chr(0x01) + convert_position(tilt))
        pan = pan + 20
        tilt = tilt + 20
        print "loop 1",pan,tilt
    while tilt > 1050:
        time.sleep(.3)
        ser.write(chr(0x84) + chr(0x00) + convert_position(pan))
        ser.write(chr(0x84) + chr(0x01) + convert_position(tilt))
        pan = pan - 20
        tilt = tilt - 20
        print "loop 2",pan,tilt
