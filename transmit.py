# Brandon Stevens
# 12/3/2019
# Inital code pulled from https://jekhokie.github.io/raspberry-pi/raspbian/xbee/python/linux/electronics/2018/12/30/raspberry-pi-xbee-rxtx.html

# tranmission device id = 0x01
# receive device id = 0x00

# import libraries
import serial
import time
import sys
from xbee import XBee

# assign the XBee device settings
SERIAL_PORT = "/dev/ttyS3"
BAUD_RATE = 9600

# configure the xbee
ser = serial.Serial(SERIAL_PORT, baudrate = BAUD_RATE)
xbee = XBee(ser, escaped = False)

# handler for sending data to a receiving XBee device
def send_data(data):
    xbee.send("tx", dest_addr=b'\x00\x00', data=bytes("{}".format(data), 'utf-8'))

# main loop/functionality
#while True:
#    try:


#    except KeyboardInterrupt:
#        break

print("Transmitting...")
send_data("hello world")

#cleanup
ser.flushInput()
ser.flushOutput()
ser.close()
xbee.halt()
