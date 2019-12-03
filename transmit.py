# import libraries
import serial
import time
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
while True:
    try:
        2+2

    except KeyboardInterrupt:
        break


#cleanup
xbee.halt()
ser.close()
