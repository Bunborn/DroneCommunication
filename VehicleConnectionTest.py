# Brandon Stevens
# 12/8/2019

#
# Connects to PX4 firmware device and outputs some basic data
#

# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

# Connect to the Vehicle
print ("Connecting")
connection_string = '/dev/ttyUSB0'
vehicle = connect(connection_string, wait_ready=True)

# Display basic vehicle state
print (" Type: %s" % vehicle._vehicle_type)
print (" Armed: %s" % vehicle.armed)
print (" System status: %s" % vehicle.system_status.state)
print (" GPS: %s" % vehicle.gps_0)
print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
print (" Heading: %s" % vehicle.heading)
print ("System status: %s" % vehicle.system_status.state)
print ("Mode: %s" % vehicle.mode.name)

while True:
    print(" Heading: %s" % vehicle.heading)
    time.sleep(0.1)