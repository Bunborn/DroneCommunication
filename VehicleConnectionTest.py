# Brandon Stevens
# 12/8/2019

#
# Connects to PX4 firmware device and outputs some basic data
#

# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math, mpu

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
print (" Mode: %s" % vehicle.mode.name)
print (" Connected!\nChoose operating mode:")
mode = input("( 1 ) for general heading, ( 2 ) for distance to fountain, ( 3 ) for collision warning ")

if mode == 1:
    count = 0
    while count < 100:
        print(" Heading: %s" % vehicle.heading)
        print(" %s" % vehicle.location.global_frame)
        print(" Velocity: %s" % vehicle.velocity)
        print(" GPS: %s\n" % vehicle.gps_0)
        time.sleep(0.1)
        count = count + 1

if mode == 2:
    # Drone location
    lat1 = vehicle.location.global_relative_frame.lat
    lon1 = vehicle.location.global_relative_frame.lon

    # Engineering Shelby quad fountain center
    lat2 = 33.214837
    lon2 = -87.542813

    # Distance calculation
    dist = mpu.haversine_distance((lat1, lon1), (lat2, lon2)) #km
    dist = dist * 3280.24 #km -> feet
    print("Distance to Shelby Engineering Quad Fountain : ", dist)
