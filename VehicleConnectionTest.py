# Brandon Stevens
# 12/8/2019

#
# Connects to PX4 firmware device and outputs some basic data
#

# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
import time, sys, argparse, math, mpu

GPIO.setwarnings(False)    # Ignore warning for now
GPIO.setmode(GPIO.BOARD)   # Use physical pin numbering
GPIO.setup(8, GPIO.OUT, initial=GPIO.LOW)   # Set pin 8 to be an output pin and set initial value to low (off)

# global variables
safeDistance = 730 #feet
secondDroneLat = 33.214837 #degrees
secondDroneLong = -87.542813 #degrees

# connect to the Vehicle
print ("Connecting"...)
connection_string = '/dev/ttyUSB0'
vehicle = connect(connection_string, wait_ready=True) #vehicle is a px4 object

# display basic vehicle state
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

elif mode == 2:
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

elif mode == 3:
    
    count = 0
    while count < 1000:
        # Drone location
        lat1 = vehicle.location.global_relative_frame.lat
        lon1 = vehicle.location.global_relative_frame.lon

        # second drone location
        lat2 = secondDroneLat
        lon2 = secondDroneLong

        # Distance calculation
        dist = mpu.haversine_distance((lat1, lon1), (lat2, lon2)) #km
        dist = dist * 3280.24 #km -> feet
        print("%s feet", dist)
        if dist < safeDistance:
            GPIO.output(8, GPIO.HIGH) # Turn on
            print("Unsafe distance!")
        else:
            GPIO.output(8, GPIO.LOW) # Turn off
        time.sleep(0.5)
        count = count + 1

else:
    print("Invalid input")
