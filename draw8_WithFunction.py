#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function
from re import X
from dronekit import connect, LocationGlobalRelative, mavutil, VehicleMode, Command, LocationGlobalRelative, LocationGlobal
import time, math
import argparse  


############# RASPBERRY 4B #######################
vehicle = connect("/dev/ttyACM0", wait_ready=True)
vehicle.mode = VehicleMode("GUIDED")
##################################################



################################# SITL ###################################################################################
# parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
# parser.add_argument('--connect',
#                     help="Vehicle connection target string. If not specified, SITL automatically started and used.")
# args = parser.parse_args()
# connection_string = args.connect
# sitl = None
# # Start SITL if no connection string specified
# if not connection_string:
#     import dronekit_sitl
#     sitl = dronekit_sitl.start_default()
#     connection_string = sitl.connection_string()
# # Connect to the Vehicle
# print('Connecting to vehicle on: %s' % connection_string)
# vehicle = connect(connection_string, wait_ready=True)
##########################################################################################################################



leftUp = LocationGlobalRelative(39.872385,32.7320469,10)
upper = LocationGlobalRelative(39.8724388,32.7321716,10)
rightUp = LocationGlobalRelative(39.8723977,32.7323096,10)
leftLow = LocationGlobalRelative(39.8722167,32.7320168,10)
lower = LocationGlobalRelative(39.8721416,32.7321706,10)
rightLow = LocationGlobalRelative(39.8722227,32.7323163,10)
middlePoint = LocationGlobalRelative(39.8723142,32.7321761,10)



def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    time.sleep(2)
    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def goPoint(point2,string,speed = 5.0,distance_kalan = 5.0):
    vehicle.simple_goto(point2)
    vehicle.airspeed = speed
    print(string,"/////////","Drone's speed: ",speed)
    while True:
        inst_loc = vehicle.location.global_relative_frame
        distance = get_distance_metres(inst_loc,point2)
        if distance <= distance_kalan:
            vehicle.airspeed = speed
            break
        else:
            time.sleep(0.5)



arm_and_takeoff(10)
time.sleep(2)
vehicle.airspeed = 10

goPoint(leftUp,"going to the left leftUp",10,3)
goPoint(upper,"going to the upper",10,3)
goPoint(rightUp,"going to the rightUp",10,3)
goPoint(leftLow,"going to the leftLow",10,3)
goPoint(lower,"going to the lower",10,3)
goPoint(rightLow,"going to the rightLow",10,3)
goPoint(middlePoint,"going to the middlePoint",10,3)

time.sleep(5)

print("mission over")
print("changing the vehicle.mode")

vehicle.mode = VehicleMode("LAND")




