#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function
from dronekit import connect, LocationGlobalRelative, mavutil, VehicleMode, Command
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

leftUp = (39.8723803,32.7319685)
upper = (39.8724621,32.7321984)
rightUp = (39.8723843,32.7324199)

leftlowback = (39.8718356,32.7320508)
leftLow = (39.8717558,32.7319907)

lower = (39.8717037,32.7321987)
rightLow = (39.8717662,32.7324181)
middlePoint = (39.8723142,32.7321761)

def arm_and_takeOff(targetAltitude):
    print("Basic pre-arm checks")
    
    while not vehicle.is_armable:
        print(" Waiting for iha to initialise...")
        time.sleep(1)

    print("Arming motors")
  
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    time.sleep(3)

    print("Arming motors")
    vehicle.armed = True

    while not vehicle.armed:
        print("waiting for the arming")
        vehicle.armed = True
        time.sleep(1)



    print("waiting for 5 seconds")
    time.sleep(5)

    print("start taking off up to", targetAltitude, " meters")
    vehicle.simple_takeoff(targetAltitude)  

    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)  

        if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95: 
                                                                                
            print("reached target altitude!")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] 
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def add_mission():
    
    global command
    command = vehicle.commands
    command.clear
    time.sleep(1)

    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,leftUp[0],leftUp[1],10))
    command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,upper[0],upper[1],10))
    command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,rightUp[0],rightUp[1],10))
    command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,leftlowback[0],leftlowback[1],10))
    command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,leftLow[0],leftLow[1],10))
    command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,lower[0],lower[1],10))
    command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,rightLow[0],rightLow[1],10))
    command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,middlePoint[0],middlePoint[1],10))
    command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,0,0,0,0,0,0))
    command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,0,0,0,0,0,0))
    command.upload()
    print("commands uploaded...")



height = 10
arm_and_takeOff(height)
add_mission()

command.next = 0
vehicle.mode = VehicleMode("AUTO")

while True:
    next_waypoint = command.next
    time.sleep(1)

    if next_waypoint == 10:
        break


print(" mission over...")
















