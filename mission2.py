#!/usr/bin/env python
# -*- coding: utf-8 -*-


import time
from re import X
from dronekit import connect, LocationGlobalRelative, mavutil, VehicleMode, Command, LocationGlobalRelative, LocationGlobal
import time, math
import numpy as np
import cv2
from threading import Thread
import argparse
import RPi.GPIO as GPIO


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


frameWidth = 640
frameHeight = 480


cxArray = list()
cyArray = list()

check = False
counter = 0

waterCounter = 0
nowaterCounter = 0
waterCheck = 0



def arm_and_takeoff(aTargetAltitude):


    print ("Basic pre-arm checks")
   
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
   
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

 
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    time.sleep(2)
    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) 


    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)
def get_location_metres(original_location, dNorth, dEast):

    earth_radius=6378137.0 
    
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

  
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)
def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):


    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       
        0, 0,    
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        0b0000111111000111, 
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 
        0, 0)    

 
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.025)
def distance_to_current_waypoint():

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
def goPoint(point2,string,speed,distance_remain = 5.0):
    vehicle.simple_goto(point2,airspeed = speed)
    while True:
        inst_loc = vehicle.location.global_relative_frame
        distance = get_distance_metres(inst_loc,point2)
        if distance <= distance_remain:
            break
        else:
            time.sleep(0.5)

def calVector():
    if len(cxArray) != 0 and len(cyArray) != 0:
        x_vector = 0
        y_vector = 0
        sumx = 0
        sumy = 0
        factor = 300
        
        for x in cxArray:
            sumx += x
        for y in cyArray:
            sumy += y
        x_vector = (sumx / len(cxArray)) / factor
        y_vector = (sumy / len(cyArray)) / factor
        print("x_Vector",x_vector)
        print("y_vector",y_vector)
        send_ned_velocity(x_vector, y_vector, 0, 1)
        cxArray.clear()
        cyArray.clear()

def display(img):
    pt1 = (285,250)
    pt2 = (355,320)
    cv2.line(img,(int(frameWidth/2),0),(int(frameWidth/2),frameHeight),(252,0,0),2)
    cv2.line(img, (0, int(frameHeight / 2) ), (frameWidth, int(frameHeight / 2) ), (252,0,0), 2)
    cv2.rectangle(img,pt1,pt2,(50,150,30),2) 
def opencv():
    global targetLoc
    global check
    global counter

    webcam = cv2.VideoCapture(0)
    while not check:

        _, imageFrame = webcam.read()

        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        
        red_lower = np.array([136, 87, 111], np.uint8)
        red_upper = np.array([180, 255, 255], np.uint8)
        
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

        contours, hierarchy = cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            areaMin = 3000
            if (area > areaMin):
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(imageFrame, (x, y),
                                           (x + w, y + h),
                                           (0, 0, 255), 2)

                cx = int(x + (w / 2))
                cy = int(y + (h / 2))
                vx = cx - (frameWidth / 2)
                vy = (frameHeight / 2) - cy
                cv2.circle(imageFrame, (cx, cy), 4, (51, 255, 51), 1)

                if 35 >= vx >= -35 and -10 >= vy >= -85:
                    counter += 1

                    if counter < 29:
                        leftUpFont = cv2.putText(imageFrame, "saving location -"+str(counter)+ "/50", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 10, 10),2)
                        leftUpFont
                    if counter >= 29:
                        leftUpFont = cv2.putText(imageFrame, "location saved ", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (7, 7, 220),2)
                        leftUpFont
                        time.sleep(1)

                    if counter >= 30:
                        print("Boom''!")
                        targetLoc = vehicle.location.global_relative_frame
                        print("location saved")
                        print(targetLoc)
                        vehicle.airspeed = 7
                        goPoint(post1_right, "going to post1_right", 7,2) 
                        check = True

                else:
                    if vy >= 0 and vx <= 0 or vy <= 0 and vx >= 0:
                        cxArray.append(-vx)
                        cyArray.append(-vy)

                    else:
                        cxArray.append(vx)
                        cyArray.append(vy)

                calVector()

        display(imageFrame)
        cv2.imshow("Color Tracking", imageFrame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            webcam.release()
            cv2.destroyAllWindows()
            break



def waterTest():
    global waterCounter
    global nowaterCounter
    global waterCheck 
    
    channel = 21
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(channel, GPIO.IN)
    
    def callback(channel):
        if GPIO.input(channel):
            print ("no Water Detected!")
        else:
            print ("Water Detected!")
    
    GPIO.add_event_detect(channel, GPIO.BOTH, bouncetime=30)  
    
    
    
    while True:
        if GPIO.input(channel):
            print ("no Water Detected!")
            nowaterCounter+=1
        else:
            print ("Water Detected!")
            waterCounter+=1
        if waterCounter == 4:
            print("all Water Detected!")
            waterCheck = 1
            print("all Water Detected!, wait 8 sec")
            time.sleep(8)
            break
        if nowaterCounter == 8:
            print("no all Water Detected!-5m :(")
            nowaterCounter = 0
            waterCheck = 0
            break
        time.sleep(0.5)

    


    if waterCheck == 0:
        print("I get a little lower and start searching for water")
        goPoint(pool4,"Descending to 4m..",1,0.3)
        time.sleep(2)
        while True:
            
            if GPIO.input(channel):
                print ("no Water Detected!")
                nowaterCounter +=1
            else:
                print ("Water Detected!")
                waterCounter += 1
            if waterCounter == 4:
                print("all Water Detected!")
                waterCheck = 1
                print("all Water Detected!, wait 8 sec")
                time.sleep(8)
                break
            if nowaterCounter == 8:
                print("no all Water Detected! -4m :(")
                nowaterCounter = 0
                waterCheck = 0
                break
            time.sleep(0.5)

    if waterCheck == 0:
        print("I get a little lower and start searching for water")
        goPoint(pool3_5,"Descending to 3_5m..",1,0.3)
        time.sleep(2)
        while True:
            
            if GPIO.input(channel):
                print ("no Water Detected!")
                nowaterCounter +=1
            else:
                print ("Water Detected!")
                waterCounter += 1
            if waterCounter == 4:
                print("all Water Detected! :)")
                waterCheck = 1
                print("all Water Detected!, wait 8 sec")
                time.sleep(8)
                break
            if nowaterCounter == 8:
                print("no all Water Detected! -3_5m :(")
                nowaterCounter = 0
                waterCheck = 0
                break
            time.sleep(0.5)    


    if waterCheck == 0:
        print("I get a little lower and start searching for water")
        goPoint(pool3,"Descending to 3m..",1,0.3)
        time.sleep(2)
        while True:
            
            if GPIO.input(channel):
                print ("no Water Detected!")
                nowaterCounter +=1
            else:
                print ("Water Detected!")
                waterCounter += 1
            if waterCounter == 4:
                print("all Water Detected! :)")
                waterCheck = 1
                print("all Water Detected!, wait 8 sec")
                time.sleep(8)
                break
            if nowaterCounter == 8:
                print("no all Water Detected! -3m :(")
                nowaterCounter = 0
                waterCheck = 0
                break
            time.sleep(0.5)

    if waterCheck == 0:
        print("I get a little lower and start searching for water")
        goPoint(pool2_5,"Descending to 2_5m..",1,0.3)
        time.sleep(2)
        while True:
            
            if GPIO.input(channel):
                print ("no Water Detected!")
                nowaterCounter +=1
            else:
                print ("Water Detected!")
                waterCounter += 1
            if waterCounter == 4:
                print("all Water Detected! :)")
                waterCheck = 1
                print("all Water Detected!, wait 8 sec")
                time.sleep(8)
                break
            if nowaterCounter == 8:
                print("no all Water Detected! -2_5m :(")
                nowaterCounter = 0
                waterCheck = 0
                break
            time.sleep(0.5)

    if waterCheck == 0:
        print("I get a little lower and start searching for water")
        goPoint(pool2,"Descending to 2m..",1,0.3)
        time.sleep(2)
        while True:
            
            if GPIO.input(channel):
                print ("no Water Detected!")
                nowaterCounter +=1
            else:
                print ("Water Detected!")
                waterCounter += 1
            if waterCounter == 4:
                print("all Water Detected! :)")
                waterCheck = 1
                print("all Water Detected!, wait 8 sec")
                time.sleep(8)
                break
            if nowaterCounter == 8:
                print("no all Water Detected! -2m :(")
                nowaterCounter = 0
                waterCheck = 0
                break
            time.sleep(0.5)

    if waterCheck == 0:
        print("I get a little lower and start searching for water")
        goPoint(pool1_8,"Descending to 1_8m..",1,0.3)
        time.sleep(2)
        while True:
            
            if GPIO.input(channel):
                print ("no Water Detected!")
                nowaterCounter +=1
            else:
                print ("Water Detected!")
                waterCounter += 1
            if waterCounter == 4:
                print("all Water Detected! :)")
                waterCheck = 1
                print("all Water Detected!, wait 8 sec")
                time.sleep(8)
                break
            if nowaterCounter == 8:
                print("no all Water Detected! -1_8m :(")
                nowaterCounter = 0
                waterCheck = 0
                break
            time.sleep(0.5)
    
    if waterCheck == 0:
        print("I get a little lower and start searching for water")
        goPoint(pool1_6,"Descending to 1_6m..",1,0.3)
        time.sleep(2)
        while True:
            
            if GPIO.input(channel):
                print ("no Water Detected!")
                nowaterCounter +=1
            else:
                print ("Water Detected!")
                waterCounter += 1
            if waterCounter == 4:
                print("all Water Detected! :)")
                waterCheck = 1
                print("all Water Detected!, wait 8 sec")
                time.sleep(8)
                break
            if nowaterCounter == 8:
                print("no all Water Detected! -1_6m :(")
                print("waiting 8 sec ...")
                nowaterCounter = 0
                waterCheck = 0
                time.sleep(8)
                break
            time.sleep(0.5)

    GPIO.cleanup() 

def fireRelay():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)
    GPIO.output(17, GPIO.LOW)

    time.sleep(4)

    GPIO.output(17, GPIO.HIGH)

    time.sleep(2)

    GPIO.cleanup()

def myMissionList():
    
    global myCommand
    myCommand = vehicle.commands
    myCommand.clear
    time.sleep(1)

    myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    
    myCommand.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,0,0,7,0,0,0,0,0))
    myCommand.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,post2_upper.lat,post2_upper.lon,10))
    
    myCommand.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,redArea_start.lat,redArea_start.lon,10))
    myCommand.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,redArea_start.lat,redArea_start.lon,10))
    

    myCommand.upload()
    
    print("commands uploaded...")
    time.sleep(1)


def missionStart():
    global myCommand
    print("vehicle.commands.next : ",vehicle.commands.next)
    myMissionList()
    myCommand.next = 0
    vehicle.mode = VehicleMode("AUTO")


    while True:
        next_waypoint = myCommand.next
        print('Distance to waypoint (%s): %s' % (next_waypoint, distance_to_current_waypoint()))
      
        if next_waypoint  == 4:
            print("command's mission over.")
            vehicle.mode = VehicleMode("GUIDED")
            print("changing the mode.\t Mod: ",vehicle.mode)
            time.sleep(1)
            break
    



post2_upper = LocationGlobalRelative(38.7893562,30.4831164,10)
post1_right = LocationGlobalRelative(38.7903095,30.4830641,10)
post1_upper = LocationGlobalRelative(38.7904015,30.4827771,10)
post1_left = LocationGlobalRelative(38.7902144,30.4825678,10)
afterPool_post2_upper = LocationGlobalRelative(38.7893329,30.4831013,10)

finish_line1 = LocationGlobalRelative(38.7900530,30.4822907,10)
finish_line2 = LocationGlobalRelative(38.7899661,30.4823291,10)

redArea_start = LocationGlobalRelative(38.7898635,30.4832852,10)
redArea_finish = LocationGlobalRelative(38.7901039,30.4832340,10)



targetLoc = LocationGlobalRelative(0,0,0)

pool = LocationGlobalRelative(38.7896756743084,30.4827595409006,10)
pool10 = LocationGlobalRelative(pool.lat,pool.lon,10)
pool8 = LocationGlobalRelative(pool.lat,pool.lon,8)
pool7 = LocationGlobalRelative(pool.lat,pool.lon,7)
pool5 = LocationGlobalRelative(pool.lat,pool.lon,5)
pool4 = LocationGlobalRelative(pool.lat,pool.lon,4)
pool3 = LocationGlobalRelative(pool.lat,pool.lon,3)
pool3_5 = LocationGlobalRelative(pool.lat,pool.lon,3.5)
pool2_5 = LocationGlobalRelative(pool.lat,pool.lon,2.5)
pool2 = LocationGlobalRelative(pool.lat,pool.lon,2)
pool1_8 = LocationGlobalRelative(pool.lat,pool.lon,1.8)
pool1_6 = LocationGlobalRelative(pool.lat,pool.lon,1.6)
pool1 = LocationGlobalRelative(pool.lat,pool.lon,1)




def startAction():
    try:
        global myCommand
        global targetLoc
        
        vehicle.parameters["WP_YAW_BEHAVIOR"] = 0
        
        height = 10
        arm_and_takeoff(height)
        time.sleep(2)
        missionStart()
        time.sleep(1)
        print("changed the mode.\t Mod: ",vehicle.mode)
        goPoint(redArea_start,"Going to the beginning of the red area..",2,1)
        vehicle.airspeed = 1
        vehicle.simple_goto(redArea_finish, airspeed=1)
        time.sleep(1)
        th2.start()
        th2.join()
        time.sleep(1)
        goPoint(post1_right, "going to post1_right", 7,2)
        goPoint(post1_upper,"going to post1_upper..",6,2)
        goPoint(post1_left,"going to post1_left",6,2)
        vehicle.airspeed = 6
        goPoint(pool,"going to pool..",6,0.3)
        print("reached the pool ..")
        time.sleep(2)
        goPoint(pool5,"descending to 5m in the pool..",1,0.3)
        time.sleep(3)
        th3.start()
        th3.join()
        print("I'm starting to rise in the pool")
        goPoint(pool10,"I'm rising..",1,0.3)
        time.sleep(8)
        print("rising is complete. 8 sec waited''")
        print("going to redArea.")
        vehicle.airspeed = 2
        goPoint(afterPool_post2_upper,"going to afterPool_post2_upper",2,1)
        goPoint(redArea_start,"going to redArea_start..",2,1)
        goPoint(targetLoc,"going to targetLoc..",2,0.3)
        print("reached the area..")
        targetLoc6 = LocationGlobalRelative(targetLoc.lat, targetLoc.lon, 6)
        time.sleep(2)
        goPoint(targetLoc6,"descending to 6m ",1,0.3)
        time.sleep(2)
        #/////////// water release /////////////#
        th4.start
        th4.join
        time.sleep(2)
        #//////////////////////////////////#
        print("Relay fired and Water released")
        goPoint(targetLoc,"rising to 10m ",1,0.3)
        time.sleep(2)
        vehicle.airspeed = 7
        goPoint(post1_right,"going to post1_right",7,2)
        goPoint(post1_upper,"going to post1_upper",7,2)
        goPoint(post1_left,"going to post1_left",6,2)
        goPoint(finish_line1,"going to finish_line1" ,6,2)
        goPoint(finish_line2,"going to finish_line2",4,2)
        time.sleep(20)

    except:
        print("there is something wrong...' Landing..!!")
        vehicle.mode = VehicleMode("LAND")

th4 = Thread(target=fireRelay)
th3 = Thread(target=waterTest)
th2 = Thread(target=opencv)   
th1 = Thread(target=startAction)

th1.start()
th1.join()