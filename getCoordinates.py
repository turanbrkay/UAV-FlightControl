import time
from re import X
from dronekit import connect, LocationGlobalRelative, mavutil, VehicleMode, Command, LocationGlobalRelative, LocationGlobal
import time, math
import numpy as np
import cv2
from threading import Thread
import argparse
import datetime


vehicle = connect("/dev/ttyACM0", wait_ready=True)
vehicle.mode = VehicleMode("GUIDED")


def getCoordinates():
    allLatitude = []
    allLongtitude = []
    lastLatitude = 0
    lastLongtitude = 0
    sumLatitude = 0
    sumLongtitude = 0
    counter = 0
    
    while True:
        allLatitude.append(vehicle.location.global_relative_frame.lat)
        allLongtitude.append(vehicle.location.global_relative_frame.lon)
        print("len(allLatitude)",len(allLatitude))
        print("len(allLongtitude)",len(allLongtitude))
        if counter == 29:
            break
        
        counter += 1
        time.sleep(2)
    print(allLatitude)
    print(allLongtitude)
    for i in allLatitude:
        sumLatitude += i
    for i in allLongtitude:
        sumLongtitude += i

    lastLatitude = (sumLatitude/(len(allLatitude)))
    lastLongtitude = (sumLongtitude/(len(allLongtitude)))
    print("lastLatitude: ",lastLatitude)
    print("lastLongtitude: ",lastLongtitude)

    date = datetime.datetime.now()
    mydate = ("%s/%s/%s  "%(date.day, date.month, date.year)+"%s:%s:%s \t\t"%(date.hour, date.minute, date.second))

    with open("/home/firmino/Documents/important/coordinates.txt","r+") as myFile:
        veri = myFile.read()
        myFile.seek(0)
        myFile.write(mydate+"\t"+str(lastLatitude)+","+str(lastLongtitude)+"\n"+veri)
        myFile.close


getCoordinates()