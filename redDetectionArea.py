#!/usr/bin/env python
# -*- coding: utf-8 -*-


import time
from re import X

import time, math
import numpy as np
import cv2
from threading import Thread
import argparse


check = False
counter = 0
frameWidth = 640
frameHeight = 480
t = 100
cxArray = list()
cyArray = list()




def display(img):
    pt1 = (275,195)
    pt2 = (365,285)
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

        _,contours, hierarchy = cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            areaMin = 2000
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

                if 45 >= vx >= -45 and 45 >= vy >= -45:
                    counter += 1

                    if counter < 49:
                        leftUpFont = cv2.putText(imageFrame, "saving location -"+str(counter)+ "/50", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 10, 10),2)
                        leftUpFont
                    if counter >= 49:
                        leftUpFont = cv2.putText(imageFrame, "location saved ", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (7, 7, 220),2)
                        leftUpFont
                        time.sleep(1)

                    if counter >= 50:
                        print("BANG'!!!!")
                        time.sleep(3)
                        check = True


        display(imageFrame)
        cv2.imshow("Color Tracking", imageFrame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            webcam.release()
            cv2.destroyAllWindows()
            break

opencv()




