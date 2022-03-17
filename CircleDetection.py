from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil

import time
import math
import numpy as np 
import psutil
import argparse
import copy
import cv2
import numpy as np
import keyboard
from Plane import Plane

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='tcp:127.0.0.1:5762')
    args = parser.parse_args()
    
    
    connection_string = args.connect
    
    #-- Create the object
    plane = Plane(connection_string)

    #-- Arm and takeoff
    if not plane.is_armed(): plane.arm_and_takeoff()


def nothing(x):
    pass

cap = cv2.VideoCapture(0)
cv2.namedWindow("Settings")
cv2.createTrackbar("Lower-Hue", "Settings", 106, 180, nothing)
cv2.createTrackbar("Lower-Saturation", "Settings", 118, 255, nothing)
cv2.createTrackbar("Lower-Value", "Settings", 0, 255, nothing)
cv2.createTrackbar("Upper-Hue", "Settings", 180, 180, nothing)
cv2.createTrackbar("Upper-Saturation", "Settings", 255, 255, nothing)
cv2.createTrackbar("Upper-Value", "Settings", 255, 255, nothing)

font = cv2.FONT_HERSHEY_SIMPLEX

while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame = cv2.flip(frame, 1)

    lh = cv2.getTrackbarPos("Lower-Hue", "Settings")
    ls = cv2.getTrackbarPos("Lower-Saturation", "Settings")
    lv = cv2.getTrackbarPos("Lower-Value", "Settings")

    uh = cv2.getTrackbarPos("Upper-Hue", "Settings")
    us = cv2.getTrackbarPos("Upper-Saturation", "Settings")
    uv = cv2.getTrackbarPos("Upper-Value", "Settings")

    lower_color = np.array([lh, ls, lv])
    upper_color = np.array([uh, us, uv])

    mask = cv2.inRange(hsv, lower_color, upper_color)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)
    mask = cv2.flip(mask, 1)
    
    contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 400:
            cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
            if len(approx) > 6:
                cv2.putText(frame, "HEDEF", (x, y), font, 1, (0, 0, 255))


            if cv2.contourArea(cnt) <= 50:
                continue    
            x,y,w,h = cv2.boundingRect(cnt)
            #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255,0), 2)
            center = (x,y)
            if x > 300:
                plane.go_to_waypoint2()
                print("Gidiyor waypoint 2")
            elif x < 300:
                plane.go_to_waypoint1()
                print("Gidiyor waypoint 1")

        
        
    cv2.resizeWindow("frame", 600, 450)
    #cv2.resizeWindow("mask", 600, 450)
    cv2.imshow("frame", frame)
    #cv2.imshow("mask", mask)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()

