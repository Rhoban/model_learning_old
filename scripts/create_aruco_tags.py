#! /usr/bin/env python
import cv2
import os
from cv2 import aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
if not os.path.exists("tmp"):
    os.mkdir("tmp")

for i in range(10):
    img = aruco.drawMarker(aruco_dict, i, 700)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,str(i),(10,45), font, 1,(100,100,100),2,cv2.LINE_AA)
    cv2.imwrite("tmp/aruco_%03d.png" % (i,), img)
