#! /usr/bin/env python

import argparse
import csv
import cv2
from cv2 import aruco

parser = argparse.ArgumentParser(description='Extract aruco markers from movie.')
parser.add_argument('-f', action='store', dest='path', help='the path to the avi file')
args = parser.parse_args()


aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
parameters = aruco.DetectorParameters_create()
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
parameters.adaptiveThreshWinSizeMin = 7
parameters.adaptiveThreshWinSizeMax = 7 
parameters.adaptiveThreshWinSizeStep = 4
parameters.cornerRefinementMaxIterations = 50
parameters.cornerRefinementMinAccuracy= 0.01
parameters.cornerRefinementWinSize = 1



cap = cv2.VideoCapture(args.path)
with open('markers_seen.csv', 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(['img_id', 'marker_id', 'img_x', 'img_y'])
    img_id = 0
    while(cap.isOpened()):
        ret, frame = cap.read()
        if frame is None:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # if ids is not None:
        #     for i in range(len(ids)):
        #         corner = corners[i][0]
        #         writer.writerow([img_id, ids[i][0], corner[:,0].mean(), corner[:,1].mean()])

        img_id += 1

        frame_marker = aruco.drawDetectedMarkers(gray, corners, ids)
        cv2.imshow('frame', frame_marker)
        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

cap.release()

