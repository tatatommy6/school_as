import cv2
from cv2 import aruco
import math
import numpy as np

class ArucoDetctor:

    def __init__(self, type = aruco.DICT_ARUCO_ORIGINAL):
        self.arucoDict = aruco.getPredefinedDictionary(type)
        self.parameters = aruco.DetectorParameters()

    def __call__(self, image):
        markerDict = dict()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.arucoDict, parameters=self.parameters)
        
        mtx = np.array([[100000,0.0,960],[0.0, 100000, 540],[0.00000, 0.00000, 1.00000]])
        dist = np.array([[0.0],[0.0],[0.0],[0.0],[0.0]])
        rvec = np.array([[],[],[]])
        tvec = np.array([[],[],[]])
        distance = 0

        if ids is None:
            pass
        else:
            ids = ids.ravel()
            for i in range(0,len(ids)):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, mtx, dist)
                distance = float(tvec[0][0][0] * (- 100 ) * 0.70)
                markerDict[ids[i]] = Marker(ids[i], np.array([corners[i][0]], np.int32), distance)
            
        return corners, ids, markerDict

    def GetAngle(self, coner) -> float:
        centerX = int((coner[0][0] + coner[2][0]) * 0.5)
        centerY = int((coner[0][1] + coner[2][1]) * 0.5)

        rad = math.atan2(centerY - coner[0][1], centerX - coner[0][0])
        dir = (rad * 180) / math.pi
        dir -= 45
        if dir < -180:
            dir += 360

        return dir

class Marker:
    def __init__(self, id, newcorners, distance):
        self.id = id
        self.corners = newcorners
        self.distance = distance
        self.centerX = int((self.corners[0][0][0] + self.corners[0][2][0]) * 0.5)
        self.centerY = int((self.corners[0][0][1] + self.corners[0][2][1]) * 0.5)
    def SetData(self, id, newcorners, distance):
        self.id = id
        self.corners = newcorners
        self.distance = distance
        self.centerX = int((self.corners[0][0] + self.corners[2][0]) * 0.5)
        self.centerY = int((self.corners[0][1] + self.corners[2][1]) * 0.5)