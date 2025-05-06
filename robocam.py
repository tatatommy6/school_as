from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import cv2
from cv2 import aruco
import time
import os
import threading
import enum
import pkg_resources
import copy

import urllib
import numpy as np

import os.path

from RoboCam.face_detector import FaceDetector
from RoboCam.face_landmark import FaceLandmark
from RoboCam.face_recognizer import FaceRecognizer
from RoboCam.aruco_detector import ArucoDetctor
from RoboCam.sketch_recognizer import SketchRecognizer
from RoboCam.number_recognizer import NumberRecognizer

class CameraEvents(enum.Enum):
    RECV_DETECTED_FACE_COUNT = 1
    RECV_DETECTED_FACE_NAME = 2
    RECV_DETECTED_FACE_RECT = 3
    RECV_LEFT_IRIS_POINT = 4
    RECV_LEFT_EYEBROW_POINT = 5
    RECV_RIGHT_IRIS_POINT = 5
    RECV_RIGHT_EYEBROW_POINT = 6
    RECV_NOSE_POINT = 7
    RECV_MOUSE_POINT = 8
    RECV_JAW_POINT = 9
    RECV_ARUCO_ID = 10
    RECV_ARUCO_IDS = 11
    RECV_ARUCO_CENTER_POINTS = 12
    RECV_ARUCO_RECT_POINTS = 13
    RECV_ARUCO_ANGLE = 14
    RECV_SKETCH_NAME = 15
    RECV_NUMBERS = 16

class RoboCam():
    def __init__(self):
        self.__cameraStreamFlag = False
        self.__cameraStreamInitFlag = False
        self.__webcamStreamFlag = False
        self.__webcamStreamInitFlag = False
        self.__eventHandlerDic = {}
        self.__stream = None
        self.__webcam = None
        self.__isConnected = False
        self.__isRunning = True
        self.__mosaicFlag = False
        self.__mosaicRate = 0

        self.__rotateFlag = False
        self.__rotateAngle = 0

        self.__flipLRFlag = False
        self.__flipUDFlag = False
        self.__raw_img = None

        # face detector
        self.__faceDetectFlag = False
        self.__drawFaceAreaFlag = True
        self.__drawFaceNameFlag = True
        self.__drawFacePointFlag = True
        self.__drawFaceSizeFlag = True
        self.__drawLandmarkFlag = True

        self.__faceDetectInitFlag = False
        self.__faceDetectedList = []

        self.__faceLandmarkInitFlag = False
        self.__faceLandmarkList = []

        self.__faceRecognizeInitFlag = False
        self.__faceRecognizedList = []

        self.__RegisterdColor = (0,0,255)
        self.__UnregisterdColor = (255,0,0)
        self.__faceDataDict = dict()

        # aruco detector
        self.__arucoDetectFlag = False
        self.__arucoDetectInitFlag = False
        self.__drawArucoAreaFlag = True
        self.__drawArucoIdFlag = True
        self.__drawArucoPointFlag = True
        self.__drawArucoSizeFlag = True
        self.__drawArucoDistanceFlag = True
        self.__arucoDetectedCorners = []
        self.__arucoDetectedIds = []
        self.__arucoDataDict = dict()

        # sketch detector
        self.__sketchDetectFlag = False
        self.__sketchDetectInitFlag = False
        self.__drawSketchAreaFlag = True
        self.__drawSketchNameFlag = True
        self.__drawSketchPointFlag = True
        self.__drawSketchSizeFlag = True
        self.__sketchRecognizedList = []
        self.__sketchDetectedList = []
        self.__sketchDataDict = dict()

        # number recognizer
        self.__numberDetectInitFlag = False
        self.__numberDetectFlag = False
        self.__drawNumberAreaFlag = True
        self.__drawNumberFlag = True
        self.__drawNumberPointFlag = True
        self.__drawNumberSizeFlag = True

        self.__numberRecognizedStr = ''
        self.__numberDetectedList = []

        print("camera module ready")

    def end(self):
        self.CameraStreamOff()
        self.WebcamStreamOff()
        if self.__faceDetectFlag:
            self.__faceDetectFlag = False
            print("Facedetector Off")

        if self.__arucoDetectFlag:
            self.__arucoDetectFlag = False
            print("Arucodetector Off")

        if self.__sketchDetectFlag:
            self.__sketchDetectFlag = False
            print("Sketchdetector Off")

        if self.__numberDetectFlag:
            self.__numberDetectFlag = False
            print("Numberdetector Off")
        self.__isRunning=False
        exit()

    def WebcamStreamInit(self, width:int = 640, height:int = 480):
        if self.__cameraStreamInitFlag is True:
            print("Camera stream is aready initialized.")
            return
        if self.__webcamStreamInitFlag is True:
            print("Webcam stream is aready initialized.")
            return
        
        while True:
            try:
                self.__webcam = cv2.VideoCapture(2) #(0)
                if self.__webcam.isOpened() == False:
                    time.sleep(0.5)
                    # print("Unable to read camera feed")
                    continue
                else:
                    break
                break
            except:
                time.sleep(0.5)
                # print('no cam yet')
                continue

        self.__camWidth = width
        self.__camHeight = height

        self.__webcamStreamInitFlag = True
        print("webcam stream ready")

        dataSenderTH = threading.Thread(target=self.__dataSender)
        dataSenderTH.daemon = True
        dataSenderTH.start()
        time.sleep(0.1)

        print("camera event module ready")


    def StartWebStream(self, width:int = 512, height:int = 512):
        if self.__cameraStreamInitFlag or self.__webcamStreamInitFlag:
            print("Camera stream is aready running")
            return

        if self.__webcamStreamInitFlag or self.__cameraStreamInitFlag:
            print("Webcam stream is aready running")
            return
        while True:
            try:
                self.__webcam = cv2.VideoCapture(2) #(0)
                if self.__webcam.isOpened() == False:
                    time.sleep(0.5)
                    # print("Unable to read camera feed")
                    continue
                else:
                    break
            except:
                time.sleep(0.5)
                continue
        self.__camWidth = width
        self.__camHeight = height
        self.__webcamStreamInitFlag = True
        print("webcam stream ready")

        dataSenderTH = threading.Thread(target=self.__dataSender)
        dataSenderTH.daemon = True
        dataSenderTH.start()
        time.sleep(0.1)

        self.__webcamStreamFlag = True
        th = threading.Thread(target=self.__webcamStreamTh) 
        th.daemon = True
        th.start()
        print("camera event module ready")

        
    def CameraStreamInit(self, width:int = 512, height:int = 512):
        if self.__cameraStreamInitFlag is True:
            print("Camera stream is aready initialized.")
            return

        if self.__webcamStreamInitFlag is True:
            print("Webcam stream is aready initialized.")
            return

        url = 'http://192.168.4.1:81/stream'
        while True:
            try:
                self.__stream = urllib.request.urlopen(url)
                self.__isConnected = True
                break
            except:
                time.sleep(0.5)
                # print('no cam yet')
                continue

        self.__camWidth = width
        self.__camHeight = height

        self.__cameraStreamInitFlag = True
        print("camera stream ready")

        dataSenderTH = threading.Thread(target=self.__dataSender)
        dataSenderTH.daemon = True
        dataSenderTH.start()
        time.sleep(0.1)

        print("camera event module ready")

    def SetEventHandler(self, event:CameraEvents, func):
        if event in self.__eventHandlerDic:
            print("Event ", event, " already exist lisntener")
        else:
            self.__eventHandlerDic[event] = func

    def RemoveEventHandler(self, event:CameraEvents):
        if event in self.__eventHandlerDic:
            del self.__eventHandlerDic[event]
            print("Event ", event, " is removed")
        else:
            print("Event ", event, " not exist lisntener")
    
    def __dataSender(self):
        while self.__isRunning:
            if CameraEvents.RECV_DETECTED_FACE_COUNT in self.__eventHandlerDic:
                count = self.GetFaceCount()
                self.__eventHandlerDic[CameraEvents.RECV_DETECTED_FACE_COUNT](count)
            if CameraEvents.RECV_DETECTED_FACE_RECT in self.__eventHandlerDic:
                rects = self.GetFaceRects()
                self.__eventHandlerDic[CameraEvents.RECV_DETECTED_FACE_RECT](rects)
            if CameraEvents.RECV_DETECTED_FACE_NAME in self.__eventHandlerDic:
                names = self.GetFaceNames()
                self.__eventHandlerDic[CameraEvents.RECV_DETECTED_FACE_NAME](names)
            if CameraEvents.RECV_LEFT_IRIS_POINT in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_LEFT_IRIS_POINT](self.GetLeftIrisPoint())
            if CameraEvents.RECV_LEFT_EYEBROW_POINT in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_LEFT_EYEBROW_POINT](self.GetLeftEyebrowPoint())
            if CameraEvents.RECV_RIGHT_IRIS_POINT in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_RIGHT_IRIS_POINT](self.GetRightIrisPoint())
            if CameraEvents.RECV_RIGHT_EYEBROW_POINT in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_RIGHT_EYEBROW_POINT](self.GetRightEyebrowPoint())
            if CameraEvents.RECV_NOSE_POINT in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_NOSE_POINT](self.GetNosePoint())
            if CameraEvents.RECV_MOUSE_POINT in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_MOUSE_POINT](self.GetMousePoint())
            if CameraEvents.RECV_JAW_POINT in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_JAW_POINT](self.GetJawPoint())
            if CameraEvents.RECV_ARUCO_ID in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_ARUCO_ID](self.GetArucoId())
            if CameraEvents.RECV_ARUCO_IDS in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_ARUCO_IDS](self.GetArucoIds())
            if CameraEvents.RECV_ARUCO_CENTER_POINTS in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_ARUCO_CENTER_POINTS](self.GetArucoCenterPoint())
            if CameraEvents.RECV_ARUCO_RECT_POINTS in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_ARUCO_RECT_POINTS](self.GetArucoRectPoint())
            if CameraEvents.RECV_ARUCO_ANGLE in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_ARUCO_ANGLE](self.GetArucoAngle())
            if CameraEvents.RECV_SKETCH_NAME in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_SKETCH_NAME](self.GetSketchNames())
            if CameraEvents.RECV_NUMBERS in self.__eventHandlerDic:
                self.__eventHandlerDic[CameraEvents.RECV_NUMBERS](self.GetRecognizedNumbers())
            time.sleep(0.15)
        
    def LeftRightFlipMode(self, flag:bool):
        self.__flipLRFlag = flag
    
    def UpDownFlipMode(self, flag:bool):
        self.__flipUDFlag = flag
        
    def MosaicMode(self, rate:int = 0):
        self.__mosaicRate = rate

        if self.__mosaicRate == 0:
            self.__mosaicFlag = False
        else:
            self.__mosaicFlag = True

    def RotateMode(self, angle:int = 90):
        self.__rotateAngle += int(angle)

        self.__rotateAngle = self.__rotateAngle % 360

        if self.__rotateAngle % 360 == 0:
            self.__rotateFlag = False
        else:
            self.__rotateFlag = True

    def DrawFaceArea(self, flag:bool):
        self.__drawFaceAreaFlag = flag

    def DrawFaceLandmark(self, flag:bool):
        self.__drawLandmarkFlag = flag

    def DrawFaceName(self, flag:bool):
        self.__drawFaceNameFlag = flag
    
    def DrawFacePoint(self, flag:bool):
        self.__drawFacePointFlag = flag
    
    def DrawFaceSize(self, flag:bool):
        self.__drawFaceSizeFlag = flag

    def DrawArucoArea(self, flag:bool):
        self.__drawArucoAreaFlag = flag

    def DrawArucoId(self, flag:bool):
        self.__drawArucoIdFlag = flag

    def DrawArucoPoint(self, flag:bool):
        self.__drawArucoPointFlag = flag

    def DrawArucoDistance(self, flag:bool):
        self.__drawArucoDistanceFlag = flag

    def DrawArucoSize(self, flag:bool):
        self.__drawArucoSizeFlag = flag

    def DrawSketchArea(self, flag:bool):
        self.__drawSketchAreaFlag = flag
    
    def DrawSketchName(self, flag:bool):
        self.__drawSketchNameFlag = flag
    
    def DrawSketchPoint(self, flag:bool):
        self.__drawSketchPointFlag = flag

    def DrawSketchSize(self, flag:bool):
        self.__drawSketchSizeFlag = flag

    def DrawNumberArea(self, flag:bool):
        self.__drawNumberAreaFlag = flag

    def DrawNumber(self, flag:bool):
        self.__drawNumberFlag = flag

    def DrawNumberPoint(self, flag:bool):
        self.__drawNumberPointFlag = flag
    
    def DrawNumberSize(self, flag:bool):
        self.__drawNumberSizeFlag = flag

    def CameraStream(self):
        if self.__cameraStreamFlag == True :
            print("The camera is already working.")
            return
        if self.__webcamStreamFlag == True:
            print("The webcam is already working.")
            return

        self.__cameraStreamFlag = True

        th = threading.Thread(target=self.__cameraStreamTh)
        th.daemon = True
        th.start()

    def WebcamStream(self):
        if self.__cameraStreamFlag == True :
            print("The camera is already working.")
            return
        if self.__webcamStreamFlag == True:
            print("The webcam is already working.")
            return

        self.__webcamStreamFlag = True

        th = threading.Thread(target=self.__webcamStreamTh)
        th.daemon = True
        th.start()

    def __cameraStreamTh(self):
        bytes = b''
        frame = None
        while self.__cameraStreamFlag:

            try:
                bytes += self.__stream.read(64)
                a = bytes.find(b'\xff\xd8')
                b = bytes.find(b'\xff\xd9')
                
                if a != -1 and b != -1:
                    jpg = bytes[a:b+2]
                    bytes = bytes[b+2:]
                    frame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    frame = cv2.resize(frame, (self.__camWidth, self.__camHeight))

                    if self.__flipLRFlag == True:
                        frame = cv2.flip(frame, 1)

                    if self.__flipUDFlag == True:
                        frame = cv2.flip(frame, 0)

                    if self.__mosaicFlag == True:
                        frame = cv2.resize(frame, (self.__camWidth//self.__mosaicRate, self.__camHeight//self.__mosaicRate))
                        frame = cv2.resize(frame, (self.__camWidth, self.__camHeight))

                    if self.__rotateFlag == True:
                        m1 = cv2.getRotationMatrix2D((self.__camWidth/2, self.__camHeight/2), self.__rotateAngle, 1)
                        frame = cv2.warpAffine(frame, m1, (self.__camWidth, self.__camHeight))

                    self.__raw_img = frame.copy()

                    if self.__faceDetectFlag == True:
                        self.__overlay_face_boxes(frame)
                    
                    if self.__arucoDetectFlag == True:
                        if self.__drawArucoAreaFlag == True:
                            self.__overlay_aruco_boxes(frame)
                    
                    if self.__sketchDetectFlag == True:
                        if self.__drawSketchAreaFlag == True:
                            self.__overlay_sketch_boxes(frame)
    
                    if self.__numberDetectFlag == True:
                        if self.__drawNumberAreaFlag == True:
                            self.__overlay_number_boxes(frame)

                    cv2.imshow('i', frame)
                    cv2.waitKey(1)
            except Exception as e:
                # print("STREAM : " , e)
                continue
        print ("Stream stopped")

    def __webcamStreamTh(self):
        frame = None
        while self.__webcamStreamFlag:

            try:
                ret, frame = self.__webcam.read()
                if ret == False:
                    continue
                frame = cv2.resize(frame, (self.__camWidth, self.__camHeight))

                if self.__flipLRFlag == True:
                    frame = cv2.flip(frame, 1)

                if self.__flipUDFlag == True:
                    frame = cv2.flip(frame, 0)

                if self.__mosaicFlag == True:
                    frame = cv2.resize(frame, (self.__camWidth//self.__mosaicRate, self.__camHeight//self.__mosaicRate))
                    frame = cv2.resize(frame, (self.__camWidth, self.__camHeight))

                if self.__rotateFlag == True:
                    m1 = cv2.getRotationMatrix2D((self.__camWidth/2, self.__camHeight/2), self.__rotateAngle, 1)
                    frame = cv2.warpAffine(frame, m1, (self.__camWidth, self.__camHeight))

                self.__raw_img = frame.copy()

                if self.__faceDetectFlag == True:
                    self.__overlay_face_boxes(frame)
                    
                if self.__arucoDetectFlag == True:
                    if self.__drawArucoAreaFlag == True:
                        self.__overlay_aruco_boxes(frame)
                    
                if self.__sketchDetectFlag == True:
                    if self.__drawSketchAreaFlag == True:
                        self.__overlay_sketch_boxes(frame)
    
                if self.__numberDetectFlag == True:
                    if self.__drawNumberAreaFlag == True:
                        self.__overlay_number_boxes(frame)

                cv2.imshow('i', frame)
                cv2.waitKey(1)
            except Exception as e:
                # print("STREAM : " , e)
                continue
        print ("Stream stopped")

    def __overlay_face_boxes(self, frame):
        color = self.__UnregisterdColor
        if self.__faceDetectedList is not None:
            for faceKey, faceData in self.__faceDataDict.items():
                addedY = 0
                if self.__drawFaceAreaFlag:
                    cv2.rectangle(frame, (int(faceData.box[0]), int(faceData.box[1])), (int(faceData.box[2]), int(faceData.box[3])), color, 3)
                if self.__drawFacePointFlag == True:
                    s = 'x=' + str(faceData.centerX) +' y='+str(faceData.centerY)
                    cv2.putText(frame, s, (int(faceData.box[2]),int(faceData.box[1]+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                    addedY += 20
                if self.__drawFaceSizeFlag == True:
                    s = 'size=' + str(faceData.size)
                    cv2.putText(frame, s, (int(faceData.box[2]),int(faceData.box[1]+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                    addedY += 20
                if self.__drawFaceNameFlag == True:
                    s = 'name=' + str(faceData.name)
                    cv2.putText(frame, s, (int(faceData.box[2]),int(faceData.box[1]+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                    addedY += 20
                if self.__drawLandmarkFlag == True:
                    for faces in faceData.landMarks:
                        cv2.circle(frame, (int(faces[0]),int(faces[1])), 3, (255,0,255), -1)
                # pointIdx = 0
                    # if pointIdx != 0 and pointIdx != 17 and pointIdx != 22 and pointIdx != 27 and pointIdx != 36 and pointIdx != 42 and pointIdx != 48 and pointIdx != 60:
                    #     cv2.line(frame, (self.__faceLandmarkList[faceIdx][pointIdx][0], self.__faceLandmarkList[faceIdx][pointIdx][1]), (self.__faceLandmarkList[faceIdx][pointIdx-1][0], self.__faceLandmarkList[faceIdx][pointIdx-1][1]), (255,255,0), 1)
                    # pointIdx += 1
                # cv2.line(frame, (self.__faceLandmarkList[faceIdx][41][0], self.__faceLandmarkList[faceIdx][41][1]), (self.__faceLandmarkList[faceIdx][36][0], self.__faceLandmarkList[faceIdx][36][1]), (255,255,0), 1)
                # cv2.line(frame, (self.__faceLandmarkList[faceIdx][47][0], self.__faceLandmarkList[faceIdx][47][1]), (self.__faceLandmarkList[faceIdx][42][0], self.__faceLandmarkList[faceIdx][42][1]), (255,255,0), 1)
                # cv2.line(frame, (self.__faceLandmarkList[faceIdx][59][0], self.__faceLandmarkList[faceIdx][59][1]), (self.__faceLandmarkList[faceIdx][48][0], self.__faceLandmarkList[faceIdx][48][1]), (255,255,0), 1)
                # cv2.line(frame, (self.__faceLandmarkList[faceIdx][67][0], self.__faceLandmarkList[faceIdx][67][1]), (self.__faceLandmarkList[faceIdx][60][0], self.__faceLandmarkList[faceIdx][60][1]), (255,255,0), 1)
    
    def __overlay_aruco_boxes(self,frame):
        duplicateId = []
        color = self.__RegisterdColor

        # for arucoKey, arucoData in self.__arucoDataDict.items():
        #     addedY = 0
            # if self.__drawArucoAreaFlag == True:
            #     cv2.polylines(frame, np.array([arucoData.corners[0]], np.int32), True, color, 3)
            # if self.__drawArucoIdFlag == True:
            #     s = 'id='+str(arucoKey)
            #     cv2.putText(frame, s, (arucoData.corners[0][3][0],arucoData.corners[0][3][1]+addedY), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
            #     addedY += 20
            # if self.__drawArucoPointFlag == True:
            #     s = 'x=' + str(arucoData.centerX) +' y='+str(arucoData.centerY)
            #     cv2.putText(frame, s, (arucoData.corners[0][3][0],arucoData.corners[0][3][1]+addedY), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
            #     addedY += 20
            # if self.__drawArucoSizeFlag == True:
            #     aruco_perimeter = cv2.arcLength(arucoData.corners[0], True) / 7
            #     s = 'size='+ str(int(aruco_perimeter))
            #     cv2.putText(frame, s, (arucoData.corners[0][3][0],arucoData.corners[0][3][1]+addedY), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
            #     addedY += 20
            # if self.__drawArucoDistanceFlag == True:
            #     s = 'distance={:.2f}'.format(arucoData.distance)
            #     cv2.putText(frame, s, (arucoData.corners[0][3][0],arucoData.corners[0][3][1]+addedY), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
            #     addedY += 20

        if self.__arucoDetectedIds is None:
            pass
        else:
            idx = 0
            for corners in self.__arucoDetectedCorners:
                addedY = 0
                id = self.__arucoDetectedIds[idx]
                if id in duplicateId:
                    color = self.__UnregisterdColor
                else:
                    color =self.__RegisterdColor
                    duplicateId.append(id)

                x = int((corners[0][0][0] + corners[0][2][0]) / 2)
                y = int((corners[0][0][1] + corners[0][2][1]) / 2)
                if self.__drawArucoAreaFlag == True:
                    cv2.polylines(frame, np.array([corners[0]], np.int32), True, color, 3)
                if self.__drawArucoIdFlag == True:
                    s = 'id='+str(id)
                    cv2.putText(frame, s, (int(corners[0][3][0]),int(corners[0][3][1])+addedY), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                    addedY += 20
                if self.__drawArucoPointFlag == True:
                    s = 'x=' + str(x) +' y='+str(y)
                    cv2.putText(frame, s, (int(corners[0][3][0]),int(corners[0][3][1])+addedY), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                    addedY += 20
                if self.__drawArucoSizeFlag == True:
                    aruco_perimeter = cv2.arcLength(corners[0], True) / 7
                    s = 'size='+ str(int(aruco_perimeter))
                    cv2.putText(frame, s, (int(corners[0][3][0]),int(corners[0][3][1])+addedY), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                    addedY += 20
                if self.__drawArucoDistanceFlag == True:
                    s = 'distance={:.2f}'.format(self.__arucoDataDict[id].distance)
                    cv2.putText(frame, s, (int(corners[0][3][0]),int(corners[0][3][1])+addedY), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                    addedY += 20

                idx+=1

    def __overlay_sketch_boxes(self, frame):
        color = self.__UnregisterdColor
        
        for sketchKey, sketchData in self.__sketchDataDict.items():
            addedY = 0
            if self.__drawSketchAreaFlag:
                cv2.polylines(frame, np.array([sketchData.box], np.int32), True, color, 3)
            if self.__drawSketchNameFlag:
                s = 'id='+str(sketchData.name)
                cv2.putText(frame, s, (int(sketchData.textX), int(sketchData.textY+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                # cv2.putText(frame, s, (int(sketchData.box[1][0]), int(sketchData.box[1][1]+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                addedY += 20
            if self.__drawSketchPointFlag == True:
                s = 'x=' + str(sketchData.centerX) +' y='+str(sketchData.centerY)
                cv2.putText(frame, s, (int(sketchData.textX), int(sketchData.textY+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                # cv2.putText(frame, s, (int(sketchData.box[1][0]), int(sketchData.box[1][1]+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                addedY += 20
            if self.__drawSketchSizeFlag == True:
                s = 'size=' + str(sketchData.size)
                cv2.putText(frame, s, (int(sketchData.textX), int(sketchData.textY+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                # cv2.putText(frame, s, (int(sketchData.box[1][0]), int(sketchData.box[1][1]+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                addedY += 20


    def __overlay_number_boxes(self, frame):
        color = self.__UnregisterdColor

        for detected in self.__numberDetectedList:
            addedY = 0
            x = int((detected[0][0] + detected[2][0]) / 2)
            y = int((detected[0][1] + detected[2][1]) / 2)
            if self.__drawNumberAreaFlag == True:
                cv2.polylines(frame, np.array([detected], np.int32), True, color, 3)
            if self.__drawNumberFlag == True:
                s = 'number='+str(self.__numberRecognizedStr)
                cv2.putText(frame, s, (int(detected[3][0]), int(detected[3][1]+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                addedY += 20
            if self.__drawNumberPointFlag == True:
                s = 'x=' + str(x) +' y='+str(y)
                cv2.putText(frame, s, (int(detected[3][0]), int(detected[3][1]+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                addedY += 20
            if self.__drawNumberSizeFlag == True:
                s = 'size=' + str( abs ( int ( detected[2][0] - detected[0][0])))
                cv2.putText(frame, s, (int(detected[3][0]), int(detected[3][1]+addedY)), cv2.FONT_HERSHEY_COMPLEX,0.8, (0,255,0), 1)
                addedY += 20
        

    def CameraStreamOff(self):
        if( self.__cameraStreamFlag == False ):
            print("The camera is already stopped.")
            return

        if self.__faceDetectFlag:
            self.__faceDetectFlag = False
            print("Facedetector Off")

        if self.__arucoDetectFlag:
            self.__arucoDetectFlag = False
            print("Arucodetector Off")

        if self.__sketchDetectFlag:
            self.__sketchDetectFlag = False
            print("Sketchdetector Off")

        if self.__numberDetectFlag:
            self.__numberDetectFlag = False
            print("Numberdetector Off")

        self.__cameraStreamFlag = False
        self.__stream.close()
        time.sleep(1)

        print("Camera off")

    def WebcamStreamOff(self):
        if( self.__webcamStreamFlag == False ):
            print("The webcam is already stopped.")
            return

        if self.__faceDetectFlag:
            self.__faceDetectFlag = False
            print("Facedetector Off")

        if self.__arucoDetectFlag:
            self.__arucoDetectFlag = False
            print("Arucodetector Off")

        if self.__sketchDetectFlag:
            self.__sketchDetectFlag = False
            print("Sketchdetector Off")

        if self.__numberDetectFlag:
            self.__numberDetectFlag = False
            print("Numberdetector Off")

        self.__webcamStreamFlag = False
        #웹캠 객체 헤제 밎 none으로 초기화
        if self.__webcam:
            self.__webcam.release()
            self.__webcam = None    #<-- 웹캠 객체 초기화

        self.__webcamStreamInitFlag = False   #<----- 웹캠 스트림 초기화 플래그 초기화

        cv2.destroyAllWindows()   #<----- cv2로 원도우 모두 종료

        time.sleep(1)

        print("Webcam off")
    
    def FacedetectorInit(self):
        if self.__faceDetectInitFlag is False:
            self.__faceD = FaceDetector()
            self.__faceDetectInitFlag = True
        
        if self.__faceLandmarkInitFlag is False:
            self.__landD = FaceLandmark()
            self.__faceLandmarkInitFlag = True
        
        if self.__faceRecognizeInitFlag is False:
            self.__faceR = FaceRecognizer()
            self.__faceRecognizeInitFlag = True

        print("Facedetector initialized")
    
    def FacedetectorStart(self):

        if self.__faceDetectInitFlag is False:
            print("Facedetector is not initialized")
            return

        if self.__faceDetectFlag == True:
            print("Facedetector is already working.")
            return
        self.__faceDetectFlag = True

        th = threading.Thread(target=self.__facedetect)
        th.daemon = True
        th.start()

    def __facedetect(self):
        while self.__faceDetectFlag:
            if self.__raw_img is None:
                time.sleep(0.1)
                # print('no input frame yet')
                continue
            try:
                self.__faceDetectedList = self.__faceD(self.__raw_img)
                self.__faceLandmarkList = self.__landD.batch_call (self.__raw_img, copy.deepcopy(self.__faceDetectedList))
                self.__faceRecognizedList = self.__faceR(self.__raw_img, copy.deepcopy(self.__faceDetectedList))

                self.__faceDataDict.clear()
                for i in range(0,len(self.__faceDetectedList)):
                    faceData = FaceData()
                    faceData.SetData(self.__faceRecognizedList[i], list(self.__faceDetectedList[i]),self.__faceLandmarkList[i])
                    self.__faceDataDict[self.__faceRecognizedList[i]] = faceData

            except Exception as e:
                # print("Detect : " , e)
                continue
            
            time.sleep(0.001)

    def FacedetectorStop(self):
        if self.__faceDetectFlag == False :
            print("Facedetector is already stopped.")
            return

        self.__faceDetectFlag = False
        time.sleep(1)

        print("Facedetector off")
        
    def FaceCapture(self, name:str, captureCount:int=5, path:str=pkg_resources.resource_filename(__package__,"res/face/")):
        if bool(name) == False:
            print("Name parameter is Empty.")
            return
        
        if os.path.isdir(path) is False:
            os.mkdir(path)
        
        if self.__faceDetectFlag is False:
            print("Facedetector did not run")
            return
        
        cnt = 0
        while cnt < captureCount:
            if len(self.__faceDataDict) == 0:
                print("Doesn't have a any face in Frame")
                continue
            
            bbox = (0, copy.deepcopy(self.__faceDetectedList.copy())[0])
            
            result = self.__faceR.SaveFace(self.__raw_img,bbox,name,path)
            if result == 0:
                cnt += 1
                time.sleep(0.1)
        print( name, " is saved")

    def TrainFaceData(self, facePath:str =pkg_resources.resource_filename(__package__,"res/face/")):
        # print(facePath)
        if os.path.isdir(facePath) is False:
            print(facePath +" is not directory.")
            return
        
        if(facePath.strip()[-1] is not '/'):
            facePath += '/'
            
        faceD = FaceDetector()
        self.__faceR.registerd.clear()

        filenames = os.listdir(facePath)
        for filename in filenames:
            name = os.path.basename(filename)
            image = cv2.imread(facePath + filename, cv2.IMREAD_ANYCOLOR)
            facedetectedList = faceD(image)

            if np.any(facedetectedList) == False:
                print("Doesn't have a any face in Frame")
                continue
            
            name = name.split('_')[0]
            bbox = (0, facedetectedList[0])
            self.__faceR.TrainModel(image, bbox, name)

    def DeleteFaceData(self, name:str, facePath:str=pkg_resources.resource_filename(__package__,"res/face/")):
        if os.path.isdir(facePath) is False:
            print(facePath +" is not directory.")
            return

        if(facePath.strip()[-1] is not '/'):
            facePath += '/'
            
        self.__faceR.RemoveFace(name, facePath)

        print(name + ' is deleted')

    def GetFaceCount(self) -> int:        
        return len(self.__faceDataDict)
    
    def GetFaceExist(self, name:str="Human0") -> bool:
        return name in self.__faceDataDict

    def GetFaceRects(self) -> list:
        if len(self.__faceDetectedList) == 0:
            return []
        
        return list(self.__faceDetectedList[0])
            
    def GetFaceNames(self) -> list:
        if len(self.__faceRecognizedList) == 0:
            return []
        
        return list(self.__faceRecognizedList)

    def GetFaceSize(self, name:str="Human0") -> int:
        if name in self.__faceDataDict:
            return self.__faceDataDict[name].size
        pass

    def GetFaceCenterPoint(self, name:str="Human0") -> list:
        if name in self.__faceDataDict:
            return [self.__faceDataDict[name].centerX,self.__faceDataDict[name].centerY]
        pass
    
    def GetLeftIrisPoint(self, name:str="Human0") -> list:
        x = 0
        y = 0
        if name in self.__faceDataDict:
            x = (self.__faceDataDict[name].landMarks[36][0] + self.__faceDataDict[name].landMarks[39][0])/2
            y = (self.__faceDataDict[name].landMarks[36][1] + self.__faceDataDict[name].landMarks[39][1])/2 

        return [x,y]

    def GetLeftEyebrowPoint(self, name:str="Human0") -> list:
        x = 0
        y = 0
        if name in self.__faceDataDict:
            x = self.__faceDataDict[name].landMarks[19][0]
            y = self.__faceDataDict[name].landMarks[19][1]
        return [x,y]

    def GetRightIrisPoint(self, name:str="Human0") -> list:
        x = 0
        y = 0
        if name in self.__faceDataDict:
            x = (self.__faceDataDict[name].landMarks[42][0] + self.__faceDataDict[name].landMarks[45][0])/2
            y = (self.__faceDataDict[name].landMarks[42][1] + self.__faceDataDict[name].landMarks[45][1])/2 
        return [x,y]

    def GetRightEyebrowPoint(self, name:str="Human0") -> list:
        x = 0
        y = 0
        if name in self.__faceDataDict:
            x = self.__faceDataDict[name].landMarks[24][0]
            y = self.__faceDataDict[name].landMarks[24][1]
        return [x,y]

    def GetNosePoint(self, name:str="Human0") -> list:
        x = 0
        y = 0
        if name in self.__faceDataDict:
            x = self.__faceDataDict[name].landMarks[33][0]
            y = self.__faceDataDict[name].landMarks[33][1]
        return [x,y]
        
    def GetMousePoint(self, name:str="Human0") -> list:
        x = 0
        y = 0
        if name in self.__faceDataDict:
            x = (self.__faceDataDict[name].landMarks[48][0] + self.__faceDataDict[name].landMarks[54][0])/2
            y = (self.__faceDataDict[name].landMarks[48][1] + self.__faceDataDict[name].landMarks[54][1])/2 
        return [x,y]

    def GetJawPoint(self, name:str="Human0") -> list:
        x = 0
        y = 0
        if name in self.__faceDataDict:
            x = self.__faceDataDict[name].landMarks[8][0]
            y = self.__faceDataDict[name].landMarks[8][1]
        return [x,y]

    def GetArucoId(self) -> int:
        if self.__arucoDetectedIds is None or len(self.__arucoDetectedIds) is 0:
            return
        else:
            return self.__arucoDetectedIds[0]

    def GetArucoIds(self) -> list:
        if self.__arucoDetectedIds is None or len(self.__arucoDetectedIds) is 0:
            return
        else:
            return self.__arucoDetectedIds
    
    def GetArucoCount(self) -> int:
        if self.__arucoDetectedIds is None or len(self.__arucoDetectedIds) is 0:
            return
        else:
            return len(self.__arucoDetectedIds)

    def GetArucoCenterPoint(self,id:int) -> list:
        if self.__arucoDetectedIds is not None and id in self.__arucoDataDict:
            return [self.__arucoDataDict[id].centerX,self.__arucoDataDict[id].centerY]
        pass
    
    def GetArucoExist(self,id:int)->bool:
        if self.__arucoDetectedIds is not None:
            return id in self.__arucoDetectedIds

    def GetArucoX(self,id:int)->int:
        if self.__arucoDetectedIds is not None and id in self.__arucoDetectedIds:
            return self.__arucoDataDict[id].centerX

    def GetArucoY(self,id:int)->int:
        if self.__arucoDetectedIds is not None and id in self.__arucoDetectedIds:
            return self.__arucoDataDict[id].centerY

    def GetArucoDistance(self,id:int)->float:
        if self.__arucoDetectedIds is not None and id in self.__arucoDataDict:
            return self.__arucoDataDict[id].distance

    def GetArucoRectPoint(self,id:int) -> list:
        ret = []
        if self.__arucoDetectedIds is not None and id in self.__arucoDataDict:
            ret = self.__arucoDataDict[id].corners[0]
        return ret

    def GetArucoAngle(self)->float:
        if self.__arucoDetectedIds is None or len(self.__arucoDetectedIds) is 0:
            pass
        else:
            return self.__arucoD.GetAngle(self.__arucoDetectedCorners[0][0])

    def ArucoDetectorInit(self):
        if self.__arucoDetectInitFlag is False:
            self.__arucoD = ArucoDetctor()
            self.__arucoDetectInitFlag = True
            self.__drawArucoAreaFlag = True

        print("Aruco detector initialized")
    
    def ArucoDetectorStart(self):
        if self.__arucoDetectInitFlag is False:
            print("Aruco detector is not initialized")
            return

        if self.__arucoDetectFlag == True:
            print("Aruco detector is already working.")
            return
        self.__arucoDetectFlag = True

        th = threading.Thread(target=self.__arucodetect)
        th.daemon = True
        th.start()

    def ArucodetectorStop(self):
        if self.__arucoDetectFlag == False :
            print("Aruco detector is already stopped.")
            return

        self.__arucoDetectFlag = False
        time.sleep(1)

        print("Aruco detector off")

    def __arucodetect(self):
        while self.__arucoDetectFlag:
            if self.__raw_img is None:
                time.sleep(0.1)
                # print('no input frame yet')
                continue
            try:
                coners, ids, markerDict = self.__arucoD(self.__raw_img)
                
                if ids is not None:
                    self.__arucoDetectedCorners = list(coners)
                    self.__arucoDetectedIds = ids
                    self.__arucoDataDict = copy.deepcopy(markerDict)
                else:
                    self.__arucoDetectedCorners = []
                    self.__arucoDetectedIds = []
                    self.__arucoDataDict = dict()

            except Exception as e:
                # print("Aruco detector error : " , e)
                continue

            time.sleep(0.001)

    def GetSketchExist(self,name:str="Sketch") ->bool:
        return name in self.__sketchDataDict

    def GetSketchCenterPoint(self, name:str) -> list:
        if name in self.__sketchDataDict:
            return [self.__sketchDataDict[name].centerX,self.__sketchDataDict[name].centerY]
        pass

    def GetSketchSize(self, name:str="Sketch") -> int:
        if name in self.__sketchDataDict:
            return self.__sketchDataDict[name].size
        pass

    def GetSketchNames(self) -> list:
        if len(self.__sketchRecognizedList) == 0:
            pass
        else:
            return list(self.__sketchRecognizedList)
    
    def SketchDetectorInit(self):
        if self.__sketchDetectInitFlag is False:
            self.__sketchR = SketchRecognizer()
            self.__sketchDetectInitFlag = True

        print("Sketch detector initialized")
    
    def SketchDetectorStart(self):
        if self.__sketchDetectInitFlag is False:
            print("Sketch detector is not initialized")
            return

        if self.__sketchDetectFlag == True:
            print("Sketch detector is already working.")
            return
        self.__sketchDetectFlag = True

        th = threading.Thread(target=self.__sketchdetect)
        th.daemon = True
        th.start()

    def __sketchdetect(self):
        while self.__sketchDetectFlag:
            if self.__raw_img is None:
                time.sleep(0.1)
                # print('no input frame yet')
                continue
            try:
                self.__sketchRecognizedList, self.__sketchDetectedList = self.__sketchR(self.__raw_img)
                self.__sketchDataDict.clear()
                for i in range(0, len(self.__sketchDetectedList)):
                    self.__sketchDataDict[self.__sketchRecognizedList[i]] = SketchData( self.__sketchRecognizedList[i], self.__sketchDetectedList[i])

                if len(self.__sketchRecognizedList) == 0:
                    time.sleep(0.0)
                    continue
            except Exception as e:
                # print("Sketch detector error : " , e)
                continue
            
            time.sleep(0.01)

    def SketchDetectorStop(self):
        if self.__sketchDetectFlag == False :
            print("Sketch detector is already stopped.")
            return

        self.__sketchDetectFlag = False
        time.sleep(1)

        print("Sketch detector off")
        
    def SketchCapture(self, name:str, captureCount:int=5, path:str=pkg_resources.resource_filename(__package__,"res/sketch/")):
        if bool(name) == False:
            print("Name parameter is Empty.")
            return

        if os.path.isdir(path) is False:
            os.mkdir(path)

        if self.__sketchDetectFlag is False:
            print("Sketchdetector did not run")
            return
        
        cnt = 0
        while cnt < captureCount:
            if len(self.__sketchRecognizedList) == 0:
                print("Doesn't have a any sketch in Frame")
                time.sleep(0.0)
                continue
            
            result = self.__sketchR.SaveSketch(self.__raw_img,name,path)
            if result == 0:
                cnt += 1
                time.sleep(0.1)
        print( name, " is saved")

    def TrainSketchData(self, sketchPath:str = pkg_resources.resource_filename(__package__,"res/sketch/")):
        if os.path.isdir(sketchPath) is False:
            print(sketchPath +" is not directory.")
            return
        
        if(sketchPath.strip()[-1] is not '/'):
            sketchPath += '/'
            
        orbDescriptors = []
        nameIndexList = []
        nameIntList = []

        sketchD = SketchRecognizer()
        filenames = os.listdir(sketchPath)
        for filename in filenames:
            name = os.path.basename(filename)
            image = cv2.imread(sketchPath+filename, cv2.IMREAD_GRAYSCALE)
            image = cv2.resize(image, (150,150))
            _, des = sketchD.orbDetector.detectAndCompute(image, None)
            name = name.split('_')[0]
            
            if not(name in nameIndexList):
                nameIndexList.append(name)
            
            nameIntList.append(nameIndexList.index(name))
            orbDescriptors.append(des)
            
        self.__sketchR.TrainModel(nameIndexList, nameIntList, orbDescriptors)

    def DeleteSketchData(self, name:str, sketchPath:str=pkg_resources.resource_filename(__package__,"res/sketch/")):

        if os.path.isdir(sketchPath) is False:
            print(sketchPath +" is not directory.")
            return
        
        if(sketchPath.strip()[-1] is not '/'):
            sketchPath += '/'

        self.__sketchR.RemoveSketch(name, sketchPath)

        print(name + ' is deleted')

    def NumberRecognizerInit(self):
        if self.__numberDetectInitFlag is False:
            self.__numberR = NumberRecognizer()
            self.__numberDetectInitFlag = True
        
        print("Number recognizer initialized")

    def GetRecognizedNumbers(self)->str:
        if self.__numberRecognizedStr:
            return self.__numberRecognizedStr

    def GetRecognizedNumberPoint(self)->list:
        if self.__numberDetectedList is not None and len(self.__numberDetectedList) > 0:
            x = int((self.__numberDetectedList[0][0][0] + self.__numberDetectedList[0][2][0]) / 2)
            y = int((self.__numberDetectedList[0][0][1] + self.__numberDetectedList[0][2][1]) / 2)
            return [x, y]
        pass

    def GetRecognizedNumberSize(self)->int:
        if self.__numberDetectedList is not None and len(self.__numberDetectedList) > 0:
            return abs(int(self.__numberDetectedList[0][2][0] - self.__numberDetectedList[0][0][0]))
            
    def NumberRecognizerStart(self):
        if self.__numberDetectInitFlag is False:
            print("Number recognizer is not initialized")
            return

        if self.__numberDetectFlag == True:
            print("Number recognizer is already working.")
            return
        self.__numberDetectFlag = True

        th = threading.Thread(target=self.__numberdetect)
        th.daemon = True
        th.start()

    def __numberdetect(self):
        while self.__numberDetectFlag:
            if self.__raw_img is None:
                time.sleep(0.1)
                # print('no input frame yet')
                continue
            try:
                self.__numberRecognizedStr,self.__numberDetectedList = self.__numberR(self.__raw_img)
            except Exception as e:
                # print("Number recognizer error : " , e)
                continue
            
            time.sleep(0.05)
    
    def NumberRecognizerStop(self):
        if self.__numberDetectFlag == False :
            print("Number recognizer is already stopped.")
            return

        self.__numberDetectFlag = False
        time.sleep(1)

        print("Number recognizer off")

    

class FaceData:
    def __init__(self):
        self.name = ''
        self.box = []
        self.landMarks = None
        self.centerX = 0
        self.centerY = 0
        self.size = 0

    def SetData(self, name:str, box, landmarks):
        self.name = name
        self.box = box
        self.landMarks = landmarks
        self.centerX = int((box[0] + box[2]) / 2)
        self.centerY = int((box[1] + box[3]) / 2)
        self.size = self.landMarks[16][0] - self.landMarks[0][0]

class SketchData:
    def __init__(self, name:str, box:list):
        self.name = name
        self.box = box
        self.centerX = int((self.box[0][0] + self.box[2][0]) / 2)
        self.centerY = int((self.box[0][1] + self.box[2][1]) / 2)
        self.size = abs(int(self.box[2][0] - self.box[0][0]))
        self.textX = 0
        self.textY = 20000
        for i in range(4):
            if self.textX < self.box[i][0]:
                self.textX = self.box[i][0]
            if self.textY > self.box[i][1]:
                self.textY = self.box[i][1]