from RoboCam.robocam import *
import time
rcam=RoboCam()

rcam.WebcamStreamInit()
rcam.WebcamStream()
rcam.FacedetectorInit()
rcam.FacedetectorStart()