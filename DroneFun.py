from djitellopy import tello
from time import sleep, time
from enforce_typing import enforce_types

import cv2
import pygame
import numpy as np

#region drone initialization

drone = tello.Tello()
drone.connect()

print(f'average temperature of drone: {drone.get_temperature()}')

print(f'current drone battery level: {drone.get_battery()}%')

global img

# drone.takeoff()
#endregion


#region basic movements

# drone.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)

# drone.send_rc_control(0,50,0,0)

# sleep(2)

# drone.send_rc_control(0,0,0,30)

# sleep(2)

# drone.send_rc_control(0,0,0,0)
# drone.land()
#endregion

#region image capture

# drone.stream_on()


# while True:

#     img = drone.get_frame_read().frame

#     img = cv2.resize(img, (360,240))

#     cv2.imshow('Image', img)

#     cv2.waitKey(1)

#endregion


#region keyboard controls

# def init():
#     pygame.init()

#     win = pygame.display.set_mode((400,400))
# init()


# # acquiring key strokes

# # for more details, check out https://www.geeksforgeeks.org/how-to-get-keyboard-input-in-pygame/

# def getKey(keyCode:str) -> bool:

#     ans = False

#     for event in pygame.event.get(): pass

#     keyPressed = pygame.key.get_pressed()

#     myKey = getattr(pygame, f'K_{keyCode}')

#     if keyPressed[myKey]:

#         ans = True
#     pygame.display.update()
#     return ans


# def getCustomizedKeyboardInputs(lrSpeed=0, fbSpeed=0, udSpeed=0, yvSpeed=0) -> list[int]:

#     lr,fb,ud,yv = 0,0,0,0

#     if getKey('LEFT'): lr = -lrSpeed

#     elif getKey('RIGHT'): lr = lrSpeed
    

#     if getKey('UP'): fb = -fbSpeed

#     elif getKey('DOWN'): fb = fbSpeed
    

#     if getKey('w'): ud = -udSpeed

#     elif getKey('s'): ud = udSpeed
    

#     if getKey('a'): yv = -yvSpeed

#     elif getKey('d'): yv = yvSpeed
    

#     # take off if on the ground

#     if drone.get_height() < 2:

#         if getKey('t'): drone.takeoff()
    

#     # quit flying

#     if getKey('q'): drone.land(); sleep(3)
    

#     if getKey('z'):

#         cv2.imwrite(f'Resources/Images/{time()}.jpg', img)

#         sleep(0.5)
    

#     return [lr,fb,ud,yv]

#endregion

#region surveillance

# control the drone

# using this coroutine

# while True:

#     lr,fb,ud,yv = getCustomizedKeyboardInputs(50,50,50,50)

#     drone.send_rc_control(lr,fb,ud,yv)
    
#     # image capturing

#     img = drone.get_frame_read().frame

#     img = cv2.resize(img, (360,240))

#     cv2.imshow('Image', img)

#     cv2.waitKey(1)


#endregion


#region face tracking

def initCamera():

    cap = cv2.VideoCapture(0)

    while True:
        _, img = cap,read()
        img = cv2.resize(img, (w,h))
        img,info = findFace(img ())
        pError = trackFace(drone, info, w, pid, pError)
        print('center:',info[0],'area:',info[1],)

        cv2.imshow('video', img)

        cv2.waitKey(1)

@enforce_types
def findFace(img, color:tuple[int,int,int]=(0,0,255)):
    '''
    method proposed by viola jones
    (draws a red rectangle around the face)
    -the center value will be used to rotate
    -the area value will be used to go forwards and backwards
    '''

    faceCascade - cv2.CascadeClassifier('./Resources/haarcascades/haarcascade_frontalface_default.xml')
    # converting to gray scale

    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(imgGray,1.2,8)
    
    # find the biggest face if there are multiple faces

    faceLstCenters = []

    faceLstArea = []

    for x,y,w,h in faces:
        startingPt = (x,y)
        endingPt = (x+w,y+h)
        cv2.rectangle(img, startingPt, endingPt, color, thickness=2)
        # get center x and center y
        cx = x + w//2
        cy = y + h//2
        area = w*h
        faceLstCenters.append([cx,cy])
        faceLstArea.append(area)
        # draw a green circle that shows the center of the face
        cv2.circle(img, (cx,cy), 5, cv2.FILLED)
    
    # get index of max area of a face
    if len(faceLstArea) > 0:
        i = faceLstArea.index(max(faceLstArea))
        return img, [faceLstCenters[i], faceLstArea[i]]
    
    return img, [[0,0],0]

fbRange = [6200,6800]
# change sensitivity of error by this value
pid = [0.4,0.4,0]
pError = 0
w,h = 360,240

def trackFace(me, info, w, pid, pError):
    area = info[1]
    x,y = info[0]
    # w//2 is center of image
    error = x - w//2
    # equation of pid (proportional, integral, derivative)
    
    speed = pid[0]*error + pid[1]*(error - pError)
    speed = int(np.clip(speed,-100,100))
    
    # keep drone stationary
    if fbRange[0] < area < fbRange[1]:
        fb = 0
    # if face is too close, then move drone away from face
    elif area > fbRange[1]:
        fb = -20
    # else if face is too far, then move drone towards face
    elif area < fbRange[0] and area != 0:
        fb = 20
    # send command to drone
    me.send_rc_control(0,fb,0,speed)
    return pError

#endregion

