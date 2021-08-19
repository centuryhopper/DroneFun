import cv2
import math
import numpy as np
import pygame
import imutils
from djitellopy import tello
from time import sleep, time
from enforce_typing import enforce_types
from pyimagesearch.objcenter import ObjCenter
from pyimagesearch.pid import PID

# region drone initialization
drone = tello.Tello()
drone.connect()
# sanity checks
# if drone.get_height() > 2:
#     drone.land()
if drone.stream_on:
    drone.streamoff()
print(f'average temperature of drone: {drone.get_temperature()}')
print(f'current drone battery level: {drone.get_battery()}%')
# endregion

# region pygame window init


def init():
    pygame.init()
    win = pygame.display.set_mode((400, 400))


# init()
# endregion

# region basic movements
# drone.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
# drone.send_rc_control(0, 50, 0, 0)
# sleep(2)
# drone.send_rc_control(0, 0, 0, 30)
# sleep(2)
# drone.send_rc_control(0, 0, 0, 0)
# drone.land()
# endregion

# region image capture
# while True:
#     img = drone.get_frame_read().frame
#     img = cv2.resize(img, (360, 240))
#     cv2.imshow('Image', img)
#     cv2.waitKey(1)
# endregion

# region keyboard controls
# acquiring key strokes
# for more details, check out https://www.geeksforgeeks.org/how-to-get-keyboard-input-in-pygame/
# region getkey()


def getKey(keyCode: str) -> bool:
    ans = False
    for event in pygame.event.get():
        pass
    keyPressed = pygame.key.get_pressed()
    myKey = getattr(pygame, f'K_{keyCode}')
    if keyPressed[myKey]:
        ans = True
    pygame.display.update()
    return ans
# endregion
# region getcustomizedkeyboardinputs()


def getCustomizedKeyboardInputs(lrSpeed=0, fbSpeed=0, udSpeed=0, yvSpeed=0) -> list[int]:
    global drone
    lr, fb, ud, yv = 0, 0, 0, 0
    if getKey('LEFT'):
        lr = - lrSpeed
    elif getKey('RIGHT'):
        lr = lrSpeed
    if getKey('UP'):
        fb = fbSpeed
    elif getKey('DOWN'):
        fb = - fbSpeed
    if getKey('w'):
        ud = udSpeed
    elif getKey('s'):
        ud = - udSpeed
    if getKey('a'):
        yv = - yvSpeed
    elif getKey('d'):
        yv = yvSpeed
    # take off if on the ground
    if drone.get_height() < 2:
        if getKey('t'):
            drone.takeoff()
    # quit flying
    if getKey('q'):
        drone.land()
        sleep(3)
    # save snapshot
    if getKey('z'):
        cv2.imwrite(f'Resources/Images/{time()}.jpg', img)
        # makes sure we don't save a bunch of images at once when user tries to take a snapshot
        sleep(0.5)
    return [lr, fb, ud, yv]
# endregion

# endregion

# region surveillance
# control the drone using this coroutine


def droneSurveillance():
    while True:
        lr, fb, ud, yv = getCustomizedKeyboardInputs(50, 50, 50, 50)
        drone.send_rc_control(lr, fb, ud, yv)
        # image capturing
        img = drone.get_frame_read().frame
        # img = cv2.resize(img, (360,240))
        img = cv2.resize(img, (1920, 1080))
        cv2.imshow('Drone Surveillance', img)
        cv2.waitKey(1)


# droneSurveillance()
# endregion

# region face tracking
# range for tracking face (lowerbound and upper bound)
fbRange = [3500, 6800]
# change sensitivity of error by this value
pid = [0.4, 0.4, 0]
pWidthError = 0
# w, h = 360, 240

# region find face()


@enforce_types
def findFace(img, color: tuple[int, int, int] = (0, 0, 255)):
    '''
    method proposed by viola/jones
    (draws a red rectangle around the face) - the center value will be used to rotate - the area value will be used to go forwards and backwards
    '''
    faceCascade = cv2.CascadeClassifier(
        'Resources/haarcascades/haarcascade_frontalface_default.xml')
    # converting to gray scale because the ML algorithm requires gray scale image to perform its classification
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # https://stackoverflow.com/questions/20801015/recommended-values-for-opencv-detectmultiscale-parameters
    faces = faceCascade.detectMultiScale(
        imgGray, scaleFactor=1.2, minNeighbors=8)
    # find the biggest face if there are multiple faces
    faceLstCenters = []
    faceLstArea = []
    for x, y, w, h in faces:
        # top left hand corner pt
        startingPt = (x, y)
        # bottom right hand corner pt
        endingPt = (x+w, y+h)
        # draw a rectangle
        cv2.rectangle(img, startingPt, endingPt, color, thickness=2)
        # get center x and center y
        cx = x + w//2
        cy = y + h//2
        area = w*h
        faceLstCenters.append([cx, cy])
        faceLstArea.append(area)
        # draw a green circle that shows the center of the face
        cv2.circle(img, center=(cx, cy), radius=5,
                   color=(0, 255, 0), thickness=cv2.FILLED)
        cv2.arrowedLine(img, center=(cx, cy), color=(
            0, 0, 255), thickness=cv2.FILLED)

    # get index of max area of a face
    if len(faceLstArea) > 0:
        i = faceLstArea.index(max(faceLstArea))
        return img, [faceLstCenters[i], faceLstArea[i]]
    return img, [[0, 0], 0]
# endregion

# region get drone forward/backward and yaw velocity
def getFBAndYawVelocity(drone, x, y, faceArea, screenWidth, pid, pWidthError):
    # area of face
    area = faceArea
    # print(f'{area = }')
    # screenWidth//2 is center of image (in terms of width of screen)
    errorWidth = x - screenWidth//2

    # equation of pid (proportional, integral, derivative)
    # region yaw calculation
    speedWidth = pid[0]*errorWidth + pid[1]*(errorWidth - pWidthError)
    speedWidth = int(np.clip(speedWidth, -100, 100))
    # endregion

    fb = 0
    # keep drone stationary if the area of the face is within the acceptable range
    if fbRange[0] < area < fbRange[1]:
        fb = 0
    # if face is too close, then move drone away from face
    elif area > fbRange[1]:
        fb = -35
    # else if face is too far, then move drone towards face
    # but also make sure that a face is present
    elif area < fbRange[0] and area != 0:
        fb = 35

    # if we don't get anything, then we have to stop
    if x == 0:
        speedWidth = 0
        errorWidth = 0

    # send command to drone to follow face or move away from face
    # drone.send_rc_control(left_right_velocity=0, forward_backward_velocity=fb,
    #                       up_down_velocity=0, yaw_velocity=speedWidth)
    return errorWidth, fb, speedWidth
# endregion

# region initCamera and track face


def initCameraAndTrackFace(shouldTakeOff=False):
    global pWidthError, drone, w, h
    if shouldTakeOff:
        # turn on video
        drone.streamon()
        drone.takeoff()
        # go up
        drone.move_up(20)

    faceCenter = ObjCenter(
        haarPath='Resources/haarcascades/haarcascade_frontalface_default.xml')
    horizontalPID = PID(kP=0.7, kI=0.0001, kD=0.1)
    verticalPID = PID(kP=0.7, kI=0.0001, kD=0.1)
    horizontalPID.initialize()
    verticalPID.initialize()

    while True:
        # get drone image
        img = drone.get_frame_read().frame
        img = imutils.resize(img, width=400)
        # get image shape and dimensions
        H, W, _ = img.shape
        print(H, W)
        centerX = W//2
        centerY = H//2
        frameCenter = (centerX, centerY)
        # draw a circle in the center of the image frame
        cv2.circle(img, center=frameCenter, radius=5,
                   color=(0, 0, 255), thickness=-1)

        # find face location
        faceLocation = faceCenter.update(img, frameCenter=None)
        ((faceX, faceY), face, d) = faceLocation

        fb,yawVel = 0,0

        if face is not None:
            (x, y, w, h) = face
            cv2.rectangle(img, (x, y), (x+w, y+h),
                          (0, 255, 0), 2)
            # draw a circle in the center of the face
            cv2.circle(img, center=(faceX, faceY), radius=5,
                       color=(255, 0, 0), thickness=-1)
            # Draw line from frameCenter to face center
            cv2.arrowedLine(img, frameCenter, (faceX, faceY),
                            color=(0, 255, 0), thickness=2)

            # region forward/backward and yaw tuning
            pWidthError, fb, yawVel = getFBAndYawVelocity(
                drone, x, y, w*h, W, pid, pWidthError)

            # endregion

        # region left/right and up/down tuning
        # endregion

        # send controls to drone
        drone.send_rc_control(left_right_velocity=0, forward_backward_velocity=fb,
                              up_down_velocity=0, yaw_velocity=yawVel)

        # display image
        cv2.imshow('video window', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('landing')
            drone.land()
            sleep(1)
            break
    cv2.destroyAllWindows()


# endregion

initCameraAndTrackFace(shouldTakeOff=True)
# endregion

# region drone mapping
######## PARAMETERS ###########
fSpeed = 117 / 10  # Forward Speed in cm/s   (15cm/s)
aSpeed = 360 / 10  # Angular Speed Degrees/s  (50d/s)
interval = 0.25
dInterval = fSpeed * interval
aInterval = aSpeed * interval
###############################################
x, y = 500, 500
a = 0
yaw = 0
points = [(0, 0), (0, 0)]
# region get keyboard input


def getKeyboardInput(speed=80, aSpeed=100):
    lr, fb, ud, yv = 0, 0, 0, 0
    global x, y, yaw, a
    d = 0
    if getKey('LEFT'):
        lr = -speed
        d = dInterval
        a = -180
    elif getKey('RIGHT'):
        lr = speed
        d = -dInterval
        a = 180
    if getKey('UP'):
        fb = speed
        d = dInterval
        a = 270
    elif getKey('DOWN'):
        fb = -speed
        d = -dInterval
        a = -90
    if getKey('w'):
        ud = speed
    elif getKey('s'):
        ud = -speed
    if getKey('a'):
        yv = -aSpeed
        yaw -= aInterval
    elif getKey('d'):
        yv = aSpeed
        yaw += aInterval
    if getKey('q'):
        drone.land()
        sleep(2)
    if getKey('t'):
        drone.takeoff()
    sleep(interval)
    a += yaw
    x += int(d * math.cos(math.radians(a)))
    y += int(d * math.sin(math.radians(a)))
    return [lr, fb, ud, yv, x, y]
# endregion
# region draw points


def drawPoints(img, points):
    '''
    draws all points provided by the 'points' list
    onto the img numpy 3D array
    '''
    for point in points:
        # In opencv, the color format is BGR instead of the usual RGB format.
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)
        cv2.circle(img, points[-1], 8, (0, 255, 0), cv2.FILLED)
        cv2.putText(img, f'({(points[-1][0] - 500 )/ 100},{(points[-1][1] - 500) / 100})m',
                    (points[-1][0] + 10, points[-1][1] + 30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)
# endregion
# region runner


def startDroneMapping():
    while True:
        vals = getKeyboardInput()
        drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        img = np.zeros((1000, 1000, 3), np.uint8)
        if (points[-1][0] != vals[4] or points[-1][1] != vals[5]):
            points.append((vals[4], vals[5]))
        drawPoints(img, points)
        cv2.imshow('Output', img)
        cv2.waitKey(1)


# endregion
# endregion

# startDroneMapping()
