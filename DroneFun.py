import cv2
import math
import numpy as np
import imutils
import signal
import sys
from PyGameInit import init
from threading import Thread
from djitellopy import tello
from time import sleep, time
from enforce_typing import enforce_types
from pyimagesearch.objcenter import ObjCenter
from pyimagesearch.pid import PID
from multiprocessing import Manager, Process, Pipe, Event
from KeyboardProcesses import getKeyboardInput

# function to handle keyboard interrupt


def signal_handler(sig, frame):
    print("Signal Handler")
    if drone:
        try:
            drone.streamoff()
            drone.land()
        except:
            pass
    sys.exit()


# range for tracking face (lowerbound and upper bound)
# observations: for ranges with lower numbers like 2200-2800,the drone tends to move away from the face
fbRange = [8200, 8800]
# change sensitivity of error by this value
pid = [0.4, 0.4, 0]
pWidthError = 0
# w, h = 360, 240

# region find face()
@enforce_types
def findFace(img, color = (0, 0, 255)):
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
        fb = -20
    # else if face is too far, then move drone towards face
    # but also make sure that a face is present
    elif area < fbRange[0] and area != 0:
        fb = 20

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
def initCameraAndTrackFace(drone, exit_event, shouldTakeOff=False, shouldShowCamera=True, maxSpeedLimit=45,):
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    global pWidthError, w, h
    if shouldTakeOff:
        drone.takeoff()
        # go up
        drone.move_up(45)
        sleep(2)
    if shouldShowCamera:
        # turn on video
        drone.streamon()

    faceCenter = ObjCenter(
        haarPath='Resources/haarcascades/haarcascade_frontalface_default.xml')
    horizontalPID = PID(kP=0.7, kI=0.0001, kD=0.1)
    verticalPID = PID(kP=0.7, kI=0.0001, kD=0.1)
    horizontalPID.initialize()
    verticalPID.initialize()
    maxThreshold = maxSpeedLimit

    sleep(2)
    while True:
        print('here in face tracker')
        # get drone image
        img = drone.get_frame_read().frame
        img = imutils.resize(img, width=400)
        # get image shape and dimensions
        H, W, _ = img.shape
        centerX = W//2
        centerY = H//2
        frameCenter = (centerX, centerY)
        # draw a circle in the center of the image frame
        cv2.circle(img, center=frameCenter, radius=5,
                   color=(0, 0, 255), thickness=-1)
        # find face location
        faceLocation = faceCenter.update(img, frameCenter=None)
        ((faceX, faceY), face, d) = faceLocation
        fb, yawVel, pan_update, tilt_update = 0, 0, 0, 0

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
            # calculate the pan and tilt errors and run through pid controllers
            pan_error = centerX - faceX
            pan_update = horizontalPID.update(pan_error, sleep=0)

            tilt_error = centerY - faceY
            tilt_update = verticalPID.update(tilt_error, sleep=0)

            cv2.putText(img, f'X Error: {pan_error} PID: {pan_update:.2f}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2, cv2.LINE_AA)

            cv2.putText(img, f'Y Error: {tilt_error} PID: {tilt_update:.2f}', (20, 70), cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255), 2, cv2.LINE_AA)

            pan_update = int(np.clip(pan_update, -maxThreshold, maxThreshold))
            # NOTE: if face is to the right of the drone, the distance will be negative, but
            # the drone has to have positive power so I am flipping the sign
            pan_update *= -1
            tilt_update = int(
                np.clip(tilt_update, -maxThreshold, maxThreshold))

            # endregion

        # send controls to drone
        if pan_update != 0 or tilt_update != 0 or fb != 0 or yawVel != 0:
            drone.send_rc_control(left_right_velocity=pan_update//3, forward_backward_velocity=fb,
                                  up_down_velocity=tilt_update//2, yaw_velocity=yawVel)

        # display image
        cv2.imshow('video window', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('landing')
            drone.land()
            sleep(1)
            break
    signal_handler(None, None)
    cv2.destroyAllWindows()
# endregion

# region drone mapping
points = [(0, 0), (0, 0)]

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


def startDroneMapping(drone, exit_event):
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    init()
    sleep(0.1)
    while True:
        vals = getKeyboardInput(drone,exit_event)
        drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        img = np.zeros((1000, 1000, 3), np.uint8)
        if (points[-1][0] != vals[4] or points[-1][1] != vals[5]):
            points.append((vals[4], vals[5]))
        drawPoints(img, points)
        cv2.imshow('Output', img)
        cv2.waitKey(1)
    signal_handler(None, None)


# endregion
# endregion


if __name__ == '__main__':
    args = sys.argv
    if len(args) != 2:
        print('must choose one function to run')
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    # region drone initialization
    drone = tello.Tello()
    drone.connect()
    # sanity checks
    if drone.get_height() > 2:
        drone.land()
    if drone.stream_on:
        drone.streamoff()
    print(f'average temperature of drone: {drone.get_temperature()}')
    print(f'current drone battery level: {drone.get_battery()}%')
    # endregion

    init()
    exit_event = Event()

    if args[1] == '1':
        initCameraAndTrackFace(drone, shouldTakeOff=True)
    elif args[1] == '2':
        print('starting drone mapping function')
        startDroneMapping(drone, exit_event)
    else:
        print('please enter 1 or 2')
    # print(args)


    # with Manager() as manager:
    #     # p1 = Process(target=initCameraAndTrackFace, args=(drone,exit_event,))
    #     p2 = Process(target=startDroneMapping, args=(drone, exit_event,))
    #     p2.start()
    #     # p1.start()

    #     # sleep(30)
    #     # while not exit_event.is_set():
    #     #     pass
    #     # p1.terminate()
    #     # p2.terminate()
    #     # p1.join()
    #     p2.join()
    print('done!')
